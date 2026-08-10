use macroquad::prelude::*;

use crate::{
    collision::{intersection, obstacle::Obstacle, potential_field::PotentialFieldConfig},
    config,
    gui::{
        input as gui_input,
        panel::{self, PanelState},
        renderer::Renderer,
    },
    kinematics::{
        damped_ls::JacobianDLS,
        inverse::{IkConfig, IkSolver},
        pseudoinverse::{JacobianPseudoInverse, JacobianPseudoInverseUndamped},
        transpose::JacobianTranspose,
    },
    math::vector2::Vector2,
    robot::{arm::RobotArm, limits::JointLimit},
    simulation::Simulation,
};

const JOINT_KEY_LABELS: [(&str, &str); 5] = [
    ("Q", "A"),
    ("W", "S"),
    ("E", "D"),
    ("Z", "X"),
    ("C", "V"),
];

const JOINT_KEY_CODES: [(KeyCode, KeyCode); 5] = [
    (KeyCode::Q, KeyCode::A),
    (KeyCode::W, KeyCode::S),
    (KeyCode::E, KeyCode::D),
    (KeyCode::Z, KeyCode::X),
    (KeyCode::C, KeyCode::V),
];

const OBSTACLE_PICK_RADIUS: f32 = 45.0;

pub struct App {
    robot: RobotArm,
    renderer: Renderer,
    simulation: Simulation,
    solver: Box<dyn IkSolver>,
    ik_config: IkConfig,
    panel_state: PanelState,
    obstacles: Vec<Obstacle>,
    field_config: PotentialFieldConfig,
}

impl App {
    pub fn new() -> Self {
        let robot = RobotArm::new(
            config::DEFAULT_BASE_POSITION,
            config::DEFAULT_SEGMENT_LENGTHS.to_vec(),
        );

        let mut simulation = Simulation::new();
        simulation.set_target(robot.end_effector());

        let panel_state = PanelState::new(robot.segment_count(), &config::DEFAULT_SEGMENT_LENGTHS);

        Self {
            robot,
            renderer: Renderer::new(),
            simulation,
            solver: Box::new(JacobianPseudoInverse),
            ik_config: IkConfig::default(),
            panel_state,
            obstacles: Vec::new(),
            field_config: PotentialFieldConfig::default(),
        }
    }

    pub async fn run(&mut self) {
        loop {
            self.handle_input();

            self.simulation.update(
                &mut self.robot,
                self.solver.as_ref(),
                &self.ik_config,
                self.panel_state.animation_speed,
                &self.obstacles,
                &self.field_config,
            );

            clear_background(config::BACKGROUND_COLOR);

            self.renderer.draw(
                &self.robot,
                &self.obstacles,
                self.panel_state.show_forces,
                &self.field_config,
            );
            self.renderer.draw_obstacles(&self.obstacles, self.robot.state.colliding);

            if self.panel_state.show_target {
                let reachable = self.robot.is_within_reach(self.simulation.target());
                self.renderer.draw_target(self.simulation.target(), reachable);
            }

            if self.panel_state.show_jacobian {
                self.renderer.draw_jacobian_overlay(&self.robot.jacobian());
            }

            self.renderer.draw_error_graph(
                &self.robot,
                screen_width() - 290.0,
                screen_height() - 180.0,
                260.0,
                120.0,
            );

            self.draw_overlay();
            self.draw_joint_tooltip();

            let events = panel::draw(&mut self.panel_state);
            self.apply_panel_events(events);

            next_frame().await;
        }
    }

    fn apply_panel_events(&mut self, events: panel::PanelEvents) {
        if events.joint_count_changed {
            let lengths = self.panel_state.active_lengths();
            let base = self.robot.base_position;

            self.robot = RobotArm::new(base, lengths);
            self.reapply_joint_limits();
            self.simulation.set_target(self.robot.end_effector());
        }

        if let Some(index) = events.segment_length_changed {
            let length = self.panel_state.segment_lengths[index];
            self.robot.set_segment_length(index, length);
            self.robot.state.clear_solver_status();
        }

        if let Some(index) = events.joint_limit_changed {
            self.apply_joint_limit(index);
            self.robot.state.clear_solver_status();
        }

        if events.solver_changed {
            self.solver = match self.panel_state.solver_index {
                0 => Box::new(JacobianTranspose) as Box<dyn IkSolver>,
                1 => Box::new(JacobianPseudoInverse) as Box<dyn IkSolver>,
                2 => Box::new(JacobianPseudoInverseUndamped) as Box<dyn IkSolver>,
                _ => Box::new(JacobianDLS) as Box<dyn IkSolver>,
            };

            self.robot.state.clear_solver_status();
        }

        if events.reset_requested {
            self.simulation.reset(&mut self.robot);
        }

        if events.random_requested {
            self.randomize_pose();
        }
    }

    fn randomize_pose(&mut self) {
        for index in 0..self.robot.actuated_joint_count() {
            let limit = self.robot.joint_limit(index).unwrap_or_default();
            let min = limit.min_angle;
            let max = limit.max_angle;
            let angle = min + ::rand::random::<f32>() * (max - min);
            self.robot.set_joint_angle(index, angle);
        }

        self.robot.state.clear_solver_status();
    }

    fn reapply_joint_limits(&mut self) {
        for index in 0..self.robot.actuated_joint_count() {
            self.apply_joint_limit(index);
        }
    }

    fn apply_joint_limit(&mut self, index: usize) {
        let limit = if self.panel_state.limit_enabled[index] {
            let min_rad = self.panel_state.min_angles_deg[index].to_radians();
            let max_rad = self.panel_state.max_angles_deg[index].to_radians();
            JointLimit::new(min_rad, max_rad)
        } else {
            JointLimit::unrestricted()
        };

        self.robot.set_joint_limit(index, limit);
    }

    fn handle_input(&mut self) {
        if is_key_pressed(KeyCode::R) {
            self.simulation.reset(&mut self.robot);
        }

        if is_key_pressed(KeyCode::Space) {
            self.simulation.toggle_pause();
        }

        if is_key_pressed(KeyCode::J) {
            self.print_jacobian();
        }

        let (_, wheel_delta) = mouse_wheel();
        if wheel_delta != 0.0 && gui_input::is_pointer_over_panel() {
            let max_scroll = (self.panel_state.content_height - screen_height()).max(0.0);
            self.panel_state.scroll_y = (self.panel_state.scroll_y - wheel_delta * 30.0)
                .clamp(0.0, max_scroll);
        }

        if is_mouse_button_down(MouseButton::Left) && !gui_input::is_pointer_over_panel() {
            let (mouse_x, mouse_y) = mouse_position();
            let point = Vector2::new(mouse_x, mouse_y);

            if !is_key_down(KeyCode::O) && !is_key_down(KeyCode::P) {
                self.robot.state.clear_solver_status();
                self.simulation.set_target(point);
            }
        }

        if is_mouse_button_pressed(MouseButton::Left) && !gui_input::is_pointer_over_panel() {
            let (mouse_x, mouse_y) = mouse_position();
            let point = Vector2::new(mouse_x, mouse_y);

            if is_key_down(KeyCode::O) {
                self.try_add_circle_obstacle(point);
            } else if is_key_down(KeyCode::P) {
                self.try_add_aabb_obstacle(point);
            }
        }

        if is_mouse_button_pressed(MouseButton::Right) && !gui_input::is_pointer_over_panel() {
            let (mouse_x, mouse_y) = mouse_position();
            self.remove_nearest_obstacle(Vector2::new(mouse_x, mouse_y));
        }

        self.handle_joint_input();
    }

    fn try_add_circle_obstacle(&mut self, position: Vector2) {
        let candidate = Obstacle::circle(position, config::DEFAULT_OBSTACLE_RADIUS);

        if intersection::arm_intersects_any(
            &self.robot.pose.joint_positions,
            std::slice::from_ref(&candidate),
        ) {
            return;
        }

        self.obstacles.push(candidate);
        self.robot.state.clear_solver_status();
    }

    fn try_add_aabb_obstacle(&mut self, position: Vector2) {
        let candidate = Obstacle::aabb(position, config::DEFAULT_OBSTACLE_HALF_EXTENTS);

        if intersection::arm_intersects_any(
            &self.robot.pose.joint_positions,
            std::slice::from_ref(&candidate),
        ) {
            return;
        }

        self.obstacles.push(candidate);
        self.robot.state.clear_solver_status();
    }

    fn remove_nearest_obstacle(&mut self, point: Vector2) {
        let nearest = self
            .obstacles
            .iter()
            .enumerate()
            .map(|(index, obstacle)| (index, Vector2::distance(obstacle.position, point)))
            .filter(|(_, distance)| *distance <= OBSTACLE_PICK_RADIUS)
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        if let Some((index, _)) = nearest {
            self.obstacles.remove(index);
            self.robot.state.clear_solver_status();
        }
    }

    fn handle_joint_input(&mut self) {
        let rotation_step = 0.05;
        let joint_count = self.robot.actuated_joint_count().min(JOINT_KEY_CODES.len());

        for (index, (positive, negative)) in JOINT_KEY_CODES.iter().enumerate().take(joint_count) {
            if is_key_down(*positive) {
                self.rotate_joint(index, rotation_step);
            }

            if is_key_down(*negative) {
                self.rotate_joint(index, -rotation_step);
            }
        }
    }

    fn rotate_joint(&mut self, index: usize, delta: f32) {
        let Some(angle) = self.robot.joint_angle(index) else {
            return;
        };

        self.robot.state.clear_solver_status();
        self.robot.set_joint_angle(index, angle + delta);
    }

    fn print_jacobian(&self) {
        let jacobian = self.robot.jacobian();

        println!();
        println!("Jacobian");
        println!("{}", jacobian.matrix);
        println!();
    }

    fn draw_joint_tooltip(&self) {
        let (mouse_x, mouse_y) = mouse_position();
        let mouse_point = Vector2::new(mouse_x, mouse_y);

        for (index, position) in self.robot.pose.joint_positions.iter().enumerate() {
            if index == 0 {
                continue;
            }

            let distance = Vector2::distance(mouse_point, *position);
            if distance < 20.0 {
                let angle = self.robot.state.joint_angles.get(index - 1).copied().unwrap_or(0.0);
                let angle_deg = angle.to_degrees();

                let text = format!("Joint {}: {:.1}°", index, angle_deg);
                let text_x = mouse_x + 16.0;
                let text_y = mouse_y - 10.0;

                let text_width = text.len() as f32 * 9.0;
                draw_rectangle(text_x - 6.0, text_y - 16.0, text_width + 12.0, 24.0, Color::new(0.0, 0.0, 0.0, 0.75));
                draw_text(&text, text_x, text_y + 2.0, 18.0, WHITE);
                break;
            }
        }
    }

    fn draw_overlay(&self) {
        let base_x = panel::PANEL_WIDTH + 20.0;
        let indent_x = base_x + 15.0;
        let mut y = 32.0;

        draw_text(config::WINDOW_TITLE, base_x, y, 32.0, WHITE);

        y += 36.0;

        draw_text(&format!("FPS : {}", get_fps()), base_x, y, 24.0, GREEN);

        y += 32.0;

        draw_text(
            &format!(
                "Simulation : {}",
                if self.simulation.config.paused {
                    "PAUSED"
                } else {
                    "RUNNING"
                }
            ),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 32.0;

        draw_text(
            &format!("Joint Count : {}", self.robot.joint_count()),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 28.0;

        draw_text(
            &format!("Segment Count : {}", self.robot.segment_count()),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 28.0;

        draw_text(
            &format!("Total Length : {:.1}", self.robot.total_length()),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 28.0;

        draw_text(
            &format!("Minimum Reach : {:.1}", self.robot.minimum_reach()),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 28.0;

        draw_text(
            &format!("Maximum Reach : {:.1}", self.robot.maximum_reach()),
            base_x,
            y,
            22.0,
            WHITE,
        );

        y += 38.0;

        draw_text("IK Solver", base_x, y, 24.0, YELLOW);

        y += 30.0;

        let target = self.simulation.target();

        draw_text(
            &format!("Target : ({:.1}, {:.1})", target.x, target.y),
            indent_x,
            y,
            21.0,
            LIGHTGRAY,
        );

        y += 25.0;

        draw_text(
            &format!("Iteration : {}", self.robot.state.iteration),
            indent_x,
            y,
            21.0,
            LIGHTGRAY,
        );

        y += 25.0;

        draw_text(
            &format!("Error : {:.3}", self.robot.state.current_error),
            indent_x,
            y,
            21.0,
            LIGHTGRAY,
        );

        y += 25.0;

        let status = if self.robot.state.singular {
            ("SINGULAR", ORANGE)
        } else if self.robot.state.constraint_blocked {
            ("STALLED (LIMIT)", ORANGE)
        } else if self.robot.state.stalled {
            ("STALLED", ORANGE)
        } else if self.robot.state.unreachable {
            ("UNREACHABLE", RED)
        } else if self.robot.state.converged {
            ("CONVERGED", GREEN)
        } else {
            ("SOLVING", YELLOW)
        };

        draw_text(
            &format!("Status : {}", status.0),
            indent_x,
            y,
            21.0,
            status.1,
        );

        y += 25.0;

        let manipulability_color = if self.robot.state.near_singular {
            ORANGE
        } else {
            LIGHTGRAY
        };

        draw_text(
            &format!(
                "Manipulability : {:.5}{}",
                self.robot.state.manipulability,
                if self.robot.state.near_singular {
                    " (NEAR SINGULAR)"
                } else {
                    ""
                }
            ),
            indent_x,
            y,
            21.0,
            manipulability_color,
        );

        y += 25.0;

        let collision_color = if self.robot.state.colliding { RED } else { LIGHTGRAY };

        draw_text(
            &format!(
                "Collision : {}",
                if self.robot.state.colliding { "YES" } else { "NO" }
            ),
            indent_x,
            y,
            21.0,
            collision_color,
        );

        y += 38.0;

        draw_text("Segment Lengths", base_x, y, 24.0, YELLOW);

        y += 30.0;

        for (index, segment) in self.robot.segments.iter().enumerate() {
            draw_text(
                &format!("Segment {} : {:.1}", index + 1, segment.length),
                indent_x,
                y,
                21.0,
                LIGHTGRAY,
            );

            y += 25.0;
        }

        y += 12.0;

        draw_text("Joint Angles", base_x, y, 24.0, YELLOW);

        y += 30.0;

        for (index, angle) in self.robot.state.joint_angles.iter().enumerate() {
            let limit = self.robot.joint_limit(index).unwrap_or_default();
            let restricted = !limit.is_unrestricted();
            let at_bound = restricted
                && ((angle - limit.min_angle).abs() <= 0.01
                    || (limit.max_angle - angle).abs() <= 0.01);

            let color = if at_bound {
                ORANGE
            } else if restricted {
                YELLOW
            } else {
                LIGHTGRAY
            };

            let suffix = if restricted { " [LIMITED]" } else { "" };

            draw_text(
                &format!(
                    "Joint {} : {:.2} rad ({:.1}°){}",
                    index + 1,
                    angle,
                    angle.to_degrees(),
                    suffix
                ),
                indent_x,
                y,
                21.0,
                color,
            );

            y += 25.0;
        }

        y += 12.0;

        let end_effector = self.robot.end_effector();

        draw_text(
            &format!(
                "End Effector : ({:.1}, {:.1})",
                end_effector.x, end_effector.y
            ),
            base_x,
            y,
            21.0,
            WHITE,
        );

        y += 28.0;

        draw_text(
            &format!("Obstacles : {}", self.obstacles.len()),
            base_x,
            y,
            21.0,
            WHITE,
        );

        y += 30.0;

        draw_text("Controls", base_x, y, 24.0, YELLOW);

        y += 28.0;

        draw_text("Click / Drag : Set Target", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        draw_text("O + Click : Add Circle Obstacle", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        draw_text("P + Click : Add AABB Obstacle", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        draw_text("Right Click : Remove Obstacle", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        let joint_count = self.robot.actuated_joint_count().min(JOINT_KEY_LABELS.len());

        for index in 0..joint_count {
            let (positive, negative) = JOINT_KEY_LABELS[index];

            draw_text(
                &format!("{} / {} : Joint {}", positive, negative, index + 1),
                indent_x,
                y,
                20.0,
                LIGHTGRAY,
            );

            y += 24.0;
        }

        y += 4.0;

        draw_text("R : Reset", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        draw_text("SPACE : Pause / Resume", indent_x, y, 20.0, LIGHTGRAY);

        y += 24.0;

        draw_text("J : Print Jacobian", indent_x, y, 20.0, LIGHTGRAY);

        y += 30.0;

        draw_text("Legend", base_x, y, 24.0, YELLOW);
        y += 28.0;

        draw_text("● Converged", indent_x, y, 18.0, GREEN);
        y += 22.0;
        draw_text("● Solving", indent_x, y, 18.0, YELLOW);
        y += 22.0;
        draw_text("● Unreachable", indent_x, y, 18.0, RED);
        y += 22.0;
        draw_text("● Singular / Stalled", indent_x, y, 18.0, ORANGE);
    }
}