use macroquad::prelude::*;

use crate::{
    collision::{
        obstacle::{Obstacle, ObstacleShape},
        potential_field::{self, PotentialFieldConfig},
    },
    config,
    kinematics::jacobian::Jacobian,
    math::vector2::Vector2,
    robot::arm::RobotArm,
};

const JOINT_LIMIT_COLOR: Color = Color::new(1.00, 0.85, 0.30, 0.85);
const JOINT_LIMIT_RADIUS: f32 = 26.0;
const JOINT_LIMIT_SEGMENTS: usize = 24;

const OBSTACLE_COLOR: Color = Color::new(0.55, 0.55, 0.60, 0.85);
const OBSTACLE_COLLIDING_COLOR: Color = Color::new(1.00, 0.35, 0.35, 0.85);

pub struct Renderer;

impl Renderer {
    pub fn new() -> Self {
        Self
    }

    pub fn draw(
        &self,
        robot: &RobotArm,
        obstacles: &[Obstacle],
        show_forces: bool,
        field_config: &PotentialFieldConfig,
    ) {
        self.draw_grid();
        self.draw_robot(robot);
        self.draw_joint_limits(robot);
        self.draw_base(robot.base_position);

        if show_forces {
            self.draw_force_arrows(robot, obstacles, field_config);
        }
    }

    pub fn draw_target(&self, target: Vector2, reachable: bool) {
        let color = if reachable { GREEN } else { RED };

        draw_circle_lines(target.x, target.y, 12.0, 2.5, color);

        draw_line(
            target.x - 16.0,
            target.y,
            target.x + 16.0,
            target.y,
            2.0,
            color,
        );

        draw_line(
            target.x,
            target.y - 16.0,
            target.x,
            target.y + 16.0,
            2.0,
            color,
        );
    }

    pub fn draw_jacobian_overlay(&self, jacobian: &Jacobian) {
        let width = 280.0;
        let x = screen_width() - width - 20.0;
        let mut y = 40.0;

        draw_rectangle(x - 10.0, y - 28.0, width + 20.0, 92.0, Color::new(0.0, 0.0, 0.0, 0.55));
        draw_text("Jacobian (2 x N)", x, y, 20.0, YELLOW);
        y += 26.0;

        for row in 0..jacobian.rows() {
            let label = if row == 0 { "dx" } else { "dy" };

            let values: Vec<String> = (0..jacobian.cols())
                .map(|col| format!("{:.1}", jacobian.get(row, col).unwrap_or(0.0)))
                .collect();

            draw_text(&format!("{} : [{}]", label, values.join(", ")), x, y, 18.0, WHITE);
            y += 24.0;
        }
    }

    pub fn draw_obstacles(&self, obstacles: &[Obstacle], colliding: bool) {
        let color = if colliding {
            OBSTACLE_COLLIDING_COLOR
        } else {
            OBSTACLE_COLOR
        };

        for obstacle in obstacles {
            match obstacle.shape {
                ObstacleShape::Circle { radius } => {
                    draw_circle(obstacle.position.x, obstacle.position.y, radius, color);
                    draw_circle_lines(obstacle.position.x, obstacle.position.y, radius, 2.0, BLACK);
                }
                ObstacleShape::Aabb { half_extents } => {
                    let x = obstacle.position.x - half_extents.x;
                    let y = obstacle.position.y - half_extents.y;
                    let width = half_extents.x * 2.0;
                    let height = half_extents.y * 2.0;

                    draw_rectangle(x, y, width, height, color);
                    draw_rectangle_lines(x, y, width, height, 2.0, BLACK);
                }
            }
        }
    }

    pub fn draw_error_graph(&self, robot: &RobotArm, x: f32, y: f32, width: f32, height: f32) {
        let history = &robot.state.error_history;

        if history.len() < 2 {
            return;
        }

        let max_error = history
            .iter()
            .fold(1.0_f32, |acc, &val| acc.max(val.max(0.001_f32)));

        draw_rectangle(x - 10.0, y - 28.0, width + 20.0, height + 40.0, Color::new(0.0, 0.0, 0.0, 0.55));
        draw_text("Error vs Iteration", x, y, 20.0, YELLOW);

        let graph_y = y + 10.0;
        let graph_height = height - 20.0;

        for i in 0..history.len() - 1 {
            let t1 = i as f32 / 100.0;
            let t2 = (i + 1) as f32 / 100.0;

            let error1 = history[i];
            let error2 = history[i + 1];

            let px1 = x + t1 * width;
            let py1 = graph_y + graph_height - (error1 / max_error) * graph_height;
            let px2 = x + t2 * width;
            let py2 = graph_y + graph_height - (error2 / max_error) * graph_height;

            let color = if error2 < error1 { GREEN } else { RED };

            draw_line(px1, py1, px2, py2, 2.0, color);
        }

        if let Some(&last) = history.last() {
            let t = (history.len() - 1) as f32 / 100.0;
            let px = x + t * width;
            let py = graph_y + graph_height - (last / max_error) * graph_height;

            draw_circle(px, py, 3.0, WHITE);
        }

        draw_text(&format!("max: {:.2}", max_error), x + width - 80.0, y + 22.0, 14.0, LIGHTGRAY);
        draw_text("iter →", x + width - 60.0, graph_y + graph_height + 18.0, 14.0, LIGHTGRAY);
    }

    fn draw_robot(&self, robot: &RobotArm) {
        self.draw_segments(robot);
        self.draw_joints(robot);
    }

    fn draw_segments(&self, robot: &RobotArm) {
        for (index, _segment) in robot.segments.iter().enumerate() {
            let Some(start) = robot.pose.joint_position(index) else {
                continue;
            };

            let Some(end) = robot.pose.joint_position(index + 1) else {
                continue;
            };

            let colliding = robot
                .state
                .segment_collision
                .get(index)
                .copied()
                .unwrap_or(false);

            let color = if colliding {
                Color::new(1.0, 0.2, 0.2, 1.0)
            } else {
                config::SEGMENT_COLOR
            };

            draw_line(
                start.x,
                start.y,
                end.x,
                end.y,
                config::DEFAULT_SEGMENT_THICKNESS,
                color,
            );

            if colliding {
                draw_circle_lines((start.x + end.x) / 2.0, (start.y + end.y) / 2.0, 16.0, 3.0, RED);
            }

            self.draw_segment_direction(start, end);
        }
    }

    fn draw_segment_direction(&self, start: Vector2, end: Vector2) {
        let direction = end - start;

        if direction.length() <= f32::EPSILON {
            return;
        }

        let midpoint = (start + end) * 0.5;
        let direction = direction.normalized();

        let marker_start = midpoint - direction * 5.0;
        let marker_end = midpoint + direction * 5.0;

        draw_line(
            marker_start.x,
            marker_start.y,
            marker_end.x,
            marker_end.y,
            2.0,
            WHITE,
        );
    }

    fn draw_joints(&self, robot: &RobotArm) {
        for (index, position) in robot.pose.joint_positions.iter().enumerate() {
            let radius = if index == 0 {
                config::DEFAULT_BASE_RADIUS
            } else {
                config::DEFAULT_JOINT_RADIUS
            };

            draw_circle(position.x, position.y, radius, config::JOINT_COLOR);
            draw_circle_lines(position.x, position.y, radius, 2.0, BLACK);
        }
    }

    fn draw_joint_limits(&self, robot: &RobotArm) {
        let mut baseline = 0.0_f32;

        for index in 0..robot.segment_count() {
            let Some(position) = robot.pose.joint_position(index) else {
                break;
            };

            let Some(limit) = robot.joint_limit(index) else {
                break;
            };

            if !limit.is_unrestricted() {
                self.draw_joint_limit_arc(position, baseline, limit.min_angle, limit.max_angle);
            }

            baseline += robot.state.joint_angles.get(index).copied().unwrap_or(0.0);
        }
    }

    fn draw_joint_limit_arc(
        &self,
        position: Vector2,
        baseline_angle: f32,
        min_angle: f32,
        max_angle: f32,
    ) {
        let start_angle = baseline_angle + min_angle;
        let end_angle = baseline_angle + max_angle;

        let mut previous = Vector2::new(
            position.x + JOINT_LIMIT_RADIUS * start_angle.cos(),
            position.y + JOINT_LIMIT_RADIUS * start_angle.sin(),
        );

        for step in 1..=JOINT_LIMIT_SEGMENTS {
            let t = step as f32 / JOINT_LIMIT_SEGMENTS as f32;
            let angle = start_angle + (end_angle - start_angle) * t;

            let current = Vector2::new(
                position.x + JOINT_LIMIT_RADIUS * angle.cos(),
                position.y + JOINT_LIMIT_RADIUS * angle.sin(),
            );

            draw_line(previous.x, previous.y, current.x, current.y, 2.0, JOINT_LIMIT_COLOR);
            previous = current;
        }

        draw_line(
            position.x,
            position.y,
            position.x + JOINT_LIMIT_RADIUS * start_angle.cos(),
            position.y + JOINT_LIMIT_RADIUS * start_angle.sin(),
            1.5,
            JOINT_LIMIT_COLOR,
        );

        draw_line(
            position.x,
            position.y,
            position.x + JOINT_LIMIT_RADIUS * end_angle.cos(),
            position.y + JOINT_LIMIT_RADIUS * end_angle.sin(),
            1.5,
            JOINT_LIMIT_COLOR,
        );
    }

    fn draw_base(&self, base: Vector2) {
        draw_circle(base.x, base.y, config::DEFAULT_BASE_RADIUS, config::BASE_COLOR);
        draw_circle_lines(base.x, base.y, config::DEFAULT_BASE_RADIUS, 2.0, BLACK);
        draw_text("Base", base.x - 22.0, base.y + 34.0, 24.0, WHITE);
    }

    fn draw_grid(&self) {
        let width = screen_width();
        let height = screen_height();

        let mut x = 0.0;

        while x <= width {
            draw_line(x, 0.0, x, height, 1.0, config::GRID_COLOR);
            x += config::GRID_SIZE;
        }

        let mut y = 0.0;

        while y <= height {
            draw_line(0.0, y, width, y, 1.0, config::GRID_COLOR);
            y += config::GRID_SIZE;
        }
    }

    fn draw_force_arrows(
        &self,
        robot: &RobotArm,
        obstacles: &[Obstacle],
        field_config: &PotentialFieldConfig,
    ) {
        let vectors = potential_field::compute_force_vectors(robot, obstacles, field_config);

        for (position, force) in vectors {
            let magnitude = force.length();
            let max_arrow_length = 30.0;
            let length = (magnitude * 0.04).min(max_arrow_length);

            if length < 2.0 {
                continue;
            }

            let direction = force.normalized();
            let end = position + direction * length;

            draw_line(position.x, position.y, end.x, end.y, 2.5, ORANGE);

            let arrow_head_size = 8.0;
            let angle = direction.angle();

            let left = end + Vector2::from_angle(angle + 2.5) * arrow_head_size;
            let right = end + Vector2::from_angle(angle - 2.5) * arrow_head_size;

            draw_line(end.x, end.y, left.x, left.y, 2.0, ORANGE);
            draw_line(end.x, end.y, right.x, right.y, 2.0, ORANGE);

            draw_circle(position.x, position.y, 2.0, YELLOW);
        }
    }
}