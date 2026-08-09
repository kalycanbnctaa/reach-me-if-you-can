use macroquad::prelude::*;

use crate::gui::{animation, colors, widgets};

pub const MIN_JOINT_COUNT: usize = 2;
pub const MAX_JOINT_COUNT: usize = 5;
pub const MIN_SEGMENT_LENGTH: f32 = 20.0;
pub const MAX_SEGMENT_LENGTH: f32 = 200.0;
pub const MIN_JOINT_ANGLE_DEG: f32 = -180.0;
pub const MAX_JOINT_ANGLE_DEG: f32 = 180.0;
pub const PANEL_WIDTH: f32 = 300.0;

pub struct PanelState {
    pub joint_count: usize,
    pub segment_lengths: [f32; MAX_JOINT_COUNT],
    pub limit_enabled: [bool; MAX_JOINT_COUNT],
    pub min_angles_deg: [f32; MAX_JOINT_COUNT],
    pub max_angles_deg: [f32; MAX_JOINT_COUNT],
    pub show_jacobian: bool,
    pub show_target: bool,
    pub solver_index: usize,
    pub dropdown_open: bool,
    pub animation_speed: usize,
    slider_dragging: [bool; MAX_JOINT_COUNT],
    min_angle_dragging: [bool; MAX_JOINT_COUNT],
    max_angle_dragging: [bool; MAX_JOINT_COUNT],
}

impl PanelState {
    pub fn new(joint_count: usize, initial_lengths: &[f32]) -> Self {
        let mut segment_lengths = [80.0; MAX_JOINT_COUNT];

        for (index, &length) in initial_lengths.iter().enumerate().take(MAX_JOINT_COUNT) {
            segment_lengths[index] = length;
        }

        Self {
            joint_count: joint_count.clamp(MIN_JOINT_COUNT, MAX_JOINT_COUNT),
            segment_lengths,
            limit_enabled: [false; MAX_JOINT_COUNT],
            min_angles_deg: [MIN_JOINT_ANGLE_DEG; MAX_JOINT_COUNT],
            max_angles_deg: [MAX_JOINT_ANGLE_DEG; MAX_JOINT_COUNT],
            show_jacobian: false,
            show_target: true,
            solver_index: 1,
            dropdown_open: false,
            animation_speed: 1,
            slider_dragging: [false; MAX_JOINT_COUNT],
            min_angle_dragging: [false; MAX_JOINT_COUNT],
            max_angle_dragging: [false; MAX_JOINT_COUNT],
        }
    }

    pub fn active_lengths(&self) -> Vec<f32> {
        self.segment_lengths[0..self.joint_count].to_vec()
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PanelEvents {
    pub joint_count_changed: bool,
    pub segment_length_changed: Option<usize>,
    pub joint_limit_changed: Option<usize>,
    pub solver_changed: bool,
    pub reset_requested: bool,
}

pub fn draw(state: &mut PanelState) -> PanelEvents {
    let mut events = PanelEvents::default();
    let screen_h = screen_height();

    draw_rectangle(0.0, 0.0, PANEL_WIDTH, screen_h, colors::PANEL_BACKGROUND);
    draw_line(PANEL_WIDTH, 0.0, PANEL_WIDTH, screen_h, 2.0, colors::PANEL_BORDER);

    let inner_x = 20.0;
    let inner_width = PANEL_WIDTH - 40.0;
    let mut y = 24.0;

    draw_text("Robot Configuration", inner_x, y, 22.0, colors::PANEL_TITLE);
    y += 34.0;

    let new_joint_count = widgets::stepper(
        inner_x,
        y,
        "N (Joints)",
        state.joint_count,
        MIN_JOINT_COUNT,
        MAX_JOINT_COUNT,
    );

    if new_joint_count != state.joint_count {
        state.joint_count = new_joint_count;
        events.joint_count_changed = true;
    }

    y += 46.0;

    draw_text("Segment Lengths", inner_x, y, 20.0, colors::PANEL_TITLE);
    y += 28.0;

    for index in 0..state.joint_count {
        let label = format!("Segment {}", index + 1);
        let mut dragging = state.slider_dragging[index];

        let result = widgets::slider(
            inner_x,
            y,
            inner_width,
            &label,
            state.segment_lengths[index],
            MIN_SEGMENT_LENGTH,
            MAX_SEGMENT_LENGTH,
            &mut dragging,
        );

        state.slider_dragging[index] = dragging;

        if result.changed {
            state.segment_lengths[index] = result.value;
            events.segment_length_changed = Some(index);
        }

        y += 46.0;
    }

    y += 10.0;

    draw_text("Joint Limits", inner_x, y, 20.0, colors::PANEL_TITLE);
    y += 28.0;

    for index in 0..state.joint_count {
        let checkbox_label = format!("Enable Limit {}", index + 1);
        let new_enabled = widgets::checkbox(inner_x, y, &checkbox_label, state.limit_enabled[index]);

        if new_enabled != state.limit_enabled[index] {
            state.limit_enabled[index] = new_enabled;
            events.joint_limit_changed = Some(index);
        }

        y += 26.0;

        if state.limit_enabled[index] {
            let mut min_dragging = state.min_angle_dragging[index];
            let min_result = widgets::slider(
                inner_x,
                y,
                inner_width,
                "Min Angle",
                state.min_angles_deg[index],
                MIN_JOINT_ANGLE_DEG,
                MAX_JOINT_ANGLE_DEG,
                &mut min_dragging,
            );
            state.min_angle_dragging[index] = min_dragging;

            y += 46.0;

            let mut max_dragging = state.max_angle_dragging[index];
            let max_result = widgets::slider(
                inner_x,
                y,
                inner_width,
                "Max Angle",
                state.max_angles_deg[index],
                MIN_JOINT_ANGLE_DEG,
                MAX_JOINT_ANGLE_DEG,
                &mut max_dragging,
            );
            state.max_angle_dragging[index] = max_dragging;

            y += 46.0;

            if min_result.changed || max_result.changed {
                state.min_angles_deg[index] = min_result.value;
                state.max_angles_deg[index] = max_result.value;

                if state.min_angles_deg[index] > state.max_angles_deg[index] {
                    let clamped = state.min_angles_deg[index].min(state.max_angles_deg[index]);
                    let upper = state.min_angles_deg[index].max(state.max_angles_deg[index]);
                    state.min_angles_deg[index] = clamped;
                    state.max_angles_deg[index] = upper;
                }

                events.joint_limit_changed = Some(index);
            }
        }
    }

    y += 10.0;

    draw_text("Display", inner_x, y, 20.0, colors::PANEL_TITLE);
    y += 28.0;

    state.show_target = widgets::checkbox(inner_x, y, "Show Target", state.show_target);
    y += 30.0;

    state.show_jacobian = widgets::checkbox(inner_x, y, "Show Jacobian", state.show_jacobian);
    y += 40.0;

    draw_text("IK Solver", inner_x, y, 20.0, colors::PANEL_TITLE);
    y += 28.0;

    let items: [(&str, bool); 4] = [
        ("Jacobian Transpose", true),
        ("Pseudo Inverse", true),
        ("Pseudo Inverse (Undamped)", true),
        ("Damped Least Squares", true),
    ];

    let dropdown_result = widgets::dropdown(
        inner_x,
        y,
        inner_width,
        "Solver",
        &items,
        state.solver_index,
        state.dropdown_open,
    );

    if dropdown_result.selected_index != state.solver_index {
        state.solver_index = dropdown_result.selected_index;
        events.solver_changed = true;
    }

    state.dropdown_open = dropdown_result.open;

    y += 30.0 + if state.dropdown_open { items.len() as f32 * 26.0 } else { 0.0 };
    y += 20.0;

    draw_text("Animation Speed", inner_x, y, 20.0, colors::PANEL_TITLE);
    y += 28.0;

    state.animation_speed = widgets::stepper(
        inner_x,
        y,
        "Steps / Frame",
        state.animation_speed,
        animation::MIN_STEPS_PER_FRAME,
        animation::MAX_STEPS_PER_FRAME,
    );

    y += 46.0;

    if widgets::button(inner_x, y, inner_width, 34.0, "Reset Arm (R)") {
        events.reset_requested = true;
    }

    events
}