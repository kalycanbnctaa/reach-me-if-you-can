use macroquad::prelude::{
    Color,
    Conf,
};

use crate::math::vector2::Vector2;

pub const WINDOW_WIDTH: i32 = 1280;
pub const WINDOW_HEIGHT: i32 = 720;

pub const WINDOW_TITLE: &str = "Reach Me If You Can";

pub const GRID_SIZE: f32 = 40.0;

pub const DEFAULT_SEGMENT_THICKNESS: f32 = 8.0;
pub const DEFAULT_JOINT_RADIUS: f32 = 10.0;
pub const DEFAULT_BASE_RADIUS: f32 = 14.0;

pub const DEFAULT_BASE_POSITION: Vector2 = Vector2 {
    x: 660.0,
    y: 400.0,
};

pub const DEFAULT_SEGMENT_LENGTHS: [f32; 3] = [
    120.0,
    100.0,
    80.0,
];

pub const DEFAULT_OBSTACLE_RADIUS: f32 = 30.0;

pub const DEFAULT_OBSTACLE_HALF_EXTENTS: Vector2 = Vector2 {
    x: 30.0,
    y: 30.0,
};

pub const BACKGROUND_COLOR: Color =
    Color::new(0.094, 0.102, 0.125, 1.0);

pub const GRID_COLOR: Color =
    Color::new(0.18, 0.18, 0.20, 1.0);

pub const SEGMENT_COLOR: Color =
    Color::new(0.42, 0.77, 1.00, 1.0);

pub const JOINT_COLOR: Color =
    Color::new(1.00, 0.65, 0.10, 1.0);

pub const BASE_COLOR: Color =
    Color::new(0.90, 0.20, 0.20, 1.0);

pub fn window_conf() -> Conf {
    Conf {
        window_title: WINDOW_TITLE.to_string(),
        window_width: WINDOW_WIDTH,
        window_height: WINDOW_HEIGHT,
        high_dpi: true,
        sample_count: 4,
        window_resizable: true,
        ..Default::default()
    }
}