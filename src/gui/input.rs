use macroquad::prelude::mouse_position;

use crate::gui::panel;

pub fn is_pointer_over_panel() -> bool {
    let (x, _) = mouse_position();
    x < panel::PANEL_WIDTH
}