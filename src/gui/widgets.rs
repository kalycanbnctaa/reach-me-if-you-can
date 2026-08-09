use macroquad::prelude::*;

use crate::gui::colors;

pub fn point_in_rect(px: f32, py: f32, x: f32, y: f32, w: f32, h: f32) -> bool {
    px >= x && px <= x + w && py >= y && py <= y + h
}

pub struct SliderResult {
    pub value: f32,
    pub changed: bool,
}

pub fn slider(
    x: f32,
    y: f32,
    width: f32,
    label: &str,
    value: f32,
    min: f32,
    max: f32,
    dragging: &mut bool,
) -> SliderResult {
    let track_height = 6.0;
    let track_y = y + 22.0;
    let handle_radius = 8.0;

    draw_text(&format!("{} : {:.1}", label, value), x, y, 18.0, colors::LABEL_TEXT);
    draw_rectangle(x, track_y, width, track_height, colors::SLIDER_TRACK);

    let ratio = ((value - min) / (max - min).max(f32::EPSILON)).clamp(0.0, 1.0);
    let fill_width = width * ratio;
    draw_rectangle(x, track_y, fill_width, track_height, colors::SLIDER_FILL);

    let handle_x = x + fill_width;
    let handle_y = track_y + track_height / 2.0;

    let (mouse_x, mouse_y) = mouse_position();
    let hovering = point_in_rect(
        mouse_x,
        mouse_y,
        handle_x - handle_radius,
        handle_y - handle_radius,
        handle_radius * 2.0,
        handle_radius * 2.0,
    ) || point_in_rect(mouse_x, mouse_y, x, track_y - 4.0, width, track_height + 8.0);

    if is_mouse_button_pressed(MouseButton::Left) && hovering {
        *dragging = true;
    }

    if is_mouse_button_released(MouseButton::Left) {
        *dragging = false;
    }

    let mut new_value = value;
    let mut changed = false;

    if *dragging {
        let clamped_x = mouse_x.clamp(x, x + width);
        let new_ratio = (clamped_x - x) / width.max(f32::EPSILON);
        new_value = min + new_ratio * (max - min);
        changed = (new_value - value).abs() > f32::EPSILON;
    }

    let handle_color = if *dragging || hovering {
        colors::SLIDER_HANDLE_HOVER
    } else {
        colors::SLIDER_HANDLE
    };

    draw_circle(handle_x, handle_y, handle_radius, handle_color);
    draw_circle_lines(handle_x, handle_y, handle_radius, 1.5, BLACK);

    SliderResult { value: new_value, changed }
}

pub fn checkbox(x: f32, y: f32, label: &str, checked: bool) -> bool {
    let box_size = 18.0;

    draw_rectangle(x, y, box_size, box_size, colors::CHECKBOX_BOX);
    draw_rectangle_lines(x, y, box_size, box_size, 1.5, colors::CHECKBOX_BORDER);

    if checked {
        draw_line(x + 3.0, y + 9.0, x + 7.5, y + 14.0, 2.5, colors::CHECKBOX_CHECK);
        draw_line(x + 7.5, y + 14.0, x + 15.0, y + 4.0, 2.5, colors::CHECKBOX_CHECK);
    }

    draw_text(label, x + box_size + 10.0, y + box_size - 4.0, 18.0, colors::LABEL_TEXT);

    let (mouse_x, mouse_y) = mouse_position();
    let clicked = is_mouse_button_pressed(MouseButton::Left)
        && point_in_rect(mouse_x, mouse_y, x, y, box_size + 140.0, box_size);

    if clicked { !checked } else { checked }
}

pub struct DropdownResult {
    pub selected_index: usize,
    pub open: bool,
}

pub fn dropdown(
    x: f32,
    y: f32,
    width: f32,
    label: &str,
    items: &[(&str, bool)],
    selected_index: usize,
    open: bool,
) -> DropdownResult {
    let button_height = 26.0;

    draw_text(label, x, y - 4.0, 18.0, colors::LABEL_TEXT);

    let button_y = y;
    let (mouse_x, mouse_y) = mouse_position();
    let hovering_button = point_in_rect(mouse_x, mouse_y, x, button_y, width, button_height);

    draw_rectangle(
        x,
        button_y,
        width,
        button_height,
        if hovering_button { colors::BUTTON_HOVER } else { colors::BUTTON_IDLE },
    );
    draw_rectangle_lines(x, button_y, width, button_height, 1.5, colors::PANEL_BORDER);

    let current_label = items.get(selected_index).map(|(name, _)| *name).unwrap_or("");
    draw_text(current_label, x + 10.0, button_y + 18.0, 18.0, colors::BUTTON_TEXT);
    draw_text(
        if open { "-" } else { "+" },
        x + width - 20.0,
        button_y + 18.0,
        18.0,
        colors::BUTTON_TEXT,
    );

    let mut new_open = open;
    let mut new_selected = selected_index;

    if is_mouse_button_pressed(MouseButton::Left) {
        if hovering_button {
            new_open = !open;
        } else if open {
            for (index, (_, enabled)) in items.iter().enumerate() {
                let item_y = button_y + button_height + index as f32 * button_height;
                let hovering_item = point_in_rect(mouse_x, mouse_y, x, item_y, width, button_height);

                if hovering_item && *enabled {
                    new_selected = index;
                    new_open = false;
                }
            }

            let total_height = button_height * (items.len() as f32 + 1.0);
            if !point_in_rect(mouse_x, mouse_y, x, button_y, width, total_height) {
                new_open = false;
            }
        }
    }

    if new_open {
        for (index, (name, enabled)) in items.iter().enumerate() {
            let item_y = button_y + button_height + index as f32 * button_height;
            let hovering_item = point_in_rect(mouse_x, mouse_y, x, item_y, width, button_height);

            let background = if !*enabled {
                colors::BUTTON_DISABLED
            } else if hovering_item {
                colors::BUTTON_HOVER
            } else {
                colors::DROPDOWN_LIST_BG
            };

            draw_rectangle(x, item_y, width, button_height, background);
            draw_rectangle_lines(x, item_y, width, button_height, 1.0, colors::PANEL_BORDER);

            let text_color = if *enabled { colors::BUTTON_TEXT } else { colors::BUTTON_TEXT_DISABLED };
            draw_text(name, x + 10.0, item_y + 18.0, 18.0, text_color);
        }
    }

    DropdownResult { selected_index: new_selected, open: new_open }
}

pub fn button(x: f32, y: f32, width: f32, height: f32, label: &str) -> bool {
    let (mouse_x, mouse_y) = mouse_position();
    let hovering = point_in_rect(mouse_x, mouse_y, x, y, width, height);

    draw_rectangle(x, y, width, height, if hovering { colors::BUTTON_HOVER } else { colors::BUTTON_IDLE });
    draw_rectangle_lines(x, y, width, height, 1.5, colors::PANEL_BORDER);
    draw_text(label, x + 10.0, y + height / 2.0 + 6.0, 18.0, colors::BUTTON_TEXT);

    is_mouse_button_pressed(MouseButton::Left) && hovering
}

pub fn stepper(x: f32, y: f32, label: &str, value: usize, min: usize, max: usize) -> usize {
    let box_size = 26.0;

    draw_text(&format!("{} : {}", label, value), x, y, 18.0, colors::LABEL_TEXT);

    let button_y = y + 8.0;
    let minus_x = x;
    let plus_x = x + box_size + 44.0;

    let (mouse_x, mouse_y) = mouse_position();
    let hovering_minus = point_in_rect(mouse_x, mouse_y, minus_x, button_y, box_size, box_size);
    let hovering_plus = point_in_rect(mouse_x, mouse_y, plus_x, button_y, box_size, box_size);

    draw_rectangle(
        minus_x,
        button_y,
        box_size,
        box_size,
        if hovering_minus { colors::BUTTON_HOVER } else { colors::BUTTON_IDLE },
    );
    draw_rectangle_lines(minus_x, button_y, box_size, box_size, 1.5, colors::PANEL_BORDER);
    draw_text("-", minus_x + 10.0, button_y + 19.0, 22.0, colors::BUTTON_TEXT);

    draw_rectangle(
        plus_x,
        button_y,
        box_size,
        box_size,
        if hovering_plus { colors::BUTTON_HOVER } else { colors::BUTTON_IDLE },
    );
    draw_rectangle_lines(plus_x, button_y, box_size, box_size, 1.5, colors::PANEL_BORDER);
    draw_text("+", plus_x + 6.0, button_y + 19.0, 22.0, colors::BUTTON_TEXT);

    draw_text(&value.to_string(), minus_x + box_size + 16.0, button_y + 19.0, 20.0, colors::VALUE_TEXT);

    let mut new_value = value;

    if is_mouse_button_pressed(MouseButton::Left) {
        if hovering_minus && value > min {
            new_value = value - 1;
        } else if hovering_plus && value < max {
            new_value = value + 1;
        }
    }

    new_value
}