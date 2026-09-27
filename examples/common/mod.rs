//! Shared macroquad helpers for the M4 demos (`stack`, `pyramid`). Not an
//! example itself: Cargo only treats `examples/<name>.rs` and
//! `examples/<name>/main.rs` as examples.
//!
//! World space is meters with "up is positive y"; screen space is macroquad's
//! "down is positive y", so `to_screen` flips y. The floor's top surface
//! (`y = 0`) is drawn `FLOOR_MARGIN` pixels above the bottom of the window.
#![allow(dead_code)]

use macroquad::prelude::*;
use mirage::{Rot2, Vec2};

/// Pixels per meter.
pub const PPM: f32 = 40.0;

const FLOOR_MARGIN: f32 = 60.0;

pub fn rect_vertices(half_x: f32, half_y: f32) -> Vec<Vec2> {
    vec![
        Vec2::new(-half_x, -half_y),
        Vec2::new(half_x, -half_y),
        Vec2::new(half_x, half_y),
        Vec2::new(-half_x, half_y),
    ]
}

pub fn to_screen(world: Vec2) -> (f32, f32) {
    (screen_width() / 2.0 + PPM * world.x, screen_height() - FLOOR_MARGIN - PPM * world.y)
}

pub fn draw_polygon_outline(position: Vec2, orientation: Rot2, local_vertices: &[Vec2], color: Color) {
    let n = local_vertices.len();
    for i in 0..n {
        let a = position + orientation.rotate(local_vertices[i]);
        let b = position + orientation.rotate(local_vertices[(i + 1) % n]);
        let (ax, ay) = to_screen(a);
        let (bx, by) = to_screen(b);
        draw_line(ax, ay, bx, by, 2.0, color);
    }
}

/// Draws one line of text per entry, top-left.
pub fn draw_hud(lines: &[String]) {
    for (i, line) in lines.iter().enumerate() {
        draw_text(line, 12.0, 24.0 + 22.0 * i as f32, 22.0, WHITE);
    }
}
