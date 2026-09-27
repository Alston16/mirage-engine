//! M4 stress case: a 5-row pyramid of 15 boxes on a static floor. Every row
//! should stay in place (press `C` to toggle contact points/normals).
//!
//! Units and drawing follow `stack.rs` (meters, `common::PPM` pixels each).

mod common;

use common::*;
use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Shape, Vec2, World};

const ROWS: usize = 5;

#[macroquad::main("pyramid")]
async fn main() {
    let mut world = World::new();
    world.add_body(RigidBody::new_static(
        Vec2::new(0.0, -10.0),
        Shape::polygon(rect_vertices(100.0, 10.0)),
    ));

    let box_vertices = rect_vertices(0.5, 0.5);
    let mut ids: Vec<BodyId> = Vec::new();
    for r in 0..ROWS {
        let count = ROWS - r;
        for i in 0..count {
            let x = i as f32 - (count as f32 - 1.0) / 2.0;
            ids.push(world.add_body(RigidBody::new_dynamic(
                Vec2::new(x, 0.5 + r as f32),
                1.0,
                Shape::polygon(box_vertices.clone()),
            )));
        }
    }
    let start: Vec<Vec2> = ids.iter().map(|&id| world.body(id).position).collect();

    let mut elapsed = 0.0_f32;
    let mut show_contacts = false;

    loop {
        let dt = get_frame_time();
        world.step(dt);
        elapsed += dt;
        if is_key_pressed(KeyCode::C) {
            show_contacts = !show_contacts;
        }

        clear_background(BLACK);

        let (_, floor_y) = to_screen(Vec2::ZERO);
        draw_line(0.0, floor_y, screen_width(), floor_y, 2.0, GRAY);

        for &id in &ids {
            let body = world.body(id);
            draw_polygon_outline(body.position, body.orientation, &box_vertices, SKYBLUE);
        }

        if show_contacts {
            for manifold in world.contacts() {
                for contact in &manifold.points {
                    let (px, py) = to_screen(contact.point);
                    draw_circle(px, py, 3.0, RED);
                    let (tx, ty) = to_screen(contact.point + contact.normal * 0.4);
                    draw_line(px, py, tx, ty, 2.0, YELLOW);
                }
            }
        }

        let moved = ids
            .iter()
            .zip(&start)
            .map(|(&id, s)| (world.body(id).position - *s).length())
            .fold(0.0, f32::max);
        draw_hud(&[
            format!("t = {elapsed:5.1} s   FPS {}", get_fps()),
            format!("largest displacement from start {moved:.3} box widths"),
            "C: toggle contacts".to_string(),
        ]);

        next_frame().await;
    }
}
