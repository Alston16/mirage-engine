//! M4 acceptance demo — the MVP's headline check: a 10-box tower on a static
//! floor. It should stand without visible jitter, drift or sinking for a full
//! 60 seconds (press `C` to toggle contact points/normals).
//!
//! World units are meters ("up is positive y"); `common::PPM` pixels make one
//! meter. The HUD shows elapsed simulation time, how far the top box has
//! sunk from its ideal touching height, the largest horizontal drift of any
//! box, and FPS (the demo should hold 60).

mod common;

use common::*;
use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Shape, Vec2, World};

const BOXES: usize = 10;

#[macroquad::main("stack")]
async fn main() {
    let mut world = World::new();
    world.add_body(RigidBody::new_static(
        Vec2::new(0.0, -10.0),
        Shape::polygon(rect_vertices(100.0, 10.0)),
    ));

    let box_vertices = rect_vertices(0.5, 0.5);
    let ids: Vec<BodyId> = (0..BOXES)
        .map(|i| {
            world.add_body(RigidBody::new_dynamic(
                Vec2::new(0.0, 0.5 + i as f32),
                1.0,
                Shape::polygon(box_vertices.clone()),
            ))
        })
        .collect();
    let top = *ids.last().unwrap();
    let ideal_top_y = BOXES as f32 - 0.5;

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

        let sink = ideal_top_y - world.body(top).position.y;
        let drift = ids.iter().map(|&id| world.body(id).position.x.abs()).fold(0.0, f32::max);
        draw_hud(&[
            format!("t = {elapsed:5.1} s   FPS {}", get_fps()),
            format!("top box sunk {:.3} box heights   max drift {:.3}", sink, drift),
            "C: toggle contacts".to_string(),
        ]);

        next_frame().await;
    }
}
