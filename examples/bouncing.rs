//! M1 acceptance demo: circles falling under gravity, no collision yet.
//!
//! World space uses standard "up is positive y" convention (matching the
//! engine's gravity vector); screen space is macroquad's "down is positive
//! y", so drawing flips the y axis: `screen_y = screen_height() - world_y`.

use macroquad::prelude::*;
use mirage::{RigidBody, Vec2, World};

#[macroquad::main("bouncing")]
async fn main() {
    let mut world = World::new();

    let radii = [15.0, 22.0, 12.0, 18.0, 10.0];
    let bodies: Vec<(mirage::BodyId, f32)> = radii
        .iter()
        .enumerate()
        .map(|(i, &radius)| {
            let x = (i as f32 - (radii.len() as f32 - 1.0) / 2.0) * 70.0;
            let y = 350.0 + i as f32 * 60.0;
            let id = world.add_body(RigidBody::new_dynamic(Vec2::new(x, y), 1.0));
            (id, radius)
        })
        .collect();

    loop {
        world.step(get_frame_time());

        clear_background(BLACK);

        for &(id, radius) in &bodies {
            let body = world.body(id);
            let screen_x = screen_width() / 2.0 + body.position.x;
            let screen_y = screen_height() - body.position.y;
            draw_circle(screen_x, screen_y, radius, SKYBLUE);
        }

        next_frame().await;
    }
}
