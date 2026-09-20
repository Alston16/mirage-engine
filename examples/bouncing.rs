//! M3 acceptance demo: balls of different restitution dropped from the same
//! height onto a static floor. `e = 0` stops dead, `e = 1` returns to (about)
//! its drop height, and the balls in between rebound proportionally lower.
//!
//! World units are metres (gravity is 9.81 m/s²); drawing scales them by
//! `PIXELS_PER_METER`. World space uses "up is positive y", so drawing flips
//! the y axis: `screen_y = screen_height() - world_y * PIXELS_PER_METER`.
//! Press R to drop the balls again.

use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Shape, Vec2, World};

const PIXELS_PER_METER: f32 = 30.0;
const BALL_RADIUS: f32 = 0.5;
const FLOOR_TOP: f32 = 1.0;
const DROP_HEIGHT: f32 = 11.0;
const RESTITUTIONS: [f32; 5] = [0.0, 0.3, 0.6, 0.85, 1.0];

fn to_screen(x: f32, y: f32) -> (f32, f32) {
    (
        screen_width() / 2.0 + x * PIXELS_PER_METER,
        screen_height() - y * PIXELS_PER_METER,
    )
}

fn build_world() -> (World, Vec<(BodyId, f32)>) {
    let mut world = World::new();

    // A wide, thin static floor whose top surface sits at `FLOOR_TOP`.
    let (half_x, half_y) = (10.0, FLOOR_TOP / 2.0);
    world.add_body(RigidBody::new_static(
        Vec2::new(0.0, half_y),
        Shape::polygon(vec![
            Vec2::new(-half_x, -half_y),
            Vec2::new(half_x, -half_y),
            Vec2::new(half_x, half_y),
            Vec2::new(-half_x, half_y),
        ]),
    ));

    let balls = RESTITUTIONS
        .iter()
        .enumerate()
        .map(|(i, &e)| {
            let x = (i as f32 - (RESTITUTIONS.len() as f32 - 1.0) / 2.0) * 2.0;
            let ball = RigidBody::new_dynamic(Vec2::new(x, DROP_HEIGHT), 1.0, Shape::circle(BALL_RADIUS))
                .with_restitution(e);
            (world.add_body(ball), e)
        })
        .collect();

    (world, balls)
}

fn window_conf() -> Conf {
    Conf {
        window_title: "bouncing".to_owned(),
        window_width: 800,
        window_height: 450,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let (mut world, mut balls) = build_world();

    loop {
        if is_key_pressed(KeyCode::R) {
            (world, balls) = build_world();
        }

        world.step(get_frame_time());

        clear_background(BLACK);

        // Floor, and a guide line at the height the balls were dropped from.
        let (left, floor_y) = to_screen(-10.0, FLOOR_TOP);
        draw_rectangle(left, floor_y, 20.0 * PIXELS_PER_METER, FLOOR_TOP * PIXELS_PER_METER, DARKGRAY);
        let (_, drop_y) = to_screen(0.0, DROP_HEIGHT);
        draw_line(left, drop_y, left + 20.0 * PIXELS_PER_METER, drop_y, 1.0, Color::new(1.0, 1.0, 1.0, 0.25));

        for &(id, e) in &balls {
            let body = world.body(id);
            let (x, y) = to_screen(body.position.x, body.position.y);
            draw_circle(x, y, BALL_RADIUS * PIXELS_PER_METER, SKYBLUE);
            let (label_x, _) = to_screen(body.position.x - 0.45, 0.0);
            draw_text(&format!("e={e}"), label_x, drop_y - 12.0, 20.0, WHITE);
        }
        draw_text("R: drop again", 10.0, 24.0, 20.0, GRAY);

        next_frame().await;
    }
}
