//! Projectile-motion demo: circles launched from the ground at a fixed speed
//! but different angles, each tracing the parabola that constant gravity and
//! semi-implicit Euler integration produce. No collisions between the balls
//! are involved before landing (gravity applies no torque through a circle's
//! center of mass, so nothing tumbles); a static floor stops them once they
//! land, with a little restitution so the bounce stays visible.
//!
//! Each ball's actual path (solid trail) is drawn alongside the closed-form
//! kinematic parabola for the same launch angle (dashed), computed with
//! `x(t) = x0 + vx*t`, `y(t) = y0 + vy*t - 1/2*g*t^2`. They should overlap
//! almost exactly while airborne — that's the integrator reproducing ideal
//! projectile motion, not just "stuff falls down".
//!
//! World units are metres (gravity is 9.81 m/s², matching `World::new`'s
//! default); drawing scales them by `PIXELS_PER_METER`. World space uses "up
//! is positive y", so drawing flips the y axis. Press R to launch again.

use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Shape, Vec2, World};

const PIXELS_PER_METER: f32 = 18.0;
const ORIGIN_X: f32 = -18.0;
const FLOOR_TOP: f32 = 0.0;
const BALL_RADIUS: f32 = 0.35;
const LAUNCH_SPEED: f32 = 14.0;
const GRAVITY: f32 = 9.81;
const ANGLES_DEG: [f32; 4] = [30.0, 45.0, 60.0, 75.0];
const COLORS: [Color; 4] = [SKYBLUE, LIME, ORANGE, PINK];

fn to_screen(x: f32, y: f32) -> (f32, f32) {
    (
        60.0 + (x - ORIGIN_X) * PIXELS_PER_METER,
        screen_height() - 40.0 - y * PIXELS_PER_METER,
    )
}

struct Ball {
    id: BodyId,
    angle_deg: f32,
    velocity0: Vec2,
    color: Color,
    trail: Vec<Vec2>,
}

fn launch_velocity(angle_deg: f32) -> Vec2 {
    let angle = angle_deg.to_radians();
    Vec2::new(LAUNCH_SPEED * angle.cos(), LAUNCH_SPEED * angle.sin())
}

fn build_world() -> (World, Vec<Ball>) {
    let mut world = World::new();

    // A wide, thin static floor whose top surface sits at `FLOOR_TOP`.
    let (half_x, half_y) = (40.0, 0.5);
    world.add_body(RigidBody::new_static(
        Vec2::new(0.0, FLOOR_TOP - half_y),
        Shape::polygon(vec![
            Vec2::new(-half_x, -half_y),
            Vec2::new(half_x, -half_y),
            Vec2::new(half_x, half_y),
            Vec2::new(-half_x, half_y),
        ]),
    ));

    let balls = ANGLES_DEG
        .iter()
        .zip(COLORS)
        .map(|(&angle_deg, color)| {
            let velocity0 = launch_velocity(angle_deg);
            let start = Vec2::new(ORIGIN_X, FLOOR_TOP + BALL_RADIUS);
            let mut body = RigidBody::new_dynamic(start, 1.0, Shape::circle(BALL_RADIUS)).with_restitution(0.4);
            body.velocity = velocity0;
            Ball {
                id: world.add_body(body),
                angle_deg,
                velocity0,
                color,
                trail: vec![start],
            }
        })
        .collect();

    (world, balls)
}

/// Samples the closed-form projectile parabola for `start`/`velocity0` from
/// t = 0 until it returns to `FLOOR_TOP`, for comparison against the engine's
/// actual (integrated) trajectory.
fn analytic_parabola(start: Vec2, velocity0: Vec2) -> Vec<Vec2> {
    let flight_time = 2.0 * velocity0.y / GRAVITY;
    let steps = 60;
    (0..=steps)
        .map(|i| {
            let t = flight_time * i as f32 / steps as f32;
            Vec2::new(
                start.x + velocity0.x * t,
                start.y + velocity0.y * t - 0.5 * GRAVITY * t * t,
            )
        })
        .collect()
}

fn draw_path(points: &[Vec2], color: Color, dashed: bool) {
    for pair in points.windows(2).step_by(if dashed { 2 } else { 1 }) {
        let (ax, ay) = to_screen(pair[0].x, pair[0].y);
        let (bx, by) = to_screen(pair[1].x, pair[1].y);
        draw_line(ax, ay, bx, by, 2.0, color);
    }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "parabola".to_owned(),
        window_width: 900,
        window_height: 500,
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

        for ball in &mut balls {
            let position = world.body(ball.id).position;
            if ball.trail.last().is_some_and(|&p| p != position) {
                ball.trail.push(position);
            }
        }

        clear_background(BLACK);

        // Floor.
        let (left, floor_y) = to_screen(ORIGIN_X, FLOOR_TOP);
        draw_rectangle(left, floor_y, 36.0 * PIXELS_PER_METER, 500.0, DARKGRAY);

        for ball in &balls {
            let dashed_color = Color::new(ball.color.r, ball.color.g, ball.color.b, 0.35);
            draw_path(&analytic_parabola(Vec2::new(ORIGIN_X, FLOOR_TOP + BALL_RADIUS), ball.velocity0), dashed_color, true);
            draw_path(&ball.trail, ball.color, false);

            let body = world.body(ball.id);
            let (x, y) = to_screen(body.position.x, body.position.y);
            draw_circle(x, y, BALL_RADIUS * PIXELS_PER_METER, ball.color);

            let (label_x, label_y) = to_screen(ORIGIN_X, FLOOR_TOP);
            draw_text(
                format!("{}\u{b0}", ball.angle_deg),
                label_x - 25.0,
                label_y - 10.0 - ANGLES_DEG.iter().position(|a| *a == ball.angle_deg).unwrap() as f32 * 20.0,
                20.0,
                ball.color,
            );
        }

        draw_text("solid: simulated path   dashed: ideal parabola", 10.0, 24.0, 20.0, GRAY);
        draw_text("R: launch again", 10.0, 46.0, 20.0, GRAY);

        next_frame().await;
    }
}
