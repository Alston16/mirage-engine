//! M4 friction demo: a box on a shallow ramp and a box on a steep ramp. With
//! the default friction (μ = 0.5 between the box and a ramp) the slide/hold
//! threshold is `atan μ ≈ 26.6°`, so the shallow (15°) box stays put and the
//! steep (35°) box slides off — the M4 "done when" made visible. Contact
//! points and normals from M2 are still debug-drawn.
//!
//! World space is meters with "up is positive y" (matching the engine's
//! gravity); screen space is macroquad's "down is positive y", so drawing
//! flips the y axis. `PPM` pixels make one meter.

use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Rot2, Shape, Vec2, World};

/// Pixels per meter.
const PPM: f32 = 40.0;

fn rect_vertices(half_x: f32, half_y: f32) -> Vec<Vec2> {
    vec![
        Vec2::new(-half_x, -half_y),
        Vec2::new(half_x, -half_y),
        Vec2::new(half_x, half_y),
        Vec2::new(-half_x, half_y),
    ]
}

fn to_screen(world: Vec2) -> (f32, f32) {
    (screen_width() / 2.0 + PPM * world.x, screen_height() - 140.0 - PPM * world.y)
}

fn draw_polygon_outline(position: Vec2, orientation: Rot2, local_vertices: &[Vec2], color: Color) {
    let n = local_vertices.len();
    for i in 0..n {
        let world_a = position + orientation.rotate(local_vertices[i]);
        let world_b = position + orientation.rotate(local_vertices[(i + 1) % n]);
        let (ax, ay) = to_screen(world_a);
        let (bx, by) = to_screen(world_b);
        draw_line(ax, ay, bx, by, 2.0, color);
    }
}

/// Adds a static ramp of half-length `half_len` centered at `center`, tilted
/// so its right end is higher by `angle` radians, and a 1 m box resting on
/// it. Returns `(ramp_id, box_id)`.
fn add_ramp_with_box(world: &mut World, center: Vec2, half_len: f32, angle: f32) -> (BodyId, BodyId) {
    let mut ramp = RigidBody::new_static(center, Shape::polygon(rect_vertices(half_len, 0.25)));
    ramp.orientation = Rot2::new(angle);
    let ramp_id = world.add_body(ramp);

    // Exactly touching: box center is 0.25 (ramp half-thickness) + 0.5 (box
    // half-extent) along the ramp's surface normal, near the ramp's middle.
    let rot = Rot2::new(angle);
    let box_center = center + rot.rotate(Vec2::new(0.0, 0.25 + 0.5));
    let mut body = RigidBody::new_dynamic(box_center, 1.0, Shape::polygon(rect_vertices(0.5, 0.5)));
    body.orientation = rot;
    let box_id = world.add_body(body);
    (ramp_id, box_id)
}

#[macroquad::main("ramp")]
async fn main() {
    let mut world = World::new();

    // Ramp angles (radians) and friction, shown in the HUD.
    let shallow = 15.0_f32.to_radians();
    let steep = 35.0_f32.to_radians();
    let mu = 0.5_f32; // default body friction; √(0.5·0.5) between box and ramp
    let (ramp_a, box_a) = add_ramp_with_box(&mut world, Vec2::new(-9.0, 3.0), 6.0, shallow);
    let (ramp_b, box_b) = add_ramp_with_box(&mut world, Vec2::new(9.0, 3.0), 6.0, steep);

    let ramp_vertices = rect_vertices(6.0, 0.25);
    let box_vertices = rect_vertices(0.5, 0.5);
    let bodies: [(BodyId, &Vec<Vec2>); 4] = [
        (ramp_a, &ramp_vertices),
        (box_a, &box_vertices),
        (ramp_b, &ramp_vertices),
        (box_b, &box_vertices),
    ];

    loop {
        world.step(get_frame_time());

        clear_background(BLACK);

        for (id, local_vertices) in &bodies {
            let body = world.body(*id);
            draw_polygon_outline(body.position, body.orientation, local_vertices, SKYBLUE);
        }

        for manifold in world.contacts() {
            for contact in &manifold.points {
                let (px, py) = to_screen(contact.point);
                draw_circle(px, py, 4.0, RED);

                let tip = contact.point + contact.normal * 0.8;
                let (tx, ty) = to_screen(tip);
                draw_line(px, py, tx, ty, 2.0, YELLOW);
            }
        }

        draw_text(
            format!("left: {:.0}\u{b0} ramp   right: {:.0}\u{b0} ramp   \u{3bc} = {mu}   atan \u{3bc} = {:.1}\u{b0}", shallow.to_degrees(), steep.to_degrees(), mu.atan().to_degrees()),
            12.0,
            24.0,
            22.0,
            WHITE,
        );
        draw_text("the shallow box should hold; the steep box should slide", 12.0, 46.0, 22.0, WHITE);

        next_frame().await;
    }
}
