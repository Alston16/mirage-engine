//! M2 acceptance demo: a box falls onto a rotated ramp and the resulting
//! contact point(s)/normal(s) are debug-drawn, so the milestone's "done
//! when" criterion (correct contacts for a box resting on a rotated ramp)
//! can be verified visually. Since M3 contacts are resolved, so the box
//! reacts to the ramp — but there is no friction until M4, so it slides down
//! rather than staying put.
//!
//! World space uses "up is positive y" (matching the engine's gravity
//! vector); screen space is macroquad's "down is positive y", so drawing
//! flips the y axis: `screen_y = screen_height() - world_y`.

use macroquad::prelude::*;
use mirage::{BodyId, RigidBody, Rot2, Shape, Vec2, World};

fn rect_vertices(half_x: f32, half_y: f32) -> Vec<Vec2> {
    vec![
        Vec2::new(-half_x, -half_y),
        Vec2::new(half_x, -half_y),
        Vec2::new(half_x, half_y),
        Vec2::new(-half_x, half_y),
    ]
}

fn to_screen(world: Vec2) -> (f32, f32) {
    (screen_width() / 2.0 + world.x, screen_height() - world.y)
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

#[macroquad::main("ramp")]
async fn main() {
    let mut world = World::new();

    let ramp_vertices = rect_vertices(300.0, 15.0);
    let mut ramp = RigidBody::new_static(
        Vec2::new(0.0, 120.0),
        Shape::polygon(ramp_vertices.clone()),
    );
    ramp.orientation = Rot2::new(0.25);
    let ramp_id = world.add_body(ramp);

    let box_vertices = rect_vertices(40.0, 40.0);
    let box_id = world.add_body(RigidBody::new_dynamic(
        Vec2::new(30.0, 220.0),
        1.0,
        Shape::polygon(box_vertices.clone()),
    ));

    let bodies: [(BodyId, Vec<Vec2>); 2] = [(ramp_id, ramp_vertices), (box_id, box_vertices)];

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
                draw_circle(px, py, 5.0, RED);

                let tip = contact.point + contact.normal * 30.0;
                let (tx, ty) = to_screen(tip);
                draw_line(px, py, tx, ty, 3.0, YELLOW);
            }
        }

        next_frame().await;
    }
}
