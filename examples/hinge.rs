//! M6 acceptance demo: a rod pinned at one end swings under gravity like a
//! pendulum. Watch the pinned end (the red ring) stay locked to the pin
//! (the yellow dot) through the whole swing — that's the revolute joint
//! holding. Press R to release it again from its start angle.
//!
//! World units are meters (gravity is 9.81 m/s²); drawing scales them by
//! `common::PPM`. World space uses "up is positive y".

mod common;

use common::*;
use macroquad::prelude::*;
use mirage::{BodyId, RevoluteJoint, RigidBody, Rot2, Shape, Vec2, World};

const PIN_POS: Vec2 = Vec2 { x: 0.0, y: 5.0 };
const HALF_LENGTH: f32 = 1.5;
const ROD_HALF_WIDTH: f32 = 0.06;
const ROD_MASS: f32 = 1.0;
const RELEASE_ANGLE_DEG: f32 = 45.0;

fn rod_vertices() -> Vec<Vec2> {
    rect_vertices(ROD_HALF_WIDTH, HALF_LENGTH)
}

struct Scene {
    world: World,
    pin: BodyId,
    rod: BodyId,
}

/// A rod pinned at its local `(0, HALF_LENGTH)` end to a fixed point,
/// released from `RELEASE_ANGLE_DEG` off straight-down — the two anchors
/// coincide exactly at creation (README § Joints; spec.md User Story 1
/// scenario 1).
fn build_scene() -> Scene {
    let mut world = World::new();
    let pin = world.add_body(RigidBody::new_static(PIN_POS, Shape::circle(0.05)));

    let angle = RELEASE_ANGLE_DEG.to_radians();
    let mut rod = RigidBody::new_dynamic(Vec2::ZERO, ROD_MASS, Shape::polygon(rod_vertices()));
    rod.orientation = Rot2::new(angle);
    rod.position = PIN_POS - rod.orientation.rotate(Vec2::new(0.0, HALF_LENGTH));
    let rod_id = world.add_body(rod);

    world.add_joint(RevoluteJoint::new(pin, rod_id, Vec2::ZERO, Vec2::new(0.0, HALF_LENGTH)));

    Scene { world, pin, rod: rod_id }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "hinge".to_owned(),
        window_width: 800,
        window_height: 600,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut scene = build_scene();

    loop {
        if is_key_pressed(KeyCode::R) {
            scene = build_scene();
        }

        scene.world.step(get_frame_time());

        clear_background(BLACK);

        let pin_body = scene.world.body(scene.pin);
        let rod_body = scene.world.body(scene.rod);
        let rod_anchor = rod_body.position + rod_body.orientation.rotate(Vec2::new(0.0, HALF_LENGTH));

        draw_polygon_outline(rod_body.position, rod_body.orientation, &rod_vertices(), SKYBLUE);

        let (pin_x, pin_y) = to_screen(pin_body.position);
        draw_circle(pin_x, pin_y, 5.0, YELLOW);
        let (anchor_x, anchor_y) = to_screen(rod_anchor);
        draw_circle_lines(anchor_x, anchor_y, 8.0, 2.0, RED);

        draw_hud(&[
            format!("angle: {:+.1} deg", rod_body.orientation.angle().to_degrees()),
            "R: release again".to_owned(),
        ]);

        next_frame().await;
    }
}
