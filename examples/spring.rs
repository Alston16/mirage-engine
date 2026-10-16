//! M7 acceptance demo: three boxes hang from fixed hooks on springs, each
//! released 1 m below its equilibrium. Left: under-damped (`ζ = 0.1`) —
//! it rings down. Middle: critically damped — it rises to its equilibrium
//! tick and stops without crossing it. Right: undamped — it keeps the same
//! amplitude for as long as you watch. Press R to release them again.
//!
//! World units are meters (gravity is 9.81 m/s²); drawing scales them by
//! `common::PPM`. World space uses "up is positive y".

mod common;

use common::*;
use macroquad::prelude::*;
use mirage::{BodyId, DistanceJoint, RigidBody, Shape, Vec2, World};

const K: f32 = 10.0;
const MASS: f32 = 1.0;
const REST: f32 = 2.0;
const HALF: f32 = 0.5;
const RELEASE: f32 = 1.0;
const HOOK_Y: f32 = 11.0;

/// Spring length at which the spring force balances the box's weight.
const EQUILIBRIUM: f32 = REST + MASS * 9.81 / K;

struct Hanger {
    label: &'static str,
    hook: BodyId,
    bx: BodyId,
}

struct Scene {
    world: World,
    hangers: Vec<Hanger>,
}

fn top_anchor() -> Vec2 {
    Vec2::new(0.0, HALF)
}

fn build_scene() -> Scene {
    let mut world = World::new();
    let critical = DistanceJoint::critical_damping(K, MASS);
    let configs = [("under-damped", -5.0, 0.1 * critical), ("critical", 0.0, critical), ("undamped", 5.0, 0.0)];

    let hangers = configs
        .into_iter()
        .map(|(label, x, damping)| {
            let hook = world.add_body(RigidBody::new_static(Vec2::new(x, HOOK_Y), Shape::circle(0.05)));
            let anchor_y = HOOK_Y - EQUILIBRIUM - RELEASE;
            let bx = world.add_body(RigidBody::new_dynamic(
                Vec2::new(x, anchor_y - HALF),
                MASS,
                Shape::polygon(rect_vertices(HALF, HALF)),
            ));
            world.add_distance_joint(
                DistanceJoint::new(hook, bx, Vec2::ZERO, top_anchor())
                    .with_length(REST)
                    .with_spring(K, damping),
            );
            Hanger { label, hook, bx }
        })
        .collect();

    Scene { world, hangers }
}

fn window_conf() -> Conf {
    Conf {
        window_title: "spring".to_owned(),
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

        let mut hud = Vec::new();
        for hanger in &scene.hangers {
            let hook = scene.world.body(hanger.hook);
            let bx = scene.world.body(hanger.bx);
            let anchor = bx.position + bx.orientation.rotate(top_anchor());

            // Equilibrium tick: where the box's top should come to rest.
            let (tick_l, tick_y) = to_screen(Vec2::new(hook.position.x - 0.8, HOOK_Y - EQUILIBRIUM));
            let (tick_r, _) = to_screen(Vec2::new(hook.position.x + 0.8, HOOK_Y - EQUILIBRIUM));
            draw_line(tick_l, tick_y, tick_r, tick_y, 1.0, DARKGRAY);

            let (hx, hy) = to_screen(hook.position);
            let (ax, ay) = to_screen(anchor);
            draw_line(hx, hy, ax, ay, 2.0, ORANGE);
            draw_circle(hx, hy, 5.0, YELLOW);
            draw_polygon_outline(bx.position, bx.orientation, &rect_vertices(HALF, HALF), SKYBLUE);

            let stretch = (anchor - hook.position).length() - EQUILIBRIUM;
            hud.push(format!("{:>12}: L - L_eq = {:+.3} m", hanger.label, stretch));
        }
        hud.push("R: release again".to_owned());
        draw_hud(&hud);

        next_frame().await;
    }
}
