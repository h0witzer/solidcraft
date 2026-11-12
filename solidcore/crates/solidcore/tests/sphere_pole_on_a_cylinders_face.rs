//! A ball cut from a cylinder's wall where the ball's pole lies on the wall (SolidCraft
//! #6748, `a_ball_centred_on_a_rolls_face_is_cut_where_it_turns_edge_on`): the r 6 sphere
//! about (0, d, 20) less from a cylinder r 50 about the z axis. At d = 50 both poles,
//! (0, 50, 26) and (0, 50, 14), lie on the wall, so the cut's outer loop runs through
//! them. There the faces cross at a right angle (sphere normal +-z, wall normal +y) and the
//! loop is regular: unrolled onto the wall (arc length s, height z) it is
//! `s^2 + (z - 20)^2 = 36` to fourth order in the angle, a circle of radius 6. Its exact
//! trims have no zero-speed end, no kink and no coincident control points at the poles.
//!
//! The sphere's seam meridian (longitude 0, +x) lies in the plane y = 50, tangent to the
//! wall along the line x = 0, y = 50: the seam touches the wall at both poles, and the
//! loop leaves each pole tangent to the seam, `x^2 / 100` off it (within 1e-5 for
//! |x| < 0.03 mm).
//!
//! SolidCraft's check refused the cut after main's kernel was merged: test 3 (first two
//! control points 2.1e-11 apart, on a wall trim starting at z = 26, the north pole),
//! test 13 (radius 3.1e-12 on the wall trim of the pole-to-waist quarter: its tolerance
//! 9.0076e-5 is that quarter's box diagonal 8.488 over its length 9.427, /1e4) and test
//! 15 twice (a span of control points within 1e-6). d = 49 puts the poles inside the
//! metal and checks clean; d = 50 - 2e-4 passes the loop 2e-4 under the poles.

#![allow(clippy::unwrap_used, clippy::expect_used)] // test helpers

use solidcore::boolean::api_brep::boolean_difference;
use solidcore::math::Point3;
use solidcore::primitive::api_primitives as api;
use solidcore::topology::brep::validation::{check_brep, CheckLevel};
use solidcore::topology::Brep;
use solidcore::Kernel;

/// SolidCraft's fix list (`kernel_cases/fixes.json`).
fn rig_kernel() -> Kernel {
    let names: Vec<String> =
        serde_json::from_str(include_str!("kernel_cases/fixes.json")).unwrap();
    let mut fixes = solidcore::kernel::UpstreamFixes::default();
    for name in &names {
        if let Some(f) = fixes.flag_mut(name) {
            *f = true;
        }
    }
    Kernel {
        fixes,
        ..Kernel::default()
    }
}

/// The check's failed lines of the cylinder r 50, z 0..40 (a tube r 48..50 when `tube`)
/// less the r 6 sphere about (0, `d`, 20).
fn failed_lines(d: f64, tube: bool) -> Vec<String> {
    let mut k = rig_kernel();
    let p = Point3::new;
    let mut body: Brep = api::create_cylinder(&mut k, p(0.0, 0.0, 0.0), 50.0, 40.0).unwrap();
    if tube {
        let bore = api::create_cylinder(&mut k, p(0.0, 0.0, -1.0), 48.0, 42.0).unwrap();
        body = boolean_difference(&mut k, body, bore).unwrap();
    }
    let ball = api::create_sphere(&mut k, p(0.0, d, 20.0), 6.0).unwrap();
    let mut cut = boolean_difference(&mut k, body, ball).unwrap();
    let c = check_brep(&mut k, &mut cut, CheckLevel::Level2).unwrap();
    if c.is_valid() {
        Vec::new()
    } else {
        c.lines()
    }
}

/// The control: the poles inside the wall's metal, the loop clear of them.
#[test]
fn a_ball_with_its_poles_inside_the_wall_is_cut_soundly() {
    for tube in [false, true] {
        let failed = failed_lines(49.0, tube);
        assert!(failed.is_empty(), "tube {tube}: {failed:#?}");
    }
}

/// The poles on the wall: the loop through them is regular, and so are its trims.
#[test]
fn a_ball_with_its_poles_on_the_wall_is_cut_soundly() {
    for tube in [false, true] {
        let failed = failed_lines(50.0, tube);
        assert!(failed.is_empty(), "tube {tube}: {failed:#?}");
    }
}

/// The loop passing 2e-4 and 1e-4 from the poles (the ball's centre that far inside the
/// wall): as regular as through them.
#[test]
fn a_ball_with_its_poles_just_inside_the_wall_is_cut_soundly() {
    for off in [2e-4, 1e-4] {
        let failed = failed_lines(50.0 - off, true);
        assert!(failed.is_empty(), "{off}: {failed:#?}");
    }
}
