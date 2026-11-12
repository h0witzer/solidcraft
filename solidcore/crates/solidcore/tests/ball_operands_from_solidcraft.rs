//! SolidCraft's ball cut on a roll (#6748), from the operands SolidCraft hands SMLib: the
//! `.smb` files `crates/kernel/tests/ball_on_a_rolls_face.rs` writes with `BALL_DUMP_DIR` set.
//! Point `BALL_ROLL_SMB` and `BALL_BALL_SMB` at one case's pair. Without them the test does
//! nothing.
//!
//! The steps are those of SolidCraft's `smlib_boolean` (`crates/smlib/src/core/boolean.rs`):
//! `boolean` (difference), then `validate_and_update_tolerances(update, Level2)`, then
//! `check_brep(Level2)`. They run under SolidCraft's preset (`crates/smlib/src/core.rs`
//! `fixes()`): `kernel_cases/fixes.json` plus the flags the preset sets that the list lacks.
//! `BALL_SKIP_UPDATE=1` leaves out the tolerance update. `BALL_FIX_OFF=a,b` turns those flags
//! off, for a bisection.

#![allow(clippy::unwrap_used, clippy::expect_used)] // test helpers

use solidcore::boolean::api_brep::boolean_difference;
use solidcore::geom::attribute::AttributeStore;
use solidcore::kernel::UpstreamFixes;
use solidcore::topology::api_files::read_brep_from_file;
use solidcore::topology::brep::p10::TopologyEnv;
use solidcore::topology::brep::p3::topology_ref;
use solidcore::topology::brep::validation::{
    check_brep, validate_and_update_tolerances, CheckLevel,
};
use solidcore::topology::topology::NotifyRecords;
use solidcore::{Brep, Kernel};

/// The flags SolidCraft's preset sets on top of `kernel_cases/fixes.json`.
const PRESET_ONLY: [&str; 6] = [
    "brep_ray_classifies_current_trims",
    "closed_smooth_pole_tangent_limit",
    "face_swap_uv_transposes_trims",
    "fillet_chamfer_corner",
    "fillet_setback_beside_curved_faces",
    "planar_section_face_tolerance",
];

/// SolidCraft's preset, less the flags named in `BALL_FIX_OFF`.
fn kernel() -> Kernel {
    let mut names: Vec<String> =
        serde_json::from_str(include_str!("kernel_cases/fixes.json")).unwrap();
    names.extend(PRESET_ONLY.iter().map(|s| (*s).to_string()));
    let off: Vec<String> = std::env::var("BALL_FIX_OFF")
        .map(|v| v.split(',').map(|s| s.trim().to_string()).collect())
        .unwrap_or_default();
    let mut f = UpstreamFixes::default();
    for name in &names {
        if off.contains(name) {
            continue;
        }
        if let Some(flag) = f.flag_mut(name) {
            *flag = true;
        }
    }
    Kernel {
        fixes: f,
        ..Kernel::default()
    }
}

fn read(k: &mut Kernel, path: &str) -> Brep {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut attributes = AttributeStore::new();
    let mut records = NotifyRecords::new();
    let mut env = TopologyEnv {
        all_attributes: &[],
        attributes: &mut attributes,
        key_of: &|e| topology_ref(0, e),
        records: &mut records,
        next_key: 1 << 50,
    };
    let mut b = read_brep_from_file(k, Some(bytes), false, false, &mut env).unwrap();
    b.refresh_cached_topology();
    b
}

#[test]
fn solidcrafts_ball_cut_on_a_roll_checks_clean() {
    let (Ok(roll), Ok(ball)) = (std::env::var("BALL_ROLL_SMB"), std::env::var("BALL_BALL_SMB"))
    else {
        return;
    };
    let mut k = kernel();
    let a = read(&mut k, &roll);
    let b = read(&mut k, &ball);
    let mut r = boolean_difference(&mut k, a, b).unwrap();
    if std::env::var("BALL_SKIP_UPDATE").is_err() {
        validate_and_update_tolerances(&mut k, &mut r, false, None, CheckLevel::Level2 as u8)
            .unwrap();
    }
    let c = check_brep(&mut k, &mut r, CheckLevel::Level2).unwrap();
    assert!(c.is_valid(), "{:#?}", c.lines());
}
