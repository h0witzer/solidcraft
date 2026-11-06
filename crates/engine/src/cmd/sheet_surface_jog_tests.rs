//! Jog on a sheet made from surfaces (#6046, #6748): an offset step across a flat panel, made as
//! two folds. These are the cases of the flat-first Jog (#6047), the spec for the rebuild.

use std::f64::consts::FRAC_PI_2;

use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn volume(s: &mut Session) -> f64 {
    run(s, "inspect.measure", json!({}))["total"]["volume_mm3"].as_f64().unwrap_or(f64::NAN)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

fn flat(s: &mut Session) -> Value {
    run(s, "sheet.flat_pattern", json!({}))
}

fn bbox(s: &mut Session) -> Value {
    run(s, "inspect.measure", json!({}))["bodies"][0]["bbox"].clone()
}

/// A w x h base flange on XY.
fn base(s: &mut Session, w: f64, h: f64) {
    run(s, "sketch.create", json!({"plane": "XY", "name": "Base"}));
    run(s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [w, h]}));
    run(s, "sketch.finish", json!({}));
    run(s, "sheet.flange", json!({"sketch": "Base"}));
}

/// A 120 x 60 plate of the rule 2 mm thick, R2, K 0.4.
fn jog_plate(s: &mut Session) {
    run(s, "sheet.manage_rules", json!({"name": "Jog", "active": true, "thickness": "2 mm", "k_factor": 0.4, "bend_radius": "2 mm"}));
    base(s, 120.0, 60.0);
}

/// The jog of a plate along x = 50 (the small side, x < 50, steps).
fn jog(s: &mut Session, extra: Value) -> Result<Value, crate::EngineError> {
    let mut q = json!({"points": [[50, 0, 2], [50, 60, 2]], "fixed": [100, 30, 2]});
    for (k, v) in extra.as_object().into_iter().flatten() {
        q[k] = v.clone();
    }
    s.execute("sheet.jog", &q)
}

fn z_range(s: &mut Session) -> (f64, f64) {
    let b = bbox(s);
    (b["min"][2].as_f64().unwrap_or(f64::NAN), b["max"][2].as_f64().unwrap_or(f64::NAN))
}

#[test]
fn jog_keeps_the_flat_size_and_makes_two_opposite_bends() {
    let ba = FRAC_PI_2 * (2.0 + 0.4 * 2.0);
    let mut s = Session::default();
    jog_plate(&mut s);
    let v0 = volume(&mut s);
    jog(&mut s, json!({"offset": 20})).unwrap();
    let f = flat(&mut s);
    assert!((f["flat_size_mm"][0].as_f64().unwrap_or(0.0) - 120.0).abs() < 1e-6, "{f}");
    assert!((f["flat_size_mm"][1].as_f64().unwrap_or(0.0) - 60.0).abs() < 1e-6, "{f}");
    let bends = f["bends"].as_array().cloned().unwrap_or_default();
    assert_eq!(bends.len(), 2, "{f}");
    let dirs: Vec<&str> = bends.iter().filter_map(|b| b["direction"].as_str()).collect();
    assert_eq!(dirs.iter().filter(|d| **d == "up").count(), 1, "{f}");
    assert_eq!(dirs.iter().filter(|d| **d == "down").count(), 1, "{f}");
    for b in &bends {
        assert!((b["angle_deg"].as_f64().unwrap_or(0.0) - 90.0).abs() < 1e-9, "{f}");
        assert!((b["length"].as_f64().unwrap_or(0.0) - 60.0).abs() < 1e-6, "{f}");
        assert!((b["allowance"].as_f64().unwrap_or(0.0) - ba).abs() < 1e-9, "{f}");
    }
    // One body; the bends' material is the sheet's own (two bends turned, none added).
    assert_eq!(s.model.state().bodies.len(), 1);
    let want = v0 - 2.0 * ba * 2.0 * 60.0 + 2.0 * FRAC_PI_2 * 2.0 * (2.0 + 1.0) * 60.0;
    assert!(rel(volume(&mut s), want) < 1e-4, "{} {want}", volume(&mut s));
}

#[test]
fn jog_step_height_is_the_offset_for_every_measure_and_angle() {
    // Outer faces (outside), inner faces (inside) or a face and the same face (faces).
    for angle in [90.0_f64, 45.0] {
        for (measure, offset, shift) in [("outside", 20.0, -2.0), ("inside", 16.0, 2.0), ("faces", 18.0, 0.0)] {
            let mut s = Session::default();
            jog_plate(&mut s);
            let q = json!({"offset": offset, "measure": measure, "angle": format!("{angle} deg")});
            jog(&mut s, q).unwrap_or_else(|e| panic!("{measure} {angle}: {e}"));
            // The plate's own lower face is z = 0; the stepped part's same face is H above it.
            let (lo, hi) = z_range(&mut s);
            let h = offset + shift;
            assert!(lo.abs() < 1e-6, "{measure} {angle}: {lo}");
            assert!((hi - (h + 2.0)).abs() < 1e-6, "{measure} {angle}: top {hi} want {}", h + 2.0);
        }
    }
    // The three measures describe the same step when their numbers differ by the thickness.
    let mut s = Session::default();
    jog_plate(&mut s);
    jog(&mut s, json!({"offset": 20, "measure": "outside"})).unwrap();
    let (_, outside) = z_range(&mut s);
    assert!((outside - 20.0).abs() < 1e-6, "outer faces exactly 20 apart: {outside}");
}

#[test]
fn jog_line_position_moves_the_step_by_half_the_span_and_the_span() {
    let ba = FRAC_PI_2 * (2.0 + 0.4 * 2.0);
    // At 90°, R2, T2, outside 20: H = 18 and the wall is 12 long.
    let span = 2.0 * ba + 12.0;
    // Where the two bends lie in the flat pattern: from the far end of the second to the line.
    let low_x = |position: &str| {
        let mut s = Session::default();
        jog_plate(&mut s);
        jog(&mut s, json!({"offset": 20, "position": position})).unwrap();
        let f = flat(&mut s);
        let xs: Vec<f64> = f["bends"]
            .as_array()
            .into_iter()
            .flatten()
            .flat_map(|b| [b["start"][0].as_f64().unwrap_or(f64::NAN), b["end"][0].as_f64().unwrap_or(f64::NAN)])
            .collect();
        xs.iter().cloned().fold(f64::MAX, f64::min)
    };
    // The bend table's lines are the bends' centre lines. Start: the zones run from the line
    // (x = 50) toward the side that steps, the span S long, the last one's centre BA / 2 short of it.
    let (start, middle, end) = (low_x("start"), low_x("middle"), low_x("end"));
    assert!((start - (50.0 - span + ba / 2.0)).abs() < 1e-6, "{start}");
    assert!((middle - start - span / 2.0).abs() < 1e-6, "{middle} {start} {span}");
    assert!((end - start - span).abs() < 1e-6, "{end} {start} {span}");
}

#[test]
fn jog_flip_steps_down_and_one_undo_removes_it() {
    let mut s = Session::default();
    jog_plate(&mut s);
    let before = s.doc.to_json();
    jog(&mut s, json!({"offset": 20, "flip": true})).unwrap();
    let (lo, hi) = z_range(&mut s);
    assert!((hi - 2.0).abs() < 1e-6 && (lo + 18.0).abs() < 1e-6, "{lo} {hi}");
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.doc.to_json(), before);
    assert!(flat(&mut s)["bends"].as_array().is_none_or(Vec::is_empty));
}

#[test]
fn jog_refuses_what_it_cannot_make_and_leaves_the_design_as_it_was() {
    let mut s = Session::default();
    jog_plate(&mut s);
    let before = s.doc.to_json();
    // At 90° the wall is H - 2R - T: an outside offset of 8 is H = 6 = 2R + T, no wall at all.
    let e = jog(&mut s, json!({"offset": 8})).unwrap_err().to_string();
    assert!(e.contains("more than 8.000 mm"), "{e}");
    let e = jog(&mut s, json!({"offset": 3, "measure": "inside"})).unwrap_err().to_string();
    assert!(e.contains("more than 4.000 mm"), "{e}");
    for bad in [
        json!({"offset": 20, "measure": "sideways"}),
        json!({"offset": 20, "position": "centerline"}),
        json!({"offset": "0 / 0 mm"}),
        json!({"offset": -5}),
        json!({"offset": 20, "angle": "0 deg"}),
        json!({"offset": 20, "angle": "180 deg"}),
        json!({"offset": 20, "radius": "-1 mm"}),
        json!({"measure": "outside"}),
        json!({"offset": 20, "points": [[50, 0, 40], [50, 60, 40]]}),
        json!({"offset": 20, "points": [[0.5, 0, 2], [0.5, 60, 2]], "fixed": [100, 30, 2]}),
    ] {
        assert!(jog(&mut s, bad.clone()).is_err(), "{bad}");
        assert_eq!(s.doc.to_json(), before, "{bad}");
    }
    // Lower than the smallest offset at 45°: shorter than the bends need.
    let e = jog(&mut s, json!({"offset": 3, "angle": "45 deg"})).unwrap_err().to_string();
    assert!(e.contains("more than 3.757 mm"), "{e}");
    assert_eq!(s.doc.to_json(), before);
}

#[test]
fn jog_on_an_unfolded_sheet_asks_to_refold_it() {
    let mut s = Session::default();
    jog_plate(&mut s);
    run(&mut s, "sheet.unfold", json!({}));
    let e = jog(&mut s, json!({"offset": 20})).unwrap_err().to_string();
    assert!(e.contains("refold"), "{e}");
}
