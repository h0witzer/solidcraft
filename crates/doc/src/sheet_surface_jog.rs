//! Jog on a sheet made from surfaces (#6046, #6748): an offset step across a flat panel, made as
//! two folds. The first bends the panel along the jog line; the second bends the panel the first
//! made back the other way, a straight wall further on, so the far side ends up parallel to where
//! it was and `offset` from it. Both bends take their zones from the panels' own material, so the
//! flat keeps its size.

use solidcore_record::{Vec2, Vec3};
use solidcraft_develop::{JointKind, develop};

use crate::{DocError, Result, SurfaceSheet};

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(DocError::Invalid(msg.into()))
}

/// The step a jog makes: its `offset`, what it is measured between (`measure`: `outside`, outer
/// face to outer face; `inside`, inner face to inner face; `faces`, a face to the same face), the
/// `angle` and inside `radius` of both bends, where the jog line sits in the whole jog
/// (`position`: `start`, `middle` or `end` of the two bends and the wall, from the line toward
/// the side that moves), and `flip`, stepping toward the material rather than away from it.
#[derive(Clone, Copy, Debug)]
pub(crate) struct JogStep<'a> {
    pub offset: f64,
    pub measure: &'a str,
    pub angle: f64,
    pub radius: f64,
    pub position: &'a str,
    pub flip: bool,
}

impl JogStep<'_> {
    /// The face-to-same-face shift `H` the offset asks for, and how the offset was measured.
    fn shift(&self, t: f64) -> Result<(f64, f64, &'static str)> {
        let (to_faces, name) = match self.measure {
            "outside" => (-t, "outside to outside"),
            "inside" => (t, "inside to inside"),
            "faces" => (0.0, "face to the same face"),
            o => return err(format!("unknown jog measure `{o}` (outside, inside or faces)")),
        };
        Ok((self.offset + to_faces, to_faces, name))
    }
}

/// The smallest face-to-same-face shift of a jog with bends of `angle` and inside `radius` in a
/// sheet `t` thick: the one whose straight wall has no length.
fn min_shift(angle: f64, radius: f64, t: f64) -> f64 {
    (2.0 * radius + t) * (1.0 - angle.cos())
}

impl SurfaceSheet {
    /// A jog along the line from `a` to `b` (the body's coordinates) on a face of one of the
    /// sheet's flat panels: the side that moves (the side away from `fixed`, or else the
    /// smaller) bends by the step's angle, runs straight for a wall
    /// `L = (H − (2R + T)(1 − cos θ)) / sin θ` and bends back, so it lies `H` from where it was,
    /// face to the same face. Two folds of [`Self::add_fold`]. Returns the second bend's joint;
    /// an error leaves the sheet as it was.
    pub(crate) fn add_jog(&mut self, a: Vec3, b: Vec3, step: JogStep<'_>, fixed: Option<Vec3>) -> Result<usize> {
        let mut work = self.clone();
        let j = work.jog(a, b, step, fixed)?;
        *self = work;
        Ok(j)
    }

    fn jog(&mut self, a: Vec3, b: Vec3, step: JogStep<'_>, fixed: Option<Vec3>) -> Result<usize> {
        let JogStep { offset, angle, radius, position, flip, .. } = step;
        if !(angle > 1e-6 && angle <= std::f64::consts::PI - 1e-6) {
            return err("the fold angle must be between 0 and 180°");
        }
        if !(radius >= 0.0) {
            return err("the bend radius must not be negative");
        }
        if !offset.is_finite() || offset < 0.0 {
            return err("the jog offset must be a number that is not negative");
        }
        let t = self.sheet.thickness;
        let (h, to_faces, name) = step.shift(t)?;
        let min_h = min_shift(angle, radius, t);
        let wall = (h - min_h) / angle.sin();
        if !(wall > 1e-3) {
            return err(format!(
                "the offset is too small for two bends of radius {radius:.3} mm at {:.1}°: measured {name} it must be more than {:.3} mm (it is {offset:.3} mm)",
                angle.to_degrees(),
                min_h - to_faces
            ));
        }
        let ba = angle * (radius + self.sheet.k_factor * t);
        let span = 2.0 * ba + wall;
        let back = match position {
            "start" => 0.0,
            "middle" | "center" | "centre" => span / 2.0,
            "end" => span,
            o => return err(format!("unknown jog position `{o}` (start, middle or end)")),
        };
        // Which way the side that moves lies from the line, as Fold chooses it: a fold of no
        // radius and next to no angle, its zone next to nothing, so it fits wherever the jog does.
        let (panel, n) = {
            let mut probe = self.clone();
            let j = probe.add_fold(a, b, 1e-3, 0.0, "start", flip, fixed)?;
            let joint = probe.sheet.joints.get(j).ok_or_else(|| DocError::Invalid("jog".into()))?;
            let n = (joint.b.origin - joint.a.origin).normalized().ok_or_else(|| DocError::Invalid("jog".into()))?;
            (joint.a.panel, n)
        };
        let place = |s: &SurfaceSheet, p: usize| -> Result<solidcraft_develop::Rigid3> {
            let dev = develop(&s.sheet, 0).map_err(|e| DocError::Invalid(e.to_string()))?;
            let folded = crate::sheet_surface_convert::fold_onto(&s.sheet, &dev, &s.places)?;
            folded.places.get(p).copied().flatten().ok_or_else(|| DocError::Invalid("a panel of the jog has no place".into()))
        };
        let on = |v: Vec2| Vec3::new(v.x, v.y, 0.0);
        let nw = place(self, panel)?.vector(on(n));
        // The first bend: its zone starts `back` short of the line, toward the side that stays. A
        // point on that side keeps it, wherever the line moved to.
        let (a1, b1) = (a - nw * back, b - nw * back);
        let stays = (a1 + b1) * 0.5 - nw;
        let joints = self.sheet.joints.len();
        let first = self.add_fold(a1, b1, angle, radius, "start", flip, Some(stays))?;
        if self.sheet.joints.len() != joints + 1 {
            return err("not supported yet: a jog whose first bend splits the sheet into separate tabs");
        }
        // The second bends the wall the first made back the other way, `wall` past the first's
        // zone, on the wall's own chart (the panel's, cut), with the wall's side staying.
        let (wall_panel, from, along, length) = match self.sheet.joints.get(first) {
            Some(j) => match &j.kind {
                JointKind::Bend(bend) => (j.b.panel, j.b.origin, j.b.dir, bend.length),
                _ => return err("jog"),
            },
            None => return err("jog"),
        };
        let at = place(self, wall_panel)?;
        let start = from + n * wall;
        let (a2, b2) = (at.point(on(start)), at.point(on(start + along * length)));
        let keep = at.point(on(from + n * (wall / 2.0)));
        let joints = self.sheet.joints.len();
        let second = self.add_fold(a2, b2, angle, radius, "start", !flip, Some(keep))?;
        if self.sheet.joints.len() != joints + 1 {
            return err("not supported yet: a jog whose second bend splits the sheet into separate tabs");
        }
        Ok(second)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_smallest_shift_leaves_no_wall() {
        // At 90° the two bends alone step 2R + T; at 45° (2R + T)(1 − cos 45°).
        assert!((min_shift(std::f64::consts::FRAC_PI_2, 2.0, 2.0) - 6.0).abs() < 1e-12);
        let at45 = min_shift(std::f64::consts::FRAC_PI_4, 2.0, 2.0);
        assert!((at45 - 6.0 * (1.0 - 0.5_f64.sqrt())).abs() < 1e-12);
    }

    #[test]
    fn each_measure_is_the_shift_by_a_thickness() {
        let step = |measure| JogStep { offset: 20.0, measure, angle: 1.0, radius: 2.0, position: "start", flip: false };
        assert_eq!(step("outside").shift(2.0).ok().map(|s| s.0), Some(18.0));
        assert_eq!(step("inside").shift(2.0).ok().map(|s| s.0), Some(22.0));
        assert_eq!(step("faces").shift(2.0).ok().map(|s| s.0), Some(20.0));
        assert!(step("sideways").shift(2.0).is_err());
    }
}
