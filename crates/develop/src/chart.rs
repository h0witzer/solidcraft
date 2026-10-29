//! A panel's neutral chart: where the points of its skin chart lie on the neutral surface, the
//! surface the flat pattern is a development of. A planar panel's neutral surface is its own
//! shape; a roll's is longer or shorter round it, by how far the neutral surface lies from the
//! skin.

use std::f64::consts::FRAC_PI_2;

use solidcraft_geom::{Loop2, Region2, Seg2, Vec2};

use crate::{Skin, Surface};

/// A segment a map does not keep is followed by chords this close to it (mm).
const CHORD_TOL: f64 = 1e-4;

/// Nearer the apex than this (mm), a point is the apex; a line whose ends' cross product is this
/// small against their lengths runs through it.
const APEX_TOL: f64 = 1e-9;

/// From a panel's skin chart to its neutral chart.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Neutral {
    /// A planar panel: the neutral surface has its shape.
    Same,
    /// A cylinder: lengths round it (the chart's y) scale by the neutral radius over the skin's.
    /// A line stays a line, and a circular arc becomes an exact elliptical one.
    Stretch(f64),
    /// A cone: distances from the apex grow by this much, the neutral surface being the skin
    /// offset along the cone's normal. Rulings and arcs about the apex stay exact; anything else
    /// is followed by chords.
    Radial(f64),
}

impl Neutral {
    pub(crate) fn of(surface: &Surface, skin: &Skin) -> Result<Neutral, &'static str> {
        let (t, k) = (skin.thickness, skin.k_factor);
        match *surface {
            Surface::Plane => Ok(Neutral::Same),
            Surface::Cylinder { radius, material_inward } => {
                // The neutral surface lies K x T from the inside of the roll.
                let neutral = if material_inward { radius - (1.0 - k) * t } else { radius + k * t };
                if !(radius > 0.0 && neutral > 0.0 && neutral.is_finite()) {
                    return Err("a roll's radius must be more than the sheet's thickness");
                }
                Ok(Neutral::Stretch(neutral / radius))
            }
            Surface::Cone { half_angle, material_inward } => {
                if !(half_angle > 0.0 && half_angle < FRAC_PI_2) {
                    return Err("a cone's rulings make more than 0 and less than a quarter turn with its axis");
                }
                // Offset along the normal by h, a cone's apex moves along the axis and every
                // point's distance from it along its ruling changes by h / tan(half angle).
                let h = if material_inward { -(1.0 - k) * t } else { k * t };
                Ok(Neutral::Radial(h / half_angle.tan()))
            }
        }
    }

    pub(crate) fn point(&self, p: Vec2) -> Vec2 {
        match *self {
            Neutral::Same => p,
            Neutral::Stretch(s) => Vec2::new(p.x, p.y * s),
            Neutral::Radial(d) => {
                let r = p.len();
                if r > APEX_TOL { p * ((r + d) / r) } else { p }
            }
        }
    }

    /// The direction `v` at `p`, mapped (the map's derivative there), as a unit vector.
    pub(crate) fn dir(&self, p: Vec2, v: Vec2) -> Option<Vec2> {
        match *self {
            Neutral::Same => v.normalized(),
            Neutral::Stretch(s) => Vec2::new(v.x, v.y * s).normalized(),
            Neutral::Radial(d) => {
                let r = p.len();
                if !(r > APEX_TOL) {
                    return v.normalized();
                }
                // p (1 + d / r): along the ruling it is the identity, across it a scale.
                let u = p * (1.0 / r);
                let along = u * u.dot(v);
                (along + (v - along) * ((r + d) / r)).normalized()
            }
        }
    }

    /// The region mapped, and for every segment of each of its loops the index of the segment
    /// of the panel's own loop it came from: an arc stretched becomes several conics, and a
    /// curve the map does not keep becomes chords.
    pub(crate) fn region(&self, r: &Region2) -> Result<(Region2, Vec<Vec<usize>>), &'static str> {
        if let Neutral::Radial(d) = *self
            && d < 0.0
        {
            for lp in std::iter::once(&r.outer).chain(&r.holes) {
                if (&lp.polyline(CHORD_TOL)).into_iter().any(|p| !(p.len() + d > 0.0)) {
                    return Err("the cone's neutral surface would pass its apex inside the panel");
                }
            }
        }
        let mut origins = Vec::new();
        let mut map_loop = |lp: &Loop2| {
            let (mut segs, mut from) = (Vec::new(), Vec::new());
            for (i, s) in lp.segs.iter().enumerate() {
                let before = segs.len();
                self.seg(s, &mut segs);
                from.extend(std::iter::repeat_n(i, segs.len() - before));
            }
            origins.push(from);
            Loop2 { segs }
        };
        let outer = map_loop(&r.outer);
        let holes = r.holes.iter().map(&mut map_loop).collect();
        Ok((Region2 { outer, holes }, origins))
    }

    fn seg(&self, s: &Seg2, out: &mut Vec<Seg2>) {
        match (*self, *s) {
            (Neutral::Same, _) => out.push(*s),
            (Neutral::Stretch(_), Seg2::Arc { center, radius, start, sweep }) => {
                out.extend(arc_as_conics(center, radius, start, sweep).iter().map(|c| self.controls(c)));
            }
            // An affine map keeps every Bezier and conic: its control points move.
            (Neutral::Stretch(_), _) => out.push(self.controls(s)),
            (Neutral::Radial(_), Seg2::Line { a, b }) if a.cross(b).abs() <= APEX_TOL * a.len() * b.len() && a.dot(b) > 0.0 => {
                out.push(Seg2::Line { a: self.point(a), b: self.point(b) });
            }
            (Neutral::Radial(d), Seg2::Arc { center, radius, start, sweep }) if center.len() <= APEX_TOL => {
                out.push(Seg2::Arc { center, radius: radius + d, start, sweep });
            }
            (Neutral::Radial(_), _) => {
                let pts: Vec<Vec2> = s.polyline(CHORD_TOL).into_iter().map(|p| self.point(p)).collect();
                out.extend(pts.windows(2).filter_map(|w| match w {
                    [a, b] => Some(Seg2::Line { a: *a, b: *b }),
                    _ => None,
                }));
            }
        }
    }

    /// A segment with its control points mapped: exact for an affine map.
    fn controls(&self, s: &Seg2) -> Seg2 {
        let m = |p: Vec2| self.point(p);
        match *s {
            Seg2::Line { a, b } => Seg2::Line { a: m(a), b: m(b) },
            Seg2::Arc { .. } => *s,
            Seg2::Cubic { p0, p1, p2, p3 } => Seg2::Cubic { p0: m(p0), p1: m(p1), p2: m(p2), p3: m(p3) },
            Seg2::Conic { a, apex, b, w } => Seg2::Conic { a: m(a), apex: m(apex), b: m(b), w },
            Seg2::Bezier { n, mut p } => {
                for q in p.iter_mut().take(usize::from(n) + 1) {
                    *q = m(*q);
                }
                Seg2::Bezier { n, p }
            }
        }
    }
}

/// A circular arc as rational quadratic pieces of at most a quarter turn each: exact, and kept
/// exact by any affine map.
fn arc_as_conics(center: Vec2, radius: f64, start: f64, sweep: f64) -> Vec<Seg2> {
    let n = (sweep.abs() / FRAC_PI_2).ceil().clamp(1.0, 8.0) as usize;
    let step = sweep / n as f64;
    (0..n)
        .map(|i| {
            let a0 = start + step * i as f64;
            let half = step / 2.0;
            Seg2::Conic {
                a: center + Vec2::from_angle(a0) * radius,
                apex: center + Vec2::from_angle(a0 + half) * (radius / half.cos()),
                b: center + Vec2::from_angle(a0 + step) * radius,
                w: half.cos(),
            }
        })
        .collect()
}
