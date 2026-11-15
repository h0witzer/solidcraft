//! The layout of a foldable print, taken from the sheet it prints (#6748): the faces of the sheet
//! (`SurfaceSheet`) are the definition, and the print's flat is derived from them as the
//! development lays them out. Nothing here is a second unfolding: which panels hang on which, which
//! joints are bends and by how much they turn, and where each panel lies in the flat all come from
//! the sheet's joints and the development of it (`solidcraft_develop::develop`), and the flat is
//! never written back.
//!
//! What a print changes is how a bend is made. Sheet metal bends round a radius, and the
//! development lays a bend zone's allowance between its two panels. A print hinges on a film at the
//! outer skin instead: the outer faces meet at the bend's virtual sharp, where the development
//! anchors it (`Datum::Sharp`). So each panel keeps the flat the development gave it, and a panel
//! that hangs on another is moved, with everything that hangs on it, until its sharp line lies on
//! its parent's: the zone's width is taken out, and nothing else changes ([`PrintPart`]).

use solidcore_record::{Region2, Seg2, Vec2};
use solidcraft_develop::prelude::*;
use solidcraft_develop::{Development, JointKind, Rigid2};
use solidcraft_geom::prelude::*;

use crate::sheet_surface_convert::SurfaceSheet;
use crate::{DocError, Result};

/// How near (mm) a point lies to a line and is on it.
const ON_LINE: f64 = 1e-6;

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(DocError::Invalid(msg.into()))
}

/// A panel of a part of the print: the flat the development gave it, moved with its parents so that
/// its sharp lines meet theirs (see the module docs).
#[derive(Clone, Debug)]
pub(crate) struct PrintPanel {
    /// The panel in the sheet.
    pub panel: usize,
    /// Its material in the part's flat: its face out to its sharps, less the cuts its rips make.
    pub region: Region2,
    /// The bend it hangs on its parent by (none for the part's first panel).
    pub bend: Option<usize>,
    /// The panel it hangs on (in the part, none for its first).
    pub parent: Option<usize>,
    /// How far the panel moved from where the development has it.
    pub shift: Vec2,
    /// Where the development has the panel (its chart in the flat, before it moved).
    pub place: Rigid2,
}

/// A bend of a part: the line where its two panels' outer faces meet, `s` a point on it and `d` its
/// unit direction, `n` the unit direction across it into the panel that hangs on the other.
#[derive(Clone, Debug)]
pub(crate) struct PrintBend {
    /// The bend's joint in the sheet.
    pub joint: usize,
    /// The panel it hangs off and the one that hangs on it (in the part).
    pub parent: usize,
    pub child: usize,
    pub s: Vec2,
    pub d: Vec2,
    pub n: Vec2,
    /// How far the sheet turns (radians, more than 0 and at most a half turn), toward the material.
    pub angle: f64,
    /// The stretches of the line, as distances along `d` from `s`, where both panels have an edge
    /// on it: where the print bends.
    pub spans: Vec<(f64, f64)>,
}

/// An edge of a panel that a rip made: the open edge between two panels of the sheet.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PrintEdge {
    /// The rip's joint in the sheet.
    pub joint: usize,
    /// The panel (in the part), and the edge from `a` to `b` in the part's flat; `n` is the unit
    /// direction from the edge into the panel.
    pub panel: usize,
    pub a: Vec2,
    pub b: Vec2,
    pub n: Vec2,
}

/// A part of the print: panels joined by bends into one piece of the flat, laid out from its first
/// panel. A sheet whose panels the bends do not all join prints as several parts.
#[derive(Clone, Debug)]
pub(crate) struct PrintPart {
    pub panels: Vec<PrintPanel>,
    pub bends: Vec<PrintBend>,
    pub edges: Vec<PrintEdge>,
}

/// The parts of a sheet's print and the development they are taken from.
pub(crate) struct PrintLayout {
    pub parts: Vec<PrintPart>,
}

/// A region moved by `by`.
fn moved(r: &Region2, by: Vec2) -> Region2 {
    Rigid2 { cos: 1.0, sin: 0.0, shift: by }.region(r)
}

/// The stretches (from `s` along the unit `d`) of `r`'s outline that run along the line through
/// `s` and `d`.
fn on_line(r: &Region2, s: Vec2, d: Vec2, tol: f64) -> Vec<(f64, f64)> {
    let across = |p: Vec2| (p - s).cross(d).abs();
    let mut out = Vec::new();
    for l in std::iter::once(&r.outer).chain(&r.holes) {
        for seg in &l.segs {
            if let Seg2::Line { a, b } = *seg
                && across(a) <= tol
                && across(b) <= tol
            {
                let (u, v) = ((a - s).dot(d), (b - s).dot(d));
                out.push((u.min(v), u.max(v)));
            }
        }
    }
    out
}

/// The stretches both `a` and `b` have: each stretch of the one cut by the other's, longer than `tol`.
fn shared(a: &[(f64, f64)], b: &[(f64, f64)], tol: f64) -> Vec<(f64, f64)> {
    let mut out: Vec<(f64, f64)> = Vec::new();
    for &(a0, a1) in a {
        for &(b0, b1) in b {
            let (lo, hi) = (a0.max(b0), a1.min(b1));
            if hi - lo > tol {
                out.push((lo, hi));
            }
        }
    }
    out.sort_by(|x, y| x.0.total_cmp(&y.0));
    // Touching stretches are one.
    let mut joined: Vec<(f64, f64)> = Vec::new();
    for (lo, hi) in out {
        match joined.last_mut() {
            Some(last) if lo <= last.1 + tol => last.1 = last.1.max(hi),
            _ => joined.push((lo, hi)),
        }
    }
    joined
}

impl SurfaceSheet {
    /// The sheet's print laid out: each piece of its development's flat as a part, its panels hung
    /// on each other by the bends (which must turn toward the material, as the faces of a solid do
    /// round its outside) with their sharp lines meeting. See the module docs.
    pub(crate) fn print_layout(&self, dev: &Development) -> Result<PrintLayout> {
        let sheet = &self.sheet;
        let reach = dev.panels.iter().flatten().map(|p| crate::sheet_surface_cuts::loop_box(&p.region.outer).size().len()).fold(1.0, f64::max);
        let tol = (reach * 1e-9).max(ON_LINE);
        let name = |i: usize| sheet.panels.get(i).map_or_else(|| format!("panel {i}"), |p| p.name.clone());
        for joint in &sheet.joints {
            match &joint.kind {
                JointKind::Bend(b) if !b.toward_material => {
                    return err(format!(
                        "not supported yet: a foldable print whose bend between {} and {} turns away from the material; pick the fixed face on the outside of the body",
                        name(joint.a.panel),
                        name(joint.b.panel)
                    ));
                }
                JointKind::Bend(b) if !(b.angle > 1e-6 && b.angle < std::f64::consts::PI - 1e-6) => {
                    return err(format!(
                        "the faces at the bend between {} and {} must meet at an angle between 0 and 180°",
                        name(joint.a.panel),
                        name(joint.b.panel)
                    ));
                }
                JointKind::Seam => {
                    return err(format!(
                        "a foldable print takes flat faces: {} and {} meet smoothly (a roll)",
                        name(joint.a.panel),
                        name(joint.b.panel)
                    ));
                }
                _ => {}
            }
        }
        let mut parts = Vec::with_capacity(dev.pieces.len());
        for piece in &dev.pieces {
            let mut piece: Vec<usize> = piece.iter().copied().filter(|p| dev.panels.get(*p).is_some_and(Option::is_some)).collect();
            piece.sort_unstable();
            let Some(&root) = piece.first() else { continue };
            parts.push(self.print_part(dev, root, tol)?);
        }
        if parts.is_empty() {
            return err("the sheet has no panels to print");
        }
        Ok(PrintLayout { parts })
    }

    /// The part of the print that hangs on panel `root`: the panels the bends reach from it, each
    /// moved by what its bends take out of the development's flat.
    fn print_part(&self, dev: &Development, root: usize, tol: f64) -> Result<PrintPart> {
        let sheet = &self.sheet;
        let name = |i: usize| sheet.panels.get(i).map_or_else(|| format!("panel {i}"), |p| p.name.clone());
        let flat =
            |i: usize| dev.panels.get(i).and_then(Option::as_ref).ok_or_else(|| DocError::Invalid(format!("{} has no place in the flat", name(i))));
        let place =
            |i: usize| dev.placements.get(i).copied().flatten().ok_or_else(|| DocError::Invalid(format!("{} has no place in the flat", name(i))));
        let mut panels = vec![PrintPanel {
            panel: root,
            region: crate::sheet_surface::on_grid(&flat(root)?.region),
            bend: None,
            parent: None,
            shift: Vec2::ZERO,
            place: place(root)?,
        }];
        let mut bends: Vec<PrintBend> = Vec::new();
        let mut at = 0;
        while at < panels.len() {
            let (me, shift_me) = match panels.get(at) {
                Some(p) => (p.panel, p.shift),
                None => break,
            };
            for (j, joint) in sheet.joints.iter().enumerate() {
                let JointKind::Bend(bend) = &joint.kind else { continue };
                let (mine, theirs, i_am_a) = if joint.a.panel == me {
                    (&joint.a, &joint.b, true)
                } else if joint.b.panel == me {
                    (&joint.b, &joint.a, false)
                } else {
                    continue;
                };
                let child = theirs.panel;
                if bends.iter().any(|b| b.joint == j) || panels.iter().any(|p| p.panel == child) {
                    continue;
                }
                // The line the bend is on the parent, and on the child where the development has it.
                let (pm, pc) = (place(me)?, place(child)?);
                let s = pm.point(mine.origin) + shift_me;
                let d = pm.vector(mine.dir).normalized().ok_or_else(|| DocError::Invalid("a bend has no direction".into()))?;
                let (o_c, d_c) = (pc.point(theirs.origin), pc.vector(theirs.dir));
                if d_c.normalized().is_none_or(|u| u.dot(d) < 1.0 - 1e-6) {
                    return err(format!("the bend between {} and {} reads two ways in the flat", name(me), name(child)));
                }
                // Moved onto its parent's line: the zone's width taken out, nothing else.
                let shift = s - o_c;
                // The child lies on the right of the line where it is the joint's `b` panel, the
                // `a` panel's material on its left.
                let n = if i_am_a { Vec2::new(d.y, -d.x) } else { Vec2::new(-d.y, d.x) };
                let child_region = crate::sheet_surface::on_grid(&moved(&flat(child)?.region, shift));
                let own = panels.get(at).map(|p| p.region.clone()).unwrap_or_default();
                let spans = shared(&on_line(&own, s, d, tol), &on_line(&child_region, s, d, tol), tol.max(1e-6));
                bends.push(PrintBend { joint: j, parent: at, child: panels.len(), s, d, n, angle: bend.angle, spans });
                panels.push(PrintPanel { panel: child, region: child_region, bend: Some(bends.len() - 1), parent: Some(at), shift, place: pc });
            }
            at += 1;
        }
        // The open edges the rips left: each rip's edge on its two panels as the faces draw it, in
        // the part's flat (the development cuts the panels back from it as the rip's lap says).
        // The material of the `a` panel lies on the left of the edge as it runs, the `b` panel's on
        // the right.
        let mut edges = Vec::new();
        for (j, joint) in sheet.joints.iter().enumerate() {
            let JointKind::Rip(rip) = &joint.kind else { continue };
            for (side, owner, pts) in [(1.0, joint.a.panel, &rip.edges.0), (-1.0, joint.b.panel, &rip.edges.1)] {
                let Some(k) = panels.iter().position(|p| p.panel == owner) else { continue };
                let (Some(panel), [p0, p1]) = (panels.get(k), pts.as_slice()) else { continue };
                let (a, b) = (panel.place.point(*p0) + panel.shift, panel.place.point(*p1) + panel.shift);
                let Some(u) = (b - a).normalized() else { continue };
                edges.push(PrintEdge { joint: j, panel: k, a, b, n: Vec2::new(-u.y, u.x) * side });
            }
        }
        Ok(PrintPart { panels, bends, edges })
    }
}
