//! Where the stretches of a split edge meet (#6748, `sheet.flange`'s `segments`), and the relief a
//! segment asks for there (`relief`, FR-13). The sheet's surfaces are the definition: a junction
//! is a pair of its bends (see [`SurfaceSheet::junctions`]), and what is cut at it is derived
//! from them.
//!
//! - Two stretches that bend to opposite sides are parted along the junction by a cut with no
//!   width, from the deeper of the two bends' tangent lines out to the nearer far edge
//!   ([`junction_slits`]); stretches that bend the same way stay joined.
//! - A relief that takes material out (`straight`, `rectangular`, `obround`) is a cut of the
//!   sheet's ([`Cut`]) on the panel the two bends stand on: `width` across the junction, centred
//!   on it, from `depth` before the deeper tangent line out to the nearer far edge. The cut
//!   crosses the parent panel, both bends' zones and both stretches' panels, and the flat, the
//!   folded body and the DXF all follow from it alike ([`SurfaceSheet::relieve_junction`]).
//! - A `tear` cuts nothing out: it is the junction's own slit, started `depth` deeper into the
//!   parent panel and made whichever sides the two bends turn to
//!   ([`SurfaceSheet::junction_tears`]).
//!
//! A relief that cannot be cut refuses, naming the segment the caller wrote; nothing is clamped.

use solidcore_record::{Loop2, Region2, Seg2, Vec2};
use solidcraft_develop::prelude::*;
use solidcraft_develop::{Anchor, Cut, Development, Joint, JointKind, Sheet};
use solidcraft_geom::prelude::*;

use crate::sheet::{FlatWork, JunctionRelief, JunctionShape};
use crate::sheet_surface::Piece;
use crate::{DocError, Result, SurfaceSheet};

/// How wide a strip is when a slit is tried on the flat: a slit has no width, so it is cut as a
/// strip this wide (mm), which reaches a hair past its far end, and the pieces left are counted
/// (see [`SurfaceSheet::flat_pieces`]).
const SLIT_STRIP: f64 = 1e-3;

/// A junction of a split edge as the flat lays it out: the line the two bends' zones end on.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Junction {
    /// The panel the two bends stand on.
    pub parent: usize,
    /// The point of the junction line on the deeper of the two bends' tangent lines.
    pub origin: Vec2,
    /// Unit along the junction line, from the parent panel toward the stretches' flanges.
    pub n: Vec2,
    /// How far out from `origin` the nearer of the two flanges' far edges lies.
    pub reach: f64,
    /// Whether the two bends turn to opposite sides of the sheet's material.
    pub opposite: bool,
    /// The lengths of the bend that ends at the junction and of the one that starts there.
    pub lengths: [f64; 2],
}

/// The junction of the pair of bends `[ends, starts]` (see [`SurfaceSheet::junctions`]) in the
/// flat `dev` of `sheet`: along the line the two zones' ends share, from the deeper of the bends'
/// tangent lines on the panel they stand on out to the nearer of their flanges' far edges (each
/// flange's farthest corner on that line). Past that only the longer flange's material lies on
/// the line: nothing of the other is left to part.
pub(crate) fn junction_of(sheet: &Sheet, dev: &Development, [ends, starts]: [usize; 2]) -> Option<Junction> {
    let bend = |j: usize| match sheet.joints.get(j) {
        Some(Joint { a, b, kind: JointKind::Bend(bend) }) => Some((a.panel, *b, bend.toward_material, bend.length)),
        _ => None,
    };
    let zone = |j: usize| dev.zones.iter().find(|z| z.joint == j);
    let (Some((parent, bx, toward_x, length)), Some((_, by, toward_y, other)), Some(zx), Some(zy)) =
        (bend(ends), bend(starts), zone(ends), zone(starts))
    else {
        return None;
    };
    // The one bend ends at the junction and the other starts there: the line across from the
    // panel they stand on toward their flanges.
    let (near_x, near_y) = (zx.near[1], zy.near[0]);
    let n = (zx.far[1] - near_x).normalized()?;
    // How far out along that line a flange reaches: its panel's farthest corner on it.
    let reach = |an: &Anchor, at: f64| -> Option<f64> {
        let place = dev.placements.get(an.panel).copied().flatten()?;
        let corner = an.origin + an.dir * at;
        let region = &sheet.panels.get(an.panel)?.region;
        region.outer.segs.iter().map(Seg2::start).filter(|v| (*v - corner).dot(an.dir).abs() <= 1e-9).map(|v| place.point(v).dot(n)).reduce(f64::max)
    };
    let (far_x, far_y) = (reach(&bx, length)?, reach(&by, 0.0)?);
    let (start, end) = (near_x.dot(n).min(near_y.dot(n)), far_x.min(far_y));
    let along = near_x.dot(n);
    Some(Junction { parent, origin: near_x + n * (start - along), n, reach: end - start, opposite: toward_x != toward_y, lengths: [length, other] })
}

/// Where the flat `dev` of `sheet` is cut along the `junctions` of split edges (see
/// [`SurfaceSheet::slits`]): at each junction whose two bends turn to opposite sides, and at each
/// of the `tears` whichever sides they turn to, from the deeper of the bends' tangent lines (a
/// tear's, as far before it as it asks) out to the nearer of their flanges' far edges.
pub(crate) fn junction_slits(sheet: &Sheet, dev: &Development, junctions: &[[usize; 2]], tears: &[([usize; 2], f64)]) -> Vec<[Vec2; 2]> {
    let mut out = Vec::new();
    for &pair in junctions {
        let tear = tears.iter().find(|(p, _)| *p == pair).map(|(_, depth)| *depth);
        let Some(j) = junction_of(sheet, dev, pair) else { continue };
        if !j.opposite && tear.is_none() {
            continue;
        }
        if j.reach > 1e-9 {
            let back = tear.unwrap_or(0.0);
            out.push([j.origin - j.n * back, j.origin + j.n * j.reach]);
        }
    }
    out
}

impl SurfaceSheet {
    /// Relieves the junction `joints` (one of [`SurfaceSheet::junctions`]'s pairs: the bend that
    /// ends at it, then the one that starts there) as `relief` says. `named` is how the caller
    /// wrote the relief (`edges[0].segments[1].relief`), and `earlier` the number of the segment
    /// before the junction, for the refusals that name one.
    ///
    /// A notch (`straight`, `rectangular`, `obround`) is cut across the junction from the panel
    /// the two bends stand on: `width` across, centred on the junction, from `depth` before the
    /// deeper bend line out to the nearer far edge, from that panel and from both stretches. The
    /// two stretches are then parted there and joined only through the parent panel, so each
    /// bend ends in a free end at the notch's side. A `tear` is the junction's own slit, deeper.
    /// A notch (the default) is no relief. Refused, never clamped: a depth past the parent
    /// panel's material inboard of the junction, an obround shallower than half its width, a
    /// notch wider than the narrower stretch.
    pub fn relieve_junction(&mut self, joints: [usize; 2], relief: &JunctionRelief, named: &str, earlier: usize) -> Result<()> {
        if relief.shape == JunctionShape::Notch {
            return Ok(());
        }
        let refuse = |why: String| -> DocError { DocError::Invalid(format!("{named}: {why}")) };
        let dev = self.develop().map_err(|e| refuse(e.to_string()))?;
        let Some(at) = junction_of(&self.sheet, &dev, joints) else {
            return Err(refuse("no stretch starts where this one ends, so there is no junction to relieve".into()));
        };
        if !(at.reach > 1e-9) {
            return Err(refuse("the two bends leave no material at the junction to relieve".into()));
        }
        let (width, depth) = (relief.width, relief.depth);
        // The deep end lies `depth` inboard of the deeper bend line, so a deeper relief moves it
        // further in, away from the far edge the outer end stops at. What it has to fit is the
        // material the junction's bends leave inboard of that line: the parent panel. Past that
        // edge the notch would run into a bend zone, which is no panel, so it refuses instead of
        // cutting short.
        let mut work = FlatWork::new();
        let pieces = crate::sheet_surface::pieces(&dev, &mut work).map_err(|e| refuse(e.to_string()))?;
        let inboard = |q: Vec2| (q - at.origin).dot(at.n);
        let holds_line = |r: &Region2| crate::sheet_surface_cuts::holds(&r.outer, at.origin - at.n * 1e-4);
        let of_parent = |(piece, _): &&(Piece, Region2)| *piece == Piece::Panel(at.parent);
        let mine: Vec<&(Piece, Region2)> = pieces.iter().filter(of_parent).collect();
        let panel = mine.iter().find(|(_, r)| holds_line(r)).or_else(|| mine.first());
        let Some((_, panel)) = panel else {
            return Err(refuse("the panel the junction's bends stand on has no material".into()));
        };
        let room = -panel.outer.polyline(1e-3).iter().map(|q| inboard(*q)).fold(f64::MAX, f64::min);
        if depth > room + 1e-9 {
            return Err(refuse(format!("a depth of {depth:.3} mm is deeper than the material inboard of the junction ({room:.3} mm)")));
        }
        if relief.shape == JunctionShape::Obround && depth < width / 2.0 - 1e-9 {
            return Err(refuse(format!("an obround relief needs a depth of at least half its width ({:.3} mm)", width / 2.0)));
        }
        if relief.shape.cuts_material() {
            // Half the width comes out of each stretch's own length, so it must fit the narrower
            // one: the earlier stretch's bend is the one that starts at the junction.
            let (narrow, which) = if at.lengths[1] <= at.lengths[0] { (at.lengths[1], earlier) } else { (at.lengths[0], earlier + 1) };
            if width / 2.0 > narrow + 1e-9 {
                return Err(DocError::Invalid(format!(
                    "{named} is {width:.3} mm wide, wider than segment {} ({narrow:.3} mm); make the segment wider or the relief narrower",
                    which + 1
                )));
            }
            let flat = notch(&at, width, depth, relief.shape);
            let place = dev.placements.get(at.parent).copied().flatten().ok_or_else(|| refuse("the parent panel has no place in the flat".into()))?;
            self.sheet.cuts.push(Cut { panel: at.parent, region: place.inverse().region(&flat) });
        } else if relief.shape == JunctionShape::Tear {
            self.junction_tears.retain(|(p, _)| *p != joints);
            self.junction_tears.push((joints, depth));
        }
        Ok(())
    }

    /// How many pieces the sheet's flat is in once every junction's slit is cut: the slits have
    /// no width and are no part of the flat's regions, so each is cut here as a strip
    /// [`SLIT_STRIP`] wide on a copy, and the pieces counted. Comparing this before and after a
    /// feature shows that its junction cuts parted the blank, a tear whose slit meets another
    /// junction's cut from the other side included.
    pub fn flat_pieces(&self) -> Result<usize> {
        let mut probe = self.clone();
        let dev = probe.develop().map_err(|e| DocError::Invalid(e.to_string()))?;
        probe.settle(&dev)?;
        let dev = probe.develop().map_err(|e| DocError::Invalid(e.to_string()))?;
        let mut work = FlatWork::new();
        let pieces = crate::sheet_surface::pieces(&dev, &mut work)?;
        let regions = crate::sheet_surface::join_pieces(&pieces, &mut work)?;
        probe.laid_out(&dev, &pieces, regions)?;
        let strips: Vec<Region2> = probe
            .slits
            .iter()
            .filter_map(|&[a, b]| {
                let unit = (b - a).normalized()?;
                let side = Vec2::new(-unit.y, unit.x) * (SLIT_STRIP / 2.0);
                let tip = b + unit * SLIT_STRIP;
                Some(Region2 { outer: Loop2::polygon(&[a + side, tip + side, tip - side, a - side]).ccw(), holes: Vec::new() })
            })
            .collect();
        if strips.is_empty() {
            return Ok(probe.regions.len());
        }
        Ok(crate::sheet_surface::boolean_past_holes(&probe.regions, &strips, solidcraft_kernel::BoolOp::Cut, &mut work)?.len())
    }
}

/// The notch of a junction relief, in the flat: `width` across the junction line, centred on it,
/// from `depth` before its origin (the deeper bend line) out to the nearer far edge. An obround
/// closes its inner end in a half round of half the width.
fn notch(at: &Junction, width: f64, depth: f64, shape: JunctionShape) -> Region2 {
    let m = Vec2::new(-at.n.y, at.n.x);
    let p = |across: f64, along: f64| at.origin + m * across + at.n * along;
    let (u, v0, v1) = (width / 2.0, -depth, at.reach);
    let outer = if shape == JunctionShape::Obround {
        // The sides and the outer end, closed on the inner end by a half round about its middle.
        let mid = v0 + u;
        Loop2 {
            segs: vec![
                Seg2::Line { a: p(-u, v1), b: p(u, v1) },
                Seg2::Line { a: p(u, v1), b: p(u, mid) },
                Seg2::Arc { center: p(0.0, mid), radius: u, start: m.angle(), sweep: std::f64::consts::PI },
                Seg2::Line { a: p(-u, mid), b: p(-u, v1) },
            ],
        }
    } else {
        Loop2::polygon(&[p(-u, v0), p(u, v0), p(u, v1), p(-u, v1)])
    };
    Region2 { outer: outer.ccw(), holes: Vec::new() }
}
