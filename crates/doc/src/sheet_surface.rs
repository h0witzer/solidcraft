//! A sheet body as surfaces (#6748): its panels in charts of their own, joined by bends anchored at
//! their virtual sharps, and laid out flat by `solidcraft-develop`. While the sheet model keeps
//! the flat pattern as its master, this is a check that its surfaces give the same flat: setting
//! `SOLIDCRAFT_SURFACE_REPORT` to a file makes every sheet body that is built append one line to it,
//! naming the thread (a test's name under `cargo test`) and how far the two flats differ.

use solidcore_record::{Loop2, Region2, Seg2, Vec2};
use solidcraft_develop::{Anchor, Bend, Datum, Development, Joint, JointKind, Panel, Relief, Rigid2, Sheet, Tail, ZoneEnds, crossings, develop};
use solidcraft_geom::prelude::*;
use solidcraft_geom::{Aabb2, Ring};
use solidcraft_kernel::BoolOp;

use crate::sheet::{FlatWork, SheetBody, SheetFlange, material_boolean, material_union};
use crate::sheet_flat_lineage::{self, Resolution};
use crate::{DocError, Result};

/// Bends closer to a half turn than this (radians) have no usable sharp: they are anchored at
/// their tangent lines.
const HALF_TURN_TOL: f64 = 1e-9;

/// The most the two flats may differ, as the area between them over the length of the sheet's
/// outline: their mean distance apart (mm).
const SAME_FLAT: f64 = 1e-6;

/// A sheet body's surfaces, its base panel's chart being the flat frame. Panel `0` is the base
/// and panel `1 + i` flange `i`'s panel; each panel's material runs on from its tangent line to
/// the bend's anchor, as the surface model keeps it.
pub(crate) fn surfaces_of(sh: &SheetBody) -> Result<Sheet> {
    let n = sh.flanges.len();
    // Each panel's chart, from the flat frame: the base's is the flat frame itself, and a
    // flange's is put at the start of its bend's anchor so that nothing of its flat place is
    // carried over.
    let mut charts = vec![Rigid2::IDENTITY; n + 1];
    let mut regions = vec![sh.base.clone()];
    let mut joints = Vec::with_capacity(n);
    for (i, f) in sh.flanges.iter().enumerate() {
        let frame = BendFrame::of(sh, i, f);
        let chart = Rigid2::onto(frame.b_origin, frame.along, Vec2::ZERO, Vec2::X)
            .ok_or_else(|| DocError::Invalid(format!("flange {i} has no bend direction")))?;
        if let Some(c) = charts.get_mut(i + 1) {
            *c = chart;
        }
        regions.push(if f.leg > 1e-9 { sh.panel(f) } else { Region2::default() });
        joints.push(frame);
    }
    // Run each panel's material on to its anchors: the parent's along the zone, the flange's
    // along its whole edge on the bend.
    let mut work = FlatWork::new();
    for frame in &joints {
        let parent = frame.parent;
        for (panel, band) in [(parent, frame.parent_band()), (frame.panel, frame.flange_band())] {
            let Some(band) = band else { continue };
            let Some(region) = regions.get_mut(panel) else { continue };
            if region.outer.segs.is_empty() {
                continue;
            }
            if let [one] = material_union(vec![region.clone(), band], &mut work)?.as_slice() {
                *region = one.clone();
            }
        }
    }
    let panels = regions
        .iter()
        .zip(&charts)
        .enumerate()
        .map(|(i, (r, c))| Panel { name: if i == 0 { "base".into() } else { format!("flange {}", i - 1) }, region: c.region(r), ..Panel::default() })
        .collect();
    let joints = joints
        .iter()
        .map(|f| {
            let (Some(pa), Some(pb)) = (charts.get(f.parent), charts.get(f.panel)) else { return Err(DocError::Invalid("flange parent".into())) };
            Ok(Joint {
                a: Anchor { panel: f.parent, origin: pa.point(f.a_origin), dir: pa.vector(f.along) },
                b: Anchor { panel: f.panel, origin: pb.point(f.b_origin), dir: pb.vector(f.along) },
                kind: JointKind::Bend(f.bend.clone()),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(Sheet { thickness: sh.t, k_factor: sh.k, panels, joints, corners: Vec::new(), cuts: Vec::new() })
}

/// One flange's bend, in the sheet's flat frame: the anchors run `along` the bend with the
/// parent's material on their left, so lengths along them are the flange's own lengths along
/// `d`, turned round when `d` runs the other way.
struct BendFrame {
    parent: usize,
    panel: usize,
    along: Vec2,
    toward: Vec2,
    /// Where the parent's flat material ends (the bend's start line) and the anchors start.
    tangent: Vec2,
    a_origin: Vec2,
    b_origin: Vec2,
    setback: f64,
    width: f64,
    /// The flange panel's edge on the bend, in lengths along the anchors.
    edge: (f64, f64),
    bend: Bend,
}

impl BendFrame {
    fn of(sh: &SheetBody, i: usize, f: &SheetFlange) -> BendFrame {
        let angle = f.angle.abs();
        let datum = if angle < std::f64::consts::PI - HALF_TURN_TOL { Datum::Sharp } else { Datum::Tangent };
        let mut bend = Bend {
            angle,
            radius: f.radius,
            toward_material: f.angle >= 0.0,
            datum,
            length: f.len,
            ends: ZoneEnds { near: (0.0, 0.0), far: (0.0, 0.0) },
            tails: (Tail::Keep, Tail::Trim),
            relief: (Relief::None, Relief::None),
        };
        let setback = bend.setback(sh.t);
        let width = sh.allowance(f);
        // The zone along the bend from `p`: its start line's span and, at a torn end, its far
        // edge running on to the panel's end (as the sheet's `zone_ends`).
        let (u0, u1) = sh.zone_span(f);
        let (e0, e1) = (if f.tear.0 { -f.ext.0 } else { u0 }, if f.tear.1 { f.len + f.ext.1 } else { u1 });
        let edge = if f.shape.is_some() || f.exact.is_some() { (e0, e1) } else { (-f.ext.0, f.len + f.ext.1) };
        // The anchors keep the parent's material on their left, so they run along `d` when the
        // flange (`n`) lies on its right, and the other way round otherwise.
        let forward = Vec2::new(f.d.y, -f.d.x).dot(f.n) > 0.0;
        let flip: fn((f64, f64)) -> (f64, f64) = if forward { |u| u } else { |u| (-u.1, -u.0) };
        let along = if forward { f.d } else { -f.d };
        bend.ends = ZoneEnds { near: flip((u0, u1)), far: flip((e0, e1)) };
        BendFrame {
            parent: f.parent.map_or(0, |k| k + 1),
            panel: i + 1,
            along,
            toward: f.n,
            tangent: f.p,
            a_origin: f.p + f.n * setback,
            b_origin: f.p + f.n * (width - setback),
            setback,
            width,
            edge: flip(edge),
            bend,
        }
    }

    /// A band across the bend from the line `from` (lengths across, toward the flange) to `to`,
    /// over `span` along it.
    fn band(&self, span: (f64, f64), from: f64, to: f64) -> Option<Region2> {
        if !(span.1 > span.0 && (to - from).abs() > 1e-12) {
            return None;
        }
        let at = |u: f64, v: f64| self.tangent + self.along * u + self.toward * v;
        let pts = [at(span.0, from), at(span.1, from), at(span.1, to), at(span.0, to)];
        Some(Region2 { outer: Loop2::polygon(&pts).ccw(), holes: Vec::new() })
    }

    /// The parent's material from its tangent line on to the sharp, along the zone.
    fn parent_band(&self) -> Option<Region2> {
        self.band(self.bend.ends.near, 0.0, self.setback)
    }

    /// The flange's material from its tangent line, the zone's width from the parent's, back
    /// to its sharp, along its whole edge.
    fn flange_band(&self) -> Option<Region2> {
        self.band(self.edge, self.width - self.setback, self.width)
    }
}

/// The sheet's flat material before its cut-outs: its panels and bend zones.
fn sheet_flat(sh: &SheetBody, work: &mut FlatWork) -> Result<Vec<Region2>> {
    let mut pieces = vec![sh.base.clone()];
    for f in &sh.flanges {
        if f.leg > 1e-9 {
            pieces.push(sh.panel(f));
        }
        pieces.push(sh.zone(f));
    }
    material_union(pieces, work)
}

/// The surface model's flat: each placed panel less its trims, and the zones; with the development it
/// was joined from.
fn surface_flat(model: &Sheet, work: &mut FlatWork) -> Result<(Vec<Region2>, Development)> {
    let dev = develop(model, 0).map_err(|e| DocError::Invalid(e.to_string()))?;
    Ok((join(&dev, work)?, dev))
}

/// A piece of a development's flat: a panel's material less its trims, a bend's zone (by its
/// place in the development's zones), or a tab: a piece of a zone that the cuts leave joined to
/// one of its panels only, which rides with that panel, flat, where the rest of the zone bends.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Piece {
    Panel(usize),
    Zone(usize),
    /// A piece of zone `.0`, riding with panel `.1`.
    Tab(usize, usize),
}

/// How near (mm) a cut may come to a piece's loops and still be a hole of it as it is: as near
/// as the sew joins points.
const CLEAR: f64 = 1e-6;

/// How far (mm) a piece's edge may lie off a zone's tangent line and still run along it.
const ON_LINE: f64 = 1e-7;

/// How near (mm) an edge of a boolean's result must follow a curve of its inputs, at its ends and
/// along it, to be a piece of that curve: ten times what the booleans here are asked to hold.
const AS_DRAWN: f64 = 1e-6;

/// How far apart (mm) the boolean may read the ends of two edges of one loop that meet at one
/// corner of its result: it reads each edge to within the B-rep's own tolerance, which a long
/// chain of booleans grows past what the next boolean takes as closed (ten times its own).
const ONE_CORNER: f64 = 1e-4;

/// `op` of `a` with `b` by the kernel's planar boolean, with each curved edge of the result that
/// runs along a curve of `a` or `b` given back as that curve's own piece. The boolean reads its
/// result again from a B-rep, and fits arcs to free-form spans that lie near a circle (a
/// letter's flat curves) and splines to conics; here a Bézier stays a Bézier and a conic a
/// conic, between the same corners, and the edges of each loop meet exactly (see
/// [`close_up`]). An edge that follows no input curve is kept as read.
///
/// Until the kernel reads a span that merely lies near a circle as the span it is (#6748): it is
/// the kernel's reader that breaks the planar booleans' own contract, that the result's
/// boundaries are the operands' curves "never refitted" (`crates/kernel/src/prism.rs`).
/// `segment` in `crates/kernel/src/prism_curve.rs` proposes a circle from samples and takes any
/// span within `tol` of it as an arc. The fix belongs there, an arc only where the span's record
/// curve is a circle or an exact rational-quadratic one, which would keep the 3D prism cut's
/// curves too; that is a `crates/kernel` change, so the curves are given back here, a layer up.
pub(crate) fn boolean_as_drawn(a: &[Region2], b: &[Region2], op: BoolOp, tol: f64) -> solidcraft_kernel::Result<Vec<Region2>> {
    let out = solidcraft_kernel::planar_regions_boolean(a, b, op, tol)?;
    Ok(as_drawn(a.iter().chain(b), out))
}

/// `out`, each edge that runs along a curve of `inputs` that curve's piece (see
/// [`boolean_as_drawn`]).
fn as_drawn<'a>(inputs: impl Iterator<Item = &'a Region2>, out: Vec<Region2>) -> Vec<Region2> {
    let curves: Vec<(Seg2, Aabb2)> = inputs
        .flat_map(|r| std::iter::once(&r.outer).chain(&r.holes))
        .flat_map(|l| &l.segs)
        .filter(|s| !matches!(s, Seg2::Line { .. }))
        .map(|s| {
            let mut b = Aabb2::EMPTY;
            for p in s.polyline(1e-4) {
                b.add(p);
            }
            (*s, b.inflated(AS_DRAWN + 2e-4))
        })
        .collect();
    let redraw = |l: Loop2| {
        let mut segs: Vec<Seg2> = l.segs.into_iter().map(|s| piece_of(&curves, &s).unwrap_or(s)).collect();
        close_up(&mut segs);
        Loop2 { segs }
    };
    out.into_iter().map(|r| Region2 { outer: redraw(r.outer), holes: r.holes.into_iter().map(redraw).collect() }).collect()
}

/// A loop's edges made to meet exactly where they meet within [`ONE_CORNER`]: the edge ending at
/// a corner moved onto the start of the next, or, where it is an arc (whose ends its angles
/// place), the next edge's start moved onto its end. Two arcs both end where their circles meet
/// at the corner (see [`circles_meet`]), each by its own angle: moving one arc's end along its
/// circle onto the other's would leave the part of the step that lies across that circle. The
/// boolean reads each edge's ends to within the B-rep's tolerance, and an arc it fits ends near
/// its corner, not on it. Left so, a step of a hair stays between neighbours: a smooth run of
/// curves can't be followed across it, and the next boolean refuses a loop whose steps outgrow
/// its own tolerance.
fn close_up(segs: &mut [Seg2]) {
    let n = segs.len();
    for i in 0..n {
        let j = (i + 1) % n;
        let (Some(&a), Some(&b)) = (segs.get(i), segs.get(j)) else { continue };
        let (end, start) = (a.end(), b.start());
        if end == start || end.dist(start) > ONE_CORNER {
            continue;
        }
        match (a, b) {
            (Seg2::Arc { center: ca, radius: ra, start: sa, sweep: wa }, Seg2::Arc { center: cb, radius: rb, start: sb, sweep: wb }) => {
                // Two pieces of one circle (or circles that don't meet there) meet where the
                // first ends.
                let x = circles_meet((ca, ra), (cb, rb), end.lerp(start, 0.5)).unwrap_or(end);
                let (ea, sb1) = (near_angle((x - ca).angle(), sa + wa), near_angle((x - cb).angle(), sb));
                if let Some(s) = segs.get_mut(i) {
                    *s = Seg2::Arc { center: ca, radius: ra, start: sa, sweep: ea - sa };
                }
                if let Some(s) = segs.get_mut(j) {
                    *s = Seg2::Arc { center: cb, radius: rb, start: sb1, sweep: sb + wb - sb1 };
                }
            }
            (Seg2::Arc { .. }, _) => {
                if let Some(x) = segs.get_mut(j) {
                    *x = ending_at(b, end, b.end());
                }
            }
            _ => {
                if let Some(x) = segs.get_mut(i) {
                    *x = ending_at(a, a.start(), start);
                }
            }
        }
    }
}

/// Where the circle about `c0` of radius `r0` and the one about `c1` of radius `r1` meet nearest
/// `near`: where they cross, or where they touch (for two read a hair apart, the point between
/// them on the line through their centres). None where that is more than [`ONE_CORNER`] from
/// `near`, and for circles about one centre.
fn circles_meet((c0, r0): (Vec2, f64), (c1, r1): (Vec2, f64), near: Vec2) -> Option<Vec2> {
    let u = (c1 - c0).normalized()?;
    let l = (c1 - c0).len();
    // From `c0` along the line of centres to the chord through the crossings, and half the chord.
    let along = (r0 * r0 - r1 * r1 + l * l) / (2.0 * l);
    let half = (r0 * r0 - along * along).max(0.0).sqrt();
    let foot = c0 + u * along;
    [foot + u.perp() * half, foot - u.perp() * half]
        .into_iter()
        .filter(|p| p.dist(near) <= ONE_CORNER)
        .min_by(|p, q| p.dist(near).total_cmp(&q.dist(near)))
}

/// The angle `a`, turned by whole turns to lie within half a turn of `to`.
fn near_angle(a: f64, to: f64) -> f64 {
    let d = a - to;
    to + d - std::f64::consts::TAU * (d / std::f64::consts::TAU).round()
}

/// The piece of one of `curves` that edge `s` runs along, from `s`'s start to its end: within
/// [`AS_DRAWN`] of `s` at both ends and at three points between. None for an edge that closes on
/// itself, and where no curve fits.
fn piece_of(curves: &[(Seg2, Aabb2)], s: &Seg2) -> Option<Seg2> {
    let (p, q) = (s.start(), s.end());
    if p.dist(q) <= AS_DRAWN {
        return None;
    }
    let probes = [0.25, 0.5, 0.75].map(|u| s.point_at(u));
    for (g, bounds) in curves {
        if !(bounds.contains(p) && bounds.contains(q)) {
            continue;
        }
        let (t0, t1) = (g.closest_param(p), g.closest_param(q));
        if g.point_at(t0).dist(p) > AS_DRAWN || g.point_at(t1).dist(q) > AS_DRAWN || (t1 - t0).abs() < 1e-12 {
            continue;
        }
        let piece = if t0 < t1 { part(g, t0, t1) } else { part(g, t1, t0).reversed() };
        if probes.iter().all(|x| piece.point_at(piece.closest_param(*x)).dist(*x) <= AS_DRAWN) {
            return Some(ending_at(piece, p, q));
        }
    }
    None
}

/// The part of `g` from parameter `t0` to `t1` (`t0` < `t1`), the same kind of curve.
pub(crate) fn part(g: &Seg2, t0: f64, t1: f64) -> Seg2 {
    let right = if t0 > 1e-12 { g.split_at(t0).1 } else { *g };
    let t = (t1 - t0) / (1.0 - t0).max(1e-300);
    if t < 1.0 - 1e-12 { right.split_at(t).0 } else { right }
}

/// `piece` with its ends put exactly on `p` and `q`, where they lie within [`AS_DRAWN`] of them:
/// the result's corners, which its neighbours start and end on. An arc keeps its centre and
/// radius, and so its ends as they are.
pub(crate) fn ending_at(piece: Seg2, p: Vec2, q: Vec2) -> Seg2 {
    match piece {
        Seg2::Cubic { p1, p2, .. } => Seg2::Cubic { p0: p, p1, p2, p3: q },
        Seg2::Conic { apex, w, .. } => Seg2::Conic { a: p, apex, b: q, w },
        Seg2::Bezier { n, p: mut c } => {
            if let Some(first) = c.first_mut() {
                *first = p;
            }
            if let Some(last) = c.get_mut(usize::from(n)) {
                *last = q;
            }
            Seg2::Bezier { n, p: c }
        }
        Seg2::Line { .. } => Seg2::Line { a: p, b: q },
        arc @ Seg2::Arc { .. } => arc,
    }
}

/// A development's pieces before they are joined: each placed panel less its trims, and the
/// zones, all less what the development cuts from the flat (the corners' reliefs and the sheet's
/// cuts), with each piece of a zone sorted by what still holds it (see [`sort_zones`]). The flat
/// pattern and the folded solid are both made of them.
pub(crate) fn pieces(dev: &Development, work: &mut FlatWork) -> Result<Vec<(Piece, Region2)>> {
    #[cfg(test)]
    crate::eval::cost_tests::count_flat();
    let mut pieces = Vec::new();
    for (i, flat) in dev.panels.iter().enumerate() {
        let Some(flat) = flat else { continue };
        let placed = on_grid(&flat.region);
        let trims: Vec<Region2> = dev.trims.iter().filter(|t| t.panel == i).map(|t| on_grid(&t.region)).collect();
        if trims.is_empty() {
            pieces.push((Piece::Panel(i), placed));
        } else {
            // Until the kernel's planar booleans take several tools that share an edge (#6748):
            // trims of two bends can share an edge and a corner with the panel, which the
            // planar booleans take apart only one tool at a time, so join them first.
            let trims = stage(&format!("joining panel {i}'s trims"), material_union(trims, work))?;
            let left = stage(&format!("trimming panel {i}"), boolean_past_holes(&[placed], &trims, BoolOp::Cut, work))?;
            pieces.extend(left.into_iter().map(|r| (Piece::Panel(i), r)));
        }
    }
    pieces.extend(dev.zones.iter().enumerate().map(|(z, zone)| (Piece::Zone(z), on_grid(&zone.region))));
    if dev.cuts.is_empty() {
        return Ok(pieces);
    }
    // A cut clear inside one piece is a hole of it as it is, its curves kept; the rest are cut
    // from every piece they reach. A cut that came clear of the pieces but meets what the others
    // cut from them is cut the same way, after them.
    let cuts: Vec<Region2> = dev.cuts.iter().map(on_grid).collect();
    let (inside, across): (Vec<&Region2>, Vec<&Region2>) = cuts.iter().partition(|c| pieces.iter().any(|(_, r)| holds_clear(r, c)));
    let mut out = cut_from(pieces, &across, work)?;
    let mut late = Vec::new();
    for c in inside {
        match out.iter_mut().find(|(_, r)| holds_clear(r, c)) {
            Some((_, r)) => r.holes.push(c.outer.ccw().reversed()),
            None => late.push(c),
        }
    }
    sort_zones(dev, cut_from(out, &late, work)?)
}

/// `pieces` less `cuts`, joined first: what each piece leaves, each a piece of the same kind.
fn cut_from(pieces: Vec<(Piece, Region2)>, cuts: &[&Region2], work: &mut FlatWork) -> Result<Vec<(Piece, Region2)>> {
    if cuts.is_empty() {
        return Ok(pieces);
    }
    let cuts = stage("joining what is cut from the flat", material_union(cuts.iter().map(|c| (*c).clone()).collect(), work))?;
    let mut out = Vec::with_capacity(pieces.len());
    for (piece, region) in pieces {
        out.extend(stage("cutting the flat", boolean_past_holes(&[region], &cuts, BoolOp::Cut, work))?.into_iter().map(|r| (piece, r)));
    }
    Ok(out)
}

/// Whether `cut` lies clear inside `region`, so that it is a hole of it as it is: within its
/// outline, outside its holes, crossing none of its loops and nowhere nearer them than
/// [`CLEAR`].
pub(crate) fn holds_clear(region: &Region2, cut: &Region2) -> bool {
    let Some(start) = cut.outer.segs.first().map(Seg2::start) else { return false };
    if !cut.holes.is_empty() || !region.contains(start) {
        return false;
    }
    // Chords this far inside the curves: two loops whose chords stay farther apart than twice
    // that and `CLEAR` are clear of each other.
    let sag = 1e-4;
    let ring = cut.outer.polyline(sag);
    let reach = ring.bounds().inflated(2.0 * sag + CLEAR);
    let near = |b: &Aabb2| b.min.x <= reach.max.x && reach.min.x <= b.max.x && b.min.y <= reach.max.y && reach.min.y <= b.max.y;
    let gap = |a: &Ring<Vec2>, b: &Ring<Vec2>| {
        a.iter().map(|p| b.edges().map(|(q0, q1)| segment_distance(*p, *q0, *q1)).fold(f64::INFINITY, f64::min)).fold(f64::INFINITY, f64::min)
    };
    std::iter::once(&region.outer).chain(&region.holes).all(|l| {
        let other = l.polyline(sag);
        !near(&other.bounds())
            || (l.segs.first().is_none_or(|s| !cut.outer.contains(s.start()))
                && crossings(&cut.outer.segs, &l.segs).is_empty()
                && gap(&ring, &other).min(gap(&other, &ring)) > 2.0 * sag + CLEAR)
    })
}

/// How far `p` lies from the segment from `a` to `b`.
fn segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let d = b - a;
    let t = if d.len2() > 0.0 { ((p - a).dot(d) / d.len2()).clamp(0.0, 1.0) } else { 0.0 };
    p.dist(a + d * t)
}

/// The pieces with each piece of a zone sorted by what still holds it: one that runs along both
/// its panels' tangent lines bends with the zone; one that runs along only one is a tab of that
/// panel. A piece that runs along neither is refused, and so is a zone left with nothing that
/// bends: the cuts part the sheet there.
fn sort_zones(dev: &Development, pieces: Vec<(Piece, Region2)>) -> Result<Vec<(Piece, Region2)>> {
    let mut bends = vec![false; dev.zones.len()];
    let mut out = Vec::with_capacity(pieces.len());
    for (piece, region) in pieces {
        let Piece::Zone(z) = piece else {
            out.push((piece, region));
            continue;
        };
        let zone = dev.zones.get(z).ok_or_else(|| DocError::Invalid(format!("the flat has no bend {}", z + 1)))?;
        let sorted = match (along(&region, zone.near), along(&region, zone.far)) {
            (true, true) => Piece::Zone(z),
            (true, false) => Piece::Tab(z, zone.panels[0]),
            (false, true) => Piece::Tab(z, zone.panels[1]),
            (false, false) => {
                return Err(DocError::Invalid(format!("a cut leaves a piece of bend {} joined to neither of its panels: not supported yet", z + 1)));
            }
        };
        if let Some(b) = bends.get_mut(z) {
            *b |= sorted == Piece::Zone(z);
        }
        out.push((sorted, region));
    }
    if let Some(z) = bends.iter().position(|b| !b) {
        return Err(DocError::Invalid(format!("a cut takes bend {} apart, cutting the sheet in two there: not supported yet", z + 1)));
    }
    Ok(out)
}

/// Whether an edge of `region`'s outline runs along the line from `a` to `b` for more than a
/// rounding.
fn along(region: &Region2, [a, b]: [Vec2; 2]) -> bool {
    let l = a.dist(b);
    let Some(u) = (b - a).normalized() else { return false };
    let on = |p: Vec2| (p - a).cross(u).abs() <= ON_LINE && (-ON_LINE..=l + ON_LINE).contains(&(p - a).dot(u));
    region.outer.segs.iter().any(|s| matches!(s, Seg2::Line { .. }) && s.length() > ON_LINE && on(s.start()) && on(s.end()))
}

/// A development's flat: each placed panel less its trims, and the zones, joined.
pub(crate) fn join(dev: &Development, work: &mut FlatWork) -> Result<Vec<Region2>> {
    join_pieces(&pieces(dev, work)?, work)
}

/// A development's pieces (see [`pieces`]) joined into its flat.
pub(crate) fn join_pieces(pieces: &[(Piece, Region2)], work: &mut FlatWork) -> Result<Vec<Region2>> {
    let regions: Vec<Region2> = pieces.iter().map(|(_, r)| r.clone()).collect();
    // The holes clear of every other piece stay out of the union, and go back after it.
    let reach: Vec<Aabb2> = regions.iter().map(|r| loop_box(&r.outer)).collect();
    let mut carried: Vec<Loop2> = Vec::new();
    let mut bare = Vec::with_capacity(regions.len());
    for (i, r) in regions.into_iter().enumerate() {
        let others: Vec<Aabb2> = reach.iter().enumerate().filter(|(k, _)| *k != i).map(|(_, b)| *b).collect();
        let (clear, near): (Vec<Loop2>, Vec<Loop2>) = r.holes.into_iter().partition(|h| boxes_clear(&loop_box(h), &others));
        carried.extend(clear);
        bare.push(Region2 { outer: r.outer, holes: near });
    }
    let joined = stage("joining the sheet's pieces", material_union(bare, work))?;
    stage("joining the sheet's pieces", put_back(joined, carried))
}

/// A loop's box, out by [`CLEAR`] and its chords' sag.
fn loop_box(l: &Loop2) -> Aabb2 {
    l.polyline(1e-4).bounds().inflated(CLEAR + 2e-4)
}

/// Whether box `b` meets none of `of`.
fn boxes_clear(b: &Aabb2, of: &[Aabb2]) -> bool {
    of.iter().all(|o| b.max.x < o.min.x || o.max.x < b.min.x || b.max.y < o.min.y || o.max.y < b.min.y)
}

/// The holes `carried` back in the regions of `out` that hold them.
fn put_back(mut out: Vec<Region2>, carried: Vec<Loop2>) -> Result<Vec<Region2>> {
    for h in carried {
        let Some(at) = h.segs.first().map(Seg2::start) else { continue };
        let r = out
            .iter_mut()
            .find(|r| r.outer.contains(at) && !r.holes.iter().any(|k| k.contains(at)))
            .ok_or_else(|| DocError::Invalid("a cut-out of the flat lost its piece".into()))?;
        r.holes.push(h);
    }
    Ok(out)
}

/// Whether loop `h` keeps clear of region `t`: apart from it by their boxes, or outside its
/// material and holding none of it, its loops crossing none of `t`'s and nowhere nearer them than
/// [`CLEAR`].
fn loop_clear_of(h: &Loop2, t: &Region2) -> bool {
    let hb = loop_box(h);
    if boxes_clear(&hb, std::slice::from_ref(&loop_box(&t.outer))) {
        return true;
    }
    let (Some(p), ring) = (h.segs.first().map(Seg2::start), h.ccw()) else { return false };
    if t.contains(p) {
        return false;
    }
    let sag = 1e-4;
    let mine = h.polyline(sag);
    std::iter::once(&t.outer).chain(&t.holes).all(|l| {
        let other = l.polyline(sag);
        if boxes_clear(&hb, std::slice::from_ref(&other.bounds())) {
            return true;
        }
        let gap = |a: &Ring<Vec2>, b: &Ring<Vec2>| {
            a.iter().map(|p| b.edges().map(|(q0, q1)| segment_distance(*p, *q0, *q1)).fold(f64::INFINITY, f64::min)).fold(f64::INFINITY, f64::min)
        };
        l.segs.first().is_none_or(|s| !ring.contains(s.start()))
            && crossings(&h.segs, &l.segs).is_empty()
            && gap(&mine, &other).min(gap(&other, &mine)) > 2.0 * sag + CLEAR
    })
}

/// `op` of `a` with `b` ([`material_boolean`]), the holes of `a` clear of every region of `b`
/// (see [`loop_clear_of`]) left out of it and put back in the region of the result that holds
/// them.
pub(crate) fn boolean_past_holes(a: &[Region2], b: &[Region2], op: BoolOp, work: &mut FlatWork) -> Result<Vec<Region2>> {
    let clear_of_tools = |h: &Loop2| b.iter().all(|t| loop_clear_of(h, t));
    let mut carried: Vec<Loop2> = Vec::new();
    let bare: Vec<Region2> = a
        .iter()
        .map(|r| {
            let (clear, near): (Vec<Loop2>, Vec<Loop2>) = r.holes.iter().cloned().partition(|h| clear_of_tools(h));
            carried.extend(clear);
            Region2 { outer: r.outer.clone(), holes: near }
        })
        .collect();
    if carried.is_empty() {
        return material_boolean(a, b, op, work);
    }
    put_back(material_boolean(&bare, b, op, work)?, carried)
}

/// A region with its points on the sheet's nanometre grid, as the sheet model rounds its
/// corners. Until the kernel's planar booleans weld edges that nearly coincide (#6748): the
/// development is exact, but a corner it reaches by another sum than the sheet model does can
/// land a rounding from the sheet's, and the planar booleans take two edges that nearly
/// coincide for two, where on the grid they coincide.
fn on_grid(r: &Region2) -> Region2 {
    let g = |v: Vec2| Vec2::new((v.x * 1e9).round() / 1e9, (v.y * 1e9).round() / 1e9);
    let seg = |s: &Seg2| match *s {
        Seg2::Line { a, b } => Seg2::Line { a: g(a), b: g(b) },
        Seg2::Arc { center, radius, start, sweep } => Seg2::Arc { center: g(center), radius, start, sweep },
        Seg2::Cubic { p0, p1, p2, p3 } => Seg2::Cubic { p0: g(p0), p1: g(p1), p2: g(p2), p3: g(p3) },
        Seg2::Conic { a, apex, b, w } => Seg2::Conic { a: g(a), apex: g(apex), b: g(b), w },
        Seg2::Bezier { n, mut p } => {
            for q in p.iter_mut().take(usize::from(n) + 1) {
                *q = g(*q);
            }
            Seg2::Bezier { n, p }
        }
    };
    let lp = |l: &Loop2| Loop2 { segs: l.segs.iter().map(seg).collect() };
    Region2 { outer: lp(&r.outer), holes: r.holes.iter().map(lp).collect() }
}

/// Names the step of the round trip an error came from.
fn stage<T>(what: &str, r: Result<T>) -> Result<T> {
    r.map_err(|e| DocError::Invalid(format!("{what}: {e}")))
}

/// A sheet's flat laid out again from its surfaces.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RoundTrip {
    /// How far the surfaces' flat is from the sheet's: the area between them over the length of
    /// the sheet's outline (mm).
    pub apart: f64,
    /// How many joints close loops.
    pub loops: usize,
    /// How the surfaces' flat edges resolve to the surfaces.
    pub edges: Resolution,
}

pub(crate) fn round_trip(sh: &SheetBody) -> Result<RoundTrip> {
    compare(sh, &stage("building its surfaces", surfaces_of(sh))?)
}

fn compare(sh: &SheetBody, model: &Sheet) -> Result<RoundTrip> {
    let mut work = FlatWork::new();
    let mine = stage("joining the sheet's pieces", sheet_flat(sh, &mut work))?;
    let (theirs, dev) = surface_flat(model, &mut work)?;
    let area = |rs: &[Region2]| rs.iter().map(Region2::area).sum::<f64>();
    let apart = area(&stage("comparing", material_boolean(&mine, &theirs, BoolOp::Cut, &mut work))?)
        + area(&stage("comparing", material_boolean(&theirs, &mine, BoolOp::Cut, &mut work))?);
    let outline: f64 = mine.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)).flat_map(|l| &l.segs).map(|s| s.length()).sum();
    let edges = sheet_flat_lineage::resolve(&theirs, &sheet_flat_lineage::sources(&dev));
    Ok(RoundTrip { apart: apart / outline.max(1e-9), loops: dev.loops.len(), edges })
}

/// Appends this sheet's round trip to the file `SOLIDCRAFT_SURFACE_REPORT` names, when it is set.
pub(crate) fn report(sh: &SheetBody) {
    #[cfg(not(target_arch = "wasm32"))]
    {
        use std::io::Write;
        let Some(path) = std::env::var_os("SOLIDCRAFT_SURFACE_REPORT") else { return };
        let what = sh
            .flanges
            .iter()
            .map(|f| {
                let mut s = String::new();
                for (on, c) in [
                    (f.parent.is_some(), 'r'),
                    (f.angle.abs() >= std::f64::consts::PI - HALF_TURN_TOL, 'h'),
                    (f.tear.0 || f.tear.1, 't'),
                    (f.slope != (0.0, 0.0), 's'),
                    (f.ext != (0.0, 0.0), 'e'),
                    (f.shape.is_some(), 'f'),
                    (f.exact.is_some(), 'x'),
                ] {
                    if on {
                        s.push(c);
                    }
                }
                if s.is_empty() { "-".to_string() } else { s }
            })
            .collect::<Vec<_>>()
            .join(",");
        let result = match round_trip(sh) {
            Ok(rt) => {
                let e = rt.edges;
                let same = if rt.apart <= SAME_FLAT { "same" } else { "differs" };
                format!("{same}\t{:.3e}\t{}\t{}\t{}\t{}\t{}", rt.apart, rt.loops, e.exact, e.merged, e.pieces, e.geometry_only)
            }
            Err(e) => format!("error\t{}", e.to_string().replace(['\t', '\n'], " ")),
        };
        let thread = std::thread::current().name().unwrap_or("main").to_string();
        let line = format!("{thread}\t{}\t{}\t{what}\t{result}\n", sh.body, sh.flanges.len());
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = file.write_all(line.as_bytes());
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use solidcore_record::{Plane, Vec3};
    use solidcraft_geom::prelude::*;

    use super::{SAME_FLAT, round_trip};
    use crate::sheet::{AngleDisplay, ReliefShape, RuleValues, SheetBody, base_flange};

    const T: f64 = 2.0;
    const R: f64 = 2.0;

    /// A 100 x 60 plate of 2 mm sheet bent R 2 at K 0.44, under a rule with `corner`.
    fn plate(corner: ReliefShape) -> SheetBody {
        let rv = RuleValues {
            thickness: T,
            k_factor: 0.44,
            bend_radius: R,
            relief_width: 1.0,
            relief_depth: 0.5,
            corner_relief: 6.0,
            corner_relief_shape: corner,
            hem_gap: 0.01,
            gap: 0.5,
            angle_display: AngleDisplay::Bend,
        };
        let pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 60.0), (0.0, 60.0)].map(|(x, y)| solidcore_record::Vec2::new(x, y));
        let region = solidcore_record::Region2 { outer: solidcore_record::Loop2::polygon(&pts).ccw(), holes: Vec::new() };
        base_flange("Plate", "Rule", &rv, Plane::XY, region)
    }

    const EDGES: [Vec3; 4] = [Vec3::new(50.0, 0.0, T), Vec3::new(100.0, 30.0, T), Vec3::new(50.0, 60.0, T), Vec3::new(0.0, 30.0, T)];

    /// The surfaces' flat is the sheet's, closes no loop, and every edge of it is something in
    /// the surfaces.
    fn same(sh: &SheetBody) {
        let rt = match round_trip(sh) {
            Ok(rt) => rt,
            Err(e) => panic!("{e}"),
        };
        assert!(rt.apart <= SAME_FLAT && rt.loops == 0, "the surfaces' flat is {:e} mm from the sheet's ({} loops)", rt.apart, rt.loops);
        assert!(rt.edges.geometry_only == 0 && rt.edges.exact > 0, "{:?}", rt.edges);
    }

    /// A planar boolean gives back the curves of its inputs as they were drawn: a short Bézier
    /// that lies within a hair of a circle stays a Bézier where the kernel reads it as an arc,
    /// and the edges of each loop of the result meet exactly.
    #[test]
    fn a_planar_boolean_keeps_its_inputs_curves() {
        use solidcore_record::{Loop2, Region2, Seg2, Vec2};
        use solidcraft_kernel::BoolOp;
        // A cap: the chord across an arc of radius 20 turning 0.2 rad, and the arc as one cubic,
        // which lies within 1e-7 mm of the circle.
        let (r, sweep) = (20.0f64, 0.2f64);
        let at = |a: f64| Vec2::new(r * a.cos(), r * a.sin());
        let k = 4.0 / 3.0 * (sweep / 4.0).tan() * r;
        let (p0, p3) = (at(0.0), at(sweep));
        let (p1, p2) = (p0 + Vec2::new(0.0, k), p3 + Vec2::new(sweep.sin(), -sweep.cos()) * k);
        let cap = Region2 { outer: Loop2 { segs: vec![Seg2::Cubic { p0, p1, p2, p3 }, Seg2::Line { a: p3, b: p0 }] }, holes: Vec::new() };
        // Joined with a box beside it, the cap's curve is read again with the box's outline.
        let square = Region2 {
            outer: Loop2::polygon(&[Vec2::new(30.0, 0.0), Vec2::new(40.0, 0.0), Vec2::new(40.0, 10.0), Vec2::new(30.0, 10.0)]).ccw(),
            holes: Vec::new(),
        };
        let kinds = |rs: &[Region2]| {
            let segs: Vec<&Seg2> = rs.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)).flat_map(|l| &l.segs).collect();
            (segs.iter().filter(|s| matches!(s, Seg2::Cubic { .. })).count(), segs.iter().filter(|s| matches!(s, Seg2::Arc { .. })).count())
        };
        let joined = match super::boolean_as_drawn(std::slice::from_ref(&cap), std::slice::from_ref(&square), BoolOp::Union, 1e-7) {
            Ok(j) => j,
            Err(e) => panic!("{e}"),
        };
        assert_eq!(kinds(&joined), (1, 0), "cubics, arcs: {joined:?}");
        let curve = joined.iter().flat_map(|r| &r.outer.segs).find(|s| matches!(s, Seg2::Cubic { .. }));
        // The cap's own cubic, end to end.
        let drawn = Seg2::Cubic { p0, p1, p2, p3 };
        assert!(curve.is_some_and(|c| [0.0, 0.3, 0.7, 1.0].iter().all(|t| c.point_at(*t).dist(drawn.point_at(*t)) < 1e-9)), "{curve:?}");
        for l in joined.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)) {
            let n = l.segs.len();
            assert!((0..n).all(|i| l.segs[i].end() == l.segs[(i + 1) % n].start()), "{l:?}");
        }
    }

    /// Two arcs that a loop's corner joins, each read a hair past or short of it, both end where
    /// their circles meet there, each by its own angle, and keep their other ends.
    #[test]
    fn two_arcs_at_a_corner_end_where_their_circles_meet() {
        use std::f64::consts::PI;

        use solidcore_record::{Seg2, Vec2};
        let arc = |x: f64, y: f64, radius: f64, start: f64, sweep: f64| Seg2::Arc { center: Vec2::new(x, y), radius, start, sweep };
        let closed = |a: Seg2, b: Seg2, at: Vec2| {
            assert!(a.end().dist(b.start()) > 1e-5, "{a:?} {b:?}");
            let mut segs = vec![a, b, Seg2::Line { a: b.end(), b: a.start() }];
            super::close_up(&mut segs);
            assert!(segs[0].end().dist(at) < 1e-12 && segs[1].start().dist(at) < 1e-12, "{segs:?}");
            assert!(segs[0].start().dist(a.start()) < 1e-12 && segs[1].end().dist(b.end()) < 1e-12, "{segs:?}");
            assert!((0..3).all(|i| segs[i].end().dist(segs[(i + 1) % 3].start()) < 1e-12), "{segs:?}");
        };
        // Square across each other: up the circle about the origin to (0, 10), then on along the
        // one about (10, 10), which crosses it there.
        closed(arc(0.0, 0.0, 10.0, 0.0, FRAC_PI_2 + 3e-6), arc(10.0, 10.0, 10.0, PI + 2e-6, -FRAC_PI_2 - 2e-6), Vec2::new(0.0, 10.0));
        // Smoothly into each other: along the circle about the origin to (0, 10), then along the
        // one about (0, 20), which touches it there.
        closed(arc(0.0, 0.0, 10.0, PI, -FRAC_PI_2 - 2e-6), arc(0.0, 20.0, 10.0, -FRAC_PI_2 - 3e-6, FRAC_PI_2 + 3e-6), Vec2::new(0.0, 10.0));
        // Two pieces of one circle: the second starts where the first ends.
        let first = arc(5.0, -3.0, 4.0, 0.0, 1.0 + 4e-6);
        closed(first, arc(5.0, -3.0, 4.0, 1.0 - 3e-6, 1.0), first.end());
    }

    #[test]
    fn a_bracket_and_a_tray_develop_from_their_surfaces() {
        let mut bracket = plate(ReliefShape::Notch);
        assert!(bracket.add_flange(Vec3::new(50.0, 0.0, T), 20.0, FRAC_PI_2, R, "inside", false).is_ok());
        same(&bracket);
        let mut tray = plate(ReliefShape::Notch);
        assert!(tray.add_flanges(&EDGES, 20.0, FRAC_PI_2, R, "inside", false).is_ok());
        same(&tray);
    }

    /// Bends that turn away from the material, past 90 degrees, and from other positions on the
    /// edge, and a hem, whose anchors are its tangent lines.
    #[test]
    fn bends_either_way_and_a_hem_develop_from_their_surfaces() {
        let mut sh = plate(ReliefShape::Notch);
        assert!(sh.add_flange(Vec3::new(50.0, 0.0, T), 20.0, 2.0, R, "tangent", false).is_ok());
        assert!(sh.add_flange(Vec3::new(100.0, 30.0, T), 15.0, 1.0, R, "outside", true).is_ok());
        assert!(sh.add_hem(Vec3::new(50.0, 60.0, T), 10.0, 0.5, false).is_ok());
        same(&sh);
    }

    /// Torn corners slant the zones' ends, and a return stands on a flange.
    #[test]
    fn torn_corners_and_a_return_develop_from_their_surfaces() {
        let mut sh = plate(ReliefShape::Tear);
        assert!(sh.add_flanges(&EDGES, 20.0, FRAC_PI_2, R, "inside", false).is_ok());
        assert!(sh.flanges.iter().any(|f| f.tear.0 || f.tear.1));
        let Some(top) = sh.far_edge_pick(0) else { panic!("the first flange has a far edge") };
        assert!(sh.add_flange(top, 10.0, FRAC_PI_2, R, "inside", false).is_ok());
        same(&sh);
    }

    /// A flap on the end of a wall: the wall's panel loses a band to its own bend and one to
    /// the flap's, which meet at its corner.
    #[test]
    fn a_flap_on_the_end_of_a_wall_develops_from_its_surfaces() {
        let mut sh = plate(ReliefShape::Notch);
        assert!(sh.add_flange(Vec3::new(0.0, 30.0, T), 40.0, FRAC_PI_2, R, "inside", false).is_ok());
        assert!(sh.add_flange(Vec3::new(1.0, 0.0, 20.0), 40.0, std::f64::consts::FRAC_PI_4, R, "inside", false).is_ok());
        assert!(sh.flanges.get(1).is_some_and(|f| f.parent == Some(0)), "the flap stands on the wall");
        same(&sh);
    }

    /// The check sees wrong surfaces: one whose K-factor is off by 0.1 puts the flange 0.1 x T x
    /// pi / 2 from where the sheet has it.
    #[test]
    fn surfaces_with_another_k_factor_do_not_give_the_sheets_flat() {
        let mut sh = plate(ReliefShape::Notch);
        assert!(sh.add_flange(Vec3::new(50.0, 0.0, T), 20.0, FRAC_PI_2, R, "inside", false).is_ok());
        let Ok(mut model) = super::surfaces_of(&sh) else { panic!("its surfaces") };
        model.k_factor += 0.1;
        let apart = super::compare(&sh, &model).map_or(0.0, |rt| rt.apart);
        // The flange and its far edge shift 0.1 pi mm: 100 mm long, over a 2 x (100 + 80.something)
        // outline, a fraction of that on average.
        assert!(apart > 0.01, "{apart}");
    }
}
