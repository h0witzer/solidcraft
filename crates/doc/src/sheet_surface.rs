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
use crate::sheet_surface_cuts::{holds, near_each_other, seg_box};
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
    let dev = develop(model, &[]).map_err(|e| DocError::Invalid(e.to_string()))?;
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

/// How far (mm) from a corner of a boolean's result, as the boolean read it, the curves of its two
/// edges may meet and still meet at that corner: ten times [`AS_DRAWN`], how far a piece of an
/// input curve may end from the corners read for its edge.
const ONE_CORNER: f64 = 1e-5;

/// How near (mm) two curves must come to cross or touch: rounding at a sheet's sizes, a tenth of
/// the least the booleans here are asked to hold.
const MEET: f64 = 1e-10;

/// `op` of `a` with `b` by the kernel's planar boolean, with each curved edge of the result that
/// runs along a curve of `a` or `b` given back as that curve's own piece. The boolean reads its
/// result again from a B-rep, and fits arcs to free-form spans that lie near a circle (a
/// letter's flat curves) and splines to conics; here a Bézier stays a Bézier and a conic a
/// conic. A corner beside such a piece is put where the curves of its two edges meet, which the
/// boolean read only to within its tolerance (see [`closed_up`]). Every other corner, and every
/// edge that follows no input curve, stays as read.
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
    let redraw = |l: Loop2| Loop2 { segs: closed_up(l.segs.into_iter().map(|s| piece_of(&curves, &s).unwrap_or(Edge::Read(s))).collect()) };
    out.into_iter().map(|r| Region2 { outer: redraw(r.outer), holes: r.holes.into_iter().map(redraw).collect() }).collect()
}

/// An edge of a planar boolean's result: as the boolean read it, or the piece of a curve of its
/// inputs that it runs along ([`piece_of`]).
#[derive(Clone, Copy, Debug)]
enum Edge {
    Read(Seg2),
    /// The part of `of` from its parameter `t0` to `t1`, either way round, which the boolean read
    /// as `read`.
    Piece {
        of: Seg2,
        t0: f64,
        t1: f64,
        read: Seg2,
    },
}

impl Edge {
    /// The edge as the boolean read it.
    fn read(&self) -> Seg2 {
        match *self {
            Edge::Read(s) | Edge::Piece { read: s, .. } => s,
        }
    }

    /// The curve the edge lies on, which its ends may move along: the line through a line's ends,
    /// an arc's circle, a piece's input curve. None for a free-form edge as read, whose curve isn't
    /// known past its ends.
    fn own(&self) -> Option<Own> {
        match *self {
            Edge::Read(Seg2::Line { a, b }) => Some(Own::Line(a, b)),
            Edge::Read(Seg2::Arc { center, radius, .. }) | Edge::Piece { of: Seg2::Arc { center, radius, .. }, .. } => {
                Some(Own::Circle(center, radius))
            }
            Edge::Read(_) => None,
            Edge::Piece { of, .. } => Some(Own::Curve(of)),
        }
    }

    /// Where the edge ends (`end`) or starts, as read, on its own curve ([`Own`]'s parameter).
    fn param(&self, end: bool) -> f64 {
        let pick = |s: f64, e: f64| if end { e } else { s };
        match *self {
            Edge::Read(Seg2::Arc { start, sweep, .. }) => pick(start, start + sweep),
            Edge::Read(_) => pick(0.0, 1.0),
            Edge::Piece { of: Seg2::Arc { start, sweep, .. }, t0, t1, .. } => start + sweep * pick(t0, t1),
            Edge::Piece { t0, t1, .. } => pick(t0, t1),
        }
    }
}

/// The curve an edge lies on ([`Edge::own`]), parametrized as the edge is: a line from its start
/// (0) to its end (1) and on past them, a circle by angle, an input curve by its own parameter.
#[derive(Clone, Copy, Debug)]
enum Own {
    Line(Vec2, Vec2),
    Circle(Vec2, f64),
    Curve(Seg2),
}

impl Own {
    fn at(&self, u: f64) -> Vec2 {
        match *self {
            Own::Line(a, b) => a.lerp(b, u),
            Own::Circle(c, r) => c + Vec2::from_angle(u) * r,
            Own::Curve(g) => g.point_at(u),
        }
    }

    fn derivative(&self, u: f64) -> Vec2 {
        match *self {
            Own::Line(a, b) => b - a,
            Own::Circle(_, r) => Vec2::from_angle(u).perp() * r,
            Own::Curve(g) => g.derivative(u),
        }
    }

    /// The parameter of the curve's point nearest `p`, from `u`: exact on a line and a circle,
    /// by Newton's steps along a curve, which find the foot of a point near it.
    fn foot(&self, p: Vec2, u: f64) -> f64 {
        match *self {
            Own::Line(a, b) => {
                let d = b - a;
                let t = (p - a).dot(d) / d.len2();
                if t.is_finite() { t } else { u }
            }
            Own::Circle(c, _) => near_angle((p - c).angle(), u),
            Own::Curve(g) => {
                let mut t = u;
                for _ in 0..16 {
                    let d = g.derivative(t);
                    let step = (p - g.point_at(t)).dot(d) / d.len2();
                    if !step.is_finite() {
                        break;
                    }
                    t += step;
                    if step.abs() <= 1e-15 {
                        break;
                    }
                }
                t
            }
        }
    }
}

/// A corner put where the curves of its two edges meet ([`meet`]): the parameter on its own curve
/// where the edge before the corner ends, and where the edge after it starts.
#[derive(Clone, Copy, Debug)]
struct Corner {
    end: f64,
    start: f64,
}

/// A loop's edges as [`as_drawn`] gives them back. At each corner beside a piece of an input curve,
/// both edges end where their curves meet near it ([`meet`]): a line stays on its line, an arc on
/// its circle and a piece on its input curve, and the two end on one point (two arcs, whose
/// angles place their ends, to rounding). The boolean reads each edge's ends only to within the
/// B-rep's tolerance, and an arc it fits ends near its corner, not on it: a smooth run of curves
/// can't be followed across the step that leaves, and the next boolean refuses a loop whose steps
/// outgrow its own tolerance.
///
/// Every other corner stays where the boolean read it, steps and all: closing a loop the boolean
/// gave back open is the kernel's to do. So does a piece's end where the curves don't meet, as
/// [`ending_at`] puts it, except an arc's, which its angles place: an arc piece with such a corner
/// is kept as read. So is a corner where the edges' meeting would turn an edge round or leave
/// nothing of it.
fn closed_up(mut edges: Vec<Edge>) -> Vec<Seg2> {
    if !edges.iter().any(|e| matches!(e, Edge::Piece { .. })) {
        return edges.iter().map(Edge::read).collect();
    }
    let n = edges.len();
    let next = |k: usize| if k + 1 < n { k + 1 } else { 0 };
    let prev = |k: usize| if k > 0 { k - 1 } else { n - 1 };
    // Corner `k` is where edge `k` ends and the next starts; `kept[k]` keeps it as read.
    let mut kept = vec![false; n];
    let meets = |edges: &[Edge], kept: &[bool], k: usize| -> Option<Corner> {
        let (a, b) = (edges.get(k)?, edges.get(next(k))?);
        if kept.get(k).copied().unwrap_or(true) || !(matches!(a, Edge::Piece { .. }) || matches!(b, Edge::Piece { .. })) {
            return None;
        }
        let (e, s) = (a.read().end(), b.read().start());
        if e.dist(s) > ONE_CORNER {
            return None;
        }
        meet(a, b, e.lerp(s, 0.5))
    };
    let mut corners: Vec<Option<Corner>> = (0..n).map(|k| meets(&edges, &kept, k)).collect();
    // Each round that changes anything keeps an arc piece as read or keeps a met corner as read,
    // and neither is undone: at most `2n` of them.
    for _ in 0..=2 * n {
        let mut changed = false;
        for i in 0..n {
            let Some(&edge) = edges.get(i) else { continue };
            let (before, after) = (corners.get(prev(i)).copied().flatten(), corners.get(i).copied().flatten());
            if let Edge::Piece { of: Seg2::Arc { .. }, read, .. } = edge
                && (before.is_none() || after.is_none())
            {
                if let Some(e) = edges.get_mut(i) {
                    *e = Edge::Read(read);
                }
                for k in [prev(i), i] {
                    let c = meets(&edges, &kept, k);
                    if let Some(slot) = corners.get_mut(k) {
                        *slot = c;
                    }
                }
                changed = true;
            } else if !keeps_its_way(&edge, before.map(|c| c.start), after.map(|c| c.end)) {
                for k in [prev(i), i] {
                    if let Some(x) = kept.get_mut(k) {
                        *x = true;
                    }
                    if let Some(x) = corners.get_mut(k) {
                        *x = None;
                    }
                }
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    let at = |k: usize| corners.get(k).copied().flatten();
    let mut segs: Vec<Seg2> = edges.iter().enumerate().map(|(i, e)| shaped(e, at(prev(i)).map(|c| c.start), at(i).map(|c| c.end))).collect();
    // Each met corner is one point: where an arc's angle puts its end, or else on a line, or else
    // where an input curve ends (at its parameter 0 or 1), or else where the edge before it ends.
    let rank = |s: &Seg2, e: Option<&Edge>, u: f64| match (s, e) {
        (Seg2::Arc { .. }, _) => 3,
        (_, Some(Edge::Read(Seg2::Line { .. }))) => 2,
        (_, Some(Edge::Piece { .. })) if [0.0, 1.0].contains(&u) => 1,
        _ => 0,
    };
    for k in 0..n {
        let Some(c) = at(k) else { continue };
        let j = next(k);
        let (Some(&a), Some(&b)) = (segs.get(k), segs.get(j)) else { continue };
        let x = if rank(&a, edges.get(k), c.end) >= rank(&b, edges.get(j), c.start) { a.end() } else { b.start() };
        if let Some(s) = segs.get_mut(k)
            && !matches!(s, Seg2::Arc { .. })
        {
            *s = ending_at(*s, s.start(), x);
        }
        if let Some(s) = segs.get_mut(j)
            && !matches!(s, Seg2::Arc { .. })
        {
            *s = ending_at(*s, x, s.end());
        }
    }
    segs
}

/// Where the curve of `a`, which ends at a corner, and that of `b`, which starts there, meet near
/// `near`, the corner as the boolean read it, and within [`ONE_CORNER`] of it:
/// - where they cross or touch, each within [`MEET`] of the point: two circles
///   ([`circles_meet`]), a line and a circle ([`line_meets_circle`]), two pieces of one input
///   curve anywhere along it, and any others where Newton's steps find one crossing ([`crossing`]);
/// - or else where an input curve ends within [`AS_DRAWN`] of the other edge's curve: a corner of
///   the inputs, where its two curves run on from each other (a smooth join has no one crossing).
///   The piece's end moves onto the other edge's curve there, by no more than [`ending_at`] moves
///   a piece's end onto a corner the boolean read.
///
/// None where they don't meet there, and beside a free-form edge as read.
fn meet(a: &Edge, b: &Edge, near: Vec2) -> Option<Corner> {
    let (ca, cb) = (a.own()?, b.own()?);
    let (ua, ub) = (a.param(true), b.param(false));
    let found = match (ca, cb) {
        (Own::Line(..), Own::Line(..)) => None,
        (Own::Circle(c0, r0), Own::Circle(c1, r1)) => circles_meet((c0, r0), (c1, r1), near).map(|x| (ca.foot(x, ua), cb.foot(x, ub))),
        (Own::Line(p, q), Own::Circle(c, r)) | (Own::Circle(c, r), Own::Line(p, q)) => {
            line_meets_circle((p, q), (c, r), near).map(|x| (ca.foot(x, ua), cb.foot(x, ub)))
        }
        // Two pieces of one curve, one after the other along it (not where it crosses itself).
        (Own::Curve(g), Own::Curve(h)) if g == h && (ca.foot(near, ua) - cb.foot(near, ub)).abs() <= 1e-9 => {
            let t = ca.foot(near, ua);
            (ca.at(t).dist(near) <= AS_DRAWN).then_some((t, t))
        }
        _ => crossing(ca, ua, cb, ub, near),
    };
    found
        .or_else(|| match ca {
            Own::Curve(g) => ends_on(&g, ua, cb, ub, near),
            _ => None,
        })
        .or_else(|| match cb {
            Own::Curve(g) => ends_on(&g, ub, ca, ua, near).map(|(v, u)| (u, v)),
            _ => None,
        })
        .map(|(end, start)| Corner { end, start })
}

/// Where curves `a` and `b` cross near their parameters `u` and `v`, as those parameters, by
/// Newton's steps: None where they run together there (with no one crossing to find), and where
/// the steps end more than [`MEET`] apart or farther than [`ONE_CORNER`] from `near`.
fn crossing(a: Own, mut u: f64, b: Own, mut v: f64, near: Vec2) -> Option<(f64, f64)> {
    for _ in 0..32 {
        let f = a.at(u) - b.at(v);
        let (da, db) = (a.derivative(u), b.derivative(v));
        let turn = da.cross(db);
        if !(turn.abs() > 1e-9 * da.len() * db.len()) {
            return None;
        }
        let (du, dv) = (-f.cross(db) / turn, da.cross(f) / turn);
        if !(du.is_finite() && dv.is_finite()) {
            return None;
        }
        u += du;
        v += dv;
        if (da * du).len() + (db * dv).len() <= MEET * 1e-3 {
            break;
        }
    }
    let (p, q) = (a.at(u), b.at(v));
    (p.dist(q) <= MEET && p.dist(near) <= ONE_CORNER).then_some((u, v))
}

/// Where input curve `g`, at its parameter `u` at a corner near `near`, ends there, within
/// [`AS_DRAWN`] of curve `other`: the end's parameter on `g`, and that of the point of `other`
/// nearest it, from `v`.
fn ends_on(g: &Seg2, u: f64, other: Own, v: f64, near: Vec2) -> Option<(f64, f64)> {
    let (t, end) = if u < 0.5 { (0.0, g.start()) } else { (1.0, g.end()) };
    if end.dist(near) > ONE_CORNER {
        return None;
    }
    let w = other.foot(end, v);
    (other.at(w).dist(end) <= AS_DRAWN).then_some((t, w))
}

/// Edge `edge` with its start and end at its own curve's parameters `s` and `e` ([`Own`]), where
/// its corners meet, and where None, as read: a piece then ends on the corner the boolean read
/// ([`ending_at`]).
fn shaped(edge: &Edge, s: Option<f64>, e: Option<f64>) -> Seg2 {
    match *edge {
        Edge::Read(Seg2::Line { a, b }) => Seg2::Line { a: s.map_or(a, |u| a.lerp(b, u)), b: e.map_or(b, |u| a.lerp(b, u)) },
        Edge::Read(Seg2::Arc { center, radius, start, sweep }) => {
            let (from, to) = (s.map_or(start, |u| near_angle(u, start)), e.map_or(start + sweep, |u| near_angle(u, start + sweep)));
            Seg2::Arc { center, radius, start: from, sweep: to - from }
        }
        Edge::Read(seg) => seg,
        Edge::Piece { of: Seg2::Arc { center, radius, start, sweep }, t0, t1, .. } => {
            let (a0, a1) = (start + sweep * t0, start + sweep * t1);
            let (from, to) = (s.map_or(a0, |u| near_angle(u, a0)), e.map_or(a1, |u| near_angle(u, a1)));
            Seg2::Arc { center, radius, start: from, sweep: to - from }
        }
        Edge::Piece { of, t0, t1, read } => {
            let piece = between(&of, s.map_or(t0, |u| u.clamp(0.0, 1.0)), e.map_or(t1, |u| u.clamp(0.0, 1.0)));
            ending_at(piece, if s.is_some() { piece.start() } else { read.start() }, if e.is_some() { piece.end() } else { read.end() })
        }
    }
}

/// Whether `edge`, started and ended at its own curve's parameters `s` and `e` (as read where
/// None), still runs the way it was read, and over more than nothing.
fn keeps_its_way(edge: &Edge, s: Option<f64>, e: Option<f64>) -> bool {
    let (s0, e0) = (edge.param(false), edge.param(true));
    let arc = matches!(edge, Edge::Read(Seg2::Arc { .. }) | Edge::Piece { of: Seg2::Arc { .. }, .. });
    // As `shaped` takes them: an arc's angles turned to its own, a piece's parameters on its curve.
    let to = |u: Option<f64>, was: f64| match u {
        Some(u) if arc => near_angle(u, was),
        Some(u) if matches!(edge, Edge::Piece { .. }) => u.clamp(0.0, 1.0),
        Some(u) => u,
        None => was,
    };
    let (was, now) = (e0 - s0, to(e, e0) - to(s, s0));
    was * now > 0.0 && now.abs() > 1e-12 && (!arc || now.abs() <= std::f64::consts::TAU + 1e-12)
}

/// Where the circle about `c0` of radius `r0` and the one about `c1` of radius `r1` meet nearest
/// `near`: where they cross or touch, within [`MEET`] of both (for two that miss by less, the
/// point between them on the line through their centres), and anywhere where they are one circle.
/// None where that is more than [`ONE_CORNER`] from `near`.
fn circles_meet((c0, r0): (Vec2, f64), (c1, r1): (Vec2, f64), near: Vec2) -> Option<Vec2> {
    let on = |p: &Vec2| (p.dist(c0) - r0).abs() <= MEET && (p.dist(c1) - r1).abs() <= MEET && p.dist(near) <= ONE_CORNER;
    let Some(u) = (c1 - c0).normalized() else {
        // About one centre: one circle, or two that never meet.
        let p = c0 + (near - c0).normalized()? * r0;
        return on(&p).then_some(p);
    };
    let l = (c1 - c0).len();
    // From `c0` along the line of centres to the chord through the crossings, and half the chord.
    let along = (r0 * r0 - r1 * r1 + l * l) / (2.0 * l);
    let half = (r0 * r0 - along * along).max(0.0).sqrt();
    let foot = c0 + u * along;
    [foot + u.perp() * half, foot - u.perp() * half].into_iter().filter(on).min_by(|p, q| p.dist(near).total_cmp(&q.dist(near)))
}

/// Where the line through `p` and `q` meets the circle about `c` of radius `r` nearest `near`:
/// where it crosses or touches it, within [`MEET`] (for one that misses by less, its point nearest
/// the centre). None where that is more than [`ONE_CORNER`] from `near`.
fn line_meets_circle((p, q): (Vec2, Vec2), (c, r): (Vec2, f64), near: Vec2) -> Option<Vec2> {
    let d = (q - p).normalized()?;
    let foot = p + d * (c - p).dot(d);
    let off = foot.dist(c);
    if off > r + MEET {
        return None;
    }
    let half = (r * r - off * off).max(0.0).sqrt();
    [foot + d * half, foot - d * half].into_iter().filter(|x| x.dist(near) <= ONE_CORNER).min_by(|x, y| x.dist(near).total_cmp(&y.dist(near)))
}

/// The angle `a`, turned by whole turns to lie within half a turn of `to`.
fn near_angle(a: f64, to: f64) -> f64 {
    let d = a - to;
    to + d - std::f64::consts::TAU * (d / std::f64::consts::TAU).round()
}

/// The piece of one of `curves` that edge `s` runs along, from `s`'s start to its end: within
/// [`AS_DRAWN`] of `s` at both ends and at three points between. None for an edge that closes on
/// itself, and where no curve fits.
fn piece_of(curves: &[(Seg2, Aabb2)], s: &Seg2) -> Option<Edge> {
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
        let piece = between(g, t0, t1);
        if piece.start().dist(p) <= AS_DRAWN
            && piece.end().dist(q) <= AS_DRAWN
            && probes.iter().all(|x| piece.point_at(piece.closest_param(*x)).dist(*x) <= AS_DRAWN)
        {
            return Some(Edge::Piece { of: *g, t0, t1, read: *s });
        }
    }
    None
}

/// The part of `g` from its parameter `t0` to `t1`, either way round.
fn between(g: &Seg2, t0: f64, t1: f64) -> Seg2 {
    if t0 < t1 { part(g, t0, t1) } else { part(g, t1, t0).reversed() }
}

/// The part of `g` from parameter `t0` to `t1` (`t0` < `t1`), the same kind of curve. A conic's
/// halves are split with their end weights made 1, which moves the parameter along the half past
/// `t0`: there `t1` lies at `v / (v + √w₀ (1 − v))`, where `v` is where it lay before the weights
/// were made 1 and `w₀ = (1 − t0)² + 2 t0 (1 − t0) w + t0²` is the conic's weight at `t0`.
fn part(g: &Seg2, t0: f64, t1: f64) -> Seg2 {
    // `split_at` splits no nearer an end than 1e-9.
    let t0 = if t0 < 1e-9 { 0.0 } else { t0.min(1.0 - 1e-9) };
    let right = if t0 > 0.0 { g.split_at(t0).1 } else { *g };
    let v = (t1 - t0) / (1.0 - t0);
    let v = match *g {
        Seg2::Conic { w, .. } if t0 > 0.0 => {
            let w0 = (1.0 - t0) * (1.0 - t0) + 2.0 * t0 * (1.0 - t0) * w + t0 * t0;
            v / (v + w0.max(0.0).sqrt() * (1.0 - v))
        }
        _ => v,
    };
    if v < 1.0 - 1e-12 { right.split_at(v).0 } else { right }
}

/// `piece` with its ends put exactly on `p` and `q`, where they lie within [`AS_DRAWN`] of them:
/// the result's corners, which its neighbours start and end on. An arc keeps its centre and
/// radius, and so its ends as they are.
fn ending_at(piece: Seg2, p: Vec2, q: Vec2) -> Seg2 {
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
/// cuts), with each piece of a zone sorted by what still holds it (see [`sort_zones`]), and their
/// longest arcs halved (see [`halved`]). The flat pattern and the folded solid are both made of
/// them, edge for edge.
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
            let trims = stage(&format!("joining panel {i}'s trims"), planar_union(trims, work))?;
            let left = stage(&format!("trimming panel {i}"), boolean_past_holes(&[placed], &trims, BoolOp::Cut, work))?;
            pieces.extend(left.into_iter().map(|r| (Piece::Panel(i), r)));
        }
    }
    pieces.extend(dev.zones.iter().enumerate().map(|(z, zone)| (Piece::Zone(z), on_grid(&zone.region))));
    if dev.cuts.is_empty() {
        return Ok(halved(pieces));
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
    Ok(halved(sort_zones(dev, cut_from(out, &late, work)?)?))
}

/// The pieces with each arc of more than three quarters of a turn (a hole's whole circle) halved
/// at the middle of its own sweep: two arcs of its circle that meet where the one did. Until the
/// kernel (solidcore, #1597) takes a circular edge round any sweep, and a whole circle on one
/// vertex, the folded solid's wall along such an arc is two faces, one on each half (see
/// `sheet_surface_fold::cells`, which takes these pieces' arcs as they are). The flat keeps the
/// same two edges: an edge of the flat is an edge of the folded sheet.
fn halved(pieces: Vec<(Piece, Region2)>) -> Vec<(Piece, Region2)> {
    let halves = |l: Loop2| Loop2 {
        segs: l
            .segs
            .into_iter()
            .flat_map(|e| match e {
                Seg2::Arc { center, radius, start, sweep } if sweep.abs() > std::f64::consts::PI * 1.5 => {
                    let half = sweep / 2.0;
                    vec![Seg2::Arc { center, radius, start, sweep: half }, Seg2::Arc { center, radius, start: start + half, sweep: half }]
                }
                e => vec![e],
            })
            .collect(),
    };
    pieces.into_iter().map(|(piece, r)| (piece, Region2 { outer: halves(r.outer), holes: r.holes.into_iter().map(halves).collect() })).collect()
}

/// `pieces` less `cuts`, joined first: what each piece leaves, each a piece of the same kind.
fn cut_from(pieces: Vec<(Piece, Region2)>, cuts: &[&Region2], work: &mut FlatWork) -> Result<Vec<(Piece, Region2)>> {
    if cuts.is_empty() {
        return Ok(pieces);
    }
    let cuts = stage("joining what is cut from the flat", planar_union(cuts.iter().map(|c| (*c).clone()).collect(), work))?;
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
/// panel. A piece that runs along neither is refused. A zone left with nothing that bends (cuts
/// part the sheet there) leaves its panels as pieces of their own, each where the bend held it.
fn sort_zones(dev: &Development, pieces: Vec<(Piece, Region2)>) -> Result<Vec<(Piece, Region2)>> {
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
        out.push((sorted, region));
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
    // The holes clear of every other piece, and of their own piece's outline and other holes
    // (see [`holes_apart`]), stay out of the union, and go back after it. A hole that touches its
    // piece's outline (a cut-out with a corner on the sheet's edge, which the planar booleans
    // read so) is the outline there: it goes through the union with it.
    let reach: Vec<Aabb2> = regions.iter().map(|r| loop_box(&r.outer)).collect();
    let mut carried: Vec<Loop2> = Vec::new();
    let mut bare = Vec::with_capacity(regions.len());
    for (i, r) in regions.into_iter().enumerate() {
        let others: Vec<Aabb2> = reach.iter().enumerate().filter(|(k, _)| *k != i).map(|(_, b)| *b).collect();
        let apart = holes_apart(&r);
        let mut near = Vec::new();
        for (h, apart) in r.holes.into_iter().zip(apart) {
            if apart && boxes_clear(&loop_box(&h), &others) {
                carried.push(h);
            } else {
                near.push(h);
            }
        }
        bare.push(Region2 { outer: r.outer, holes: near });
    }
    let joined = stage("joining the sheet's pieces", planar_union(bare, work))?;
    stage("joining the sheet's pieces", put_back(joined, carried))
}

/// A loop's box, out by [`CLEAR`]: the boxes that hold its segments (see [`seg_box`]).
fn loop_box(l: &Loop2) -> Aabb2 {
    l.segs.iter().map(seg_box).fold(Aabb2::EMPTY, |b, (min, max)| b.union(&Aabb2 { min, max })).inflated(CLEAR)
}

/// Whether box `b` meets none of `of`.
fn boxes_clear(b: &Aabb2, of: &[Aabb2]) -> bool {
    of.iter().all(|o| b.max.x < o.min.x || o.max.x < b.min.x || b.max.y < o.min.y || o.max.y < b.min.y)
}

/// Whether loops `h` and `l`, each with its box (see [`loop_box`]), keep farther apart than
/// [`CLEAR`]: their boxes apart, or no segment of one that near a segment of the other, read
/// exactly (see [`near_each_other`]). A pair of curves no exact reading is known for is taken as
/// near: its loop goes through the planar booleans, which read it exactly.
fn loops_apart((h, hb): (&Loop2, Aabb2), (l, lb): (&Loop2, Aabb2)) -> bool {
    boxes_clear(&hb, std::slice::from_ref(&lb)) || h.segs.iter().all(|s| l.segs.iter().all(|e| matches!(near_each_other(s, e, CLEAR), Ok(false))))
}

/// For each hole of `region`, whether it keeps clear of the region's outline and its other holes
/// (see [`loops_apart`]). A hole that touches one of them, at a point or along an edge, bounds the
/// region's material with it: the planar booleans read a cut-out with a corner on the outline as
/// a hole that meets the outline at that corner. Such a hole is not a hole of the region as it
/// is: a boolean of the region takes it with the outline, never past it (see [`put_back`]).
fn holes_apart(region: &Region2) -> Vec<bool> {
    let loops: Vec<(&Loop2, Aabb2)> = std::iter::once(&region.outer).chain(&region.holes).map(|l| (l, loop_box(l))).collect();
    loops.iter().enumerate().skip(1).map(|(k, h)| loops.iter().enumerate().all(|(j, l)| j == k || loops_apart(*h, *l))).collect()
}

/// The holes `carried` back in the regions of `out` that hold them: each kept clear of every
/// other loop (see [`holes_apart`] and [`loop_clear_of`]), so that the region holding a point of
/// it holds it all, read exactly (see [`holds`]).
fn put_back(mut out: Vec<Region2>, carried: Vec<Loop2>) -> Result<Vec<Region2>> {
    for h in carried {
        let Some(at) = h.segs.first().map(Seg2::start) else { continue };
        let r = out
            .iter_mut()
            .find(|r| holds(&r.outer, at) && !r.holes.iter().any(|k| holds(k, at)))
            .ok_or_else(|| DocError::Invalid("a cut-out of the flat lost its piece".into()))?;
        r.holes.push(h);
    }
    Ok(out)
}

/// Whether loop `h` keeps clear of region `t`: apart from it by their boxes, or farther than
/// [`CLEAR`] from each of its loops (see [`loops_apart`]), outside its material and holding none
/// of it. Apart from every loop of `t`, `h` lies all inside or all outside each, and each of them
/// all inside or all outside `h`: one point of each answers, exactly (see [`holds`]).
fn loop_clear_of(h: &Loop2, t: &Region2) -> bool {
    let hb = loop_box(h);
    if boxes_clear(&hb, std::slice::from_ref(&loop_box(&t.outer))) {
        return true;
    }
    let loops: Vec<&Loop2> = std::iter::once(&t.outer).chain(&t.holes).collect();
    if !loops.iter().all(|l| loops_apart((h, hb), (*l, loop_box(l)))) {
        return false;
    }
    let Some(p) = h.segs.first().map(Seg2::start) else { return false };
    let within = holds(&t.outer, p) && !t.holes.iter().any(|k| holds(k, p));
    !within && !loops.iter().any(|l| l.segs.first().is_some_and(|s| holds(h, s.start())))
}

/// `op` of `a` with `b` ([`planar_boolean`]), the holes of `a` clear of every region of `b`
/// (see [`loop_clear_of`]) and of their own region's other loops (see [`holes_apart`]) left out
/// of it and put back in the region of the result that holds them.
pub(crate) fn boolean_past_holes(a: &[Region2], b: &[Region2], op: BoolOp, work: &mut FlatWork) -> Result<Vec<Region2>> {
    let clear_of_tools = |h: &Loop2| b.iter().all(|t| loop_clear_of(h, t));
    let mut carried: Vec<Loop2> = Vec::new();
    let bare: Vec<Region2> = a
        .iter()
        .map(|r| {
            let mut near = Vec::new();
            for (h, apart) in r.holes.iter().zip(holes_apart(r)) {
                if apart && clear_of_tools(h) {
                    carried.push(h.clone());
                } else {
                    near.push(h.clone());
                }
            }
            Region2 { outer: r.outer.clone(), holes: near }
        })
        .collect();
    if carried.is_empty() {
        return planar_boolean(a, b, op, work);
    }
    put_back(planar_boolean(&bare, b, op, work)?, carried)
}

/// How far apart (mm) the planar booleans let a loop's segments end and still close it: the
/// kernel's own reading of a loop, which moves nothing.
const CLOSE: f64 = 1e-7;

/// `op` of `a` with `b` by the kernel's planar booleans, as the regions are: every edge of the
/// result is a piece of an edge of `a` or `b`, however short, its curves as drawn
/// ([`boolean_as_drawn`]), and no corner moves. (The flat
/// model's `material_boolean` first welds each gap of up to 50 nm between a loop's segments,
/// which moved a corner a hair from a bend's tangent line.) The work is charged to `work`, which
/// bounds the flat's.
pub(crate) fn planar_boolean(a: &[Region2], b: &[Region2], op: BoolOp, work: &mut FlatWork) -> Result<Vec<Region2>> {
    let edges =
        |rs: &[Region2]| rs.iter().flat_map(|r| std::iter::once(&r.outer).chain(&r.holes)).fold(0usize, |n, l| n.saturating_add(l.segs.len()));
    let n = edges(a).saturating_add(edges(b));
    work.charge(n.saturating_mul(n).saturating_mul(16))?;
    Ok(boolean_as_drawn(a, b, op, CLOSE)?)
}

/// The union of `regions` ([`planar_boolean`]), in balanced rounds: many pieces side by side do
/// not each meet a growing union one at a time.
pub(crate) fn planar_union(regions: Vec<Region2>, work: &mut FlatWork) -> Result<Vec<Region2>> {
    let mut groups: Vec<Vec<Region2>> = regions.into_iter().map(|r| vec![r]).collect();
    while groups.len() > 1 {
        let mut next = Vec::with_capacity(groups.len().div_ceil(2));
        let mut iter = groups.into_iter();
        while let Some(a) = iter.next() {
            match iter.next() {
                Some(b) => next.push(planar_boolean(&a, &b, BoolOp::Union, work)?),
                None => next.push(a),
            }
        }
        groups = next;
    }
    Ok(groups.pop().unwrap_or_default())
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

    /// Two arcs of input circles that a loop's corner joins, each read a hair past or short of it,
    /// both end where their circles meet there, each by its own angle, and keep their other ends.
    #[test]
    fn two_arcs_at_a_corner_end_where_their_circles_meet() {
        use std::f64::consts::PI;

        use solidcore_record::{Seg2, Vec2};

        use super::Edge;
        let arc = |x: f64, y: f64, radius: f64, start: f64, sweep: f64| Seg2::Arc { center: Vec2::new(x, y), radius, start, sweep };
        // An arc the boolean read, as the piece of input arc `of` it runs along.
        let piece = |of: Seg2, read: Seg2| Edge::Piece { of, t0: of.closest_param(read.start()), t1: of.closest_param(read.end()), read };
        let closed = |(a, of_a): (Seg2, Seg2), (b, of_b): (Seg2, Seg2), at: Vec2| {
            assert!(a.end().dist(b.start()) > 1e-6, "{a:?} {b:?}");
            let segs = super::closed_up(vec![piece(of_a, a), piece(of_b, b), Edge::Read(Seg2::Line { a: b.end(), b: a.start() })]);
            assert!(matches!(segs[..], [Seg2::Arc { .. }, Seg2::Arc { .. }, Seg2::Line { .. }]), "{segs:?}");
            assert!(segs[0].end().dist(at) < 1e-12 && segs[1].start().dist(at) < 1e-12, "{segs:?}");
            assert!(segs[0].start().dist(a.start()) < 1e-12 && segs[1].end().dist(b.end()) < 1e-12, "{segs:?}");
            assert!((0..3).all(|i| segs[i].end().dist(segs[(i + 1) % 3].start()) < 1e-12), "{segs:?}");
        };
        // Square across each other: up the circle about the origin to (0, 10), then on along the
        // one about (10, 10), which crosses it there.
        closed(
            (arc(0.0, 0.0, 10.0, 0.0, FRAC_PI_2 + 2e-7), arc(0.0, 0.0, 10.0, -0.5, 3.0)),
            (arc(10.0, 10.0, 10.0, PI + 1e-7, -FRAC_PI_2 - 1e-7), arc(10.0, 10.0, 10.0, PI + 0.5, -2.5)),
            Vec2::new(0.0, 10.0),
        );
        // Smoothly into each other: along the circle about the origin to (0, 10), then along the
        // one about (0, 20), which touches it there.
        closed(
            (arc(0.0, 0.0, 10.0, PI, -FRAC_PI_2 - 1e-7), arc(0.0, 0.0, 10.0, PI + 0.5, -2.5)),
            (arc(0.0, 20.0, 10.0, -FRAC_PI_2 - 2e-7, FRAC_PI_2 + 2e-7), arc(0.0, 20.0, 10.0, -PI, 3.5)),
            Vec2::new(0.0, 10.0),
        );
        // Two pieces of one circle: they meet on it nearest the corner as read.
        let (first, second, circle) = (arc(5.0, -3.0, 4.0, 0.0, 1.0 + 4e-7), arc(5.0, -3.0, 4.0, 1.0 - 3e-7, 1.0), arc(5.0, -3.0, 4.0, -1.0, 4.0));
        let near = first.end().lerp(second.start(), 0.5);
        closed((first, circle), (second, circle), Vec2::new(5.0, -3.0) + (near - Vec2::new(5.0, -3.0)).normalized().unwrap_or(Vec2::X) * 4.0);
    }

    /// A corner beside a piece of an input curve is put where the curve meets its neighbour's
    /// curve, so a line stays on its line; a corner between two edges as the boolean read them
    /// stays as read, with its step.
    #[test]
    fn a_corner_beside_a_piece_keeps_its_line_and_the_kernels_corners_stay() {
        use solidcore_record::{Seg2, Vec2};

        use super::Edge;
        let v = Vec2::new;
        // A quarter round from (10, 0) to (0, 10), drawn as one cubic, read with each end a hair off
        // its corner; the lines it meets read a hair past it, and short of each other.
        let drawn = Seg2::Cubic { p0: v(10.0, 0.0), p1: v(10.0, 5.5), p2: v(5.5, 10.0), p3: v(0.0, 10.0) };
        let read = Seg2::Cubic { p0: v(10.0, 4e-7), p1: v(10.0, 5.5), p2: v(5.5, 10.0), p3: v(-2e-7, 10.0) };
        let (bottom, left) = (Seg2::Line { a: v(0.0, 0.0), b: v(10.0 + 3e-7, 0.0) }, Seg2::Line { a: v(0.0, 10.0), b: v(0.0, 1e-7) });
        let segs = super::closed_up(vec![Edge::Read(bottom), Edge::Piece { of: drawn, t0: 0.0, t1: 1.0, read }, Edge::Read(left)]);
        let [Seg2::Line { a: a0, b: b0 }, curve @ Seg2::Cubic { .. }, Seg2::Line { a: a2, b: b2 }] = segs[..] else { panic!("{segs:?}") };
        // The bottom line ends on its own line where the curve starts.
        assert!(a0 == v(0.0, 0.0) && b0.y == 0.0 && (b0.x - 10.0).abs() < 1e-12, "{segs:?}");
        assert!(b0 == curve.start() && curve.end() == a2, "{segs:?}");
        assert!((a2.x == 0.0) && (a2.y - 10.0).abs() < 1e-12, "{segs:?}");
        assert!([0.0, 0.3, 0.7, 1.0].iter().all(|t| curve.point_at(*t).dist(drawn.point_at(*t)) < 1e-12), "{curve:?}");
        // The kernel's own corner, where the left line ends short of the bottom one, is left open.
        assert!(b2 == v(0.0, 1e-7) && a0 != b2, "{segs:?}");
    }

    /// An arc piece whose neighbour's line passes it by, more than rounding, has no corner to end
    /// on: the boolean's arc is kept as read.
    #[test]
    fn an_arc_piece_its_neighbour_passes_by_is_kept_as_read() {
        use std::f64::consts::PI;

        use solidcore_record::{Seg2, Vec2};

        use super::Edge;
        let v = Vec2::new;
        // A slot's round end, read on a circle a nanometre larger than the drawn one, between two
        // straight sides; the lower side runs 1e-8 mm below the drawn circle.
        let read = Seg2::Arc { center: v(0.0, 0.0), radius: 5.0 + 1e-9, start: -FRAC_PI_2, sweep: PI };
        let drawn = Seg2::Arc { center: v(0.0, 0.0), radius: 5.0, start: -FRAC_PI_2 - 0.5, sweep: PI + 1.0 };
        let (lower, upper) = (Seg2::Line { a: v(-10.0, -5.0 - 1e-8), b: v(0.0, -5.0 - 1e-8) }, Seg2::Line { a: read.end(), b: v(-10.0, 5.0) });
        let piece = Edge::Piece { of: drawn, t0: drawn.closest_param(read.start()), t1: drawn.closest_param(read.end()), read };
        let back = Seg2::Line { a: v(-10.0, 5.0), b: v(-10.0, -5.0 - 1e-8) };
        let segs = super::closed_up(vec![Edge::Read(lower), piece, Edge::Read(upper), Edge::Read(back)]);
        assert_eq!(segs, vec![lower, read, upper, back]);
    }

    /// A piece of a conic whose middle weight isn't 1 (a quarter of an ellipse), cut at two points
    /// inside it, stays on the ellipse: splitting a conic moves the parameter along its second half.
    #[test]
    fn a_strip_across_an_ellipse_keeps_it_an_ellipse() {
        use std::f64::consts::FRAC_1_SQRT_2;

        use solidcore_record::{Loop2, Region2, Seg2, Vec2};
        use solidcraft_kernel::BoolOp;
        let v = Vec2::new;
        // The 15 x 8 ellipse about the origin, in four quarters, and a strip from x = 5 to 10, which
        // crosses its upper and its lower right quarter twice each.
        let quarter = |a: Vec2, apex: Vec2, b: Vec2| Seg2::Conic { a, apex, b, w: FRAC_1_SQRT_2 };
        let ellipse = Region2 {
            outer: Loop2 {
                segs: vec![
                    quarter(v(15.0, 0.0), v(15.0, 8.0), v(0.0, 8.0)),
                    quarter(v(0.0, 8.0), v(-15.0, 8.0), v(-15.0, 0.0)),
                    quarter(v(-15.0, 0.0), v(-15.0, -8.0), v(0.0, -8.0)),
                    quarter(v(0.0, -8.0), v(15.0, -8.0), v(15.0, 0.0)),
                ],
            },
            holes: Vec::new(),
        };
        let strip = Region2 { outer: Loop2::polygon(&[v(5.0, -10.0), v(10.0, -10.0), v(10.0, 10.0), v(5.0, 10.0)]).ccw(), holes: Vec::new() };
        let cut = match super::boolean_as_drawn(std::slice::from_ref(&ellipse), std::slice::from_ref(&strip), BoolOp::Intersect, 1e-7) {
            Ok(c) => c,
            Err(e) => panic!("{e}"),
        };
        let [one] = &cut[..] else { panic!("{cut:?}") };
        let segs = &one.outer.segs;
        let on_ellipse = |p: Vec2| (p.x * p.x / 225.0 + p.y * p.y / 64.0 - 1.0).abs() < 1e-9;
        let conics: Vec<&Seg2> = segs.iter().filter(|s| matches!(s, Seg2::Conic { .. })).collect();
        assert_eq!(conics.len(), 2, "{segs:?}");
        for c in conics {
            assert!([0.0, 0.25, 0.5, 0.75, 1.0].iter().all(|t| on_ellipse(c.point_at(*t))), "{c:?}");
        }
        // The strip's sides keep to x = 5 and x = 10, and every corner is one point.
        for s in segs.iter().filter(|s| matches!(s, Seg2::Line { .. })) {
            let (a, b) = (s.start(), s.end());
            assert!([5.0, 10.0].iter().any(|x| (a.x - x).abs() < 1e-12 && (b.x - x).abs() < 1e-12), "{s:?}");
        }
        let n = segs.len();
        assert!((0..n).all(|i| segs[i].end() == segs[(i + 1) % n].start()), "{segs:?}");
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
