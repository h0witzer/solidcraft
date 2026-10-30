//! Convert to Sheet Metal through the skin (#6748): a body's faces as a skin of panels, rolls
//! among them, laid out flat by `solidcraft-develop`. No face is fixed: every face of the sheet
//! is a panel like any other, and the flat is drawn in the largest one's plane.
//!
//! The sheet is every face of a solid, or of one side of a shell or a plate (the side its largest
//! face is on, reached across every edge but a rim's), less the faces the user skips. A cylinder
//! or a cone is a roll that keeps its modeled radius, joined by seams where the sheet runs on
//! smoothly. Every sharp edge between two faces of the sheet is cut open, half the rule's gap
//! from the plane halfway between the two faces as `sheet.convert` cuts it, unless the user picks
//! it as a bend. A ring of bends and seams is refused, naming its edges. A solid's material lies
//! inside its faces, or outside them; a shell or a plate keeps its own.
//!
//! No panel comes first, and nothing depends on which way a joint runs. Two bends standing on one
//! panel and meeting at a corner of it end where their tangent lines on it cross, with the
//! rule's corner relief: a notch; a round or a square relief, which belongs to the corner and is
//! cut from every piece of the flat it reaches; or a tear, the bends' ends running tight against
//! the plane of the rip that parts their other panels. A bend that a rip cuts beside ends where
//! its metal reaches the rip's plane, and a panel that runs on past a bend's end gets the rule's
//! bend relief there. A wall standing on part of another panel's edge bends only where it
//! stands.
//!
//! Not converted yet: a free-form face (the kernel does not hand over its spline yet), a sphere
//! or a torus (neither is developable), a cylinder or cone that closes round its axis, a bend on
//! the edge of a roll, and a shell's face that meets the sheet at a sharp edge but is not a plane
//! (it is left out).

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use solidcraft_develop::{
    Anchor, Bend, Corner, Datum, Development, Fold, Joint, JointKind, Lap, Panel, Relief, Rigid3, Rip, Skin, Surface, Tail, ZoneEnds, develop, fold,
};
use solidcraft_geom::{Loop2, Mesh, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_kernel::{Body, FaceSurface, face_surface};

use crate::sheet::FlatWork;

use crate::sheet::{ReliefShape, outline_loop, round_wall};
use crate::sheet_skin::Piece;
use crate::{DocError, ModelBody, Result};

/// Normals closer than this (radians) meet smoothly: the sheet runs on across their edge.
const SMOOTH: f64 = 1e-6;

/// The rule's values Convert works with.
#[derive(Clone, Copy, Debug)]
pub struct SkinRule {
    pub thickness: f64,
    pub k_factor: f64,
    pub bend_radius: f64,
    /// What a rip leaves between the panels it cuts apart.
    pub gap: f64,
    /// What is cut where two bends on one base meet at a corner of it, and how big.
    pub corner_relief: ReliefShape,
    pub corner_relief_size: f64,
    /// The bend relief where a bend ends beside more of its base: how wide along the bend, and
    /// how far inside the base's tangent line it is cut from.
    pub relief_width: f64,
    pub relief_depth: f64,
}

/// A rip whose edge leaves a bend's end this near the bend's own way (the cosine between them)
/// carries on along the bend: the mesh gives an edge's way to about a thousandth.
const ALONGSIDE: f64 = 1.0 - 1e-4;

/// Relief sizes below this (mm) are not cut, as on a flanged sheet.
const MIN_RELIEF: f64 = 0.01;

/// A body converted through its skin: the skin and its development, the body's face each panel
/// is and where that face puts the panel's frame in space, a point on the edge of each joint,
/// the sheet's thickness (a shell's own), and why any relief the rule asks for was not cut.
#[derive(Clone, Debug)]
pub struct SkinConversion {
    pub skin: Skin,
    pub development: Development,
    pub faces: Vec<usize>,
    pub places: Vec<Rigid3>,
    pub edges: Vec<Vec3>,
    pub thickness: f64,
    pub notes: Vec<String>,
    /// The flat's pieces, cut once for both the flat and the folded solid.
    pieces: std::sync::OnceLock<Vec<(Piece, Region2)>>,
}

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(DocError::Invalid(msg.into()))
}

fn at(p: Vec3) -> String {
    format!("({:.3}, {:.3}, {:.3})", p.x, p.y, p.z)
}

/// What a face of the body is as a panel, and how its points map to the surface's parameters,
/// which run with `u x v` toward the material (opposite the face's outward normal).
#[derive(Clone, Copy, Debug)]
enum Shape {
    Plane { plane: Plane, out: Vec3 },
    Cylinder { origin: Vec3, axis: Vec3, e1: Vec3, e2: Vec3, radius: f64, inward: bool },
    Cone { apex: Vec3, axis: Vec3, e1: Vec3, e2: Vec3, half: f64, inward: bool },
}

impl Shape {
    /// Face `f`'s shape. A cylinder's or cone's material lies toward its axis when the face's
    /// outward normal (from the mesh's winding) points away from it; `flip` puts the material
    /// outside the body instead.
    fn of(b: &Body, mesh: &Mesh, f: usize, flip: bool) -> std::result::Result<Shape, &'static str> {
        let frame = |axis: Vec3| {
            let pick = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
            let e1 = (pick - axis * pick.dot(axis)).normalized().unwrap_or(Vec3::X);
            (e1, axis.cross(e1))
        };
        let outward = |from_axis: &dyn Fn(Vec3) -> Vec3| -> bool {
            let area = |t: &[Vec3; 3]| (t[1] - t[0]).cross(t[2] - t[0]);
            let tri = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, g)| **g as usize == f).filter_map(|(t, _)| mesh.tri(t));
            tri.max_by(|x, y| area(x).len().total_cmp(&area(y).len()))
                .is_some_and(|t| area(&t).dot(from_axis((t[0] + t[1] + t[2]) * (1.0 / 3.0))) > 0.0)
        };
        match face_surface(b, f) {
            Some(FaceSurface::Plane { normal, point }) => {
                let out = if flip { -*normal } else { *normal };
                Plane::from_normal(point, -out).map(|plane| Shape::Plane { plane, out }).ok_or("a face of the body has no plane")
            }
            Some(FaceSurface::Cylinder { origin, axis, radius }) => {
                let axis = *axis;
                let (e1, e2) = frame(axis);
                let inward = outward(&|p| (p - origin) - axis * (p - origin).dot(axis)) != flip;
                Ok(Shape::Cylinder { origin, axis, e1, e2, radius, inward })
            }
            Some(FaceSurface::Cone { apex, axis, half_angle }) => {
                let axis = *axis;
                let (e1, e2) = frame(axis);
                let inward = outward(&|p| (p - apex) - axis * (p - apex).dot(axis)) != flip;
                Ok(Shape::Cone { apex, axis, e1, e2, half: half_angle, inward })
            }
            Some(FaceSurface::Sphere { .. } | FaceSurface::Torus { .. }) => Err("a sphere or a torus is not developable"),
            None => Err("not supported yet: a free-form face (the kernel does not hand over its spline yet)"),
        }
    }

    fn surface(&self) -> Surface {
        match *self {
            Shape::Plane { .. } => Surface::Plane,
            // The skin's radius is the face's; with the material outward the face is the inside.
            Shape::Cylinder { radius, inward, .. } => Surface::Cylinder { radius, material_inward: inward },
            Shape::Cone { half, inward, .. } => Surface::Cone { half_angle: half, material_inward: inward },
        }
    }

    /// The parameters of a point of the face, its angle not yet unwrapped.
    fn params(&self, p: Vec3) -> Vec2 {
        let turn = |w: Vec3, e1: Vec3, e2: Vec3, inward: bool| w.dot(e2).atan2(w.dot(e1)) * if inward { 1.0 } else { -1.0 };
        match *self {
            Shape::Plane { plane, .. } => plane.to_local(p),
            Shape::Cylinder { origin, axis, e1, e2, inward, .. } => Vec2::new((p - origin).dot(axis), turn(p - origin, e1, e2, inward)),
            Shape::Cone { apex, e1, e2, inward, .. } => Vec2::new((p - apex).len(), turn(p - apex, e1, e2, inward)),
        }
    }

    fn angular(&self) -> bool {
        !matches!(self, Shape::Plane { .. })
    }

    /// Where the face puts its panel's frame (see `solidcraft_develop::fold`): a plane's
    /// parameters are its frame's x and y, toward the material its z; a cylinder's or a cone's
    /// axis is its frame's z, its angle measured from x.
    fn frame(&self) -> Option<Rigid3> {
        match *self {
            Shape::Plane { plane, .. } => Rigid3::frame(plane.origin, plane.x, plane.y),
            Shape::Cylinder { origin, e1, e2, .. } => Rigid3::frame(origin, e1, e2),
            Shape::Cone { apex, e1, e2, .. } => Rigid3::frame(apex, e1, e2),
        }
    }

    /// How far a point lies out from the face's surface along its outward normal (below 0 inside
    /// the material), near the surface.
    fn off(&self, p: Vec3) -> f64 {
        let side = |inward: bool| if inward { 1.0 } else { -1.0 };
        let radial = |o: Vec3, axis: Vec3| ((p - o) - axis * (p - o).dot(axis)).len();
        match *self {
            Shape::Plane { plane, .. } => -plane.height(p),
            Shape::Cylinder { origin, axis, radius, inward, .. } => (radial(origin, axis) - radius) * side(inward),
            Shape::Cone { apex, axis, half, inward, .. } => (radial(apex, axis) * half.cos() - (p - apex).dot(axis) * half.sin()) * side(inward),
        }
    }

    /// The outward unit normal at a point of the face.
    fn out(&self, p: Vec3) -> Vec3 {
        let side = |inward: bool| if inward { 1.0 } else { -1.0 };
        match *self {
            Shape::Plane { out, .. } => out,
            Shape::Cylinder { origin, axis, inward, .. } => {
                ((p - origin) - axis * (p - origin).dot(axis)).normalized().unwrap_or(axis) * side(inward)
            }
            Shape::Cone { apex, axis, half, inward, .. } => {
                let c = ((p - apex) - axis * (p - apex).dot(axis)).normalized().unwrap_or(axis);
                (c * half.cos() - axis * half.sin()) * side(inward)
            }
        }
    }
}

/// Until the kernel hands over a face's loops (#1597): read them from the B-rep, not chained
/// from the display mesh's edge polylines.
///
/// Face `f`'s boundary loops from the mesh's edges, chained head to tail: each point with the
/// face across the loop from it to the next.
fn face_loops(mesh: &Mesh, f: usize, tol: f64) -> Result<Vec<Vec<(Vec3, Option<usize>)>>> {
    let mut edges: Vec<(&[Vec3], Option<usize>)> = mesh
        .edges
        .iter()
        .zip(&mesh.edge_faces)
        .filter(|(line, faces)| line.len() >= 2 && faces.iter().any(|g| *g as usize == f))
        .map(|(line, faces)| (line.as_slice(), faces.iter().map(|g| *g as usize).find(|g| *g != f)))
        .collect();
    let mut loops = Vec::new();
    while let Some((line, across)) = edges.pop() {
        let (Some(&start), Some(&last)) = (line.first(), line.last()) else { continue };
        let mut lp: Vec<(Vec3, Option<usize>)> = line.iter().take(line.len().saturating_sub(1)).map(|p| (*p, across)).collect();
        let mut end = last;
        while end.dist(start) > tol {
            let meets = |p: Option<&Vec3>| p.is_some_and(|p| p.dist(end) <= tol);
            let Some(k) = edges.iter().position(|(l, _)| meets(l.first()) || meets(l.last())) else {
                return err("the outline of a face of the body does not close");
            };
            let (l, g) = edges.swap_remove(k);
            let run: Vec<Vec3> = if meets(l.first()) { l.to_vec() } else { l.iter().rev().copied().collect() };
            lp.extend(run.iter().take(run.len().saturating_sub(1)).map(|p| (*p, g)));
            end = run.last().copied().unwrap_or(start);
        }
        loops.push(lp);
    }
    Ok(loops)
}

/// A face's region in its surface's parameters. A plane's loops are read exactly, an arc where
/// one runs along a round wall standing square on it; a roll's are straight in its parameters
/// along its rulings and its circles, and kept as the mesh's points elsewhere.
fn region(b: &Body, shape: &Shape, loops: &[Vec<(Vec3, Option<usize>)>], tol: f64) -> Result<Region2> {
    let mut shaped: Vec<Loop2> = Vec::new();
    for lp in loops {
        let mut pts: Vec<Vec2> = lp.iter().map(|(p, _)| shape.params(*p)).collect();
        if shape.angular() {
            // The angle unwrapped along the loop; a loop that comes back a turn away closes
            // round the axis.
            for k in 1..pts.len() {
                let (Some(prev), Some(cur)) = (pts.get(k - 1).copied(), pts.get_mut(k)) else { continue };
                cur.y += ((prev.y - cur.y) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
            }
            if let (Some(first), Some(last)) = (pts.first(), pts.last())
                && (last.y - first.y).abs() > std::f64::consts::PI
            {
                return err("not supported yet: a cylinder or cone that closes round its axis (split it, or pick its seam as a rip)");
            }
        }
        shaped.push(match shape {
            Shape::Plane { plane, out } => {
                let tagged: Vec<(Vec2, Option<u32>)> = pts.iter().zip(lp).map(|(p, (_, g))| (*p, g.and_then(|g| u32::try_from(g).ok()))).collect();
                let mut wall = |g: u32| round_wall(b, plane, *out, g);
                outline_loop(&tagged, &mut wall, tol * 10.0)
            }
            _ => Loop2 { segs: straight_runs(&pts, tol) },
        });
    }
    let area = |l: &Loop2| l.signed_area().abs();
    shaped.sort_by(|x, y| area(y).total_cmp(&area(x)));
    let mut it = shaped.into_iter();
    let Some(outer) = it.next() else { return err("a face of the body has no edges") };
    let mut holes: Vec<Loop2> = it.collect();
    if shape.angular() {
        // A hole is unwrapped on its own: bring it to the turn the outline is on.
        let mean = |l: &Loop2| l.segs.iter().map(|s| s.start().y).sum::<f64>() / l.segs.len().max(1) as f64;
        let m = mean(&outer);
        for h in &mut holes {
            let shift = ((m - mean(h)) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
            if shift != 0.0 {
                let mv = solidcraft_develop::Rigid2 { cos: 1.0, sin: 0.0, shift: Vec2::new(0.0, shift) };
                *h = mv.loop2(h);
            }
        }
    }
    Ok(Region2 { outer: outer.ccw(), holes: holes.iter().map(|h| h.ccw().reversed()).collect() })
}

/// A closed polyline as lines, each run of points along one straight line joined into one.
fn straight_runs(pts: &[Vec2], tol: f64) -> Vec<Seg2> {
    let n = pts.len();
    let straight = |a: Vec2, p: Vec2, c: Vec2| (c - a).normalized().is_some_and(|d| (p - a).cross(d).abs() <= tol * 1e-3);
    // Corners: points where the outline turns.
    let corners: Vec<Vec2> = (0..n)
        .filter_map(|i| {
            let (a, p, c) = (pts.get((i + n - 1) % n)?, pts.get(i)?, pts.get((i + 1) % n)?);
            (!straight(*a, *p, *c)).then_some(*p)
        })
        .collect();
    let m = corners.len();
    (0..m).filter_map(|i| Some(Seg2::Line { a: *corners.get(i)?, b: *corners.get((i + 1) % m)? })).collect()
}

/// The normal of the plane halfway between panels `x` and `y` through the edge between them,
/// toward `x`'s side. `e` runs along the edge with `x` on its left seen from outside; normals are
/// outward.
fn rip_plane(nx: Vec3, ny: Vec3, e: Vec3) -> Option<Vec3> {
    let (wx, wy) = (nx.cross(e), e.cross(ny));
    (wx - wy).normalized().filter(|m| wx.dot(*m) > 1e-9)
}

/// A rip as the corners see it: its joint, its edge's two ends, and at each the plane halfway
/// between its faces there (normal toward its `a` face).
/// The edge's way at each end, along the edge from its start to its end, comes last.
type RipEnds = (usize, [Vec3; 2], [Option<Vec3>; 2], [Vec3; 2]);

/// The unit way an edge given as points runs at its `k`th point: across its neighbors, and at its
/// ends to second order, so that a curve's ends lean no more than its middle.
fn way3(points: &[Vec3], k: usize) -> Option<Vec3> {
    let at = |i: usize| points.get(i).copied();
    let n = points.len();
    let way = match (k, n) {
        (_, 0 | 1) => return None,
        (_, 2) => at(1)? - at(0)?,
        (0, _) => at(1)? * 4.0 - at(0)? * 3.0 - at(2)?,
        (k, n) if k + 1 == n => at(k)? * 3.0 - at(k - 1)? * 4.0 + at(k - 2)?,
        (k, _) => at(k + 1)? - at(k - 1)?,
    };
    way.normalized()
}

/// Points of an edge in a face's parameters, the angle round a roll unwrapped along the edge
/// from the side of the turn its region `near` is on.
fn chart_path(shape: &Shape, points: &[Vec3], near: &Region2) -> Vec<Vec2> {
    let mut out: Vec<Vec2> = Vec::with_capacity(points.len());
    for p in points {
        let mut q = shape.params(*p);
        if shape.angular() {
            let reference = out.last().map_or_else(|| middle_turn(near).unwrap_or(q.y), |prev| prev.y);
            q.y += ((reference - q.y) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
        }
        out.push(q);
    }
    out
}

/// Until the kernel hands over every edge's exact curve (#1597): this reads only edges lying in
/// a plane, found by their place in the display mesh's edge list, which must match the kernel's.
///
/// Edge `e` of the body exactly in a planar face's parameters (lines, circular arcs, free-form
/// spans), run the way its points `path` there run; empty on a roll, and where the body's edge
/// doesn't end where the points do.
fn exact_edge(b: &Body, shape: &Shape, e: usize, path: &[Vec2], tol: f64) -> Vec<Seg2> {
    let Shape::Plane { plane, .. } = shape else { return Vec::new() };
    let (Some(first), Some(last)) = (path.first(), path.last()) else { return Vec::new() };
    let Some(Some(curve)) = b.planar_edges(plane, &[e]).into_iter().next() else { return Vec::new() };
    let Some((p, q)) = curve.first().zip(curve.last()).map(|(p, q)| (p.start(), q.end())) else { return Vec::new() };
    if p.dist(*first) <= tol && q.dist(*last) <= tol {
        curve
    } else if q.dist(*first) <= tol && p.dist(*last) <= tol {
        curve.iter().rev().map(Seg2::reversed).collect()
    } else {
        Vec::new()
    }
}

/// The angle halfway across those a roll's panel spans. A point of the panel's edge unwrapped
/// to the turn nearest it lies on the panel's side, even at an end of a half turn, where the
/// panel's first corner is half a turn off and either turn is as near to that.
fn middle_turn(region: &Region2) -> Option<f64> {
    let turns = region.outer.segs.iter().flat_map(|s| [s.start().y, s.point_at(0.5).y]);
    let (lo, hi) = turns.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), y| (lo.min(y), hi.max(y)));
    (lo <= hi).then_some((lo + hi) / 2.0)
}

/// How far a bend is cut back square along its edge from the corner `v` so that none of its
/// metal comes nearer than half the gap to a rip's plane through `v` (normal `m`, toward the side
/// the bend keeps), as the rip cuts the panels beside it. `d` runs along the edge from `v`; `na`
/// and `nb` are the two panels' normals away from their material. The bend's metal is the ring
/// sector between radii `r` and `r + t` round its axis, which lies `r + t` inside both faces
/// where it turns toward the material (the faces are its outside) and `r` outside them where it
/// turns away (they are its inside). None where running along the edge leads no further from the
/// plane.
fn bend_cut(v: Vec3, d: Vec3, m: Vec3, normals: (Vec3, Vec3), toward: bool, r: f64, t: f64, gap: f64) -> Option<f64> {
    let away = d.dot(m);
    if away <= 1e-9 {
        return None;
    }
    let (axis, ua, across, theta) = bend_section(v, normals, toward, r, t)?;
    let (cu, cv) = (ua.dot(m), across.dot(m));
    let lowest = (-cv).atan2(-cu).rem_euclid(std::f64::consts::TAU);
    let turns = [0.0, theta, if lowest <= theta { lowest } else { 0.0 }];
    let nearest =
        turns.iter().flat_map(|a| [r, r + t].map(|radius| (axis - v).dot(m) + radius * (a.cos() * cu + a.sin() * cv))).fold(f64::INFINITY, f64::min);
    Some(((gap / 2.0 - nearest) / away).max(0.0))
}

/// A bend as Convert works out its ends: its joint, its edge's ends (its `a` face's material on
/// the left running from the first), its faces' normals `(a, b)` away from their material,
/// whether it turns toward the material, its edge's length, its anchors, and how far each anchor
/// lies from its tangent line.
struct BendEdge {
    joint: usize,
    ends: [Vec3; 2],
    normals: (Vec3, Vec3),
    toward: bool,
    len: f64,
    a: Anchor,
    b: Anchor,
    setback: f64,
}

impl BendEdge {
    /// The bend's anchor on panel `p`, and the side of it the panel's material lies on (1 on
    /// the left, the `a` anchor's; -1 on the right, the `b` one's).
    fn on(&self, p: usize) -> Option<(Anchor, f64)> {
        if self.a.panel == p {
            Some((self.a, 1.0))
        } else if self.b.panel == p {
            Some((self.b, -1.0))
        } else {
            None
        }
    }

    /// The bend's anchor on its panel other than `p`, and the side of it that panel's material
    /// lies on.
    fn other(&self, p: usize) -> Option<(Anchor, f64)> {
        self.on(if self.a.panel == p { self.b.panel } else { self.a.panel })
    }

    /// The bend's tangent line on its panel at anchor `at` (`side` as from [`BendEdge::on`]).
    fn tangent(&self, (at, side): (Anchor, f64)) -> (Vec2, Vec2) {
        (at.origin + at.dir.perp() * (side * self.setback), at.dir)
    }
}

/// Two bends meeting at a corner of a panel both stand on: each one's end there (the bend, and
/// 0 for its start or 1 for its end), the corner of their edges, the panel, and how far along
/// each bend their tangent lines on the panel cross. Neither panel round the corner comes first:
/// the bends end alike whichever way their joints run.
struct BendCorner {
    ends: [(usize, usize); 2],
    v: Vec3,
    panel: usize,
    at: [f64; 2],
}

/// Where a region's outline crosses a line (a point and a unit direction), as lengths along the
/// line from its point: the least and the greatest. `None` where it doesn't.
fn extent_across(r: &Region2, (p, u): (Vec2, Vec2)) -> Option<(f64, f64)> {
    let n = u.perp();
    let mut out: Option<(f64, f64)> = None;
    for l in std::iter::once(&r.outer).chain(&r.holes) {
        let pts: Vec<Vec2> = (&l.polyline(1e-4)).into_iter().copied().collect();
        for (k, a) in pts.iter().enumerate() {
            let Some(b) = pts.get((k + 1) % pts.len()) else { continue };
            let (sa, sb) = ((*a - p).dot(n), (*b - p).dot(n));
            if (sa > 0.0) != (sb > 0.0) {
                let along = (*a + (*b - *a) * (sa / (sa - sb)) - p).dot(u);
                out = Some(out.map_or((along, along), |(lo, hi)| (lo.min(along), hi.max(along))));
            }
        }
    }
    out
}

/// The point where two lines, each a point and a direction, cross.
fn crossing((p, u): (Vec2, Vec2), (q, w): (Vec2, Vec2)) -> Option<Vec2> {
    let cross = u.cross(w);
    (cross.abs() > 1e-12).then(|| p + u * ((q - p).cross(w) / cross))
}

/// The corners where two bends standing on one panel meet (see [`BendCorner`]).
fn bend_corners(bent: &[BendEdge], near: impl Fn(Vec3, Vec3) -> bool) -> Vec<BendCorner> {
    let ends = || bent.iter().enumerate().flat_map(|(i, b)| b.ends.iter().enumerate().map(move |(e, p)| (i, e, b, *p)));
    let shared = |x: &BendEdge, y: &BendEdge| [x.a.panel, x.b.panel].into_iter().find(|p| y.on(*p).is_some());
    let (mut out, mut used) = (Vec::new(), Vec::new());
    for (i, e, bi, v) in ends() {
        if used.contains(&(i, e)) {
            continue;
        }
        let other = ends().find(|(k, f, bk, p)| *k != i && !used.contains(&(*k, *f)) && near(*p, v) && shared(bi, bk).is_some());
        let Some((k, f, bk, _)) = other else { continue };
        let Some(panel) = shared(bi, bk) else { continue };
        let (Some(ai), Some(ak)) = (bi.on(panel), bk.on(panel)) else { continue };
        let into = |(at, _): (Anchor, f64), end: usize| if end == 0 { at.dir } else { -at.dir };
        let (di, dk) = (into(ai, e), into(ak, f));
        // A convex corner: each bend runs off into the panel's side of the other.
        if !(dk.dot(ai.0.dir.perp() * ai.1) > 1e-9 && di.dot(ak.0.dir.perp() * ak.1) > 1e-9) {
            continue;
        }
        let Some(c) = crossing(bi.tangent(ai), bk.tangent(ak)) else { continue };
        used.extend([(i, e), (k, f)]);
        out.push(BendCorner { ends: [(i, e), (k, f)], v, panel, at: [(c - ai.0.origin).dot(ai.0.dir), (c - ak.0.origin).dot(ak.0.dir)] });
    }
    out
}

/// A tear at corner `k`, where a rip parts the bends' other panels: each bend's ends there, on
/// its `a` and `b` sides, and the tear. On the panel the bends share, the bend ends where their
/// tangent lines cross; on its other panel, where that panel's material, cut back by the rip,
/// meets its tangent line; and between, it runs tight against the rip's plane. `None` at an open
/// corner, where no rip parts the two panels.
#[allow(clippy::too_many_arguments)]
fn tear(
    bent: &[BendEdge],
    joints: &[Joint],
    rips: &[RipEnds],
    places: &[Rigid3],
    k: &BendCorner,
    t: f64,
    near: impl Fn(Vec3, Vec3) -> bool,
) -> std::result::Result<Option<[(f64, f64, Relief); 2]>, String> {
    let [(i, _), (j, _)] = k.ends;
    let (Some(bi), Some(bj)) = (bent.get(i), bent.get(j)) else { return Ok(None) };
    let (Some(wi), Some(wj)) = (bi.other(k.panel), bj.other(k.panel)) else { return Ok(None) };
    let walls = [wi.0.panel, wj.0.panel];
    let rip = rips.iter().find_map(|(r, ends, planes, _)| {
        let jr = joints.get(*r)?;
        let pair = [jr.a.panel, jr.b.panel];
        let [wa, wb] = walls;
        let end = ends.iter().position(|p| near(*p, k.v))?;
        (pair == [wa, wb] || pair == [wb, wa]).then_some((jr, end, planes.get(end).copied().flatten()?))
    });
    let Some((joint, end, m)) = rip else { return Ok(None) };
    let JointKind::Rip(rip) = &joint.kind else { return Ok(None) };
    let at_end = |v: &[Vec2]| {
        if end == 0 { v.first().zip(v.get(1)).map(|(p, q)| (*p, *q - *p)) } else { v.last().zip(v.iter().rev().nth(1)).map(|(p, q)| (*p, *p - *q)) }
    };
    let turn = if end == 0 { rip.turns.first() } else { rip.turns.last() };
    let cut = rip.lap.cuts(turn.copied().unwrap_or_default(), t);
    let mut out = [(0.0, 0.0, Relief::None); 2];
    for ((slot, (b, wall)), corner_at) in out.iter_mut().zip([(bi, wi), (bj, wj)]).zip(k.at) {
        // The wall's edge at the corner, cut back into its material (on the left of the rip's
        // way on its `a` panel, on the right on its `b` one), and the bend's tangent line on the
        // wall.
        let (edge, back) = if joint.a.panel == wall.0.panel { (&rip.edges.0, cut.0) } else { (&rip.edges.1, -cut.1) };
        let Some((point, way)) = at_end(edge) else { return Err("a rip has no edge".into()) };
        let Some(way) = way.normalized() else { return Err("a rip's edge has no length".into()) };
        let Some(q) = crossing(b.tangent(wall), (point + way.perp() * back, way)) else {
            return Err("a bend runs along the rip".into());
        };
        let wall_at = (q - wall.0.origin).dot(wall.0.dir);
        // The rip's plane through the corner, toward the bend's own wall, in the bend's frame
        // at its `a` anchor.
        let own = if joint.a.panel == wall.0.panel { m } else { -m };
        let Some((normal, offset)) = anchor_plane(places, b.a, own, k.v) else { return Err("a bend's panel has no place".into()) };
        let (near_at, far_at) = if b.a.panel == k.panel { (corner_at, wall_at) } else { (wall_at, corner_at) };
        *slot = (near_at, far_at, Relief::Tear { normal, offset });
    }
    Ok(Some(out))
}

/// The plane through `v` with normal `m`, in a bend's frame at its `a` anchor `a` (`x` along the
/// joint, `y` toward the material, `z` toward the `b` panel): its normal there and its offset.
fn anchor_plane(places: &[Rigid3], a: Anchor, m: Vec3, v: Vec3) -> Option<(Vec3, f64)> {
    let place = places.get(a.panel)?;
    let (o, x, y) = (place.point(a.origin.extend(0.0)), place.vector(a.dir.extend(0.0)), place.z);
    let z = x.cross(y);
    Some((Vec3::new(m.dot(x), m.dot(y), m.dot(z)), (v - o).dot(m)))
}

/// A bend's section at the corner `v` of its edge: its axis there, the unit radius toward its
/// `a` face and the one square to it toward its `b` face, and how far it turns. `na` and `nb`
/// are the faces' normals away from their material. The bend's metal is the ring sector between
/// radii `r` and `r + t` round its axis, which lies `r + t` inside both faces where the bend
/// turns toward the material (the faces are its outside) and `r` outside them where it turns
/// away (they are its inside). None where the faces lie flat or fold onto each other.
fn bend_section(v: Vec3, (na, nb): (Vec3, Vec3), toward: bool, r: f64, t: f64) -> Option<(Vec3, Vec3, Vec3, f64)> {
    let cos = na.dot(nb).clamp(-1.0, 1.0);
    if 1.0 + cos <= 1e-9 {
        return None;
    }
    let depth = if toward { -(r + t) } else { r };
    let axis = v + (na + nb) * (depth / (1.0 + cos));
    let (ua, ub) = if toward { (na, nb) } else { (-na, -nb) };
    let across = (ub - ua * ub.dot(ua)).normalized()?;
    Some((axis, ua, across, cos.acos()))
}

/// What an edge between two faces of the sheet is.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Role {
    Seam,
    Bend,
    Rip,
}

/// Converts `mb` through its skin, less the faces at the `skip` points, bending at the edges at
/// the `bends` points and cutting every other sharp edge between two faces of the sheet open.
/// A solid's material lies inside its faces, or outside them when `outward`.
pub fn convert_skin(mb: &ModelBody, rule: &SkinRule, bends: &[Vec3], skip: &[Vec3], outward: bool) -> Result<SkinConversion> {
    // Until the kernel (solidcore, #1597) hands over each face's loops with each edge's exact
    // curve on any surface: everything below that reads `mesh` (the faces' outlines, the edges
    // between them, rays for the sheet's thickness, the faces' areas) reads the display
    // tessellation, and a rip along a curved edge off a plane follows its facets. Read the B-rep
    // instead and delete the mesh from this function.
    let (b, mesh) = (&mb.body, mb.mesh());
    let size = b.size();
    if !(size.is_finite() && size > 0.0) {
        return err("the body has no size");
    }
    let (tol, near) = ((size * 1e-6).max(1e-6), (size * 2e-3).max(1e-3));
    let count = mesh.tri_face.iter().map(|g| *g as usize + 1).max().unwrap_or(0);
    let area: Vec<f64> = (0..count).map(|f| face_area(&mesh, f)).collect();
    let area_of = |f: usize| area.get(f).copied().unwrap_or(0.0);
    let on_edge = |p: Vec3, line: &[Vec3]| line.windows(2).map(|w| p.dist_to_segment(w[0], w[1])).fold(f64::INFINITY, f64::min) <= near;
    let picked = |picks: &[Vec3], line: &[Vec3]| picks.iter().any(|p| on_edge(*p, line));
    if let Some(p) = bends.iter().find(|p| !mesh.edges.iter().any(|l| on_edge(**p, l))) {
        return err(format!("no edge of the body at the bend point {}", at(*p)));
    }
    let mut skipped = BTreeSet::new();
    for p in skip {
        let Some((_, g)) = crate::appearance::nearest_face(mb, *p).filter(|(d, _)| *d <= near) else {
            return err(format!("no face of the body at the skipped face point {}", at(*p)));
        };
        skipped.insert(g);
    }

    // The walk starts on the largest face, whatever its shape: nothing about the sheet depends
    // on where. A shell or a plate: a face facing back at it the same small distance behind it.
    // It keeps its thickness and its material; a solid takes the rule's.
    let modelled: Vec<Option<Shape>> = (0..count).map(|f| Shape::of(b, &mesh, f, false).ok()).collect();
    let probe = |f: usize| modelled.get(f).copied().flatten();
    let Some(start) = (0..count).filter(|f| probe(*f).is_some()).max_by(|x, y| area_of(*x).total_cmp(&area_of(*y))) else {
        return err("not supported yet: a body with no face that is a plane, a cylinder or a cone");
    };
    let sheet = behind(&mesh, &probe, start).filter(|d| *d > 0.0 && d * d * 16.0 <= area_of(start));
    let t = sheet.map_or(rule.thickness, |d| (d * 1e9).round() / 1e9);
    let shapes: Vec<std::result::Result<Shape, &'static str>> = (0..count).map(|f| Shape::of(b, &mesh, f, outward && sheet.is_none())).collect();
    let shape = |f: usize| shapes.get(f).copied().unwrap_or(Err("a face the mesh does not have"));
    let smooth = |line: &[Vec3], x: &Shape, y: &Shape| -> bool {
        let m = middle(line).unwrap_or_default();
        x.out(m).dot(y.out(m)).clamp(-1.0, 1.0).acos() <= SMOOTH
    };

    // The faces of the sheet: a solid's all, a shell's or a plate's side the start is on. That
    // side runs on smoothly, or across a sharp edge between two planes that is no rim: a face's
    // rims (its thickness faces) touch the face behind it. A wall's outside that runs on in one
    // plane with the base's edge touches it too, and is told from a rim by standing the sheet's
    // thickness in front of the face behind it. A face whose face behind is the sheet's is the
    // sheet's other side.
    let touches = |g: usize, h: usize| mesh.edge_faces.iter().any(|fs| fs.contains(&(g as u32)) && fs.contains(&(h as u32)));
    let thick = |g: usize| sheet.is_some_and(|d| behind(&mesh, &probe, g).is_some_and(|e| (e - d).abs() <= tol * 10.0));
    let mut behind_of: BTreeMap<usize, Option<usize>> = BTreeMap::new();
    let mut sheet_faces = BTreeSet::from([start]);
    let mut queue = VecDeque::from([start]);
    while let Some(f) = queue.pop_front() {
        for (line, faces) in mesh.edges.iter().zip(&mesh.edge_faces) {
            let (&[x, y], true) = (faces.as_slice(), line.len() >= 2) else { continue };
            let (x, y) = (x as usize, y as usize);
            let g = if x == f {
                y
            } else if y == f {
                x
            } else {
                continue;
            };
            if g == f || sheet_faces.contains(&g) {
                continue;
            }
            let crosses = sheet.is_none() || {
                let rim = behind_of.entry(f).or_insert_with(|| behind_face(&mesh, f)).is_some_and(|h| touches(g, h)) && !thick(g);
                let other_side = behind_of.entry(g).or_insert_with(|| behind_face(&mesh, g)).is_some_and(|h| sheet_faces.contains(&h));
                match (shape(f), shape(g)) {
                    (Ok(sf), Ok(sg)) => {
                        !rim && !other_side && (smooth(line, &sf, &sg) || matches!((sf, sg), (Shape::Plane { .. }, Shape::Plane { .. })))
                    }
                    _ => false,
                }
            };
            if crosses {
                sheet_faces.insert(g);
                queue.push_back(g);
            }
        }
    }
    sheet_faces.retain(|f| !skipped.contains(f));
    let mut shaped: BTreeMap<usize, Shape> = BTreeMap::new();
    for &f in &sheet_faces {
        match shape(f) {
            Ok(s) => shaped.insert(f, s),
            Err(why) => return err(format!("the face at {} can't be a panel: {why}. Skip it to leave it out", at(face_point(&mesh, f)))),
        };
    }

    // Each edge between two faces of the sheet: a seam where it runs on smoothly, a bend where
    // the user picked it, and cut open everywhere else.
    let mut roles: Vec<(usize, usize, usize, Role)> = Vec::new();
    for (e, (line, faces)) in mesh.edges.iter().zip(&mesh.edge_faces).enumerate() {
        let (&[x, y], true) = (faces.as_slice(), line.len() >= 2) else { continue };
        let (x, y) = (x as usize, y as usize);
        let (Some(sx), Some(sy)) = (shaped.get(&x), shaped.get(&y)) else { continue };
        if x == y {
            continue;
        }
        let role = if smooth(line, sx, sy) {
            Role::Seam
        } else if picked(bends, line) {
            Role::Bend
        } else {
            Role::Rip
        };
        roles.push((e, x, y, role));
    }
    let between = |p: &Vec3| roles.iter().any(|(e, ..)| mesh.edges.get(*e).is_some_and(|l| on_edge(*p, l)));
    if let Some(p) = bends.iter().find(|p| !between(p)) {
        return err(format!("the bend at {} is not between two faces of the sheet", at(*p)));
    }

    // The panels, each piece from its largest face, whatever its shape, across its bends and
    // seams; the first piece's start is the flat's frame.
    let largest = |left: &BTreeSet<usize>| left.iter().copied().max_by(|x, y| area_of(*x).total_cmp(&area_of(*y)).then(y.cmp(x)));
    let mut left = sheet_faces.clone();
    let mut parent: BTreeMap<usize, usize> = BTreeMap::new();
    // The edges the walk crossed, each to a face it had not reached.
    let mut crossed: BTreeSet<usize> = BTreeSet::new();
    let mut order = Vec::with_capacity(left.len());
    while let Some(root) = largest(&left) {
        left.remove(&root);
        order.push(root);
        let mut queue = VecDeque::from([root]);
        while let Some(f) = queue.pop_front() {
            for &(e, x, y, role) in &roles {
                let g = if x == f {
                    y
                } else if y == f {
                    x
                } else {
                    continue;
                };
                if role != Role::Rip && left.remove(&g) {
                    crossed.insert(e);
                    parent.insert(g, f);
                    order.push(g);
                    queue.push_back(g);
                }
            }
        }
    }
    if order.is_empty() {
        return err("every face of the body is skipped");
    }
    let index: BTreeMap<usize, usize> = order.iter().enumerate().map(|(i, f)| (*f, i)).collect();
    let mut panels = Vec::with_capacity(order.len());
    let mut places = Vec::with_capacity(order.len());
    for &f in &order {
        let Some(shape) = shaped.get(&f) else { continue };
        places.push(shape.frame().ok_or_else(|| DocError::Invalid(format!("the face at {} has no frame", at(face_point(&mesh, f)))))?);
        let loops = face_loops(&mesh, f, tol)?;
        let region = region(b, shape, &loops, tol).map_err(|e| {
            DocError::Invalid(format!("the face at {} can't be a panel: {e}", at(loops.first().and_then(|l| l.first()).map_or(Vec3::ZERO, |p| p.0))))
        })?;
        panels.push(Panel { name: format!("face {f}"), surface: shape.surface(), region });
    }

    // The joints: every edge between two faces of the sheet, `a` the face the other was reached
    // from where there is one; nothing about the sheet depends on which way a joint runs. Each
    // rip's plane is kept, by its ends and toward its `a` panel, and each bend's edge, for the
    // bends' ends below.
    let (mut joints, mut edges) = (Vec::new(), Vec::new());
    let mut rips: Vec<RipEnds> = Vec::new();
    let mut bent: Vec<BendEdge> = Vec::new();
    for &(e, x, y, role) in &roles {
        // A ring of smooth seams (a tube) can't lie flat: it opens where the walk closed it, at a
        // seam the walk did not cross. (A ring of bends is the user's to open: where its seam
        // goes is the design's.)
        let role = if role == Role::Seam && !crossed.contains(&e) { Role::Rip } else { role };
        let Some(line) = mesh.edges.get(e) else { continue };
        let (Some(&p0), Some(&p1)) = (line.first(), line.last()) else { continue };
        let (Some(&ix), Some(&iy), Some(sx), Some(sy)) = (index.get(&x), index.get(&y), shaped.get(&x), shaped.get(&y)) else { continue };
        let a_first = parent.get(&x) != Some(&y);
        let ((ia, sa), (ib, sb)) = if a_first { ((ix, sx), (iy, sy)) } else { ((iy, sy), (ix, sx)) };
        let mid = middle(line).unwrap_or(p0);
        // The edge's ends in each panel's parameters, run so that `a`'s material is on the left.
        let (Some(ra), Some(rb)) = (panels.get(ia).map(|p: &Panel| p.region.clone()), panels.get(ib).map(|p: &Panel| p.region.clone())) else {
            continue;
        };
        let ends = |s: &Shape, from: Vec3, to: Vec3, near_to: &Region2| -> Option<(Vec2, Vec2)> {
            let (mut u0, mut u1) = (s.params(from), s.params(to));
            // On a roll, the angle on the side the panel is.
            if s.angular() {
                let c = middle_turn(near_to).unwrap_or(u0.y);
                for u in [&mut u0, &mut u1] {
                    u.y += ((c - u.y) / std::f64::consts::TAU).round() * std::f64::consts::TAU;
                }
            }
            Some((u0, u1))
        };
        let Some((mut a0, mut a1)) = ends(sa, p0, p1, &ra) else { continue };
        // Which way `a`'s material lies, looked for beside the edge's own middle along its own way
        // there: beside the line between its ends, a curved edge's panel can lie both ways.
        let left = {
            let path = chart_path(sa, line, &ra);
            // The piece of the edge its halfway point lies on, and that point.
            let total: f64 = line.windows(2).map(|w| w[0].dist(w[1])).sum();
            let mut rest = total / 2.0;
            let mut middle = None;
            for (k, w) in line.windows(2).enumerate() {
                let l = w[0].dist(w[1]);
                if rest <= l && l > 0.0 {
                    middle = path.get(k).zip(path.get(k + 1)).map(|(p, q)| (p.lerp(*q, rest / l), *q - *p));
                    break;
                }
                rest -= l;
            }
            let extent = ra.outer.polyline(1e-3).bounds().size().len().max(1e-9);
            match middle.and_then(|(q, d)| Some((q, d.normalized()?))) {
                Some((q, d)) => ra.contains(q + d.perp() * (extent * 1e-4)),
                None => false,
            }
        };
        let (from, to) = if left { (p0, p1) } else { (p1, p0) };
        if from != p0 {
            std::mem::swap(&mut a0, &mut a1);
        }
        let Some((b0, b1)) = ends(sb, from, to, &rb) else { continue };
        let (Some(da), Some(db)) = ((a1 - a0).normalized(), (b1 - b0).normalized()) else { continue };
        let (mut anchor_a, mut anchor_b) = (Anchor { panel: ia, origin: a0, dir: da }, Anchor { panel: ib, origin: b0, dir: db });
        let kind = if role == Role::Bend {
            let (Shape::Plane { out: na, .. }, Shape::Plane { out: nb, .. }) = (sa, sb) else {
                return err(format!("not supported yet: a bend on the edge of a roll, at {}", at(mid)));
            };
            let theta = na.dot(*nb).clamp(-1.0, 1.0).acos();
            // The bend turns toward the material where `b` rises behind `a`'s face.
            let lift = panels.get(ib).map_or(0.0, |p| {
                let pts = p.region.outer.polyline(1e-3);
                let Shape::Plane { plane, .. } = sb else { return 0.0 };
                (&pts).into_iter().map(|q| (plane.to_world(*q) - from).dot(*na)).fold(0.0, |m: f64, h| if h.abs() > m.abs() { h } else { m })
            });
            let len = a0.dist(a1);
            let mut bend = Bend {
                angle: theta,
                radius: rule.bend_radius,
                toward_material: lift < 0.0,
                datum: Datum::Sharp,
                ends: ZoneEnds { near: (0.0, len), far: (0.0, len) },
                tails: (Tail::Keep, Tail::Keep),
                relief: (Relief::None, Relief::None),
            };
            // A wall standing on part of the other panel's edge, its outside in one plane with
            // that edge where it runs on past the wall, bends only where it stands. Past the
            // wall's ends its face is a strip no taller than the sheet (the other panel's own
            // edge), so its outline crosses its tangent line where it crosses a line just above
            // the strip, and runs further along a line within it. A panel whose sides slant, a
            // triangle's, crosses the two lines in different places and bends its whole edge.
            // Past the wall's ends, what of its face lies in the bend's band is trimmed off it.
            let setback = bend.setback(t);
            let (mut edge, mut len) = ([from, to], len);
            let stands = |r: &Region2, o: Vec2, d: Vec2, side: f64| {
                let at = |h: f64| extent_across(r, (o + d.perp() * (side * h), d)).map(|(lo, hi): (f64, f64)| (lo.max(0.0), hi.min(len)));
                let same = |x: (f64, f64), y: (f64, f64)| (x.0 - y.0).abs() <= tol && (x.1 - y.1).abs() <= tol;
                match (setback > t * (1.0 + 1e-9), at(setback), at((t + setback) / 2.0), at(t / 2.0)) {
                    (true, Some(top), Some(above), Some(strip)) if same(top, above) && !same(top, strip) => top,
                    _ => (0.0, len),
                }
            };
            let (stand_a, stand_b) = (stands(&ra, a0, da, 1.0), stands(&rb, b0, db, -1.0));
            let (lo, hi) = (stand_a.0.max(stand_b.0), stand_a.1.min(stand_b.1));
            if hi - lo > tol && (lo > tol || hi < len - tol) {
                let along = (to - from) * (1.0 / len);
                edge = [from + along * lo, from + along * hi];
                anchor_a.origin = a0 + da * lo;
                anchor_b.origin = b0 + db * lo;
                let tail = |s: (f64, f64)| if s.0 > tol || s.1 < len - tol { Tail::Trim } else { Tail::Keep };
                bend.tails = (tail(stand_a), tail(stand_b));
                len = hi - lo;
                bend.ends = ZoneEnds { near: (0.0, len), far: (0.0, len) };
            }
            bent.push(BendEdge { joint: joints.len(), ends: edge, normals: (*na, *nb), toward: lift < 0.0, len, a: anchor_a, b: anchor_b, setback });
            JointKind::Bend(bend)
        } else if role == Role::Seam {
            JointKind::Seam
        } else {
            // The edge as the body has it, from `from` to `to`: on each panel, and the sheet's turn
            // across it at each point, from the two faces' own normals there.
            let mut points = line.clone();
            if points.first().is_some_and(|p| p.dist(from) > p.dist(to)) {
                points.reverse();
            }
            let ways: Vec<Vec3> = (0..points.len()).filter_map(|k| way3(&points, k)).collect();
            if ways.len() != points.len() {
                return err(format!("the edge at {} has no length", at(mid)));
            }
            let turns = points.iter().zip(&ways).map(|(p, x)| {
                // In the `a` face's frame: x along the edge, y toward its material, z on toward
                // the `b` face; the `b` face runs on from the edge into its own material.
                let (y, into_b) = (-sa.out(*p), sb.out(*p).cross(*x));
                into_b.dot(y).atan2(into_b.dot(x.cross(y)))
            });
            let planes = [(points.first(), ways.first()), (points.last(), ways.last())]
                .map(|(p, x)| p.zip(x).and_then(|(p, x)| rip_plane(sa.out(*p), sb.out(*p), -*x)));
            let (Some(&first), Some(&last)) = (ways.first(), ways.last()) else { return err(format!("the edge at {} has no length", at(mid))) };
            rips.push((joints.len(), [from, to], planes, [first, last]));
            let (pa, pb) = (chart_path(sa, &points, &ra), chart_path(sb, &points, &rb));
            JointKind::Rip(Rip {
                curves: (exact_edge(b, sa, e, &pa, tol), exact_edge(b, sb, e, &pb, tol)),
                edges: (pa, pb),
                turns: turns.collect(),
                lap: Lap::Miter { gap: rule.gap },
            })
        };
        joints.push(Joint { a: anchor_a, b: anchor_b, kind });
        edges.push(mid);
    }

    // Where each bend ends, and what is cut there: (along its `a` anchor, along its `b` anchor,
    // the relief), at its start and at its end. Where two bends standing on one panel meet at a
    // corner of it, both end where their tangent lines on it cross, with the rule's corner
    // relief: a notch, a round or a square relief belonging to the corner, or a tear tight
    // against the rip that parts their other panels. Where a rip cuts a panel the bend joins,
    // its metal ends at the rip's plane. Elsewhere the rule's bend relief is cut in whichever
    // panel runs on past the end.
    let near = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let slot = Relief::Slot { width: rule.relief_width, depth: rule.relief_depth };
    let mut ends: Vec<[(f64, f64, Relief); 2]> = bent.iter().map(|e| [(0.0, 0.0, slot), (e.len, e.len, slot)]).collect();
    let (mut notes, mut corners) = (Vec::new(), Vec::new());
    let found = bend_corners(&bent, near);
    let size = rule.corner_relief_size;
    for k in &found {
        let notched = || k.at.map(|a| (a, a, Relief::Notch));
        let values = match rule.corner_relief {
            ReliefShape::Tear => match tear(&bent, &joints, &rips, &places, k, t, near) {
                Ok(Some(torn)) => torn,
                Ok(None) => notched(),
                Err(why) => {
                    notes.push(format!("the corner at {} keeps its notch: {why}", at(k.v)));
                    notched()
                }
            },
            ReliefShape::Round | ReliefShape::Rectangular => {
                // How much of each bend runs on from the corner.
                let left = k.ends.iter().zip(k.at).map(|((i, e), a)| bent.get(*i).map_or(0.0, |b| if *e == 0 { b.len - a } else { a }));
                if !(size >= MIN_RELIEF) {
                    notes.push(format!("no corner relief at {}: the relief size {size:.4} mm is below {MIN_RELIEF} mm", at(k.v)));
                } else if left.fold(f64::INFINITY, f64::min) <= size / 2.0 + 1e-6 {
                    notes.push(format!("no corner relief at {}: a relief {size:.3} mm across would take a whole bend away", at(k.v)));
                } else {
                    let joints = k.ends.map(|(i, _)| bent.get(i).map_or(usize::MAX, |b| b.joint));
                    corners.push(Corner { joints, round: rule.corner_relief == ReliefShape::Round, size });
                }
                notched()
            }
            ReliefShape::Notch => notched(),
        };
        for ((i, e), value) in k.ends.iter().zip(values) {
            if let Some(slot) = ends.get_mut(*i).and_then(|x| x.get_mut(*e)) {
                *slot = value;
            }
        }
    }
    for (i, b) in bent.iter().enumerate() {
        let Some(d) = (b.ends[1] - b.ends[0]).normalized() else { continue };
        for e in 0..2 {
            if found.iter().any(|k| k.ends.contains(&(i, e))) {
                continue;
            }
            let (v, into) = if e == 0 { (b.ends[0], d) } else { (b.ends[1], -d) };
            // A rip that carries on along the bend's edge, tangent to it, runs alongside the bend
            // rather than across its end, and cuts nothing off it.
            let planes = rips.iter().filter_map(|(r, ends, planes, ways)| {
                let end = ends.iter().position(|p| near(*p, v))?;
                if ways.get(end).is_some_and(|w| w.dot(into).abs() > ALONGSIDE) {
                    return None;
                }
                let m = planes.get(end).copied().flatten()?;
                let rip = joints.get(*r)?;
                let mine = |p: usize| p == b.a.panel || p == b.b.panel;
                if mine(rip.a.panel) {
                    Some(m)
                } else if mine(rip.b.panel) {
                    Some(-m)
                } else {
                    None
                }
            });
            if let Some(s) = planes.filter_map(|m| bend_cut(v, into, m, b.normals, b.toward, rule.bend_radius, t, 0.0)).reduce(f64::max) {
                let a = if e == 0 { s } else { b.len - s };
                if let Some(slot) = ends.get_mut(i).and_then(|x| x.get_mut(e)) {
                    *slot = (a, a, Relief::Notch);
                }
            }
        }
    }
    for (b, [(n0, f0, r0), (n1, f1, r1)]) in bent.iter().zip(ends) {
        if n0 >= n1 && f0 >= f1 {
            return err(format!("the bend at {} is cut away by the rips at its ends", at(b.ends[0].lerp(b.ends[1], 0.5))));
        }
        if let Some(Joint { kind: JointKind::Bend(bend), .. }) = joints.get_mut(b.joint) {
            bend.ends = ZoneEnds { near: (n0, n1), far: (f0, f1) };
            bend.relief = (r0, r1);
        }
    }
    let skin = Skin { thickness: t, k_factor: rule.k_factor, panels, joints, corners };
    let development = develop(&skin, 0).map_err(|e| DocError::Invalid(e.to_string()))?;
    if !development.loops.is_empty() {
        let names: Vec<String> = development.loops.iter().filter_map(|m| edges.get(m.joint)).map(|p| at(*p)).collect();
        return err(format!(
            "the edge at {} closes a ring of bends and seams: leave one of the ring's edges out of the bends to cut it open",
            names.join(" and the one at ")
        ));
    }
    Ok(SkinConversion { skin, development, faces: order, places, edges, thickness: t, notes, pieces: std::sync::OnceLock::new() })
}

/// Until the kernel measures a face (#1597): the area of its display triangles, which only
/// picks the largest face, so a facet's error does not change the sheet.
///
/// The area of face `f`'s triangles.
fn face_area(mesh: &Mesh, f: usize) -> f64 {
    let tris = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, g)| **g as usize == f).filter_map(|(t, _)| mesh.tri(t));
    tris.map(|t| (t[1] - t[0]).cross(t[2] - t[0]).len() / 2.0).sum()
}

/// A point in face `f`: the middle of its largest triangle.
fn face_point(mesh: &Mesh, f: usize) -> Vec3 {
    let area = |t: &[Vec3; 3]| (t[1] - t[0]).cross(t[2] - t[0]).len();
    let tri = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, g)| **g as usize == f).filter_map(|(t, _)| mesh.tri(t));
    tri.max_by(|x, y| area(x).total_cmp(&area(y))).map_or(Vec3::ZERO, |t| (t[0] + t[1] + t[2]) * (1.0 / 3.0))
}

/// Until the kernel casts rays on its exact faces (#1597): a ray through the display mesh.
///
/// The face straight through the material behind face `f`, from a ray through its largest
/// triangle along its inward normal.
fn behind_face(mesh: &Mesh, f: usize) -> Option<usize> {
    let area = |t: &[Vec3; 3]| (t[1] - t[0]).cross(t[2] - t[0]);
    let tri = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, g)| **g as usize == f).filter_map(|(t, _)| mesh.tri(t));
    let t = tri.max_by(|x, y| area(x).len().total_cmp(&area(y).len()))?;
    let n = area(&t).normalized()?;
    let (_, hit) = mesh.raycast((t[0] + t[1] + t[2]) * (1.0 / 3.0), n * -1.0)?;
    mesh.tri_face.get(hit).map(|g| *g as usize).filter(|g| *g != f)
}

/// Until the kernel casts rays on its exact faces (#1597): the ray goes through the display
/// mesh, and its ends are taken onto the exact surfaces after.
///
/// How far behind face `f` the face facing back at it lies: from `f`'s surface to that face's,
/// along `f`'s normal through its largest triangle, each end taken onto its face's own surface
/// (`shape` of each face as modelled), so that a roll's facets measure as the roll does.
fn behind(mesh: &Mesh, shape: &impl Fn(usize) -> Option<Shape>, f: usize) -> Option<f64> {
    let area = |t: &[Vec3; 3]| (t[1] - t[0]).cross(t[2] - t[0]).len();
    let tris = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, g)| **g as usize == f).filter_map(|(t, _)| mesh.tri(t));
    let middle = tris.max_by(|x, y| area(x).total_cmp(&area(y))).map(|t| (t[0] + t[1] + t[2]) * (1.0 / 3.0))?;
    let front = shape(f)?;
    let n = front.out(middle);
    let (d, hit) = mesh.raycast(middle, n * -1.0)?;
    let g = *mesh.tri_face.get(hit)? as usize;
    let back = shape(g)?;
    let at = middle - n * d;
    (g != f && back.out(at).dot(n) < -1.0 + 1e-9).then(|| d - front.off(middle) - back.off(at))
}

impl SkinConversion {
    /// The flat pattern: each placed panel less its trims, and the bend zones, joined.
    pub fn flat(&self) -> Result<Vec<Region2>> {
        crate::sheet_skin::join_pieces(self.pieces()?, &mut FlatWork::new())
    }

    /// The flat's pieces: each placed panel less its trims, and the zones, all less the corners'
    /// reliefs. Cut the first time they are asked for.
    fn pieces(&self) -> Result<&[(Piece, Region2)]> {
        if let Some(p) = self.pieces.get() {
            return Ok(p);
        }
        let cut = crate::sheet_skin::pieces(&self.development, &mut FlatWork::new())?;
        Ok(self.pieces.get_or_init(|| cut))
    }

    /// The skin folded, each piece of the flat where its first panel's face is. Every other
    /// panel folds onto its own face through the bends and seams that reach it: anywhere else,
    /// a joint was read wrong, and that is an error rather than a sheet in the wrong place.
    pub fn fold(&self) -> Result<Fold> {
        let roots = self.development.pieces.iter().map(|p| p.first().and_then(|i| self.places.get(*i)).copied());
        let roots: Vec<Rigid3> = roots.collect::<Option<_>>().ok_or_else(|| DocError::Invalid("a piece of the sheet has no face".into()))?;
        let folded = fold(&self.skin, &self.development, &roots).map_err(|e| DocError::Invalid(e.to_string()))?;
        let reach = self.development.panels.iter().flatten().map(|p| p.region.outer.polyline(1e-3).bounds().size().len()).fold(1.0, f64::max);
        for (i, (got, own)) in folded.places.iter().zip(&self.places).enumerate() {
            let Some(got) = got else { continue };
            let off = got.shift.dist(own.shift) + reach * (got.x.dist(own.x) + got.y.dist(own.y) + got.z.dist(own.z));
            if off > (reach * 1e-9).max(1e-9) {
                let face = self.faces.get(i).copied().unwrap_or_default();
                return err(format!("the sheet folds face {face} {off:.3e} mm from where the body has it"));
            }
        }
        Ok(folded)
    }

    /// The folded sheet: its solid, sewn from the flat's pieces carried into space.
    pub fn solid(&self) -> Result<Body> {
        crate::sheet_skin_fold::solid(&self.development, &self.fold()?, self.thickness, self.pieces()?, &mut FlatWork::new())
    }
}

/// The point halfway along a polyline.
fn middle(line: &[Vec3]) -> Option<Vec3> {
    let total: f64 = line.windows(2).map(|w| w[0].dist(w[1])).sum();
    let mut left = total / 2.0;
    for w in line.windows(2) {
        let l = w[0].dist(w[1]);
        if left <= l && l > 0.0 {
            return Some(w[0].lerp(w[1], left / l));
        }
        left -= l;
    }
    line.first().copied()
}

/// A body converted through its skin, as the model keeps it: its flat pattern, which
/// `sheet.flat_pattern` shows. The body itself becomes the folded sheet.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SkinFlat {
    pub body: String,
    pub rule: String,
    pub thickness: f64,
    pub k_factor: f64,
    /// The flat material, in the plane of the sheet's largest face.
    pub regions: Vec<Region2>,
    pub bends: Vec<SkinBendLine>,
    /// The largest strain any panel's flat leaves: none but on a free-form panel.
    pub stretch: f64,
    /// How many of the panels are rolls.
    pub rolls: usize,
}

/// A bend in a skin's flat: its zone, its center line, how far it turns (radians), its inside
/// radius, whether it turns toward the material, and the zone's width.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SkinBendLine {
    pub zone: Region2,
    pub start: Vec2,
    pub end: Vec2,
    pub angle: f64,
    pub radius: f64,
    pub up: bool,
    pub allowance: f64,
}

impl SkinConversion {
    /// The flat pattern as the model keeps it, for the body `body` under the rule `rule`.
    pub fn record(&self, body: &str, rule: &str) -> Result<SkinFlat> {
        let bends = self
            .development
            .zones
            .iter()
            .filter_map(|z| {
                let JointKind::Bend(b) = &self.skin.joints.get(z.joint)?.kind else { return None };
                Some(SkinBendLine {
                    zone: z.region.clone(),
                    start: (z.near[0] + z.far[0]) * 0.5,
                    end: (z.near[1] + z.far[1]) * 0.5,
                    angle: b.angle,
                    radius: b.radius,
                    up: b.toward_material,
                    allowance: b.allowance(self.skin.thickness, self.skin.k_factor),
                })
            })
            .collect();
        Ok(SkinFlat {
            body: body.into(),
            rule: rule.into(),
            thickness: self.thickness,
            k_factor: self.skin.k_factor,
            regions: self.flat()?,
            bends,
            stretch: self.development.panels.iter().flatten().map(|p| p.stretch).fold(0.0, f64::max),
            rolls: self.skin.panels.iter().filter(|p| p.surface != Surface::Plane).count(),
        })
    }
}
