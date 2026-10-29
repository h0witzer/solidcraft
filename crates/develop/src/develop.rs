//! Laying a skin's panels out flat through its bends and seams.

use std::collections::VecDeque;
use std::f64::consts::PI;

use solidcraft_geom::{Aabb2, Loop2, Region2, Seg2, Vec2, Vec3};

use crate::chart::Flatten;
use crate::fold::Wrap;
use crate::{Anchor, Bend, Corner, Joint, JointKind, Panel, Relief, Rigid2, Skin, Surface, Tail};

/// The most panels and joints one development takes: a work limit, never a truncation.
const MAX_PANELS: usize = 10_000;
const MAX_JOINTS: usize = 20_000;

/// How far from a unit vector an anchor's direction may be.
const UNIT_TOL: f64 = 1e-9;

/// Points closer than this (mm) along a polyline are one point when a panel's reach is measured.
const REACH_TOL: f64 = 1e-3;

/// A panel's edge this close (mm) to a bend zone's end ends with it.
const EDGE_TOL: f64 = 1e-6;

/// How far (mm) a torn end's spline may lie from the curve the tear follows: well inside what a
/// press brake holds, so that a couple of spans do; the most spans it takes; and how many points
/// along it are looked at.
const TEAR_TOL: f64 = 1e-3;
const MAX_TEAR_SPANS: usize = 16;
const TEAR_SAMPLES: usize = 64;

/// A skin laid out flat.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Development {
    /// Where each panel's neutral chart lies in the flat frame (a planar panel's neutral chart
    /// is its chart).
    pub placements: Vec<Option<Rigid2>>,
    /// Each panel with material, in the flat frame.
    pub panels: Vec<Option<FlatPanel>>,
    /// The pieces of the flat pattern: the panels that bends and seams hold together. The root's
    /// piece is first and keeps the flat frame; the others are laid beside it, left to right.
    pub pieces: Vec<Vec<usize>>,
    /// Each bend's zone in the flat frame.
    pub zones: Vec<Zone>,
    /// What to take out of each panel before the pieces are joined: its material between its
    /// tangent line and a bend's anchor, along the zone and, where its tail is trimmed, beyond
    /// it; and what a rip cuts from its edge.
    pub trims: Vec<Trim>,
    /// Bends and seams that close a loop, where the layout reached both panels another way.
    pub loops: Vec<Mismatch>,
    /// What the corners' reliefs cut from the flat, from every piece they reach.
    pub cuts: Vec<Region2>,
}

/// A panel laid flat. For each loop of its region, `origins` gives each segment's index in the
/// same loop of the panel's own region: what the flat edge is in the skin.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlatPanel {
    pub region: Region2,
    pub origins: Vec<Vec<usize>>,
    /// The largest strain the flat leaves on the panel's neutral surface: none on a plane, a
    /// cylinder or a cone; on a free-form panel what its fit could not avoid, which is more than
    /// a rounding only where the panel is not developable.
    pub stretch: f64,
}

/// A bend's zone in the flat frame: its region, its edge on the `a` panel's side (`near`) and on
/// the `b` panel's side (`far`), each from the end at the anchors' origin to the other. Its
/// ends join `near[k]` to `far[k]`.
#[derive(Clone, Debug, PartialEq)]
pub struct Zone {
    pub joint: usize,
    pub region: Region2,
    pub near: [Vec2; 2],
    pub far: [Vec2; 2],
}

/// What a joint takes from a panel: the region to cut, and the line along which the panel's
/// material ends once it is cut (its tangent line at a bend, its cut edge at a rip).
#[derive(Clone, Debug, PartialEq)]
pub struct Trim {
    pub panel: usize,
    pub joint: usize,
    pub region: Region2,
    pub line: [Vec2; 2],
}

/// How far a joint that closes a loop is from fitting: laid out through the rest of the loop,
/// its second panel lands `distance` (mm) and `angle` (radians) from where the joint itself
/// would put it. A ring of bends and seams never fits; it needs a rip.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mismatch {
    pub joint: usize,
    pub distance: f64,
    pub angle: f64,
}

#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum DevelopError {
    #[error("the skin has no panel {0}")]
    NoPanel(usize),
    #[error("panel {0}: {1}")]
    Panel(usize, &'static str),
    #[error("joint {0}: {1}")]
    Joint(usize, &'static str),
    #[error("the skin exceeds the development's work limit")]
    TooLarge,
    #[error("the fold takes a place in space for each of the flat's {0} pieces")]
    Roots(usize),
}

/// A joint's frame in the flat: where its `a` anchor lies, the joint's direction, and the
/// direction from the `a` panel toward the `b` panel.
struct JointFrame {
    origin: Vec2,
    along: Vec2,
    toward_b: Vec2,
}

impl JointFrame {
    fn from_a(joint: &Joint, a: &Rigid2) -> Option<JointFrame> {
        let along = a.vector(joint.a.dir).normalized()?;
        Some(JointFrame { origin: a.point(joint.a.origin), along, toward_b: right_of(along) })
    }

    /// The frame seen from the `b` panel, whose anchor lies `offset` from the `a` one.
    fn from_b(joint: &Joint, b: &Rigid2, offset: f64) -> Option<JointFrame> {
        let along = b.vector(joint.b.dir).normalized()?;
        let toward_b = right_of(along);
        Some(JointFrame { origin: b.point(joint.b.origin) - toward_b * offset, along, toward_b })
    }

    fn place_b(&self, joint: &Joint, offset: f64) -> Option<Rigid2> {
        Rigid2::onto(joint.b.origin, joint.b.dir, self.origin + self.toward_b * offset, self.along)
    }

    fn place_a(&self, joint: &Joint) -> Option<Rigid2> {
        Rigid2::onto(joint.a.origin, joint.a.dir, self.origin, self.along)
    }

    fn at(&self, along: f64, across: f64) -> Vec2 {
        self.origin + self.along * along + self.toward_b * across
    }
}

/// Rotated -90 degrees: with a panel's material on the left of an anchor, the other panel's
/// side.
fn right_of(v: Vec2) -> Vec2 {
    Vec2::new(v.y, -v.x)
}

/// How far the `b` anchor lies from the `a` one in the flat, toward the `b` panel, for a joint
/// the layout follows: for a bend, the zone's width less each panel's setback from its tangent
/// line to its anchor; for a seam, nothing. A rip is not followed.
fn anchor_offset(kind: &JointKind, skin: &Skin) -> Option<f64> {
    match kind {
        JointKind::Bend(bend) => Some(bend.allowance(skin.thickness, skin.k_factor) - 2.0 * bend.setback(skin.thickness)),
        JointKind::Seam => Some(0.0),
        JointKind::Rip { .. } => None,
    }
}

/// Lays the skin out flat. Each panel is first put on its neutral surface (a roll's is longer
/// or shorter round it than its skin), then the `root` panel's neutral chart is the flat frame
/// and every panel that a chain of bends and seams reaches from it is turned and shifted so that
/// each bend's zone fits between its two panels and each seam's panels meet edge to edge. Panels
/// that rips keep apart are laid out the same way as pieces of their own, beside the root's. A
/// joint that closes a loop is laid out once, through the rest of the loop, and reported in
/// [`Development::loops`].
pub fn develop(skin: &Skin, root: usize) -> Result<Development, DevelopError> {
    if skin.panels.len() > MAX_PANELS || skin.joints.len() > MAX_JOINTS {
        return Err(DevelopError::TooLarge);
    }
    if root >= skin.panels.len() {
        return Err(DevelopError::NoPanel(root));
    }
    for (j, joint) in skin.joints.iter().enumerate() {
        check_joint(skin, j, joint)?;
    }
    // Every panel on its neutral surface, and the joints' anchors with them: from here on the
    // layout works on planar charts.
    let maps: Vec<Flatten> = skin
        .panels
        .iter()
        .enumerate()
        .map(|(i, p)| Flatten::of(&p.surface, skin).map_err(|e| DevelopError::Panel(i, e)))
        .collect::<Result<_, _>>()?;
    let mut neutral = Skin {
        thickness: skin.thickness,
        k_factor: skin.k_factor,
        panels: Vec::with_capacity(skin.panels.len()),
        joints: Vec::with_capacity(skin.joints.len()),
        corners: skin.corners.clone(),
    };
    let mut origins = Vec::with_capacity(skin.panels.len());
    for (i, (panel, map)) in skin.panels.iter().zip(&maps).enumerate() {
        let (region, from) = map.region(&panel.region).map_err(|e| DevelopError::Panel(i, e))?;
        neutral.panels.push(Panel { name: panel.name.clone(), surface: Surface::Plane, region });
        origins.push(from);
    }
    for (j, joint) in skin.joints.iter().enumerate() {
        let moved = |a: &Anchor| -> Result<Anchor, DevelopError> {
            let map = maps.get(a.panel).ok_or(DevelopError::NoPanel(a.panel))?;
            let dir = map.dir(a.origin, a.dir).ok_or(DevelopError::Joint(j, "an anchor has no direction"))?;
            Ok(Anchor { panel: a.panel, origin: map.point(a.origin), dir })
        };
        neutral.joints.push(Joint { a: moved(&joint.a)?, b: moved(&joint.b)?, kind: joint.kind.clone() });
    }
    let stretch: Vec<f64> = maps.iter().map(Flatten::stretch).collect();
    let skin = &neutral;

    let mut touching: Vec<Vec<usize>> = vec![Vec::new(); skin.panels.len()];
    for (j, joint) in skin.joints.iter().enumerate() {
        if anchor_offset(&joint.kind, skin).is_some() {
            for p in [joint.a.panel, joint.b.panel] {
                touching.get_mut(p).ok_or(DevelopError::NoPanel(p))?.push(j);
            }
        }
    }
    let mut placements: Vec<Option<Rigid2>> = vec![None; skin.panels.len()];
    let mut followed = vec![false; skin.joints.len()];
    let mut pieces = Vec::new();
    let mut next = Some(root);
    while let Some(start) = next {
        if let Some(p) = placements.get_mut(start) {
            *p = Some(Rigid2::IDENTITY);
        }
        pieces.push(lay_out(skin, start, &touching, &mut placements, &mut followed)?);
        next = placements.iter().position(Option::is_none);
    }
    // The pieces beside the root's, left to right, with their lowest edges level.
    let gap = (10.0 * skin.thickness).max(5.0);
    let bounds: Vec<Option<Aabb2>> = pieces.iter().map(|piece| piece_bounds(skin, piece, &placements)).collect();
    if let Some(Some(first)) = bounds.first() {
        let mut x = first.max.x + gap;
        for (piece, b) in pieces.iter().zip(&bounds).skip(1) {
            let Some(b) = b else { continue };
            let shift = Vec2::new(x - b.min.x, first.min.y - b.min.y);
            for &p in piece {
                if let Some(Some(place)) = placements.get_mut(p) {
                    place.shift += shift;
                }
            }
            x += b.max.x - b.min.x + gap;
        }
    }

    let mut out = Development { pieces, ..Development::default() };
    out.panels = skin
        .panels
        .iter()
        .zip(&placements)
        .zip(origins)
        .zip(stretch)
        .map(|(((panel, place), origins), stretch)| match place {
            Some(place) if !panel.region.outer.segs.is_empty() => Some(FlatPanel { region: place.region(&panel.region), origins, stretch }),
            _ => None,
        })
        .collect();
    out.placements = placements;
    for (j, joint) in skin.joints.iter().enumerate() {
        let (Some(a), Some(b)) = (out.placements.get(joint.a.panel).copied().flatten(), out.placements.get(joint.b.panel).copied().flatten()) else {
            continue;
        };
        let frame = JointFrame::from_a(joint, &a).ok_or(DevelopError::Joint(j, "an anchor has no direction"))?;
        if let Some(offset) = anchor_offset(&joint.kind, skin)
            && !followed.get(j).copied().unwrap_or(false)
            && let Some(fits) = frame.place_b(joint, offset)
        {
            let distance = b.point(joint.b.origin).dist(fits.point(joint.b.origin));
            out.loops.push(Mismatch { joint: j, distance, angle: fits.inverse().then(&b).angle().abs() });
        }
        match &joint.kind {
            JointKind::Bend(bend) => add_zone(&mut out, skin, j, joint, bend, &frame, (&a, &b))?,
            JointKind::Seam => {}
            JointKind::Rip { cut } => add_rip(&mut out, skin, j, joint, *cut, (&a, &b))?,
        }
    }
    out.cuts = skin.corners.iter().filter_map(|c| corner_cut(&out, c)).collect();
    Ok(out)
}

/// Lays out the piece that bends and seams hold together with `start`, already placed: a
/// breadth-first walk that places each panel from the first joint that reaches it. Returns the
/// piece's panels.
fn lay_out(
    skin: &Skin,
    start: usize,
    touching: &[Vec<usize>],
    placements: &mut [Option<Rigid2>],
    followed: &mut [bool],
) -> Result<Vec<usize>, DevelopError> {
    let mut piece = vec![start];
    let mut queue = VecDeque::from([start]);
    while let Some(panel) = queue.pop_front() {
        let Some(at) = placements.get(panel).copied().flatten() else { continue };
        for &j in touching.get(panel).map(Vec::as_slice).unwrap_or_default() {
            let (Some(joint), Some(false)) = (skin.joints.get(j), followed.get(j).copied()) else { continue };
            let Some(offset) = anchor_offset(&joint.kind, skin) else { continue };
            let (other, placed) = if joint.a.panel == panel {
                (joint.b.panel, JointFrame::from_a(joint, &at).and_then(|f| f.place_b(joint, offset)))
            } else {
                (joint.a.panel, JointFrame::from_b(joint, &at, offset).and_then(|f| f.place_a(joint)))
            };
            if placements.get(other).copied().flatten().is_some() {
                continue;
            }
            let placed = placed.ok_or(DevelopError::Joint(j, "an anchor has no direction"))?;
            if let Some(slot) = placements.get_mut(other) {
                *slot = Some(placed);
            }
            if let Some(f) = followed.get_mut(j) {
                *f = true;
            }
            piece.push(other);
            queue.push_back(other);
        }
    }
    Ok(piece)
}

/// The flat bounding box of a piece's material.
fn piece_bounds(skin: &Skin, piece: &[usize], placements: &[Option<Rigid2>]) -> Option<Aabb2> {
    let mut b: Option<Aabb2> = None;
    for &p in piece {
        let (Some(panel), Some(Some(place))) = (skin.panels.get(p), placements.get(p)) else { continue };
        if panel.region.outer.segs.is_empty() {
            continue;
        }
        let pb = place.loop2(&panel.region.outer).polyline(REACH_TOL).bounds();
        b = Some(b.map_or(pb, |b| b.union(&pb)));
    }
    b
}

fn check_joint(skin: &Skin, j: usize, joint: &Joint) -> Result<(), DevelopError> {
    for anchor in [&joint.a, &joint.b] {
        if anchor.panel >= skin.panels.len() {
            return Err(DevelopError::NoPanel(anchor.panel));
        }
        if !anchor.origin.is_finite() || !((anchor.dir.len() - 1.0).abs() <= UNIT_TOL) {
            return Err(DevelopError::Joint(j, "an anchor's direction is not a unit vector"));
        }
    }
    if joint.a.panel == joint.b.panel {
        return Err(DevelopError::Joint(j, "both anchors are on one panel"));
    }
    match &joint.kind {
        JointKind::Bend(bend) => {
            if !(bend.angle > 0.0 && bend.angle <= PI) || !(bend.radius >= 0.0) || !bend.radius.is_finite() {
                return Err(DevelopError::Joint(j, "a bend turns more than 0 and at most a half turn, round a radius of 0 or more"));
            }
            if bend.datum == crate::Datum::Sharp && !(bend.setback(skin.thickness).is_finite() && bend.angle < PI) {
                return Err(DevelopError::Joint(j, "a half turn has no virtual sharp: anchor it at its tangent lines"));
            }
            let (near, far) = (bend.ends.near, bend.ends.far);
            if ![near.0, near.1, far.0, far.1].iter().all(|v| v.is_finite()) {
                return Err(DevelopError::Joint(j, "the zone's ends are not numbers"));
            }
            let flat = |a: &Anchor| skin.panels.get(a.panel).is_some_and(|p| p.surface == Surface::Plane);
            if !(flat(&joint.a) && flat(&joint.b)) {
                return Err(DevelopError::Joint(j, "not supported yet: a bend on the edge of a roll"));
            }
        }
        JointKind::Seam => {}
        JointKind::Rip { cut } => {
            if !(cut.0 >= 0.0 && cut.1 >= 0.0 && cut.0.is_finite() && cut.1.is_finite()) {
                return Err(DevelopError::Joint(j, "a rip cuts 0 or more from each side"));
            }
        }
    }
    Ok(())
}

/// What a rip cuts from each panel: a band along its edge, as deep as that side's cut, laid
/// out with the panel wherever its piece put it.
fn add_rip(out: &mut Development, skin: &Skin, j: usize, joint: &Joint, cut: (f64, f64), placed: (&Rigid2, &Rigid2)) -> Result<(), DevelopError> {
    for (anchor, place, depth, into) in [(&joint.a, placed.0, cut.0, -1.0), (&joint.b, placed.1, cut.1, 1.0)] {
        if !(depth > 0.0) {
            continue;
        }
        let Some(along) = place.vector(anchor.dir).normalized() else { continue };
        let (o, across) = (place.point(anchor.origin), right_of(along) * into);
        let reach = panel_reach(skin, anchor.panel)?;
        let corners = [o - along * reach, o + along * reach, o + along * reach + across * depth, o - along * reach + across * depth];
        let region = Region2 { outer: Loop2::polygon(&corners).ccw(), holes: Vec::new() };
        out.trims.push(Trim { panel: anchor.panel, joint: j, region, line: [corners[3], corners[2]] });
    }
    Ok(())
}

/// The bend's zone, and what each panel loses to it.
fn add_zone(
    out: &mut Development,
    skin: &Skin,
    j: usize,
    joint: &Joint,
    bend: &Bend,
    frame: &JointFrame,
    placed: (&Rigid2, &Rigid2),
) -> Result<(), DevelopError> {
    let width = bend.allowance(skin.thickness, skin.k_factor);
    let setback = bend.setback(skin.thickness);
    // Across the joint, from the `a` anchor: the `a` panel's tangent line, then the zone, then
    // the `b` panel's tangent line.
    let (near_line, far_line) = (-setback, width - setback);
    let (near, far) = (bend.ends.near, bend.ends.far);
    if width > 0.0 && (near.1 > near.0 || far.1 > far.0) {
        let (n0, n1, f0, f1) = (frame.at(near.0, near_line), frame.at(near.1, near_line), frame.at(far.0, far_line), frame.at(far.1, far_line));
        // Each end straight across, or torn along the curve it follows; the loop runs along the
        // `a` side, up the end, back along the `b` side and down the start.
        let end = |relief: &Relief, (from, to): (f64, f64)| match *relief {
            Relief::Tear { normal, offset } => torn(skin, bend, frame, (normal, offset), (from, to)),
            _ => vec![Seg2::Line { a: frame.at(from, near_line), b: frame.at(to, far_line) }],
        };
        let mut segs = vec![Seg2::Line { a: n0, b: n1 }];
        segs.extend(end(&bend.relief.1, (near.1, far.1)));
        segs.push(Seg2::Line { a: f1, b: f0 });
        segs.extend(end(&bend.relief.0, (near.0, far.0)).iter().rev().map(Seg2::reversed));
        out.zones.push(Zone { joint: j, region: Region2 { outer: Loop2 { segs }.ccw(), holes: Vec::new() }, near: [n0, n1], far: [f0, f1] });
    }
    // A panel's band runs from its tangent line across its anchor, and a thickness past it so
    // that an edge lying on the anchor leaves no sliver.
    let depth = setback + skin.thickness.max(REACH_TOL);
    let bands = [(joint.a.panel, placed.0, near, near_line, depth, bend.tails.0), (joint.b.panel, placed.1, far, far_line, -depth, bend.tails.1)];
    let relieved = |r: &Relief| matches!(r, Relief::Notch | Relief::Tear { .. });
    for (panel, place, (u0, u1), line, depth, tail) in bands {
        // The band's ends run on past the panel wherever nothing of the panel lies beyond them
        // in the band: always for a trimmed tail, which goes the same way, and for a kept one
        // where the panel ends with the zone. Either way the cut is the same, and an end left
        // on the panel's own edge would be a rounding away from it, for the planar booleans to
        // trip on.
        let reach = panel_reach(skin, panel)?;
        let extent = if tail == Tail::Keep { band_extent(skin, panel, place, frame, line, depth)? } else { None };
        let (s0, s1) = match (tail, extent) {
            (Tail::Trim, _) => (u0.min(u1) - reach, u0.max(u1) + reach),
            (Tail::Keep, None) => continue,
            (Tail::Keep, Some((lo, hi))) => (
                if lo >= u0 - EDGE_TOL || relieved(&bend.relief.0) { u0 - reach } else { u0 },
                if hi <= u1 + EDGE_TOL || relieved(&bend.relief.1) { u1 + reach } else { u1 },
            ),
        };
        if s1 > s0 {
            let corners = [frame.at(s0, line), frame.at(s1, line), frame.at(s1, line + depth), frame.at(s0, line + depth)];
            let region = Region2 { outer: Loop2::polygon(&corners).ccw(), holes: Vec::new() };
            out.trims.push(Trim { panel, joint: j, region, line: [corners[0], corners[1]] });
        }
        // A bend relief where the panel runs on past an end: a slot past the end, from inside
        // the panel's tangent line out through the band. `out_dir` runs past the end.
        let Some((lo, hi)) = extent else { continue };
        for (relief, u, out_dir) in [(bend.relief.0, u0, -1.0), (bend.relief.1, u1, 1.0)] {
            let Relief::Slot { width: w, depth: d } = relief else { continue };
            let runs_on = if out_dir < 0.0 { lo < u - w - EDGE_TOL } else { hi > u + w + EDGE_TOL };
            if !(runs_on && w > 0.0) {
                continue;
            }
            let (s0, s1) = if out_dir < 0.0 { (u - w, u) } else { (u, u + w) };
            let (l0, l1) = (line - depth.signum() * d.max(0.0), line + depth);
            let corners = [frame.at(s0, l0), frame.at(s1, l0), frame.at(s1, l1), frame.at(s0, l1)];
            let region = Region2 { outer: Loop2::polygon(&corners).ccw(), holes: Vec::new() };
            out.trims.push(Trim { panel, joint: j, region, line: [corners[0], corners[1]] });
        }
    }
    Ok(())
}

/// A torn end of a bend's zone in the flat, from `from` along the joint on the `a` panel's tangent
/// line to `to` on the `b` panel's, as a cubic spline within [`TEAR_TOL`] of the curve it follows:
/// at each turn round the bend, the length along the joint where the bend's metal, through its
/// whole thickness, keeps off the tear's plane by as much as the ends give in proportion.
fn torn(skin: &Skin, bend: &Bend, frame: &JointFrame, (normal, offset): (Vec3, f64), (from, to): (f64, f64)) -> Vec<Seg2> {
    let (t, theta) = (skin.thickness, bend.angle);
    let wrap = Wrap::of(bend, t, skin.k_factor);
    let neutral = bend.radius + skin.k_factor * t;
    let straight = vec![Seg2::Line { a: frame.at(from, -bend.setback(t)), b: frame.at(to, theta * neutral - bend.setback(t)) }];
    // Along the joint, the distance off the plane changes alike at every depth.
    if !(normal.x.abs() > 1e-9 && neutral > 0.0 && theta > 0.0) {
        return straight;
    }
    let off = |along: f64, phi: f64, depth: f64| wrap.at(along, phi, depth).dot(normal) - offset;
    let least = |along: f64, phi: f64| off(along, phi, 0.0).min(off(along, phi, t));
    let (c0, c1) = (least(from, 0.0), least(to, theta));
    // Where along the joint each face of the sheet would keep its share off the plane: the
    // nearer of the two to the plane keeps the metal off it.
    let need = |phi: f64| {
        let clear = c0 + (c1 - c0) * phi / theta;
        [0.0, t].map(|depth| from + (clear - off(from, phi, depth)) / normal.x)
    };
    let deep = |phi: f64| {
        let [skin, deep] = need(phi);
        if normal.x > 0.0 { deep > skin } else { deep < skin }
    };
    let at = |phi: f64| -> Vec2 {
        let [skin, deep] = need(phi);
        let along = if normal.x > 0.0 { skin.max(deep) } else { skin.min(deep) };
        frame.at(along, phi * neutral - bend.setback(t))
    };
    // The curve is smooth but where the face that keeps the metal off changes: there it turns a
    // corner. Each smooth run is a cubic spline through it.
    let mut breaks = vec![0.0];
    let mid = |k: usize| theta * (k as f64 + 0.5) / TEAR_SAMPLES as f64;
    for k in 1..TEAR_SAMPLES {
        let (mut lo, mut hi) = (mid(k - 1), mid(k));
        if deep(lo) != deep(hi) {
            let side = deep(lo);
            for _ in 0..60 {
                let m = (lo + hi) / 2.0;
                if deep(m) == side {
                    lo = m;
                } else {
                    hi = m;
                }
            }
            breaks.push((lo + hi) / 2.0);
        }
    }
    breaks.push(theta);
    let mut out = Vec::new();
    for w in breaks.windows(2) {
        if let [p0, p1] = w {
            out.extend(spline(&at, *p0, *p1));
        }
    }
    out
}

/// The curve `at` from `a` to `b` as a cubic spline in the fewest even spans that keep within
/// [`TEAR_TOL`] of it: through the curve at the spans' ends, along it at its two ends (its
/// direction there taken from inside the run), and joined with no change of curvature.
fn spline(at: &dyn Fn(f64) -> Vec2, a: f64, b: f64) -> Vec<Seg2> {
    let rate = |x: f64, h: f64| (at(x) * -3.0 + at(x + h) * 4.0 - at(x + 2.0 * h)) * (1.0 / (2.0 * h));
    let step = (b - a) * 1e-5;
    let (start, end) = (rate(a, step), rate(b, -step));
    let mut segs = Vec::new();
    for spans in 1..=MAX_TEAR_SPANS {
        let h = (b - a) / spans as f64;
        let points: Vec<Vec2> = (0..=spans).map(|i| at(a + h * i as f64)).collect();
        let rates = clamped_rates(&points, h, start, end);
        segs = points
            .windows(2)
            .zip(rates.windows(2))
            .filter_map(|(p, m)| match (p, m) {
                ([p0, p1], [m0, m1]) => Some(Seg2::Cubic { p0: *p0, p1: *p0 + *m0 * (h / 3.0), p2: *p1 - *m1 * (h / 3.0), p3: *p1 }),
                _ => None,
            })
            .collect();
        let strays = (0..=TEAR_SAMPLES).any(|k| {
            let x = k as f64 / TEAR_SAMPLES as f64 * spans as f64;
            let i = (x.floor() as usize).min(spans - 1);
            segs.get(i).is_some_and(|seg| seg.point_at(x - i as f64).dist(at(a + h * x)) > TEAR_TOL)
        });
        if !strays {
            break;
        }
    }
    segs
}

/// A clamped cubic spline's rates (per unit of its parameter) at `points`, `h` apart: `start` and
/// `end` at its ends, and between them those that keep its curvature unbroken,
/// `m[i - 1] + 4 m[i] + m[i + 1] = 3 (p[i + 1] - p[i - 1]) / h`, solved down the tridiagonal.
fn clamped_rates(points: &[Vec2], h: f64, start: Vec2, end: Vec2) -> Vec<Vec2> {
    let n = points.len().saturating_sub(1);
    let mut m = vec![start; n + 1];
    if let Some(last) = m.last_mut() {
        *last = end;
    }
    if n < 2 {
        return m;
    }
    // Forward sweep over the inner rates 1 ..= n - 1, then back.
    let (mut diag, mut rhs) = (vec![4.0; n + 1], vec![Vec2::ZERO; n + 1]);
    for i in 1..n {
        rhs[i] = (points[i + 1] - points[i - 1]) * (3.0 / h);
    }
    rhs[1] -= start;
    rhs[n - 1] -= end;
    for i in 2..n {
        let f = 1.0 / diag[i - 1];
        diag[i] -= f;
        let carried = rhs[i - 1] * f;
        rhs[i] -= carried;
    }
    for i in (1..n).rev() {
        let next = if i + 1 < n { m[i + 1] } else { Vec2::ZERO };
        m[i] = (rhs[i] - next) * (1.0 / diag[i]);
    }
    m
}

/// The relief a corner cuts from the flat: centered where its two bends' zones' center lines
/// cross. `None` where either bend has no zone, or the two run alike.
fn corner_cut(dev: &Development, corner: &Corner) -> Option<Region2> {
    let center_line = |j: usize| {
        let z = dev.zones.iter().find(|z| z.joint == j)?;
        let [n0, n1] = z.near;
        let [f0, f1] = z.far;
        let (a, b) = ((n0 + f0) * 0.5, (n1 + f1) * 0.5);
        Some((a, (b - a).normalized()?))
    };
    let [ja, jb] = corner.joints;
    let ((p, u), (q, w)) = (center_line(ja)?, center_line(jb)?);
    let cross = u.cross(w);
    let r = corner.size / 2.0;
    if !(cross.abs() > 1e-9 && r > 0.0) {
        return None;
    }
    let c = p + u * ((q - p).cross(w) / cross);
    let outer = if corner.round {
        Loop2 { segs: vec![Seg2::Arc { center: c, radius: r, start: 0.0, sweep: PI }, Seg2::Arc { center: c, radius: r, start: PI, sweep: PI }] }
    } else {
        // Its sides run along the two bends, each `r` from the center.
        let k = r / cross.abs();
        Loop2::polygon(&[c + (u + w) * k, c + (w - u) * k, c - (u + w) * k, c + (u - w) * k])
    };
    Some(Region2 { outer: outer.ccw(), holes: Vec::new() })
}

/// Where along a bend a panel's outline lies inside the band from `line` to `line + depth`
/// across it, in the flat: the least and greatest lengths along the bend, or `None` when none
/// of it does.
fn band_extent(skin: &Skin, panel: usize, place: &Rigid2, frame: &JointFrame, line: f64, depth: f64) -> Result<Option<(f64, f64)>, DevelopError> {
    let region = &skin.panels.get(panel).ok_or(DevelopError::NoPanel(panel))?.region;
    let (lo, hi) = (line.min(line + depth) - EDGE_TOL, line.max(line + depth) + EDGE_TOL);
    let pts: Vec<(f64, f64)> = (&region.outer.polyline(REACH_TOL))
        .into_iter()
        .map(|q| {
            let p = place.point(*q) - frame.origin;
            (p.dot(frame.along), p.dot(frame.toward_b))
        })
        .collect();
    let mut extent: Option<(f64, f64)> = None;
    let mut take = |u: f64| extent = Some(extent.map_or((u, u), |(a, b)| (a.min(u), b.max(u))));
    // Each edge of the outline, clipped to the band.
    for (k, &(u_a, v_a)) in pts.iter().enumerate() {
        let Some(&(u_b, v_b)) = pts.get((k + 1) % pts.len()) else { continue };
        let at = |v: f64| if (v_b - v_a).abs() > 1e-15 { u_a + (u_b - u_a) * (v - v_a) / (v_b - v_a) } else { u_a };
        let (t0, t1) = (v_a.min(v_b).max(lo), v_a.max(v_b).min(hi));
        if t0 <= t1 {
            take(at(t0));
            take(at(t1));
            if (lo..=hi).contains(&v_a) {
                take(u_a);
            }
        }
    }
    Ok(extent)
}

/// How far a panel's material reaches: the diagonal of its region's bounding box, which bounds
/// how far along a joint a trimmed tail must go.
fn panel_reach(skin: &Skin, panel: usize) -> Result<f64, DevelopError> {
    let region = &skin.panels.get(panel).ok_or(DevelopError::NoPanel(panel))?.region;
    if region.outer.segs.is_empty() {
        return Ok(0.0);
    }
    Ok(region.outer.polyline(REACH_TOL).bounds().size().len())
}
