//! Foldable 3D print: a sheet made as one flat body to print and fold. Its outer skin lies on the print bed (z = 0 of the flat frame) and every bend turns
//! toward +z. A V-groove is cut down from the top at each bend's virtual sharp, its two walls at
//! half the bend angle each side of upright, so that once folded they meet as a mitre. Under the
//! groove a film `film` thick is left: the hinge the panels fold on. `clearance` moves each wall
//! of the groove that far apart from the other, measured square to the walls once folded.

use serde::{Deserialize, Serialize};

use solidcore_record::{Vec2, Vec3};
use solidcraft_geom::prelude::*;

/// Values a foldable print takes, as checked by `check_print_fold`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrintFold {
    /// The film left under each V-groove (mm).
    pub film: f64,
    /// How far the folded walls of a groove stay apart (mm).
    pub clearance: f64,
}

/// The default film under a V-groove (mm): two or three printed layers.
pub const DEFAULT_FILM: f64 = 0.4;

/// The default clearance of a foldable print (mm): fingers that print side by side stay this far
/// apart, so they don't fuse (a PLA test print at 0.1 mm joined them a little).
pub const DEFAULT_PRINT_CLEARANCE: f64 = 0.15;

/// The narrowest a joint's finger is made (mm): an edge too short for the fingers asked for gets
/// fewer, at least two (a test print's 3 mm fingers were frail).
pub const MIN_FINGER_WIDTH: f64 = 5.0;

/// How many fingers a joint asked for `segments` gets on a stretch `len` long: no more than
/// leave each [`MIN_FINGER_WIDTH`] wide, and at least two.
pub fn finger_count(segments: u32, len: f64) -> u32 {
    let fit = (len / MIN_FINGER_WIDTH).floor();
    let fit = if fit.is_finite() && fit >= 2.0 { fit.min(f64::from(u32::MAX)) as u32 } else { 2 };
    segments.min(fit).max(2.min(segments.max(1)))
}

/// The default wall round the channel or the nub of a joint on a bend, inside its fingers (mm):
/// two lines of a 0.4 mm nozzle.
pub const DEFAULT_JOINT_WALL: f64 = 0.8;

/// How a foldable print's panels meet where two of them touch along an edge that is no bend
/// (an edge cut open in the flat).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PrintJoint {
    /// Square ends: the panel nearer the fixed face runs on to the corner, the other ends on its
    /// inside, the clearance short of it.
    #[default]
    Butt,
    /// Both ends cut on the plane halfway between the panels, each half the clearance short of
    /// it: their outer faces meet at the corner.
    Mitre,
}

impl PrintJoint {
    pub fn parse(s: &str) -> Option<PrintJoint> {
        match s.trim().to_ascii_lowercase().as_str() {
            "butt" => Some(PrintJoint::Butt),
            "mitre" | "miter" => Some(PrintJoint::Mitre),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            PrintJoint::Butt => "butt",
            PrintJoint::Mitre => "mitre",
        }
    }
}

/// The fingers a filament joint has by default along its edge.
pub const DEFAULT_SEGMENTS: u32 = 5;

/// The default filament diameter (mm).
pub const DEFAULT_FILAMENT: f64 = 1.75;

/// The default play of a filament channel (mm): test prints took the filament at no play, and in
/// places at 0.05 mm, only with a lot of force.
pub const DEFAULT_CHANNEL_TOLERANCE: f64 = 0.1;

/// A filament joint on an open edge of a foldable print, the edge picked at `at`: the edge
/// split evenly into `segments` fingers that belong to its two panels by turns, and a square
/// channel through all of them for a filament `diameter` thick, `tolerance` wider (negative:
/// narrower, for a press fit). The panel nearer the fixed face has the first finger at the
/// edge's first end, or the other one with `flip`; an odd number of fingers gives one panel
/// both ends.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrintFilament {
    pub at: Vec3,
    #[serde(default = "default_segments")]
    pub segments: u32,
    #[serde(default = "default_filament")]
    pub diameter: f64,
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub flip: bool,
    /// A press fit in place of the filament: the same fingers, no channel, a nub on each end
    /// face of the starting panel's fingers that snaps into a dimple in the next finger.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub press: Option<PressFit>,
    /// On a bend: the wall round the channel or the nub inside the fingers (mm).
    #[serde(default = "default_wall")]
    pub wall: f64,
}

/// The default press-fit nub: its radius on the finger's end face and how far it sticks out (mm).
pub const DEFAULT_NUB_RADIUS: f64 = 0.8;
pub const DEFAULT_NUB_HEIGHT: f64 = 0.3;

/// A press fit's nub and dimple: a cap of a ball, `radius` round on the finger's end face and
/// `height` high; the dimple is the same ball `tolerance` larger (negative: tighter).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PressFit {
    pub radius: f64,
    pub height: f64,
    #[serde(default)]
    pub tolerance: f64,
}

impl PressFit {
    /// The radius of the ball the nub is a cap of.
    pub fn ball(&self) -> f64 {
        (self.radius * self.radius + self.height * self.height) / (2.0 * self.height)
    }

    /// The dimple's radius where it opens: the larger ball's cap beyond the nub's face.
    pub fn dimple_radius(&self) -> f64 {
        let (big, back) = (self.ball() + self.tolerance, self.ball() - self.height);
        (big * big - back * back).max(0.0).sqrt()
    }

    /// Refuse a nub that can't be made or can't snap: `clearance` is the gap between the
    /// fingers, `room` how far from the middle of the wall (`t` thick) the nub may reach.
    pub fn check(&self, clearance: f64, room: f64, t: f64) -> std::result::Result<(), String> {
        if !(self.radius.is_finite() && self.radius > 0.0 && self.radius <= 20.0) {
            return Err("the nub's radius must be more than 0 and at most 20 mm".into());
        }
        if !(self.height.is_finite() && self.height > 0.0 && self.height <= self.radius) {
            return Err("the nub's height must be more than 0 and at most its radius".into());
        }
        if !(self.height > clearance) {
            return Err(format!(
                "the nub sticks out {:.2} mm, no more than the {clearance:.2} mm clearance between the fingers: it can't reach the dimple",
                self.height
            ));
        }
        if !(self.tolerance.is_finite() && self.tolerance > -self.height) {
            return Err("the dimple's tolerance must leave a dimple".into());
        }
        if !(self.radius.max(self.dimple_radius()) <= room) {
            return Err(format!(
                "a nub {:.2} mm round needs walls at least {:.2} mm thick (the nub, and the film either side)",
                self.radius,
                t + 2.0 * (self.radius - room)
            ));
        }
        Ok(())
    }
}

fn default_segments() -> u32 {
    DEFAULT_SEGMENTS
}

fn default_filament() -> f64 {
    DEFAULT_FILAMENT
}

fn default_tolerance() -> f64 {
    DEFAULT_CHANNEL_TOLERANCE
}

fn default_wall() -> f64 {
    DEFAULT_JOINT_WALL
}

impl PrintFilament {
    /// The square channel's side.
    pub fn side(&self) -> f64 {
        self.diameter + self.tolerance
    }
}

/// A filament joint's cuts in one panel of the flat: the open edge from `a` to `b` on the
/// outer skin as the face drew it, `n` its unit direction into the panel. `notches` are the
/// stretches along the edge (from `a`, mm) where the other panel's fingers are: this panel ends
/// `depth` in from the edge there. The channel runs along the whole edge, `offset` in from it
/// at mid-thickness, square, `side` wide.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FilamentCut {
    pub a: Vec2,
    pub b: Vec2,
    pub n: Vec2,
    pub notches: Vec<(f64, f64)>,
    pub depth: f64,
    pub offset: f64,
    pub side: f64,
    /// A press fit in place of the channel, and whether this panel's fingers carry the nubs (else
    /// the dimples).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub press: Option<PressFit>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub nubs: bool,
    /// The fold between this panel and the other (radians, 0 where both lie flat on each other):
    /// under 90° the corner both walls share reaches past this panel's edge, and its fingers take
    /// that part too.
    #[serde(default)]
    pub angle: f64,
    /// The panel this edge is on (its flange; none for the base).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub panel: Option<usize>,
}

impl FilamentCut {
    /// The stretches along the edge (from `a`, mm) where this panel's own fingers are.
    pub fn own_fingers(&self) -> Vec<(f64, f64)> {
        let len = (self.b - self.a).len();
        let mut notches = self.notches.clone();
        notches.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut out = Vec::new();
        let mut at = 0.0_f64;
        for (lo, hi) in notches {
            if lo > at + 1e-9 {
                out.push((at, lo.min(len)));
            }
            at = at.max(hi);
        }
        if at < len - 1e-9 {
            out.push((at, len));
        }
        out.retain(|(s, e)| e - s > 1e-9);
        out
    }

    /// How far past the edge (on its inner skin) this panel's fingers reach, the corner both
    /// walls share out to the other panel's outer face: `t` over the tangent of the fold.
    pub fn reach(&self, t: f64) -> f64 {
        if self.angle > 1e-6 && self.angle < std::f64::consts::FRAC_PI_2 - 1e-6 { t / self.angle.tan() } else { 0.0 }
    }
}

/// A filament joint on a bend of a foldable print (flange `flange`'s): the bend split evenly into
/// `segments` fingers along its line, each giving the folded corner to one of its two panels by
/// turns (the parent first, or the flange with `flip`), the film under them left whole. The
/// channel, square and `side` wide, runs along the bend on its core ([`fold_core`]), `wall`
/// inside the fingers round it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FoldFilament {
    pub flange: usize,
    pub segments: u32,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub flip: bool,
    pub side: f64,
    /// A press fit in place of the channel (the starting panel's fingers carry the nubs).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub press: Option<PressFit>,
    #[serde(default = "default_wall")]
    pub wall: f64,
    /// Where another face of the print meets an end of the bend.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ends: Vec<FoldEnd>,
}

impl FoldFilament {
    /// The joint's core on a bend folded by `angle` in a wall `t` thick.
    pub fn core(&self, t: f64, angle: f64, pf: PrintFold) -> std::result::Result<FoldCore, String> {
        match self.press {
            Some(fit) => fold_core(t, angle, pf, fit.radius.max(fit.dimple_radius()), true, self.wall),
            None => fold_core(t, angle, pf, self.side / 2.0, false, self.wall),
        }
    }
}

/// Another face of a foldable print at an end of a joined bend: the end `u` along the bend (from
/// its flange's `p` along `d`, mm), the sine of the angle between the bend's line and that face,
/// and the flange whose bend is joined there too, if one is.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FoldEnd {
    pub u: f64,
    pub sin: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joined: Option<usize>,
}

/// Where a joint on a bend runs: its core `d` from the bend's sharp (on the outer skin) along
/// the bisector of the fold, inside it; its fingers a sector `r` round the sharp. A press fit's
/// nub sits at `nub` on the parent's fingers (across the bend from the sharp toward the flange,
/// and up from the outer skin): on the core, or where the core is no place for it, nearer the
/// parent's wall ([`fold_core`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FoldCore {
    pub d: f64,
    pub r: f64,
    pub nub: Vec2,
}

/// The fold a joint on a bend takes (degrees).
pub const FOLD_JOINT_ANGLES: (f64, f64) = (30.0, 150.0);

/// The core of a joint on a bend folded by `angle` in a wall `t` thick: a channel (square,
/// `half` its side each way, lined up with the wall of the finger it runs in) or a nub (`round`,
/// `half` its radius), `wall` from the panel's outer skin and, with the film and the clearance
/// besides, from the other panel's. The fingers are a sector round the sharp large enough for
/// the core with its wall and for the corner where the two walls overlap once folded.
pub fn fold_core(t: f64, angle: f64, pf: PrintFold, half: f64, round: bool, wall: f64) -> std::result::Result<FoldCore, String> {
    let angle = angle.abs();
    let (lo, hi) = FOLD_JOINT_ANGLES;
    if !(angle.is_finite() && angle >= lo.to_radians() - 1e-6 && angle <= hi.to_radians() + 1e-6) {
        return Err(format!("it needs a fold of {lo}° to {hi}°, not {:.1}°", angle.to_degrees()));
    }
    if !(wall.is_finite() && wall > 0.0 && wall <= 20.0) {
        return Err("the wall round its channel or nub must be more than 0 and at most 20 mm".into());
    }
    if !(half.is_finite() && half > 0.0 && half <= 20.0) {
        return Err("its channel or nub must be more than 0 and at most 40 mm across".into());
    }
    let (sin, cos) = (angle.sin(), angle.cos());
    let (s2, c2) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    let m = pf.film + pf.clearance;
    // How far the core's outline reaches toward the other panel's outer skin.
    let reach = if round { half } else { half * (sin + cos.abs()) };
    let d = ((reach + m + wall) / c2).max((half + wall) / c2);
    let far = if round { d + half } else { (d * s2 + half).hypot(d * c2 + half) };
    // The corner both walls fill once folded, and the line beside the other's film at the inner skin.
    let keep_top = (t * cos - m) / sin;
    let corner = (t / c2).max(t / sin).max(keep_top.hypot(t));
    let over = (t * 0.05).max(1e-3);
    let r = (far + wall).max(corner + 0.05).max(t + over + 0.05);
    if !(d.is_finite() && r.is_finite() && r <= 100.0) {
        return Err("its fingers would reach more than 100 mm round the bend".into());
    }
    // A nub stands out of its finger's end face into the next finger's stretch of the bend. In
    // the flat that finger is the other panel's, mirrored across the sharp: it lies where
    // `x·sin + z·cos` is at least `m`, and the nub must print clear of it. At a small fold it
    // covers the core; the nub then goes as low as its wall lets it once folded (the other
    // panel's finger starts `m` above this one's outer skin), and as far toward this panel's
    // wall as clears the other's finger in the flat.
    let core = Vec2::new(-d * s2, d * c2);
    let gap = pf.clearance.max(0.05);
    let clear_of_mirror = |p: Vec2| p.x * sin + p.y * cos <= m - half - gap + 1e-9;
    let nub = if !round || clear_of_mirror(core) {
        core
    } else {
        let z = m + half + wall;
        let low = Vec2::new((m - half - gap - z * cos) / sin, z);
        if !(low.x * sin - low.y * cos <= -m - half) {
            return Err(format!("at a fold of {:.1}° its fingers have no room for a nub clear of the next finger in the flat", angle.to_degrees()));
        }
        low
    };
    // The fingers reach round the nub with its wall too.
    let r = r.max(nub.len() + half + wall);
    if !(r <= 100.0) {
        return Err("its fingers would reach more than 100 mm round the bend".into());
    }
    Ok(FoldCore { d, r, nub })
}
