//! Foldable 3D print: a sheet made as one flat body to print and fold. Its outer skin lies on the
//! print bed (z = 0 of the flat frame) and every bend turns toward +z. A V-groove is cut down from
//! the top at each bend's virtual sharp, its two walls at half the bend angle each side of upright,
//! so that once folded they meet as a mitre. Under the groove a film `film` thick is left: the
//! hinge the panels fold on. `clearance` moves each wall of the groove that far apart from the
//! other, measured square to the walls once folded.
//!
//! The sheet (`SurfaceSheet`, built from the faces of the body) is the definition of the print and
//! its flat is derived from it, never written back: which panels hang on which, which edges are
//! bends and by how much each turns come from the sheet's joints and its development (see
//! `sheet_print_layout`). This file holds the values a print takes, the joints it can be made
//! with (filament channels and press fits) and the geometry of its solid.

use serde::{Deserialize, Serialize};

use solidcore_record::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};
use solidcraft_develop::prelude::*;
use solidcraft_develop::{JointKind, Lap, Rigid2, Rigid3};
use solidcraft_geom::prelude::*;
use solidcraft_kernel::{self as kernel, Body};

use crate::sheet::FlatWork;
use crate::sheet_print_layout::{PrintBend, PrintLayout, PrintPart};
use crate::sheet_surface_convert::{SurfaceBendLine, SurfaceSheet};
use crate::{DocError, IDENTITY, Mat, Result, apply_point, apply_vector, mat_mul, rigid};

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

/// The least two panels of a foldable print that lie side by side in the flat stay apart where
/// their fingers lean toward each other (mm): a line of a 0.4 mm nozzle, so they print apart.
pub const FINGER_FLAT_GAP: f64 = 0.4;

/// A loose piece of a foldable print smaller than this is a crumb its cuts left, and is dropped (mm³).
pub const CRUMB: f64 = 5.0;

/// A corner fill smaller than this share of a cube of the wall's thickness is a fin, and is left out.
pub const FIN: f64 = 0.15;

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

/// Sockets along a straight edge of a hole through a face of a foldable print, the edge from
/// `a` to `b` (the body's coordinates): the hole's wall there is printed as a part of its own
/// (a carved-out group of Unfold for Printing), and fingers of it stand in the face along
/// `fingers` (stretches from `a`, mm). The face is notched for each, a wall and the clearance
/// deep, with a dimple for a press-fit nub in both sides of each notch, or with `filament` a
/// channel along the edge through what is left of the face there.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrintSocket {
    pub a: Vec3,
    pub b: Vec3,
    pub fingers: Vec<(f64, f64)>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub filament: bool,
}

/// Where a hole wall `len` long has its fingers in the face it meets (from the edge's first end):
/// every other stretch of the edge split as a joint's is, never the first or the last, so the
/// corners of the hole stay the face's. An edge too short for three stretches of
/// [`MIN_FINGER_WIDTH`] gets three narrower ones from 9 mm on, and none below.
pub fn socket_fingers(len: f64) -> Vec<(f64, f64)> {
    if !(len.is_finite() && len >= 9.0) {
        return Vec::new();
    }
    let n = finger_count(DEFAULT_SEGMENTS, len).max(3);
    let each = len / f64::from(n);
    (0..n).filter(|k| k % 2 == 1 && k + 1 < n).map(|k| (f64::from(k) * each, f64::from(k + 1) * each)).collect()
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

// ---------------------------------------------------------------------------------------------
// The print's solid, from the layout the sheet gives it (see `sheet_print_layout`).
// ---------------------------------------------------------------------------------------------

/// The gap between two parts of a split foldable print on the bed (mm).
pub const PART_GAP: f64 = 5.0;

/// How far a tool runs past an end of what it cuts (mm), so that no face of it lies on a face of
/// the print.
const END_OVER: f64 = 0.01;

/// What a foldable print is made with, kept on its sheet: the film and the clearance, how panels
/// meet on an open edge, and the joints the user picked on edges (in the body's own coordinates).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PrintSpec {
    pub fold: PrintFold,
    pub joint: PrintJoint,
    /// Filament joints and press fits, on open edges and on bends.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub filament: Vec<PrintFilament>,
    /// Straight edges of holes through a face where the face ends in a mitre.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mitres: Vec<Vec3>,
    /// Straight edges of holes through a face notched for the fingers of a wall printed apart.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub sockets: Vec<PrintSocket>,
}

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(DocError::Invalid(msg.into()))
}

fn extrude(plane: &Plane, regions: &[Region2], z0: f64, z1: f64) -> Result<Body> {
    Ok(kernel::extrude_planar_regions(plane, regions, z0, z1)?)
}

/// A frame as a matrix (see [`Mat`]).
pub(crate) fn mat_of(r: &Rigid3) -> Mat {
    [[r.x.x, r.x.y, r.x.z, 0.0], [r.y.x, r.y.y, r.y.z, 0.0], [r.z.x, r.z.y, r.z.z, 0.0], [r.shift.x, r.shift.y, r.shift.z, 1.0]]
}

/// Refuse values a foldable print can't be made with, saying which and why.
pub fn check_print_fold(t: f64, pf: PrintFold) -> Result<()> {
    if !(pf.film.is_finite() && pf.film > 0.0 && pf.film < t) {
        return err(format!("the film thickness must be more than 0 and less than the wall thickness ({t:.3} mm)"));
    }
    if !(pf.clearance.is_finite() && pf.clearance >= 0.0) {
        return err("the clearance must not be negative");
    }
    Ok(())
}

/// How far a groove's half-width grows at every height for `clearance`: the walls lean by half the
/// bend angle, so a shift square to a wall is 1/cos of it across the flat.
fn clearance_shift(clearance: f64, angle: f64) -> f64 {
    clearance / 2.0 / (angle.abs() / 2.0).cos()
}

/// The laps the print's rips take, as the print joint says (see [`PrintJoint`]): at an edge where
/// two faces meet round the outside of the body (the sheet turns toward the material across it by
/// `φ`) a mitre is the sheet's own mitre, each panel `clearance / 2` off the plane halfway between
/// them; a butt joint keeps the panel reached first whole and ends the other on its inner face,
/// `(t + clearance − min(t·cos φ, 0)) / sin φ` short of the edge, a wall and the clearance away.
/// At an inside corner, or where the faces run on flat, the panels are cut as the sheet cuts them
/// but never closer than [`PRINT_FLAT_GAP`] each: a slicer prints nearer lines as one.
fn print_laps(sheet: &mut solidcraft_develop::Sheet, spec: &PrintSpec, places: &[Option<Rigid3>]) {
    let (t, c) = (sheet.thickness, spec.fold.clearance);
    // The edges a joint holds: both panels run on to the corner, the fingers share it out.
    let joined: Vec<usize> = spec.filament.iter().filter_map(|f| joint_at(sheet, places, f.at)).collect();
    for (j, joint) in sheet.joints.iter_mut().enumerate() {
        let (a_first, JointKind::Rip(rip)) = (joint.a.panel < joint.b.panel, &mut joint.kind) else { continue };
        // One turn for the whole edge, or the rip is left as it is.
        let Some(&turn) = rip.turns.first() else { continue };
        if rip.turns.iter().any(|x| (x - turn).abs() > 1e-6) {
            continue;
        }
        if joined.contains(&j) {
            rip.lap = Lap::Cut(0.0, 0.0);
            continue;
        }
        rip.lap = if turn > 1e-6 && turn < std::f64::consts::PI - 1e-6 {
            match spec.joint {
                PrintJoint::Mitre => {
                    // Each end square at the outer skin `c / 2 / sin half` short of the corner
                    // (`half`: half the angle between the two faces), the mitre's slope cut after.
                    let s = c / 2.0 / ((std::f64::consts::PI - turn) / 2.0).sin();
                    Lap::Cut(s, s)
                }
                PrintJoint::Butt => {
                    let second = (t + c - (t * turn.cos()).min(0.0)) / turn.sin();
                    if a_first { Lap::Cut(0.0, second) } else { Lap::Cut(second, 0.0) }
                }
            }
        } else {
            Lap::Miter { gap: c.max(2.0 * PRINT_FLAT_GAP) }
        };
    }
}

/// The least a panel of a foldable print ends short of an inside corner (mm): a line of a 0.4 mm
/// nozzle, so that it prints apart from a panel it lies beside in the flat.
pub const PRINT_FLAT_GAP: f64 = 0.4;

impl SurfaceSheet {
    /// The foldable print of this sheet (see [`PrintSpec`]): the flat print while the sheet is
    /// unfolded and the print folded back up otherwise. What the print's flat is, and where it
    /// folds, comes from the sheet's development; nothing is written back to the sheet but the
    /// flat pattern a `sheet.flat_pattern` shows.
    pub(crate) fn build_print(&mut self) -> Result<Body> {
        let Some(spec) = self.print.clone() else { return err("the sheet is no foldable print") };
        let t = self.sheet.thickness;
        check_print_fold(t, spec.fold)?;
        let mut sheet = self.sheet.clone();
        print_laps(&mut sheet, &spec, &self.places);
        // A cut back farther than its panel reaches leaves no panel to cut.
        for joint in &sheet.joints {
            let JointKind::Rip(solidcraft_develop::Rip { lap: Lap::Cut(a, b), .. }) = &joint.kind else { continue };
            for (cut, owner) in [(*a, joint.a.panel), (*b, joint.b.panel)] {
                let reach = sheet.panels.get(owner).map(|p| {
                    let size = crate::sheet_surface_cuts::loop_box(&p.region.outer).size();
                    size.x.min(size.y)
                });
                if reach.is_some_and(|r| cut >= r) {
                    return err("the clearance leaves gaps at the cut edges wider than the panels they cut: make it smaller");
                }
            }
        }
        let anchors = crate::sheet_surface_convert::flat_anchors(&sheet, &self.places, &self.flat);
        let dev = solidcraft_develop::develop(&sheet, &anchors).map_err(|e| {
            if spec.fold.clearance > 0.0 {
                DocError::Invalid(format!("the clearance leaves gaps at the cut edges the panels can't take ({e}): make it smaller"))
            } else {
                DocError::Invalid(e.to_string())
            }
        })?;
        if let Some(m) = dev.loops.first() {
            let name = sheet.panels.get(sheet.joints.get(m.joint).map_or(0, |j| j.a.panel)).map_or("", |p| p.name.as_str());
            return err(format!("the sheet closes a ring of bends at {name}: pick a split edge in it"));
        }
        for (i, p) in dev.panels.iter().enumerate() {
            if p.as_ref().is_some_and(|p| p.region.outer.segs.is_empty()) && sheet.panels.get(i).is_some_and(|q| !q.region.outer.segs.is_empty()) {
                return err("the clearance leaves gaps at the cut edges wider than the panels they cut: make it smaller");
            }
        }
        let laid = SurfaceSheet { sheet, ..self.clone() };
        let layout = laid.print_layout(&dev)?;
        let shifts = parts_beside(&layout, t);
        if let Some(msg) = self.print_overlap(&layout) {
            return err(msg);
        }
        let joints = layout.parts.iter().map(|part| part_joints(part, &laid.sheet, &self.places, &spec)).collect::<Result<Vec<_>>>()?;
        self.parts = Vec::new();
        self.frames = Vec::new();
        self.laid_out_print(&layout, &shifts)?;
        let pf = spec.fold;
        if self.unfolded {
            let mut flats: Vec<Body> = Vec::with_capacity(layout.parts.len());
            for ((part, shift), joints) in layout.parts.iter().zip(&shifts).zip(&joints) {
                let body = print_flat(part, t, pf, joints)?;
                flats.push(kernel::transform(&body, Vec3::new(shift.x, shift.y, 0.0), Vec3::ZERO, Vec3::Z, 0.0)?);
            }
            let refs: Vec<&Body> = flats.iter().collect();
            let body = joined_flats(&refs)?;
            return Ok(kernel::transform_matrix(&body, mat_of(&self.flat))?);
        }
        let mut pieces = Vec::with_capacity(layout.parts.len());
        for (part, joints) in layout.parts.iter().zip(&joints) {
            let root = part.panels.first().map_or(0, |p| p.panel);
            let at = self.places.get(root).copied().flatten().ok_or_else(|| DocError::Invalid("a panel of the print has no place".into()))?;
            let frame = dev.placements.get(root).copied().flatten().ok_or_else(|| DocError::Invalid("a panel of the print has no place".into()))?;
            let world = mat_of(&crate::sheet_surface_cuts::lift(&at, &frame.inverse()));
            let folded = print_folded(part, t, pf, joints)?;
            pieces.push(kernel::transform_matrix(&folded, world)?);
        }
        joined(pieces)
    }

    /// The flat pattern of the print, kept for the model's flat pattern: its flat in one piece,
    /// each part beside the first as it prints, and its bends where the print folds.
    fn laid_out_print(&mut self, layout: &PrintLayout, shifts: &[Vec2]) -> Result<()> {
        let t = self.sheet.thickness;
        let mut work = FlatWork::new();
        let mut all: Vec<Region2> = Vec::new();
        let mut bends = Vec::new();
        for (part, shift) in layout.parts.iter().zip(shifts) {
            let by = Rigid2 { cos: 1.0, sin: 0.0, shift: *shift };
            all.extend(part.panels.iter().map(|p| by.region(&p.region)));
            for b in &part.bends {
                let ba = 2.0 * t * (b.angle / 2.0).tan();
                for &(u0, u1) in &b.spans {
                    let (start, end) = (b.s + b.d * u0 + *shift, b.s + b.d * u1 + *shift);
                    let w = b.n * (ba / 2.0);
                    let zone = Region2 { outer: Loop2::polygon(&[start - w, end - w, end + w, start + w]).ccw(), holes: Vec::new() };
                    bends.push(SurfaceBendLine {
                        zone,
                        start,
                        end,
                        angle: b.angle,
                        radius: 0.0,
                        up: false,
                        allowance: ba,
                        tabs: Vec::new(),
                        joint: b.joint,
                    });
                }
            }
        }
        self.regions = crate::sheet_surface::planar_union(all, &mut work)?;
        self.bends = bends;
        self.slits = Vec::new();
        self.roll_lines = Vec::new();
        self.rolls = 0;
        self.thickness = t;
        Ok(())
    }

    /// A message naming two panels of one part that overlap in the flat, where there are any: the
    /// pattern would print one on the other (a split edge between them lets them print apart).
    fn print_overlap(&self, layout: &PrintLayout) -> Option<String> {
        let eps = (self.sheet.thickness * 1e-3).max(1e-6);
        let mut work = FlatWork::new();
        for part in &layout.parts {
            for (i, a) in part.panels.iter().enumerate() {
                for b in part.panels.iter().skip(i + 1) {
                    // Pieces that only touch along an edge or at a corner don't overlap.
                    let common = crate::sheet_surface::planar_boolean(
                        std::slice::from_ref(&a.region),
                        std::slice::from_ref(&b.region),
                        kernel::BoolOp::Intersect,
                        &mut work,
                    )
                    .ok()?;
                    if common.iter().map(|r| r.outer.signed_area().abs()).sum::<f64>() > eps * eps.max(1.0) * 1e3 {
                        let at = |p: &crate::sheet_print_layout::PrintPanel| {
                            let frame = self.places.get(p.panel).copied().flatten()?;
                            let q = frame.point(self.sheet.panels.get(p.panel)?.region.interior_point().extend(0.0));
                            Some(format!("({:.3}, {:.3}, {:.3})", q.x, q.y, q.z))
                        };
                        let name = |p: &crate::sheet_print_layout::PrintPanel| at(p).unwrap_or_else(|| format!("panel {}", p.panel));
                        return Some(format!(
                            "the print overlaps itself: the faces at {} and {} unfold onto each other; pick a split edge between them (Split Edges) so that they print apart",
                            name(a),
                            name(b)
                        ));
                    }
                }
            }
        }
        None
    }
}

/// The shift (in the flat) that puts each part beside the first, as the parts print: the first
/// where its flat lies, every other one beside it in a row along y, [`PART_GAP`] (or two walls, if
/// more) apart, all lined up on the first's least x.
fn parts_beside(layout: &PrintLayout, t: f64) -> Vec<Vec2> {
    let bounds = |part: &PrintPart| {
        let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for p in &part.panels {
            for q in p.region.outer.polyline(1e-3).iter() {
                lo = Vec2::new(lo.x.min(q.x), lo.y.min(q.y));
                hi = Vec2::new(hi.x.max(q.x), hi.y.max(q.y));
            }
        }
        (lo, hi)
    };
    let gap = PART_GAP.max(2.0 * t);
    let mut out = Vec::with_capacity(layout.parts.len());
    let mut first: Option<(Vec2, Vec2)> = None;
    let mut y = 0.0;
    for part in &layout.parts {
        let (lo, hi) = bounds(part);
        match first {
            None => {
                first = Some((lo, hi));
                y = hi.y;
                out.push(Vec2::new(0.0, 0.0));
            }
            Some((first_lo, _)) => {
                out.push(Vec2::new(first_lo.x - lo.x, y + gap - lo.y));
                y += gap + (hi.y - lo.y);
            }
        }
    }
    out
}

/// Pieces that meet face to face as one body: their union, or where the kernel can't make one
/// (faces that lie on each other), the pieces side by side in one body.
fn joined(pieces: Vec<Body>) -> Result<Body> {
    if pieces.len() < 2 {
        return pieces.into_iter().next().ok_or_else(|| DocError::Invalid("the foldable print has no pieces".into()));
    }
    let refs: Vec<&Body> = pieces.iter().collect();
    if let Ok(body) = kernel::union_all(&refs) {
        return Ok(body);
    }
    kernel::join_apart(&pieces)?.ok_or_else(|| DocError::Invalid("the foldable print's pieces can't be joined".into()))
}

/// The flats of a print's parts as one body.
fn joined_flats(parts: &[&Body]) -> Result<Body> {
    let Some((first, rest)) = parts.split_first() else { return err("a foldable print needs at least one part") };
    let mut body = (*first).clone();
    for p in rest {
        body = kernel::boolean(&body, p, kernel::BoolOp::Union)?
            .ok_or_else(|| DocError::Invalid("the parts of the foldable print leave nothing".into()))?;
    }
    Ok(body)
}

/// A V-groove along the bend, from `u0` to `u1` along it from its start `s`.
fn v_groove(body: Body, b: &PrintBend, pf: PrintFold, t: f64, over: f64, u0: f64, u1: f64, index: usize) -> Result<Body> {
    let half = (b.angle.abs() / 2.0).tan();
    let shift = clearance_shift(pf.clearance, b.angle);
    let top = t + over;
    let (low, high) = (pf.film * half + shift, top * half + shift);
    let profile = Loop2::polygon(&[Vec2::new(-low, pf.film), Vec2::new(low, pf.film), Vec2::new(high, top), Vec2::new(-high, top)]);
    let mid = b.s + b.d * ((u0 + u1) / 2.0);
    let plane = Plane { origin: Vec3::new(mid.x, mid.y, 0.0), x: Vec3::new(b.d.y, -b.d.x, 0.0), y: Vec3::Z };
    let reach = (u1 - u0) / 2.0;
    let tool = extrude(&plane, &[Region2 { outer: profile.ccw(), holes: Vec::new() }], -reach, reach)?;
    kernel::boolean(&body, &tool, kernel::BoolOp::Cut)?
        .ok_or_else(|| DocError::Invalid(format!("the V-groove at bend {index} leaves nothing of the print")))
}

/// A mitred end of a panel in the flat: the edge from `a` to `b` on the outer skin (z = 0, as the
/// face drew it), `n` its unit direction into the panel. The panel's end is square at `at` in from
/// the edge; the mitre takes it `extra` further in at the inner skin (z = T), in a straight line
/// between.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bevel {
    pub a: Vec2,
    pub b: Vec2,
    pub n: Vec2,
    pub at: f64,
    pub extra: f64,
}

/// What one part of a print has besides its panels and bends: the mitred ends of its open edges,
/// and its joints on them.
pub(crate) struct PartJoints {
    pub bevels: Vec<Bevel>,
    pub filament: Vec<FilamentCut>,
    /// The joints on the bends, by bend.
    pub fold: Vec<Option<FoldFilament>>,
}

/// The mitred ends and the joints of `part`, where `sheet` (as the print has it) is its sheet.
fn part_joints(part: &PrintPart, sheet: &solidcraft_develop::Sheet, places: &[Option<Rigid3>], spec: &PrintSpec) -> Result<PartJoints> {
    let t = sheet.thickness;
    let mut filament = filament_cuts(part, sheet, places, spec)?;
    let (mut bevels, holes) = (Vec::new(), hole_cuts(part, sheet, places, spec));
    bevels.extend(holes.0);
    filament.extend(holes.1);
    let fold = fold_filaments(part, sheet, places, spec)?;
    if spec.joint == PrintJoint::Mitre {
        for e in &part.edges {
            let Some(JointKind::Rip(rip)) = sheet.joints.get(e.joint).map(|j| &j.kind) else { continue };
            // A joint's own cuts stand for the mitre.
            if filament.iter().any(|f| f.panel == Some(e.panel) && (f.a - e.a).len() < 1e-6 && (f.b - e.b).len() < 1e-6) {
                continue;
            }
            let Some(&turn) = rip.turns.first() else { continue };
            if rip.turns.iter().any(|x| (x - turn).abs() > 1e-6) || !(turn > 1e-6 && turn < std::f64::consts::PI - 1e-6) {
                continue;
            }
            let half = (std::f64::consts::PI - turn) / 2.0;
            bevels.push(Bevel { a: e.a, b: e.b, n: e.n, at: spec.fold.clearance / 2.0 / half.sin(), extra: t / half.tan() });
        }
    }
    Ok(PartJoints { bevels, filament, fold })
}

/// The flat body of one part of a foldable print, in the part's own flat coordinates (the outer
/// skin on z = 0): its panels' flat, a V-groove along each bend, its mitred ends and its joints.
fn print_flat(part: &PrintPart, t: f64, pf: PrintFold, joints: &PartJoints) -> Result<Body> {
    let mut work = FlatWork::new();
    let regions: Vec<Region2> = part.panels.iter().map(|p| p.region.clone()).collect();
    let union = crate::sheet_surface::planar_union(regions, &mut work)?;
    let mut body = extrude(&Plane::XY, &union, 0.0, t)?;
    // The groove's tool runs a little past the top, and past the bend's ends.
    let over = (t * 0.05).max(1e-3);
    // Every channel is cut last.
    let mut channels: Vec<Body> = Vec::new();
    let joined_r = |k: usize| {
        let (ff, b) = (joints.fold.get(k)?.as_ref()?, part.bends.get(k)?);
        ff.core(t, b.angle, pf).ok().map(|c| c.r)
    };
    for (k, b) in part.bends.iter().enumerate() {
        for &(u0, u1) in &b.spans {
            match joints.fold.get(k).and_then(Option::as_ref) {
                // A joint on the bend: fingers in place of its V-groove.
                Some(ff) => {
                    body = fold_fingers(t, body, b.s + b.d * u0, b.s + b.d * u1, b, ff, pf, over, k + 1, &mut channels, &joined_r)?;
                }
                None => body = v_groove(body, b, pf, t, over, u0 - END_OVER, u1 + END_OVER, k + 1)?,
            }
        }
    }
    // Mitred ends: the wedge between the square end and the mitre, from nothing at the outer skin
    // to `extra` at the inner one (and on past both, so no face of it lies on the sheet's).
    for v in &joints.bevels {
        let along = v.b - v.a;
        let len = along.len();
        if !(v.extra > 0.0 && len > 0.0) {
            continue;
        }
        let top = t + over;
        let reach_in = v.at + v.extra * top / t;
        let profile = Loop2::polygon(&[Vec2::new(v.at - over, 0.0), Vec2::new(v.at, 0.0), Vec2::new(reach_in, top), Vec2::new(v.at - over, top)]);
        let mid = v.a + along * 0.5;
        let plane = Plane { origin: Vec3::new(mid.x, mid.y, 0.0), x: Vec3::new(v.n.x, v.n.y, 0.0), y: Vec3::Z };
        let reach = len / 2.0 + END_OVER;
        let tool = extrude(&plane, &[Region2 { outer: profile.ccw(), holes: Vec::new() }], -reach, reach)?;
        body = kernel::boolean(&body, &tool, kernel::BoolOp::Cut)?
            .ok_or_else(|| DocError::Invalid("a mitred end leaves nothing of the print".into()))?;
    }
    // Joints on open edges.
    body = cut_filament(body, &joints.filament, t, pf, over, &mut channels)?;
    for tool in channels {
        body = kernel::boolean(&body, &tool, kernel::BoolOp::Cut)?
            .ok_or_else(|| DocError::Invalid("a filament joint leaves nothing of the print".into()))?;
    }
    // Crumbs the cuts left loose (a sliver between a channel and a groove) are no part of the
    // print: they would print as loose bits beside it.
    let lumps = body.lumps()?;
    if lumps.len() > 1 {
        let mut kept = Vec::with_capacity(lumps.len());
        for lump in lumps {
            if kernel::measure(&lump)?.volume >= CRUMB {
                kept.push(lump);
            }
        }
        if kept.len() == 1 {
            body = kept.remove(0);
        } else if let Some(j) = kernel::join_apart(&kept)? {
            body = j;
        }
    }
    Ok(body)
}

/// The motion of each panel of a part folding back up, in the part's flat: each bend turns its
/// panel (with the panels that hang on it) about the bend's line on the outer skin, up toward the
/// material.
fn fold_motions(part: &PrintPart) -> Vec<Mat> {
    let mut out: Vec<Mat> = Vec::with_capacity(part.panels.len());
    for p in &part.panels {
        let parent = p.parent.and_then(|k| out.get(k)).copied().unwrap_or(IDENTITY);
        let own = match p.bend.and_then(|k| part.bends.get(k)) {
            Some(b) => {
                let axis = Vec3::new(b.n.x, b.n.y, 0.0).cross(Vec3::Z);
                rigid(Vec3::ZERO, Vec3::new(b.s.x, b.s.y, 0.0), axis, b.angle)
            }
            None => IDENTITY,
        };
        out.push(mat_mul(&parent, &own));
    }
    out
}

/// A part of the print folded back up, in its flat's coordinates: each panel's piece of the flat
/// print (the panel to its sharp lines, the film under its half of each bend) turned about the
/// bends that carry it, so that the grooves close.
fn print_folded(part: &PrintPart, t: f64, pf: PrintFold, joints: &PartJoints) -> Result<Body> {
    let flat = print_flat(part, t, pf, joints)?;
    let over = (t * 0.05).max(1e-3);
    let motions = fold_motions(part);
    // A joined bend's fingers stand out of the walls, and reach past the sharp: those parts go with
    // the panel whose finger it is.
    let joined_r = |k: usize| {
        let (ff, b) = (joints.fold.get(k)?.as_ref()?, part.bends.get(k)?);
        ff.core(t, b.angle, pf).ok().map(|c| c.r)
    };
    let mut top = t + over;
    let mut moves: Vec<(usize, bool, Body)> = Vec::new();
    for (k, b) in part.bends.iter().enumerate() {
        let Some(ff) = joints.fold.get(k).and_then(Option::as_ref) else { continue };
        if let Ok(core) = ff.core(t, b.angle, pf) {
            top = top.max(core.r + pf.clearance + over);
        }
        for &(u0, u1) in &b.spans {
            for (parent, tool) in fold_moves(t, b.s + b.d * u0, b.s + b.d * u1, b, ff, pf, &joined_r)? {
                moves.push((k, parent, tool));
            }
        }
    }
    let step = |what: &str, k: usize, e: kernel::KernelError| DocError::Invalid(format!("folding the print back up ({what}, panel {}): {e}", k + 1));
    let mut pieces = Vec::with_capacity(part.panels.len());
    for (k, (p, m)) in part.panels.iter().zip(&motions).enumerate() {
        let mut tool = extrude(&Plane::XY, std::slice::from_ref(&p.region), -over, top)?;
        // An open edge's fingers past their panel's edge go with that panel. Each strip stops a
        // little short of the tool's top and bottom, at its own height: faces lying on each other
        // are where the kernel's booleans fail.
        let mut n = 0usize;
        for fc in joints.filament.iter().filter(|fc| fc.panel == Some(k)) {
            let reach = fc.reach(t);
            let Some(d) = (fc.b - fc.a).normalized() else { continue };
            if !(reach > 1e-9) {
                continue;
            }
            for (s, e) in fc.own_fingers() {
                let (a, b) = (fc.a + d * s, fc.a + d * e);
                let (inside, outside) = (fc.n * 0.01, fc.n * -(reach + 0.01));
                let step_z = over * 0.05 * (n % 8) as f64;
                n += 1;
                let strip = extrude(
                    &Plane::XY,
                    &[Region2 { outer: Loop2::polygon(&[a + inside, b + inside, b + outside, a + outside]).ccw(), holes: Vec::new() }],
                    -over * 0.6 + step_z,
                    t + over - over * 0.4 - step_z,
                )?;
                tool = kernel::boolean(&tool, &strip, kernel::BoolOp::Union).map_err(|e| step("an open edge's fingers", k, e))?.unwrap_or(tool);
            }
        }
        for (bend, parent, tool_part) in &moves {
            let Some(g) = part.bends.get(*bend) else { continue };
            let (gains, loses) = if k == g.parent {
                (*parent, !*parent)
            } else if k == g.child {
                (!*parent, *parent)
            } else {
                (false, false)
            };
            if gains {
                tool = kernel::boolean(&tool, tool_part, kernel::BoolOp::Union).map_err(|e| step("a finger past a sharp", k, e))?.unwrap_or(tool);
            } else if loses {
                tool = match kernel::boolean(&tool, tool_part, kernel::BoolOp::Cut).map_err(|e| step("a finger past a sharp", k, e))? {
                    Some(x) => x,
                    None => continue,
                };
            }
        }
        let Some(piece) = kernel::boolean(&flat, &tool, kernel::BoolOp::Intersect).map_err(|e| step("its piece of the flat", k, e))? else {
            continue;
        };
        pieces.push(kernel::transform_matrix(&piece, *m)?);
    }
    joined(pieces)
}

// ---------------------------------------------------------------------------------------------
// Joints on the open edges of a print: filament channels and press fits.
// ---------------------------------------------------------------------------------------------

/// How near (mm) a point picked on an edge lies to it and is on it.
const PICK_NEAR: f64 = 1e-2;

/// Where along a filament edge (from its first point `ends.0`, mm) the fingers of the panel across
/// it are, notched out of this panel: the edge split evenly from the end that sorts first, the
/// first finger the starting panel's (the one reached first from the fixed face, `nearer`, unless
/// flipped), each notch `clearance / 2` wider each side so the fingers stay that far apart.
fn finger_notches(fl: &PrintFilament, ends: (Vec3, Vec3), nearer: bool, clearance: f64, tol: f64) -> std::result::Result<Vec<(f64, f64)>, String> {
    const MIN_FINGER: f64 = 1.0;
    let (p, q) = ends;
    let len = p.dist(q);
    if !(2..=200).contains(&fl.segments) {
        return Err("a filament joint needs 2…200 fingers".into());
    }
    // A short edge gets fewer fingers, none narrower than MIN_FINGER_WIDTH (both panels count
    // the same, from the same edge).
    let n = finger_count(fl.segments, len);
    if fl.press.is_none() && !(fl.diameter.is_finite() && fl.diameter > 0.0 && fl.diameter <= 20.0) {
        return Err("the filament diameter must be more than 0 and at most 20 mm".into());
    }
    if fl.press.is_none() && !(fl.tolerance.is_finite() && fl.side() >= 0.1) {
        return Err("the tolerance closes the channel; it must leave it at least 0.1 mm wide".into());
    }
    let each = len / f64::from(n);
    if !(each - clearance >= MIN_FINGER) {
        return Err(format!("{n} fingers on {len:.2} mm leave each {:.2} mm, less than {MIN_FINGER} mm; take fewer", each - clearance));
    }
    // The end the edge is split from: the one that sorts first, the same seen from either panel.
    let d = q - p;
    let first_is_p = [d.x, d.y, d.z].into_iter().find(|c| c.abs() > tol).is_none_or(|c| c > 0.0);
    let mine = nearer != fl.flip;
    let over = tol * 10.0 + clearance;
    let mut notches = Vec::new();
    for k in 0..n {
        // Even fingers are the starting panel's.
        if (k % 2 == 0) == mine {
            continue;
        }
        let (s, e) = (f64::from(k) * each, f64::from(k + 1) * each);
        let (s, e) = if first_is_p { (s, e) } else { (len - e, len - s) };
        let lo = if s <= tol { -over } else { s - clearance / 2.0 };
        let hi = if e >= len - tol { len + over } else { e + clearance / 2.0 };
        notches.push((lo, hi));
    }
    Ok(notches)
}

/// A filament joint's corner, seen from panel `x` on its edge with `y`, the two meeting on the
/// outside of the body (the sheet turns toward the material across the edge by `turn`): how far in
/// from the edge `x` ends where a finger of `y` runs on to the corner (as a butt joint's second
/// panel, `clearance` short of it), and how far in from the edge the channel's middle lies: at
/// half the wall's thickness behind both faces.
fn filament_cut(turn: f64, t: f64, clearance: f64) -> std::result::Result<(f64, f64), String> {
    if !(turn > 1e-9 && turn < std::f64::consts::PI - 1e-9) {
        return Err("a filament joint needs faces that meet on the outside of the body, not in an inside corner".into());
    }
    let (sin, cos) = (turn.sin(), turn.cos());
    let depth = (t + clearance - (t * cos).min(0.0)) / sin;
    Ok((depth, t / (2.0 * (1.0 + cos)) * sin))
}

/// Where a joint's edge lies in the body's coordinates: its points in order.
fn joint_edge_points(sheet: &solidcraft_develop::Sheet, places: &[Option<Rigid3>], j: usize) -> Vec<Vec3> {
    let Some(joint) = sheet.joints.get(j) else { return Vec::new() };
    let Some(at) = places.get(joint.a.panel).copied().flatten() else { return Vec::new() };
    let on = |p: Vec2| at.point(p.extend(0.0));
    match &joint.kind {
        JointKind::Rip(r) => r.edges.0.iter().copied().map(on).collect(),
        JointKind::Bend(b) => vec![on(joint.a.origin), on(joint.a.origin + joint.a.dir * b.length)],
        JointKind::Seam => Vec::new(),
    }
}

/// How far `p` lies from the polyline `chain`.
fn chain_distance(chain: &[Vec3], p: Vec3) -> f64 {
    chain
        .windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let d = b - a;
            let u = if d.len2() > 0.0 { ((p - a).dot(d) / d.len2()).clamp(0.0, 1.0) } else { 0.0 };
            p.dist(a + d * u)
        })
        .fold(f64::INFINITY, f64::min)
}

/// The joint of the sheet whose edge a point picked on the body lies on (the nearest, within
/// [`PICK_NEAR`]).
pub(crate) fn joint_at(sheet: &solidcraft_develop::Sheet, places: &[Option<Rigid3>], p: Vec3) -> Option<usize> {
    (0..sheet.joints.len())
        .map(|j| (j, chain_distance(&joint_edge_points(sheet, places, j), p)))
        .filter(|(_, d)| *d <= PICK_NEAR)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(j, _)| j)
}

/// The cuts of the filament joints and press fits picked on open edges, in the part's flat: for each
/// the two panels' sides of the edge ([`FilamentCut`]).
pub(crate) fn filament_cuts(
    part: &PrintPart,
    sheet: &solidcraft_develop::Sheet,
    places: &[Option<Rigid3>],
    spec: &PrintSpec,
) -> Result<Vec<FilamentCut>> {
    let (t, clearance) = (sheet.thickness, spec.fold.clearance);
    let mut out = Vec::new();
    for fl in &spec.filament {
        let Some(j) = joint_at(sheet, places, fl.at) else {
            return err(format!("the joint at ({:.3}, {:.3}, {:.3}) is on no edge of the print", fl.at.x, fl.at.y, fl.at.z));
        };
        let Some(JointKind::Rip(rip)) = sheet.joints.get(j).map(|x| &x.kind) else { continue };
        let at = |what: &str| format!("the filament edge at ({:.3}, {:.3}, {:.3}): {what}", fl.at.x, fl.at.y, fl.at.z);
        let turn = rip.turns.first().copied().unwrap_or(0.0);
        let (depth, offset) = filament_cut(turn, t, clearance).map_err(|why| DocError::Invalid(at(&why)))?;
        for e in part.edges.iter().filter(|e| e.joint == j) {
            let Some(p) = part.panels.get(e.panel) else { continue };
            let Some(frame) = places.get(p.panel).copied().flatten() else { continue };
            let body = |q: Vec2| {
                let c = p.place.inverse().point(q - p.shift);
                frame.point(c.extend(0.0))
            };
            let other = sheet.joints.get(j).map(|x| if x.a.panel == p.panel { x.b.panel } else { x.a.panel }).unwrap_or(usize::MAX);
            let nearer = p.panel < other;
            let notches =
                finger_notches(fl, (body(e.a), body(e.b)), nearer, clearance, 1e-6 * t.max(1.0)).map_err(|why| DocError::Invalid(at(&why)))?;
            out.push(FilamentCut {
                a: e.a,
                b: e.b,
                n: e.n,
                notches,
                depth,
                offset,
                side: if fl.press.is_some() { 0.0 } else { fl.side() },
                press: fl.press,
                // The starting panel's fingers carry the nubs.
                nubs: nearer != fl.flip,
                angle: turn,
                panel: Some(e.panel),
            });
        }
    }
    Ok(out)
}

/// `body` with the filament joints' cuts made: the other panel's fingers notched out of each panel,
/// the fingers' corner, the press fits' nubs and dimples, and the channels last (as `channels`).
pub(crate) fn cut_filament(mut body: Body, fcs: &[FilamentCut], t: f64, pf: PrintFold, over: f64, channels: &mut Vec<Body>) -> Result<Body> {
    let cut = |body: Body, tool: Body| -> Result<Body> {
        kernel::boolean(&body, &tool, kernel::BoolOp::Cut)?.ok_or_else(|| DocError::Invalid("a filament joint leaves nothing of the print".into()))
    };
    for fc in fcs {
        let along = fc.b - fc.a;
        let len = along.len();
        let Some(d) = along.normalized() else { continue };
        if let Some(fit) = fc.press {
            fit.check(pf.clearance, t / 2.0 - pf.film, t).map_err(|why| DocError::Invalid(format!("a press fit: {why}")))?;
        } else if !(fc.side > 0.0 && fc.side + 2.0 * pf.film <= t) {
            return err(format!(
                "a filament channel {:.2} mm wide needs walls at least {:.2} mm thick (the channel and the film's thickness either side)",
                fc.side,
                fc.side + 2.0 * pf.film
            ));
        }
        for (u0, u1) in &fc.notches {
            let p = |u: f64, w: f64| fc.a + d * u + fc.n * w;
            let rect = Loop2::polygon(&[p(*u0, -over), p(*u1, -over), p(*u1, fc.depth), p(*u0, fc.depth)]);
            let tool = extrude(&Plane::XY, &[Region2 { outer: rect.ccw(), holes: Vec::new() }], -over, t + over)?;
            body = cut(body, tool)?;
        }
        // At a fold under 90° the corner both walls share reaches past this panel's edge, up to the
        // other's outer face: each of this panel's fingers takes it, so the corner is closed. In
        // the flat it stands past the edge, leaning out by the fold (its top dips a hair into the
        // panel, so no face of it lies on the sheet's).
        let reach = fc.reach(t);
        if reach > 1e-9 {
            let eps = (over * 0.1).min(0.005);
            let wedge = Loop2::polygon(&[Vec2::new(0.0, 0.0), Vec2::new(reach, t), Vec2::new(-eps, t - eps)]);
            // In the flat the panel across the joint lies beside this one, turned away by the
            // corner of the face both hang on: past the line halfway between their two edges a
            // finger leaning out would stand in that panel. It ends short of that line.
            let mut sides: Vec<Body> = Vec::new();
            for other in fcs.iter().filter(|o| o.panel != fc.panel) {
                let (r, q) = (fc.b - fc.a, other.b - other.a);
                let den = r.cross(q);
                if den.abs() < 1e-9 {
                    continue;
                }
                let c = fc.a + r * ((other.a - fc.a).cross(q) / den);
                let near = |a: Vec2, b: Vec2| (c - a).len().min((c - b).len()) <= 4.0 * t + reach;
                if !(near(fc.a, fc.b) && near(other.a, other.b)) {
                    continue;
                }
                let (Some(own), Some(theirs)) = (((fc.a + fc.b) * 0.5 - c).normalized(), ((other.a + other.b) * 0.5 - c).normalized()) else {
                    continue;
                };
                if own.dot(theirs).abs() > 0.996 {
                    continue;
                }
                let Some(m) = (own + theirs).normalized() else { continue };
                let side = if Vec2::new(-m.y, m.x).dot(own) >= 0.0 { Vec2::new(-m.y, m.x) } else { Vec2::new(m.y, -m.x) };
                let (far, gap) = (len + q.len() + 10.0 * t + reach, FINGER_FLAT_GAP / 2.0);
                let p = |along: f64, across: f64| c + m * along + side * across;
                let half = Loop2::polygon(&[p(-far, gap), p(far, gap), p(far, gap + far), p(-far, gap + far)]);
                sides.push(extrude(&Plane::XY, &[Region2 { outer: half.ccw(), holes: Vec::new() }], -2.0 * over, t + 2.0 * over)?);
            }
            'fingers: for (s, e) in fc.own_fingers() {
                let mid = fc.a + d * ((s + e) / 2.0);
                let plane = Plane { origin: Vec3::new(mid.x, mid.y, 0.0), x: Vec3::new(-fc.n.x, -fc.n.y, 0.0), y: Vec3::Z };
                let mut tool = extrude(&plane, &[Region2 { outer: wedge.ccw(), holes: Vec::new() }], -(e - s) / 2.0, (e - s) / 2.0)?;
                if !sides.is_empty() {
                    let whole = kernel::measure(&tool)?.volume;
                    for side in &sides {
                        match kernel::boolean(&tool, side, kernel::BoolOp::Intersect)? {
                            Some(left) => tool = left,
                            None => continue 'fingers,
                        }
                    }
                    // What is left of a finger cut back to a fin would break off in folding.
                    if kernel::measure(&tool)?.volume < 0.6 * whole {
                        continue 'fingers;
                    }
                }
                body = kernel::boolean(&body, &tool, kernel::BoolOp::Union)?
                    .ok_or_else(|| DocError::Invalid("a joint on an open edge leaves nothing of the print".into()))?;
            }
        }
        if let Some(fit) = fc.press {
            body = press_nubs(body, fc, &fit, d, len, t, pf.clearance, over)?;
            continue;
        }
        let h = fc.side / 2.0;
        let (o, z) = (fc.offset, t / 2.0);
        let square = Loop2::polygon(&[Vec2::new(o - h, z - h), Vec2::new(o + h, z - h), Vec2::new(o + h, z + h), Vec2::new(o - h, z + h)]);
        let plane = Plane { origin: Vec3::new(fc.a.x, fc.a.y, 0.0), x: Vec3::new(fc.n.x, fc.n.y, 0.0), y: Vec3::Z };
        // The plane's normal (n × z) runs along the edge one way or the other.
        let (lo, hi) = if fc.n.y * d.x - fc.n.x * d.y >= 0.0 { (-END_OVER, len + END_OVER) } else { (-len - END_OVER, END_OVER) };
        channels.push(extrude(&plane, &[Region2 { outer: square.ccw(), holes: Vec::new() }], lo, hi)?);
    }
    Ok(body)
}

/// A press fit's nubs or dimples on one panel's fingers along an open edge: at each end face of a
/// finger (where a notch for the other panel's finger begins or ends, not at the edge's ends), in
/// the middle of the folded corner. A nub is the cap of a ball whose centre lies behind the face; a
/// dimple is that ball (the other panel's nub), `tolerance` larger.
#[allow(clippy::too_many_arguments)]
fn press_nubs(mut body: Body, fc: &FilamentCut, fit: &PressFit, d: Vec2, len: f64, t: f64, clearance: f64, over: f64) -> Result<Body> {
    let (ball, back) = (fit.ball(), fit.ball() - fit.height);
    let at = |u: f64| {
        let p = fc.a + d * u + fc.n * fc.offset;
        Vec3::new(p.x, p.y, t / 2.0)
    };
    for (lo, hi) in &fc.notches {
        // This panel's finger ends at `lo` looking along +u into the notch, and starts again at `hi`.
        for (face, dir) in [(*lo, 1.0), (*hi, -1.0)] {
            if face <= 0.0 || face >= len {
                continue;
            }
            if fc.nubs {
                let centre = face - dir * back;
                let (u0, u1) = (face - dir * over, face + dir * (fit.height + over));
                let (u0, u1) = if u0 < u1 { (u0, u1) } else { (u1, u0) };
                let w = ball + over;
                let p = |u: f64, x: f64| fc.a + d * u + fc.n * x;
                let o = fc.offset;
                let rect = Loop2::polygon(&[p(u0, o - w), p(u1, o - w), p(u1, o + w), p(u0, o + w)]);
                let cap = extrude(&Plane::XY, &[Region2 { outer: rect.ccw(), holes: Vec::new() }], t / 2.0 - w, t / 2.0 + w)?;
                let nub = kernel::boolean(&kernel::sphere(at(centre), ball)?, &cap, kernel::BoolOp::Intersect)?
                    .ok_or_else(|| DocError::Invalid("a press-fit nub came out empty".into()))?;
                body = kernel::boolean(&body, &nub, kernel::BoolOp::Union)?
                    .ok_or_else(|| DocError::Invalid("a press-fit nub leaves nothing of the print".into()))?;
            } else {
                // The other panel's finger stands `clearance` away, its nub's ball centre behind it;
                // the dimple is the larger ball's cap beyond that face, nothing behind it.
                let nub_face = face + dir * clearance;
                let centre = nub_face + dir * back;
                let eps = over.min(0.1);
                let (u0, u1) = (nub_face + dir * eps, nub_face - dir * (fit.height + fit.tolerance.max(0.0) + over));
                let (u0, u1) = if u0 < u1 { (u0, u1) } else { (u1, u0) };
                let w = fit.dimple_radius() + eps;
                let p = |u: f64, x: f64| fc.a + d * u + fc.n * x;
                let o = fc.offset;
                let rect = Loop2::polygon(&[p(u0, o - w), p(u1, o - w), p(u1, o + w), p(u0, o + w)]);
                let open = extrude(&Plane::XY, &[Region2 { outer: rect.ccw(), holes: Vec::new() }], t / 2.0 - w, t / 2.0 + w)?;
                let dimple = kernel::boolean(&kernel::sphere(at(centre), ball + fit.tolerance)?, &open, kernel::BoolOp::Intersect)?
                    .ok_or_else(|| DocError::Invalid("a press-fit dimple came out empty".into()))?;
                body = kernel::boolean(&body, &dimple, kernel::BoolOp::Cut)?
                    .ok_or_else(|| DocError::Invalid("a press-fit dimple leaves nothing of the print".into()))?;
            }
        }
    }
    Ok(body)
}

/// The mitred ends and the sockets on the edges of holes through the faces of a part, where a wall
/// of the hole is printed as a part of its own (Unfold for Printing's carved-out groups): the
/// face ends in a mitre where the wall's does (the wall square to the face), or is notched for the
/// wall's fingers. The edges are picked in the body's coordinates; the face keeps its outline.
pub(crate) fn hole_cuts(
    part: &PrintPart,
    sheet: &solidcraft_develop::Sheet,
    places: &[Option<Rigid3>],
    spec: &PrintSpec,
) -> (Vec<Bevel>, Vec<FilamentCut>) {
    let (t, clearance) = (sheet.thickness, spec.fold.clearance);
    let (mut bevels, mut cuts) = (Vec::new(), Vec::new());
    if spec.mitres.is_empty() && spec.sockets.is_empty() {
        return (bevels, cuts);
    }
    for (k, p) in part.panels.iter().enumerate() {
        let Some(frame) = places.get(p.panel).copied().flatten() else { continue };
        let body = |q: Vec2| frame.point(p.place.inverse().point(q - p.shift).extend(0.0));
        for l in &p.region.holes {
            for seg in &l.segs {
                let Seg2::Line { a, b } = *seg else { continue };
                let Some(u) = (b - a).normalized() else { continue };
                let n = Vec2::new(-u.y, u.x);
                let (wa, wb) = (body(a), body(b));
                let near = |q: Vec3| chain_distance(&[wa, wb], q) <= PICK_NEAR;
                if spec.mitres.iter().any(|m| near(*m)) {
                    // A wall square to its face: the mitre is a quarter turn's half.
                    bevels.push(Bevel { a, b, n, at: 0.0, extra: t });
                    continue;
                }
                let Some(s) = spec.sockets.iter().find(|s| near(s.a) && near(s.b)) else { continue };
                let (from, to) = if wa.dist(s.a) <= wa.dist(s.b) { (a, b) } else { (b, a) };
                cuts.push(FilamentCut {
                    a: from,
                    b: to,
                    n,
                    notches: s.fingers.iter().map(|(lo, hi)| (lo - clearance / 2.0, hi + clearance / 2.0)).collect(),
                    depth: t + clearance,
                    offset: t / 2.0,
                    side: if s.filament { DEFAULT_FILAMENT + DEFAULT_CHANNEL_TOLERANCE } else { 0.0 },
                    press: (!s.filament).then_some(PressFit { radius: DEFAULT_NUB_RADIUS, height: DEFAULT_NUB_HEIGHT, tolerance: 0.0 }),
                    nubs: false,
                    angle: std::f64::consts::FRAC_PI_2,
                    panel: Some(k),
                });
            }
        }
    }
    (bevels, cuts)
}

// ---------------------------------------------------------------------------------------------
// Joints on the bends of a print: fingers that share the folded corner, round a channel or a nub.
// ---------------------------------------------------------------------------------------------

/// Where a joint's fingers are along one stretch of a bend ([`fold_layout`]).
struct FoldLayout {
    core: FoldCore,
    a0: f64,
    a1: f64,
    len: f64,
    k: u32,
    each: f64,
}

/// Where a joint on the bend from `start` to `end` (flange `f`'s) has its fingers, along the
/// bend from `start`: its core ([`FoldFilament::core`]), and the stretch `a0`…`a1` the
/// fingers share, short of every other face of the print that meets an end of the bend: the
/// face's wall (or its own joint's fingers) and the clearance, more where it meets the bend
/// askew.
fn fold_layout(
    t: f64,
    start: Vec2,
    end: Vec2,
    f: &PrintBend,
    ff: &FoldFilament,
    pf: PrintFold,
    joined_r: &dyn Fn(usize) -> Option<f64>,
) -> Result<Option<FoldLayout>> {
    let len = (end - start).len();
    if !(len > 1e-9) {
        return Ok(None);
    }
    let core = ff.core(t, f.angle, pf).map_err(|why| DocError::Invalid(format!("a joint on a bend: {why}")))?;
    let (u_s, u_e) = ((start - f.s).dot(f.d), (end - f.s).dot(f.d));
    // Along the bend from its flange's `p`: the fingers keep clear of each face at an end of the
    // edge (the bend itself may stop short of the edge's end, a wall's thickness say).
    let (mut lo, mut hi) = (u_s, u_e);
    for e in &ff.ends {
        let sin = e.sin.clamp(0.1, 1.0);
        let other = e.joined.and_then(joined_r).map_or(t, |r| r.max(t));
        let trim = (other + pf.clearance) / sin + core.r * (1.0 - sin * sin).max(0.0).sqrt() / sin;
        if e.u <= (u_s + u_e) / 2.0 {
            lo = lo.max(e.u + trim);
        } else {
            hi = hi.min(e.u - trim);
        }
    }
    let (a0, a1) = ((lo - u_s).max(0.0), (hi - u_s).min(len));
    let k = finger_count(ff.segments.max(1), a1 - a0);
    let each = (a1 - a0) / f64::from(k);
    if !(each - pf.clearance >= 1.0) {
        return Err(DocError::Invalid(format!(
            "a joint on a bend {len:.2} mm long, its fingers {:.2} mm round the bend and kept clear of the faces at its ends, leaves {k} fingers {:.2} mm each; take fewer fingers",
            core.r,
            each - pf.clearance
        )));
    }
    Ok(Some(FoldLayout { core, a0, a1, len, k, each }))
}

/// A joint on a bend (filament or press fit), cut in the flat in place of its V-groove. Along
/// the bend from `start` to `end` the fingers take turns between its two panels. Each is a
/// sector of a circle round the bend's sharp on the outer skin, the line the bend folds about:
/// from its own panel's outer skin round to the film and the clearance short of the other's,
/// as far out as its channel or nub with its wall needs ([`fold_core`]), so inside the fold it
/// stands out of the walls. The other panel is cut back to the clearance round it, down to its
/// film, which stays whole. Folding turns the panels about that sharp, so neither passes
/// through the other's fingers on the way. Where another face of the print meets an end of
/// the bend, the fingers stop short of it and the bend is a V-groove there, the channel on
/// through it. A channel is square, lined up with the wall of the finger it runs in: the
/// round filament runs through the squares of both panels.
#[allow(clippy::too_many_arguments)]
fn fold_fingers(
    t: f64,
    mut body: Body,
    start: Vec2,
    end: Vec2,
    f: &PrintBend,
    ff: &FoldFilament,
    pf: PrintFold,
    over: f64,
    index: usize,
    channels: &mut Vec<Body>,
    joined_r: &dyn Fn(usize) -> Option<f64>,
) -> Result<Body> {
    use std::f64::consts::PI;
    let Some(lay) = fold_layout(t, start, end, f, ff, pf, joined_r)? else { return Ok(body) };
    let Some(d) = (end - start).normalized() else { return Ok(body) };
    let FoldCore { d: core, r, .. } = lay.core;
    let angle = f.angle.abs();
    let (sin, cos) = (angle.sin(), angle.cos());
    let (s2, c2) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    let (film, clear) = (pf.film, pf.clearance);
    let m = film + clear;
    let rho = r + clear;
    let top = t + over;
    let (len, a0, a1, k, each) = (lay.len, lay.a0, lay.a1, lay.k, lay.each);
    let zc = core * c2;
    let pol = |rad: f64, a: f64| Vec2::new(rad * a.cos(), rad * a.sin());
    let arc = |rad: f64, a0: f64, a1: f64| Seg2::Arc { center: Vec2::new(0.0, 0.0), radius: rad, start: a0, sweep: a1 - a0 };
    let line = |a: Vec2, b: Vec2| Seg2::Line { a, b };
    // Across the bend (from its sharp, toward the flange) and up, extruded along it.
    let tool_at = |lp: &Loop2, u0: f64, u1: f64| {
        let mid = start + d * ((u0 + u1) / 2.0);
        let plane = Plane { origin: Vec3::new(mid.x, mid.y, 0.0), x: Vec3::new(f.n.x, f.n.y, 0.0), y: Vec3::Z };
        let half = (u1 - u0) / 2.0;
        extrude(&plane, &[Region2 { outer: lp.ccw(), holes: Vec::new() }], -half, half)
    };
    let cut = |body: Body, tool: Body| -> Result<Body> {
        kernel::boolean(&body, &tool, kernel::BoolOp::Cut)?.ok_or_else(|| DocError::Invalid("a joint on a bend leaves nothing of the print".into()))
    };
    // The flange's finger is the parent's mirrored across the sharp.
    let mirror = |lp: &Loop2| Loop2 {
        segs: lp
            .segs
            .iter()
            .map(|s| match s {
                Seg2::Line { a, b } => Seg2::Line { a: Vec2::new(-a.x, a.y), b: Vec2::new(-b.x, b.y) },
                Seg2::Arc { center, radius, start, sweep } => {
                    Seg2::Arc { center: Vec2::new(-center.x, center.y), radius: *radius, start: PI - *start, sweep: -*sweep }
                }
                other => *other,
            })
            .collect(),
    };
    // The dome every finger is cut from (both sides of the sharp, over the film's lower half so
    // none of its faces lies on the sheet's).
    let lo_dome = (film / 2.0 / r).asin();
    let dome = Loop2 { segs: vec![line(pol(r, PI - lo_dome), pol(r, lo_dome)), arc(r, lo_dome, PI - lo_dome)] };
    // A parent's finger: what lies within the film and the clearance of the flange's outer
    // skin once folded goes (the line through `q0` and `q2`), and the flange's wall the
    // clearance round the finger.
    let q0 = Vec2::new((film * cos - m) / sin, film);
    let lam = (rho * rho - m * m).max(0.0).sqrt();
    let q2 = Vec2::new(-m * sin + lam * cos, m * cos + lam * sin);
    let (b1, b2) = ((film / rho).asin(), q2.y.atan2(q2.x));
    if !(b2 > b1 + 1e-9 && r > top) {
        return Err(DocError::Invalid(format!("the joint on bend {index} has no room for its fingers")));
    }
    let skin = Loop2 { segs: vec![line(q0, pol(rho, b1)), arc(rho, b1, b2), line(q2, q0)] };
    let band = (clear > 1e-9).then(|| {
        let (af, at, bt) = ((film / r).asin(), (top / r).asin(), (top / rho).asin());
        Loop2 { segs: vec![line(pol(r, af), pol(rho, b1)), arc(rho, b1, bt), line(pol(rho, bt), pol(r, at)), arc(r, at, af)] }
    });
    // Between two fingers: nothing above the film within the clearance round them.
    let gap = Loop2 { segs: vec![line(pol(rho, PI - b1), pol(rho, b1)), arc(rho, b1, PI - b1)] };
    let square = |x: f64| {
        let h = ff.side / 2.0;
        Loop2::polygon(&[Vec2::new(x - h, zc - h), Vec2::new(x + h, zc - h), Vec2::new(x + h, zc + h), Vec2::new(x - h, zc + h)])
    };
    let (parent_sq, flange_sq) = (square(-core * s2), square(core * s2));
    let filament = ff.press.is_none();
    // The ends another face meets: a V-groove, the channel on through both panels.
    for (u0, u1) in [(-over, a0), (a1, len + over)] {
        if u1 - u0 <= over + 1e-9 {
            continue;
        }
        body = v_groove(body, &PrintBend { s: start, ..f.clone() }, pf, t, over, u0, u1, index)?;
        if filament {
            for sq in [&parent_sq, &flange_sq] {
                channels.push(tool_at(sq, u0, u1)?);
            }
        }
    }
    let eps = (over * 0.2).min(0.01);
    body = kernel::boolean(&body, &tool_at(&dome, a0 + eps, a1 - eps)?, kernel::BoolOp::Union)?
        .ok_or_else(|| DocError::Invalid("a joint on a bend leaves nothing of the print".into()))?;
    let open_start = a0 <= 1e-9;
    let open_end = a1 >= len - 1e-9;
    for i in 0..k {
        let parent = (i % 2 == 0) != ff.flip;
        let (s, e) = (a0 + f64::from(i) * each, a0 + f64::from(i + 1) * each);
        let u0 = if i == 0 && open_start { -over } else { s - clear / 2.0 };
        let u1 = if i + 1 == k && open_end { len + over } else { e + clear / 2.0 };
        let side = |lp: &Loop2| if parent { lp.clone() } else { mirror(lp) };
        body = cut(body, tool_at(&side(&skin), u0, u1)?)?;
        if let Some(b) = &band {
            body = cut(body, tool_at(&side(b), u0, u1)?)?;
        }
        if filament {
            let q0 = if i == 0 && open_start { -over } else { s - clear / 4.0 };
            let q1 = if i + 1 == k && open_end { len + over } else { e + clear / 4.0 };
            channels.push(tool_at(if parent { &parent_sq } else { &flange_sq }, q0, q1)?);
        }
    }
    if clear > 1e-9 {
        for i in 0..=k {
            if (i == 0 && open_start) || (i == k && open_end) {
                continue;
            }
            let b = a0 + f64::from(i) * each;
            body = cut(body, tool_at(&gap, b - clear / 2.0, b + clear / 2.0)?)?;
        }
    }
    let Some(fit) = ff.press else { return Ok(body) };
    // Each finger of the starting panel has a nub on its end faces ([`FoldCore::nub`]); the
    // next finger the same ball, a little larger, as a dimple where that nub lands once
    // folded: the nub's place turned back by the fold, in the other panel's flat.
    fit.check(clear, f64::INFINITY, t).map_err(|why| DocError::Invalid(format!("a press fit on a bend: {why}")))?;
    let (ball, back) = (fit.ball(), fit.ball() - fit.height);
    let at = |u: f64, p: Vec2| {
        let q = start + d * u + f.n * p.x;
        Vec3::new(q.x, q.y, p.y)
    };
    let p = lay.core.nub;
    let q = Vec2::new(p.x * cos + p.y * sin, -p.x * sin + p.y * cos);
    let (nub_at, dimple_at) = if ff.flip { (Vec2::new(-p.x, p.y), Vec2::new(-q.x, q.y)) } else { (p, q) };
    let boxed = |c: Vec2, w: f64| {
        Loop2::polygon(&[Vec2::new(c.x - w, c.y - w), Vec2::new(c.x + w, c.y - w), Vec2::new(c.x + w, c.y + w), Vec2::new(c.x - w, c.y + w)])
    };
    for i in 1..k {
        let b = a0 + f64::from(i) * each;
        // The starting panel's finger before the boundary (its face looks along +u) or after it.
        let before = (i - 1) % 2 == 0;
        let dir = if before { 1.0 } else { -1.0 };
        let face = b - dir * clear / 2.0;
        let centre = face - dir * back;
        let (lo, hi) = (face - dir * over, face + dir * (fit.height + over));
        let (u0, u1) = if lo < hi { (lo, hi) } else { (hi, lo) };
        let nub =
            kernel::boolean(&kernel::sphere(at(centre, nub_at), ball)?, &tool_at(&boxed(nub_at, ball + over), u0, u1)?, kernel::BoolOp::Intersect)?
                .ok_or_else(|| DocError::Invalid("a press-fit nub on a bend came out empty".into()))?;
        body = kernel::boolean(&body, &nub, kernel::BoolOp::Union)?
            .ok_or_else(|| DocError::Invalid("a press-fit nub on a bend leaves nothing of the print".into()))?;
        // The dimple: the larger ball's cap in the next finger, from the middle of the gap on
        // (nothing of the nub's own finger, nor of what lies beside it in the flat).
        let w = fit.dimple_radius() + over.min(0.1);
        let (lo, hi) = (face + dir * clear / 2.0, face + dir * (fit.height + fit.tolerance.max(0.0) + over));
        let (u0, u1) = if lo < hi { (lo, hi) } else { (hi, lo) };
        let dimple = kernel::boolean(
            &kernel::sphere(at(centre, dimple_at), ball + fit.tolerance)?,
            &tool_at(&boxed(dimple_at, w), u0, u1)?,
            kernel::BoolOp::Intersect,
        )?
        .ok_or_else(|| DocError::Invalid("a press-fit dimple on a bend came out empty".into()))?;
        body = cut(body, dimple)?;
    }
    Ok(body)
}

/// What of a joined bend's flat moves with the other panel when folded: each finger's part
/// past the sharp (above the film, where the other panel is cut back), as tools for the
/// parent's piece and for the flange's.
fn fold_moves(
    t: f64,
    start: Vec2,
    end: Vec2,
    f: &PrintBend,
    ff: &FoldFilament,
    pf: PrintFold,
    joined_r: &dyn Fn(usize) -> Option<f64>,
) -> Result<Vec<(bool, Body)>> {
    use std::f64::consts::{FRAC_PI_2, PI};
    let Some(lay) = fold_layout(t, start, end, f, ff, pf, joined_r)? else { return Ok(Vec::new()) };
    let Some(d) = (end - start).normalized() else { return Ok(Vec::new()) };
    // A finger reaches past the sharp only at a fold under 90°: there, between the sharp's
    // plane and the line the clearance and the film short of the other panel's outer skin
    // once folded (half the clearance further, so nothing of that panel is in it).
    let angle = f.angle.abs();
    let cos = angle.cos();
    let m = pf.film + pf.clearance / 2.0;
    let rr = lay.core.r + pf.clearance / 2.0;
    if !(cos > 1e-6 && m / cos < rr) {
        return Ok(Vec::new());
    }
    let pol = |rad: f64, a: f64| Vec2::new(rad * a.cos(), rad * a.sin());
    let z0 = m / cos;
    let phi = angle + (m / rr).asin();
    if !(phi < FRAC_PI_2) {
        return Ok(Vec::new());
    }
    let parent_part = Loop2 {
        segs: vec![
            Seg2::Line { a: Vec2::new(0.0, z0), b: pol(rr, phi) },
            Seg2::Arc { center: Vec2::new(0.0, 0.0), radius: rr, start: phi, sweep: FRAC_PI_2 - phi },
            Seg2::Line { a: Vec2::new(0.0, rr), b: Vec2::new(0.0, z0) },
        ],
    };
    let flange_part = Loop2 {
        segs: vec![
            Seg2::Line { a: Vec2::new(0.0, z0), b: Vec2::new(0.0, rr) },
            Seg2::Arc { center: Vec2::new(0.0, 0.0), radius: rr, start: FRAC_PI_2, sweep: FRAC_PI_2 - phi },
            Seg2::Line { a: pol(rr, PI - phi), b: Vec2::new(0.0, z0) },
        ],
    };
    let mut out = Vec::with_capacity(lay.k as usize);
    for i in 0..lay.k {
        let parent = (i % 2 == 0) != ff.flip;
        let (s, e) = (lay.a0 + f64::from(i) * lay.each, lay.a0 + f64::from(i + 1) * lay.each);
        let mid = start + d * ((s + e) / 2.0);
        let plane = Plane { origin: Vec3::new(mid.x, mid.y, 0.0), x: Vec3::new(f.n.x, f.n.y, 0.0), y: Vec3::Z };
        let lp = if parent { &parent_part } else { &flange_part };
        let half = (e - s) / 2.0;
        out.push((parent, extrude(&plane, &[Region2 { outer: lp.ccw(), holes: Vec::new() }], -half, half)?));
    }
    Ok(out)
}
/// The joints picked on the bends of `part`, by bend (see [`FoldFilament`]): for each, the faces of
/// the print that meet an end of the bend, which its fingers keep clear of.
pub(crate) fn fold_filaments(
    part: &PrintPart,
    sheet: &solidcraft_develop::Sheet,
    places: &[Option<Rigid3>],
    spec: &PrintSpec,
) -> Result<Vec<Option<FoldFilament>>> {
    let mut out: Vec<Option<FoldFilament>> = vec![None; part.bends.len()];
    if spec.filament.is_empty() {
        return Ok(out);
    }
    let motions = fold_motions(part);
    let dir3 =
        |k: usize, d: Vec2| apply_vector(motions.get(part.bends.get(k).map_or(0, |b| b.parent)).unwrap_or(&IDENTITY), Vec3::new(d.x, d.y, 0.0));
    let folded = |b: &PrintBend, u: f64| {
        let m = motions.get(b.parent).unwrap_or(&IDENTITY);
        apply_point(m, Vec3::new(b.s.x + b.d.x * u, b.s.y + b.d.y * u, 0.0))
    };
    for fl in &spec.filament {
        let Some(j) = joint_at(sheet, places, fl.at) else {
            return err(format!("the joint at ({:.3}, {:.3}, {:.3}) is on no edge of the print", fl.at.x, fl.at.y, fl.at.z));
        };
        if !matches!(sheet.joints.get(j).map(|x| &x.kind), Some(JointKind::Bend(_))) {
            continue;
        }
        for (k, b) in part.bends.iter().enumerate().filter(|(_, b)| b.joint == j) {
            let mut ends = Vec::new();
            for &(u0, u1) in &b.spans {
                for u in [u0, u1] {
                    let p_end = b.s + b.d * u;
                    for e in part.edges.iter().filter(|e| e.panel == b.parent || e.panel == b.child) {
                        if (e.a - p_end).len() > 1e-6 && (e.b - p_end).len() > 1e-6 {
                            continue;
                        }
                        let Some(rip) = sheet.joints.get(e.joint) else { continue };
                        let mine = part.panels.get(e.panel).map_or(usize::MAX, |p| p.panel);
                        let theirs = if rip.a.panel == mine { rip.b.panel } else { rip.a.panel };
                        let Some(q) = part.panels.iter().position(|p| p.panel == theirs) else { continue };
                        let normal = motions.get(q).map(|m| apply_vector(m, Vec3::new(0.0, 0.0, -1.0)));
                        let (Some(n), Some(dd)) = (normal.and_then(|n| n.normalized()), dir3(k, b.d).normalized()) else { continue };
                        ends.push(FoldEnd { u, sin: dd.dot(n).abs().min(1.0), joined: None });
                    }
                }
            }
            out[k] = Some(FoldFilament { flange: k, segments: fl.segments, flip: fl.flip, side: fl.side(), press: fl.press, wall: fl.wall, ends });
        }
    }
    // Where the joint of another bend ends at the same corner, its own fingers are what is kept clear of.
    let found = out.clone();
    for (k, b) in part.bends.iter().enumerate() {
        let Some(ff) = out.get_mut(k).and_then(Option::as_mut) else { continue };
        for end in &mut ff.ends {
            let at = folded(b, end.u);
            end.joined = part.bends.iter().enumerate().find_map(|(k2, b2)| {
                found.get(k2).and_then(Option::as_ref)?;
                (k2 != k && b2.spans.iter().any(|&(u0, u1)| [u0, u1].iter().any(|u| folded(b2, *u).dist(at) <= 1e-3))).then_some(k2)
            });
        }
    }
    Ok(out)
}
