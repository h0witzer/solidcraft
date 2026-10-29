//! The skin of a sheet: its panels, each drawn in its own chart, and the joints where they meet.

use solidcraft_geom::{Region2, Vec2};

/// One skin of a sheet: the panels and the joints between them. The material lies on the side
/// of the skin that a chart's `u × v` points to, `thickness` deep.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Skin {
    pub thickness: f64,
    /// Where the neutral surface lies through the thickness, as a fraction of it from the
    /// inside of each bend.
    pub k_factor: f64,
    pub panels: Vec<Panel>,
    pub joints: Vec<Joint>,
}

/// A face of the skin. Its region is drawn in the panel's chart: a map of the skin's surface
/// onto the plane that keeps lengths and angles, so the region is the panel's own shape on the
/// skin. An empty region is a panel with no material of its own, which still carries the joints
/// on it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Panel {
    pub name: String,
    pub surface: Surface,
    pub region: Region2,
}

/// The surface a panel lies on, and so what its chart is. A curved panel is a roll: its radius
/// is the panel's own geometry, where a bend's is a value of the joint. It flattens through its
/// neutral surface, as a bend does, so a roll and a bend of one radius give the same flat.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Surface {
    /// A planar panel: its chart is its plane's coordinates.
    #[default]
    Plane,
    /// A panel rolled on a cylinder of `radius` (the skin's). The chart's x runs along the axis
    /// and its y round it, as length on the skin.
    Cylinder { radius: f64, material_inward: bool },
    /// A panel rolled on a cone whose rulings make `half_angle` (radians) with its axis. The
    /// chart is the skin's development: the apex at the origin, a point at its distance from the
    /// apex along its ruling.
    Cone { half_angle: f64, material_inward: bool },
}

/// Where a joint lies on one of its panels: a straight line in the panel's chart, through
/// `origin` along the unit `dir`. The two anchors of a joint are the same line on the sheet,
/// with the same origin and direction, and the `a` panel's material lies on the left of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Anchor {
    pub panel: usize,
    pub origin: Vec2,
    pub dir: Vec2,
}

/// Two panels meeting along a line.
#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    pub a: Anchor,
    pub b: Anchor,
    pub kind: JointKind,
}

#[derive(Clone, Debug, PartialEq)]
pub enum JointKind {
    /// The sheet runs on from one panel to the other round a bend.
    Bend(Bend),
    /// The sheet runs on from one panel into the next with no bend between: where a roll
    /// meets a flat panel or another roll tangent to it.
    Seam,
    /// The panels are cut apart. Each loses `cut` (mm, `(a, b)`) from its edge along the
    /// joint, and the flat pattern lays them out apart unless other joints hold them together.
    Rip { cut: (f64, f64) },
}

/// A bend between two panels.
#[derive(Clone, Debug, PartialEq)]
pub struct Bend {
    /// How far the sheet turns, in radians, more than 0 and at most a half turn.
    pub angle: f64,
    /// The inside radius.
    pub radius: f64,
    /// The bend turns toward the material side of the skin, so the skin runs round the
    /// outside of the bend; otherwise round the inside.
    pub toward_material: bool,
    /// What the anchors mark on the panels.
    pub datum: Datum,
    /// The bend zone's ends along the joint.
    pub ends: ZoneEnds,
    /// What each panel keeps beyond the zone's ends, between its anchor and where the bend
    /// would start: `(a, b)`.
    pub tails: (Tail, Tail),
}

/// The line the anchors mark.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Datum {
    /// The virtual sharp, where the two panels' skins would meet if the bend had no radius.
    /// Changing the radius moves neither panel. A half turn has no sharp.
    Sharp,
    /// The tangent line, where each panel's flat material ends and the bend starts. A hem,
    /// whose panels never meet, is anchored here.
    Tangent,
}

/// The bend zone along its joint, in lengths along the anchors from their origin: where its
/// edge on the `a` side starts and ends (`near`), and where its edge on the `b` side does
/// (`far`). A square end has the same length in both; a torn end does not.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneEnds {
    pub near: (f64, f64),
    pub far: (f64, f64),
}

/// What a panel keeps beyond the bend zone's ends, between its anchor and its tangent line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tail {
    /// The panel's own outline: a base that runs on beside a shorter flange keeps its edge.
    Keep,
    /// Nothing: past the zone's ends the panel starts where the bend would.
    Trim,
}

impl Bend {
    /// The flat width of the bend zone: the length of its neutral arc.
    pub fn allowance(&self, thickness: f64, k_factor: f64) -> f64 {
        self.angle * (self.radius + k_factor * thickness)
    }

    /// How far each anchor lies from its panel's tangent line, toward the other panel.
    pub fn setback(&self, thickness: f64) -> f64 {
        match self.datum {
            Datum::Tangent => 0.0,
            Datum::Sharp => {
                let skin_radius = if self.toward_material { self.radius + thickness } else { self.radius };
                skin_radius * (self.angle / 2.0).tan()
            }
        }
    }
}
