//! Manufacturing data of a design: the tool library and the setups with their operations.
//! Toolpaths are not stored; they are worked out from this and the evaluated model, so they
//! follow parameter changes.

use serde::{Deserialize, Serialize};
use solidcraft_cam::{Side, Tool};
use solidcraft_geom::Vec3;

use crate::{Expr, ProfileSel};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Manufacture {
    /// The tool library, by tool number.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tools: Vec<Tool>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub setups: Vec<Setup>,
}

impl Manufacture {
    pub fn is_empty(&self) -> bool {
        self.tools.is_empty() && self.setups.is_empty()
    }
    pub fn tool(&self, number: u32) -> Option<&Tool> {
        self.tools.iter().find(|t| t.number == number)
    }
}

/// One clamping of the part: which way the tool points, where the work origin is, the stock
/// the part is cut from, and what is cut in order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    pub id: u64,
    pub name: String,
    /// The tool axis: the setup's +Z as a direction in the world (the tool points along −Z).
    #[serde(default = "world_up")]
    pub axis: Vec3,
    pub origin: SetupOrigin,
    #[serde(default)]
    pub stock: StockDef,
    /// What the stock is made of: operations take their tool's cutting data for it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material: Option<String>,
    /// The built-in post processor programs are written with.
    pub post: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub operations: Vec<CamOperation>,
}

fn world_up() -> Vec3 {
    Vec3::new(0.0, 0.0, 1.0)
}

/// Where a setup's work origin is.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SetupOrigin {
    /// A fixed point in the world.
    Point { at: Vec3 },
    /// A point of the box around the stock or the part, in the setup's own axes: a corner, the
    /// middle of an edge or a face, or the centre. It follows the design.
    Box { of: BoxOf, x: Anchor, y: Anchor, z: Anchor },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoxOf {
    Stock,
    Part,
}

/// Where along one axis of a box.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Anchor {
    Min,
    Center,
    Max,
}

/// The stock: the box around the bodies (in the setup's axes) grown by these offsets (length expressions).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct StockDef {
    /// Added on each of the four sides.
    pub side: Expr,
    /// Added above the part (towards the tool).
    pub top: Expr,
    /// Added below the part.
    pub bottom: Expr,
}

impl Default for StockDef {
    fn default() -> Self {
        StockDef { side: "0 mm".into(), top: "0 mm".into(), bottom: "0 mm".into() }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CamOperation {
    pub id: u64,
    pub name: String,
    /// Tool number in the library.
    pub tool: u32,
    pub spindle_rpm: f64,
    #[serde(default)]
    pub coolant: bool,
    /// Feed rates (mm/min).
    pub cut_feed: f64,
    pub plunge_feed: f64,
    /// Heights in the setup's Z, as length expressions. One left out follows the geometry:
    /// the sketch plane or picked face, and the picked face's body (see the engine's `cam`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub clearance: Option<Expr>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract: Option<Expr>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub top: Option<Expr>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bottom: Option<Expr>,
    /// Depth per pass, as a length expression.
    pub stepdown: Expr,
    /// How many depth passes, instead of the stepdown (each then the depth over this).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub passes: Option<u32>,
    /// Where to cut.
    pub geometry: CamGeometry,
    pub kind: CamOperationKind,
}

/// What an operation's regions come from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CamGeometry {
    /// Profiles of a sketch.
    Sketch { sketch: u64, profiles: ProfileSel },
    /// The outlines of planar body faces (their inner loops are islands, or holes to drill).
    Faces { faces: Vec<FaceRef> },
    /// The stock's top, seen along the tool: what a facing operation covers.
    Stock,
    /// The box round the bodies, seen along the tool.
    Part,
}

/// A body face: the one with the persistent name `name` on `body`, else the piece of it nearest `at`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceRef {
    pub body: String,
    pub name: String,
    pub at: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CamOperationKind {
    /// Faces the box around its boundary (profiles, faces, the stock or the part).
    Face {
        stepover: Expr,
        /// The tool stays inside the boundary; otherwise it cuts over it.
        #[serde(default)]
        inside: bool,
        /// How far inside or over the boundary (a length expression); left out: over by the tool's radius, inside by nothing.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        margin: Option<Expr>,
    },
    Contour {
        side: Side,
        climb: bool,
        stock_to_leave: Expr,
        /// Steepest descent into each depth pass (an angle expression; 0 plunges).
        #[serde(default = "default_ramp")]
        ramp_angle: Expr,
        /// Tabs left standing to hold a part being cut out.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        tabs: Option<TabsDef>,
        /// Allowance the depth passes leave on the wall for one finishing pass at full depth (a length expression).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        finish: Option<Expr>,
    },
    Pocket {
        stepover: Expr,
        climb: bool,
        stock_to_leave: Expr,
        #[serde(default = "default_ramp")]
        ramp_angle: Expr,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        finish: Option<Expr>,
    },
    /// Mills round holes larger than the tool on a helix, `pitch` (a length expression) down a turn.
    Bore { climb: bool, stock_to_leave: Expr, pitch: Expr },
    /// Drills at the centre of each profile.
    Drill {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck: Option<Expr>,
        #[serde(default)]
        dwell: f64,
    },
}

/// A contour's tabs: how many on each loop, how long along the part's edge and how high over
/// the bottom (length expressions).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TabsDef {
    pub count: u32,
    pub width: Expr,
    pub height: Expr,
}

fn default_ramp() -> Expr {
    "3 deg".into()
}

impl CamOperationKind {
    pub fn label(&self) -> &'static str {
        match self {
            CamOperationKind::Face { .. } => "face",
            CamOperationKind::Contour { .. } => "contour",
            CamOperationKind::Pocket { .. } => "pocket",
            CamOperationKind::Bore { .. } => "bore",
            CamOperationKind::Drill { .. } => "drill",
        }
    }
}
