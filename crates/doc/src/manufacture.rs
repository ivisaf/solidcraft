//! Manufacturing data of a design: the tool library and the setups with their operations.
//! Toolpaths are not stored; they are worked out from this and the evaluated model, so they
//! follow parameter changes.

use serde::{Deserialize, Serialize};
use solidcraft_cam::{Side, Tool};
use solidcraft_geom::Vec3;

use crate::ProfileSel;

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

/// One clamping of the part: where the work origin is, and what is cut in order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Setup {
    pub id: u64,
    pub name: String,
    /// The work origin in world coordinates. The work axes are the world's: tool along +Z.
    pub origin: Vec3,
    /// The built-in post processor programs are written with.
    pub post: String,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub operations: Vec<CamOperation>,
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
    /// Heights in the setup's Z and the depth per pass, as length expressions.
    pub clearance: String,
    pub retract: String,
    pub top: String,
    pub bottom: String,
    pub stepdown: String,
    /// The sketch whose profiles say where to cut, and which of them.
    pub sketch: u64,
    pub profiles: ProfileSel,
    pub kind: CamOperationKind,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CamOperationKind {
    /// Faces the box around the profiles.
    Face {
        stepover: String,
    },
    Contour {
        side: Side,
        climb: bool,
        stock_to_leave: String,
    },
    Pocket {
        stepover: String,
        climb: bool,
        stock_to_leave: String,
    },
    /// Drills at the centre of each profile.
    Drill {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        peck: Option<String>,
        #[serde(default)]
        dwell: f64,
    },
}

impl CamOperationKind {
    pub fn label(&self) -> &'static str {
        match self {
            CamOperationKind::Face { .. } => "face",
            CamOperationKind::Contour { .. } => "contour",
            CamOperationKind::Pocket { .. } => "pocket",
            CamOperationKind::Drill { .. } => "drill",
        }
    }
}
