//! Cutting tools.

use serde::{Deserialize, Serialize};

use crate::{Result, invalid};

/// Largest tool number a post processor is asked to write.
pub const MAX_TOOL_NUMBER: u32 = 9999;
/// Largest tool diameter (mm).
pub const MAX_TOOL_DIAMETER: f64 = 1000.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolKind {
    FlatEndmill,
    BallEndmill,
    Drill,
    /// A cone to a point (or a small flat), for engraving along a line.
    VBit,
}

impl ToolKind {
    pub fn label(self) -> &'static str {
        match self {
            ToolKind::FlatEndmill => "flat end mill",
            ToolKind::BallEndmill => "ball end mill",
            ToolKind::Drill => "drill",
            ToolKind::VBit => "V-bit",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// The pocket the machine holds it in (`T` word).
    pub number: u32,
    pub kind: ToolKind,
    /// Cutting diameter (mm).
    pub diameter: f64,
    /// Length of the cutting edges (mm).
    pub flute_length: f64,
    #[serde(default)]
    pub description: String,
    /// Number of cutting edges.
    #[serde(default = "two")]
    pub flutes: u32,
    /// The cutting data operations start from: spindle speed (rev/min) and feeds (mm/min).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    /// The depth per pass and the distance between passes operations start from (mm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
    /// The point's included angle (degrees): a V-bit's (90 when left out) or a drill's (118).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_angle: Option<f64>,
    /// A V-bit's flat at its point (mm; none when left out).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_diameter: Option<f64>,
    /// The shank's diameter and the tool's length out of the holder (mm): what its picture shows.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shank_diameter: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length: Option<f64>,
}

fn two() -> u32 {
    2
}

impl Tool {
    pub fn new(number: u32, kind: ToolKind, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            number,
            kind,
            diameter,
            flute_length,
            description: String::new(),
            flutes: 2,
            spindle_rpm: None,
            feed: None,
            plunge_feed: None,
            stepdown: None,
            stepover: None,
            tip_angle: None,
            tip_diameter: None,
            shank_diameter: None,
            length: None,
        }
    }
    /// The point's included angle (degrees): 180 for a flat or ball end.
    pub fn point_angle(&self) -> f64 {
        match self.kind {
            ToolKind::VBit => self.tip_angle.unwrap_or(90.0),
            ToolKind::Drill => self.tip_angle.unwrap_or(118.0),
            ToolKind::FlatEndmill | ToolKind::BallEndmill => 180.0,
        }
    }
    /// A V-bit's flat at its point (mm).
    pub fn tip(&self) -> f64 {
        match self.kind {
            ToolKind::VBit => self.tip_diameter.unwrap_or(0.0),
            _ => 0.0,
        }
    }
    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }
    pub fn validate(&self) -> Result<()> {
        if self.number > MAX_TOOL_NUMBER {
            return invalid(format!("tool number {} is above {MAX_TOOL_NUMBER}", self.number));
        }
        if !(self.diameter > 0.0) || !(self.diameter <= MAX_TOOL_DIAMETER) {
            return invalid("the tool diameter must be above 0 and at most 1000 mm");
        }
        if !(self.flute_length > 0.0) || !self.flute_length.is_finite() {
            return invalid("the tool's flute length must be above 0");
        }
        if !(1..=100).contains(&self.flutes) {
            return invalid("a tool has 1 to 100 flutes");
        }
        if [self.spindle_rpm, self.feed, self.plunge_feed, self.stepdown, self.stepover].iter().flatten().any(|v| !(*v > 0.0) || !(*v <= 1.0e6)) {
            return invalid("a tool's spindle speed, feeds, stepdown and stepover must be above 0");
        }
        if self.tip_angle.is_some_and(|a| !(a > 0.0 && a < 180.0)) {
            return invalid("a point angle must be between 0 and 180 degrees");
        }
        if self.tip_diameter.is_some_and(|d| !(0.0..self.diameter).contains(&d)) {
            return invalid("a V-bit's tip diameter must be 0 or more and less than its diameter");
        }
        if self.shank_diameter.is_some_and(|d| !(d > 0.0 && d <= MAX_TOOL_DIAMETER)) {
            return invalid("the shank diameter must be above 0 and at most 1000 mm");
        }
        if self.length.is_some_and(|l| !(l >= self.flute_length && l <= 10.0 * MAX_TOOL_DIAMETER)) {
            return invalid("a tool's length must be at least its flute length");
        }
        Ok(())
    }
}
