//! Cutting tools.

use serde::{Deserialize, Serialize};

use crate::{Result, invalid};

/// Largest tool number a post processor is asked to write.
pub const MAX_TOOL_NUMBER: u32 = 9999;
/// Largest tool diameter (mm).
pub const MAX_TOOL_DIAMETER: f64 = 1000.0;
/// Highest surface speed a tool is given (m/min).
pub const MAX_SURFACE_SPEED: f64 = 10_000.0;
/// Largest chip load a tool is given (mm per tooth).
pub const MAX_CHIP_LOAD: f64 = 100.0;
/// Most materials a tool has cutting data for (and a tool library names).
pub const MAX_MATERIALS: usize = 100;
/// Longest material name (characters).
pub const MAX_MATERIAL_NAME: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolKind {
    FlatEndmill,
    BallEndmill,
    /// A flat end rounded at its rim by a corner radius ([`Tool::corner_radius`]).
    BullNose,
    Drill,
    /// A cone to a point (or a small flat), for engraving along a line.
    VBit,
}

impl ToolKind {
    pub fn label(self) -> &'static str {
        match self {
            ToolKind::FlatEndmill => "flat end mill",
            ToolKind::BallEndmill => "ball end mill",
            ToolKind::BullNose => "bull-nose end mill",
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
    /// Instead of the spindle speed and the feed: the speed at the cutting edge (m/min) and the
    /// feed per tooth (mm), which the speed and feed are worked out from (see [`Tool::speeds`]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chip_load: Option<f64>,
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
    /// A bull-nose end mill's corner radius, where its end rounds into its side (mm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_radius: Option<f64>,
    /// The shank's diameter and the tool's length out of the holder (mm): what its picture shows.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shank_diameter: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length: Option<f64>,
    /// The holder the tool sits in (mm): how wide it is and how long, from where the tool comes
    /// out of it up; what must stay clear of clamps.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holder_diameter: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holder_length: Option<f64>,
    /// Cutting data for particular materials; the data above is for every other one.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cutting: Vec<CuttingData>,
}

/// A tool's cutting data in one material: what it says replaces the tool's own data when the
/// setup's stock is of that material; what it leaves out stays the tool's own.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CuttingData {
    pub material: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chip_load: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
}

impl CuttingData {
    /// Whether it says nothing beyond its material.
    pub fn is_empty(&self) -> bool {
        [self.spindle_rpm, self.feed, self.surface_speed, self.chip_load, self.plunge_feed, self.stepdown, self.stepover].iter().all(Option::is_none)
    }
}

/// The spindle speed a surface speed (m/min) gives a tool `diameter` mm across, in whole rev/min.
pub fn rpm_from_surface_speed(surface_speed: f64, diameter: f64) -> f64 {
    (surface_speed * 1000.0 / (std::f64::consts::PI * diameter)).round()
}

/// The surface speed (m/min) a spindle speed gives a tool `diameter` mm across.
pub fn surface_speed_from_rpm(rpm: f64, diameter: f64) -> f64 {
    rpm * std::f64::consts::PI * diameter / 1000.0
}

/// Whether two material names name the same material (case and surrounding spaces aside).
pub fn same_material(a: &str, b: &str) -> bool {
    a.trim().to_lowercase() == b.trim().to_lowercase()
}

/// A material name as given, checked: not empty, at most [`MAX_MATERIAL_NAME`] characters, no control characters.
pub fn material_name(name: &str) -> Result<String> {
    let name = name.trim();
    if name.is_empty() || name.chars().count() > MAX_MATERIAL_NAME || name.chars().any(char::is_control) {
        return invalid(format!("a material name has 1 to {MAX_MATERIAL_NAME} characters"));
    }
    Ok(name.to_string())
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
            surface_speed: None,
            chip_load: None,
            plunge_feed: None,
            stepdown: None,
            stepover: None,
            tip_angle: None,
            tip_diameter: None,
            corner_radius: None,
            shank_diameter: None,
            holder_diameter: None,
            holder_length: None,
            length: None,
            cutting: Vec::new(),
        }
    }
    /// The tool's cutting data in `material`, if it has any of its own for it.
    pub fn cutting_in(&self, material: &str) -> Option<&CuttingData> {
        self.cutting.iter().find(|c| same_material(&c.material, material))
    }
    /// The tool as it cuts `material`: its data for that material where it has some, else its own.
    pub fn in_material(&self, material: Option<&str>) -> Tool {
        let mut tool = self.clone();
        if let Some(c) = material.and_then(|m| self.cutting_in(m)) {
            // A speed, or a feed, the material gives replaces the tool's however the tool gives it.
            if c.spindle_rpm.is_some() || c.surface_speed.is_some() {
                (tool.spindle_rpm, tool.surface_speed) = (c.spindle_rpm, c.surface_speed);
            }
            if c.feed.is_some() || c.chip_load.is_some() {
                (tool.feed, tool.chip_load) = (c.feed, c.chip_load);
            }
            tool.plunge_feed = c.plunge_feed.or(tool.plunge_feed);
            tool.stepdown = c.stepdown.or(tool.stepdown);
            tool.stepover = c.stepover.or(tool.stepover);
        }
        tool
    }
    /// The spindle speed (rev/min) and feed (mm/min) the tool cuts at: a surface speed gives the
    /// speed for its diameter, else its spindle speed; a chip load gives the feed at that speed
    /// (speed × flutes × chip load), else its feed. A speed above `max_rpm` (the machine's
    /// spindle) is cut down to it, and a chip load then feeds slower to match.
    pub fn speeds(&self, max_rpm: Option<f64>) -> (Option<f64>, Option<f64>) {
        let rpm = match self.surface_speed {
            Some(v) => Some(rpm_from_surface_speed(v, self.diameter)),
            None => self.spindle_rpm,
        };
        let rpm = match (rpm, max_rpm.filter(|m| *m > 0.0)) {
            (Some(r), Some(m)) => Some(r.min(m)),
            (r, _) => r,
        };
        let feed = match (self.chip_load, rpm) {
            (Some(c), Some(r)) => Some((r * f64::from(self.flutes) * c * 10.0).round() / 10.0),
            (Some(_), None) => None,
            (None, _) => self.feed,
        };
        (rpm, feed)
    }
    /// The tool with its speed and feed worked out (see [`Tool::speeds`]).
    pub fn with_speeds(&self, max_rpm: Option<f64>) -> Tool {
        let mut tool = self.clone();
        (tool.spindle_rpm, tool.feed) = self.speeds(max_rpm);
        (tool.surface_speed, tool.chip_load) = (None, None);
        tool
    }
    /// The point's included angle (degrees): 180 for a flat or ball end.
    pub fn point_angle(&self) -> f64 {
        match self.kind {
            ToolKind::VBit => self.tip_angle.unwrap_or(90.0),
            ToolKind::Drill => self.tip_angle.unwrap_or(118.0),
            ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::BullNose => 180.0,
        }
    }
    /// How far the end rounds into the side (mm): a ball's radius, a bull nose's corner radius,
    /// nothing for the rest.
    pub fn corner(&self) -> f64 {
        match self.kind {
            ToolKind::BallEndmill => self.radius(),
            ToolKind::BullNose => self.corner_radius.unwrap_or(0.0).clamp(0.0, self.radius()),
            _ => 0.0,
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
        cutting_rates(self.surface_speed, self.chip_load, "")?;
        if self.tip_angle.is_some_and(|a| !(a > 0.0 && a < 180.0)) {
            return invalid("a point angle must be between 0 and 180 degrees");
        }
        if self.tip_diameter.is_some_and(|d| !(0.0..self.diameter).contains(&d)) {
            return invalid("a V-bit's tip diameter must be 0 or more and less than its diameter");
        }
        if self.kind == ToolKind::BullNose && !self.corner_radius.is_some_and(|c| c > 0.0 && c <= self.radius()) {
            return invalid("a bull-nose end mill's corner radius must be above 0 and at most half its diameter");
        }
        if self.shank_diameter.is_some_and(|d| !(d > 0.0 && d <= MAX_TOOL_DIAMETER)) {
            return invalid("the shank diameter must be above 0 and at most 1000 mm");
        }
        if self.holder_diameter.is_some_and(|d| !(d > 0.0 && d <= MAX_TOOL_DIAMETER)) {
            return invalid("the holder's diameter must be above 0 and at most 1000 mm");
        }
        if self.holder_length.is_some_and(|l| !(l > 0.0 && l <= 10.0 * MAX_TOOL_DIAMETER)) {
            return invalid("the holder's length must be above 0");
        }
        if self.length.is_some_and(|l| !(l >= self.flute_length && l <= 10.0 * MAX_TOOL_DIAMETER)) {
            return invalid("a tool's length must be at least its flute length");
        }
        if self.cutting.len() > MAX_MATERIALS {
            return invalid(format!("a tool has cutting data for at most {MAX_MATERIALS} materials"));
        }
        for (i, c) in self.cutting.iter().enumerate() {
            material_name(&c.material)?;
            if self.cutting.iter().take(i).any(|d| same_material(&d.material, &c.material)) {
                return invalid(format!("the tool has cutting data for `{}` twice", c.material.trim()));
            }
            if [c.spindle_rpm, c.feed, c.plunge_feed, c.stepdown, c.stepover].iter().flatten().any(|v| !(*v > 0.0) || !(*v <= 1.0e6)) {
                return invalid(format!("the spindle speed, feeds, stepdown and stepover in `{}` must be above 0", c.material.trim()));
            }
            cutting_rates(c.surface_speed, c.chip_load, &format!(" in `{}`", c.material.trim()))?;
        }
        Ok(())
    }
}

/// A surface speed and chip load, checked: above 0 and at most [`MAX_SURFACE_SPEED`] and [`MAX_CHIP_LOAD`].
fn cutting_rates(surface_speed: Option<f64>, chip_load: Option<f64>, place: &str) -> Result<()> {
    if surface_speed.is_some_and(|v| !(v > 0.0 && v <= MAX_SURFACE_SPEED)) {
        return invalid(format!("the surface speed{place} must be above 0 and at most {MAX_SURFACE_SPEED} m/min"));
    }
    if chip_load.is_some_and(|c| !(c > 0.0 && c <= MAX_CHIP_LOAD)) {
        return invalid(format!("the chip load{place} must be above 0 and at most {MAX_CHIP_LOAD} mm per tooth"));
    }
    Ok(())
}
