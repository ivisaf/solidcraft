//! Post processors: how a program is written for one kind of controller. A post is data (it
//! can be read from JSON), not code: line templates around the motion blocks, and a few
//! switches for how the motion itself is written.
//!
//! Templates may use `{program}` (program name), `{units}` (`G21` or `G20`), `{wcs}` (the work
//! offset, `G54` to `G59`), `{tool}` (tool
//! number), `{diameter}` (tool diameter in output units) and `{rpm}` (spindle speed); the dwell
//! template uses `{seconds}` or `{milliseconds}`.

use serde::{Deserialize, Serialize};

use crate::{CamError, Result, invalid};

/// Most lines in one template block.
const MAX_TEMPLATE_LINES: usize = 64;
/// Longest template line.
const MAX_TEMPLATE_LEN: usize = 200;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Units {
    Mm,
    Inch,
}

impl Units {
    /// Millimetres per output unit.
    pub fn mm(self) -> f64 {
        match self {
            Units::Mm => 1.0,
            Units::Inch => 25.4,
        }
    }
    pub fn code(self) -> &'static str {
        match self {
            Units::Mm => "G21",
            Units::Inch => "G20",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommentStyle {
    /// `(comment)`
    Parentheses,
    /// `; comment`
    Semicolon,
    /// No comments are written.
    None,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineNumbers {
    pub start: u32,
    pub step: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Post {
    pub name: String,
    /// File extension without the dot.
    pub extension: String,
    pub units: Units,
    /// Decimals of coordinates (at most 6).
    pub decimals: u8,
    /// Decimals of feed rates (at most 4).
    pub feed_decimals: u8,
    #[serde(default)]
    pub line_numbers: Option<LineNumbers>,
    pub comments: CommentStyle,
    /// Written once at the start.
    pub header: Vec<String>,
    /// Written before the first operation of each tool.
    pub tool_change: Vec<String>,
    /// Whether `tool_change` is also written for the program's first tool.
    pub tool_change_first: bool,
    pub spindle_on: Vec<String>,
    pub spindle_off: Vec<String>,
    pub coolant_on: Vec<String>,
    pub coolant_off: Vec<String>,
    /// Written once at the end.
    pub footer: Vec<String>,
    /// Whether arcs are written as `G2`/`G3` with `I` `J`; otherwise as short lines.
    pub arcs: bool,
    /// Chord error of arcs written as lines (mm).
    pub arc_tolerance: f64,
    /// Whether an arc that also moves in Z (a helix, as ramps make) is written as an arc; otherwise as short lines.
    #[serde(default = "yes")]
    pub helical_arcs: bool,
    /// Whether whole numbers are written with a decimal point (`X10.`), for controllers that
    /// read `X10` as ten of their smallest unit.
    #[serde(default)]
    pub trailing_decimal: bool,
    /// Whether motion codes are written with two digits (`G01` rather than `G1`).
    #[serde(default)]
    pub two_digit_codes: bool,
    /// How a dwell is written: `{seconds}` or `{milliseconds}` (a whole number) stands for its length.
    #[serde(default = "default_dwell")]
    pub dwell: String,
}

fn yes() -> bool {
    true
}

fn default_dwell() -> String {
    "G4 P{seconds}".into()
}

fn lines(l: &[&str]) -> Vec<String> {
    l.iter().map(|s| s.to_string()).collect()
}

impl Post {
    /// Controllers that follow the NIST RS-274/NGC language closely (LinuxCNC and relatives):
    /// automatic tool change with tool length offsets.
    pub fn rs274() -> Post {
        Post {
            name: "rs274".into(),
            extension: "ngc".into(),
            units: Units::Mm,
            decimals: 4,
            feed_decimals: 1,
            line_numbers: None,
            comments: CommentStyle::Parentheses,
            header: lines(&["G17 {units} G40 G49 {wcs} G80 G90 G94"]),
            tool_change: lines(&["M5", "T{tool} M6", "G43 H{tool}"]),
            tool_change_first: true,
            spindle_on: lines(&["S{rpm} M3"]),
            spindle_off: lines(&["M5"]),
            coolant_on: lines(&["M8"]),
            coolant_off: lines(&["M9"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
            helical_arcs: true,
            trailing_decimal: false,
            two_digit_codes: false,
            dwell: default_dwell(),
        }
    }

    /// Grbl 1.1: no tool changer, so a second tool stops the program (`M0`) for a manual change.
    pub fn grbl() -> Post {
        Post {
            name: "grbl".into(),
            extension: "nc".into(),
            units: Units::Mm,
            decimals: 3,
            feed_decimals: 0,
            line_numbers: None,
            comments: CommentStyle::Parentheses,
            header: lines(&["G17 {units} {wcs} G90 G94"]),
            tool_change: lines(&["M5", "M0"]),
            tool_change_first: false,
            spindle_on: lines(&["S{rpm} M3"]),
            spindle_off: lines(&["M5"]),
            coolant_on: lines(&["M8"]),
            coolant_off: lines(&["M9"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
            helical_arcs: true,
            trailing_decimal: false,
            two_digit_codes: false,
            dwell: default_dwell(),
        }
    }

    /// grblHAL: Grbl's language plus `T… M6`, which the controller answers as its `$341`
    /// setting says (a manual change, touch-off by hand or on a tool sensor, an automatic changer,
    /// or nothing). The first tool is the one already in the spindle, touched off by hand, so only
    /// a later tool is called with `M6`; the controller keeps its length offset (`G43.1`),
    /// so the program writes no `G43`, which needs a tool table grblHAL may be built without.
    pub fn grblhal() -> Post {
        Post {
            name: "grblhal".into(),
            header: lines(&["G17 {units} G40 G49 {wcs} G80 G90 G94"]),
            tool_change: lines(&["M5", "T{tool} M6"]),
            ..Post::grbl()
        }
    }

    /// LNC MW/M-series controllers with a manual tool change, after the Advantech-LNC milling
    /// programming manual (V01.02.000): numbers always carry a decimal point as in its
    /// examples and motion codes two digits; `G04 X` dwells in seconds; arcs are `G02`/`G03` with `I` `J` (helical arcs are
    /// not documented, so ramps go out as lines). `G21` is never written: on these controls it
    /// raises an alert and resets the coordinate systems. Nothing here touches the work
    /// offset or the tool length compensation: a second tool stops the spindle and the
    /// program (`M00`) for a change by hand.
    pub fn lnc() -> Post {
        Post {
            name: "lnc".into(),
            extension: "nc".into(),
            units: Units::Mm,
            decimals: 3,
            feed_decimals: 0,
            line_numbers: None,
            comments: CommentStyle::Parentheses,
            header: lines(&["G17 G40 G80 G90 G94"]),
            tool_change: lines(&["M05", "M00"]),
            tool_change_first: false,
            spindle_on: lines(&["M03 S{rpm}"]),
            spindle_off: lines(&["M05"]),
            coolant_on: lines(&["M08"]),
            coolant_off: lines(&["M09"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
            helical_arcs: false,
            trailing_decimal: true,
            two_digit_codes: true,
            dwell: "G04 X{seconds}".into(),
        }
    }

    /// [`Post::lnc`] for a machine with a tool changer: `T… M06` and `G43 H…` per tool, in
    /// work offset `G54`. `M06` is defined by the machine builder, not by the controller.
    pub fn lnc_atc() -> Post {
        Post {
            name: "lnc_atc".into(),
            header: lines(&["G17 G40 G49 G80 G90 G94 {wcs}"]),
            tool_change: lines(&["M05", "T{tool} M06", "G43 H{tool}"]),
            tool_change_first: true,
            ..Post::lnc()
        }
    }

    /// The built-in post called `name`.
    pub fn builtin(name: &str) -> Option<Post> {
        match name.trim().to_ascii_lowercase().as_str() {
            "rs274" | "linuxcnc" => Some(Post::rs274()),
            "grbl" => Some(Post::grbl()),
            "grblhal" | "grbl_hal" => Some(Post::grblhal()),
            "lnc" => Some(Post::lnc()),
            "lnc_atc" => Some(Post::lnc_atc()),
            _ => None,
        }
    }

    pub const BUILTIN: &'static [&'static str] = &["rs274", "grbl", "grblhal", "lnc", "lnc_atc"];

    pub fn validate(&self) -> Result<()> {
        if self.decimals > 6 || self.feed_decimals > 4 {
            return invalid("a post writes at most 6 decimals for coordinates and 4 for feeds");
        }
        if !(self.arc_tolerance > 0.0) || !self.arc_tolerance.is_finite() {
            return invalid("the post's arc tolerance must be above 0");
        }
        if self.line_numbers.is_some_and(|n| n.step == 0) {
            return invalid("the post's line number step must be above 0");
        }
        if self.extension.is_empty() || self.extension.len() > 8 || !self.extension.chars().all(|c| c.is_ascii_alphanumeric()) {
            return invalid("the post's file extension must be 1 to 8 letters or digits");
        }
        if self.dwell.len() > MAX_TEMPLATE_LEN || self.dwell.chars().any(char::is_control) {
            return invalid("the post's dwell template is too long or has control characters");
        }
        fill(&self.dwell, &Values::default())?;
        for block in [&self.header, &self.tool_change, &self.spindle_on, &self.spindle_off, &self.coolant_on, &self.coolant_off, &self.footer] {
            if block.len() > MAX_TEMPLATE_LINES {
                return Err(CamError::TooLarge(format!("a post template has more than {MAX_TEMPLATE_LINES} lines")));
            }
            for line in block {
                if line.len() > MAX_TEMPLATE_LEN || line.chars().any(char::is_control) {
                    return invalid("a post template line is too long or has control characters");
                }
                // Catches unknown placeholders before any program is written.
                fill(line, &Values::default())?;
            }
        }
        Ok(())
    }
}

/// What the placeholders stand for.
#[derive(Clone, Debug, Default)]
pub(crate) struct Values {
    pub program: String,
    pub units: String,
    pub wcs: String,
    pub tool: String,
    pub diameter: String,
    pub rpm: String,
    pub seconds: String,
    pub milliseconds: String,
}

/// `line` with its placeholders replaced.
pub(crate) fn fill(line: &str, v: &Values) -> Result<String> {
    let mut out = String::with_capacity(line.len());
    let mut rest = line;
    while let Some(open) = rest.find('{') {
        let (before, after) = rest.split_at(open);
        out.push_str(before);
        let Some(close) = after.find('}') else { return invalid(format!("post template `{line}`: `{{` without `}}`")) };
        let (name, tail) = after.split_at(close);
        out.push_str(match name.strip_prefix('{').unwrap_or(name) {
            "program" => &v.program,
            "units" => &v.units,
            "wcs" => &v.wcs,
            "tool" => &v.tool,
            "diameter" => &v.diameter,
            "rpm" => &v.rpm,
            "seconds" => &v.seconds,
            "milliseconds" => &v.milliseconds,
            other => return invalid(format!("post template `{line}`: unknown placeholder `{{{other}}}`")),
        });
        rest = tail.strip_prefix('}').unwrap_or(tail);
    }
    out.push_str(rest);
    Ok(out)
}
