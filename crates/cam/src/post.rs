//! Post processors: how a program is written for one kind of controller. A post is data (it
//! can be read from JSON), not code: line templates around the motion blocks, and a few
//! switches for how the motion itself is written.
//!
//! Templates may use `{program}` (program name), `{units}` (`G21` or `G20`), `{tool}` (tool
//! number), `{diameter}` (tool diameter in output units) and `{rpm}` (spindle speed).

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
            header: lines(&["G17 {units} G40 G49 G54 G80 G90 G94"]),
            tool_change: lines(&["M5", "T{tool} M6", "G43 H{tool}"]),
            tool_change_first: true,
            spindle_on: lines(&["S{rpm} M3"]),
            spindle_off: lines(&["M5"]),
            coolant_on: lines(&["M8"]),
            coolant_off: lines(&["M9"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
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
            header: lines(&["G17 {units} G54 G90 G94"]),
            tool_change: lines(&["M5", "M0"]),
            tool_change_first: false,
            spindle_on: lines(&["S{rpm} M3"]),
            spindle_off: lines(&["M5"]),
            coolant_on: lines(&["M8"]),
            coolant_off: lines(&["M9"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
        }
    }

    /// The built-in post called `name`.
    pub fn builtin(name: &str) -> Option<Post> {
        match name.trim().to_ascii_lowercase().as_str() {
            "rs274" | "linuxcnc" => Some(Post::rs274()),
            "grbl" => Some(Post::grbl()),
            _ => None,
        }
    }

    pub const BUILTIN: &'static [&'static str] = &["rs274", "grbl"];

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
    pub tool: String,
    pub diameter: String,
    pub rpm: String,
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
            "tool" => &v.tool,
            "diameter" => &v.diameter,
            "rpm" => &v.rpm,
            other => return invalid(format!("post template `{line}`: unknown placeholder `{{{other}}}`")),
        });
        rest = tail.strip_prefix('}').unwrap_or(tail);
    }
    out.push_str(rest);
    Ok(out)
}
