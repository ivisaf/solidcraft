//! The machine's tool library: the tools every design on this machine can use, with the
//! cutting data operations start from. It is one JSON file in the per-user data folder
//! (`tools.json`, beside the recovery folder), or wherever `SOLIDCRAFT_TOOLS` says. A design
//! keeps its own copy of each tool it uses, so it opens the same on a machine without them.

use std::fs;
use std::path::{Path, PathBuf};

use serde_json::{Value, json};
use solidcraft_cam::Tool;

use crate::{EngineError, Result};

/// What a tool library file says it is.
pub const FORMAT: &str = "solidcraft-tools/1";
/// Most tools in a library.
pub const MAX_TOOLS: usize = 1000;
/// Largest library file read.
const MAX_BYTES: u64 = 16 << 20;

/// The library file: `SOLIDCRAFT_TOOLS`, else `tools.json` in the per-user data folder.
pub fn default_path() -> Option<PathBuf> {
    // Tests never touch the user's real library.
    if cfg!(test) {
        return Some(std::env::temp_dir().join(format!("solidcraft-test-tools-{}.json", std::process::id())));
    }
    if let Some(p) = solidcraft_geom::env::var_os("TOOLS").filter(|p| !p.is_empty()) {
        return Some(PathBuf::from(p));
    }
    crate::recovery::default_dir()?.parent().map(|dir| dir.join("tools.json"))
}

fn fail(path: &Path, why: impl std::fmt::Display) -> EngineError {
    EngineError::Other(format!("tool library {}: {why}", path.display()))
}

/// The tools in a library file text, checked: every one valid, numbered from 1, none twice.
pub fn parse(text: &[u8]) -> std::result::Result<Vec<Tool>, String> {
    let file: Value = serde_json::from_slice(text).map_err(|e| format!("not a tool library: {e}"))?;
    if file.get("format").and_then(Value::as_str) != Some(FORMAT) {
        return Err(format!("not a tool library (no `format`: `{FORMAT}`)"));
    }
    let mut tools: Vec<Tool> = serde_json::from_value(file.get("tools").cloned().unwrap_or_default()).map_err(|e| e.to_string())?;
    if tools.len() > MAX_TOOLS {
        return Err(format!("more than {MAX_TOOLS} tools"));
    }
    tools.sort_by_key(|t| t.number);
    for tool in &tools {
        tool.validate().map_err(|e| format!("tool {}: {e}", tool.number))?;
        if tool.number == 0 {
            return Err("tool numbers start at 1".into());
        }
    }
    if tools.windows(2).any(|w| matches!(w, [a, b] if a.number == b.number)) {
        return Err("a tool number is used twice".into());
    }
    Ok(tools)
}

/// The tools in a library file of ours or in the RP23CNC tool library format (its
/// `toollibrary.json`: `Tools`, and cutting data in `Presets`), with notes on what was left out.
pub fn parse_any(text: &[u8]) -> std::result::Result<(Vec<Tool>, Vec<String>), String> {
    let file: Value = serde_json::from_slice(text).map_err(|e| format!("not a tool library: {e}"))?;
    match file.get("Tools") {
        Some(Value::Array(tools)) if file.get("format").is_none() => from_rp23cnc(tools, file.get("Presets")),
        _ => parse(text).map(|tools| (tools, Vec::new())),
    }
}

/// RP23CNC tools as ours: end mills, ball end mills and drills, each with the cutting data of
/// the first preset for it (its stepover is a percentage of the diameter). Other kinds of tool
/// are left out, and so is a tool that is not a usable one; the notes say which and why.
fn from_rp23cnc(tools: &[Value], presets: Option<&Value>) -> std::result::Result<(Vec<Tool>, Vec<String>), String> {
    use solidcraft_cam::ToolKind;
    let (mut out, mut notes): (Vec<Tool>, Vec<String>) = (Vec::new(), Vec::new());
    let presets = presets.and_then(Value::as_array).cloned().unwrap_or_default();
    for t in tools.iter().take(MAX_TOOLS) {
        let number = t.get("Number").and_then(Value::as_u64).and_then(|n| u32::try_from(n).ok()).unwrap_or(0);
        let name = t.get("Name").and_then(Value::as_str).unwrap_or("").chars().filter(|c| !c.is_control()).take(128).collect::<String>();
        let which = if name.is_empty() { format!("tool {number}") } else { format!("tool {number} ({name})") };
        let kind = match t.get("Type").and_then(Value::as_str).unwrap_or("").to_ascii_lowercase().as_str() {
            "endmill" | "flatendmill" | "flat" => ToolKind::FlatEndmill,
            "ballendmill" | "ballnose" | "ball" => ToolKind::BallEndmill,
            "drill" => ToolKind::Drill,
            other => {
                notes.push(format!("{which}: a {other} is not a kind of tool SolidCraft has yet"));
                continue;
            }
        };
        let Some(diameter) = t.get("DiameterMm").and_then(Value::as_f64).filter(|d| *d > 0.0 && d.is_finite()) else {
            notes.push(format!("{which}: no diameter"));
            continue;
        };
        let mut tool = Tool::new(number, kind, diameter, diameter * 3.0);
        tool.description = name;
        if let Some(n) = t.get("FluteCount").and_then(Value::as_u64).filter(|n| (1..=100).contains(n)) {
            tool.flutes = n as u32;
        }
        if t.get("CornerRadiusMm").and_then(Value::as_f64).is_some_and(|r| r > 0.0) && kind == ToolKind::FlatEndmill {
            notes.push(format!("{which}: taken as a flat end mill, without its corner radius"));
        }
        // The first preset for the tool gives its cutting data.
        let preset = presets.iter().find(|p| p.get("ToolNumber").and_then(Value::as_u64) == Some(u64::from(number)));
        if let Some(p) = preset {
            let positive = |k: &str| p.get(k).and_then(Value::as_f64).filter(|v| *v > 0.0 && v.is_finite());
            tool.spindle_rpm = positive("SpindleRpm");
            tool.feed = positive("FeedMmMin");
            tool.plunge_feed = positive("PlungeMmMin");
            tool.stepdown = positive("DepthOfCutMm");
            tool.stepover = positive("StepoverPercent").map(|pc| (pc / 100.0 * diameter).min(diameter));
            let more = presets.iter().filter(|q| q.get("ToolNumber").and_then(Value::as_u64) == Some(u64::from(number))).count();
            if more > 1 {
                let name = p.get("Name").and_then(Value::as_str).unwrap_or("the first");
                notes.push(format!("{which}: {more} presets; the cutting data is `{name}`'s"));
            }
        }
        if number == 0 || out.iter().any(|o: &Tool| o.number == number) {
            notes.push(format!("{which}: its number is missing or used twice"));
            continue;
        }
        if let Err(e) = tool.validate() {
            notes.push(format!("{which}: {e}"));
            continue;
        }
        out.push(tool);
    }
    out.sort_by_key(|t| t.number);
    Ok((out, notes))
}

/// A library as file text.
pub fn text(tools: &[Tool]) -> Result<String> {
    serde_json::to_string_pretty(&json!({"format": FORMAT, "tools": tools})).map_err(|e| EngineError::Other(e.to_string()))
}

/// The tools in the library file; a file that does not exist yet is an empty library.
pub fn load(path: &Path) -> Result<Vec<Tool>> {
    match fs::metadata(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(e) => return Err(fail(path, e)),
        Ok(m) if m.len() > MAX_BYTES => return Err(fail(path, "the file is too large")),
        Ok(_) => {}
    }
    let bytes = fs::read(path).map_err(|e| fail(path, e))?;
    parse(&bytes).map_err(|e| fail(path, e))
}

/// Writes the library: to a file beside it first, then over it, so a crash leaves the old or the new library.
pub fn save(path: &Path, tools: &[Tool]) -> Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        fs::create_dir_all(dir).map_err(|e| fail(path, e))?;
    }
    let mut partial = path.as_os_str().to_owned();
    partial.push(".tmp");
    let partial = PathBuf::from(partial);
    fs::write(&partial, text(tools)?).map_err(|e| fail(path, e))?;
    fs::rename(&partial, path).map_err(|e| fail(path, e))
}
