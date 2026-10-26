//! The machine's tool library: the tools every design on this machine can use, with the
//! cutting data operations start from, and the materials stock is made of. It is one JSON file in the per-user data folder
//! (`tools.json`, beside the recovery folder), or wherever `SOLIDCRAFT_TOOLS` says. A design
//! keeps its own copy of each tool it uses, so it opens the same on a machine without them.

use std::fs;
use std::path::{Path, PathBuf};

use serde_json::{Value, json};
use solidcraft_cam::{MAX_MATERIALS, Tool, material_name, same_material};

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

/// A tool library: its tools, and the materials stock on this machine is made of.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Library {
    pub tools: Vec<Tool>,
    pub materials: Vec<String>,
}

impl Library {
    /// Adds `name` to the materials unless it is there already (in any case); `false` when the list is full.
    pub fn add_material(&mut self, name: &str) -> bool {
        if self.materials.iter().any(|m| same_material(m, name)) {
            return true;
        }
        if self.materials.len() >= MAX_MATERIALS {
            return false;
        }
        self.materials.push(name.trim().to_string());
        true
    }
    /// Names every material a tool has cutting data for in the materials as well.
    pub fn add_tool_materials(&mut self) {
        let named: Vec<String> = self.tools.iter().flat_map(|t| t.cutting.iter().map(|c| c.material.clone())).collect();
        for name in named {
            self.add_material(&name);
        }
    }
}

fn fail(path: &Path, why: impl std::fmt::Display) -> EngineError {
    EngineError::Other(format!("tool library {}: {why}", path.display()))
}

/// A library file's text, checked: every tool valid, numbered from 1, none twice; materials named once each.
pub fn parse(text: &[u8]) -> std::result::Result<Library, String> {
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
    let names: Vec<String> = serde_json::from_value(file.get("materials").cloned().unwrap_or_else(|| json!([]))).map_err(|e| e.to_string())?;
    if names.len() > MAX_MATERIALS {
        return Err(format!("more than {MAX_MATERIALS} materials"));
    }
    let mut library = Library { tools, materials: Vec::new() };
    for name in names {
        library.add_material(&material_name(&name).map_err(|e| e.to_string())?);
    }
    library.add_tool_materials();
    Ok(library)
}

/// A library file of ours or in the RP23CNC tool library format (its `toollibrary.json`:
/// `Tools`, `Materials`, and cutting data in `Presets`), with notes on what was left out.
pub fn parse_any(text: &[u8]) -> std::result::Result<(Library, Vec<String>), String> {
    let file: Value = serde_json::from_slice(text).map_err(|e| format!("not a tool library: {e}"))?;
    match file.get("Tools") {
        Some(Value::Array(tools)) if file.get("format").is_none() => from_rp23cnc(tools, file.get("Materials"), file.get("Presets")),
        _ => parse(text).map(|library| (library, Vec::new())),
    }
}

/// RP23CNC tools as ours: end mills, ball end mills, drills and V-bits. A preset without a
/// material gives a tool its own cutting data (the first such one), a preset for a material its
/// data in that material (the first for it); a preset's stepover is a percentage of the
/// diameter. Other kinds of tool are left out, and so is a tool that is not a usable one; the
/// notes say which and why.
fn from_rp23cnc(tools: &[Value], materials: Option<&Value>, presets: Option<&Value>) -> std::result::Result<(Library, Vec<String>), String> {
    use solidcraft_cam::{CuttingData, ToolKind};
    let (mut out, mut notes): (Vec<Tool>, Vec<String>) = (Vec::new(), Vec::new());
    let presets = presets.and_then(Value::as_array).cloned().unwrap_or_default();
    let mut library = Library::default();
    for m in materials.and_then(Value::as_array).into_iter().flatten().take(MAX_MATERIALS) {
        match m.get("Name").and_then(Value::as_str).map(material_name) {
            Some(Ok(name)) => {
                library.add_material(&name);
            }
            _ => notes.push("a material without a usable name".to_string()),
        }
    }
    for t in tools.iter().take(MAX_TOOLS) {
        let number = t.get("Number").and_then(Value::as_u64).and_then(|n| u32::try_from(n).ok()).unwrap_or(0);
        let name = t.get("Name").and_then(Value::as_str).unwrap_or("").chars().filter(|c| !c.is_control()).take(128).collect::<String>();
        let which = if name.is_empty() { format!("tool {number}") } else { format!("tool {number} ({name})") };
        let kind = match t.get("Type").and_then(Value::as_str).unwrap_or("").to_ascii_lowercase().as_str() {
            "endmill" | "flatendmill" | "flat" => ToolKind::FlatEndmill,
            "ballendmill" | "ballnose" | "ball" => ToolKind::BallEndmill,
            "bullnose" | "bull_nose" | "bull-nose" | "bullnoseendmill" | "torus" => ToolKind::BullNose,
            "drill" => ToolKind::Drill,
            "vbit" | "v-bit" | "v_bit" => ToolKind::VBit,
            other => {
                notes.push(format!("{which}: a {other} is not a kind of tool SolidCraft has yet"));
                continue;
            }
        };
        let Some(diameter) = t.get("DiameterMm").and_then(Value::as_f64).filter(|d| *d > 0.0 && d.is_finite()) else {
            notes.push(format!("{which}: no diameter"));
            continue;
        };
        // An end mill rounded at its rim is a bull nose (rounded all the way: a ball).
        let corner = t.get("CornerRadiusMm").and_then(Value::as_f64).filter(|r| *r > 0.0 && r.is_finite());
        let kind = match (kind, corner) {
            (ToolKind::FlatEndmill | ToolKind::BullNose, Some(c)) if c >= diameter * 0.5 - 1e-9 => ToolKind::BallEndmill,
            (ToolKind::FlatEndmill, Some(_)) => ToolKind::BullNose,
            _ => kind,
        };
        if kind == ToolKind::BullNose && corner.is_none() {
            notes.push(format!("{which}: a bull nose without its corner radius"));
            continue;
        }
        let mut tool = Tool::new(number, kind, diameter, diameter * 3.0);
        tool.corner_radius = corner.filter(|_| kind == ToolKind::BullNose);
        tool.description = name;
        if let Some(n) = t.get("FluteCount").and_then(Value::as_u64).filter(|n| (1..=100).contains(n)) {
            tool.flutes = n as u32;
        }
        // A V-bit's point: its angle and the flat at its tip.
        if kind == ToolKind::VBit {
            tool.tip_angle = t.get("VeeAngleDeg").and_then(Value::as_f64).filter(|a| *a > 0.0 && *a < 180.0);
            tool.tip_diameter = t.get("TipDiameterMm").and_then(Value::as_f64).filter(|d| *d > 0.0 && *d < diameter);
        }
        // Its presets: the first without a material is the tool's own data, the first for a material its data in that.
        let mut own: Option<&Value> = None;
        for p in presets.iter().filter(|p| p.get("ToolNumber").and_then(Value::as_u64) == Some(u64::from(number))) {
            let positive = |k: &str| p.get(k).and_then(Value::as_f64).filter(|v| *v > 0.0 && *v <= 1.0e6);
            let data = CuttingData {
                material: String::new(),
                spindle_rpm: positive("SpindleRpm"),
                feed: positive("FeedMmMin"),
                surface_speed: None,
                chip_load: None,
                plunge_feed: positive("PlungeMmMin"),
                stepdown: positive("DepthOfCutMm"),
                stepover: positive("StepoverPercent").map(|pc| (pc / 100.0 * diameter).min(diameter)),
            };
            let preset = p.get("Name").and_then(Value::as_str).unwrap_or("a preset");
            match p.get("Material").and_then(Value::as_str).filter(|m| !m.trim().is_empty()) {
                None if own.is_some() => notes.push(format!(
                    "preset `{preset}` of {which}: its own data is `{}`'s",
                    own.and_then(|o| o.get("Name")).and_then(Value::as_str).unwrap_or("the first")
                )),
                None => {
                    own = Some(p);
                    (tool.spindle_rpm, tool.feed, tool.plunge_feed) = (data.spindle_rpm, data.feed, data.plunge_feed);
                    (tool.stepdown, tool.stepover) = (data.stepdown, data.stepover);
                }
                Some(material) => match material_name(material) {
                    Ok(material) if tool.cutting_in(&material).is_some() => {
                        notes.push(format!("preset `{preset}` of {which}: an earlier one is for {material} already"))
                    }
                    Ok(material) if tool.cutting.len() < MAX_MATERIALS => {
                        library.add_material(&material);
                        tool.cutting.push(CuttingData { material, ..data });
                    }
                    _ => notes.push(format!("preset `{preset}` of {which}: its material cannot be used")),
                },
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
    library.tools = out;
    Ok((library, notes))
}

/// A library as file text.
pub fn text(library: &Library) -> Result<String> {
    serde_json::to_string_pretty(&json!({"format": FORMAT, "tools": library.tools, "materials": library.materials}))
        .map_err(|e| EngineError::Other(e.to_string()))
}

/// The library file; a file that does not exist yet is an empty library.
pub fn load(path: &Path) -> Result<Library> {
    match fs::metadata(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Library::default()),
        Err(e) => return Err(fail(path, e)),
        Ok(m) if m.len() > MAX_BYTES => return Err(fail(path, "the file is too large")),
        Ok(_) => {}
    }
    let bytes = fs::read(path).map_err(|e| fail(path, e))?;
    parse(&bytes).map_err(|e| fail(path, e))
}

/// Writes the library: to a file beside it first, then over it, so a crash leaves the old or the new library.
pub fn save(path: &Path, library: &Library) -> Result<()> {
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        fs::create_dir_all(dir).map_err(|e| fail(path, e))?;
    }
    let mut partial = path.as_os_str().to_owned();
    partial.push(".tmp");
    let partial = PathBuf::from(partial);
    fs::write(&partial, text(library)?).map_err(|e| fail(path, e))?;
    fs::rename(&partial, path).map_err(|e| fail(path, e))
}
