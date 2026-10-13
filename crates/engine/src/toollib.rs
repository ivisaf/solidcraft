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
