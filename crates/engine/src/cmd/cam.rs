//! Manufacture: the tool library, setups and their milling and drilling operations, the
//! toolpaths worked out from them, and G-code programs. Operations cut where a sketch's
//! profiles are; toolpaths are never stored, so they follow the design.

use serde_json::{Value, json};
use solidcraft_cam::{
    self as cam, AdaptiveParams, BoreParams, ChamferParams, ContourParams, Cut, CuttingData, DrillParams, Feeds, Heights, PocketParams, Post, Side,
    Tool, ToolKind, Toolpath,
};
use solidcraft_doc::Expr;
use solidcraft_doc::expr::Kind;
use solidcraft_doc::manufacture::{Anchor, BoxOf, CamGeometry, CamOperation, CamOperationKind, FaceRef, Setup, SetupOrigin, StockDef, TabsDef};
use solidcraft_doc::sketch::{LinkGeom, LinkKind};
use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};

use super::CommandSpec;
use crate::params::{bad, bool_, expr, num, req_expr, str_, vec3};
use crate::{EngineError, Result, Session, toollib};

const MAX_TOOLS: usize = toollib::MAX_TOOLS;
const MAX_SETUPS: usize = 100;
const MAX_OPERATIONS: usize = 1000;
const MAX_NAME: usize = 128;
/// Least depth into the part `cam.list` warns of (mm): a column of the part's height field either way.
const GOUGE_REPORTED: f64 = 0.05;
/// Most selected items a form's heights are read from.
const MAX_SELECTED: usize = 10_000;
/// Chord error allowed where spline profiles are followed (mm).
const TOLERANCE: f64 = 0.01;
/// Most toolpath points `cam.toolpath` returns for one operation.
const MAX_POINTS: usize = 500_000;

/// Parameter documentation of an operation: its own inputs, then what every operation takes.
macro_rules! op_doc {
    ($own:literal) => {
        concat!(
            $own,
            "; library?: the machine's tool library file (default: this machine's); sketch: id or name, with profiles?: as solid.extrude (default all) — or faces: [{body?, point: [x, y, z]}], planar faces square to Z (a pocket's floor, a part's top; their holes are islands, or what a drill drills) or round walls along Z (a hole's or a boss's: their circle, from the wall's top to its foot); tool: number, from this design or the machine's library (the design then keeps a copy); spindle_rpm?, feed?: mm/min, plunge_feed?: mm/min \
             (defaults: the tool's data in the setup's material, else its own; plunge_feed else feed/3); bottom?, top?: exprs in the setup's Z (a sketch gives the top and needs `bottom`; faces give both from the face and its body); \
             retract?: expr (default top + 2 mm); clearance?: expr (default 10 mm above the top and the body); stepdown?: expr (default: the tool's, else half its diameter); passes?: how many depth passes instead (each the depth over this; null: by the stepdown); coolant?: bool; \
             setup?: id or name (default: the only setup); name? → operation: id, name, and its toolpath's cut_length, rapid_length, cut_minutes"
        )
    };
}

const SETUP_DOC: &str = "name?; axis?: +x|-x|+y|-y|+z|-z or [x, y, z], the direction the tool comes from: the setup's +Z (default +z); \
     stock?: {side?, top?, bottom?}: lengths added to the box around the bodies, seen along the axis (default 0); origin?: the work origin: \
     [x, y, z] in the world, or {of?: stock|part, x?, y?, z?: min|center|max} on the stock's or the part's box in the setup's axes (default of \
     the object: the stock's top at its lowest X and Y; default without `origin`: the world origin); post?: rs274|grbl|grblhal|lnc|lnc_atc (default \
     rs274); material?: what the stock is made of (operations take their tool's cutting data for it; null: none) → setup: id, name";

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool.define", "Define Tool", tool_define).icon("library")
        .params(
            "number: 1–9999; kind: flat|ball|drill|vbit; diameter: expr; tip_angle?: degrees, the point's included angle (V-bit: default 90; drill: 118); tip_diameter?: mm, a V-bit's flat at its point; shank_diameter?, length?: mm, the shank and the length out of the holder (for its picture); flute_length?: expr (default 3 × diameter; a V-bit's, its cone's height); description?; flutes?: 1–100 \
             (default 2); spindle_rpm?, feed?, plunge_feed?: mm/min, stepdown?, stepover?: mm — the cutting data operations with this tool start \
             from (null clears one); chip_load?: mm per tooth, instead of feed: feed = spindle_rpm × flutes × chip_load (listings report it); cutting?: [{material, spindle_rpm?, feed?, chip_load?, plunge_feed?, stepdown?, stepover?}], its data in particular materials, replacing its own where given (null: none) — adds the tool to this design or replaces its tool of that number (cam.library.define: the machine's library)",
        ),
    CommandSpec::new("cam.tool.update", "Update Design Tools", tool_update).icon("update")
        .params("number?: one tool (default: every tool) — replaces this design's copy with the machine library's tool of that number where they differ; library? → updated: tool numbers"),
    CommandSpec::new("cam.library.list", "Tool Library", library_list)
        .at("MANUFACTURE", "SETUP")
        .icon("cam_tool")
        .noundo()
        .params("library?: file (default: the machine's library) → path, tools (each with design: absent|same|differs, this design's copy against it), only_in_design, materials: the names stock on this machine is made of"),
    CommandSpec::new("cam.library.define", "Define Library Tool", library_define).icon("library")
        .noundo()
        .params("as cam.tool.define, into the machine's library: tools every design on this machine can use; library?: file"),
    CommandSpec::new("cam.library.remove", "Remove Library Tool", library_remove).icon("delete").noundo().params("number; library? (designs keep their copies)"),
    CommandSpec::new("cam.library.add", "Add To Library", library_add).icon("library").noundo().params("number: a tool of this design to put in the machine's library — or all: true; library?"),
    CommandSpec::new("cam.library.export", "Export Library", library_export).icon("export").noundo().params("path: a file to write the machine's library to; library?"),
    CommandSpec::new("cam.library.import", "Import Library", library_import).icon("import")
        .noundo()
        .params("path: a tool library file (ours, or an RP23CNC toollibrary.json: its end mills, ball end mills and drills, with their first preset's cutting data); replace?: bool (default false: tools whose number is taken are skipped); library? → added, replaced, skipped, notes: what was left out"),
    CommandSpec::new("cam.material.define", "Define Material", material_define)
        .icon("library")
        .noundo()
        .params("name: a material stock on this machine is made of, for setups to choose and tools to have cutting data for; library?"),
    CommandSpec::new("cam.material.remove", "Remove Material", material_remove)
        .icon("delete")
        .noundo()
        .params("name; library? — the library's tools lose their cutting data for it (designs keep theirs) → removed, tools: the numbers of those that had some"),
    CommandSpec::new("cam.tool.export", "Export Tools", tool_export).icon("export").noundo().params("path: a JSON file to write the tool library to"),
    CommandSpec::new("cam.tool.import", "Import Tools", tool_import).icon("import")
        .params("path: a tool library written by cam.tool.export; replace?: bool (default false: tools whose number is taken are skipped) → added, replaced, skipped: tool numbers"),
    CommandSpec::new("cam.tool.remove", "Remove Tool", tool_remove).icon("delete").params("number (refused while an operation uses it)"),
    CommandSpec::new("cam.setup.create", "New Setup", setup_create)
        .at("MANUFACTURE", "SETUP")
        .icon("cam_setup")
        .params(SETUP_DOC),
    CommandSpec::new("cam.setup.edit", "Edit Setup", setup_edit).icon("edit").params("setup: id or name; then any of name, axis, origin, stock, post as in cam.setup.create (stock offsets left out stay as they are)"),
    CommandSpec::new("cam.setup.remove", "Remove Setup", setup_remove).icon("delete").params("setup: id or name"),
    CommandSpec::new("cam.op.face", "Facing", op_face).at("MANUFACTURE", "MILLING").icon("cam_face").params(op_doc!("stepover?: expr (default: the tool's own, else half its diameter; at most the diameter); faces the box around the profiles or faces — or, without them, boundary?: stock|part (default stock), from the stock's top down to the part's; limit?: over|inside the boundary (default over); margin?: expr, how far over or inside (default: over by the tool's radius, inside by 0)")),
    CommandSpec::new("cam.op.contour", "2D Contour", op_contour).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("side?: outside|inside|on (default outside: the profiles are the part); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down); tabs?: {count?: on each loop (default 4), width?: expr along the edge (default 6 mm), height?: expr over the bottom (default 2 mm)}, holding a part being cut out (null: none); finish?: expr, an allowance the depth passes leave on the wall for one more pass at full depth (null: none)")),
    CommandSpec::new("cam.op.pocket", "2D Pocket", op_pocket).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("stepover?: expr (default: the tool's own, else 40% of its diameter; at most the diameter); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down); finish?: expr, an allowance the depth passes leave on the walls for one more pass along them at full depth (null: none); rest_from?: a larger tool's number (of this design) that cleared the pocket before: only what it left (corners, narrow places) is cut (null: the whole pocket); inner profiles are islands")),
    CommandSpec::new("cam.op.adaptive", "Adaptive", op_adaptive).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("stepover_percent?: the heaviest cut, as a percentage of the tool's diameter (2 to 50, default 20): the tool steps through the material so that no step cuts wider, inner corners and narrow places included; stock_to_leave?: expr; finish?: expr, an allowance the passes leave on the walls for one more lap at full depth (null: none); helix_angle?: angle expr, the entry helix's (or ramp's) slope (default 5 deg); helix_max?, helix_min?: the entry helix's diameter as a percentage of the tool's, at most and at least (default 100, 10); climb?: bool (default true); rest_from?: a larger tool's number (of this design) that cleared the pocket before: only what it left is cut (null: the whole pocket); feed_up?: percent of the feed (100 to 500; null or 100: never faster): a step lighter than the stepover makes thinner chips and goes as much faster, up to this; cuts go as arcs where their steps lie on one; inner profiles are islands")),
    CommandSpec::new("cam.op.chamfer", "Chamfer", op_chamfer).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("width?: expr across the top (default 1 mm); tip_offset?: expr, how much deeper than the chamfer the tool's point goes (default 0.5 mm); side?: outside|inside (default outside: the profiles are the part); climb?: bool (default true); ramp_angle?: angle expr (default 3 deg); chamfers the top edges of the profiles or picked faces in one pass with a V-bit or a drill, at the tool's angle; the depth comes from the width (bottom is not used)")),
    CommandSpec::new("cam.op.bore", "Bore", op_bore).at("MANUFACTURE", "DRILLING").icon("cam_drill").params(op_doc!("pitch?: expr, how far down one turn of the helix goes (default: the tool's stepdown, else a quarter of its diameter); climb?: bool (default true); stock_to_leave?: expr; mills each round profile (or a picked face's round holes, a picked hole's wall) larger than the tool on a helix down its wall, then once round at the bottom; a hole more than twice the tool's diameter keeps a core")),
    CommandSpec::new("cam.op.drill", "Drill", op_drill).at("MANUFACTURE", "DRILLING").icon("cam_drill").params(op_doc!("peck?: expr (depth per peck; default one pass); dwell?: seconds at the bottom; drills at the centre of each profile at the plunge feed")),
    CommandSpec::new("cam.op.edit", "Edit Operation", op_edit).icon("edit").params(
        "op: id or name; setup?: id or name; then any input of the operation's own command to change it (name, tool, spindle_rpm, feed, \
         plunge_feed, coolant, stepdown, passes, sketch with profiles?, faces, boundary, limit, margin, stepover, side, climb, stock_to_leave, ramp_angle, tabs, finish, rest_from, stepover_percent, helix_angle, helix_max, helix_min, feed_up, pitch, width, tip_offset, peck, dwell); a height \
         (top, bottom, retract, clearance) or peck given as null goes back to automatic → the operation's toolpath lengths and time",
    ),
    CommandSpec::new("cam.op.move", "Move Operation", op_move).icon("reorder")
        .params("op: id or name; to: position in the setup's order (0 is cut first) — or by: places later (negative: earlier); setup?: id or name → order: the setup's operation names"),
    CommandSpec::new("cam.op.remove", "Remove Operation", op_remove).icon("delete").params("op: id or name; setup?: id or name"),
    CommandSpec::new("cam.list", "Manufacture", list)
        .at("MANUFACTURE", "ACTIONS")
        .icon("cam_setup")
        .noundo()
        .params("→ tools and setups with their operations; each operation with its toolpath's lengths and time, or the error that stops it; warning: how deep it cuts into the part, when it does; materials: those the setups and tools name"),
    CommandSpec::new("cam.toolpath", "Show Toolpaths", toolpath).at("MANUFACTURE", "ACTIONS").icon("cam_contour").noundo().params(
        "setup?: id or name; op?: id or name (default every operation); points?: bool (default true) → operations: [{id, name, cut_length, \
         rapid_length, cut_minutes, lowest_z, points: [[x, y, z, cutting]] in world coordinates}]; heights?: bool, also each one's heights: {clearance, retract, top, bottom, levels: [depth passes]} in the setup's Z",
    ),
    CommandSpec::new("cam.simulate", "Simulate", simulate).at("MANUFACTURE", "ACTIONS").icon("cam_simulate").noundo().params(
        "setup?: id or name; stock?: {min: [x, y, z], max: [x, y, z]} in the setup's work coordinates (default: the setup's stock); cell?: column \
         width in mm (default: the stock's longer side / 300) — or columns?: how many along the stock's longer side (10 to 2000) → per operation removed_volume, rapid_collisions, max_engagement and warnings; and \
         against the bodies: max_gouge, gouge_area (cut into the part), max_leftover, leftover_volume (left on it); field?: bool, also the cut stock as columns {frame, origin, cell, nx, ny, floor, z: [nx × ny heights], \
         deviation: [per column: below 0 cut into the part, above 0 left on it]} in work coordinates, with the frame that places them. Top-down: undercuts are not seen",
    ),
    CommandSpec::new("cam.post", "Post Process", post).at("MANUFACTURE", "ACTIONS").icon("cam_post").noundo().params(
        "setup?: id or name; op?: id or name, to write that operation alone (default: every operation, in order); post?: rs274|grbl|grblhal|lnc|lnc_atc or a post definition object (default: the setup's); path?: file to write → lines, extension, \
         cut_minutes, and gcode (the program text) when no path is given",
    ),
];

fn cam_error(e: cam::CamError) -> EngineError {
    EngineError::Other(e.to_string())
}

/// A length as an expression, to the nanometre (so 6 × 0.4 reads `2.4 mm`).
fn mm(v: f64) -> Expr {
    Expr::new(format!("{} mm", (v * 1e6).round() / 1e6))
}

fn length(s: &Session, cmd: &str, key: &str, e: &str) -> Result<f64> {
    s.doc.eval(e, Kind::Length).map_err(|e| bad(cmd, format!("{key}: {e}")))
}

fn name_param(p: &Value, cmd: &str) -> Result<Option<String>> {
    match str_(p, "name").map(str::trim).filter(|n| !n.is_empty()) {
        Some(n) if n.len() <= MAX_NAME && !n.chars().any(char::is_control) => Ok(Some(n.to_string())),
        Some(_) => Err(bad(cmd, "`name` is too long or has control characters")),
        None => Ok(None),
    }
}

/// A key that names something by id or by name.
fn key(p: &Value, k: &str) -> Option<String> {
    match p.get(k)? {
        Value::Number(n) => Some(n.to_string()),
        Value::String(s) if !s.trim().is_empty() => Some(s.trim().to_string()),
        _ => None,
    }
}

fn setup_index(s: &Session, p: &Value, cmd: &str) -> Result<usize> {
    let setups = &s.doc.manufacture.setups;
    match key(p, "setup") {
        Some(k) => setups
            .iter()
            .position(|x| x.id.to_string() == k)
            .or_else(|| setups.iter().position(|x| x.name.eq_ignore_ascii_case(&k)))
            .ok_or_else(|| bad(cmd, format!("no setup `{k}`"))),
        None if setups.len() == 1 => Ok(0),
        None if setups.is_empty() => Err(bad(cmd, "there is no setup yet (cam.setup.create first)")),
        None => Err(bad(cmd, "`setup` is required: the design has several setups")),
    }
}

fn setup<'a>(s: &'a Session, p: &Value, cmd: &str) -> Result<&'a Setup> {
    let i = setup_index(s, p, cmd)?;
    s.doc.manufacture.setups.get(i).ok_or_else(|| bad(cmd, "no such setup"))
}

fn next_id(s: &mut Session) -> u64 {
    let d = s.doc_mut();
    let id = d.next_id.max(1);
    d.next_id = id + 1;
    id
}

/// A tool from a command's inputs. Number, kind and diameter are always given; the rest, left
/// out, keeps what the tool `was`. Cutting data: a number sets it, null clears it.
fn tool_param(s: &Session, p: &Value, cmd: &str, was: Option<&Tool>) -> Result<Tool> {
    let number =
        p.get("number").and_then(Value::as_u64).filter(|n| (1..=9999).contains(n)).ok_or_else(|| bad(cmd, "`number` must be 1 to 9999"))? as u32;
    let kind = match str_(p, "kind").map(|k| k.trim().to_ascii_lowercase()).as_deref() {
        Some("flat" | "flat_endmill" | "endmill") => ToolKind::FlatEndmill,
        Some("ball" | "ball_endmill") => ToolKind::BallEndmill,
        Some("drill") => ToolKind::Drill,
        Some("vbit" | "v_bit" | "v-bit") => ToolKind::VBit,
        _ => return Err(bad(cmd, "`kind` must be flat, ball, drill or vbit")),
    };
    let diameter = length(s, cmd, "diameter", &req_expr(cmd, p, "diameter")?)?;
    // What is not named stays as the tool had it.
    let flute_length = match expr(p, "flute_length") {
        Some(e) => length(s, cmd, "flute_length", &e)?,
        None => was.map_or_else(
            || match kind {
                // A V-bit cuts with its cone: as long as the cone is high.
                ToolKind::VBit => {
                    let angle = num(p, "tip_angle").filter(|a| *a > 0.0 && *a < 180.0).unwrap_or(90.0);
                    let tip = num(p, "tip_diameter").filter(|t| *t >= 0.0 && *t < diameter).unwrap_or(0.0);
                    ((diameter - tip) * 0.5 / (angle * 0.5).to_radians().tan()).max(0.01)
                }
                _ => diameter * 3.0,
            },
            |t| t.flute_length,
        ),
    };
    let description: String = match p.get("description") {
        None => was.map(|t| t.description.clone()).unwrap_or_default(),
        Some(_) => str_(p, "description").unwrap_or("").chars().filter(|c| !c.is_control()).take(MAX_NAME).collect(),
    };
    let data = |k: &str, was: Option<f64>| match p.get(k) {
        None => Ok(was),
        Some(Value::Null) => Ok(None),
        Some(_) => num(p, k).filter(|v| *v > 0.0 && *v <= 1.0e6).map(Some).ok_or_else(|| bad(cmd, format!("`{k}` must be above 0"))),
    };
    let flutes = match p.get("flutes") {
        None => was.map_or(2, |t| t.flutes),
        Some(v) => v.as_u64().filter(|n| (1..=100).contains(n)).ok_or_else(|| bad(cmd, "`flutes` must be 1 to 100"))? as u32,
    };
    let spindle_rpm = data("spindle_rpm", was.and_then(|t| t.spindle_rpm))?;
    let mut feed = data("feed", was.and_then(|t| t.feed))?;
    // Chip load (feed per tooth) ties the feed to the speed and the flutes: feed = speed × flutes × chip load.
    if p.get("chip_load").is_some_and(|v| !v.is_null()) {
        let chip = num(p, "chip_load").filter(|c| *c > 0.0 && *c <= 100.0).ok_or_else(|| bad(cmd, "`chip_load` must be above 0 (mm per tooth)"))?;
        let rpm = spindle_rpm.ok_or_else(|| bad(cmd, "`chip_load` needs a `spindle_rpm` to work the feed out from"))?;
        let from_chip = (rpm * f64::from(flutes) * chip * 1e6).round() / 1e6;
        match p.get("feed").filter(|v| !v.is_null()).and(feed) {
            Some(given) if (given - from_chip).abs() > from_chip * 0.005 => {
                return Err(bad(
                    cmd,
                    format!("`feed` {given} and `chip_load` {chip} disagree: at {rpm} rev/min with {flutes} flutes the chip load gives {from_chip}"),
                ));
            }
            Some(_) => {}
            None => feed = Some(from_chip),
        }
    }
    let cutting = match p.get("cutting") {
        None => was.map(|t| t.cutting.clone()).unwrap_or_default(),
        Some(v) => cutting_param(v, cmd, flutes, spindle_rpm)?,
    };
    let tool = Tool {
        number,
        kind,
        diameter,
        flute_length,
        description,
        flutes,
        spindle_rpm,
        feed,
        plunge_feed: data("plunge_feed", was.and_then(|t| t.plunge_feed))?,
        stepdown: data("stepdown", was.and_then(|t| t.stepdown))?,
        stepover: data("stepover", was.and_then(|t| t.stepover))?,
        // The point (a V-bit's, a drill's) and the picture's shank and length: kept as they were when not named.
        tip_angle: data("tip_angle", was.and_then(|t| t.tip_angle))?,
        tip_diameter: match p.get("tip_diameter") {
            None => was.and_then(|t| t.tip_diameter),
            Some(Value::Null) => None,
            Some(_) => {
                Some(num(p, "tip_diameter").filter(|d| *d >= 0.0 && d.is_finite()).ok_or_else(|| bad(cmd, "`tip_diameter` must be 0 or more"))?)
                    .filter(|d| *d > 0.0)
            }
        },
        shank_diameter: data("shank_diameter", was.and_then(|t| t.shank_diameter))?,
        length: data("length", was.and_then(|t| t.length))?,
        cutting,
    };
    tool.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(tool)
}

/// `cutting`: a tool's data in particular materials (null: none). A chip load gives the feed
/// from the material's spindle speed, else the tool's own; a material given nothing is left out.
fn cutting_param(v: &Value, cmd: &str, flutes: u32, own_rpm: Option<f64>) -> Result<Vec<CuttingData>> {
    if v.is_null() {
        return Ok(Vec::new());
    }
    let list = v
        .as_array()
        .filter(|l| l.len() <= cam::MAX_MATERIALS)
        .ok_or_else(|| bad(cmd, format!("`cutting` must be a list of at most {} materials", cam::MAX_MATERIALS)))?;
    let (mut out, mut seen): (Vec<CuttingData>, Vec<String>) = (Vec::new(), Vec::new());
    for e in list {
        let material = cam::material_name(str_(e, "material").unwrap_or("")).map_err(|err| bad(cmd, format!("cutting: {err}")))?;
        if seen.iter().any(|m| cam::same_material(m, &material)) {
            return Err(bad(cmd, format!("cutting: {material} twice")));
        }
        seen.push(material.clone());
        let value = |k: &str| match e.get(k) {
            None | Some(Value::Null) => Ok(None),
            Some(_) => num(e, k)
                .filter(|v| *v > 0.0 && *v <= 1.0e6)
                .map(Some)
                .ok_or_else(|| bad(cmd, format!("cutting in {material}: `{k}` must be above 0"))),
        };
        let mut data = CuttingData {
            material: material.clone(),
            spindle_rpm: value("spindle_rpm")?,
            feed: value("feed")?,
            plunge_feed: value("plunge_feed")?,
            stepdown: value("stepdown")?,
            stepover: value("stepover")?,
        };
        if let Some(chip) = value("chip_load")?.filter(|_| data.feed.is_none()) {
            let rpm = data.spindle_rpm.or(own_rpm).ok_or_else(|| bad(cmd, format!("cutting in {material}: `chip_load` needs a spindle speed")))?;
            data.feed = Some((rpm * f64::from(flutes) * chip * 1e6).round() / 1e6);
        }
        if !data.is_empty() {
            out.push(data);
        }
    }
    Ok(out)
}

/// The materials a design names: its setups' and its tools' (in the order met, each once).
fn design_materials(s: &Session) -> Vec<String> {
    let m = &s.doc.manufacture;
    let mut out: Vec<String> = Vec::new();
    let named = m.setups.iter().filter_map(|x| x.material.clone()).chain(m.tools.iter().flat_map(|t| t.cutting.iter().map(|c| c.material.clone())));
    for name in named {
        if !out.iter().any(|o| cam::same_material(o, &name)) {
            out.push(name);
        }
    }
    out
}

/// A setup's `material`: `None` when not given, `Some(None)` for null or "".
fn material_param(p: &Value, cmd: &str) -> Result<Option<Option<String>>> {
    match p.get("material") {
        None => Ok(None),
        Some(Value::Null) => Ok(Some(None)),
        Some(Value::String(m)) if m.trim().is_empty() => Ok(Some(None)),
        Some(Value::String(m)) => cam::material_name(m).map(|m| Some(Some(m))).map_err(|e| bad(cmd, e.to_string())),
        Some(_) => Err(bad(cmd, "`material` must be a name")),
    }
}

/// A tool for a listing: itself, and its chip load (mm per tooth) when its speed and feed say what it is.
fn tool_json(tool: &Tool) -> Value {
    let mut v = json!(tool);
    if let (Some(rpm), Some(feed)) = (tool.spindle_rpm, tool.feed) {
        v["chip_load"] = json!(feed / (rpm * f64::from(tool.flutes)));
    }
    v
}

fn tool_define(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.define";
    let number = p.get("number").and_then(Value::as_u64).unwrap_or(0);
    let was = s.doc.manufacture.tools.iter().find(|t| u64::from(t.number) == number).cloned();
    let tool = tool_param(s, p, cmd, was.as_ref())?;
    if was.is_none() && s.doc.manufacture.tools.len() >= MAX_TOOLS {
        return Err(bad(cmd, "the tool library is full"));
    }
    let tools = &mut s.doc_mut().manufacture.tools;
    tools.retain(|t| t.number != tool.number);
    tools.push(tool.clone());
    tools.sort_by_key(|t| t.number);
    Ok(json!({ "tool": tool_json(&tool) }))
}

/// The machine's library file a command works on: `library`, else the default.
fn library_path(p: &Value, cmd: &str) -> Result<std::path::PathBuf> {
    match str_(p, "library").filter(|d| !d.trim().is_empty() && d.len() < 4096) {
        Some(d) => Ok(d.into()),
        None => toollib::default_path().ok_or_else(|| bad(cmd, "no place for a tool library on this system (give `library`)")),
    }
}

/// The machine library's tool `number`, if the library can be read and has it.
fn library_tool(p: &Value, number: u64) -> Option<Tool> {
    let path = library_path(p, "").ok()?;
    toollib::load(&path).ok()?.tools.into_iter().find(|t| u64::from(t.number) == number)
}

fn library_list(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.list";
    let path = library_path(p, cmd)?;
    let library = toollib::load(&path)?;
    let tools: Vec<Value> = library
        .tools
        .iter()
        .map(|tool| {
            let mut v = tool_json(tool);
            // How this design's copy of the tool stands: none, the same, or changed since.
            v["design"] = json!(match s.doc.manufacture.tool(tool.number) {
                None => "absent",
                Some(copy) if copy == tool => "same",
                Some(_) => "differs",
            });
            v
        })
        .collect();
    let numbers: Vec<u32> = tools.iter().filter_map(|t| t["number"].as_u64()).map(|n| n as u32).collect();
    let only_in_design: Vec<Value> = s.doc.manufacture.tools.iter().filter(|t| !numbers.contains(&t.number)).map(tool_json).collect();
    Ok(json!({"path": path, "tools": tools, "only_in_design": only_in_design, "materials": library.materials}))
}

fn library_define(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.define";
    let path = library_path(p, cmd)?;
    let mut library = toollib::load(&path)?;
    let number = p.get("number").and_then(Value::as_u64).unwrap_or(0);
    let was = library.tools.iter().find(|t| u64::from(t.number) == number).cloned();
    let tool = tool_param(s, p, cmd, was.as_ref())?;
    if was.is_none() && library.tools.len() >= toollib::MAX_TOOLS {
        return Err(bad(cmd, "the tool library is full"));
    }
    let tools = &mut library.tools;
    tools.retain(|t| t.number != tool.number);
    tools.push(tool.clone());
    tools.sort_by_key(|t| t.number);
    library.add_tool_materials();
    toollib::save(&path, &library)?;
    Ok(json!({ "tool": tool_json(&tool), "path": path }))
}

fn library_remove(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.remove";
    let path = library_path(p, cmd)?;
    let number = p.get("number").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`number` is required"))?;
    let mut library = toollib::load(&path)?;
    if !library.tools.iter().any(|t| u64::from(t.number) == number) {
        return Err(bad(cmd, format!("no tool {number} in the library")));
    }
    library.tools.retain(|t| u64::from(t.number) != number);
    toollib::save(&path, &library)?;
    Ok(json!({ "removed": number }))
}

/// Puts a design's tool into the machine library (`number`), or every tool of the design (`all: true`).
fn library_add(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.add";
    let path = library_path(p, cmd)?;
    let number = p.get("number").and_then(Value::as_u64);
    if number.is_none() && !bool_(p, "all").unwrap_or(false) {
        return Err(bad(cmd, "give `number`, or `all`: true"));
    }
    let chosen: Vec<Tool> = s.doc.manufacture.tools.iter().filter(|t| number.is_none_or(|n| u64::from(t.number) == n)).cloned().collect();
    if chosen.is_empty() {
        return Err(bad(cmd, "the design has no such tool"));
    }
    let mut library = toollib::load(&path)?;
    let tools = &mut library.tools;
    tools.retain(|t| !chosen.iter().any(|c| c.number == t.number));
    tools.extend(chosen.iter().cloned());
    if tools.len() > toollib::MAX_TOOLS {
        return Err(bad(cmd, "the tool library would be too large"));
    }
    tools.sort_by_key(|t| t.number);
    library.add_tool_materials();
    toollib::save(&path, &library)?;
    Ok(json!({ "added": chosen.iter().map(|t| t.number).collect::<Vec<_>>() }))
}

/// Brings this design's copies of library tools up to date: `number`, or every copy that differs.
fn tool_update(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.update";
    let path = library_path(p, cmd)?;
    let library = toollib::load(&path)?;
    let number = p.get("number").and_then(Value::as_u64);
    let mut updated = Vec::new();
    let mut tools = s.doc.manufacture.tools.clone();
    for copy in tools.iter_mut().filter(|t| number.is_none_or(|n| u64::from(t.number) == n)) {
        if let Some(tool) = library.tools.iter().find(|t| t.number == copy.number).filter(|t| *t != copy) {
            *copy = tool.clone();
            updated.push(tool.number);
        }
    }
    if let Some(n) = number.filter(|n| !tools.iter().any(|t| u64::from(t.number) == *n)) {
        return Err(bad(cmd, format!("the design has no tool {n}")));
    }
    if !updated.is_empty() {
        s.doc_mut().manufacture.tools = tools;
    }
    Ok(json!({ "updated": updated }))
}

/// A library file a command was pointed at (ours, or another program's we read), and notes on what was left out.
fn read_library(path: &str, cmd: &str) -> Result<(toollib::Library, Vec<String>)> {
    let bytes = solidcraft_io::vfs::read(path, 16 << 20).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    toollib::parse_any(&bytes).map_err(|e| bad(cmd, format!("{path}: {e}")))
}

/// `incoming` merged into `library`: tools whose number is taken replace it or are skipped. → added, replaced, skipped.
fn merge_tools(library: &mut Vec<Tool>, incoming: Vec<Tool>, replace: bool, cmd: &str) -> Result<Value> {
    let (mut added, mut replaced, mut skipped) = (Vec::new(), Vec::new(), Vec::new());
    for tool in incoming {
        match library.iter_mut().find(|t| t.number == tool.number) {
            Some(_) if !replace => skipped.push(tool.number),
            Some(slot) => {
                replaced.push(tool.number);
                *slot = tool;
            }
            None => {
                added.push(tool.number);
                library.push(tool);
            }
        }
    }
    if library.len() > MAX_TOOLS {
        return Err(bad(cmd, "the tool library would be too large"));
    }
    library.sort_by_key(|t| t.number);
    Ok(json!({"added": added, "replaced": replaced, "skipped": skipped}))
}

fn tool_export(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.export";
    let path = str_(p, "path").ok_or_else(|| bad(cmd, "`path` is required"))?;
    let library = toollib::Library { tools: s.doc.manufacture.tools.clone(), materials: design_materials(s) };
    solidcraft_io::vfs::write(path, toollib::text(&library)?.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    Ok(json!({"path": path, "tools": library.tools.len()}))
}

fn tool_import(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.import";
    let path = str_(p, "path").ok_or_else(|| bad(cmd, "`path` is required"))?;
    let (incoming, notes) = read_library(path, cmd)?;
    let mut library = s.doc.manufacture.tools.clone();
    let mut out = merge_tools(&mut library, incoming.tools, bool_(p, "replace").unwrap_or(false), cmd)?;
    s.doc_mut().manufacture.tools = library;
    out["notes"] = json!(notes);
    Ok(out)
}

fn library_export(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.export";
    let path = str_(p, "path").ok_or_else(|| bad(cmd, "`path` is required"))?;
    let library = toollib::load(&library_path(p, cmd)?)?;
    solidcraft_io::vfs::write(path, toollib::text(&library)?.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
    Ok(json!({"path": path, "tools": library.tools.len()}))
}

fn library_import(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.library.import";
    let path = str_(p, "path").ok_or_else(|| bad(cmd, "`path` is required"))?;
    let (incoming, notes) = read_library(path, cmd)?;
    let file = library_path(p, cmd)?;
    let mut library = toollib::load(&file)?;
    let mut out = merge_tools(&mut library.tools, incoming.tools, bool_(p, "replace").unwrap_or(false), cmd)?;
    for name in &incoming.materials {
        library.add_material(name);
    }
    library.add_tool_materials();
    toollib::save(&file, &library)?;
    out["notes"] = json!(notes);
    out["materials"] = json!(incoming.materials);
    Ok(out)
}

fn material_define(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.material.define";
    let name = cam::material_name(str_(p, "name").unwrap_or("")).map_err(|e| bad(cmd, e.to_string()))?;
    let path = library_path(p, cmd)?;
    let mut library = toollib::load(&path)?;
    if !library.add_material(&name) {
        return Err(bad(cmd, format!("the tool library has {} materials already", cam::MAX_MATERIALS)));
    }
    toollib::save(&path, &library)?;
    Ok(json!({ "material": name, "materials": library.materials }))
}

fn material_remove(_: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.material.remove";
    let name = str_(p, "name").ok_or_else(|| bad(cmd, "`name` is required"))?;
    let path = library_path(p, cmd)?;
    let mut library = toollib::load(&path)?;
    if !library.materials.iter().any(|m| cam::same_material(m, name)) {
        return Err(bad(cmd, format!("no material `{}` in the library", name.trim())));
    }
    library.materials.retain(|m| !cam::same_material(m, name));
    let mut tools = Vec::new();
    for tool in &mut library.tools {
        if tool.cutting_in(name).is_some() {
            tool.cutting.retain(|c| !cam::same_material(&c.material, name));
            tools.push(tool.number);
        }
    }
    toollib::save(&path, &library)?;
    Ok(json!({ "removed": name.trim(), "tools": tools }))
}

fn tool_remove(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.remove";
    let number = p.get("number").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`number` is required"))?;
    let m = &s.doc.manufacture;
    if !m.tools.iter().any(|t| u64::from(t.number) == number) {
        return Err(bad(cmd, format!("no tool {number}")));
    }
    if let Some(op) = m.setups.iter().flat_map(|x| &x.operations).find(|o| u64::from(o.tool) == number) {
        return Err(bad(cmd, format!("tool {number} is used by operation `{}`", op.name)));
    }
    let rest_of = |o: &&CamOperation| matches!(o.kind, CamOperationKind::Pocket { rest_from: Some(n), .. } | CamOperationKind::Adaptive { rest_from: Some(n), .. } if u64::from(n) == number);
    if let Some(op) = m.setups.iter().flat_map(|x| &x.operations).find(rest_of) {
        return Err(bad(cmd, format!("operation `{}` machines the rest of tool {number}", op.name)));
    }
    s.doc_mut().manufacture.tools.retain(|t| u64::from(t.number) != number);
    Ok(json!({ "removed": number }))
}

fn setup_create(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.create";
    let setups = &s.doc.manufacture.setups;
    if setups.len() >= MAX_SETUPS {
        return Err(bad(cmd, "too many setups"));
    }
    let name = match name_param(p, cmd)? {
        Some(n) if setups.iter().any(|x| x.name.eq_ignore_ascii_case(&n)) => return Err(bad(cmd, format!("there is already a setup `{n}`"))),
        Some(n) => n,
        None => (setups.len() + 1..).map(|k| format!("Setup{k}")).find(|n| !setups.iter().any(|x| x.name == *n)).unwrap_or_default(),
    };
    let origin = match p.get("origin") {
        Some(v) => origin_param(v, cmd)?,
        None => SetupOrigin::Point { at: Vec3::ZERO },
    };
    let axis = match p.get("axis") {
        Some(v) => axis_param(v, cmd)?,
        None => Vec3::new(0.0, 0.0, 1.0),
    };
    let stock = match p.get("stock") {
        Some(v) => stock_param(s, v, &StockDef::default(), cmd)?,
        None => StockDef::default(),
    };
    let post = str_(p, "post").unwrap_or("rs274");
    let post = Post::builtin(post).ok_or_else(|| bad(cmd, format!("unknown post `{post}` (known: {})", Post::BUILTIN.join(", "))))?.name;
    let material = material_param(p, cmd)?.flatten();
    let mut setup = Setup { id: 0, name: name.clone(), axis, origin, stock, material, post, operations: Vec::new() };
    resolve(s, &setup).map_err(|e| bad(cmd, e))?;
    setup.id = next_id(s);
    let id = setup.id;
    s.doc_mut().manufacture.setups.push(setup);
    Ok(json!({ "setup": id, "name": name }))
}

/// Changes what is given of a setup: name, post, tool axis, origin, stock, material.
fn setup_edit(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.edit";
    let i = setup_index(s, p, cmd)?;
    let mut setup = s.doc.manufacture.setups.get(i).cloned().ok_or_else(|| bad(cmd, "no such setup"))?;
    if let Some(n) = name_param(p, cmd)? {
        if s.doc.manufacture.setups.iter().enumerate().any(|(j, x)| j != i && x.name.eq_ignore_ascii_case(&n)) {
            return Err(bad(cmd, format!("there is already a setup `{n}`")));
        }
        setup.name = n;
    }
    if let Some(post) = p.get("post") {
        let post = post.as_str().unwrap_or_default();
        setup.post = Post::builtin(post).ok_or_else(|| bad(cmd, format!("unknown post `{post}` (known: {})", Post::BUILTIN.join(", "))))?.name;
    }
    if let Some(v) = p.get("axis") {
        setup.axis = axis_param(v, cmd)?;
    }
    if let Some(v) = p.get("origin") {
        setup.origin = origin_param(v, cmd)?;
    }
    if let Some(v) = p.get("stock") {
        setup.stock = stock_param(s, v, &setup.stock, cmd)?;
    }
    if let Some(material) = material_param(p, cmd)? {
        setup.material = material;
    }
    resolve(s, &setup).map_err(|e| bad(cmd, e))?;
    let (id, name) = (setup.id, setup.name.clone());
    if let Some(slot) = s.doc_mut().manufacture.setups.get_mut(i) {
        *slot = setup;
    }
    Ok(json!({ "setup": id, "name": name }))
}

/// The tool axis from `"+z"`, `"-x"`… or a direction `[x, y, z]`.
fn axis_param(v: &Value, cmd: &str) -> Result<Vec3> {
    let named = v.as_str().map(|a| a.trim().to_ascii_lowercase()).and_then(|a| {
        let (sign, letter) = match a.strip_prefix('-') {
            Some(rest) => (-1.0, rest.to_string()),
            None => (1.0, a.trim_start_matches('+').to_string()),
        };
        match letter.as_str() {
            "x" => Some(Vec3::new(sign, 0.0, 0.0)),
            "y" => Some(Vec3::new(0.0, sign, 0.0)),
            "z" => Some(Vec3::new(0.0, 0.0, sign)),
            _ => None,
        }
    });
    named.or_else(|| vec3(v).and_then(|d| d.normalized())).ok_or_else(|| bad(cmd, "`axis` must be +x, -x, +y, -y, +z, -z or a direction [x, y, z]"))
}

/// The work origin from a world point `[x, y, z]` or `{of?: stock|part, x?, y?, z?: min|center|max}`.
fn origin_param(v: &Value, cmd: &str) -> Result<SetupOrigin> {
    if let Some(at) = vec3(v).filter(|o| o.x.abs() <= 1e6 && o.y.abs() <= 1e6 && o.z.abs() <= 1e6) {
        return Ok(SetupOrigin::Point { at });
    }
    let wrong =
        || bad(cmd, "`origin` must be [x, y, z] or {of?: stock|part, x?, y?, z?: min|center|max} (default: the stock's top, at its lowest X and Y)");
    let o = v.as_object().ok_or_else(wrong)?;
    if o.keys().any(|k| !matches!(k.as_str(), "of" | "x" | "y" | "z")) {
        return Err(wrong());
    }
    let of = match o.get("of").map(|x| x.as_str().unwrap_or_default().to_ascii_lowercase()).as_deref() {
        None | Some("stock") => BoxOf::Stock,
        Some("part") => BoxOf::Part,
        _ => return Err(wrong()),
    };
    let anchor = |k: &str, default: Anchor| match o.get(k).map(|x| x.as_str().unwrap_or_default().to_ascii_lowercase()).as_deref() {
        None => Ok(default),
        Some("min") => Ok(Anchor::Min),
        Some("center" | "centre" | "mid" | "middle") => Ok(Anchor::Center),
        Some("max") => Ok(Anchor::Max),
        _ => Err(wrong()),
    };
    Ok(SetupOrigin::Box { of, x: anchor("x", Anchor::Min)?, y: anchor("y", Anchor::Min)?, z: anchor("z", Anchor::Max)? })
}

/// Stock offsets from `{side?, top?, bottom?}`; what is left out keeps its value in `was`.
fn stock_param(s: &Session, v: &Value, was: &StockDef, cmd: &str) -> Result<StockDef> {
    if !v.is_object() {
        return Err(bad(cmd, "`stock` must be {side?, top?, bottom?}: lengths added to the box around the bodies"));
    }
    let offset = |k: &str, was: &Expr| -> Result<Expr> {
        match v.get(k) {
            None => Ok(was.clone()),
            Some(_) => {
                let e = req_expr(cmd, v, k)?;
                let d = length(s, cmd, k, &e)?;
                if (0.0..=1.0e6).contains(&d) { Ok(e) } else { Err(bad(cmd, format!("the stock's `{k}` offset must be 0 or more"))) }
            }
        }
    };
    Ok(StockDef { side: offset("side", &was.side)?, top: offset("top", &was.top)?, bottom: offset("bottom", &was.bottom)? })
}

/// A setup's work coordinates: origin and axes in the world.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frame {
    o: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

impl Frame {
    /// A world point in work coordinates.
    fn local(&self, p: Vec3) -> Vec3 {
        let d = p - self.o;
        Vec3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }
    fn world(&self, q: Vec3) -> Vec3 {
        self.o + self.x * q.x + self.y * q.y + self.z * q.z
    }
    fn json(&self) -> Value {
        json!({"origin": self.o, "x": self.x, "y": self.y, "z": self.z})
    }
}

/// Work X and Y for a tool axis. Along a world axis they are world axes too: for +Z, world X
/// and Y; for −Z, X and −Y; for ±X, ±Y and Z; for +Y, X and −Z; for −Y, X and Z. For any other
/// direction X is level (square to world Z).
fn axes(axis: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
    let z = axis.normalized().filter(|z| z.is_finite())?;
    let x = if z.z.abs() > 1.0 - 1e-9 || z.y.abs() > 1.0 - 1e-9 {
        Vec3::new(1.0, 0.0, 0.0)
    } else if z.x.abs() > 1.0 - 1e-9 {
        Vec3::new(0.0, z.x.signum(), 0.0)
    } else {
        Vec3::new(0.0, 0.0, 1.0).cross(z).normalized()?
    };
    Some((x, z.cross(x), z))
}

/// A box in work coordinates.
type Box3 = (Vec3, Vec3);

/// A setup worked out against the model: its frame, and the boxes round the bodies and the stock.
pub(crate) struct Resolved {
    frame: Frame,
    part: Option<Box3>,
    stock: Option<Box3>,
}

fn resolve(s: &Session, setup: &Setup) -> std::result::Result<Resolved, String> {
    let (x, y, z) = axes(setup.axis).ok_or("the setup's tool axis is not a direction")?;
    // The bodies' extent along the setup's axes, measured from the world origin.
    let state = s.model.state();
    let mut part: Option<Box3> = None;
    for body in &state.bodies {
        for p in &body.mesh().positions {
            let q = Vec3::new(p.dot(x), p.dot(y), p.dot(z));
            part = Some(part.map_or((q, q), |(lo, hi)| (lo.min(q), hi.max(q))));
        }
    }
    let offset = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| format!("the stock's {what} offset: {e}")).map(|d| d.max(0.0));
    let (side, top, bottom) = (offset("side", &setup.stock.side)?, offset("top", &setup.stock.top)?, offset("bottom", &setup.stock.bottom)?);
    let stock = part.map(|(lo, hi)| (lo - Vec3::new(side, side, bottom), hi + Vec3::new(side, side, top)));
    let o = match &setup.origin {
        SetupOrigin::Point { at } => *at,
        SetupOrigin::Box { of, x: ax, y: ay, z: az } => {
            let (lo, hi) = match of {
                BoxOf::Stock => stock,
                BoxOf::Part => part,
            }
            .ok_or("the setup's origin is on the box round the bodies, and there is no body")?;
            let at = |a: &Anchor, lo: f64, hi: f64| match a {
                Anchor::Min => lo,
                Anchor::Center => (lo + hi) * 0.5,
                Anchor::Max => hi,
            };
            x * at(ax, lo.x, hi.x) + y * at(ay, lo.y, hi.y) + z * at(az, lo.z, hi.z)
        }
    };
    let shift = Vec3::new(o.dot(x), o.dot(y), o.dot(z));
    let moved = |b: Box3| (b.0 - shift, b.1 - shift);
    Ok(Resolved { frame: Frame { o, x, y, z }, part: part.map(moved), stock: stock.map(moved) })
}

fn setup_remove(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.remove";
    if key(p, "setup").is_none() {
        return Err(bad(cmd, "`setup` is required"));
    }
    let i = setup_index(s, p, cmd)?;
    let removed = s.doc_mut().manufacture.setups.remove(i);
    Ok(json!({ "removed": removed.id, "name": removed.name }))
}

/// Sketch-plane regions in a setup's XY. The sketch plane must face along the tool axis.
fn to_setup(plane: &Plane, frame: &Frame, regions: &[Region2]) -> Result<Vec<Region2>> {
    let facing = plane.normal().dot(frame.z);
    if !(facing.abs() > 1.0 - 1e-9) {
        return Err(EngineError::Other("not supported yet: operations on a sketch or face that is not square to the setup's tool axis".into()));
    }
    let mirrored = facing < 0.0;
    let x_axis = plane.dir_to_world(Vec2::new(1.0, 0.0));
    let turn = x_axis.dot(frame.y).atan2(x_axis.dot(frame.x));
    let point = |p: Vec2| {
        let w = frame.local(plane.to_world(p));
        Vec2::new(w.x, w.y)
    };
    let seg = |s: &Seg2| match *s {
        Seg2::Line { a, b } => Seg2::Line { a: point(a), b: point(b) },
        Seg2::Arc { center, radius, start, sweep } => Seg2::Arc {
            center: point(center),
            radius,
            start: if mirrored { turn - start } else { turn + start },
            sweep: if mirrored { -sweep } else { sweep },
        },
        Seg2::Cubic { p0, p1, p2, p3 } => Seg2::Cubic { p0: point(p0), p1: point(p1), p2: point(p2), p3: point(p3) },
        Seg2::Conic { a, apex, b, w } => Seg2::Conic { a: point(a), apex: point(apex), b: point(b), w },
        Seg2::Bezier { n, p } => Seg2::Bezier { n, p: p.map(point) },
    };
    let lp = |l: &Loop2| Loop2 { segs: l.segs.iter().map(seg).collect() };
    Ok(regions.iter().map(|r| Region2 { outer: lp(&r.outer), holes: r.holes.iter().map(lp).collect() }).collect())
}

/// Closed loops out of projected face edges: the largest is the outline, the others its holes.
fn outline(geom: &[LinkGeom]) -> std::result::Result<Region2, String> {
    let mut loops: Vec<Loop2> = Vec::new();
    let mut open: Vec<Seg2> = Vec::new();
    for g in geom {
        match *g {
            LinkGeom::Point(_) => {}
            LinkGeom::Line(a, b) => open.push(Seg2::Line { a, b }),
            LinkGeom::Circle(c, r) => loops.push(Loop2::circle(c, r)),
            LinkGeom::Arc { c, a, b } => {
                let (start, end) = ((a - c).angle(), (b - c).angle());
                let sweep = (end - start).rem_euclid(std::f64::consts::TAU);
                open.push(Seg2::Arc { center: c, radius: a.dist(c), start, sweep: if sweep > 1e-12 { sweep } else { std::f64::consts::TAU } });
            }
            LinkGeom::Conic { a, apex, b, rho } if rho > 1e-6 && rho < 1.0 - 1e-6 => open.push(Seg2::Conic { a, apex, b, w: rho / (1.0 - rho) }),
            _ => return Err("not supported yet: faces with elliptical or spline edges".into()),
        }
    }
    if open.len() > 20_000 {
        return Err("the face has too many edges".into());
    }
    // Edges come in no order and either direction: follow them end to end.
    const JOIN: f64 = 1e-3;
    let mut used = vec![false; open.len()];
    for first in 0..open.len() {
        if used.get(first).copied().unwrap_or(true) {
            continue;
        }
        let Some(seed) = open.get(first).copied() else { continue };
        if let Some(flag) = used.get_mut(first) {
            *flag = true;
        }
        let mut chain = vec![seed];
        let origin = seed.start();
        let mut end = seed.end();
        while end.dist(origin) > JOIN {
            let next = open.iter().zip(&used).enumerate().filter(|(_, (_, taken))| !**taken).find_map(|(i, (seg, _))| {
                if seg.start().dist(end) <= JOIN {
                    Some((i, *seg))
                } else if seg.end().dist(end) <= JOIN {
                    Some((i, seg.reversed()))
                } else {
                    None
                }
            });
            let Some((i, seg)) = next else { return Err("the face's edges do not form closed loops".into()) };
            if let Some(flag) = used.get_mut(i) {
                *flag = true;
            }
            end = seg.end();
            chain.push(seg);
        }
        loops.push(Loop2 { segs: chain });
    }
    let outer = loops
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.signed_area().abs().total_cmp(&b.1.signed_area().abs()))
        .map(|(i, _)| i)
        .ok_or("the face has no outline")?;
    let outer = loops.remove(outer);
    Ok(Region2 { outer, holes: loops })
}

/// The centre and radius of a loop that is one circle.
fn round_hole(lp: &Loop2) -> Option<(Vec2, f64)> {
    let Some(Seg2::Arc { center, radius, .. }) = lp.segs.first().copied() else { return None };
    let same = |s: &Seg2| matches!(*s, Seg2::Arc { center: c, radius: r, .. } if c.dist(center) <= 1e-6 && (r - radius).abs() <= 1e-6);
    let turned: f64 = lp.segs.iter().map(|s| if let Seg2::Arc { sweep, .. } = s { sweep.abs() } else { 0.0 }).sum();
    (lp.segs.iter().all(same) && (turned - std::f64::consts::TAU).abs() < 1e-6).then_some((center, radius))
}

/// The regions without those that only fill another's hole: a circle drawn inside a rectangle
/// is the rectangle's island, which pockets, contours and chamfers go round, not a second area.
fn without_filled_holes(regions: &[Region2]) -> Vec<Region2> {
    let fills = |r: &Region2, hole: &Loop2| {
        let (a, b) = (r.outer.signed_area().abs(), hole.signed_area().abs());
        (a - b).abs() <= 1e-6 * a.max(1.0) && hole.contains(r.interior_point())
    };
    regions
        .iter()
        .enumerate()
        .filter(|(i, r)| !regions.iter().enumerate().any(|(j, q)| j != *i && q.holes.iter().any(|h| fills(r, h))))
        .map(|(_, r)| r.clone())
        .collect()
}

/// Where an operation cuts, in its setup's work coordinates.
struct Geometry {
    regions: Vec<Region2>,
    /// Height of the sketch plane or the picked faces.
    z: f64,
    /// Lowest and highest point of the picked faces' bodies.
    body_z: Option<(f64, f64)>,
    /// A picked face looks away from the tool (the far side of the part).
    facing_away: bool,
    /// Lowest and highest point of the picked round walls (a hole's or a boss's), along the tool.
    walls: Option<(f64, f64)>,
    /// The picked walls of holes, as circles: what a drill drills.
    bores: Vec<(Vec2, f64)>,
    /// Picked faces: the highest the part reaches just outside their outlines, below the faces
    /// (the floor a boss stands on); `None` where nothing of the part is there.
    rim: Option<f64>,
}

fn geometry(s: &Session, at: &Resolved, source: &CamGeometry) -> std::result::Result<Geometry, String> {
    let state = s.model.state();
    let frame = &at.frame;
    match source {
        CamGeometry::Stock | CamGeometry::Part => {
            let (lo, hi) = if *source == CamGeometry::Part { at.part } else { at.stock }.ok_or("there is no body to take the boundary from")?;
            let outer = Loop2::polygon(&[Vec2::new(lo.x, lo.y), Vec2::new(hi.x, lo.y), Vec2::new(hi.x, hi.y), Vec2::new(lo.x, hi.y)]);
            Ok(Geometry {
                regions: vec![Region2 { outer, holes: Vec::new() }],
                // Either box is faced from the top of the stock.
                z: at.stock.map_or(hi.z, |(_, top)| top.z),
                body_z: at.part.map(|(lo, hi)| (lo.z, hi.z)),
                facing_away: false,
                walls: None,
                bores: Vec::new(),
                rim: None,
            })
        }
        CamGeometry::Sketch { sketch, profiles } => {
            let sketch =
                state.sketches.iter().find(|k| k.feature == *sketch).ok_or("its sketch is not built (deleted, rolled back, suppressed or failed)")?;
            let regions = solidcraft_doc::select_profiles(sketch, profiles).map_err(|e| e.to_string())?;
            let regions = to_setup(&sketch.plane, frame, &regions).map_err(|e| e.to_string())?;
            Ok(Geometry {
                regions,
                z: frame.local(sketch.plane.origin).z,
                body_z: None,
                facing_away: false,
                walls: None,
                bores: Vec::new(),
                rim: None,
            })
        }
        CamGeometry::Faces { faces } => {
            let mut regions = Vec::new();
            let mut z: Option<f64> = None;
            let mut body_z: Option<(f64, f64)> = None;
            let mut facing_away = false;
            let mut walls: Option<(f64, f64)> = None;
            let mut bores = Vec::new();
            for face in faces {
                let body = state.body(&face.body).ok_or_else(|| format!("body `{}` is gone", face.body))?;
                let (index, ..) =
                    solidcraft_doc::naming::face_on_body(body, &face.name, face.at).ok_or_else(|| format!("its face on `{}` is gone", face.body))?;
                let (lo, hi) =
                    body.mesh().positions.iter().map(|p| frame.local(*p).z).fold((f64::INFINITY, f64::NEG_INFINITY), |m, z| (m.0.min(z), m.1.max(z)));
                body_z = Some(body_z.map_or((lo, hi), |(a, b)| (a.min(lo), b.max(hi))));
                // A round wall along the tool (a hole's, a boss's) is cut as its circle, over its height.
                let Some(plane) = body.body.planar_face(index) else {
                    let wall = round_wall(body, index).ok_or("a picked face is neither flat nor a round wall")?;
                    if wall.axis.dot(frame.z).abs() < 1.0 - 1e-6 {
                        return Err("a picked round wall does not run along the tool; machine it in a setup whose axis does".into());
                    }
                    let ends = [wall.start, wall.end].map(|t| frame.local(wall.axis_point + wall.axis * t).z);
                    let (a, b) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
                    walls = Some(walls.map_or((a, b), |(lo, hi)| (lo.min(a), hi.max(b))));
                    let c = frame.local(wall.axis_point);
                    if wall.internal {
                        bores.push((Vec2::new(c.x, c.y), wall.radius));
                    }
                    regions.push(Region2 { outer: Loop2::circle(Vec2::new(c.x, c.y), wall.radius), holes: Vec::new() });
                    continue;
                };
                let resolved = solidcraft_doc::project::resolve_face_indices(&state, &plane, LinkKind::Project, &face.body, &[index])
                    .map_err(|e| e.to_string())?;
                let region = outline(&resolved.geom)?;
                regions.extend(to_setup(&plane, frame, &[region]).map_err(|e| e.to_string())?);
                facing_away |= plane.normal().dot(frame.z) < 0.0;
                let height = frame.local(plane.origin).z;
                if z.is_some_and(|other| (other - height).abs() > 1e-6) {
                    return Err("the picked faces are at different heights; use one operation per height".into());
                }
                z = Some(height);
            }
            // Round walls alone: the plane they are cut from is their top.
            let z = z.or(walls.map(|(_, hi)| hi)).ok_or("no face is picked")?;
            let rim = rim_floor(s, at, &regions, z, body_z.map(|(lo, _)| lo));
            Ok(Geometry { regions, z, body_z, facing_away, walls, bores, rim })
        }
    }
}

/// Face `index` of `body` as a cylinder, if it is one.
fn round_wall(body: &solidcraft_doc::ModelBody, index: usize) -> Option<solidcraft_kernel::CylinderFace> {
    solidcraft_kernel::cylinder_face(&body.body, &body.mesh(), index)
}

/// The bodies seen from above in a setup, over `lo`–`hi` in its work coordinates; `floor` where
/// there is nothing. `None` without bodies, or when the bodies are too detailed to look at.
fn part_field(s: &Session, at: &Resolved, lo: Vec2, hi: Vec2, floor: f64) -> Option<cam::HeightField> {
    part_field_with(s, at, lo, hi, floor, true)
}

/// [`part_field`]; `edges`: the triangles' edges mark the columns they cross too (see
/// [`cam::HeightField::of_triangles`]).
fn part_field_with(s: &Session, at: &Resolved, lo: Vec2, hi: Vec2, floor: f64, edges: bool) -> Option<cam::HeightField> {
    let triangles = part_triangles(s, at);
    if triangles.is_empty() || !lo.is_finite() || !hi.is_finite() {
        return None;
    }
    let size = (hi.x - lo.x).max(hi.y - lo.y);
    let cell = (size / 300.0).max(0.05);
    cam::HeightField::of_triangles(lo, hi, cell, floor, &triangles, edges).ok()
}

/// How far outside an outline the part is looked at for the floor next to it (mm).
const RIM: f64 = 1.5;

/// The highest the part reaches just outside the outer outlines of `regions` (within [`RIM`]),
/// below `top` and above the part's foot `lowest`: the floor a picked boss's top stands on.
/// `None` where nothing of the part is next to the outlines (the part's own outline).
fn rim_floor(s: &Session, at: &Resolved, regions: &[Region2], top: f64, lowest: Option<f64>) -> Option<f64> {
    let lowest = lowest?;
    let outlines: Vec<Vec<Vec2>> = regions.iter().map(|r| r.outer.polyline(TOLERANCE).into_vec()).filter(|p| p.len() > 2).collect();
    let all = outlines.iter().flatten();
    let lo = all.clone().fold(Vec2::new(f64::INFINITY, f64::INFINITY), |m, p| Vec2::new(m.x.min(p.x), m.y.min(p.y)));
    let hi = all.fold(Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY), |m, p| Vec2::new(m.x.max(p.x), m.y.max(p.y)));
    let around = Vec2::new(RIM * 2.0, RIM * 2.0);
    let field = part_field_with(s, at, lo - around, hi + around, lowest - 1.0, false)?;
    let inside = |p: Vec2| regions.iter().any(|r| r.outer.contains(p));
    let near = |p: Vec2| {
        outlines.iter().any(|line| {
            line.iter().zip(line.iter().cycle().skip(1)).take(line.len()).any(|(a, b)| {
                let ab = *b - *a;
                let t = if ab.dot(ab) > 1e-18 { ((p - *a).dot(ab) / ab.dot(ab)).clamp(0.0, 1.0) } else { 0.0 };
                p.dist(*a + ab * t) <= RIM
            })
        })
    };
    let mut floor: Option<f64> = None;
    for j in 0..field.ny {
        for i in 0..field.nx {
            let (Some(h), c) = (field.get(i, j), field.centre(i, j)) else { continue };
            if h > lowest - 0.5 && h < top - 1e-3 && !inside(c) && near(c) {
                floor = Some(floor.map_or(h, |f| f.max(h)));
            }
        }
    }
    floor
}

/// The part seen from above over a setup's part box, wide enough round it for any tool of the
/// design: what an operation's toolpath is checked against.
fn setup_part(s: &Session, at: &Resolved) -> Option<cam::HeightField> {
    let (lo, hi) = at.part?;
    let reach = s.doc.manufacture.tools.iter().map(|t| t.diameter).fold(1.0, f64::max);
    let around = Vec2::new(reach, reach);
    // Each column as its centre sees the part: a tool along a wall is not in it.
    part_field_with(s, at, Vec2::new(lo.x, lo.y) - around, Vec2::new(hi.x, hi.y) + around, lo.z - 1.0, false)
}

/// The bodies' triangles in a setup's work coordinates.
fn part_triangles(s: &Session, at: &Resolved) -> Vec<[Vec3; 3]> {
    let state = s.model.state();
    let mut triangles = Vec::new();
    for body in &state.bodies {
        let mesh = body.mesh();
        for t in &mesh.triangles {
            let corner = |k: usize| t.get(k).and_then(|v| mesh.positions.get(*v as usize)).map(|p| at.frame.local(*p));
            if let (Some(a), Some(b), Some(c)) = (corner(0), corner(1), corner(2)) {
                triangles.push([a, b, c]);
            }
        }
    }
    triangles
}

/// What an operation cuts, as far as the heights it finds by itself go.
#[derive(Clone, Copy)]
enum OpShape {
    Face,
    Contour,
    Chamfer,
    Pocket,
    Drill,
}

impl OpShape {
    fn of(kind: &CamOperationKind) -> Self {
        match kind {
            CamOperationKind::Face { .. } => OpShape::Face,
            CamOperationKind::Contour { .. } => OpShape::Contour,
            CamOperationKind::Chamfer { .. } => OpShape::Chamfer,
            CamOperationKind::Pocket { .. } | CamOperationKind::Adaptive { .. } => OpShape::Pocket,
            // Holes, as drilled: their heights are the same.
            CamOperationKind::Drill { .. } | CamOperationKind::Bore { .. } => OpShape::Drill,
        }
    }
}

/// The top and bottom an operation takes from its geometry (see [`build`]); `None` where it has none.
fn auto_heights(at: &Resolved, g: &Geometry, shape: OpShape, source: &CamGeometry) -> (Option<f64>, Option<f64>) {
    let (top, bottom) = auto_heights_of_part(at, g, shape, source);
    // A chamfer is on the picked face's own edge, wherever the stock is.
    if matches!(shape, OpShape::Chamfer) {
        return (top, bottom);
    }
    // Picked faces and walls give the part's heights; while the stock still stands above them,
    // cutting starts at its top (not down in the material, at a rapid).
    let stock_top = at.stock.map(|(_, hi)| hi.z);
    match (source, top, stock_top) {
        (CamGeometry::Faces { .. }, Some(t), Some(s)) => (Some(t.max(s)), bottom),
        _ => (top, bottom),
    }
}

fn auto_heights_of_part(at: &Resolved, g: &Geometry, shape: OpShape, source: &CamGeometry) -> (Option<f64>, Option<f64>) {
    // A chamfer is on the top edge of what it is given: a picked face's (a picked hole wall's
    // top), or where a contour of the sketch starts. Its depth comes from its width.
    if matches!(shape, OpShape::Chamfer) {
        let top = match source {
            CamGeometry::Faces { .. } => Some(g.walls.map_or(g.z, |(_, hi)| hi)),
            _ => auto_heights_of_part(at, g, OpShape::Contour, source).0,
        };
        return (top, top);
    }
    let picked = matches!(source, CamGeometry::Faces { .. });
    let on_sketch = matches!(source, CamGeometry::Sketch { .. });
    let stock_top = at.stock.map(|(_, hi)| hi.z);
    // Picked round walls: from their top down to their foot (a flat face picked with them can lower it).
    if let Some((lo, hi)) = g.walls.filter(|_| !matches!(shape, OpShape::Face)) {
        return (Some(hi.max(g.z)), Some(lo.min(g.z)));
    }
    match (shape, g.body_z) {
        (OpShape::Pocket, Some((_, hi))) if picked => (Some(hi), Some(g.z)),
        // Round a picked face down to the floor beside it (a boss's), or through the part.
        (OpShape::Contour, Some((lo, hi))) if picked => (Some(hi), Some(g.rim.unwrap_or(lo))),
        (OpShape::Drill, Some((lo, _))) if picked => (Some(g.z), Some(lo)),
        (OpShape::Face, _) if picked => (Some(g.z), Some(g.z)),
        // Facing a sketch's box: down to the sketch's plane, from the top of the stock if that is above it.
        (OpShape::Face, _) if on_sketch => (Some(at.stock.map_or(g.z, |(_, hi)| hi.z.max(g.z))), Some(g.z)),
        // Facing the stock: from its top down to the top of the part.
        (OpShape::Face, Some((_, hi))) if matches!(source, CamGeometry::Stock | CamGeometry::Part) => (Some(g.z), Some(hi)),
        // A sketch below the part's top (an outline drawn on its floor): cut from the top of the
        // material, the stock's or else the part's, down to the sketch.
        (_, _) if on_sketch && at.part.is_some_and(|(_, hi)| g.z < hi.z - 1e-6) => {
            (stock_top.or(at.part.map(|(_, hi)| hi.z)).map(|t| t.max(g.z)), Some(g.z))
        }
        // A sketch on the part's top: a contour or holes go through the part; a pocket's depth is the user's.
        (OpShape::Contour | OpShape::Drill, _) if on_sketch => (Some(g.z), at.part.map(|(lo, _)| lo.z).filter(|lo| *lo < g.z - 1e-6)),
        _ => (Some(g.z), None),
    }
}

/// What `op` cuts with in its setup: the heights it gives or finds (see [`build`]), its feeds and stepdown.
fn op_cut(s: &Session, at: &Resolved, op: &CamOperation, g: &Geometry) -> Result<Cut> {
    let fail = |why: String| EngineError::Other(format!("operation `{}`: {why}", op.name));
    let len = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| fail(format!("{what}: {e}")));
    let stock_top = at.stock.map(|(_, hi)| hi.z);
    let (auto_top, auto_bottom) = auto_heights(at, g, OpShape::of(&op.kind), &op.geometry);
    let height = |what: &str, given: &Option<Expr>, auto: Option<f64>| match given {
        Some(e) => len(what, e),
        None => auto.ok_or_else(|| fail(format!("`{what}` is required"))),
    };
    let top = height("top", &op.top, auto_top)?;
    let shown = |z: f64| format!("{} mm", (z * 1e3).round() / 1e3);
    let bottom = match (&op.bottom, auto_bottom) {
        (None, None) => {
            return Err(fail(format!(
                "`bottom` is required: how deep it cuts, in the setup's Z (zero at its work origin; the top is at {})",
                shown(top)
            )));
        }
        _ => height("bottom", &op.bottom, auto_bottom)?,
    };
    if bottom > top + 1e-9 {
        return Err(fail(format!(
            "the bottom ({}) is above the top ({}); heights are in the setup's Z, zero at its work origin",
            shown(bottom),
            shown(top)
        )));
    }
    let heights = Heights {
        clearance: height("clearance", &op.clearance, Some(g.body_z.map_or(top, |(_, hi)| hi.max(top)).max(stock_top.unwrap_or(top)) + 10.0))?,
        retract: height("retract", &op.retract, Some(top + 2.0))?,
        top,
        bottom,
    };
    // A number of passes divides the depth evenly.
    let stepdown = match op.passes {
        Some(n) => ((top - bottom) / f64::from(n.max(1))).max(1e-6),
        None => len("stepdown", &op.stepdown)?,
    };
    Ok(Cut { heights, feeds: Feeds { cut: op.cut_feed, plunge: op.plunge_feed }, stepdown })
}

/// The toolpath of `op` in its setup's work coordinates, from the model as it is now.
///
/// Heights the operation leaves out follow its geometry. A sketch gives the top (its plane).
/// Picked faces give: for a pocket, the bottom (the face: the pocket's floor) and the top (the
/// top of its body); for a contour, top and bottom of the body; for facing, a single pass at
/// the face, and likewise down to a sketch's plane (from the stock's top when that is higher);
/// for drilling, the top (the face) and the bottom (the bottom of its body); a picked round wall,
/// its top and its foot. Where the stock stands higher than a picked face's top, cutting starts
/// at the stock's top. Retract
/// and clearance are 2 mm and 10 mm above the top (clearance above the whole body and the
/// stock). Facing without a sketch or faces covers the stock, from its top to the part's.
pub(crate) fn build(s: &Session, setup: &Setup, op: &CamOperation) -> Result<Toolpath> {
    let at = resolve(s, setup).map_err(|e| EngineError::Other(format!("setup `{}`: {e}", setup.name)))?;
    build_in(s, &at, op)
}

/// [`build`] in a setup already worked out.
fn build_in(s: &Session, at: &Resolved, op: &CamOperation) -> Result<Toolpath> {
    let fail = |why: String| EngineError::Other(format!("operation `{}`: {why}", op.name));
    let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| fail(format!("there is no tool {} in the library", op.tool)))?;
    let len = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| fail(format!("{what}: {e}")));
    let angle = |e: &str| s.doc.eval(e, Kind::Angle).map_err(|e| fail(format!("ramp_angle: {e}")));
    // A V-bit cuts wider the deeper it goes: it engraves on a line or spots a hole; pockets,
    // facing and contours beside a line assume a tool as wide all the way down.
    if tool.kind == ToolKind::VBit {
        match &op.kind {
            CamOperationKind::Contour { side: Side::On, .. } | CamOperationKind::Drill { .. } | CamOperationKind::Chamfer { .. } => {}
            CamOperationKind::Contour { .. } => {
                return Err(fail("a V-bit engraves on the line: set the side to `on`".into()));
            }
            _ => return Err(fail("a V-bit engraves along a line (2D Contour, side `on`), chamfers edges (Chamfer) or spots holes (Drill)".into())),
        }
    }
    let g = geometry(s, at, &op.geometry).map_err(fail)?;
    let picked = matches!(op.geometry, CamGeometry::Faces { .. });
    // Only a contour can use the outline of a face on the far side.
    if g.facing_away && !matches!(op.kind, CamOperationKind::Contour { .. }) {
        return Err(fail("a picked face points away from the tool; machine it in a setup whose axis faces it".into()));
    }
    let cut = op_cut(s, at, op, &g)?;
    // Areas are cut round their islands; holes and drills want each circle as drawn.
    let kept: Vec<Region2>;
    let regions = match op.kind {
        CamOperationKind::Contour { .. } | CamOperationKind::Pocket { .. } | CamOperationKind::Adaptive { .. } | CamOperationKind::Chamfer { .. } => {
            kept = without_filled_holes(&g.regions);
            &kept
        }
        _ => &g.regions,
    };
    let path = match &op.kind {
        CamOperationKind::Face { stepover, inside, margin } => {
            let (mut min, mut max) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
            for p in regions.iter().flat_map(|r| r.outer.polyline(TOLERANCE)) {
                (min, max) = (Vec2::new(min.x.min(p.x), min.y.min(p.y)), Vec2::new(max.x.max(p.x), max.y.max(p.y)));
            }
            // Left out: over the boundary by the tool's radius (its nose clear of the ends), inside it by nothing.
            let margin = match margin {
                Some(e) => len("margin", e)?,
                None if *inside => 0.0,
                None => tool.radius(),
            };
            let params = cam::FaceParams { stepover: len("stepover", stepover)?, inside: *inside, margin };
            // The part seen from above round the area, so that passes below its top go round it.
            let reach = margin + tool.diameter;
            let around = Vec2::new(reach, reach);
            match part_field(s, at, min - around, max + around, cut.heights.bottom - 1.0) {
                Some(part) => cam::face_around(min, max, tool, &params, &cut, &part),
                None => cam::face(min, max, tool, &params, &cut),
            }
        }
        CamOperationKind::Contour { side, climb, stock_to_leave, ramp_angle, tabs, finish } => {
            let tabs = match tabs {
                Some(t) => Some(cam::Tabs { count: t.count, width: len("tabs width", &t.width)?, height: len("tabs height", &t.height)? }),
                None => None,
            };
            let params = ContourParams {
                side: *side,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                tolerance: TOLERANCE,
                ramp_angle: angle(ramp_angle)?,
                tabs,
                finish: finish.as_ref().map(|e| len("finish", e)).transpose()?.unwrap_or(0.0),
            };
            // Round a picked face means round the part: its holes are other operations' work.
            let outer: Vec<Region2>;
            let regions = if picked && *side == Side::Outside {
                outer = regions.iter().map(|r| Region2 { outer: r.outer.clone(), holes: Vec::new() }).collect();
                &outer
            } else {
                regions
            };
            cam::contour(regions, tool, &params, &cut)
        }
        CamOperationKind::Pocket { stepover, climb, stock_to_leave, ramp_angle, finish, rest_from } => {
            // Rest machining after a larger tool: its diameter.
            let rest_diameter = match rest_from {
                Some(n) => {
                    Some(s.doc.manufacture.tool(*n).map(|t| t.diameter).ok_or_else(|| fail(format!("there is no tool {n} to machine the rest of")))?)
                }
                None => None,
            };
            let params = PocketParams {
                stepover: len("stepover", stepover)?,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                tolerance: TOLERANCE,
                ramp_angle: angle(ramp_angle)?,
                finish: finish.as_ref().map(|e| len("finish", e)).transpose()?.unwrap_or(0.0),
                rest_diameter,
            };
            cam::pocket(regions, tool, &params, &cut)
        }
        CamOperationKind::Adaptive { stepover_percent, climb, stock_to_leave, finish, helix_angle, helix_max, helix_min, rest_from, feed_up } => {
            let rest_diameter = match rest_from {
                Some(n) => {
                    Some(s.doc.manufacture.tool(*n).map(|t| t.diameter).ok_or_else(|| fail(format!("there is no tool {n} to machine the rest of")))?)
                }
                None => None,
            };
            let params = AdaptiveParams {
                stepover: stepover_percent / 100.0,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                finish: finish.as_ref().map(|e| len("finish", e)).transpose()?.unwrap_or(0.0),
                tolerance: TOLERANCE,
                helix_angle: angle(helix_angle)?,
                helix_max: helix_max / 100.0,
                helix_min: helix_min / 100.0,
                feed_up: feed_up.unwrap_or(100.0) / 100.0,
                rest_diameter,
            };
            cam::adaptive(regions, tool, &params, &cut)
        }
        CamOperationKind::Chamfer { side, climb, width, tip_offset, ramp_angle } => {
            let params = ChamferParams {
                side: *side,
                climb: *climb,
                width: len("width", width)?,
                tip_offset: len("tip_offset", tip_offset)?,
                tolerance: TOLERANCE,
                ramp_angle: angle(ramp_angle)?,
            };
            // Round a picked face means round the part's outline, as a contour.
            let outer: Vec<Region2>;
            let regions = if picked && *side == Side::Outside {
                outer = regions.iter().map(|r| Region2 { outer: r.outer.clone(), holes: Vec::new() }).collect();
                &outer
            } else {
                regions
            };
            cam::chamfer(regions, tool, &params, &cut)
        }
        CamOperationKind::Bore { climb, stock_to_leave, pitch } => {
            let params = BoreParams { pitch: len("pitch", pitch)?, climb: *climb, stock_to_leave: len("stock_to_leave", stock_to_leave)? };
            // Round profiles of a sketch; a picked face's round holes and picked hole walls the tool fits in.
            let holes: Vec<(Vec2, f64)> = if picked {
                let walls = g.bores.iter().copied();
                regions.iter().flat_map(|r| r.holes.iter().filter_map(round_hole)).chain(walls).filter(|(_, r)| 2.0 * r > tool.diameter).collect()
            } else {
                let round: Vec<_> = regions.iter().filter_map(|r| round_hole(&r.outer)).collect();
                if round.len() < regions.len() {
                    return Err(fail("only round profiles can be bored; use a pocket for the others".into()));
                }
                round
            };
            if holes.is_empty() {
                return Err(fail(format!("the picked faces have no round holes wider than the {} mm tool", tool.diameter)));
            }
            let holes: Vec<(Vec2, f64)> = holes.into_iter().map(|(c, r)| (c, 2.0 * r)).collect();
            cam::bore(&holes, tool, &params, &cut)
        }
        CamOperationKind::Drill { peck, dwell } => {
            let peck = peck.as_ref().map(|e| len("peck", e)).transpose()?;
            // A sketch's profiles are the holes; a picked face's round inner loops are, those a
            // drill of this size can make (not the round opening of a pocket in the face).
            let fits = |r: f64| 2.0 * r <= tool.diameter * 1.1 + 1e-9;
            // A picked hole's wall is such a hole too.
            let points: Vec<Vec2> = if picked {
                let walls = g.bores.iter().copied();
                regions.iter().flat_map(|r| r.holes.iter().filter_map(round_hole)).chain(walls).filter(|(_, r)| fits(*r)).map(|(c, _)| c).collect()
            } else {
                regions.iter().map(|r| r.outer.centroid()).collect()
            };
            if points.is_empty() {
                return Err(fail(format!("the picked faces have no round holes a {} mm drill can make", tool.diameter)));
            }
            cam::drill(&points, tool, &DrillParams { peck, dwell: *dwell }, &cut)
        }
    };
    path.map_err(|e| fail(e.to_string()))
}

/// The faces an operation is to cut, from `faces: [{body?, point}]`: each the face holding the point.
fn faces_param(s: &Session, v: &Value, cmd: &str) -> Result<Vec<FaceRef>> {
    let items = v
        .as_array()
        .filter(|a| !a.is_empty() && a.len() <= 256)
        .ok_or_else(|| bad(cmd, "`faces` must be a list of 1 to 256 {body?, point: [x, y, z]}"))?;
    let state = s.model.state();
    let mut out = Vec::new();
    for item in items {
        let at = item.get("point").and_then(vec3).ok_or_else(|| bad(cmd, "each face needs `point`: [x, y, z], a point on it"))?;
        let wanted = item.get("body").and_then(Value::as_str);
        let found = state
            .bodies
            .iter()
            .filter(|b| wanted.is_none_or(|w| b.name.eq_ignore_ascii_case(w)))
            .find_map(|b| solidcraft_doc::naming::face_name_at_body(b, at).map(|name| FaceRef { body: b.name.clone(), name, at }));
        out.push(found.ok_or_else(|| bad(cmd, format!("no face at [{}, {}, {}]", at.x, at.y, at.z)))?);
    }
    Ok(out)
}

fn stats_json(path: &Toolpath) -> Value {
    let st = path.stats();
    json!({"cut_length": st.cut_length, "rapid_length": st.rapid_length, "cut_minutes": st.cut_minutes, "lowest_z": st.lowest_z, "moves": path.moves.len()})
}

/// Where an operation cuts, as `cmd` names it: a sketch, picked faces, or (facing) a boundary.
fn geometry_param(s: &Session, p: &Value, cmd: &str) -> Result<CamGeometry> {
    Ok(match (p.get("faces"), p.get("sketch")) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give either `sketch` or `faces`, not both")),
        (Some(faces), None) => CamGeometry::Faces { faces: faces_param(s, faces, cmd)? },
        (None, None) if cmd == "cam.op.face" => boundary_param(p, cmd)?,
        _ => CamGeometry::Sketch {
            sketch: super::features::sketch_id(s, p.get("sketch"), cmd, "sketch")?,
            profiles: super::features::profiles(p, cmd)?,
        },
    })
}

/// The heights an operation of `cmd` (a `cam.op.*` command) finds by itself: its top and bottom
/// in the setup's Z, `None` where it has to be given. `p` names the setup and the geometry as
/// `cmd` takes them. Forms show these heights and tell from them what is still missing.
pub fn cam_heights(s: &Session, cmd: &str, p: &Value) -> Result<(Option<f64>, Option<f64>)> {
    let shape = match cmd {
        "cam.op.face" => OpShape::Face,
        "cam.op.contour" => OpShape::Contour,
        "cam.op.chamfer" => OpShape::Chamfer,
        "cam.op.pocket" | "cam.op.adaptive" => OpShape::Pocket,
        "cam.op.drill" | "cam.op.bore" => OpShape::Drill,
        _ => return Err(bad(cmd, "not an operation")),
    };
    let setup = setup(s, p, cmd)?;
    let at = resolve(s, setup).map_err(|e| bad(cmd, format!("setup `{}`: {e}", setup.name)))?;
    let source = geometry_param(s, p, cmd)?;
    let g = geometry(s, &at, &source).map_err(|e| bad(cmd, e))?;
    Ok(auto_heights(&at, &g, shape, &source))
}

/// What an operation command would cut, with nothing in the design changed: `cmd` (a
/// `cam.op.*` command, or `cam.op.edit`) runs with `p` on a scratch copy of the session, and the
/// toolpath it makes comes back as points in world coordinates, each with whether the tool cuts
/// on the way to it. Forms show it before OK.
pub fn cam_preview(s: &Session, cmd: &str, p: &Value) -> Result<CamPreview> {
    if !cmd.starts_with("cam.op.") || matches!(cmd, "cam.op.move" | "cam.op.remove") {
        return Err(bad(cmd, "not an operation to preview"));
    }
    let mut copy = s.scratch();
    let made = copy.execute(cmd, p)?;
    let ask = json!({ "setup": made["setup"], "op": made["operation"], "heights": true });
    let path = copy.execute("cam.toolpath", &ask)?;
    let op = path["operations"].get(0).cloned().unwrap_or_default();
    let points = op["points"].as_array().cloned().unwrap_or_default();
    let h = &op["heights"];
    let heights = match (h["clearance"].as_f64(), h["retract"].as_f64(), h["top"].as_f64(), h["bottom"].as_f64()) {
        (Some(clearance), Some(retract), Some(top), Some(bottom)) => {
            let levels = h["levels"].as_array().into_iter().flatten().filter_map(Value::as_f64).collect();
            Some(PreviewHeights { clearance, retract, top, bottom, levels })
        }
        _ => None,
    };
    let points = points
        .iter()
        .filter_map(|q| Some((Vec3::new(q.get(0)?.as_f64()?, q.get(1)?.as_f64()?, q.get(2)?.as_f64()?), q.get(3)?.as_bool()?)))
        .collect();
    // How deep it would cut into the part, as the listing would say once it is added.
    let listed = copy.execute("cam.list", &json!({}))?;
    let gouge = listed["setups"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|x| x["id"] == made["setup"])
        .flat_map(|x| x["operations"].as_array().cloned().unwrap_or_default())
        .find(|o| o["id"] == made["operation"])
        .and_then(|o| o["warning"].as_str().map(str::to_string));
    Ok(CamPreview { points, warning: gouge, heights })
}

/// The heights a previewed operation cuts at, in its setup's Z, and its depth passes.
#[derive(Clone, Debug, PartialEq)]
pub struct PreviewHeights {
    pub clearance: f64,
    pub retract: f64,
    pub top: f64,
    pub bottom: f64,
    pub levels: Vec<f64>,
}

/// What [`cam_preview`] finds: the toolpath's points in world coordinates, each with whether the
/// tool cuts on the way to it, and a warning when it would cut into the part.
#[derive(Clone, Debug, PartialEq)]
pub struct CamPreview {
    pub points: Vec<(Vec3, bool)>,
    pub warning: Option<String>,
    pub heights: Option<PreviewHeights>,
}

/// How high the selection reaches in a setup: the lowest and highest point of the selected
/// vertices, edges, faces and bodies, in the setup's Z (`p` names the setup; default: the only
/// one). `None` when nothing selected is part of a body. Forms take heights from it.
pub fn cam_selection_heights(s: &Session, p: &Value, selection: &[crate::Sel]) -> Result<Option<(f64, f64)>> {
    let cmd = "cam.op";
    let setup = setup(s, p, cmd)?;
    let at = resolve(s, setup).map_err(|e| bad(cmd, format!("setup `{}`: {e}", setup.name)))?;
    let state = s.model.state();
    let mut points: Vec<Vec3> = Vec::new();
    for item in selection.iter().take(MAX_SELECTED) {
        match item {
            crate::Sel::Vertex { point, .. } => points.push(*point),
            crate::Sel::Edge { body, index, point } => match state.body(body).map(|b| b.mesh()) {
                Some(mesh) if mesh.edges.get(*index).is_some_and(|e| !e.is_empty()) => points.extend(mesh.edges.get(*index).into_iter().flatten()),
                _ => points.push(*point),
            },
            crate::Sel::Face { body, index, point } => {
                let before = points.len();
                if let Some(mesh) = state.body(body).map(|b| b.mesh()) {
                    let on_face = mesh.triangles.iter().zip(&mesh.tri_face).filter(|(_, f)| usize::try_from(**f).ok() == Some(*index));
                    points.extend(on_face.flat_map(|(t, _)| t.iter().filter_map(|v| mesh.positions.get(*v as usize).copied())));
                }
                if points.len() == before {
                    points.push(*point);
                }
            }
            crate::Sel::Body { name } => {
                if let Some(mesh) = state.body(name).map(|b| b.mesh()) {
                    points.extend(mesh.positions.iter().copied());
                }
            }
            _ => {}
        }
    }
    Ok(points
        .iter()
        .map(|q| at.frame.local(*q).z)
        .filter(|z| z.is_finite())
        .fold(None, |r: Option<(f64, f64)>, z| Some(r.map_or((z, z), |(lo, hi)| (lo.min(z), hi.max(z))))))
}

/// Reads what every operation takes and adds the operation once its toolpath has been worked out.
fn add_op(s: &mut Session, p: &Value, cmd: &str, label: &str, kind: impl FnOnce(&Tool) -> CamOperationKind) -> Result<Value> {
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?.clone();
    if setup.operations.len() >= MAX_OPERATIONS {
        return Err(bad(cmd, "the setup has too many operations"));
    }
    let geometry = geometry_param(s, p, cmd)?;
    let number = p.get("tool").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`tool` must be a tool number"))?;
    // The design's own copy of the tool, else the machine library's, which the design then keeps a copy of.
    let own = s.doc.manufacture.tools.iter().find(|t| u64::from(t.number) == number).cloned();
    let from_library = own.is_none();
    let stored =
        own.or_else(|| library_tool(p, number)).ok_or_else(|| bad(cmd, format!("no tool {number} (define it in the tool library first)")))?;
    // Its cutting data in the stock's material, where it has some.
    let tool = stored.in_material(setup.material.as_deref());
    let positive = |k: &str| num(p, k).filter(|v| *v > 0.0 && *v <= 1.0e6);
    let none = match &setup.material {
        Some(m) => format!("none for {m} and none of its own"),
        None => "none of its own".to_string(),
    };
    // Speed and feeds left out are the tool's.
    let given = |k: &str, what: &str, tools: Option<f64>| match p.get(k) {
        Some(_) => positive(k).ok_or_else(|| bad(cmd, format!("`{k}` must be above 0{what}"))),
        None => tools.ok_or_else(|| bad(cmd, format!("`{k}` is required: tool {} has {none}", tool.number))),
    };
    let spindle_rpm = given("spindle_rpm", "", tool.spindle_rpm)?;
    let cut_feed = given("feed", " (mm/min)", tool.feed)?;
    let plunge_feed = given("plunge_feed", " (mm/min)", Some(tool.plunge_feed.unwrap_or(cut_feed / 3.0)))?;
    // A height left out follows the geometry (see `build`).
    let heights = ["clearance", "retract", "top", "bottom"].map(|k| match p.get(k) {
        Some(_) => req_expr(cmd, p, k).and_then(|e| length(s, cmd, k, &e).map(|_| Some(e))),
        None => Ok(None),
    });
    let [clearance, retract, top, bottom] = heights;
    let (clearance, retract, top, bottom) = (clearance?, retract?, top?, bottom?);
    let stepdown = expr(p, "stepdown").unwrap_or_else(|| mm(tool.stepdown.unwrap_or(tool.diameter * 0.5)));
    let passes = passes_param(p, cmd)?;
    let name = match name_param(p, cmd)? {
        Some(n) if setup.operations.iter().any(|o| o.name.eq_ignore_ascii_case(&n)) => {
            return Err(bad(cmd, format!("the setup already has an operation `{n}`")));
        }
        Some(n) => n,
        None => (1..).map(|k| format!("{label}{k}")).find(|n| !setup.operations.iter().any(|o| o.name == *n)).unwrap_or_default(),
    };
    let kind = kind(&tool);
    let mut op = CamOperation {
        id: 0,
        name,
        tool: tool.number,
        spindle_rpm,
        coolant: bool_(p, "coolant").unwrap_or(false),
        cut_feed,
        plunge_feed,
        clearance,
        retract,
        top,
        bottom,
        stepdown,
        passes,
        geometry,
        kind,
    };
    // An operation that cannot be cut is not added.
    if from_library {
        let tools = &mut s.doc_mut().manufacture.tools;
        tools.push(stored.clone());
        tools.sort_by_key(|t| t.number);
    }
    let path = match build(s, &setup, &op) {
        Ok(path) => path,
        Err(e) => {
            if from_library {
                s.doc_mut().manufacture.tools.retain(|t| t.number != tool.number);
            }
            return Err(bad(cmd, e.to_string()));
        }
    };
    op.id = next_id(s);
    let (id, name) = (op.id, op.name.clone());
    if let Some(target) = s.doc_mut().manufacture.setups.get_mut(si) {
        target.operations.push(op);
    }
    let mut out = stats_json(&path);
    out["operation"] = json!(id);
    out["name"] = json!(name);
    out["setup"] = json!(setup.id);
    Ok(out)
}

fn ramp_angle(p: &Value) -> Expr {
    expr(p, "ramp_angle").unwrap_or_else(|| "3 deg".into())
}

fn stock_to_leave(p: &Value) -> Expr {
    expr(p, "stock_to_leave").unwrap_or_else(|| mm(0.0))
}

/// `passes`: how many depth passes (null or left out: by the stepdown).
fn passes_param(p: &Value, cmd: &str) -> Result<Option<u32>> {
    match p.get("passes") {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v.as_u64().filter(|n| (1..=1000).contains(n)).map(|n| Some(n as u32)).ok_or_else(|| bad(cmd, "`passes` must be 1 to 1000")),
    }
}

/// A length expression checked now, so that a bad one names its input.
fn length_expr(s: &Session, cmd: &str, what: &str, e: Expr) -> Result<Expr> {
    s.doc.eval(&e, Kind::Length).map_err(|err| bad(cmd, format!("{what}: {err}")))?;
    Ok(e)
}

/// `tabs`: `{count?, width?, height?}` (defaults 4, 6 mm, 2 mm), or null for none.
fn tabs_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<TabsDef>> {
    let v = match p.get("tabs") {
        None | Some(Value::Null) => return Ok(None),
        Some(v) if v.is_object() => v,
        Some(_) => return Err(bad(cmd, "`tabs` must be {count?, width?, height?} or null")),
    };
    let count = match v.get("count") {
        None => 4,
        Some(c) => c
            .as_u64()
            .filter(|n| (1..=u64::from(cam::MAX_TABS)).contains(n))
            .ok_or_else(|| bad(cmd, format!("the tabs' `count` must be 1 to {}", cam::MAX_TABS)))? as u32,
    };
    let width = length_expr(s, cmd, "tabs width", expr(v, "width").unwrap_or_else(|| mm(6.0)))?;
    let height = length_expr(s, cmd, "tabs height", expr(v, "height").unwrap_or_else(|| mm(2.0)))?;
    Ok(Some(TabsDef { count, width, height }))
}

/// `rest_from`: the larger tool (a number of this design's) whose pocket this one machines the
/// rest of (null or left out: the whole pocket).
fn rest_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<u32>> {
    match p.get("rest_from") {
        None | Some(Value::Null) => Ok(None),
        Some(v) => {
            let n = v.as_u64().and_then(|n| u32::try_from(n).ok()).ok_or_else(|| bad(cmd, "`rest_from` must be a tool number"))?;
            s.doc.manufacture.tool(n).map(|_| Some(n)).ok_or_else(|| bad(cmd, format!("`rest_from`: the design has no tool {n}")))
        }
    }
}

/// `finish`: the finishing allowance (null or left out: none).
fn finish_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<Expr>> {
    match expr(p, "finish") {
        Some(e) => length_expr(s, cmd, "finish", e).map(Some),
        None => Ok(None),
    }
}

fn op_face(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.face";
    let stepover = expr(p, "stepover");
    let inside = limit_param(p, cmd)?;
    let margin = match p.get("margin") {
        Some(v) if !v.is_null() => Some(margin_param(s, p, cmd)?),
        _ => None,
    };
    add_op(s, p, cmd, "Face", |tool| CamOperationKind::Face {
        stepover: stepover.unwrap_or_else(|| mm(tool.stepover.unwrap_or(tool.diameter * 0.5))),
        inside,
        margin,
    })
}

/// Whether a facing operation's tool stays inside its boundary (`limit`: over|inside; default over).
fn limit_param(p: &Value, cmd: &str) -> Result<bool> {
    match (p.get("limit"), str_(p, "limit").map(|k| k.trim().to_ascii_lowercase()).as_deref()) {
        (None, _) | (_, Some("over")) => Ok(false),
        (_, Some("inside")) => Ok(true),
        _ => Err(bad(cmd, "`limit` must be over or inside")),
    }
}

/// How far inside or over its boundary a facing operation cuts: a length of 0 or more.
fn margin_param(s: &Session, p: &Value, cmd: &str) -> Result<Expr> {
    let e = req_expr(cmd, p, "margin")?;
    if length(s, cmd, "margin", &e)? >= 0.0 { Ok(e) } else { Err(bad(cmd, "`margin` must be 0 or more")) }
}

/// The box a facing operation covers when no sketch or faces are given (`boundary`: stock|part; default stock).
fn boundary_param(p: &Value, cmd: &str) -> Result<CamGeometry> {
    match (p.get("boundary"), str_(p, "boundary").map(|k| k.trim().to_ascii_lowercase()).as_deref()) {
        (None, _) | (_, Some("stock")) => Ok(CamGeometry::Stock),
        (_, Some("part" | "body" | "bodies")) => Ok(CamGeometry::Part),
        _ => Err(bad(cmd, "`boundary` must be stock or part (or give `sketch` or `faces`)")),
    }
}

fn side_param(p: &Value, cmd: &str) -> Result<Side> {
    match (p.get("side"), str_(p, "side").map(|k| k.trim().to_ascii_lowercase()).as_deref()) {
        (None, _) | (_, Some("outside")) => Ok(Side::Outside),
        (_, Some("inside")) => Ok(Side::Inside),
        (_, Some("on")) => Ok(Side::On),
        _ => Err(bad(cmd, "`side` must be outside, inside or on")),
    }
}

fn op_contour(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.contour";
    let side = side_param(p, cmd)?;
    let (tabs, finish) = (tabs_param(s, p, cmd)?, finish_param(s, p, cmd)?);
    let kind = CamOperationKind::Contour {
        side,
        climb: bool_(p, "climb").unwrap_or(true),
        stock_to_leave: stock_to_leave(p),
        ramp_angle: ramp_angle(p),
        tabs,
        finish,
    };
    add_op(s, p, cmd, "Contour", |_| kind)
}

fn op_pocket(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.pocket";
    let (stepover, climb, leave, ramp) = (expr(p, "stepover"), bool_(p, "climb").unwrap_or(true), stock_to_leave(p), ramp_angle(p));
    let finish = finish_param(s, p, cmd)?;
    let rest_from = rest_param(s, p, cmd)?;
    add_op(s, p, cmd, "Pocket", |tool| CamOperationKind::Pocket {
        stepover: stepover.unwrap_or_else(|| mm(tool.stepover.unwrap_or(tool.diameter * 0.4))),
        climb,
        stock_to_leave: leave,
        ramp_angle: ramp,
        finish,
        rest_from,
    })
}

fn op_drill(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.drill";
    let dwell = match p.get("dwell") {
        Some(_) => num(p, "dwell").filter(|d| (0.0..=3600.0).contains(d)).ok_or_else(|| bad(cmd, "`dwell` must be 0 to 3600 seconds"))?,
        None => 0.0,
    };
    let peck = expr(p, "peck");
    add_op(s, p, cmd, "Drill", |_| CamOperationKind::Drill { peck, dwell })
}

/// A number given as `k`, between `lo` and `hi`, else `default`.
fn number_in(p: &Value, k: &str, lo: f64, hi: f64, default: f64, cmd: &str) -> Result<f64> {
    match p.get(k) {
        None | Some(Value::Null) => Ok(default),
        Some(_) => num(p, k).filter(|v| (lo..=hi).contains(v)).ok_or_else(|| bad(cmd, format!("`{k}` must be {lo} to {hi}"))),
    }
}

fn op_adaptive(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.adaptive";
    let helix_angle = match expr(p, "helix_angle") {
        Some(e) => {
            s.doc.eval(&e, Kind::Angle).map_err(|err| bad(cmd, format!("helix_angle: {err}")))?;
            e
        }
        None => "5 deg".into(),
    };
    let (helix_max, helix_min) = (number_in(p, "helix_max", 1.0, 1000.0, 100.0, cmd)?, number_in(p, "helix_min", 1.0, 1000.0, 10.0, cmd)?);
    if helix_min > helix_max {
        return Err(bad(cmd, "`helix_min` must be no larger than `helix_max`"));
    }
    let kind = CamOperationKind::Adaptive {
        stepover_percent: number_in(p, "stepover_percent", 2.0, 50.0, 20.0, cmd)?,
        climb: bool_(p, "climb").unwrap_or(true),
        stock_to_leave: stock_to_leave(p),
        finish: finish_param(s, p, cmd)?,
        helix_angle,
        helix_max,
        helix_min,
        rest_from: rest_param(s, p, cmd)?,
        feed_up: feed_up_param(p, cmd)?,
    };
    add_op(s, p, cmd, "Adaptive", |_| kind)
}

/// How much faster light adaptive steps may go: percent of the feed, none when 100 or left out.
fn feed_up_param(p: &Value, cmd: &str) -> Result<Option<f64>> {
    let v = number_in(p, "feed_up", 100.0, 500.0, 100.0, cmd)?;
    Ok((v > 100.0).then_some(v))
}

fn op_chamfer(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.chamfer";
    let side = side_param(p, cmd)?;
    if side == Side::On {
        return Err(bad(cmd, "`side` must be outside or inside: a chamfer is beside an edge"));
    }
    let kind = CamOperationKind::Chamfer {
        side,
        climb: bool_(p, "climb").unwrap_or(true),
        width: expr(p, "width").unwrap_or_else(|| mm(1.0)),
        tip_offset: expr(p, "tip_offset").unwrap_or_else(|| mm(0.5)),
        ramp_angle: ramp_angle(p),
    };
    add_op(s, p, cmd, "Chamfer", |_| kind)
}

fn op_bore(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.bore";
    let (pitch, climb, leave) = (expr(p, "pitch"), bool_(p, "climb").unwrap_or(true), stock_to_leave(p));
    add_op(s, p, cmd, "Bore", |tool| CamOperationKind::Bore {
        climb,
        stock_to_leave: leave,
        pitch: pitch.unwrap_or_else(|| mm(tool.stepdown.unwrap_or(tool.diameter * 0.25))),
    })
}

fn op_position(setup: &Setup, k: &str) -> Option<usize> {
    setup.operations.iter().position(|o| o.id.to_string() == k).or_else(|| setup.operations.iter().position(|o| o.name.eq_ignore_ascii_case(k)))
}

/// Changes the inputs of an operation that are given; the change is kept only if the operation can still be cut.
fn op_edit(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.edit";
    let k = key(p, "op").ok_or_else(|| bad(cmd, "`op` is required"))?;
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?.clone();
    let oi = op_position(&setup, &k).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?;
    let mut op = setup.operations.get(oi).cloned().ok_or_else(|| bad(cmd, "no such operation"))?;
    let positive = |k: &str| num(p, k).filter(|v| *v > 0.0 && *v <= 1.0e6).ok_or_else(|| bad(cmd, format!("`{k}` must be above 0")));
    let has = |k: &str| p.get(k).is_some();
    // An expression checked now, so that a bad one names its input.
    let checked = |k: &str, kind: Kind| -> Result<Expr> {
        let e = req_expr(cmd, p, k)?;
        s.doc.eval(&e, kind).map_err(|err| bad(cmd, format!("{k}: {err}")))?;
        Ok(e)
    };
    // `null` puts an optional expression back to automatic.
    let optional =
        |k: &str| -> Result<Option<Expr>> { if p.get(k).is_some_and(Value::is_null) { Ok(None) } else { checked(k, Kind::Length).map(Some) } };
    if let Some(n) = name_param(p, cmd)? {
        if setup.operations.iter().enumerate().any(|(i, o)| i != oi && o.name.eq_ignore_ascii_case(&n)) {
            return Err(bad(cmd, format!("the setup already has an operation `{n}`")));
        }
        op.name = n;
    }
    if has("tool") {
        let number = p.get("tool").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`tool` must be a tool number"))?;
        op.tool = s
            .doc
            .manufacture
            .tools
            .iter()
            .find(|t| u64::from(t.number) == number)
            .map(|t| t.number)
            .ok_or_else(|| bad(cmd, format!("no tool {number}")))?;
    }
    if has("spindle_rpm") {
        op.spindle_rpm = positive("spindle_rpm")?;
    }
    if has("feed") {
        op.cut_feed = positive("feed")?;
    }
    if has("plunge_feed") {
        op.plunge_feed = positive("plunge_feed")?;
    }
    if has("coolant") {
        op.coolant = bool_(p, "coolant").ok_or_else(|| bad(cmd, "`coolant` must be true or false"))?;
    }
    if has("stepdown") {
        op.stepdown = checked("stepdown", Kind::Length)?;
    }
    if has("passes") {
        op.passes = passes_param(p, cmd)?;
    }
    for (k, slot) in [("clearance", &mut op.clearance), ("retract", &mut op.retract), ("top", &mut op.top), ("bottom", &mut op.bottom)] {
        if has(k) {
            *slot = optional(k)?;
        }
    }
    match (p.get("faces"), p.get("sketch")) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give either `sketch` or `faces`, not both")),
        (Some(faces), None) => op.geometry = CamGeometry::Faces { faces: faces_param(s, faces, cmd)? },
        (None, Some(sketch)) => {
            op.geometry = CamGeometry::Sketch {
                sketch: super::features::sketch_id(s, Some(sketch), cmd, "sketch")?,
                profiles: super::features::profiles(p, cmd)?,
            }
        }
        (None, None) => {
            if p.get("boundary").is_some() {
                if !matches!(op.kind, CamOperationKind::Face { .. }) {
                    return Err(bad(cmd, format!("a {} operation has no `boundary`", op.kind.label())));
                }
                op.geometry = boundary_param(p, cmd)?;
            }
        }
    }
    let unknown = |k: &str| bad(cmd, format!("a {} operation has no `{k}`", op.kind.label()));
    let mut kind = op.kind.clone();
    for k in [
        "stepover",
        "side",
        "climb",
        "stock_to_leave",
        "ramp_angle",
        "tabs",
        "finish",
        "rest_from",
        "stepover_percent",
        "helix_angle",
        "helix_max",
        "helix_min",
        "feed_up",
        "pitch",
        "width",
        "tip_offset",
        "peck",
        "dwell",
        "limit",
        "margin",
    ] {
        if !has(k) {
            continue;
        }
        match (&mut kind, k) {
            (CamOperationKind::Face { stepover, .. } | CamOperationKind::Pocket { stepover, .. }, "stepover") => {
                *stepover = checked(k, Kind::Length)?
            }
            (CamOperationKind::Face { inside, .. }, "limit") => *inside = limit_param(p, cmd)?,
            (CamOperationKind::Face { margin, .. }, "margin") => {
                *margin = if p.get(k).is_some_and(Value::is_null) { None } else { Some(margin_param(s, p, cmd)?) }
            }
            (CamOperationKind::Contour { side, .. }, "side") => *side = side_param(p, cmd)?,
            (CamOperationKind::Pocket { rest_from, .. } | CamOperationKind::Adaptive { rest_from, .. }, "rest_from") => {
                *rest_from = rest_param(s, p, cmd)?
            }
            (CamOperationKind::Adaptive { stepover_percent, .. }, "stepover_percent") => {
                *stepover_percent = number_in(p, k, 2.0, 50.0, *stepover_percent, cmd)?
            }
            (CamOperationKind::Adaptive { helix_angle, .. }, "helix_angle") => *helix_angle = checked(k, Kind::Angle)?,
            (CamOperationKind::Adaptive { helix_max, .. }, "helix_max") => *helix_max = number_in(p, k, 1.0, 1000.0, *helix_max, cmd)?,
            (CamOperationKind::Adaptive { helix_min, .. }, "helix_min") => *helix_min = number_in(p, k, 1.0, 1000.0, *helix_min, cmd)?,
            (CamOperationKind::Adaptive { feed_up, .. }, "feed_up") => *feed_up = feed_up_param(p, cmd)?,
            (CamOperationKind::Bore { pitch, .. }, "pitch") => *pitch = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { width, .. }, "width") => *width = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { tip_offset, .. }, "tip_offset") => *tip_offset = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { side, .. }, "side") => *side = side_param(p, cmd)?,
            (
                CamOperationKind::Contour { climb, .. }
                | CamOperationKind::Pocket { climb, .. }
                | CamOperationKind::Bore { climb, .. }
                | CamOperationKind::Chamfer { climb, .. }
                | CamOperationKind::Adaptive { climb, .. },
                "climb",
            ) => *climb = bool_(p, k).ok_or_else(|| bad(cmd, "`climb` must be true or false"))?,
            (
                CamOperationKind::Contour { stock_to_leave, .. }
                | CamOperationKind::Pocket { stock_to_leave, .. }
                | CamOperationKind::Bore { stock_to_leave, .. }
                | CamOperationKind::Adaptive { stock_to_leave, .. },
                "stock_to_leave",
            ) => *stock_to_leave = checked(k, Kind::Length)?,
            (
                CamOperationKind::Contour { ramp_angle, .. }
                | CamOperationKind::Pocket { ramp_angle, .. }
                | CamOperationKind::Chamfer { ramp_angle, .. },
                "ramp_angle",
            ) => *ramp_angle = checked(k, Kind::Angle)?,
            (CamOperationKind::Contour { tabs, .. }, "tabs") => *tabs = tabs_param(s, p, cmd)?,
            (
                CamOperationKind::Contour { finish, .. } | CamOperationKind::Pocket { finish, .. } | CamOperationKind::Adaptive { finish, .. },
                "finish",
            ) => *finish = finish_param(s, p, cmd)?,
            (CamOperationKind::Drill { peck, .. }, "peck") => *peck = optional(k)?,
            (CamOperationKind::Drill { dwell, .. }, "dwell") => {
                *dwell = num(p, k).filter(|d| (0.0..=3600.0).contains(d)).ok_or_else(|| bad(cmd, "`dwell` must be 0 to 3600 seconds"))?
            }
            _ => return Err(unknown(k)),
        }
    }
    op.kind = kind;
    let path = build(s, &setup, &op).map_err(|e| bad(cmd, e.to_string()))?;
    let (id, name) = (op.id, op.name.clone());
    if let Some(slot) = s.doc_mut().manufacture.setups.get_mut(si).and_then(|x| x.operations.get_mut(oi)) {
        *slot = op;
    }
    let mut out = stats_json(&path);
    out["operation"] = json!(id);
    out["name"] = json!(name);
    out["setup"] = json!(setup.id);
    Ok(out)
}

/// Operations are cut in their setup's order; this moves one within it.
fn op_move(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.move";
    let k = key(p, "op").ok_or_else(|| bad(cmd, "`op` is required"))?;
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?;
    let from = op_position(setup, &k).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?;
    let last = setup.operations.len().saturating_sub(1) as i64;
    let to = match (p.get("to").and_then(Value::as_i64), p.get("by").and_then(Value::as_i64)) {
        (Some(to), None) => to,
        (None, Some(by)) => (from as i64).saturating_add(by),
        _ => return Err(bad(cmd, "give `to` (a position) or `by` (a number of places), as whole numbers")),
    }
    .clamp(0, last) as usize;
    let Some(setup) = s.doc_mut().manufacture.setups.get_mut(si) else { return Err(bad(cmd, "no such setup")) };
    let op = setup.operations.remove(from);
    setup.operations.insert(to.min(setup.operations.len()), op);
    Ok(json!({ "order": setup.operations.iter().map(|o| o.name.clone()).collect::<Vec<_>>() }))
}

fn op_remove(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.remove";
    let k = key(p, "op").ok_or_else(|| bad(cmd, "`op` is required"))?;
    let si = setup_index(s, p, cmd)?;
    let oi = s.doc.manufacture.setups.get(si).and_then(|x| op_position(x, &k)).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?;
    let removed = s.doc_mut().manufacture.setups.get_mut(si).map(|x| x.operations.remove(oi));
    Ok(json!({ "removed": removed.map(|o| o.id) }))
}

fn list(s: &mut Session, _: &Value) -> Result<Value> {
    if let Ok(cache) = s.cam_list_cache.lock()
        && let Some((revision, doc, listed)) = cache.as_ref()
        && *revision == s.revision
        && std::sync::Arc::ptr_eq(doc, &s.doc)
    {
        return Ok(listed.clone());
    }
    let listed = list_now(s);
    if let Ok(mut cache) = s.cam_list_cache.lock() {
        *cache = Some((s.revision, s.doc.clone(), listed.clone()));
    }
    Ok(listed)
}

fn list_now(s: &Session) -> Value {
    let m = &s.doc.manufacture;
    let setups: Vec<Value> = m
        .setups
        .iter()
        .map(|setup| {
            // The part seen from above, to tell an operation that cuts into it.
            let part = resolve(s, setup).ok().and_then(|at| setup_part(s, &at));
            let ops: Vec<Value> = setup
                .operations
                .iter()
                .map(|op| {
                    let mut v = json!(op);
                    // The kind's own inputs (stepover, side…) beside its name.
                    v["inputs"] = v["kind"].take();
                    v["kind"] = json!(op.kind.label());
                    match build(s, setup, op) {
                        Ok(path) => {
                            v["toolpath"] = stats_json(&path);
                            let depth = match (&part, m.tool(op.tool)) {
                                (Some(part), Some(tool)) => cam::gouge(&path, tool, part),
                                _ => 0.0,
                            };
                            // A chamfer cuts the part's edge on purpose: the model has no chamfer to compare with.
                            if depth > GOUGE_REPORTED && !matches!(op.kind, CamOperationKind::Chamfer { .. }) {
                                v["warning"] = json!(format!("cuts up to {depth:.2} mm into the part"));
                            }
                        }
                        Err(e) => v["error"] = json!(e.to_string()),
                    }
                    v
                })
                .collect();
            let mut v = json!({"id": setup.id, "name": setup.name, "axis": setup.axis, "origin": setup.origin, "stock": setup.stock,
                "material": setup.material, "post": setup.post, "operations": ops});
            // Worked out against the model: the work frame in the world, and the boxes in work coordinates.
            match resolve(s, setup) {
                Ok(at) => {
                    let boxed = |b: Option<Box3>| b.map(|(min, max)| json!({"min": min, "max": max}));
                    v["frame"] = at.frame.json();
                    v["stock_box"] = json!(boxed(at.stock));
                    v["part_box"] = json!(boxed(at.part));
                }
                Err(e) => v["error"] = json!(e),
            }
            v
        })
        .collect();
    json!({ "tools": m.tools.iter().map(tool_json).collect::<Vec<_>>(), "setups": setups, "posts": Post::BUILTIN, "materials": design_materials(s) })
}

fn toolpath(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.toolpath";
    let setup = setup(s, p, cmd)?;
    let only = match key(p, "op") {
        Some(k) => Some(op_position(setup, &k).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?),
        None => None,
    };
    let with_points = bool_(p, "points").unwrap_or(true);
    let at = resolve(s, setup).map_err(|e| bad(cmd, e))?;
    let mut out = Vec::new();
    for (i, op) in setup.operations.iter().enumerate() {
        if only.is_some_and(|o| o != i) {
            continue;
        }
        let path = build_in(s, &at, op)?;
        let mut v = stats_json(&path);
        v["id"] = json!(op.id);
        v["name"] = json!(op.name);
        v["kind"] = json!(op.kind.label());
        v["tool"] = json!(op.tool);
        // The heights it cuts at, in the setup's Z: for a form's picture of them.
        if bool_(p, "heights").unwrap_or(false) {
            let g = geometry(s, &at, &op.geometry).map_err(|e| bad(cmd, e))?;
            let c = op_cut(s, &at, op, &g)?;
            let h = c.heights;
            v["heights"] =
                json!({"clearance": h.clearance, "retract": h.retract, "top": h.top, "bottom": h.bottom, "levels": c.levels().map_err(cam_error)?});
        }
        if with_points {
            let points = path.points(TOLERANCE);
            if points.len() > MAX_POINTS {
                v["points_omitted"] = json!(points.len());
            } else {
                v["points"] = points
                    .iter()
                    .map(|(q, cutting)| {
                        let w = at.frame.world(*q);
                        json!([w.x, w.y, w.z, cutting])
                    })
                    .collect();
            }
        }
        out.push(v);
    }
    Ok(json!({ "setup": setup.id, "operations": out }))
}

/// The raw block a simulation starts from, on its columns: `stock` ({min, max}) or the setup's,
/// `cell` wide or `columns` across its longer side (default 300); and its corners.
fn sim_block(p: &Value, cmd: &str, at: &Resolved) -> Result<(cam::HeightField, Box3)> {
    let (min, max) = match p.get("stock") {
        Some(v) => {
            let corner = |k: &str| v.get(k).and_then(vec3).ok_or_else(|| bad(cmd, "`stock` must be {min: [x, y, z], max: [x, y, z]}"));
            (corner("min")?, corner("max")?)
        }
        None => at.stock.ok_or_else(|| bad(cmd, "there is no body to take the stock from; give `stock`"))?,
    };
    let cell = match (p.get("cell"), p.get("columns")) {
        (Some(_), _) => num(p, "cell").filter(|c| *c > 0.0).ok_or_else(|| bad(cmd, "`cell` must be above 0"))?,
        (None, Some(_)) => {
            let n = num(p, "columns").filter(|n| (10.0..=2000.0).contains(n)).ok_or_else(|| bad(cmd, "`columns` must be 10 to 2000"))?;
            ((max.x - min.x).max(max.y - min.y) / n).max(0.02)
        }
        (None, None) => ((max.x - min.x).max(max.y - min.y) / 300.0).max(0.02),
    };
    let stock = cam::HeightField::block(min, max, cell).map_err(|e| bad(cmd, e.to_string()))?;
    Ok((stock, (min, max)))
}

/// A setup's program to show running: what [`cam_playback`] gives.
pub struct CamPlayback {
    pub setup: u64,
    /// The setup's work frame in the world: origin, X, Y, Z. Everything else is in work coordinates.
    pub frame: [Vec3; 4],
    /// The raw block, uncut.
    pub stock: cam::HeightField,
    /// The operations' moves one after the other in time.
    pub playback: cam::Playback,
    /// Each operation's name and tool, by the playback's toolpath index.
    pub ops: Vec<String>,
    pub tools: Vec<Tool>,
}

/// A setup's program to show running (`p`: setup?, stock?, cell?, columns? as `cam.simulate`;
/// rapid_rate?: mm/min, default [`cam::DEFAULT_RAPID_RATE`]): the raw block and every
/// operation's moves in time, to cut it as far as the program has got.
pub fn cam_playback(s: &Session, p: &Value) -> Result<CamPlayback> {
    let cmd = "cam.simulate";
    let setup = setup(s, p, cmd)?;
    if setup.operations.is_empty() {
        return Err(bad(cmd, format!("setup `{}` has no operation", setup.name)));
    }
    let at = resolve(s, setup).map_err(|e| bad(cmd, e))?;
    let (stock, _) = sim_block(p, cmd, &at)?;
    let (mut paths, mut ops, mut tools) = (Vec::new(), Vec::new(), Vec::new());
    for op in &setup.operations {
        paths.push(build_in(s, &at, op)?);
        tools.push(s.doc.manufacture.tool(op.tool).cloned().ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?);
        ops.push(op.name.clone());
    }
    let rapid = match p.get("rapid_rate") {
        Some(_) => num(p, "rapid_rate").ok_or_else(|| bad(cmd, "`rapid_rate` must be a number (mm/min)"))?,
        None => cam::DEFAULT_RAPID_RATE,
    };
    let playback = cam::Playback::new(&paths.iter().collect::<Vec<_>>(), stock.cell, rapid).map_err(|e| bad(cmd, e.to_string()))?;
    let f = &at.frame;
    Ok(CamPlayback { setup: setup.id, frame: [f.o, f.x, f.y, f.z], stock, playback, ops, tools })
}

fn simulate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.simulate";
    let setup = setup(s, p, cmd)?;
    if setup.operations.is_empty() {
        return Err(bad(cmd, format!("setup `{}` has no operation", setup.name)));
    }
    let at = resolve(s, setup).map_err(|e| bad(cmd, e))?;
    let state = s.model.state();
    let meshes: Vec<_> = state.bodies.iter().map(|b| b.mesh()).collect();
    let (mut stock, (min, max)) = sim_block(p, cmd, &at)?;
    let cell = stock.cell;
    let mut ops = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut removed = 0.0;
    // Chamfers cut the part's edges on purpose, and the model has none: the stock without them
    // is what is checked against the part.
    let is_chamfer = |op: &CamOperation| matches!(op.kind, CamOperationKind::Chamfer { .. });
    let mut plain = setup.operations.iter().any(is_chamfer).then(|| stock.clone());
    for op in &setup.operations {
        let path = build_in(s, &at, op)?;
        let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        let report = cam::simulate(&mut stock, &path, tool).map_err(cam_error)?;
        if let Some(plain) = plain.as_mut().filter(|_| !is_chamfer(op)) {
            cam::simulate(plain, &path, tool).map_err(cam_error)?;
        }
        removed += report.removed_volume;
        let mut own = Vec::new();
        if report.rapid_collisions > 0 {
            own.push(format!("{} rapid move(s) run into material", report.rapid_collisions));
        }
        if report.max_engagement > tool.flute_length + 1e-6 {
            own.push(format!("the tool is buried {:.2} mm deep but its flutes are {:.2} mm long", report.max_engagement, tool.flute_length));
        }
        warnings.extend(own.iter().map(|w| format!("{}: {w}", op.name)));
        ops.push(json!({"id": op.id, "name": op.name, "removed_volume": report.removed_volume, "rapid_collisions": report.rapid_collisions,
            "max_engagement": report.max_engagement, "flute_length": tool.flute_length, "warnings": own}));
    }
    let mut out = json!({"setup": setup.id, "cell": cell, "columns": [stock.nx, stock.ny], "operations": ops, "removed_volume": removed});
    // The bodies seen from above, on the stock's columns.
    let triangles: u64 = meshes.iter().map(|m| m.triangles.len() as u64).sum();
    let mut deviation: Option<Vec<f64>> = None;
    if triangles == 0 {
        warnings.push("there is no body to compare the result with".into());
    } else {
        // Each column's centre seen from above, on the stock's own columns.
        let seen = cam::HeightField::of_triangles(Vec2::new(min.x, min.y), Vec2::new(max.x, max.y), cell, min.z, &part_triangles(s, &at), false);
        match seen {
            Ok(part) => {
                let c = cam::compare(&stock, &part).map_err(cam_error)?;
                let checked = match &plain {
                    Some(plain) => cam::compare(plain, &part).map_err(cam_error)?,
                    None => c,
                };
                // A column's width of error is expected at walls; less than that is not reported.
                if checked.max_gouge > 1e-3 {
                    warnings.push(format!("the program cuts up to {:.3} mm into the part, over {:.1} mm²", checked.max_gouge, checked.gouge_area));
                }
                out["max_gouge"] = json!(checked.max_gouge);
                out["gouge_area"] = json!(checked.gouge_area);
                out["max_leftover"] = json!(c.max_leftover);
                out["leftover_volume"] = json!(c.leftover_volume);
                let mut dev = cam::deviations(&stock, &part).map_err(cam_error)?;
                // Where only a chamfer went below the part, it is the chamfer, not a fault.
                if let Some(plain) = &plain {
                    let without = cam::deviations(plain, &part).map_err(cam_error)?;
                    for (((d, cut), uncut), w) in dev.iter_mut().zip(&stock.z).zip(&plain.z).zip(&without) {
                        if *d < 0.0 && *cut < *uncut - 1e-9 {
                            *d = w.min(0.0);
                        }
                    }
                    out["notes"] = json!(["Chamfers are left out of the check against the part: the model has none"]);
                }
                deviation = Some(dev);
            }
            Err(_) => warnings.push("the bodies are too detailed to compare at this cell size; use a larger `cell`".into()),
        }
    }
    if bool_(p, "field").unwrap_or(false) {
        // Columns in work coordinates, with the frame that puts them in the world; `deviation`
        // per column: below 0 cut into the part, above 0 left on it.
        out["field"] = json!({
            "frame": at.frame.json(),
            "origin": [stock.origin.x, stock.origin.y],
            "cell": stock.cell,
            "nx": stock.nx,
            "ny": stock.ny,
            "floor": stock.floor,
            "z": stock.z,
            "deviation": deviation,
        });
    }
    out["warnings"] = json!(warnings);
    Ok(out)
}

fn post(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.post";
    let setup = setup(s, p, cmd)?;
    let post = match p.get("post") {
        None => Post::builtin(&setup.post).ok_or_else(|| bad(cmd, format!("the setup's post `{}` is not known to this build", setup.post)))?,
        Some(Value::String(n)) => Post::builtin(n).ok_or_else(|| bad(cmd, format!("unknown post `{n}` (known: {})", Post::BUILTIN.join(", "))))?,
        Some(v @ Value::Object(_)) => serde_json::from_value(v.clone()).map_err(|e| bad(cmd, format!("post definition: {e}")))?,
        Some(_) => return Err(bad(cmd, "`post` must be a post name or a post definition object")),
    };
    if setup.operations.is_empty() {
        return Err(bad(cmd, format!("setup `{}` has no operation", setup.name)));
    }
    // One operation on its own, or the setup's whole program.
    let only = match key(p, "op") {
        Some(k) => Some(op_position(setup, &k).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?),
        None => None,
    };
    let name = match only.and_then(|i| setup.operations.get(i)) {
        Some(op) => format!("{} {} {}", s.doc.name, setup.name, op.name),
        None => format!("{} {}", s.doc.name, setup.name),
    };
    let mut program = cam::Program { name, operations: Vec::new() };
    let mut minutes = 0.0;
    let at = resolve(s, setup).map_err(|e| bad(cmd, e))?;
    for (_, op) in setup.operations.iter().enumerate().filter(|(i, _)| only.is_none_or(|o| o == *i)) {
        let path = build_in(s, &at, op)?;
        minutes += path.stats().cut_minutes;
        let tool = s.doc.manufacture.tool(op.tool).cloned().ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        program.operations.push(cam::Operation { name: op.name.clone(), tool, spindle_rpm: op.spindle_rpm, coolant: op.coolant, path });
    }
    let text = cam::emit(&program, &post).map_err(cam_error)?;
    let mut out = json!({"lines": text.lines().count(), "extension": post.extension, "post": post.name, "cut_minutes": minutes});
    match str_(p, "path") {
        Some(path) => {
            solidcraft_io::vfs::write(path, text.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            out["path"] = json!(path);
        }
        None => out["gcode"] = json!(text),
    }
    Ok(out)
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
