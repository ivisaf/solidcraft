//! Manufacture: the tool library, setups and their milling and drilling operations, the
//! toolpaths worked out from them, and G-code programs. Operations cut where a sketch's
//! profiles are; toolpaths are never stored, so they follow the design.

use serde_json::{Value, json};
use solidcraft_cam::{self as cam, ContourParams, Cut, DrillParams, Feeds, Heights, PocketParams, Post, Side, Tool, ToolKind, Toolpath};
use solidcraft_doc::expr::Kind;
use solidcraft_doc::manufacture::{CamOperation, CamOperationKind, Setup};
use solidcraft_geom::{Loop2, Plane, Region2, Seg2, Vec2, Vec3};

use super::CommandSpec;
use crate::params::{bad, bool_, expr, num, req_expr, str_, vec3};
use crate::{EngineError, Result, Session};

const MAX_TOOLS: usize = 1000;
const MAX_SETUPS: usize = 100;
const MAX_OPERATIONS: usize = 1000;
const MAX_NAME: usize = 128;
/// Chord error allowed where spline profiles are followed (mm).
const TOLERANCE: f64 = 0.01;
/// Most toolpath points `cam.toolpath` returns for one operation.
const MAX_POINTS: usize = 500_000;

/// Parameter documentation of an operation: its own inputs, then what every operation takes.
macro_rules! op_doc {
    ($own:literal) => {
        concat!(
            $own,
            "; sketch: id or name; profiles?: as solid.extrude (default all); tool: number; spindle_rpm; feed: mm/min; plunge_feed?: mm/min \
             (default feed/3); bottom: expr, final depth in the setup's Z; top?: expr (default: the sketch plane's height); retract?: expr \
             (default top + 2 mm); clearance?: expr (default top + 10 mm); stepdown?: expr (default half the tool diameter); coolant?: bool; \
             setup?: id or name (default: the only setup); name? → operation: id, name, and its toolpath's cut_length, rapid_length, cut_minutes"
        )
    };
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool.define", "Define Tool", tool_define)
        .params("number: 1–9999; kind: flat|ball|drill; diameter: expr; flute_length?: expr (default 3 × diameter); description? — adds the tool to the library or replaces the one with that number"),
    CommandSpec::new("cam.tool.remove", "Remove Tool", tool_remove).params("number (refused while an operation uses it)"),
    CommandSpec::new("cam.setup.create", "New Setup", setup_create)
        .params("name?; origin?: [x, y, z] work origin in world coordinates (default the world origin; work axes are the world's, tool along +Z); post?: rs274|grbl (default rs274) → setup: id, name"),
    CommandSpec::new("cam.setup.remove", "Remove Setup", setup_remove).params("setup: id or name"),
    CommandSpec::new("cam.op.face", "Face", op_face).params(op_doc!("stepover: expr (at most the tool diameter); faces the box around the profiles")),
    CommandSpec::new("cam.op.contour", "2D Contour", op_contour).params(op_doc!("side?: outside|inside|on (default outside: the profiles are the part); climb?: bool (default true); stock_to_leave?: expr")),
    CommandSpec::new("cam.op.pocket", "2D Pocket", op_pocket).params(op_doc!("stepover?: expr (default 40% of the tool diameter, at most its radius); climb?: bool (default true); stock_to_leave?: expr; inner profiles are islands")),
    CommandSpec::new("cam.op.drill", "Drill", op_drill).params(op_doc!("peck?: expr (depth per peck; default one pass); dwell?: seconds at the bottom; drills at the centre of each profile at the plunge feed")),
    CommandSpec::new("cam.op.remove", "Remove Operation", op_remove).params("op: id or name; setup?: id or name"),
    CommandSpec::new("cam.list", "List Manufacture", list)
        .noundo()
        .params("→ tools and setups with their operations; each operation with its toolpath's lengths and time, or the error that stops it"),
    CommandSpec::new("cam.toolpath", "Toolpath", toolpath).noundo().params(
        "setup?: id or name; op?: id or name (default every operation); points?: bool (default true) → operations: [{id, name, cut_length, \
         rapid_length, cut_minutes, lowest_z, points: [[x, y, z, cutting]] in world coordinates}]",
    ),
    CommandSpec::new("cam.post", "Post Process", post).noundo().params(
        "setup?: id or name; post?: rs274|grbl or a post definition object (default: the setup's); path?: file to write → lines, extension, \
         cut_minutes, and gcode (the program text) when no path is given",
    ),
];

fn cam_error(e: cam::CamError) -> EngineError {
    EngineError::Other(e.to_string())
}

fn mm(v: f64) -> String {
    format!("{v} mm")
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

fn tool_define(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool.define";
    let number =
        p.get("number").and_then(Value::as_u64).filter(|n| (1..=9999).contains(n)).ok_or_else(|| bad(cmd, "`number` must be 1 to 9999"))? as u32;
    let kind = match str_(p, "kind").map(|k| k.trim().to_ascii_lowercase()).as_deref() {
        Some("flat" | "flat_endmill" | "endmill") => ToolKind::FlatEndmill,
        Some("ball" | "ball_endmill") => ToolKind::BallEndmill,
        Some("drill") => ToolKind::Drill,
        _ => return Err(bad(cmd, "`kind` must be flat, ball or drill")),
    };
    let diameter = length(s, cmd, "diameter", &req_expr(cmd, p, "diameter")?)?;
    let flute_length = match expr(p, "flute_length") {
        Some(e) => length(s, cmd, "flute_length", &e)?,
        None => diameter * 3.0,
    };
    let description: String = str_(p, "description").unwrap_or("").chars().filter(|c| !c.is_control()).take(MAX_NAME).collect();
    let tool = Tool { number, kind, diameter, flute_length, description };
    tool.validate().map_err(|e| bad(cmd, e.to_string()))?;
    if s.doc.manufacture.tool(number).is_none() && s.doc.manufacture.tools.len() >= MAX_TOOLS {
        return Err(bad(cmd, "the tool library is full"));
    }
    let tools = &mut s.doc_mut().manufacture.tools;
    tools.retain(|t| t.number != number);
    tools.push(tool.clone());
    tools.sort_by_key(|t| t.number);
    Ok(json!({ "tool": tool }))
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
        Some(v) => {
            vec3(v).filter(|o| o.x.abs() <= 1e6 && o.y.abs() <= 1e6 && o.z.abs() <= 1e6).ok_or_else(|| bad(cmd, "`origin` must be [x, y, z]"))?
        }
        None => Vec3::ZERO,
    };
    let post = str_(p, "post").unwrap_or("rs274");
    let post = Post::builtin(post).ok_or_else(|| bad(cmd, format!("unknown post `{post}` (known: {})", Post::BUILTIN.join(", "))))?.name;
    let id = next_id(s);
    s.doc_mut().manufacture.setups.push(Setup { id, name: name.clone(), origin, post, operations: Vec::new() });
    Ok(json!({ "setup": id, "name": name }))
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
fn to_setup(plane: &Plane, origin: Vec3, regions: &[Region2]) -> Result<Vec<Region2>> {
    let n = plane.normal();
    if !(n.z.abs() > 1.0 - 1e-9) {
        return Err(EngineError::Other("not supported yet: operations on a sketch whose plane is not square to the setup's tool axis (Z)".into()));
    }
    let mirrored = n.z < 0.0;
    let x_axis = plane.dir_to_world(Vec2::new(1.0, 0.0));
    let turn = x_axis.y.atan2(x_axis.x);
    let point = |p: Vec2| {
        let w = plane.to_world(p) - origin;
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

/// The toolpath of `op` in its setup's work coordinates, from the model as it is now.
pub(crate) fn build(s: &Session, setup: &Setup, op: &CamOperation) -> Result<Toolpath> {
    let fail = |why: String| EngineError::Other(format!("operation `{}`: {why}", op.name));
    let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| fail(format!("there is no tool {} in the library", op.tool)))?;
    let len = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| fail(format!("{what}: {e}")));
    let heights = Heights {
        clearance: len("clearance", &op.clearance)?,
        retract: len("retract", &op.retract)?,
        top: len("top", &op.top)?,
        bottom: len("bottom", &op.bottom)?,
    };
    let cut = Cut { heights, feeds: Feeds { cut: op.cut_feed, plunge: op.plunge_feed }, stepdown: len("stepdown", &op.stepdown)? };
    let state = s.model.state();
    let sketch = state
        .sketches
        .iter()
        .find(|k| k.feature == op.sketch)
        .ok_or_else(|| fail("its sketch is not built (deleted, rolled back, suppressed or failed)".into()))?;
    let regions = solidcraft_doc::select_profiles(sketch, &op.profiles).map_err(|e| fail(e.to_string()))?;
    let regions = to_setup(&sketch.plane, setup.origin, &regions).map_err(|e| fail(e.to_string()))?;
    let path = match &op.kind {
        CamOperationKind::Face { stepover } => {
            let (mut min, mut max) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
            for p in regions.iter().flat_map(|r| r.outer.polyline(TOLERANCE)) {
                (min, max) = (Vec2::new(min.x.min(p.x), min.y.min(p.y)), Vec2::new(max.x.max(p.x), max.y.max(p.y)));
            }
            cam::face(min, max, tool, len("stepover", stepover)?, &cut)
        }
        CamOperationKind::Contour { side, climb, stock_to_leave } => {
            let params = ContourParams { side: *side, climb: *climb, stock_to_leave: len("stock_to_leave", stock_to_leave)?, tolerance: TOLERANCE };
            cam::contour(&regions, tool, &params, &cut)
        }
        CamOperationKind::Pocket { stepover, climb, stock_to_leave } => {
            let params = PocketParams {
                stepover: len("stepover", stepover)?,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                tolerance: TOLERANCE,
            };
            cam::pocket(&regions, tool, &params, &cut)
        }
        CamOperationKind::Drill { peck, dwell } => {
            let peck = peck.as_ref().map(|e| len("peck", e)).transpose()?;
            let points: Vec<Vec2> = regions.iter().map(|r| r.outer.centroid()).collect();
            cam::drill(&points, tool, &DrillParams { peck, dwell: *dwell }, &cut)
        }
    };
    path.map_err(|e| fail(e.to_string()))
}

fn stats_json(path: &Toolpath) -> Value {
    let st = path.stats();
    json!({"cut_length": st.cut_length, "rapid_length": st.rapid_length, "cut_minutes": st.cut_minutes, "lowest_z": st.lowest_z, "moves": path.moves.len()})
}

/// Reads what every operation takes and adds the operation once its toolpath has been worked out.
fn add_op(s: &mut Session, p: &Value, cmd: &str, label: &str, kind: impl FnOnce(&Tool) -> CamOperationKind) -> Result<Value> {
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?.clone();
    if setup.operations.len() >= MAX_OPERATIONS {
        return Err(bad(cmd, "the setup has too many operations"));
    }
    let sketch = super::features::sketch_id(s, p.get("sketch"), cmd, "sketch")?;
    let profiles = super::features::profiles(p, cmd)?;
    let number = p.get("tool").and_then(Value::as_u64).ok_or_else(|| bad(cmd, "`tool` must be a tool number"))?;
    let tool = s
        .doc
        .manufacture
        .tools
        .iter()
        .find(|t| u64::from(t.number) == number)
        .cloned()
        .ok_or_else(|| bad(cmd, format!("no tool {number} (cam.tool.define first)")))?;
    let positive = |k: &str| num(p, k).filter(|v| *v > 0.0 && *v <= 1.0e6);
    let spindle_rpm = positive("spindle_rpm").ok_or_else(|| bad(cmd, "`spindle_rpm` must be above 0"))?;
    let cut_feed = positive("feed").ok_or_else(|| bad(cmd, "`feed` must be above 0 (mm/min)"))?;
    let plunge_feed = match p.get("plunge_feed") {
        Some(_) => positive("plunge_feed").ok_or_else(|| bad(cmd, "`plunge_feed` must be above 0 (mm/min)"))?,
        None => cut_feed / 3.0,
    };
    let bottom = req_expr(cmd, p, "bottom")?;
    length(s, cmd, "bottom", &bottom)?;
    // Heights default from where the sketch is now; give an expression to make one follow the design.
    let top = match expr(p, "top") {
        Some(e) => e,
        None => {
            let state = s.model.state();
            let plane = state.sketches.iter().find(|k| k.feature == sketch).map(|k| k.plane).ok_or_else(|| bad(cmd, "the sketch is not built"))?;
            mm(plane.origin.z - setup.origin.z)
        }
    };
    let top_z = length(s, cmd, "top", &top)?;
    let retract = expr(p, "retract").unwrap_or_else(|| mm(top_z + 2.0));
    let clearance = expr(p, "clearance").unwrap_or_else(|| mm(top_z + 10.0));
    let stepdown = expr(p, "stepdown").unwrap_or_else(|| mm(tool.diameter * 0.5));
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
        sketch,
        profiles,
        kind,
    };
    // An operation that cannot be cut is not added.
    let path = build(s, &setup, &op).map_err(|e| bad(cmd, e.to_string()))?;
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

fn stock_to_leave(p: &Value) -> String {
    expr(p, "stock_to_leave").unwrap_or_else(|| mm(0.0))
}

fn op_face(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.face";
    let stepover = req_expr(cmd, p, "stepover")?;
    add_op(s, p, cmd, "Face", |_| CamOperationKind::Face { stepover })
}

fn op_contour(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.contour";
    let side = match str_(p, "side").map(|k| k.trim().to_ascii_lowercase()).as_deref() {
        None | Some("outside") => Side::Outside,
        Some("inside") => Side::Inside,
        Some("on") => Side::On,
        _ => return Err(bad(cmd, "`side` must be outside, inside or on")),
    };
    let kind = CamOperationKind::Contour { side, climb: bool_(p, "climb").unwrap_or(true), stock_to_leave: stock_to_leave(p) };
    add_op(s, p, cmd, "Contour", |_| kind)
}

fn op_pocket(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.pocket";
    let (stepover, climb, leave) = (expr(p, "stepover"), bool_(p, "climb").unwrap_or(true), stock_to_leave(p));
    add_op(s, p, cmd, "Pocket", |tool| CamOperationKind::Pocket {
        stepover: stepover.unwrap_or_else(|| mm(tool.diameter * 0.4)),
        climb,
        stock_to_leave: leave,
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

fn op_position(setup: &Setup, k: &str) -> Option<usize> {
    setup.operations.iter().position(|o| o.id.to_string() == k).or_else(|| setup.operations.iter().position(|o| o.name.eq_ignore_ascii_case(k)))
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
    let m = &s.doc.manufacture;
    let setups: Vec<Value> = m
        .setups
        .iter()
        .map(|setup| {
            let ops: Vec<Value> = setup
                .operations
                .iter()
                .map(|op| {
                    let mut v = json!(op);
                    v["kind"] = json!(op.kind.label());
                    match build(s, setup, op) {
                        Ok(path) => v["toolpath"] = stats_json(&path),
                        Err(e) => v["error"] = json!(e.to_string()),
                    }
                    v
                })
                .collect();
            json!({"id": setup.id, "name": setup.name, "origin": setup.origin, "post": setup.post, "operations": ops})
        })
        .collect();
    Ok(json!({ "tools": m.tools, "setups": setups, "posts": Post::BUILTIN }))
}

fn toolpath(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.toolpath";
    let setup = setup(s, p, cmd)?;
    let only = match key(p, "op") {
        Some(k) => Some(op_position(setup, &k).ok_or_else(|| bad(cmd, format!("no operation `{k}`")))?),
        None => None,
    };
    let with_points = bool_(p, "points").unwrap_or(true);
    let mut out = Vec::new();
    for (i, op) in setup.operations.iter().enumerate() {
        if only.is_some_and(|o| o != i) {
            continue;
        }
        let path = build(s, setup, op)?;
        let mut v = stats_json(&path);
        v["id"] = json!(op.id);
        v["name"] = json!(op.name);
        v["kind"] = json!(op.kind.label());
        v["tool"] = json!(op.tool);
        if with_points {
            let points = path.points(TOLERANCE);
            if points.len() > MAX_POINTS {
                v["points_omitted"] = json!(points.len());
            } else {
                v["points"] =
                    points.iter().map(|(q, cutting)| json!([q.x + setup.origin.x, q.y + setup.origin.y, q.z + setup.origin.z, cutting])).collect();
            }
        }
        out.push(v);
    }
    Ok(json!({ "setup": setup.id, "operations": out }))
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
    let mut program = cam::Program { name: format!("{} {}", s.doc.name, setup.name), operations: Vec::new() };
    let mut minutes = 0.0;
    for op in &setup.operations {
        let path = build(s, setup, op)?;
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
