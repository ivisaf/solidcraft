//! Manufacture: the tool library, setups and their milling and drilling operations, the
//! toolpaths worked out from them, and G-code programs. Operations cut where a sketch's
//! profiles are; toolpaths are never stored, so they follow the design.

use serde_json::{Value, json};
use solidcraft_cam::{self as cam, ContourParams, Cut, DrillParams, Feeds, Heights, PocketParams, Post, Side, Tool, ToolKind, Toolpath};
use solidcraft_doc::expr::Kind;
use solidcraft_doc::manufacture::{CamGeometry, CamOperation, CamOperationKind, FaceRef, Setup};
use solidcraft_doc::sketch::{LinkGeom, LinkKind};
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
            "; sketch: id or name, with profiles?: as solid.extrude (default all) — or faces: [{body?, point: [x, y, z]}], planar faces square to Z (a pocket's floor, a part's top; their holes are islands, or what a drill drills); tool: number; spindle_rpm; feed: mm/min; plunge_feed?: mm/min \
             (default feed/3); bottom?, top?: exprs in the setup's Z (a sketch gives the top and needs `bottom`; faces give both from the face and its body); \
             retract?: expr (default top + 2 mm); clearance?: expr (default 10 mm above the top and the body); stepdown?: expr (default half the tool diameter); coolant?: bool; \
             setup?: id or name (default: the only setup); name? → operation: id, name, and its toolpath's cut_length, rapid_length, cut_minutes"
        )
    };
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool.define", "Define Tool", tool_define)
        .at("MANUFACTURE", "SETUP")
        .icon("cam_tool")
        .params("number: 1–9999; kind: flat|ball|drill; diameter: expr; flute_length?: expr (default 3 × diameter); description? — adds the tool to the library or replaces the one with that number"),
    CommandSpec::new("cam.tool.remove", "Remove Tool", tool_remove).params("number (refused while an operation uses it)"),
    CommandSpec::new("cam.setup.create", "New Setup", setup_create)
        .at("MANUFACTURE", "SETUP")
        .icon("cam_setup")
        .params("name?; origin?: [x, y, z] work origin in world coordinates (default the world origin; work axes are the world's, tool along +Z); post?: rs274|grbl|lnc|lnc_atc (default rs274) → setup: id, name"),
    CommandSpec::new("cam.setup.remove", "Remove Setup", setup_remove).params("setup: id or name"),
    CommandSpec::new("cam.op.face", "Face", op_face).at("MANUFACTURE", "MILLING").icon("cam_face").params(op_doc!("stepover: expr (at most the tool diameter); faces the box around the profiles")),
    CommandSpec::new("cam.op.contour", "2D Contour", op_contour).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("side?: outside|inside|on (default outside: the profiles are the part); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down)")),
    CommandSpec::new("cam.op.pocket", "2D Pocket", op_pocket).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("stepover?: expr (default 40% of the tool diameter, at most its radius); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down); inner profiles are islands")),
    CommandSpec::new("cam.op.drill", "Drill", op_drill).at("MANUFACTURE", "DRILLING").icon("cam_drill").params(op_doc!("peck?: expr (depth per peck; default one pass); dwell?: seconds at the bottom; drills at the centre of each profile at the plunge feed")),
    CommandSpec::new("cam.op.edit", "Edit Operation", op_edit).params(
        "op: id or name; setup?: id or name; then any input of the operation's own command to change it (name, tool, spindle_rpm, feed, \
         plunge_feed, coolant, stepdown, sketch with profiles?, faces, stepover, side, climb, stock_to_leave, ramp_angle, peck, dwell); a height \
         (top, bottom, retract, clearance) or peck given as null goes back to automatic → the operation's toolpath lengths and time",
    ),
    CommandSpec::new("cam.op.move", "Move Operation", op_move)
        .params("op: id or name; to: position in the setup's order (0 is cut first) — or by: places later (negative: earlier); setup?: id or name → order: the setup's operation names"),
    CommandSpec::new("cam.op.remove", "Remove Operation", op_remove).params("op: id or name; setup?: id or name"),
    CommandSpec::new("cam.list", "Manufacture", list)
        .at("MANUFACTURE", "ACTIONS")
        .icon("cam_setup")
        .noundo()
        .params("→ tools and setups with their operations; each operation with its toolpath's lengths and time, or the error that stops it"),
    CommandSpec::new("cam.toolpath", "Show Toolpaths", toolpath).at("MANUFACTURE", "ACTIONS").icon("cam_contour").noundo().params(
        "setup?: id or name; op?: id or name (default every operation); points?: bool (default true) → operations: [{id, name, cut_length, \
         rapid_length, cut_minutes, lowest_z, points: [[x, y, z, cutting]] in world coordinates}]",
    ),
    CommandSpec::new("cam.simulate", "Simulate", simulate).at("MANUFACTURE", "ACTIONS").icon("cam_setup").noundo().params(
        "setup?: id or name; stock?: {min: [x, y, z], max: [x, y, z]} in world coordinates (default: the box around the bodies); cell?: column \
         width in mm (default: the stock's longer side / 300) → per operation removed_volume, rapid_collisions, max_engagement and warnings; and \
         against the bodies: max_gouge, gouge_area (cut into the part), max_leftover, leftover_volume (left on it); field?: bool, also the cut stock as columns {origin, cell, nx, ny, floor, z: [nx × ny heights], \
         deviation: [per column: below 0 cut into the part, above 0 left on it]} in world coordinates. Top-down: undercuts are not seen",
    ),
    CommandSpec::new("cam.post", "Post Process", post).at("MANUFACTURE", "ACTIONS").icon("cam_post").noundo().params(
        "setup?: id or name; post?: rs274|grbl|lnc|lnc_atc or a post definition object (default: the setup's); path?: file to write → lines, extension, \
         cut_minutes, and gcode (the program text) when no path is given",
    ),
];

fn cam_error(e: cam::CamError) -> EngineError {
    EngineError::Other(e.to_string())
}

/// A length as an expression, to the nanometre (so 6 × 0.4 reads `2.4 mm`).
fn mm(v: f64) -> String {
    format!("{} mm", (v * 1e6).round() / 1e6)
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

/// The centre of a loop that is one circle.
fn circle_centre(lp: &Loop2) -> Option<Vec2> {
    let Some(Seg2::Arc { center, radius, .. }) = lp.segs.first().copied() else { return None };
    let same = |s: &Seg2| matches!(*s, Seg2::Arc { center: c, radius: r, .. } if c.dist(center) <= 1e-6 && (r - radius).abs() <= 1e-6);
    let turned: f64 = lp.segs.iter().map(|s| if let Seg2::Arc { sweep, .. } = s { sweep.abs() } else { 0.0 }).sum();
    (lp.segs.iter().all(same) && (turned - std::f64::consts::TAU).abs() < 1e-6).then_some(center)
}

/// Where an operation cuts, in its setup's work coordinates.
struct Geometry {
    regions: Vec<Region2>,
    /// Height of the sketch plane or the picked faces.
    z: f64,
    /// Lowest and highest point of the picked faces' bodies.
    body_z: Option<(f64, f64)>,
}

fn geometry(s: &Session, setup: &Setup, source: &CamGeometry) -> std::result::Result<Geometry, String> {
    let state = s.model.state();
    match source {
        CamGeometry::Sketch { sketch, profiles } => {
            let sketch =
                state.sketches.iter().find(|k| k.feature == *sketch).ok_or("its sketch is not built (deleted, rolled back, suppressed or failed)")?;
            let regions = solidcraft_doc::select_profiles(sketch, profiles).map_err(|e| e.to_string())?;
            let regions = to_setup(&sketch.plane, setup.origin, &regions).map_err(|e| e.to_string())?;
            Ok(Geometry { regions, z: sketch.plane.origin.z - setup.origin.z, body_z: None })
        }
        CamGeometry::Faces { faces } => {
            let mut regions = Vec::new();
            let mut z: Option<f64> = None;
            let mut body_z: Option<(f64, f64)> = None;
            for face in faces {
                let body = state.body(&face.body).ok_or_else(|| format!("body `{}` is gone", face.body))?;
                let (index, ..) =
                    solidcraft_doc::naming::face_on_body(body, &face.name, face.at).ok_or_else(|| format!("its face on `{}` is gone", face.body))?;
                let plane = body.body.planar_face(index).ok_or("a picked face is not planar")?;
                let resolved = solidcraft_doc::project::resolve_face_indices(&state, &plane, LinkKind::Project, &face.body, &[index])
                    .map_err(|e| e.to_string())?;
                let region = outline(&resolved.geom)?;
                regions.extend(to_setup(&plane, setup.origin, &[region]).map_err(|e| e.to_string())?);
                let height = plane.origin.z - setup.origin.z;
                if z.is_some_and(|other| (other - height).abs() > 1e-6) {
                    return Err("the picked faces are at different heights; use one operation per height".into());
                }
                z = Some(height);
                let bounds = body.mesh().bounds();
                let (lo, hi) = (bounds.min.z - setup.origin.z, bounds.max.z - setup.origin.z);
                body_z = Some(body_z.map_or((lo, hi), |(a, b)| (a.min(lo), b.max(hi))));
            }
            Ok(Geometry { regions, z: z.ok_or("no face is picked")?, body_z })
        }
    }
}

/// The toolpath of `op` in its setup's work coordinates, from the model as it is now.
///
/// Heights the operation leaves out follow its geometry. A sketch gives the top (its plane).
/// Picked faces give: for a pocket, the bottom (the face: the pocket's floor) and the top (the
/// top of its body); for a contour, top and bottom of the body; for facing, a single pass at
/// the face; for drilling, the top (the face) and the bottom (the bottom of its body). Retract
/// and clearance are 2 mm and 10 mm above the top (clearance above the whole body).
pub(crate) fn build(s: &Session, setup: &Setup, op: &CamOperation) -> Result<Toolpath> {
    let fail = |why: String| EngineError::Other(format!("operation `{}`: {why}", op.name));
    let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| fail(format!("there is no tool {} in the library", op.tool)))?;
    let len = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| fail(format!("{what}: {e}")));
    let angle = |e: &str| s.doc.eval(e, Kind::Angle).map_err(|e| fail(format!("ramp_angle: {e}")));
    let g = geometry(s, setup, &op.geometry).map_err(fail)?;
    let picked = matches!(op.geometry, CamGeometry::Faces { .. });
    let (auto_top, auto_bottom) = match (&op.kind, g.body_z) {
        (CamOperationKind::Pocket { .. }, Some((_, hi))) if picked => (Some(hi), Some(g.z)),
        (CamOperationKind::Contour { .. }, Some((lo, hi))) if picked => (Some(hi), Some(lo)),
        (CamOperationKind::Drill { .. }, Some((lo, _))) if picked => (Some(g.z), Some(lo)),
        (CamOperationKind::Face { .. }, _) if picked => (Some(g.z), Some(g.z)),
        _ => (Some(g.z), None),
    };
    let height = |what: &str, given: &Option<String>, auto: Option<f64>| match given {
        Some(e) => len(what, e),
        None => auto.ok_or_else(|| fail(format!("`{what}` is required"))),
    };
    let top = height("top", &op.top, auto_top)?;
    let bottom = height("bottom", &op.bottom, auto_bottom)?;
    let heights = Heights {
        clearance: height("clearance", &op.clearance, Some(g.body_z.map_or(top, |(_, hi)| hi.max(top)) + 10.0))?,
        retract: height("retract", &op.retract, Some(top + 2.0))?,
        top,
        bottom,
    };
    let cut = Cut { heights, feeds: Feeds { cut: op.cut_feed, plunge: op.plunge_feed }, stepdown: len("stepdown", &op.stepdown)? };
    let regions = &g.regions;
    let path = match &op.kind {
        CamOperationKind::Face { stepover } => {
            let (mut min, mut max) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
            for p in regions.iter().flat_map(|r| r.outer.polyline(TOLERANCE)) {
                (min, max) = (Vec2::new(min.x.min(p.x), min.y.min(p.y)), Vec2::new(max.x.max(p.x), max.y.max(p.y)));
            }
            cam::face(min, max, tool, len("stepover", stepover)?, &cut)
        }
        CamOperationKind::Contour { side, climb, stock_to_leave, ramp_angle } => {
            let params = ContourParams {
                side: *side,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                tolerance: TOLERANCE,
                ramp_angle: angle(ramp_angle)?,
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
        CamOperationKind::Pocket { stepover, climb, stock_to_leave, ramp_angle } => {
            let params = PocketParams {
                stepover: len("stepover", stepover)?,
                climb: *climb,
                stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                tolerance: TOLERANCE,
                ramp_angle: angle(ramp_angle)?,
            };
            cam::pocket(regions, tool, &params, &cut)
        }
        CamOperationKind::Drill { peck, dwell } => {
            let peck = peck.as_ref().map(|e| len("peck", e)).transpose()?;
            // A sketch's profiles are the holes; a picked face's round inner loops are.
            let points: Vec<Vec2> = if picked {
                regions.iter().flat_map(|r| r.holes.iter().filter_map(circle_centre)).collect()
            } else {
                regions.iter().map(|r| r.outer.centroid()).collect()
            };
            if points.is_empty() {
                return Err(fail("the picked faces have no round holes to drill".into()));
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

/// Reads what every operation takes and adds the operation once its toolpath has been worked out.
fn add_op(s: &mut Session, p: &Value, cmd: &str, label: &str, kind: impl FnOnce(&Tool) -> CamOperationKind) -> Result<Value> {
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?.clone();
    if setup.operations.len() >= MAX_OPERATIONS {
        return Err(bad(cmd, "the setup has too many operations"));
    }
    let geometry = match (p.get("faces"), p.get("sketch")) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give either `sketch` or `faces`, not both")),
        (Some(faces), None) => CamGeometry::Faces { faces: faces_param(s, faces, cmd)? },
        _ => CamGeometry::Sketch {
            sketch: super::features::sketch_id(s, p.get("sketch"), cmd, "sketch")?,
            profiles: super::features::profiles(p, cmd)?,
        },
    };
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
    // A height left out follows the geometry (see `build`).
    let heights = ["clearance", "retract", "top", "bottom"].map(|k| match p.get(k) {
        Some(_) => req_expr(cmd, p, k).and_then(|e| length(s, cmd, k, &e).map(|_| Some(e))),
        None => Ok(None),
    });
    let [clearance, retract, top, bottom] = heights;
    let (clearance, retract, top, bottom) = (clearance?, retract?, top?, bottom?);
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
        geometry,
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

fn ramp_angle(p: &Value) -> String {
    expr(p, "ramp_angle").unwrap_or_else(|| "3 deg".into())
}

fn stock_to_leave(p: &Value) -> String {
    expr(p, "stock_to_leave").unwrap_or_else(|| mm(0.0))
}

fn op_face(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.face";
    let stepover = req_expr(cmd, p, "stepover")?;
    add_op(s, p, cmd, "Face", |_| CamOperationKind::Face { stepover })
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
    let kind =
        CamOperationKind::Contour { side, climb: bool_(p, "climb").unwrap_or(true), stock_to_leave: stock_to_leave(p), ramp_angle: ramp_angle(p) };
    add_op(s, p, cmd, "Contour", |_| kind)
}

fn op_pocket(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.pocket";
    let (stepover, climb, leave, ramp) = (expr(p, "stepover"), bool_(p, "climb").unwrap_or(true), stock_to_leave(p), ramp_angle(p));
    add_op(s, p, cmd, "Pocket", |tool| CamOperationKind::Pocket {
        stepover: stepover.unwrap_or_else(|| mm(tool.diameter * 0.4)),
        climb,
        stock_to_leave: leave,
        ramp_angle: ramp,
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
    let checked = |k: &str, kind: Kind| -> Result<String> {
        let e = req_expr(cmd, p, k)?;
        s.doc.eval(&e, kind).map_err(|err| bad(cmd, format!("{k}: {err}")))?;
        Ok(e)
    };
    // `null` puts an optional expression back to automatic.
    let optional =
        |k: &str| -> Result<Option<String>> { if p.get(k).is_some_and(Value::is_null) { Ok(None) } else { checked(k, Kind::Length).map(Some) } };
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
        (None, None) => {}
    }
    let unknown = |k: &str| bad(cmd, format!("a {} operation has no `{k}`", op.kind.label()));
    let mut kind = op.kind.clone();
    for k in ["stepover", "side", "climb", "stock_to_leave", "ramp_angle", "peck", "dwell"] {
        if !has(k) {
            continue;
        }
        match (&mut kind, k) {
            (CamOperationKind::Face { stepover } | CamOperationKind::Pocket { stepover, .. }, "stepover") => *stepover = checked(k, Kind::Length)?,
            (CamOperationKind::Contour { side, .. }, "side") => *side = side_param(p, cmd)?,
            (CamOperationKind::Contour { climb, .. } | CamOperationKind::Pocket { climb, .. }, "climb") => {
                *climb = bool_(p, k).ok_or_else(|| bad(cmd, "`climb` must be true or false"))?
            }
            (CamOperationKind::Contour { stock_to_leave, .. } | CamOperationKind::Pocket { stock_to_leave, .. }, "stock_to_leave") => {
                *stock_to_leave = checked(k, Kind::Length)?
            }
            (CamOperationKind::Contour { ramp_angle, .. } | CamOperationKind::Pocket { ramp_angle, .. }, "ramp_angle") => {
                *ramp_angle = checked(k, Kind::Angle)?
            }
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
                    // The kind's own inputs (stepover, side…) beside its name.
                    v["inputs"] = v["kind"].take();
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

/// Most ray tests when the bodies are sampled for the comparison.
const MAX_PART_WORK: u64 = 400_000_000;

fn simulate(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.simulate";
    let setup = setup(s, p, cmd)?;
    if setup.operations.is_empty() {
        return Err(bad(cmd, format!("setup `{}` has no operation", setup.name)));
    }
    let state = s.model.state();
    let meshes: Vec<_> = state.bodies.iter().map(|b| b.mesh()).collect();
    let (min, max) = match p.get("stock") {
        Some(v) => {
            let corner = |k: &str| v.get(k).and_then(vec3).ok_or_else(|| bad(cmd, "`stock` must be {min: [x, y, z], max: [x, y, z]}"));
            (corner("min")?, corner("max")?)
        }
        None => {
            let bounds = meshes.iter().fold(solidcraft_geom::Aabb3::EMPTY, |a, m| a.union(&m.bounds()));
            if bounds.is_empty() {
                return Err(bad(cmd, "there is no body to take the stock from; give `stock`"));
            }
            (bounds.min, bounds.max)
        }
    };
    let (min, max) = (min - setup.origin, max - setup.origin);
    let cell = match p.get("cell") {
        Some(_) => num(p, "cell").filter(|c| *c > 0.0).ok_or_else(|| bad(cmd, "`cell` must be above 0"))?,
        None => ((max.x - min.x).max(max.y - min.y) / 300.0).max(0.02),
    };
    let mut stock = cam::HeightField::block(min, max, cell).map_err(|e| bad(cmd, e.to_string()))?;
    let mut ops = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut removed = 0.0;
    for op in &setup.operations {
        let path = build(s, setup, op)?;
        let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        let report = cam::simulate(&mut stock, &path, tool).map_err(cam_error)?;
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
    } else if triangles.saturating_mul((stock.nx * stock.ny) as u64) > MAX_PART_WORK {
        warnings.push("the bodies are too detailed to compare at this cell size; use a larger `cell`".into());
    } else {
        let mut part = stock.clone();
        let above = max.z + 1.0;
        for j in 0..part.ny {
            for i in 0..part.nx {
                let c = part.centre(i, j);
                let from = Vec3::new(c.x, c.y, above) + setup.origin;
                let top = meshes.iter().filter_map(|m| m.raycast(from, Vec3::new(0.0, 0.0, -1.0))).map(|(t, _)| above - t).fold(part.floor, f64::max);
                if let Some(z) = part.z.get_mut(j * part.nx + i) {
                    *z = top;
                }
            }
        }
        let c = cam::compare(&stock, &part).map_err(cam_error)?;
        // A column's width of error is expected at walls; less than that is not reported.
        if c.max_gouge > 1e-3 {
            warnings.push(format!("the program cuts up to {:.3} mm into the part, over {:.1} mm²", c.max_gouge, c.gouge_area));
        }
        out["max_gouge"] = json!(c.max_gouge);
        out["gouge_area"] = json!(c.gouge_area);
        out["max_leftover"] = json!(c.max_leftover);
        out["leftover_volume"] = json!(c.leftover_volume);
        deviation = Some(cam::deviations(&stock, &part).map_err(cam_error)?);
    }
    if bool_(p, "field").unwrap_or(false) {
        // Heights in world Z; `deviation` per column: below 0 cut into the part, above 0 left on it.
        let lift = setup.origin.z;
        out["field"] = json!({
            "origin": [stock.origin.x + setup.origin.x, stock.origin.y + setup.origin.y],
            "cell": stock.cell,
            "nx": stock.nx,
            "ny": stock.ny,
            "floor": stock.floor + lift,
            "z": stock.z.iter().map(|z| z + lift).collect::<Vec<_>>(),
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
