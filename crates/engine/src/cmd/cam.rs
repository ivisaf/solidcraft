//! Manufacture: the tool library, setups and their milling and drilling operations, the
//! toolpaths worked out from them, and G-code programs. Operations cut where a sketch's
//! profiles are; toolpaths are never stored, so they follow the design.

use serde_json::{Value, json};
use solidcraft_cam::{
    self as cam, AdaptiveParams, BoreParams, ChamferParams, ContourParams, Cut, CuttingData, DrillParams, Feeds, Heights, PocketParams, Post, Side,
    Tool, ToolKind, Toolpath, VCarveParams,
};
use solidcraft_doc::Expr;
use solidcraft_doc::expr::Kind;
use solidcraft_doc::manufacture::{
    Anchor, BoxOf, CamGeometry, CamOperation, CamOperationKind, Clamp, Copies, EdgePoint, FaceRef, OriginZ, Setup, SetupOrigin, StockDef, StockShape,
    TabsDef,
};
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
             (defaults: the tool's data in the setup's material, else its own, a surface speed and chip load worked out for its diameter and flutes; plunge_feed else feed/3); max_spindle_rpm?: the machine's spindle, which a worked-out speed is cut down to (a chip load then feeds slower); bottom?, top?: exprs in the setup's Z (a sketch gives the top and needs `bottom`; faces give both from the face and its body); \
             retract?: expr (default top + 2 mm); clearance?: expr (default 10 mm above the top and the body); stepdown?: expr (default: the tool's, else half its diameter); passes?: how many depth passes instead (each the depth over this; null: by the stepdown); coolant?: bool; \
             setup?: id or name (default: the only setup); name? → operation: id, name, and its toolpath's cut_length, rapid_length, cut_minutes"
        )
    };
}

const SETUP_DOC: &str = "name?; axis?: +x|-x|+y|-y|+z|-z or [x, y, z], the direction the tool comes from: the setup's +Z (default +z); \
     stock?: {side?, top?, bottom?}: lengths added to the box around the bodies, seen along the axis (default 0), and shape?: offset (that box, the default) | \
     box, with size: [x, y, z] (lengths along the setup's axes; centred on the part across, its top `top` above the part's) | round, with diameter (a bar along \
     the axis, centred on the part across, from `bottom` below the part to `top` above it; no copies); bodies?: [names] the bodies that are the part \
     (default, or null: all; others are neither cut round nor checked against); origin?: the work origin: \
     [x, y, z] in the world, or {of?: stock|part, x?, y?, z?: min|center|max} on the stock's or the part's box in the setup's axes (default of \
     the object: the stock's top at its lowest X and Y; default without `origin`: the world origin), or {edge: [x, y, z] a point on a body's edge \
     or a vertex, body?, on?: vertex|middle|center (the end nearest the point, halfway along, or the centre of its circle; default: vertex at an \
     end, else center on a round edge, else middle), z?: point | {of?: stock|part, z?: min|center|max} (default: the stock's top)}: across the \
     tool axis at that point of the edge, which it follows as the design changes; post?: rs274|grbl|grblhal|lnc|lnc_atc (default \
     rs274); material?: what the stock is made of (operations take their tool's cutting data for it; null: none); work_offset?: G54 to G59 \
     (or 54 to 59; default G54), the work offset its programs run in; stock_from?: an earlier setup (id or name; null: none) whose result \
     is the stock, its tool axis the same or turned over (the part turned over about the world X axis for a setup from -z); x_axis?: the setup's X direction across its \
     axis, as axis is given (default: worked out from the axis; from -z, +x turns the part about X and -x about Y); clamps?: \
     [{name?, min: [x, y], max: [x, y], top}] boxes in work coordinates no tool may enter (operations that do are warned about); copies?: \
     {nx, ny, dx, dy}: copies of the part in rows along X and columns along Y, dx and dy apart, each cut by every operation (null: one) → setup: id, name";

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool.define", "Define Tool", tool_define).icon("library")
        .params(
            "number: 1–9999; kind: flat|ball|bull|drill|vbit (bull: a bull-nose end mill, its end rounded at the rim); diameter: expr; corner_radius?: mm, a bull nose's corner radius (needed; at most half the diameter); tip_angle?: degrees, the point's included angle (V-bit: default 90; drill: 118); tip_diameter?: mm, a V-bit's flat at its point; shank_diameter?, length?: mm, the shank and the length out of the holder (for its picture); holder_diameter?, holder_length?: mm, the holder above it, checked against clamps (it needs length); flute_length?: expr (default 3 × diameter; a V-bit's, its cone's height); description?; flutes?: 1–100 \
             (default 2); spindle_rpm?, feed?, plunge_feed?: mm/min, stepdown?, stepover?: mm — the cutting data operations with this tool start \
             from (null clears one); surface_speed?: m/min, instead of spindle_rpm: speed = surface_speed × 1000 / (π × diameter); chip_load?: mm per tooth, instead of feed: feed = speed × flutes × chip_load (giving one of a pair clears the other; listings report the chip load a feed makes); cutting?: [{material, spindle_rpm?, surface_speed?, feed?, chip_load?, plunge_feed?, stepdown?, stepover?}], its data in particular materials, replacing its own where given (null: none) — adds the tool to this design or replaces its tool of that number (cam.library.define: the machine's library)",
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
    CommandSpec::new("cam.setup.edit", "Edit Setup", setup_edit).icon("edit").params("setup: id or name; then any of name, axis, origin, stock, bodies, post, material, work_offset, stock_from, x_axis, clamps, copies as in cam.setup.create (what stock leaves out stays as it is)"),
    CommandSpec::new("cam.sheet", "Setup Sheet", sheet).icon("cam_post").noundo().params(
        "setup?: id or name; path?: file to write → lines, and text (Markdown) when no path is given: the setup sheet to print for the machine: \
         the stock and the work zero, the tools, the operations in order with their speeds, feeds, depths and times, and the warnings",
    ),
    CommandSpec::new("cam.template.save", "Save Template", template_save).icon("cam_setup").noundo().params(
        "setup?: id or name; path?: file to write → operations, and template (the JSON) when no path is given: the setup's operations and \
         their tools, sketches by name, to add to a setup of this design or another (cam.template.apply)",
    ),
    CommandSpec::new("cam.template.apply", "Apply Template", template_apply).icon("cam_setup").params(
        "setup?: id or name; path?: a template file, or template?: its JSON (what cam.template.save gives) → added: the new operations' \
         names. Sketches are found by name; a tool the design does not have is added from the template; an operation whose sketch \
         or faces this design does not have is added all the same, and says so until it is edited",
    ),
    CommandSpec::new("cam.setup.order", "Order by Tool", setup_order).icon("reorder").params(
        "setup?: id or name → order: the operations' names, tool_changes: before and after. Puts each tool's operations together, the tools \
         in the order they are first used and each tool's operations in their own order, so that the program changes tools as seldom as it can",
    ),
    CommandSpec::new("cam.setup.remove", "Remove Setup", setup_remove).icon("delete").params("setup: id or name"),
    CommandSpec::new("cam.op.face", "Facing", op_face).at("MANUFACTURE", "MILLING").icon("cam_face").params(op_doc!("stepover?: expr (default: the tool's own, else half its diameter; at most the diameter); faces the box around the profiles or faces — or, without them, boundary?: stock|part (default stock), from the stock's top down to the part's; limit?: over|inside the boundary (default over); margin?: expr, how far over or inside (default: over by the tool's radius, inside by 0)")),
    CommandSpec::new("cam.op.contour", "2D Contour", op_contour).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("lead?: expr, the radius of a quarter-circle lead-in and lead-out onto the wall (default none: it ramps along the loop; not with tabs); dogbones?: bool, into the sharp inside corners; skin?: expr, an onion skin left on the bottom, taken off in a last pass with skin_last?: bool; side?: outside|inside|on (default outside: the profiles are the part); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down); tabs?: {count?: on each loop (default 4), width?: expr along the edge (default 6 mm), height?: expr over the bottom (default 2 mm)}, holding a part being cut out (null: none); finish?: expr, an allowance the depth passes leave on the wall for one more pass at full depth (null: none)")),
    CommandSpec::new("cam.op.pocket", "2D Pocket", op_pocket).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("dogbones?: bool, into the pocket's sharp corners; stepover?: expr (default: the tool's own, else 40% of its diameter; at most the diameter); climb?: bool (default true); stock_to_leave?: expr; ramp_angle?: angle expr (default 3 deg; 0 plunges straight down); finish?: expr, an allowance the depth passes leave on the walls for one more pass along them at full depth (null: none); rest_from?: a larger tool's number (of this design) that cleared the pocket before: only what it left (corners, narrow places) is cut (null: the whole pocket); inner profiles are islands")),
    CommandSpec::new("cam.op.adaptive3d", "3D Adaptive", op_adaptive3d).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("roughs the stock down to the model level by level with a flat or ball end mill, at the steady load of cam.op.adaptive: the tool follows the model's shape seen from the tool's side (no undercuts), levels a stepdown apart at most and one on each flat floor of the model, so floors come out level; without a sketch or faces it roughs the whole stock (boundary?: stock|part, the box it stays in), with them only inside their profiles (where the tool's axis stays); the top defaults to the stock's top and the bottom to the model's deepest floor seen from the tool (not beside the part down to the stock's bottom); stepover_percent?, helix_angle?, helix_max?, helix_min?, climb?, feed_up?: as cam.op.adaptive; stock_to_leave?: expr, left on the model all round (sides and floors); step_up?: expr, levels in between on slopes this far apart at most, taking the steps the levels leave (default a quarter of the stepdown; 0: none; null: the default); it starts from what earlier operations of the setup (and the setup it takes its stock from) left, and a tool with a length and a holder diameter keeps its holder clear of the model and the stock")),
    CommandSpec::new("cam.op.parallel3d", "3D Parallel", op_parallel3d).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("finishes the model's shape in 3D with a flat, ball or bull-nose end mill: passes side by side across the area, each following the model seen from the tool's side (no undercuts) as low as the tool may go without coming nearer than stock_to_leave; without a sketch or faces the area is the model's box (boundary?: part|stock), with them inside their profiles (where the tool's axis stays); the top defaults to the stock's top and the bottom to the model's deepest floor seen from the tool; stepover?: expr, how far apart the passes are (default a tenth of the tool's diameter); angle?: degrees from X the passes run (default 0); stock_to_leave?: expr (default 0); zigzag?: bool, back and forth (default true; false: every pass the same way, up between them); rest?: bool, only where the setup's earlier operations left material on the model (rest finishing with a smaller tool; default false)")),
    CommandSpec::new("cam.op.zlevel3d", "3D Z-Level", op_zlevel3d).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!("finishes the model's steep walls in 3D with a flat, ball or bull-nose end mill: level by level from the top down, the tool runs round the model where it touches it at that height (a ball exactly against the model's triangles), where the model is at least `steep` steep; with 3D Parallel for the flatter areas; without a sketch or faces the area is the model's box (boundary?: part|stock), with them inside their profiles (where the tool's axis stays); the top defaults to the stock's top and the bottom to the model's deepest floor seen from the tool; step?: expr, how far apart the levels are (default a twelfth of the tool's diameter); steep?: degrees, only walls at least this steep (0 to under 90, default 30; 0: everywhere it touches); stock_to_leave?: expr (default 0); climb?: bool (default true: the model on the right of the tool's travel); rest?: bool, only where the setup's earlier operations left material on the model (default false)")),
    CommandSpec::new("cam.op.adaptive", "Adaptive", op_adaptive).at("MANUFACTURE", "MILLING").icon("cam_pocket").params(op_doc!("stepover_percent?: the heaviest cut, as a percentage of the tool's diameter (2 to 50, default 20): the tool steps through the material so that no step cuts wider, inner corners and narrow places included; stock_to_leave?: expr; finish?: expr, an allowance the passes leave on the walls for one more lap at full depth (null: none); helix_angle?: angle expr, the entry helix's (or ramp's) slope (default 5 deg); helix_max?, helix_min?: the entry helix's diameter as a percentage of the tool's, at most and at least (default 100, 10); climb?: bool (default true); rest_from?: a larger tool's number (of this design) that cleared the pocket before: only what it left is cut (null: the whole pocket); feed_up?: percent of the feed (100 to 500; null or 100: never faster): a step lighter than the stepover makes thinner chips and goes as much faster, up to this; cuts go as arcs where their steps lie on one; inner profiles are islands")),
    CommandSpec::new("cam.op.vcarve", "V-Carve", op_vcarve).at("MANUFACTURE", "MILLING").icon("cam_contour").params(op_doc!(
        "stepover?: expr, the distance between the rings (default: 2% of the tool's diameter); max_depth?: expr, the deepest the point goes \
         below the top (default: as deep as the shape asks); climb?: bool (default true); V-carves the closed profiles (letters, a logo) with a \
         V-bit: rings inside the outlines, each where the cone just touches the walls; the depth comes from the shape (bottom is not used)"
    )),
    CommandSpec::new("cam.op.thread", "Thread", op_thread).at("MANUFACTURE", "DRILLING").icon("cam_drill").params(op_doc!(
        "pitch: expr, the thread's pitch; diameter?: expr, its major diameter (default: a hole drawn at its core diameter plus 1.0825 pitches, a \
         boss as drawn); side?: inside|outside (default inside: a hole's thread; outside: a boss's); thread_passes?: 1 to 20 out to the full thread \
         depth (default 1); hand?: right|left (default right); climb?: bool (default true); mills a thread round each round profile or picked hole with a single-profile \
         thread mill, from the bottom up"
    )),
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
        "setup?: id or name; ops?: [id or name, …] the operations to cut and report (default: every one); first?: [id or name, …] operations cut \
         before them without a report, for the stock they leave (one operation on the stock the ones before it left: ops [it], first [those before]); \
         stock?: {min: [x, y, z], max: [x, y, z]} in the setup's work coordinates (default: the setup's stock); cell?: column \
         width in mm (default: the stock's longer side / 300) — or columns?: how many along the stock's longer side (10 to 2000) → per operation removed_volume, rapid_collisions, max_engagement and warnings; and \
         against the bodies: max_gouge, gouge_area (cut into the part), max_leftover, leftover_volume (left on it); field?: bool, also the cut stock as columns {frame, origin, cell, nx, ny, floor, z: [nx × ny heights], \
         deviation: [per column: below 0 cut into the part, above 0 left on it]} in work coordinates, with the frame that places them. Top-down: undercuts are not seen",
    ),
    CommandSpec::new("cam.post", "Post Process", post).at("MANUFACTURE", "ACTIONS").icon("cam_post").noundo().params(
        "setup?: id or name; op?: id or name, to write that operation alone (default: every operation, in order); post?: rs274|grbl|grblhal|lnc|lnc_atc or a post definition object (default: the setup's); path?: file to write; spindle_delay?: seconds to wait \
         after the spindle starts (0 to 60); optional_stops?: bool, an M1 before each operation after the first; air_cut?: mm every Z is raised (0 to 1000), \
         to run the program above the stock first → lines, extension, \
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
        Some("bull" | "bull_nose" | "bull-nose" | "bullnose") => ToolKind::BullNose,
        Some("drill") => ToolKind::Drill,
        Some("vbit" | "v_bit" | "v-bit") => ToolKind::VBit,
        _ => return Err(bad(cmd, "`kind` must be flat, ball, bull, drill or vbit")),
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
    // A speed is given as one of a pair, a feed as one of another: naming one clears the other.
    let (spindle_rpm, surface_speed) = rate_pair(p, cmd, ("spindle_rpm", "surface_speed"), was.map(|t| (t.spindle_rpm, t.surface_speed)), "")?;
    let (feed, chip_load) = rate_pair(p, cmd, ("feed", "chip_load"), was.map(|t| (t.feed, t.chip_load)), "")?;
    let cutting = match p.get("cutting") {
        None => was.map(|t| t.cutting.clone()).unwrap_or_default(),
        Some(v) => cutting_param(v, cmd)?,
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
        surface_speed,
        chip_load,
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
        // A bull nose's corner radius; another kind has none.
        corner_radius: data("corner_radius", was.and_then(|t| t.corner_radius))?.filter(|_| kind == ToolKind::BullNose),
        shank_diameter: data("shank_diameter", was.and_then(|t| t.shank_diameter))?,
        length: data("length", was.and_then(|t| t.length))?,
        holder_diameter: data("holder_diameter", was.and_then(|t| t.holder_diameter))?,
        holder_length: data("holder_length", was.and_then(|t| t.holder_length))?,
        cutting,
    };
    tool.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(tool)
}

/// `cutting`: a tool's data in particular materials (null: none); a material given nothing is left out.
fn cutting_param(v: &Value, cmd: &str) -> Result<Vec<CuttingData>> {
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
        let place = format!("cutting in {material}: ");
        let (spindle_rpm, surface_speed) = rate_pair(e, cmd, ("spindle_rpm", "surface_speed"), None, &place)?;
        let (feed, chip_load) = rate_pair(e, cmd, ("feed", "chip_load"), None, &place)?;
        let data = CuttingData {
            material: material.clone(),
            spindle_rpm,
            feed,
            surface_speed,
            chip_load,
            plunge_feed: value("plunge_feed")?,
            stepdown: value("stepdown")?,
            stepover: value("stepover")?,
        };
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
    if let (None, Some(rpm), Some(feed)) = (tool.chip_load, tool.spindle_rpm, tool.feed) {
        v["chip_load"] = json!(feed / (rpm * f64::from(tool.flutes)));
    }
    v
}

/// A pair of inputs that give one value two ways (a spindle speed or a surface speed, a feed or
/// a chip load): naming one clears the other, null clears it, and what is not named stays as `was`.
/// `place` starts its messages (where in the input the pair is).
fn rate_pair(p: &Value, cmd: &str, (a, b): (&str, &str), was: Option<(Option<f64>, Option<f64>)>, place: &str) -> Result<(Option<f64>, Option<f64>)> {
    let named = |k: &str| p.get(k).is_some_and(|v| !v.is_null());
    if named(a) && named(b) {
        return Err(bad(cmd, format!("{place}give `{a}` or `{b}`, not both")));
    }
    let limit = |k: &str| match k {
        "surface_speed" => (cam::MAX_SURFACE_SPEED, " (m/min)"),
        "chip_load" => (cam::MAX_CHIP_LOAD, " (mm per tooth)"),
        _ => (1.0e6, ""),
    };
    let value = |k: &str, was: Option<f64>| match p.get(k) {
        None => Ok(was),
        Some(Value::Null) => Ok(None),
        Some(_) => {
            let (max, unit) = limit(k);
            num(p, k)
                .filter(|v| *v > 0.0 && *v <= max)
                .map(Some)
                .ok_or_else(|| bad(cmd, format!("{place}`{k}` must be above 0 and at most {max}{unit}")))
        }
    };
    let (was_a, was_b) = was.unwrap_or_default();
    let (va, vb) = (value(a, was_a)?, value(b, was_b)?);
    Ok(match (named(a), named(b)) {
        (true, _) => (va, None),
        (_, true) => (None, vb),
        _ => (va, vb),
    })
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

/// `stock_from`: an earlier setup (id or name) whose result is the stock, or null for none. The
/// earlier setup's tool axis must be the same as `axis` or turned over. `at`: the setup's own
/// place in the list (a new one is last).
fn stock_from_param(s: &Session, p: &Value, at: Option<usize>, axis: Vec3, cmd: &str) -> Result<Option<Option<u64>>> {
    match p.get("stock_from") {
        None => Ok(None),
        Some(Value::Null) => Ok(Some(None)),
        Some(_) => {
            let k = key(p, "stock_from").ok_or_else(|| bad(cmd, "`stock_from` must be a setup (id or name) or null"))?;
            let setups = &s.doc.manufacture.setups;
            let before = at.unwrap_or(setups.len());
            let j = setups
                .iter()
                .position(|x| x.id.to_string() == k || x.name.eq_ignore_ascii_case(&k))
                .ok_or_else(|| bad(cmd, format!("no setup `{k}`")))?;
            if j >= before {
                return Err(bad(cmd, "a setup takes its stock from one before it"));
            }
            let earlier = setups.get(j).ok_or_else(|| bad(cmd, format!("no setup `{k}`")))?;
            if facing(earlier.axis, axis).is_none() {
                return Err(bad(cmd, "a setup takes its stock from one whose tool axis is the same or turned over"));
            }
            Ok(Some(Some(earlier.id)))
        }
    }
}

/// Most copies of the part on one setup.
const MAX_COPIES: u32 = 1000;

/// `copies`: `{nx, ny, dx, dy}` (`null`: one part).
fn copies_param(p: &Value, cmd: &str) -> Result<Option<Option<Copies>>> {
    let refused = || {
        bad(
            cmd,
            format!("`copies` must be {{nx, ny: 1 or more, {MAX_COPIES} in all; dx, dy: how far apart (mm), above 0 where there is more than one}}"),
        )
    };
    let c = match p.get("copies") {
        None => return Ok(None),
        Some(Value::Null) => return Ok(Some(None)),
        Some(c) => c,
    };
    let count = |k: &str| c.get(k).map_or(Some(1), Value::as_u64).filter(|n| (1..=u64::from(MAX_COPIES)).contains(n)).map(|n| n as u32);
    let gap = |k: &str| c.get(k).map_or(Some(0.0), Value::as_f64).filter(|v| v.is_finite() && (0.0..=1e5).contains(v));
    let (nx, ny) = (count("nx").ok_or_else(refused)?, count("ny").ok_or_else(refused)?);
    let (dx, dy) = (gap("dx").ok_or_else(refused)?, gap("dy").ok_or_else(refused)?);
    if nx * ny > MAX_COPIES || (nx > 1 && !(dx > 0.0)) || (ny > 1 && !(dy > 0.0)) {
        return Err(refused());
    }
    Ok(Some((nx * ny > 1).then_some(Copies { nx, ny, dx, dy })))
}

/// Most clamps on one setup.
const MAX_CLAMPS: usize = 64;

/// `clamps`: `[{name?, min: [x, y], max: [x, y], top}]` in work coordinates (`null` or `[]`: none).
fn clamps_param(p: &Value, cmd: &str) -> Result<Option<Vec<Clamp>>> {
    let refused = || bad(cmd, "`clamps` must be a list of {name?, min: [x, y], max: [x, y], top} with min below max");
    let list = match p.get("clamps") {
        None => return Ok(None),
        Some(Value::Null) => return Ok(Some(Vec::new())),
        Some(Value::Array(a)) if a.len() <= MAX_CLAMPS => a,
        Some(_) => return Err(refused()),
    };
    let mut out = Vec::new();
    for c in list {
        let pair = |k: &str| -> Option<[f64; 2]> {
            let a = c.get(k)?.as_array()?;
            let (x, y) = (a.first()?.as_f64()?, a.get(1)?.as_f64()?);
            (x.is_finite() && y.is_finite() && x.abs() <= 1e6 && y.abs() <= 1e6).then_some([x, y])
        };
        let (min, max) = (pair("min").ok_or_else(refused)?, pair("max").ok_or_else(refused)?);
        let top = c.get("top").and_then(Value::as_f64).filter(|t| t.is_finite() && t.abs() <= 1e6).ok_or_else(refused)?;
        if !(min[0] < max[0] && min[1] < max[1]) {
            return Err(refused());
        }
        let name: String = c.get("name").and_then(Value::as_str).unwrap_or_default().chars().take(MAX_NAME).collect();
        out.push(Clamp { name, min, max, top });
    }
    Ok(Some(out))
}

/// The first clamp a toolpath's tool, `radius` round, comes into: anywhere below a clamp's top
/// within `radius` of its box across. Its name, or its number counted from 1.
fn clamp_hit(path: &cam::Toolpath, radius: f64, clamps: &[Clamp]) -> Option<String> {
    if clamps.is_empty() {
        return None;
    }
    let points = path.points(0.05);
    let step = (radius * 0.5).max(0.05);
    let mut samples = 0usize;
    for w in points.windows(2) {
        let (a, b) = (w[0].0, w[1].0);
        let n = (((b - a).len() / step).ceil() as usize).clamp(1, 10_000);
        for k in 0..=n {
            samples += 1;
            if samples > MAX_POINTS {
                return None;
            }
            let q = a + (b - a) * (k as f64 / n as f64);
            for (i, c) in clamps.iter().enumerate() {
                let dx = (c.min[0] - q.x).max(q.x - c.max[0]).max(0.0);
                let dy = (c.min[1] - q.y).max(q.y - c.max[1]).max(0.0);
                if q.z < c.top - 1e-9 && dx.hypot(dy) < radius - 1e-9 {
                    return Some(if c.name.is_empty() { format!("{}", i + 1) } else { c.name.clone() });
                }
            }
        }
    }
    None
}

/// Whether two tool axes are the same (`Some(true)`) or turned over (`Some(false)`).
fn facing(a: Vec3, b: Vec3) -> Option<bool> {
    let d = a.normalized()?.dot(b.normalized()?);
    if d > 1.0 - 1e-9 {
        Some(true)
    } else if d < -1.0 + 1e-9 {
        Some(false)
    } else {
        None
    }
}

/// `x_axis`: the setup's X direction, as `axis` is given (`null`: worked out from the axis).
fn x_axis_param(p: &Value, cmd: &str) -> Result<Option<Option<Vec3>>> {
    match p.get("x_axis") {
        None => Ok(None),
        Some(Value::Null) => Ok(Some(None)),
        Some(v) => {
            axis_param(v, cmd).map(|x| Some(Some(x))).map_err(|_| bad(cmd, "`x_axis` must be +x, -x, +y, -y, +z, -z or a direction [x, y, z]"))
        }
    }
}

/// `work_offset`: 54 to 59, or `G54` to `G59`.
fn work_offset_param(p: &Value, cmd: &str) -> Result<Option<u8>> {
    let refused = || bad(cmd, "`work_offset` must be G54 to G59 (or 54 to 59)");
    let n = match p.get("work_offset") {
        None | Some(Value::Null) => return Ok(None),
        Some(Value::Number(n)) => n.as_u64().ok_or_else(refused)?,
        Some(Value::String(t)) => t.trim().trim_start_matches(['G', 'g']).parse::<u64>().map_err(|_| refused())?,
        Some(_) => return Err(refused()),
    };
    (54..=59).contains(&n).then_some(Some(n as u8)).ok_or_else(refused)
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
        Some(v) => origin_param(s, v, cmd)?,
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
    let work_offset = work_offset_param(p, cmd)?.unwrap_or(54);
    let stock_from = stock_from_param(s, p, None, axis, cmd)?.flatten();
    let x_axis = x_axis_param(p, cmd)?.flatten();
    let clamps = clamps_param(p, cmd)?.unwrap_or_default();
    let copies = copies_param(p, cmd)?.flatten();
    let bodies = bodies_param(s, p, cmd)?.unwrap_or_default();
    let mut setup = Setup {
        id: 0,
        name: name.clone(),
        axis,
        origin,
        stock,
        bodies,
        material,
        post,
        work_offset,
        stock_from,
        clamps,
        copies,
        x_axis,
        operations: Vec::new(),
    };
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
        setup.origin = origin_param(s, v, cmd)?;
    }
    if let Some(v) = p.get("stock") {
        setup.stock = stock_param(s, v, &setup.stock, cmd)?;
    }
    if let Some(bodies) = bodies_param(s, p, cmd)? {
        setup.bodies = bodies;
    }
    if let Some(offset) = work_offset_param(p, cmd)? {
        setup.work_offset = offset;
    }
    if let Some(from) = stock_from_param(s, p, Some(i), setup.axis, cmd)? {
        setup.stock_from = from;
    }
    if let Some(clamps) = clamps_param(p, cmd)? {
        setup.clamps = clamps;
    }
    if let Some(copies) = copies_param(p, cmd)? {
        setup.copies = copies;
    }
    if let Some(x) = x_axis_param(p, cmd)? {
        setup.x_axis = x;
    }
    // A turned axis can leave a setup it takes its stock from facing neither way.
    if let Some(from) = setup.stock_from
        && let Some(earlier) = s.doc.manufacture.setups.iter().find(|x| x.id == from)
        && facing(earlier.axis, setup.axis).is_none()
    {
        return Err(bad(cmd, "a setup takes its stock from one whose tool axis is the same or turned over"));
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

/// The work origin from a world point `[x, y, z]`, `{of?: stock|part, x?, y?, z?: min|center|max}`
/// or `{edge: [x, y, z], body?, on?: vertex|middle|center, z?: point | {of?, z?}}`.
fn origin_param(s: &Session, v: &Value, cmd: &str) -> Result<SetupOrigin> {
    if let Some(at) = vec3(v).filter(|o| o.x.abs() <= 1e6 && o.y.abs() <= 1e6 && o.z.abs() <= 1e6) {
        return Ok(SetupOrigin::Point { at });
    }
    let wrong = || {
        bad(
            cmd,
            "`origin` must be [x, y, z], {of?: stock|part, x?, y?, z?: min|center|max} or {edge: [x, y, z], body?, on?: vertex|middle|center, \
             z?: point | {of?: stock|part, z?: min|center|max}} (default: the stock's top, at its lowest X and Y)",
        )
    };
    let o = v.as_object().ok_or_else(wrong)?;
    if o.contains_key("edge") {
        return edge_origin_param(s, o, cmd);
    }
    if o.keys().any(|k| !matches!(k.as_str(), "of" | "x" | "y" | "z")) {
        return Err(wrong());
    }
    let of = box_of(o.get("of")).ok_or_else(wrong)?;
    let anchor = |k: &str, default: Anchor| anchor_param(o.get(k), default).ok_or_else(wrong);
    Ok(SetupOrigin::Box { of, x: anchor("x", Anchor::Min)?, y: anchor("y", Anchor::Min)?, z: anchor("z", Anchor::Max)? })
}

/// Which box, from `stock` or `part` (default: the stock's).
fn box_of(v: Option<&Value>) -> Option<BoxOf> {
    match v.map(|x| x.as_str().unwrap_or_default().to_ascii_lowercase()).as_deref() {
        None | Some("stock") => Some(BoxOf::Stock),
        Some("part") => Some(BoxOf::Part),
        _ => None,
    }
}

/// Where along a box's axis, from `min`, `center` or `max`.
fn anchor_param(v: Option<&Value>, default: Anchor) -> Option<Anchor> {
    match v.map(|x| x.as_str().unwrap_or_default().to_ascii_lowercase()).as_deref() {
        None => Some(default),
        Some("min") => Some(Anchor::Min),
        Some("center" | "centre" | "mid" | "middle") => Some(Anchor::Center),
        Some("max") => Some(Anchor::Max),
        _ => None,
    }
}

/// A work origin on the body edge nearest `edge` (a point on it or one of its ends).
fn edge_origin_param(s: &Session, o: &serde_json::Map<String, Value>, cmd: &str) -> Result<SetupOrigin> {
    let wrong = |what: &str| bad(cmd, format!("the origin's {what}"));
    if o.keys().any(|k| !matches!(k.as_str(), "edge" | "body" | "on" | "z")) {
        return Err(wrong("keys are edge, body, on and z"));
    }
    let at = o.get("edge").and_then(vec3).filter(|p| p.is_finite()).ok_or_else(|| wrong("`edge` must be [x, y, z], a point on the edge"))?;
    let wanted = match o.get("body") {
        None | Some(Value::Null) => None,
        Some(v) => Some(v.as_str().ok_or_else(|| wrong("`body` must be a body's name"))?),
    };
    let state = s.model.state();
    // The edge nearest the point, of the bodies it may be on; of edges through it, one it lies
    // along before one that ends there (a hole's rim, not the seam down its side).
    let mut best: Option<((u8, f64), &solidcraft_doc::ModelBody, usize)> = None;
    for b in state.bodies.iter().filter(|b| !b.body.is_mesh() && wanted.is_none_or(|w| b.name.eq_ignore_ascii_case(w))) {
        let (m, tol) = (b.mesh(), (b.body.size() * 1e-3).max(1e-3));
        for (i, e) in m.edges.iter().enumerate() {
            let d = e.windows(2).map(|w| at.dist_to_segment(w[0], w[1])).fold(f64::MAX, f64::min);
            let ends_there = [e.first(), e.last()].into_iter().flatten().any(|p| p.dist(at) <= tol);
            let key = (u8::from(!(d <= tol && !ends_there)), d);
            if best.as_ref().is_none_or(|(bk, ..)| key.0.cmp(&bk.0).then(key.1.total_cmp(&bk.1)).is_lt()) {
                best = Some((key, b, i));
            }
        }
    }
    let ((_, d), body, index) = best.ok_or_else(|| wrong("edge: there is no body edge"))?;
    let tol = (body.body.size() * 1e-3).max(1e-3);
    if d > tol {
        return Err(bad(cmd, format!("no body edge at [{}, {}, {}]", mm(at.x), mm(at.y), mm(at.z))));
    }
    let edge = solidcraft_doc::naming::edge_names(body).get(index).cloned().filter(|n| !n.is_empty()).ok_or_else(|| wrong("edge has no name"))?;
    let on = match o.get("on").map(|x| x.as_str().unwrap_or_default().to_ascii_lowercase()).as_deref() {
        Some("vertex" | "corner" | "end") => EdgePoint::Vertex,
        Some("middle" | "mid") => EdgePoint::Middle,
        Some("center" | "centre") => EdgePoint::Center,
        None => {
            let pts = body.mesh().edges.get(index).cloned().unwrap_or_default();
            let at_end = [pts.first(), pts.last()].into_iter().flatten().any(|p| p.dist(at) <= tol);
            if at_end {
                EdgePoint::Vertex
            } else if super::measure_sel::edge_centre(body, index).is_some() {
                EdgePoint::Center
            } else {
                EdgePoint::Middle
            }
        }
        Some(_) => return Err(wrong("`on` must be vertex, middle or center")),
    };
    if on == EdgePoint::Center && super::measure_sel::edge_centre(body, index).is_none() {
        return Err(bad(cmd, "the origin's edge is not round: it has no centre (take its vertex or middle)"));
    }
    let z = match o.get("z") {
        None => OriginZ::default(),
        Some(Value::String(t)) if t.eq_ignore_ascii_case("point") => OriginZ::Point,
        Some(Value::Object(z)) if z.keys().all(|k| matches!(k.as_str(), "of" | "z")) => OriginZ::Box {
            of: box_of(z.get("of")).ok_or_else(|| wrong("z's `of` must be stock or part"))?,
            z: anchor_param(z.get("z"), Anchor::Max).ok_or_else(|| wrong("z's `z` must be min, center or max"))?,
        },
        Some(_) => return Err(wrong("`z` must be point or {of?: stock|part, z?: min|center|max}")),
    };
    Ok(SetupOrigin::Edge { body: body.name.clone(), edge, at, on, z })
}

/// The point `on` of the edge `name` of `body` (found by name, else as the piece nearest `at`).
fn edge_origin(body: &solidcraft_doc::ModelBody, name: &str, at: Vec3, on: EdgePoint) -> std::result::Result<Vec3, String> {
    let hit = solidcraft_doc::naming::locate_edge(body, name, at).map_err(|e| format!("the setup's origin: {e}"))?;
    let m = body.mesh();
    let pts = m.edges.get(hit.edge).filter(|e| !e.is_empty()).ok_or("the setup's origin edge has no points")?;
    match on {
        EdgePoint::Vertex => [pts.first(), pts.last()]
            .into_iter()
            .flatten()
            .min_by(|a, b| a.dist(at).total_cmp(&b.dist(at)))
            .copied()
            .ok_or_else(|| "the setup's origin edge has no ends".into()),
        EdgePoint::Middle => {
            let half = pts.windows(2).map(|w| w[0].dist(w[1])).sum::<f64>() * 0.5;
            let mut gone = 0.0;
            for w in pts.windows(2) {
                let l = w[0].dist(w[1]);
                if gone + l >= half && l > 0.0 {
                    return Ok(w[0] + (w[1] - w[0]) * ((half - gone) / l));
                }
                gone += l;
            }
            pts.first().copied().ok_or_else(|| "the setup's origin edge has no points".into())
        }
        EdgePoint::Center => super::measure_sel::edge_centre(body, hit.edge)
            .ok_or_else(|| format!("the setup's origin edge `{name}` is no longer round: it has no centre")),
    }
}

/// Stock from `{side?, top?, bottom?, shape?, size?, diameter?}`; what is left out keeps its value in `was`.
fn stock_param(s: &Session, v: &Value, was: &StockDef, cmd: &str) -> Result<StockDef> {
    if !v.is_object() {
        return Err(bad(cmd, "`stock` must be {side?, top?, bottom?, shape?, size?, diameter?}"));
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
    // A size above nothing, as typed.
    let size = |what: &str, e: &Value| -> Result<Expr> {
        let e = match e {
            Value::Number(n) => format!("{n} mm"),
            Value::String(t) if !t.trim().is_empty() => t.trim().to_string(),
            _ => return Err(bad(cmd, format!("the stock's {what} must be a length"))),
        };
        let d = length(s, cmd, what, &e)?;
        if d > 0.0 && d <= 1.0e6 { Ok(e.into()) } else { Err(bad(cmd, format!("the stock's {what} must be above 0"))) }
    };
    let named = v.get("shape").map(|n| n.as_str().unwrap_or_default());
    let shape = match (named, &was.shape) {
        (Some("offset"), _) | (None, StockShape::Offset) => StockShape::Offset,
        (Some("box"), _) | (None, StockShape::Box { .. }) => {
            let was = match &was.shape {
                StockShape::Box { x, y, z } => Some([x.clone(), y.clone(), z.clone()]),
                _ => None,
            };
            let [x, y, z] = match (v.get("size"), was) {
                (Some(Value::Array(a)), _) if a.len() == 3 => {
                    [size("size along X", &a[0])?, size("size along Y", &a[1])?, size("size along Z", &a[2])?]
                }
                (Some(_), _) => return Err(bad(cmd, "the stock's `size` must be [x, y, z], lengths along the setup's axes")),
                (None, Some(was)) => was,
                (None, None) => return Err(bad(cmd, "a box stock needs its `size`: [x, y, z]")),
            };
            StockShape::Box { x, y, z }
        }
        (Some("round"), _) | (None, StockShape::Round { .. }) => {
            let diameter = match (v.get("diameter"), &was.shape) {
                (Some(d), _) => size("diameter", d)?,
                (None, StockShape::Round { diameter }) => diameter.clone(),
                (None, _) => return Err(bad(cmd, "a round stock needs its `diameter`")),
            };
            StockShape::Round { diameter }
        }
        (Some(other), _) => return Err(bad(cmd, format!("unknown stock shape `{other}` (known: offset, box, round)"))),
    };
    Ok(StockDef { side: offset("side", &was.side)?, top: offset("top", &was.top)?, bottom: offset("bottom", &was.bottom)?, shape })
}

/// Most bodies a setup names as its part.
const MAX_PART_BODIES: usize = 256;

/// `bodies`: the bodies that are the part, each named as the design has it (`Some(empty)` for
/// null or []: all of them); `None` when not given.
fn bodies_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<Vec<String>>> {
    let list = match p.get("bodies") {
        None => return Ok(None),
        Some(Value::Null) => return Ok(Some(Vec::new())),
        Some(v) => v
            .as_array()
            .filter(|l| l.len() <= MAX_PART_BODIES)
            .ok_or_else(|| bad(cmd, format!("`bodies` must be a list of at most {MAX_PART_BODIES} body names")))?,
    };
    let state = s.model.state();
    let mut out: Vec<String> = Vec::new();
    for name in list {
        let name = name.as_str().ok_or_else(|| bad(cmd, "`bodies` must be body names"))?;
        let body = state.bodies.iter().find(|b| b.name.eq_ignore_ascii_case(name.trim())).ok_or_else(|| bad(cmd, format!("no body `{name}`")))?;
        if !out.iter().any(|o| o.eq_ignore_ascii_case(&body.name)) {
            out.push(body.name.clone());
        }
    }
    Ok(Some(out))
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

/// A setup's work X, Y and Z: its own X direction where it has one (made square to the axis),
/// else as [`axes`] works them out.
fn setup_axes(setup: &Setup) -> Option<(Vec3, Vec3, Vec3)> {
    let (x, _, z) = axes(setup.axis)?;
    let x = match setup.x_axis {
        Some(v) => (v - z * v.dot(z)).normalized().filter(|x| x.is_finite())?,
        None => x,
    };
    Some((x, z.cross(x), z))
}

/// A box in work coordinates.
type Box3 = (Vec3, Vec3);

/// A setup worked out against the model: its frame, and the boxes round the bodies and the stock.
pub(crate) struct Resolved {
    /// The setup it is.
    setup: u64,
    frame: Frame,
    part: Option<Box3>,
    stock: Option<Box3>,
    /// A round stock's centre and radius across (its box is `stock`).
    round: Option<(Vec2, f64)>,
    /// The bodies that are the part (empty: every body).
    bodies: Vec<String>,
    /// Where each copy of the part is, from the first (one at 0, 0 without copies).
    copies: Vec<(f64, f64)>,
}

impl Resolved {
    /// Whether the body called `name` is part of the part.
    fn is_part(&self, name: &str) -> bool {
        self.bodies.is_empty() || self.bodies.iter().any(|b| b.eq_ignore_ascii_case(name))
    }
}

fn resolve(s: &Session, setup: &Setup) -> std::result::Result<Resolved, String> {
    let (x, y, z) = setup_axes(setup).ok_or("the setup's tool axis is not a direction, or its X direction runs along it")?;
    // The bodies' extent along the setup's axes, measured from the world origin.
    let state = s.model.state();
    if let Some(gone) = setup.bodies.iter().find(|n| !state.bodies.iter().any(|b| b.name.eq_ignore_ascii_case(n))) {
        return Err(format!("the setup's part names the body `{gone}`, which the design no longer has"));
    }
    let wanted = |name: &str| setup.bodies.is_empty() || setup.bodies.iter().any(|b| b.eq_ignore_ascii_case(name));
    let mut part: Option<Box3> = None;
    for body in state.bodies.iter().filter(|b| wanted(&b.name)) {
        for p in &body.mesh().positions {
            let q = Vec3::new(p.dot(x), p.dot(y), p.dot(z));
            part = Some(part.map_or((q, q), |(lo, hi)| (lo.min(q), hi.max(q))));
        }
    }
    let offset = |what: &str, e: &str| s.doc.eval(e, Kind::Length).map_err(|e| format!("the stock's {what} offset: {e}")).map(|d| d.max(0.0));
    let (side, top, bottom) = (offset("side", &setup.stock.side)?, offset("top", &setup.stock.top)?, offset("bottom", &setup.stock.bottom)?);
    let size = |what: &str, e: &str| {
        s.doc
            .eval(e, Kind::Length)
            .map_err(|e| format!("the stock's {what}: {e}"))
            .and_then(|d| if d > 0.0 && d.is_finite() { Ok(d) } else { Err(format!("the stock's {what} must be above 0")) })
    };
    let mut round = None;
    let stock = match (&setup.stock.shape, part) {
        (_, None) => None,
        (StockShape::Offset, Some((lo, hi))) => Some((lo - Vec3::new(side, side, bottom), hi + Vec3::new(side, side, top))),
        // Centred on the part across; its top `top` above the part's.
        (StockShape::Box { x: sx, y: sy, z: sz }, Some((lo, hi))) => {
            let (sx, sy, sz) = (size("size along X", sx)?, size("size along Y", sy)?, size("size along Z", sz)?);
            let (c, top) = ((lo + hi) * 0.5, hi.z + top);
            Some((Vec3::new(c.x - sx * 0.5, c.y - sy * 0.5, top - sz), Vec3::new(c.x + sx * 0.5, c.y + sy * 0.5, top)))
        }
        (StockShape::Round { diameter }, Some((lo, hi))) => {
            if setup.copies.is_some() {
                return Err("a round stock holds one part: copies need a box stock".into());
            }
            let (c, r) = ((lo + hi) * 0.5, size("diameter", diameter)? * 0.5);
            round = Some((Vec2::new(c.x, c.y), r));
            Some((Vec3::new(c.x - r, c.y - r, lo.z - bottom), Vec3::new(c.x + r, c.y + r, hi.z + top)))
        }
    };
    let boxed = |of: &BoxOf| {
        match of {
            BoxOf::Stock => stock,
            BoxOf::Part => part,
        }
        .ok_or("the setup's origin is on the box round the bodies, and there is no body")
    };
    let at = |a: &Anchor, lo: f64, hi: f64| match a {
        Anchor::Min => lo,
        Anchor::Center => (lo + hi) * 0.5,
        Anchor::Max => hi,
    };
    let o = match &setup.origin {
        SetupOrigin::Point { at } => *at,
        SetupOrigin::Box { of, x: ax, y: ay, z: az } => {
            let (lo, hi) = boxed(of)?;
            x * at(ax, lo.x, hi.x) + y * at(ay, lo.y, hi.y) + z * at(az, lo.z, hi.z)
        }
        SetupOrigin::Edge { body, edge, at: near, on, z: oz } => {
            let b = state
                .bodies
                .iter()
                .find(|b| b.name.eq_ignore_ascii_case(body))
                .ok_or_else(|| format!("the setup's origin is on the body `{body}`, which the design no longer has"))?;
            let p = edge_origin(b, edge, *near, *on)?;
            let height = match oz {
                OriginZ::Point => p.dot(z),
                OriginZ::Box { of, z: az } => {
                    let (lo, hi) = boxed(of)?;
                    at(az, lo.z, hi.z)
                }
            };
            x * p.dot(x) + y * p.dot(y) + z * height
        }
    };
    let shift = Vec3::new(o.dot(x), o.dot(y), o.dot(z));
    let moved = |b: Box3| (b.0 - shift, b.1 - shift);
    // The stock reaches over every copy.
    let copies = setup.copies.map_or_else(|| vec![(0.0, 0.0)], |c| c.offsets());
    let (far_x, far_y) = copies.iter().fold((0.0f64, 0.0f64), |m, c| (m.0.max(c.0), m.1.max(c.1)));
    let stock = stock.map(moved).map(|(lo, hi)| (lo, hi + Vec3::new(far_x, far_y, 0.0)));
    let round = round.map(|(c, r)| (c - Vec2::new(shift.x, shift.y), r));
    Ok(Resolved { setup: setup.id, frame: Frame { o, x, y, z }, part: part.map(moved), stock, round, bodies: setup.bodies.clone(), copies })
}

/// How many tools a setup's program loads: one for its first operation, and one each time the
/// next operation's tool is another.
fn tool_changes(setup: &Setup) -> usize {
    let mut last = None;
    setup.operations.iter().filter(|op| last.replace(op.tool) != Some(op.tool)).count()
}

/// Groups a setup's operations by tool (see the command's description).
fn setup_order(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.order";
    let si = setup_index(s, p, cmd)?;
    let setup = s.doc.manufacture.setups.get(si).ok_or_else(|| bad(cmd, "no such setup"))?;
    let before = tool_changes(setup);
    let mut tools: Vec<u32> = Vec::new();
    for op in &setup.operations {
        if !tools.contains(&op.tool) {
            tools.push(op.tool);
        }
    }
    let mut ordered = setup.operations.clone();
    // Stable: each tool's operations keep their order.
    ordered.sort_by_key(|op| tools.iter().position(|t| *t == op.tool).unwrap_or(usize::MAX));
    let setup = s.doc_mut().manufacture.setups.get_mut(si).ok_or_else(|| bad(cmd, "no such setup"))?;
    setup.operations = ordered;
    let after = tool_changes(setup);
    Ok(json!({ "order": setup.operations.iter().map(|o| o.name.clone()).collect::<Vec<_>>(), "tool_changes": [before, after] }))
}

/// What a template file says it is.
const TEMPLATE_FORMAT: &str = "solidcraft-cam-template";

/// A setup's operations as a template: each without its id, its sketch by name, and the tools
/// they cut with.
fn template_save(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.template.save";
    let setup = setup(s, p, cmd)?;
    let sketch_name = |id: u64| s.doc.features.iter().find(|f| f.id == id).map(|f| f.name.clone());
    let mut ops = Vec::new();
    let mut tools: Vec<Tool> = Vec::new();
    for op in &setup.operations {
        let mut v = json!(op);
        if let Some(o) = v.as_object_mut() {
            o.remove("id");
        }
        if let CamGeometry::Sketch { sketch, .. } = &op.geometry {
            v["sketch_name"] = json!(sketch_name(*sketch));
        }
        ops.push(v);
        if let Some(t) = s.doc.manufacture.tool(op.tool)
            && !tools.iter().any(|x| x.number == t.number)
        {
            tools.push(t.clone());
        }
    }
    let template = json!({ "format": TEMPLATE_FORMAT, "version": 1, "operations": ops, "tools": tools });
    let mut out = json!({ "operations": setup.operations.len() });
    match str_(p, "path") {
        Some(path) => {
            let text = serde_json::to_string_pretty(&template).map_err(|e| bad(cmd, e.to_string()))?;
            solidcraft_io::vfs::write(path, text.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            out["path"] = json!(path);
        }
        None => out["template"] = template,
    }
    Ok(out)
}

/// Adds a template's operations to a setup, after those it has.
fn template_apply(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.template.apply";
    let si = setup_index(s, p, cmd)?;
    let template = match (str_(p, "path"), p.get("template")) {
        (Some(path), _) => {
            let bytes = solidcraft_io::vfs::read(path, 16 << 20).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            serde_json::from_slice::<Value>(&bytes).map_err(|e| bad(cmd, format!("{path}: not a template ({e})")))?
        }
        (None, Some(t)) => t.clone(),
        (None, None) => return Err(bad(cmd, "give `path` or `template`")),
    };
    if template["format"] != TEMPLATE_FORMAT {
        return Err(bad(cmd, "not a SolidCraft operation template"));
    }
    let items = template["operations"].as_array().cloned().unwrap_or_default();
    let have = s.doc.manufacture.setups.get(si).map_or(0, |x| x.operations.len());
    if items.is_empty() || have + items.len() > MAX_OPERATIONS {
        return Err(bad(cmd, format!("a template adds 1 to {} operations to this setup", MAX_OPERATIONS.saturating_sub(have))));
    }
    // Tools the design does not have yet, from the template.
    for t in template["tools"].as_array().into_iter().flatten().take(MAX_TOOLS) {
        let tool: Tool = serde_json::from_value(t.clone()).map_err(|e| bad(cmd, format!("a tool of the template: {e}")))?;
        tool.validate().map_err(|e| bad(cmd, format!("a tool of the template: {e}")))?;
        let m = &mut s.doc_mut().manufacture;
        if m.tool(tool.number).is_none() && m.tools.len() < MAX_TOOLS {
            m.tools.push(tool);
        }
    }
    let mut added = Vec::new();
    for item in items {
        let mut v = item.clone();
        let sketch = v.get("sketch_name").and_then(Value::as_str).map(str::to_string);
        if let Some(o) = v.as_object_mut() {
            o.remove("sketch_name");
            o.insert("id".into(), json!(0));
        }
        let mut op: CamOperation = serde_json::from_value(v).map_err(|e| bad(cmd, format!("an operation of the template: {e}")))?;
        // This design's sketch of that name (none: an id no feature has, which the operation reports).
        if let (CamGeometry::Sketch { sketch: id, .. }, Some(name)) = (&mut op.geometry, sketch) {
            *id = s.doc.features.iter().find(|f| matches!(f.kind, solidcraft_doc::FeatureKind::Sketch { .. }) && f.name == name).map_or(0, |f| f.id);
        }
        let names: Vec<String> = s.doc.manufacture.setups.get(si).map(|x| x.operations.iter().map(|o| o.name.clone()).collect()).unwrap_or_default();
        let base: String = op.name.trim_end_matches(|c: char| c.is_ascii_digit()).to_string();
        if names.iter().any(|n| n.eq_ignore_ascii_case(&op.name)) {
            op.name = (1..).map(|k| format!("{base}{k}")).find(|n| !names.iter().any(|x| x.eq_ignore_ascii_case(n))).unwrap_or_default();
        }
        op.id = next_id(s);
        added.push(op.name.clone());
        if let Some(setup) = s.doc_mut().manufacture.setups.get_mut(si) {
            setup.operations.push(op);
        }
    }
    Ok(json!({ "added": added }))
}

fn setup_remove(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup.remove";
    if key(p, "setup").is_none() {
        return Err(bad(cmd, "`setup` is required"));
    }
    let i = setup_index(s, p, cmd)?;
    let doc = s.doc_mut();
    let removed = doc.manufacture.setups.remove(i);
    // Setups that took their stock from it start from their own stock box again.
    for later in &mut doc.manufacture.setups {
        if later.stock_from == Some(removed.id) {
            later.stock_from = None;
        }
    }
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
            // A round bar's own circle: no cuts in the air round it.
            let outer = match (source, at.round) {
                (CamGeometry::Stock, Some((c, r))) => Loop2::circle(c, r),
                _ => Loop2::polygon(&[Vec2::new(lo.x, lo.y), Vec2::new(hi.x, lo.y), Vec2::new(hi.x, hi.y), Vec2::new(lo.x, hi.y)]),
            };
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
    for body in state.bodies.iter().filter(|b| at.is_part(&b.name)) {
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
    /// Roughing down to the model in 3D.
    Rough3d,
}

impl OpShape {
    fn of(kind: &CamOperationKind) -> Self {
        match kind {
            CamOperationKind::Face { .. } => OpShape::Face,
            CamOperationKind::Contour { .. } => OpShape::Contour,
            // Both start at the top edge and find their depth themselves.
            CamOperationKind::Chamfer { .. } | CamOperationKind::VCarve { .. } => OpShape::Chamfer,
            CamOperationKind::Adaptive { model: true, .. } | CamOperationKind::Parallel3d { .. } | CamOperationKind::ZLevel3d { .. } => {
                OpShape::Rough3d
            }
            CamOperationKind::Pocket { .. } | CamOperationKind::Adaptive { .. } => OpShape::Pocket,
            // Holes, as drilled: their heights are the same.
            CamOperationKind::Drill { .. } | CamOperationKind::Bore { .. } | CamOperationKind::Thread { .. } => OpShape::Drill,
        }
    }
}

/// [`auto_heights`], and for roughing in 3D the deepest floor of the model seen from the tool
/// inside the geometry's box: deep enough for every floor, never down beside the part to the
/// stock's bottom (cutting it loose).
fn heights_found(s: &Session, at: &Resolved, g: &Geometry, shape: OpShape, source: &CamGeometry) -> (Option<f64>, Option<f64>) {
    let (top, bottom) = auto_heights(at, g, shape, source);
    if !matches!(shape, OpShape::Rough3d) {
        return (top, bottom);
    }
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for q in g.regions.iter().flat_map(|r| r.outer.polyline(TOLERANCE)) {
        (lo, hi) = (Vec2::new(lo.x.min(q.x), lo.y.min(q.y)), Vec2::new(hi.x.max(q.x), hi.y.max(q.y)));
    }
    let floor = at.part.map_or(0.0, |(lo, _)| lo.z) - 1.0;
    let deepest = part_field(s, at, lo, hi, floor).and_then(|f| f.z.iter().copied().filter(|z| *z > f.floor).reduce(f64::min));
    (top, deepest.or(bottom))
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
    // Roughing in 3D: all the material, from the stock's top down to the model's lowest point.
    if matches!(shape, OpShape::Rough3d) {
        return (Some(stock_top.unwrap_or(g.z)), at.part.map(|(lo, _)| lo.z));
    }
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
    let (auto_top, auto_bottom) = heights_found(s, at, g, OpShape::of(&op.kind), &op.geometry);
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
            CamOperationKind::Contour { side: Side::On, .. }
            | CamOperationKind::Drill { .. }
            | CamOperationKind::Chamfer { .. }
            | CamOperationKind::VCarve { .. } => {}
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
        CamOperationKind::Contour { .. }
        | CamOperationKind::Pocket { .. }
        | CamOperationKind::Adaptive { .. }
        | CamOperationKind::Chamfer { .. }
        | CamOperationKind::VCarve { .. } => {
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
            // Facing a round bar's stock goes across its circle only.
            let round = at.round.filter(|_| op.geometry == CamGeometry::Stock);
            let params = cam::FaceParams { stepover: len("stepover", stepover)?, inside: *inside, margin, round };
            // The part seen from above round the area, so that passes below its top go round it.
            let reach = margin + tool.diameter;
            let around = Vec2::new(reach, reach);
            match part_field(s, at, min - around, max + around, cut.heights.bottom - 1.0) {
                Some(part) => cam::face_around(min, max, tool, &params, &cut, &part),
                None => cam::face(min, max, tool, &params, &cut),
            }
        }
        CamOperationKind::Contour { side, climb, stock_to_leave, ramp_angle, tabs, finish, lead, dogbones, skin, skin_last } => {
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
                lead: lead.as_ref().map(|e| len("lead", e)).transpose()?.unwrap_or(0.0),
                dogbones: *dogbones,
                skin: skin.as_ref().map(|e| len("skin", e)).transpose()?.unwrap_or(0.0),
                skin_last: *skin_last,
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
        CamOperationKind::Pocket { stepover, climb, stock_to_leave, ramp_angle, finish, rest_from, dogbones } => {
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
                dogbones: *dogbones,
            };
            cam::pocket(regions, tool, &params, &cut)
        }
        CamOperationKind::Adaptive {
            stepover_percent,
            climb,
            stock_to_leave,
            finish,
            helix_angle,
            helix_max,
            helix_min,
            rest_from,
            feed_up,
            model,
            step_up,
        } => {
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
            if *model {
                let stock = at.stock.ok_or_else(|| fail("there is no body to take the stock from".into()))?;
                // Over the whole stock every copy of the part is in the way; inside a sketch the
                // path is cut on each copy, as other operations are.
                let one = part_triangles(s, at);
                if one.is_empty() {
                    return Err(fail("there is no model to rough down to".into()));
                }
                let all_copies = matches!(op.geometry, CamGeometry::Stock | CamGeometry::Part);
                let shifts: &[(f64, f64)] = if all_copies { &at.copies } else { &[(0.0, 0.0)] };
                let part: Vec<[Vec3; 3]> =
                    shifts.iter().flat_map(|&(dx, dy)| one.iter().map(move |t| t.map(|v| v + Vec3::new(dx, dy, 0.0)))).collect();
                let step_up = match step_up {
                    Some(e) => len("step_up", e)?,
                    None => cut.stepdown * 0.25,
                };
                let key = rough_key(s, op);
                if let Some(path) = key.and_then(|k| s.cam_path_cache.lock().ok()?.get(&k).cloned()) {
                    Ok(path)
                } else {
                    let left = stock_before(s, at, op, tool);
                    let job = cam::Rough3d {
                        boundary: regions,
                        part: &part,
                        stock,
                        left: left.as_ref(),
                        step_up,
                        holder: tool.holder_diameter.zip(tool.length),
                    };
                    let made = cam::adaptive_3d(&job, tool, &params, &cut);
                    if let (Some(k), Ok(path), Ok(mut cache)) = (key, &made, s.cam_path_cache.lock()) {
                        cache.insert(k, path.clone());
                    }
                    made
                }
            } else {
                cam::adaptive(regions, tool, &params, &cut)
            }
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
        CamOperationKind::Parallel3d { stepover, angle, stock_to_leave, zigzag, rest } => {
            let one = part_triangles(s, at);
            if one.is_empty() {
                return Err(fail("there is no model to finish".into()));
            }
            // Over the whole box every copy of the part is in the way, as for roughing.
            let all_copies = matches!(op.geometry, CamGeometry::Stock | CamGeometry::Part);
            let shifts: &[(f64, f64)] = if all_copies { &at.copies } else { &[(0.0, 0.0)] };
            let part: Vec<[Vec3; 3]> = shifts.iter().flat_map(|&(dx, dy)| one.iter().map(move |t| t.map(|v| v + Vec3::new(dx, dy, 0.0)))).collect();
            let key = rough_key(s, op);
            if let Some(path) = key.and_then(|k| s.cam_path_cache.lock().ok()?.get(&k).cloned()) {
                Ok(path)
            } else {
                // What earlier operations left: rest machining goes by it, any finishing comes down fast to it.
                let left = match rest_stock(s, at, op, tool) {
                    Some(left) => Some(left),
                    None if *rest => return Err(fail(NOTHING_BEFORE.into())),
                    None => None,
                };
                let job = cam::Finish3d {
                    boundary: regions,
                    part: &part,
                    stepover: len("stepover", stepover)?,
                    angle: *angle,
                    stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                    tolerance: TOLERANCE,
                    zigzag: *zigzag,
                    left: left.as_ref(),
                    rest: *rest,
                };
                let made = cam::parallel_3d(&job, tool, &cut);
                if let (Some(k), Ok(path), Ok(mut cache)) = (key, &made, s.cam_path_cache.lock()) {
                    cache.insert(k, path.clone());
                }
                made
            }
        }
        CamOperationKind::ZLevel3d { step, stock_to_leave, steep, climb, rest } => {
            let one = part_triangles(s, at);
            if one.is_empty() {
                return Err(fail("there is no model to finish".into()));
            }
            let all_copies = matches!(op.geometry, CamGeometry::Stock | CamGeometry::Part);
            let shifts: &[(f64, f64)] = if all_copies { &at.copies } else { &[(0.0, 0.0)] };
            let part: Vec<[Vec3; 3]> = shifts.iter().flat_map(|&(dx, dy)| one.iter().map(move |t| t.map(|v| v + Vec3::new(dx, dy, 0.0)))).collect();
            let key = rough_key(s, op);
            if let Some(path) = key.and_then(|k| s.cam_path_cache.lock().ok()?.get(&k).cloned()) {
                Ok(path)
            } else {
                // What earlier operations left: rest machining goes by it, any finishing comes down fast to it.
                let left = match rest_stock(s, at, op, tool) {
                    Some(left) => Some(left),
                    None if *rest => return Err(fail(NOTHING_BEFORE.into())),
                    None => None,
                };
                let job = cam::ZLevel3d {
                    boundary: regions,
                    part: &part,
                    step: len("step", step)?,
                    stock_to_leave: len("stock_to_leave", stock_to_leave)?,
                    tolerance: TOLERANCE,
                    steep: *steep,
                    climb: *climb,
                    left: left.as_ref(),
                    rest: *rest,
                };
                let made = cam::zlevel_3d(&job, tool, &cut);
                if let (Some(k), Ok(path), Ok(mut cache)) = (key, &made, s.cam_path_cache.lock()) {
                    cache.insert(k, path.clone());
                }
                made
            }
        }
        CamOperationKind::VCarve { stepover, max_depth, climb } => {
            let params = VCarveParams {
                stepover: len("stepover", stepover)?,
                max_depth: max_depth.as_ref().map(|e| len("max_depth", e)).transpose()?,
                climb: *climb,
                tolerance: TOLERANCE,
            };
            cam::vcarve(regions, tool, &params, &cut)
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
        CamOperationKind::Thread { pitch, diameter, internal, climb, passes, left_hand } => {
            let pitch = len("pitch", pitch)?;
            // Round profiles of a sketch; a picked face's round holes and picked hole walls.
            let holes: Vec<(Vec2, f64)> = if picked {
                regions.iter().flat_map(|r| r.holes.iter().filter_map(round_hole)).chain(g.bores.iter().copied()).collect()
            } else {
                let round: Vec<_> = regions.iter().filter_map(|r| round_hole(&r.outer)).collect();
                if round.len() < regions.len() {
                    return Err(fail("only round profiles can be threaded".into()));
                }
                round
            };
            let Some(&(_, drawn)) = holes.first() else { return Err(fail("there is no round hole or boss to thread".into())) };
            // A hole is drawn at its core diameter, a boss at its major one (ISO metric).
            let major = match diameter {
                Some(e) => len("diameter", e)?,
                None if *internal => 2.0 * drawn + 1.0825 * pitch,
                None => 2.0 * drawn,
            };
            let params = cam::ThreadParams { pitch, diameter: major, internal: *internal, climb: *climb, passes: *passes, left_hand: *left_hand };
            let centres: Vec<Vec2> = holes.iter().map(|(c, _)| *c).collect();
            cam::thread_mill(&centres, tool, &params, &cut)
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
    // Facing the stock covers every copy already: the stock reaches over them.
    let copies: &[(f64, f64)] = if matches!(op.geometry, CamGeometry::Stock) { &[(0.0, 0.0)] } else { &at.copies };
    path.map(|p| copied(p, copies)).map_err(|e| fail(e.to_string()))
}

/// Why 3D rest finishing has nothing to go by.
const NOTHING_BEFORE: &str = "rest finishing goes by what earlier operations left: put it after them in the setup";

/// What the operations before `op` left of the stock, on columns fine enough to see what a
/// finishing pass leaves (a tenth of `tool`'s radius, coarser on a large stock); `None` when
/// nothing cut it before.
fn rest_stock(s: &Session, at: &Resolved, op: &CamOperation, tool: &Tool) -> Option<cam::HeightField> {
    // An operation being made is not in its setup yet: it comes after all there is.
    let (setup, index) =
        s.doc.manufacture.setups.iter().find_map(|x| x.operations.iter().position(|o| o.id == op.id).map(|k| (x, k))).or_else(|| {
            let setup = s.doc.manufacture.setups.iter().find(|x| x.id == at.setup)?;
            Some((setup, setup.operations.len()))
        })?;
    if index == 0 && setup.stock_from.is_none() {
        return None;
    }
    let (lo, hi) = at.stock?;
    let cell = (tool.radius() / 10.0).max((hi.x - lo.x).max(hi.y - lo.y) / 1500.0);
    let (mut field, _) = start_stock(s, setup, at, &json!({ "cell": cell }), "cam.op.parallel3d").ok()?;
    for earlier in setup.operations.iter().take(index) {
        let (Ok(path), Some(t)) = (build_in(s, at, earlier), s.doc.manufacture.tool(earlier.tool)) else { continue };
        cam::simulate(&mut field, &path, t).ok()?;
    }
    Some(field)
}

/// 3D roughing toolpaths by [`rough_key`], the last few.
#[derive(Default)]
pub(crate) struct PathCache {
    paths: Vec<(u64, Toolpath)>,
}

/// How many 3D roughing toolpaths are kept.
const KEPT_PATHS: usize = 8;

impl PathCache {
    fn get(&self, key: &u64) -> Option<&Toolpath> {
        self.paths.iter().find(|(k, _)| k == key).map(|(_, p)| p)
    }

    fn insert(&mut self, key: u64, path: Toolpath) {
        self.paths.retain(|(k, _)| *k != key);
        if self.paths.len() >= KEPT_PATHS {
            self.paths.remove(0);
        }
        self.paths.push((key, path));
    }
}

/// A fingerprint of everything a 3D roughing toolpath of `op` is made from: the design without
/// its manufacturing (the model and its parameters), every setup up to and with `op`'s (their
/// operations up to `op`: what they cut is its stock), and the tools.
fn rough_key(s: &Session, op: &CamOperation) -> Option<u64> {
    use std::hash::{Hash, Hasher};
    let m = &s.doc.manufacture;
    let at = m.setups.iter().position(|x| x.operations.iter().any(|o| o.id == op.id))?;
    let mut setups: Vec<Value> = Vec::new();
    for (k, setup) in m.setups.iter().take(at + 1).enumerate() {
        let mut v = serde_json::to_value(setup).ok()?;
        if k == at {
            let upto = setup.operations.iter().position(|o| o.id == op.id)?;
            v["operations"] = serde_json::to_value(setup.operations.get(..=upto)?).ok()?;
        }
        setups.push(v);
    }
    let mut design = (*s.doc).clone();
    design.manufacture = Default::default();
    let mut h = std::collections::hash_map::DefaultHasher::new();
    serde_json::to_vec(&design).ok()?.hash(&mut h);
    serde_json::to_vec(&(setups, &m.tools)).ok()?.hash(&mut h);
    Some(h.finish())
}

/// What the operations before `op` in its setup (and the setup it takes its stock from) leave
/// of the stock, on columns a fifth of `tool`'s radius wide (coarser on a large stock); `None`
/// when nothing cut it before.
fn stock_before(s: &Session, at: &Resolved, op: &CamOperation, tool: &Tool) -> Option<cam::HeightField> {
    // An operation being made is not in its setup yet: it comes after all there is.
    let (setup, index) =
        s.doc.manufacture.setups.iter().find_map(|x| x.operations.iter().position(|o| o.id == op.id).map(|k| (x, k))).or_else(|| {
            let setup = s.doc.manufacture.setups.iter().find(|x| x.id == at.setup)?;
            Some((setup, setup.operations.len()))
        })?;
    if index == 0 && setup.stock_from.is_none() {
        return None;
    }
    let (lo, hi) = at.stock?;
    let cell = (tool.radius() / 5.0).max((hi.x - lo.x).max(hi.y - lo.y) / 600.0);
    let (mut field, _) = start_stock(s, setup, at, &json!({ "cell": cell }), "cam.op.adaptive3d").ok()?;
    for earlier in setup.operations.iter().take(index) {
        let (Ok(path), Some(t)) = (build_in(s, at, earlier), s.doc.manufacture.tool(earlier.tool)) else { continue };
        cam::simulate(&mut field, &path, t).ok()?;
    }
    Some(field)
}

/// A toolpath cut on every copy, one after the other, the tool up at its highest between them.
fn copied(path: Toolpath, copies: &[(f64, f64)]) -> Toolpath {
    if copies.len() <= 1 {
        return path;
    }
    let up = path.moves.iter().filter_map(|m| m.target()).map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    let mut moves = Vec::with_capacity(path.moves.len() * copies.len() + copies.len());
    for &(dx, dy) in copies {
        if let Some(last) = moves.iter().rev().find_map(|m: &cam::Move| m.target()) {
            moves.push(cam::Move::Rapid { to: Vec3::new(last.x, last.y, up) });
        }
        moves.extend(path.moves.iter().map(|m| m.shifted(dx, dy)));
    }
    Toolpath { moves }
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

/// Where an operation cuts, as `cmd` names it: a sketch, picked faces, or (facing and 3D
/// roughing) a boundary.
fn geometry_param(s: &Session, p: &Value, cmd: &str) -> Result<CamGeometry> {
    Ok(match (p.get("faces"), p.get("sketch")) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give either `sketch` or `faces`, not both")),
        (Some(faces), None) => CamGeometry::Faces { faces: faces_param(s, faces, cmd)? },
        (None, None) if matches!(cmd, "cam.op.face" | "cam.op.adaptive3d") => boundary_param(p, cmd)?,
        // Finishing covers the model's box unless told the stock's.
        (None, None) if matches!(cmd, "cam.op.parallel3d" | "cam.op.zlevel3d") => match p.get("boundary") {
            None => CamGeometry::Part,
            Some(_) => boundary_param(p, cmd)?,
        },
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
        "cam.op.chamfer" | "cam.op.vcarve" => OpShape::Chamfer,
        "cam.op.pocket" | "cam.op.adaptive" => OpShape::Pocket,
        "cam.op.adaptive3d" | "cam.op.parallel3d" | "cam.op.zlevel3d" => OpShape::Rough3d,
        "cam.op.drill" | "cam.op.bore" => OpShape::Drill,
        _ => return Err(bad(cmd, "not an operation")),
    };
    let setup = setup(s, p, cmd)?;
    let at = resolve(s, setup).map_err(|e| bad(cmd, format!("setup `{}`: {e}", setup.name)))?;
    let source = geometry_param(s, p, cmd)?;
    let g = geometry(s, &at, &source).map_err(|e| bad(cmd, e))?;
    Ok(heights_found(s, &at, &g, shape, &source))
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

/// The work origin the selection offers: the first selected vertex or body edge, as `origin`'s
/// `{edge, body}` takes it, and which of its points fits (a vertex's corner, a round edge's
/// centre, else its middle).
pub fn cam_origin_pick(s: &Session, selection: &[crate::Sel]) -> Option<(String, Vec3, EdgePoint)> {
    let state = s.model.state();
    selection.iter().take(MAX_SELECTED).find_map(|item| match item {
        crate::Sel::Vertex { body, point } => Some((body.clone(), *point, EdgePoint::Vertex)),
        crate::Sel::Edge { body, index, point } => {
            let b = state.body(body).filter(|b| !b.body.is_mesh())?;
            let round = super::measure_sel::edge_centre(b, *index).is_some();
            Some((body.clone(), *point, if round { EdgePoint::Center } else { EdgePoint::Middle }))
        }
        _ => None,
    })
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
    let positive = |k: &str| num(p, k).filter(|v| *v > 0.0 && *v <= 1.0e6);
    // Its cutting data in the stock's material, where it has some; a speed given here is the one a
    // chip load feeds at. A surface speed's speed stays within the machine's spindle.
    let mut cutting = stored.in_material(setup.material.as_deref());
    if let Some(rpm) = positive("spindle_rpm") {
        (cutting.spindle_rpm, cutting.surface_speed) = (Some(rpm), None);
    }
    let max_rpm = match p.get("max_spindle_rpm") {
        None | Some(Value::Null) => None,
        Some(_) => Some(positive("max_spindle_rpm").ok_or_else(|| bad(cmd, "`max_spindle_rpm` must be above 0"))?),
    };
    let tool = cutting.with_speeds(max_rpm);
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
/// An optional length expression `k` (`null`: none).
fn optional_length(s: &Session, p: &Value, k: &str, cmd: &str) -> Result<Option<Expr>> {
    match expr(p, k) {
        Some(e) => length_expr(s, cmd, k, e).map(Some),
        None => Ok(None),
    }
}

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
    let lead = optional_length(s, p, "lead", cmd)?;
    let skin = optional_length(s, p, "skin", cmd)?;
    let kind = CamOperationKind::Contour {
        side,
        climb: bool_(p, "climb").unwrap_or(true),
        stock_to_leave: stock_to_leave(p),
        ramp_angle: ramp_angle(p),
        tabs,
        finish,
        lead,
        dogbones: bool_(p, "dogbones").unwrap_or(false),
        skin,
        skin_last: bool_(p, "skin_last").unwrap_or(false),
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
        dogbones: bool_(p, "dogbones").unwrap_or(false),
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
    let kind = adaptive_kind(s, p, "cam.op.adaptive", false)?;
    add_op(s, p, "cam.op.adaptive", "Adaptive", |_| kind)
}

fn op_adaptive3d(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.adaptive3d";
    for k in ["finish", "rest_from"] {
        if p.get(k).is_some_and(|v| !v.is_null()) {
            return Err(bad(cmd, format!("`{k}` is not offered in 3D roughing yet")));
        }
    }
    let kind = adaptive_kind(s, p, cmd, true)?;
    add_op(s, p, cmd, "Adaptive3D", |_| kind)
}

/// An adaptive operation's kind from `p`; `model`: roughing down to the model in 3D.
fn adaptive_kind(s: &Session, p: &Value, cmd: &str, model: bool) -> Result<CamOperationKind> {
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
    Ok(CamOperationKind::Adaptive {
        stepover_percent: number_in(p, "stepover_percent", 2.0, 50.0, 20.0, cmd)?,
        climb: bool_(p, "climb").unwrap_or(true),
        stock_to_leave: stock_to_leave(p),
        finish: finish_param(s, p, cmd)?,
        helix_angle,
        helix_max,
        helix_min,
        rest_from: rest_param(s, p, cmd)?,
        feed_up: feed_up_param(p, cmd)?,
        model,
        step_up: if model { step_up_param(s, p, cmd)? } else { None },
    })
}

/// How far apart 3D roughing's levels in between on slopes are at most: a length of 0 or more
/// (0: none; left out or null: a quarter of the stepdown).
fn step_up_param(s: &Session, p: &Value, cmd: &str) -> Result<Option<Expr>> {
    match p.get("step_up") {
        None | Some(Value::Null) => Ok(None),
        Some(_) => {
            let e = req_expr(cmd, p, "step_up")?;
            if length(s, cmd, "step_up", &e)? >= 0.0 { Ok(Some(e)) } else { Err(bad(cmd, "`step_up` must be 0 or more")) }
        }
    }
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

fn op_vcarve(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.vcarve";
    let max_depth = optional_length(s, p, "max_depth", cmd)?;
    let stepover = expr(p, "stepover");
    let climb = bool_(p, "climb").unwrap_or(true);
    add_op(s, p, cmd, "VCarve", |tool| CamOperationKind::VCarve {
        stepover: stepover.unwrap_or_else(|| mm((tool.diameter * 0.02).max(0.05))),
        max_depth,
        climb,
    })
}

fn op_parallel3d(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.parallel3d";
    let stepover = expr(p, "stepover");
    if let Some(e) = &stepover
        && !(length(s, cmd, "stepover", e)? > 0.0)
    {
        return Err(bad(cmd, "`stepover` must be above 0"));
    }
    let angle = match p.get("angle") {
        None => 0.0,
        Some(_) => num(p, "angle").filter(|a| a.is_finite() && a.abs() <= 360.0).ok_or_else(|| bad(cmd, "`angle` must be -360 to 360 degrees"))?,
    };
    let zigzag = bool_(p, "zigzag").unwrap_or(true);
    let rest = bool_(p, "rest").unwrap_or(false);
    let stock_to_leave = stock_to_leave(p);
    add_op(s, p, cmd, "Parallel3D", |tool| CamOperationKind::Parallel3d {
        stepover: stepover.unwrap_or_else(|| mm(tool.diameter * 0.1)),
        angle,
        stock_to_leave,
        zigzag,
        rest,
    })
}

fn op_zlevel3d(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.zlevel3d";
    let step = expr(p, "step");
    if let Some(e) = &step
        && !(length(s, cmd, "step", e)? > 0.0)
    {
        return Err(bad(cmd, "`step` must be above 0"));
    }
    let steep = steep_param(p, cmd)?.unwrap_or(30.0);
    let (climb, stock_to_leave, rest) = (bool_(p, "climb").unwrap_or(true), stock_to_leave(p), bool_(p, "rest").unwrap_or(false));
    add_op(s, p, cmd, "ZLevel3D", |tool| CamOperationKind::ZLevel3d {
        step: step.unwrap_or_else(|| mm(tool.diameter / 12.0)),
        stock_to_leave,
        steep,
        climb,
        rest,
    })
}

/// How steep a wall Z-level finishing takes at least: degrees from 0 to under 90 (`None` when left out).
fn steep_param(p: &Value, cmd: &str) -> Result<Option<f64>> {
    match p.get("steep") {
        None => Ok(None),
        Some(_) => num(p, "steep").filter(|a| (0.0..90.0).contains(a)).map(Some).ok_or_else(|| bad(cmd, "`steep` must be 0 to under 90 degrees")),
    }
}

fn op_thread(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.op.thread";
    let pitch = expr(p, "pitch").ok_or_else(|| bad(cmd, "`pitch` is required: the thread's pitch (1.25 mm for M8, say)"))?;
    let diameter = optional_length(s, p, "diameter", cmd)?;
    let internal = !matches!(str_(p, "side"), Some("outside"));
    let passes = match p.get("thread_passes") {
        None => 1,
        Some(v) => v.as_u64().filter(|n| (1..=20).contains(n)).ok_or_else(|| bad(cmd, "`thread_passes` must be 1 to 20"))? as u32,
    };
    let climb = bool_(p, "climb").unwrap_or(true);
    let left_hand = hand_param(p, cmd)?.unwrap_or(false);
    add_op(s, p, cmd, "Thread", |_| CamOperationKind::Thread { pitch, diameter, internal, climb, passes, left_hand })
}

/// `hand`: right or left (a thread's).
fn hand_param(p: &Value, cmd: &str) -> Result<Option<bool>> {
    match (p.get("hand"), str_(p, "hand")) {
        (None, _) => Ok(None),
        (_, Some("right")) => Ok(Some(false)),
        (_, Some("left")) => Ok(Some(true)),
        _ => Err(bad(cmd, "`hand` must be right or left")),
    }
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

/// The operations a simulation cuts and shows (`ops`, default every one) and those it cuts
/// first, unseen, for the stock they leave (`first`): positions in the setup's order. A list
/// is of ids or names, at most one entry per operation of the setup.
fn simulated_ops(setup: &Setup, p: &Value, cmd: &str) -> Result<(Vec<usize>, Vec<usize>)> {
    let list = |k: &str| -> Result<Option<Vec<usize>>> {
        let Some(v) = p.get(k).filter(|v| !v.is_null()) else { return Ok(None) };
        let items = v.as_array().ok_or_else(|| bad(cmd, format!("`{k}` must be a list of operations (ids or names)")))?;
        if items.len() > setup.operations.len() {
            return Err(bad(cmd, format!("`{k}` names more operations than the setup has")));
        }
        let mut at = Vec::new();
        for item in items {
            let name = match item {
                Value::Number(n) => n.to_string(),
                Value::String(t) => t.trim().to_string(),
                _ => return Err(bad(cmd, format!("`{k}` must be a list of operations (ids or names)"))),
            };
            at.push(op_position(setup, &name).ok_or_else(|| bad(cmd, format!("no operation `{name}`")))?);
        }
        // In the setup's order, each once.
        at.sort_unstable();
        at.dedup();
        Ok(Some(at))
    };
    let shown = list("ops")?.unwrap_or_else(|| (0..setup.operations.len()).collect());
    if shown.is_empty() {
        return Err(bad(cmd, "`ops` names no operation to simulate"));
    }
    let first: Vec<usize> = list("first")?.unwrap_or_default().into_iter().filter(|i| !shown.contains(i)).collect();
    Ok((shown, first))
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
                if !matches!(
                    op.kind,
                    CamOperationKind::Face { .. }
                        | CamOperationKind::Adaptive { model: true, .. }
                        | CamOperationKind::Parallel3d { .. }
                        | CamOperationKind::ZLevel3d { .. }
                ) {
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
        "max_depth",
        "diameter",
        "thread_passes",
        "hand",
        "lead",
        "dogbones",
        "skin",
        "skin_last",
        "rest_from",
        "stepover_percent",
        "helix_angle",
        "helix_max",
        "helix_min",
        "feed_up",
        "step_up",
        "angle",
        "zigzag",
        "step",
        "steep",
        "rest",
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
            (CamOperationKind::VCarve { max_depth, .. }, "max_depth") => *max_depth = optional_length(s, p, k, cmd)?,
            (CamOperationKind::Thread { pitch, .. }, "pitch") => *pitch = checked(k, Kind::Length)?,
            (CamOperationKind::Thread { diameter, .. }, "diameter") => *diameter = optional_length(s, p, k, cmd)?,
            (CamOperationKind::Thread { left_hand, .. }, "hand") => *left_hand = hand_param(p, cmd)?.unwrap_or(false),
            (CamOperationKind::Thread { internal, .. }, "side") => *internal = !matches!(str_(p, k), Some("outside")),
            (CamOperationKind::Thread { passes, .. }, "thread_passes") => {
                *passes =
                    p.get(k).and_then(Value::as_u64).filter(|n| (1..=20).contains(n)).ok_or_else(|| bad(cmd, "`thread_passes` must be 1 to 20"))?
                        as u32
            }
            (
                CamOperationKind::Face { stepover, .. }
                | CamOperationKind::Pocket { stepover, .. }
                | CamOperationKind::VCarve { stepover, .. }
                | CamOperationKind::Parallel3d { stepover, .. },
                "stepover",
            ) => *stepover = checked(k, Kind::Length)?,
            (CamOperationKind::Parallel3d { angle, .. }, "angle") => {
                *angle = num(p, k).filter(|a| a.is_finite() && a.abs() <= 360.0).ok_or_else(|| bad(cmd, "`angle` must be -360 to 360 degrees"))?
            }
            (CamOperationKind::Parallel3d { rest, .. } | CamOperationKind::ZLevel3d { rest, .. }, "rest") => {
                *rest = bool_(p, k).ok_or_else(|| bad(cmd, "`rest` must be true or false"))?
            }
            (CamOperationKind::ZLevel3d { step, .. }, "step") => *step = checked(k, Kind::Length)?,
            (CamOperationKind::ZLevel3d { steep, .. }, "steep") => *steep = steep_param(p, cmd)?.unwrap_or(30.0),
            (CamOperationKind::Parallel3d { zigzag, .. }, "zigzag") => {
                *zigzag = bool_(p, k).ok_or_else(|| bad(cmd, "`zigzag` must be true or false"))?
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
            (CamOperationKind::Adaptive { step_up, model: true, .. }, "step_up") => *step_up = step_up_param(s, p, cmd)?,
            (CamOperationKind::Bore { pitch, .. }, "pitch") => *pitch = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { width, .. }, "width") => *width = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { tip_offset, .. }, "tip_offset") => *tip_offset = checked(k, Kind::Length)?,
            (CamOperationKind::Chamfer { side, .. }, "side") => *side = side_param(p, cmd)?,
            (
                CamOperationKind::Contour { climb, .. }
                | CamOperationKind::Pocket { climb, .. }
                | CamOperationKind::Bore { climb, .. }
                | CamOperationKind::Chamfer { climb, .. }
                | CamOperationKind::VCarve { climb, .. }
                | CamOperationKind::Thread { climb, .. }
                | CamOperationKind::Adaptive { climb, .. }
                | CamOperationKind::ZLevel3d { climb, .. },
                "climb",
            ) => *climb = bool_(p, k).ok_or_else(|| bad(cmd, "`climb` must be true or false"))?,
            (
                CamOperationKind::Contour { stock_to_leave, .. }
                | CamOperationKind::Pocket { stock_to_leave, .. }
                | CamOperationKind::Bore { stock_to_leave, .. }
                | CamOperationKind::Adaptive { stock_to_leave, .. }
                | CamOperationKind::Parallel3d { stock_to_leave, .. }
                | CamOperationKind::ZLevel3d { stock_to_leave, .. },
                "stock_to_leave",
            ) => *stock_to_leave = checked(k, Kind::Length)?,
            (
                CamOperationKind::Contour { ramp_angle, .. }
                | CamOperationKind::Pocket { ramp_angle, .. }
                | CamOperationKind::Chamfer { ramp_angle, .. },
                "ramp_angle",
            ) => *ramp_angle = checked(k, Kind::Angle)?,
            (CamOperationKind::Contour { tabs, .. }, "tabs") => *tabs = tabs_param(s, p, cmd)?,
            (CamOperationKind::Contour { lead, .. }, "lead") => *lead = optional_length(s, p, k, cmd)?,
            (CamOperationKind::Contour { skin, .. }, "skin") => *skin = optional_length(s, p, k, cmd)?,
            (CamOperationKind::Contour { skin_last, .. }, "skin_last") => {
                *skin_last = bool_(p, k).ok_or_else(|| bad(cmd, "`skin_last` must be true or false"))?
            }
            (CamOperationKind::Contour { dogbones, .. } | CamOperationKind::Pocket { dogbones, .. }, "dogbones") => {
                *dogbones = bool_(p, k).ok_or_else(|| bad(cmd, "`dogbones` must be true or false"))?
            }
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

/// Whether `cam.list` answers at once: it was worked out for the design as it is, or every 3D
/// roughing toolpath it needs is known already (those take seconds; the rest is quick). A
/// window asks before it lists, and lists a 3D roughing job on a copy in the background first.
pub fn cam_list_ready(s: &Session) -> bool {
    if let Ok(cache) = s.cam_list_cache.lock()
        && let Some((revision, doc, _)) = cache.as_ref()
        && *revision == s.revision
        && std::sync::Arc::ptr_eq(doc, &s.doc)
    {
        return true;
    }
    let rough = s.doc.manufacture.setups.iter().flat_map(|x| &x.operations).filter(|o| slow_kind(&o.kind));
    let Ok(cache) = s.cam_path_cache.lock() else { return true };
    rough.into_iter().all(|op| rough_key(s, op).is_none_or(|k| cache.get(&k).is_some()))
}

/// Operations following the model in 3D: their toolpaths take seconds.
fn slow_kind(kind: &CamOperationKind) -> bool {
    matches!(kind, CamOperationKind::Adaptive { model: true, .. } | CamOperationKind::Parallel3d { .. } | CamOperationKind::ZLevel3d { .. })
}

/// Whether `cmd` with `p` (as [`cam_preview`] takes them) makes or changes a 3D roughing
/// operation, whose toolpath takes seconds.
pub fn cam_slow(s: &Session, cmd: &str, p: &Value) -> bool {
    match cmd {
        "cam.op.adaptive3d" | "cam.op.parallel3d" | "cam.op.zlevel3d" => true,
        "cam.op.edit" => key(p, "op").is_some_and(|k| {
            s.doc.manufacture.setups.iter().flat_map(|x| &x.operations).any(|o| slow_kind(&o.kind) && (k == o.id.to_string() || k == o.name))
        }),
        _ => false,
    }
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

/// The warning of a contour that cuts through the stock with no tabs.
const THROUGH_WITHOUT_TABS: &str = "cuts through the stock without tabs, so the part comes loose on the last pass";

fn list_now(s: &Session) -> Value {
    let m = &s.doc.manufacture;
    let setups: Vec<Value> = m
        .setups
        .iter()
        .map(|setup| {
            // The part seen from above, to tell an operation that cuts into it; the stock's bottom,
            // to tell one that cuts through it.
            let resolved = resolve(s, setup).ok();
            let part = resolved.as_ref().and_then(|at| setup_part(s, at));
            let stock_bottom = resolved.as_ref().and_then(|at| at.stock).map(|(lo, _)| lo.z);
            let stock_top = resolved.as_ref().and_then(|at| at.stock).map(|(_, hi)| hi.z);
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
                            let mut warnings = Vec::new();
                            // A chamfer cuts the part's edge on purpose: the model has no chamfer to compare with.
                            if depth > GOUGE_REPORTED && !matches!(op.kind, CamOperationKind::Chamfer { .. } | CamOperationKind::VCarve { .. }) {
                                warnings.push(format!("cuts up to {depth:.2} mm into the part"));
                            }
                            // A contour through the whole stock with nothing holding the part: it comes loose on the last pass.
                            let through = matches!((path.stats().lowest_z, stock_bottom), (Some(z), Some(bottom)) if z <= bottom + 1e-6);
                            if through && matches!(op.kind, CamOperationKind::Contour { tabs: None, .. } | CamOperationKind::Adaptive { model: true, .. }) {
                                warnings.push(THROUGH_WITHOUT_TABS.to_string());
                            }
                            if let Some(tool) = m.tool(op.tool) {
                                if let Some(clamp) = clamp_hit(&path, tool.radius(), &setup.clamps) {
                                    warnings.push(format!("runs into clamp {clamp}"));
                                } else if let (Some(d), Some(reach)) = (tool.holder_diameter, tool.length) {
                                    // The holder, from where the tool comes out of it up: as a tool that wide, that much higher.
                                    let lifted = cam::Toolpath { moves: path.moves.iter().map(|m| m.lifted(Vec3::new(0.0, 0.0, reach))).collect() };
                                    if let Some(clamp) = clamp_hit(&lifted, d * 0.5, &setup.clamps) {
                                        warnings.push(format!("the holder runs into clamp {clamp}"));
                                    }
                                }
                                // Deeper below the stock's top than the tool reaches out of its holder: the holder hits the
                                // stock. 3D roughing with a holder given keeps it clear itself.
                                let holder_kept = matches!(op.kind, CamOperationKind::Adaptive { model: true, .. }) && tool.holder_diameter.is_some();
                                if let (Some(reach), Some(lowest), Some(top), false) = (tool.length, path.stats().lowest_z, stock_top, holder_kept) {
                                    let depth = top - lowest;
                                    if depth > reach + 1e-6 {
                                        warnings.push(format!("goes {depth:.1} mm deep but the tool reaches {reach:.1} mm out of its holder"));
                                    }
                                }
                            }
                            if !warnings.is_empty() {
                                v["warning"] = json!(warnings.join("; "));
                            }
                        }
                        Err(e) => v["error"] = json!(e.to_string()),
                    }
                    v
                })
                .collect();
            let mut v = json!({"id": setup.id, "name": setup.name, "axis": setup.axis, "origin": setup.origin, "stock": setup.stock, "bodies": setup.bodies,
                "material": setup.material, "post": setup.post, "work_offset": setup.work_offset, "x_axis": setup.x_axis, "stock_from": setup.stock_from, "clamps": setup.clamps, "copies": setup.copies, "tool_changes": tool_changes(setup), "operations": ops});
            // Worked out against the model: the work frame in the world, and the boxes in work coordinates.
            match resolve(s, setup) {
                Ok(at) => {
                    let boxed = |b: Option<Box3>| b.map(|(min, max)| json!({"min": min, "max": max}));
                    v["frame"] = at.frame.json();
                    v["stock_box"] = json!(boxed(at.stock));
                    v["part_box"] = json!(boxed(at.part));
                    if let Some((c, r)) = at.round {
                        v["stock_round"] = json!({"centre": [c.x, c.y], "radius": r});
                    }
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
/// A setup's stock before its operations: its block, or (`stock_from`, without a `stock` of
/// `p`'s) what the earlier setup leaves of its own, seen along this setup's axis. Turned over,
/// what the earlier setup cut from its side is the bottom of this setup's columns, and a column
/// it cut through is empty.
fn start_stock(s: &Session, setup: &Setup, at: &Resolved, p: &Value, cmd: &str) -> Result<(cam::HeightField, Box3)> {
    let (mut field, block) = sim_block(p, cmd, at)?;
    let earlier = setup.stock_from.filter(|_| p.get("stock").is_none()).and_then(|id| {
        let setups = &s.doc.manufacture.setups;
        let here = setups.iter().position(|x| x.id == setup.id)?;
        // Only an earlier one: the chain ends.
        setups.iter().take(here).find(|x| x.id == id)
    });
    let Some(earlier) = earlier else { return Ok((field, block)) };
    if facing(earlier.axis, setup.axis).is_none() {
        return Ok((field, block));
    }
    let before = resolve(s, earlier).map_err(|e| bad(cmd, e))?;
    let (mut left, _) = start_stock(s, earlier, &before, &json!({ "cell": field.cell }), cmd)?;
    for op in &earlier.operations {
        let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        cam::simulate(&mut left, &build_in(s, &before, op)?, tool).map_err(cam_error)?;
    }
    let (floor, top) = (field.floor, block.1.z);
    let mut bottom = vec![floor; field.z.len()];
    for j in 0..field.ny {
        for i in 0..field.nx {
            let c = field.centre(i, j);
            // The column's line in the earlier setup's work coordinates.
            let there = before.frame.local(at.frame.world(Vec3::new(c.x, c.y, 0.0)));
            let q = Vec2::new(there.x, there.y);
            let (z, base) = (left.at(q), left.base_at(q));
            // Both ends of its material here; turned over, the earlier top is this bottom.
            let here = |z: f64| at.frame.local(before.frame.world(Vec3::new(there.x, there.y, z))).z.clamp(floor, top);
            let (a, b) = (here(z), here(base));
            let (lo, hi) = if z > base + 1e-9 && (a - b).abs() > 1e-9 { (a.min(b), a.max(b)) } else { (floor, floor) };
            let k = j * field.nx + i;
            if let (Some(cell), Some(under)) = (field.z.get_mut(k), bottom.get_mut(k)) {
                (*cell, *under) = (hi, lo);
            }
        }
    }
    if bottom.iter().any(|b| *b > floor + 1e-9) {
        field.bottom = bottom;
    }
    Ok((field, block))
}

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
    let mut stock = cam::HeightField::block(min, max, cell).map_err(|e| bad(cmd, e.to_string()))?;
    // A round bar holds no material outside its circle.
    if let (None, Some((c, r))) = (p.get("stock"), at.round) {
        for j in 0..stock.ny {
            for i in 0..stock.nx {
                if stock.centre(i, j).dist(c) > r
                    && let Some(z) = stock.z.get_mut(j * stock.nx + i)
                {
                    *z = stock.floor;
                }
            }
        }
    }
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
    let (mut stock, _) = start_stock(s, setup, &at, p, cmd)?;
    let (shown, first) = simulated_ops(setup, p, cmd)?;
    // The operations cut first leave the stock the played ones start from.
    for op in first.iter().filter_map(|i| setup.operations.get(*i)) {
        let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        cam::simulate(&mut stock, &build_in(s, &at, op)?, tool).map_err(cam_error)?;
    }
    let (mut paths, mut ops, mut tools) = (Vec::new(), Vec::new(), Vec::new());
    for op in shown.iter().filter_map(|i| setup.operations.get(*i)) {
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
    let meshes: Vec<_> = state.bodies.iter().filter(|b| at.is_part(&b.name)).map(|b| b.mesh()).collect();
    let (mut stock, (min, max)) = start_stock(s, setup, &at, p, cmd)?;
    let cell = stock.cell;
    let (shown, first) = simulated_ops(setup, p, cmd)?;
    let mut ops = Vec::new();
    let mut warnings: Vec<String> = Vec::new();
    let mut removed = 0.0;
    // Chamfers cut the part's edges on purpose, and the model has none: the stock without them
    // is what is checked against the part.
    let is_chamfer = |op: &CamOperation| matches!(op.kind, CamOperationKind::Chamfer { .. } | CamOperationKind::VCarve { .. });
    let cut = |i: &usize| setup.operations.get(*i);
    let mut plain = shown.iter().chain(&first).filter_map(cut).any(is_chamfer).then(|| stock.clone());
    // The operations cut first leave the stock the reported ones start from; they report nothing.
    for op in first.iter().filter_map(cut) {
        let path = build_in(s, &at, op)?;
        let tool = s.doc.manufacture.tool(op.tool).ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        cam::simulate(&mut stock, &path, tool).map_err(cam_error)?;
        if let Some(plain) = plain.as_mut().filter(|_| !is_chamfer(op)) {
            cam::simulate(plain, &path, tool).map_err(cam_error)?;
        }
    }
    for op in shown.iter().filter_map(cut) {
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
            "bottom": stock.bottom,
            "deviation": deviation,
        });
    }
    out["warnings"] = json!(warnings);
    Ok(out)
}

/// The comment lines at the top of a program, for checking it at the machine: the stock and
/// where its top and bottom are, where the work zero is, the tools and the cutting time.
fn program_notes(setup: &Setup, at: &Resolved, ops: &[cam::Operation], minutes: f64) -> Vec<String> {
    let mm = |v: f64| {
        let t = format!("{:.3}", v + 0.0);
        let t = t.trim_end_matches('0').trim_end_matches('.').to_string();
        if t == "-0" { "0".to_string() } else { t }
    };
    let mut notes = Vec::new();
    if let (Some((lo, hi)), Some((c, r))) = (at.stock, at.round) {
        notes.push(format!(
            "Stock round bar D{} x {} mm, centre X{} Y{}, top at Z{}, bottom at Z{}",
            mm(2.0 * r),
            mm(hi.z - lo.z),
            mm(c.x),
            mm(c.y),
            mm(hi.z),
            mm(lo.z)
        ));
    } else if let Some((lo, hi)) = at.stock {
        notes.push(format!(
            "Stock {} x {} x {} mm, top at Z{}, bottom at Z{}",
            mm(hi.x - lo.x),
            mm(hi.y - lo.y),
            mm(hi.z - lo.z),
            mm(hi.z),
            mm(lo.z)
        ));
    }
    let side = |a: Anchor, low: &str, high: &str| match a {
        Anchor::Min => low.to_string(),
        Anchor::Center => "centre".to_string(),
        Anchor::Max => high.to_string(),
    };
    notes.push(match &setup.origin {
        SetupOrigin::Box { of, x, y, z } => {
            let of = if *of == BoxOf::Stock { "stock" } else { "part" };
            let (x, y, z) = (side(*x, "left", "right"), side(*y, "front", "back"), side(*z, "bottom", "top"));
            format!("Work zero G{}: {of} box, X {x}, Y {y}, Z {z}", setup.work_offset)
        }
        SetupOrigin::Point { at } => format!("Work zero G{}: the world point {}, {}, {}", setup.work_offset, mm(at.x), mm(at.y), mm(at.z)),
        SetupOrigin::Edge { body, on, z, .. } => {
            let on = match on {
                EdgePoint::Vertex => "a corner",
                EdgePoint::Middle => "the middle of an edge",
                EdgePoint::Center => "the centre of a round edge",
            };
            let z = match z {
                OriginZ::Point => "Z there".to_string(),
                OriginZ::Box { of, z } => format!("Z on the {} {}", if *of == BoxOf::Stock { "stock" } else { "part" }, side(*z, "bottom", "top")),
            };
            format!("Work zero G{}: {on} of {body}, {z}", setup.work_offset)
        }
    });
    // Seen from below: how the part is turned over for it.
    if setup.axis.z < -1.0 + 1e-9 {
        let about = if at.frame.x.x > 0.5 {
            "X"
        } else if at.frame.x.x < -0.5 {
            "Y"
        } else {
            "Z and over"
        };
        notes.push(format!("Turned over: part flipped about the {about} axis, the tool from below"));
    }
    if let Some(c) = setup.copies {
        notes.push(format!("Copies: {} x {}, {} x {} mm apart", c.nx, c.ny, mm(c.dx), mm(c.dy)));
    }
    if let Some(material) = &setup.material {
        notes.push(format!("Material: {material}"));
    }
    let mut seen = Vec::new();
    for op in ops {
        if !seen.contains(&op.tool.number) {
            seen.push(op.tool.number);
            let t = &op.tool;
            let about = if t.description.trim().is_empty() { String::new() } else { format!(" {}", t.description.trim()) };
            notes.push(format!("T{}: D{} {}{about}", t.number, mm(t.diameter), t.kind.label()));
        }
    }
    let mut last = None;
    let changes = ops.iter().filter(|op| last.replace(op.tool.number) != Some(op.tool.number)).count();
    notes.push(format!("Tools loaded: {changes}"));
    notes.push(format!("Cutting time about {} min", mm((minutes * 10.0).round() / 10.0)));
    notes
}

/// A setup's sheet for the machine, in Markdown: what to clamp and where the zero is, the
/// tools to load, and the operations in the order they run.
fn sheet(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.sheet";
    let setup = setup(s, p, cmd)?.clone();
    let at = resolve(s, &setup).map_err(|e| bad(cmd, e))?;
    let listed = list_now(s);
    let ops: Vec<Value> = listed["setups"]
        .as_array()
        .into_iter()
        .flatten()
        .find(|x| x["id"] == json!(setup.id))
        .and_then(|x| x["operations"].as_array().cloned())
        .unwrap_or_default();
    // Text that cannot break a table row.
    let cell = |t: &str| t.replace('|', "/").replace(['\n', '\r'], " ");
    let mm = |v: f64| {
        let t = format!("{:.3}", v + 0.0);
        let t = t.trim_end_matches('0').trim_end_matches('.').to_string();
        if t == "-0" { "0".to_string() } else { t }
    };
    let mut out = vec![format!("# {} – {}", cell(&s.doc.name), cell(&setup.name)), String::new()];
    let programs: Vec<cam::Operation> = setup
        .operations
        .iter()
        .filter_map(|op| {
            Some(cam::Operation {
                name: op.name.clone(),
                tool: s.doc.manufacture.tool(op.tool)?.clone(),
                spindle_rpm: op.spindle_rpm,
                coolant: op.coolant,
                path: cam::Toolpath::default(),
            })
        })
        .collect();
    let minutes: f64 = ops.iter().filter_map(|o| o["toolpath"]["cut_minutes"].as_f64()).sum();
    for note in program_notes(&setup, &at, &programs, minutes) {
        out.push(format!("- {}", cell(&note)));
    }
    out.push(format!("- Post: {}", cell(&setup.post)));
    out.push(String::new());
    out.push("## Tools".into());
    out.push(String::new());
    out.push("| T | Kind | Diameter | Flutes | Flute length | Out of the holder | Description |".into());
    out.push("|---|---|---|---|---|---|---|".into());
    let mut seen = Vec::new();
    for op in &setup.operations {
        if seen.contains(&op.tool) {
            continue;
        }
        seen.push(op.tool);
        if let Some(t) = s.doc.manufacture.tool(op.tool) {
            let length = t.length.map_or_else(|| "–".to_string(), |l| format!("{} mm", mm(l)));
            out.push(format!(
                "| T{} | {} | {} mm | {} | {} mm | {length} | {} |",
                t.number,
                t.kind.label(),
                mm(t.diameter),
                t.flutes,
                mm(t.flute_length),
                cell(&t.description)
            ));
        }
    }
    out.push(String::new());
    out.push("## Operations".into());
    out.push(String::new());
    out.push("| # | Operation | Kind | Tool | Spindle | Feed | Plunge | Stepdown | Lowest Z | Time |".into());
    out.push("|---|---|---|---|---|---|---|---|---|---|".into());
    let mut warnings = Vec::new();
    for (k, o) in ops.iter().enumerate() {
        let text = |key: &str| o[key].as_str().map(str::to_string).unwrap_or_else(|| o[key].to_string());
        let number = |v: &Value| v.as_f64().map_or_else(|| "–".to_string(), mm);
        out.push(format!(
            "| {} | {} | {} | T{} | {} rev/min | {} mm/min | {} mm/min | {} | {} | {} min |",
            k + 1,
            cell(&text("name")),
            cell(&text("kind")),
            o["tool"],
            number(&o["spindle_rpm"]),
            number(&o["cut_feed"]),
            number(&o["plunge_feed"]),
            cell(&text("stepdown")),
            number(&o["toolpath"]["lowest_z"]),
            number(&o["toolpath"]["cut_minutes"].as_f64().map(|m| (m * 10.0).round() / 10.0).map_or(Value::Null, |m| json!(m))),
        ));
        for key in ["error", "warning"] {
            if let Some(w) = o[key].as_str() {
                warnings.push(format!("- {}: {}", cell(&text("name")), cell(w)));
            }
        }
    }
    if !warnings.is_empty() {
        out.push(String::new());
        out.push("## Warnings".into());
        out.push(String::new());
        out.extend(warnings);
    }
    let text = out.join("\n") + "\n";
    let mut result = json!({ "lines": out.len() });
    match str_(p, "path") {
        Some(path) => {
            solidcraft_io::vfs::write(path, text.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
            result["path"] = json!(path);
        }
        None => result["text"] = json!(text),
    }
    Ok(result)
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
    let seconds = |k: &str, most: f64| match p.get(k) {
        None | Some(Value::Null) => Ok(0.0),
        Some(_) => num(p, k).filter(|v| (0.0..=most).contains(v)).ok_or_else(|| bad(cmd, format!("`{k}` must be 0 to {most}"))),
    };
    let safety = cam::Safety {
        spindle_delay: seconds("spindle_delay", 60.0)?,
        optional_stops: bool_(p, "optional_stops").unwrap_or(false),
        air_cut: seconds("air_cut", 1000.0)?,
    };
    let mut program = cam::Program { name, operations: Vec::new(), notes: Vec::new(), safety, work_offset: Some(setup.work_offset) };
    let mut minutes = 0.0;
    let at = resolve(s, setup).map_err(|e| bad(cmd, e))?;
    for (_, op) in setup.operations.iter().enumerate().filter(|(i, _)| only.is_none_or(|o| o == *i)) {
        let path = build_in(s, &at, op)?;
        minutes += path.stats().cut_minutes;
        let tool = s.doc.manufacture.tool(op.tool).cloned().ok_or_else(|| bad(cmd, format!("no tool {}", op.tool)))?;
        program.operations.push(cam::Operation { name: op.name.clone(), tool, spindle_rpm: op.spindle_rpm, coolant: op.coolant, path });
    }
    program.notes = program_notes(setup, &at, &program.operations, minutes);
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
