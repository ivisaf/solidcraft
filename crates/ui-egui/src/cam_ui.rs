//! The Manufacture window and the toolpaths drawn in the viewport.
//!
//! A window over the engine's `cam.*` commands: the tool library, the setups with their
//! operations (each with its cutting time, or the error that stops it), a form for whichever
//! command the toolbar started, and Post Process. Toolpaths come from `cam.toolpath` and are
//! redrawn when the design changes.

use egui::{RichText, Stroke};
use serde_json::{Value, json};
use solidcraft_engine::geom::Vec3;

use crate::SolveApp;
use crate::theme::Tokens;
use crate::viewport::Proj;

const TOOL_KINDS: [&str; 3] = ["flat", "ball", "drill"];
const SIDES: [&str; 3] = ["outside", "inside", "on"];
/// What a facing operation's boundary is, when no faces are picked.
const FACE_BOUNDS: [&str; 3] = ["stock", "part", "sketch"];
/// Whether a facing operation cuts over its boundary or stays inside it.
const FACE_LIMITS: [&str; 2] = ["over", "inside"];
/// Tool axes a setup form offers; the last stands for a direction set by command, kept as it is.
const AXES: [&str; 7] = ["+z", "-z", "+x", "-x", "+y", "-y", "custom"];
const ANCHORS: [&str; 3] = ["min", "center", "max"];
/// What the work origin is on.
const ORIGINS: [&str; 3] = ["stock box", "part box", "world point"];
/// Most toolpath segments drawn.
const MAX_SEGMENTS: usize = 400_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    Face,
    Contour,
    Pocket,
    Drill,
}

impl OpKind {
    fn command(self) -> &'static str {
        match self {
            OpKind::Face => "cam.op.face",
            OpKind::Contour => "cam.op.contour",
            OpKind::Pocket => "cam.op.pocket",
            OpKind::Drill => "cam.op.drill",
        }
    }
    fn title(self) -> &'static str {
        match self {
            OpKind::Face => "FACE",
            OpKind::Contour => "2D CONTOUR",
            OpKind::Pocket => "2D POCKET",
            OpKind::Drill => "DRILL",
        }
    }
}

/// What is being typed for the command the toolbar started.
#[derive(Clone, Debug, PartialEq)]
pub enum Form {
    Tool(Box<ToolForm>),
    Setup(Box<SetupForm>),
    Op(Box<OpForm>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct ToolForm {
    /// Changing a tool that exists (its number then stays).
    pub editing: bool,
    /// A tool of the machine's library; otherwise one that is only in this design.
    pub library: bool,
    pub number: String,
    pub kind: usize,
    pub diameter: String,
    pub flute_length: String,
    pub description: String,
    pub flutes: String,
    pub spindle_rpm: String,
    pub feed: String,
    pub plunge_feed: String,
    /// Depth per pass and distance between passes (mm).
    pub stepdown: String,
    pub stepover: String,
    /// Feed per tooth (mm), kept in step with the feed: feed = speed × flutes × chip load.
    pub chip_load: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SetupForm {
    /// The setup being changed (`None`: a new one).
    pub editing: Option<u64>,
    pub name: String,
    /// Index into [`AXES`].
    pub axis: usize,
    /// Index into [`ORIGINS`], then where on the box ([`ANCHORS`] per axis) or which point.
    pub origin: usize,
    pub anchors: [usize; 3],
    pub point: [String; 3],
    /// Stock offsets.
    pub side: String,
    pub top: String,
    pub bottom: String,
    pub post: usize,
}

/// The operation a form edits, and the geometry it had when the form opened.
#[derive(Clone, Debug, PartialEq)]
pub struct Editing {
    pub setup: u64,
    pub op: u64,
    /// Index into [`FACE_BOUNDS`] (facing only).
    pub boundary: usize,
    pub sketch: String,
    pub faces: Vec<(String, Vec3)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpForm {
    pub kind: OpKind,
    /// The tool whose cutting data the form was last filled from: choosing another fills it again.
    pub filled_for: String,
    /// That tool's flute count (0: not known), and the chip load the feed and speed make with it.
    pub flutes: u32,
    pub chip_load: String,
    /// Facing: the boundary (index into [`FACE_BOUNDS`]), whether the tool cuts over it or stays
    /// inside it (index into [`FACE_LIMITS`]), and by how much.
    pub boundary: usize,
    pub limit: usize,
    pub margin: String,
    /// The operation being changed (`None`: a new one).
    pub editing: Option<Editing>,
    pub name: String,
    /// Picked faces (body, a point on the face); when there are any they are cut, not the sketch.
    pub faces: Vec<(String, Vec3)>,
    pub sketch: String,
    pub tool: String,
    pub spindle_rpm: String,
    pub feed: String,
    pub plunge_feed: String,
    pub top: String,
    pub bottom: String,
    pub stepdown: String,
    pub stepover: String,
    pub side: usize,
    pub climb: bool,
    pub stock_to_leave: String,
    pub ramp_angle: String,
    pub peck: String,
    pub dwell: String,
    pub coolant: bool,
}

/// The cut stock as columns in world coordinates (see `cam.simulate`'s `field`).
#[derive(Clone, Debug, PartialEq)]
pub struct StockField {
    /// The setup's work frame in the world: origin, X, Y, Z. Columns are in work coordinates.
    pub frame: [Vec3; 4],
    pub origin: [f64; 2],
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    pub floor: f64,
    pub z: Vec<f64>,
    /// Per column: below 0 cut into the part, above 0 left on it. Empty when there was no part to compare with.
    pub deviation: Vec<f64>,
}

impl StockField {
    fn from_json(v: &Value) -> Option<StockField> {
        let numbers = |k: &str| v[k].as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect::<Vec<f64>>());
        let (nx, ny) = (v["nx"].as_u64()? as usize, v["ny"].as_u64()? as usize);
        let z = numbers("z")?;
        let deviation = numbers("deviation").unwrap_or_default();
        let f = &v["frame"];
        (nx.checked_mul(ny) == Some(z.len()) && (deviation.is_empty() || deviation.len() == z.len())).then_some(StockField {
            frame: [point(&f["origin"])?, point(&f["x"])?, point(&f["y"])?, point(&f["z"])?],
            origin: [v["origin"][0].as_f64()?, v["origin"][1].as_f64()?],
            cell: v["cell"].as_f64()?,
            nx,
            ny,
            floor: v["floor"].as_f64()?,
            z,
            deviation,
        })
    }

    /// A point in work coordinates, in the world.
    fn world(&self, q: Vec3) -> Vec3 {
        let [o, x, y, z] = self.frame;
        o + x * q.x + y * q.y + z * q.z
    }

    /// Column `(i, j)` in the world: the top of it at its centre, the point on the floor
    /// under that, and its deviation from the part.
    fn column(&self, i: usize, j: usize) -> Option<(Vec3, Vec3, f64)> {
        let k = j.checked_mul(self.nx)?.checked_add(i)?;
        let z = *self.z.get(k).filter(|_| i < self.nx && j < self.ny)?;
        let (cx, cy) = (self.origin[0] + (i as f64 + 0.5) * self.cell, self.origin[1] + (j as f64 + 0.5) * self.cell);
        Some((self.world(Vec3::new(cx, cy, z)), self.world(Vec3::new(cx, cy, self.floor)), self.deviation.get(k).copied().unwrap_or(0.0)))
    }
}

/// Deviations smaller than this are drawn as the part (mm).
const SHOWN_DEVIATION: f64 = 1e-3;

/// What a triangle of the drawn stock is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StockMark {
    Fine,
    /// Cut into the part.
    Gouge,
    /// Material left on the part.
    Leftover,
}

/// Whether the simulated stock is drawn (in place of the bodies).
pub fn stock_shown(app: &SolveApp) -> bool {
    app.manufacture.open && app.manufacture.show_stock && app.manufacture.stock.is_some()
}

/// Changes when what the viewport draws of the stock changes.
pub fn stock_key(app: &SolveApp) -> (bool, u64) {
    (stock_shown(app), app.manufacture.stock_stamp)
}

/// The simulated stock as triangles: its top through the column centres and walls down to
/// its floor round the edge, each with what it is.
pub fn stock_triangles(field: &StockField) -> Vec<([Vec3; 3], StockMark)> {
    let mut out = Vec::new();
    let mark = |d: [f64; 3]| {
        if d.iter().any(|d| *d < -SHOWN_DEVIATION) {
            StockMark::Gouge
        } else if d.iter().any(|d| *d > SHOWN_DEVIATION) {
            StockMark::Leftover
        } else {
            StockMark::Fine
        }
    };
    for j in 0..field.ny.saturating_sub(1) {
        for i in 0..field.nx.saturating_sub(1) {
            let (Some(a), Some(b), Some(c), Some(d)) =
                (field.column(i, j), field.column(i + 1, j), field.column(i + 1, j + 1), field.column(i, j + 1))
            else {
                continue;
            };
            out.push(([a.0, b.0, c.0], mark([a.2, b.2, c.2])));
            out.push(([a.0, c.0, d.0], mark([a.2, c.2, d.2])));
        }
    }
    // Walls round the edge, outward-facing: along each side, from the top's rim down to the floor.
    let mut wall = |a: Option<(Vec3, Vec3, f64)>, b: Option<(Vec3, Vec3, f64)>| {
        if let (Some((a, a_floor, _)), Some((b, b_floor, _))) = (a, b) {
            out.push(([a, a_floor, b_floor], StockMark::Fine));
            out.push(([a, b_floor, b], StockMark::Fine));
        }
    };
    let (lx, ly) = (field.nx.saturating_sub(1), field.ny.saturating_sub(1));
    for i in 0..lx {
        wall(field.column(i, 0), field.column(i + 1, 0));
        wall(field.column(i + 1, ly), field.column(i, ly));
    }
    for j in 0..ly {
        wall(field.column(0, j + 1), field.column(0, j));
        wall(field.column(lx, j), field.column(lx, j + 1));
    }
    out
}

/// Adds the simulated stock to the model scene: the part's colour where it matches the part,
/// the error colour where it is cut into, the warning colour where material is left.
pub fn stock_scene(app: &SolveApp, sc: &mut crate::gpu::GpuScene) {
    let Some(field) = app.manufacture.stock.as_ref().filter(|_| stock_shown(app)) else { return };
    let t = Tokens::get();
    let rgba = |c: egui::Color32| [c.r(), c.g(), c.b(), 255];
    for (tri, mark) in stock_triangles(field) {
        let [a, b, c] = tri;
        let n = (b - a).cross(c - a).normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0)).to_f32();
        let colour = rgba(match mark {
            StockMark::Fine => t.body,
            StockMark::Gouge => t.error,
            StockMark::Leftover => t.warning,
        });
        for p in tri {
            sc.tri(p.to_f32(), n, colour);
        }
    }
}

/// One operation's toolpath in world coordinates: `(point, reached at feed)`.
pub type Path = Vec<(Vec3, bool)>;

/// A setup as the viewport shows it: its work frame (origin, X, Y, Z in the world) and its
/// stock box in work coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct SetupView {
    pub frame: [Vec3; 4],
    pub stock: Option<(Vec3, Vec3)>,
}

pub struct CamUi {
    pub open: bool,
    /// The Tool Library window.
    pub tools_open: bool,
    /// The tool picked in the Tool Library window: whether it is the machine library's, and its number.
    pub selected_tool: Option<(bool, u64)>,
    /// The tool library file to use instead of the machine's own (tests).
    pub library: Option<String>,
    /// The machine library as last listed, and how many frames ago.
    listing: Option<(u32, Value)>,
    /// Whether setups' stock boxes and work origins are drawn.
    pub show_setups: bool,
    pub form: Option<Form>,
    pub show_paths: bool,
    pub error: Option<String>,
    /// The last simulation: what it found, and its warnings.
    pub simulation: Option<(Vec<String>, Vec<String>)>,
    /// The stock as the last simulation left it, drawn in place of the bodies.
    pub stock: Option<StockField>,
    pub show_stock: bool,
    /// Counts changes of `stock` (the viewport redraws when it moves on).
    stock_stamp: u64,
    /// The toolpaths of every setup, for the design revision they were worked out at.
    cache: Option<(u64, Vec<Path>, Vec<SetupView>)>,
}

impl Default for CamUi {
    fn default() -> Self {
        CamUi {
            open: false,
            tools_open: false,
            selected_tool: None,
            library: None,
            listing: None,
            show_setups: true,
            form: None,
            show_paths: true,
            error: None,
            simulation: None,
            stock: None,
            show_stock: true,
            stock_stamp: 0,
            cache: None,
        }
    }
}

/// A point as the engine writes one: `[x, y, z]` or `{x, y, z}`.
fn point(v: &Value) -> Option<Vec3> {
    let c = |i: usize, k: &str| v.get(i).or_else(|| v.get(k)).and_then(Value::as_f64);
    Some(Vec3::new(c(0, "x")?, c(1, "y")?, c(2, "z")?))
}

fn text(v: &Value, k: &str) -> String {
    v.get(k).and_then(Value::as_str).unwrap_or_default().to_string()
}

/// The faces in the selection, as an operation takes them.
fn selected_faces(app: &SolveApp) -> Vec<(String, Vec3)> {
    app.session
        .selection
        .iter()
        .filter_map(|s| if let solidcraft_engine::Sel::Face { body, point, .. } = s { Some((body.clone(), *point)) } else { None })
        .collect()
}

/// Whether `id` is a Manufacture command, and the window it opens if so.
pub fn start(app: &mut SolveApp, id: &str) -> bool {
    if !id.starts_with("cam.") {
        return false;
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let lists = {
        let library = library_listing(app, true);
        choices(app, &listed, &library)
    };
    let first_tool = lists.tools.first().map(|t| t.0.clone()).unwrap_or_default();
    let op = |kind: OpKind| {
        let sketch = app
            .session
            .doc
            .features
            .iter()
            .rev()
            .find(|f| matches!(f.kind, solidcraft_engine::doc::FeatureKind::Sketch { .. }))
            .map(|f| f.name.clone())
            .unwrap_or_default();
        let faces = selected_faces(app);
        Form::Op(Box::new(OpForm {
            kind,
            filled_for: String::new(),
            flutes: 0,
            chip_load: String::new(),
            // Facing covers the stock when there is a body to have stock round, else a sketch.
            boundary: if app.session.world_state().bodies.is_empty() { 2 } else { 0 },
            limit: 0,
            margin: String::new(),
            editing: None,
            name: String::new(),
            bottom: if faces.is_empty() { "-5 mm".into() } else { String::new() },
            faces,
            sketch,
            tool: first_tool.clone(),
            spindle_rpm: String::new(),
            feed: String::new(),
            plunge_feed: String::new(),
            top: String::new(),
            stepdown: String::new(),
            stepover: if kind == OpKind::Face { "5 mm".into() } else { String::new() },
            side: 0,
            climb: true,
            stock_to_leave: String::new(),
            ramp_angle: String::new(),
            peck: String::new(),
            dwell: String::new(),
            coolant: false,
        }))
    };
    let form = match id {
        "cam.tool.define" => {
            let library = library_listing(app, true);
            let taken = |n: u64| library["tools"].as_array().is_some_and(|a| a.iter().any(|t| t["number"].as_u64() == Some(n)));
            Some(Form::Tool(Box::new(ToolForm {
                editing: false,
                library: true,
                number: (1u64..).find(|n| !taken(*n)).unwrap_or(1).to_string(),
                kind: 0,
                diameter: "6 mm".into(),
                flute_length: String::new(),
                description: String::new(),
                flutes: "2".into(),
                spindle_rpm: String::new(),
                feed: String::new(),
                plunge_feed: String::new(),
                stepdown: String::new(),
                stepover: String::new(),
                chip_load: String::new(),
            })))
        }
        "cam.setup.create" => Some(Form::Setup(Box::new(SetupForm {
            editing: None,
            name: String::new(),
            axis: 0,
            // With a body to put a box round: the stock's top at its lowest X and Y.
            origin: if app.session.world_state().bodies.is_empty() { 2 } else { 0 },
            anchors: [0, 0, 2],
            point: ["0".into(), "0".into(), "0".into()],
            side: String::new(),
            top: String::new(),
            bottom: String::new(),
            post: 0,
        }))),
        "cam.op.face" => Some(op(OpKind::Face)),
        "cam.op.contour" => Some(op(OpKind::Contour)),
        "cam.op.pocket" => Some(op(OpKind::Pocket)),
        "cam.op.drill" => Some(op(OpKind::Drill)),
        _ => None,
    };
    if id == "cam.toolpath" {
        app.manufacture.show_paths = true;
    }
    if id == "cam.simulate" {
        simulate(app);
    }
    // Tools have a window of their own.
    if id.starts_with("cam.tool.") || id.starts_with("cam.library.") {
        app.manufacture.tools_open = true;
    } else {
        app.manufacture.open = true;
    }
    let mut form = form;
    if let Some(Form::Op(f)) = &mut form {
        prefill(f, &lists);
    }
    app.manufacture.form = form;
    app.manufacture.error = None;
    true
}

/// The command a form stands for. Fields left empty are left to the command's defaults.
pub fn command(form: &Form) -> Result<(&'static str, Value), String> {
    let number = |what: &str, s: &str| s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{what} must be a number"));
    let mut p = serde_json::Map::new();
    fn optional(p: &mut serde_json::Map<String, Value>, k: &str, s: &str) {
        if !s.trim().is_empty() {
            p.insert(k.into(), json!(s.trim()));
        }
    }
    match form {
        Form::Tool(f) => {
            optional(&mut p, "diameter", &f.diameter);
            optional(&mut p, "flute_length", &f.flute_length);
            // Sent even when empty, so that a description can be removed.
            p.insert("description".into(), json!(f.description.trim()));
            p.insert("number".into(), json!(number("The tool number", &f.number)? as u64));
            p.insert("kind".into(), json!(TOOL_KINDS.get(f.kind).copied().unwrap_or("flat")));
            if !f.flutes.trim().is_empty() {
                p.insert("flutes".into(), json!(number("The number of flutes", &f.flutes)? as u64));
            }
            // An emptied value is cleared on a tool being edited.
            for (k, what, text) in [
                ("spindle_rpm", "The spindle speed", &f.spindle_rpm),
                ("feed", "The feed", &f.feed),
                ("plunge_feed", "The plunge feed", &f.plunge_feed),
                ("stepdown", "The stepdown", &f.stepdown),
                ("stepover", "The stepover", &f.stepover),
            ] {
                if !text.trim().is_empty() {
                    p.insert(k.into(), json!(number(what, text)?));
                } else if f.editing {
                    p.insert(k.into(), Value::Null);
                }
            }
            // A chip load without a feed: the engine works the feed out.
            if f.feed.trim().is_empty() && !f.chip_load.trim().is_empty() {
                p.insert("feed".into(), Value::Null);
                p.insert("chip_load".into(), json!(number("The chip load", &f.chip_load)?));
            }
            Ok((if f.library { "cam.library.define" } else { "cam.tool.define" }, Value::Object(p)))
        }
        Form::Setup(f) => {
            optional(&mut p, "name", &f.name);
            if let Some(axis) = AXES.get(f.axis).filter(|a| **a != "custom") {
                p.insert("axis".into(), json!(axis));
            }
            let origin = match f.origin {
                2 => {
                    let [x, y, z] = &f.point;
                    json!([number("The origin's X", x)?, number("The origin's Y", y)?, number("The origin's Z", z)?])
                }
                of => {
                    let at = |i: usize| f.anchors.get(i).and_then(|a| ANCHORS.get(*a)).copied().unwrap_or("min");
                    json!({"of": if of == 1 { "part" } else { "stock" }, "x": at(0), "y": at(1), "z": at(2)})
                }
            };
            p.insert("origin".into(), origin);
            let mut stock = serde_json::Map::new();
            for (k, text) in [("side", &f.side), ("top", &f.top), ("bottom", &f.bottom)] {
                // Emptied on an edit means no offset; left empty on a new setup too.
                if !text.trim().is_empty() {
                    stock.insert(k.into(), json!(text.trim()));
                } else if f.editing.is_some() {
                    stock.insert(k.into(), json!(0));
                }
            }
            if !stock.is_empty() {
                p.insert("stock".into(), Value::Object(stock));
            }
            p.insert("post".into(), json!(solidcraft_engine::cam::Post::BUILTIN.get(f.post).copied().unwrap_or("rs274")));
            match f.editing {
                Some(id) => {
                    p.insert("setup".into(), json!(id));
                    Ok(("cam.setup.edit", Value::Object(p)))
                }
                None => Ok(("cam.setup.create", Value::Object(p))),
            }
        }
        Form::Op(f) => {
            optional(&mut p, "name", &f.name);
            optional(&mut p, "stepdown", &f.stepdown);
            match &f.editing {
                // A new operation: what is empty is left to the command.
                None => {
                    if f.faces.is_empty() {
                        match (f.kind, f.boundary) {
                            // Facing the stock is what the command does with nothing named.
                            (OpKind::Face, 0) => {}
                            (OpKind::Face, 1) => {
                                p.insert("boundary".into(), json!("part"));
                            }
                            _ => optional(&mut p, "sketch", &f.sketch),
                        }
                    }
                    optional(&mut p, "top", &f.top);
                    optional(&mut p, "bottom", &f.bottom);
                }
                // An edit: an emptied height goes back to automatic; geometry is sent only when it changed.
                Some(e) => {
                    for (k, text) in [("top", &f.top), ("bottom", &f.bottom)] {
                        p.insert(k.into(), if text.trim().is_empty() { Value::Null } else { json!(text.trim()) });
                    }
                    if f.kind == OpKind::Drill && f.peck.trim().is_empty() {
                        p.insert("peck".into(), Value::Null);
                    }
                    let on_sketch = f.kind != OpKind::Face || f.boundary == 2;
                    let was_on_sketch = f.kind != OpKind::Face || e.boundary == 2;
                    if f.faces.is_empty() && on_sketch && (f.sketch.trim() != e.sketch || !e.faces.is_empty() || !was_on_sketch) {
                        p.insert("sketch".into(), json!(f.sketch.trim()));
                    } else if f.faces.is_empty() && !on_sketch && (f.boundary != e.boundary || !e.faces.is_empty()) {
                        p.insert("boundary".into(), json!(FACE_BOUNDS.get(f.boundary).copied().unwrap_or("stock")));
                    }
                    p.insert("op".into(), json!(e.op));
                    p.insert("setup".into(), json!(e.setup));
                }
            }
            match f.kind {
                OpKind::Face => {
                    optional(&mut p, "stepover", &f.stepover);
                    p.insert("limit".into(), json!(FACE_LIMITS.get(f.limit).copied().unwrap_or("over")));
                    // Left empty: over by the tool's radius, inside by nothing.
                    if !f.margin.trim().is_empty() {
                        p.insert("margin".into(), json!(f.margin.trim()));
                    } else if f.editing.is_some() {
                        p.insert("margin".into(), Value::Null);
                    }
                }
                OpKind::Contour => {
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    optional(&mut p, "ramp_angle", &f.ramp_angle);
                }
                OpKind::Pocket => {
                    optional(&mut p, "stepover", &f.stepover);
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    optional(&mut p, "ramp_angle", &f.ramp_angle);
                }
                OpKind::Drill => optional(&mut p, "peck", &f.peck),
            }
            if !f.faces.is_empty() && f.editing.as_ref().is_none_or(|e| e.faces != f.faces) {
                let faces: Vec<Value> = f.faces.iter().map(|(body, at)| json!({"body": body, "point": [at.x, at.y, at.z]})).collect();
                p.insert("faces".into(), json!(faces));
            }
            p.insert("tool".into(), json!(number("The tool", &f.tool)? as u64));
            // A chip load without a feed gives the feed, at this operation's speed and its tool's flutes.
            let mut from_chip = String::new();
            if f.feed.trim().is_empty()
                && let (Some(chip), Some(rpm)) = (parsed(&f.chip_load), parsed(&f.spindle_rpm))
                && f.flutes > 0
            {
                from_chip = shown(rpm * f64::from(f.flutes) * chip, 1);
            }
            let feed = if from_chip.is_empty() { &f.feed } else { &from_chip };
            // Left empty, speed and feeds are the tool's own (or, on an edit, stay as they are).
            for (k, what, text) in
                [("spindle_rpm", "The spindle speed", &f.spindle_rpm), ("feed", "The feed", feed), ("plunge_feed", "The plunge feed", &f.plunge_feed)]
            {
                if !text.trim().is_empty() {
                    p.insert(k.into(), json!(number(what, text)?));
                }
            }
            if f.kind == OpKind::Drill && !f.dwell.trim().is_empty() {
                p.insert("dwell".into(), json!(number("The dwell", &f.dwell)?));
            }
            if f.kind == OpKind::Contour {
                p.insert("side".into(), json!(SIDES.get(f.side).copied().unwrap_or("outside")));
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket) {
                p.insert("climb".into(), json!(f.climb));
            }
            p.insert("coolant".into(), json!(f.coolant));
            Ok((if f.editing.is_some() { "cam.op.edit" } else { f.kind.command() }, Value::Object(p)))
        }
    }
}

/// A form filled in from a tool as `cam.list` reports it.
pub fn tool_form(tool: &Value, library: bool) -> Option<Form> {
    let number = |k: &str| tool[k].as_f64().map(|x| x.to_string()).unwrap_or_default();
    let kind = match tool["kind"].as_str()? {
        "ball_endmill" => 1,
        "drill" => 2,
        _ => 0,
    };
    Some(Form::Tool(Box::new(ToolForm {
        editing: true,
        library,
        number: tool["number"].as_u64()?.to_string(),
        kind,
        diameter: format!("{} mm", tool["diameter"].as_f64()?),
        flute_length: format!("{} mm", tool["flute_length"].as_f64()?),
        description: text(tool, "description"),
        flutes: number("flutes"),
        spindle_rpm: number("spindle_rpm"),
        feed: number("feed"),
        plunge_feed: number("plunge_feed"),
        stepdown: number("stepdown"),
        stepover: number("stepover"),
        chip_load: tool["chip_load"].as_f64().map(|c| shown(c, 4)).unwrap_or_default(),
    })))
}

/// A form filled in from a setup as `cam.list` reports it.
pub fn setup_form(setup: &Value) -> Option<Form> {
    let axis = point(&setup["axis"])?;
    let named = [(0.0, 0.0, 1.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)];
    let axis = named.iter().position(|d| axis.dist(Vec3::new(d.0, d.1, d.2)) < 1e-9).unwrap_or(AXES.len() - 1);
    let origin = &setup["origin"];
    let anchor = |k: &str| ANCHORS.iter().position(|a| Some(*a) == origin[k].as_str()).unwrap_or(0);
    let at = point(&origin["at"]).unwrap_or(Vec3::ZERO);
    let offset = |k: &str| Some(text(&setup["stock"], k)).filter(|e| !matches!(e.as_str(), "0" | "0 mm")).unwrap_or_default();
    Some(Form::Setup(Box::new(SetupForm {
        editing: Some(setup["id"].as_u64()?),
        name: text(setup, "name"),
        axis,
        origin: match (origin["type"].as_str(), origin["of"].as_str()) {
            (Some("box"), Some("part")) => 1,
            (Some("box"), _) => 0,
            _ => 2,
        },
        anchors: [anchor("x"), anchor("y"), anchor("z")],
        point: [at.x.to_string(), at.y.to_string(), at.z.to_string()],
        side: offset("side"),
        top: offset("top"),
        bottom: offset("bottom"),
        post: solidcraft_engine::cam::Post::BUILTIN.iter().position(|p| Some(*p) == setup["post"].as_str()).unwrap_or(0),
    })))
}

/// A form filled in from an operation as `cam.list` reports it.
pub fn edit_form(app: &SolveApp, setup: &Value, op: &Value) -> Option<Form> {
    let kind = match op["kind"].as_str()? {
        "face" => OpKind::Face,
        "contour" => OpKind::Contour,
        "pocket" => OpKind::Pocket,
        "drill" => OpKind::Drill,
        _ => return None,
    };
    let number = |v: &Value| v.as_f64().map(|x| x.to_string()).unwrap_or_default();
    let flutes = op["tool"].as_u64().and_then(|n| app.session.doc.manufacture.tool(n as u32)).map_or(0, |t| t.flutes);
    let sketch = op["geometry"]["sketch"].as_u64().and_then(|id| app.session.doc.feature(id)).map(|f| f.name.clone()).unwrap_or_default();
    let faces: Vec<(String, Vec3)> = op["geometry"]["faces"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|f| Some((f["body"].as_str()?.to_string(), Vec3::new(f["at"]["x"].as_f64()?, f["at"]["y"].as_f64()?, f["at"]["z"].as_f64()?))))
        .collect();
    let inputs = &op["inputs"];
    let boundary = match op["geometry"]["type"].as_str() {
        Some("part") => 1,
        Some("sketch") => 2,
        _ => 0,
    };
    Some(Form::Op(Box::new(OpForm {
        kind,
        // An operation's own values are shown; they are only replaced when another tool is chosen.
        filled_for: op["tool"].as_u64()?.to_string(),
        flutes,
        chip_load: match (op["cut_feed"].as_f64(), op["spindle_rpm"].as_f64()) {
            (Some(feed), Some(rpm)) if flutes > 0 && rpm > 0.0 => shown(feed / (rpm * f64::from(flutes)), 4),
            _ => String::new(),
        },
        boundary,
        limit: usize::from(inputs["inside"].as_bool().unwrap_or(false)),
        margin: text(inputs, "margin"),
        editing: Some(Editing { setup: setup["id"].as_u64()?, op: op["id"].as_u64()?, boundary, sketch: sketch.clone(), faces: faces.clone() }),
        name: text(op, "name"),
        faces,
        sketch,
        tool: op["tool"].as_u64()?.to_string(),
        spindle_rpm: number(&op["spindle_rpm"]),
        feed: number(&op["cut_feed"]),
        plunge_feed: number(&op["plunge_feed"]),
        top: text(op, "top"),
        bottom: text(op, "bottom"),
        stepdown: text(op, "stepdown"),
        stepover: text(inputs, "stepover"),
        side: SIDES.iter().position(|s| Some(*s) == inputs["side"].as_str()).unwrap_or(0),
        climb: inputs["climb"].as_bool().unwrap_or(true),
        stock_to_leave: text(inputs, "stock_to_leave"),
        ramp_angle: text(inputs, "ramp_angle"),
        peck: text(inputs, "peck"),
        dwell: inputs["dwell"].as_f64().filter(|d| *d > 0.0).map(|d| d.to_string()).unwrap_or_default(),
        coolant: op["coolant"].as_bool().unwrap_or(false),
    })))
}

/// Runs a command with this window's tool library, if it has one of its own.
fn run(app: &mut SolveApp, id: &str, mut p: Value) -> Result<Value, String> {
    if let (Some(library), Some(o)) = (&app.manufacture.library, p.as_object_mut()) {
        o.insert("library".into(), json!(library));
    }
    app.manufacture.listing = None;
    app.run(id, p)
}

/// The machine's tool library as `cam.library.list` reports it, read again now and then (another
/// design may have changed it) and whenever `fresh` is asked for.
fn library_listing(app: &mut SolveApp, fresh: bool) -> Value {
    if let Some((age, listing)) = &mut app.manufacture.listing
        && !fresh
        && *age < 120
    {
        *age += 1;
        return listing.clone();
    }
    let p = match &app.manufacture.library {
        Some(library) => json!({ "library": library }),
        None => json!({}),
    };
    let listing = match app.session.execute("cam.library.list", &p) {
        Ok(v) => v,
        Err(e) => json!({ "error": e.to_string() }),
    };
    app.manufacture.listing = Some((0, listing.clone()));
    listing
}

/// Fills an operation form's speed, feeds, stepdown and stepover from its tool, when the tool is
/// not the one it was last filled from. They stay the operation's own to change.
fn prefill(f: &mut OpForm, lists: &Choices) {
    if f.filled_for == f.tool {
        return;
    }
    f.filled_for = f.tool.clone();
    let Some(data) = lists.data.iter().find(|(number, _)| *number == f.tool).map(|(_, d)| d) else { return };
    let show = |v: Option<f64>| v.map(|x| x.to_string()).unwrap_or_default();
    f.spindle_rpm = show(data.spindle_rpm);
    f.feed = show(data.feed);
    f.plunge_feed = show(data.plunge_feed);
    f.flutes = data.flutes;
    f.chip_load = match (data.feed, data.spindle_rpm) {
        (Some(feed), Some(rpm)) if data.flutes > 0 => shown(feed / (rpm * f64::from(data.flutes)), 4),
        _ => String::new(),
    };
    let length = |v: Option<f64>| v.map(|x| format!("{x} mm")).unwrap_or_default();
    f.stepdown = length(data.stepdown);
    f.stepover = length(data.stepover);
}

/// Runs the open form's command; the form closes when the command succeeds.
pub fn apply(app: &mut SolveApp) {
    let Some(form) = app.manufacture.form.clone() else { return };
    match command(&form).and_then(|(id, p)| run(app, id, p)) {
        Ok(_) => {
            app.manufacture.form = None;
            app.manufacture.error = None;
        }
        Err(e) => app.manufacture.error = Some(e),
    }
}

/// Simulates the first setup (what the toolbar's Simulate does).
pub fn simulate(app: &mut SolveApp) {
    match app.session.doc.manufacture.setups.first().map(|s| s.id) {
        Some(setup) => simulate_setup(app, setup),
        None => app.manufacture.error = Some("There is no setup to simulate".into()),
    }
}

/// Runs `cam.simulate` on a setup and keeps what it found for the window.
pub fn simulate_setup(app: &mut SolveApp, setup: u64) {
    app.manufacture.stock_stamp += 1;
    match app.session.execute("cam.simulate", &json!({ "setup": setup, "field": true })) {
        Ok(v) => {
            let number = |k: &str| v[k].as_f64();
            let mut found = vec![format!("Removed {:.2} cm³", number("removed_volume").unwrap_or(0.0) / 1000.0)];
            if let (Some(gouge), Some(left), Some(volume)) = (number("max_gouge"), number("max_leftover"), number("leftover_volume")) {
                found.push(if gouge > 1e-3 { format!("Cuts up to {gouge:.3} mm into the part") } else { "Does not cut into the part".into() });
                found.push(format!("Leaves up to {left:.2} mm on the part ({:.2} cm³)", volume / 1000.0));
            }
            let warnings = v["warnings"].as_array().into_iter().flatten().filter_map(Value::as_str).map(str::to_string).collect();
            app.manufacture.simulation = Some((found, warnings));
            app.manufacture.stock = StockField::from_json(&v["field"]);
            app.manufacture.error = None;
        }
        Err(e) => {
            app.manufacture.simulation = None;
            app.manufacture.stock = None;
            app.manufacture.error = Some(e.to_string());
        }
    }
}

/// Works the toolpaths and the setups' frames out again when the design has changed.
fn refresh(app: &mut SolveApp) {
    let revision = app.session.revision;
    if app.manufacture.cache.as_ref().is_some_and(|(r, ..)| *r == revision) {
        return;
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let (mut all, mut views) = (Vec::new(), Vec::new());
    for setup in listed["setups"].as_array().into_iter().flatten() {
        let f = &setup["frame"];
        if let (Some(o), Some(x), Some(y), Some(z)) = (point(&f["origin"]), point(&f["x"]), point(&f["y"]), point(&f["z"])) {
            let b = &setup["stock_box"];
            views.push(SetupView { frame: [o, x, y, z], stock: point(&b["min"]).zip(point(&b["max"])) });
        }
        // A setup with an operation that cannot be cut draws no toolpath; the window says why.
        let Ok(v) = app.session.execute("cam.toolpath", &json!({ "setup": setup["id"] })) else { continue };
        for op in v["operations"].as_array().into_iter().flatten() {
            let path: Path = op["points"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|p| Some((Vec3::new(p.get(0)?.as_f64()?, p.get(1)?.as_f64()?, p.get(2)?.as_f64()?), p.get(3)?.as_bool()?)))
                .collect();
            all.push(path);
        }
    }
    // A simulation of the design as it was says nothing about it now.
    app.manufacture.simulation = None;
    app.manufacture.stock = None;
    app.manufacture.stock_stamp += 1;
    // These commands are not changes, but keep the key the revision as it is now.
    app.manufacture.cache = Some((app.session.revision, all, views));
}

/// The toolpaths to draw.
pub fn paths(app: &mut SolveApp) -> &[Path] {
    refresh(app);
    app.manufacture.cache.as_ref().map_or(&[], |(_, p, _)| p.as_slice())
}

/// The setups to draw.
pub fn setup_views(app: &mut SolveApp) -> &[SetupView] {
    refresh(app);
    app.manufacture.cache.as_ref().map_or(&[], |(_, _, v)| v.as_slice())
}

/// The world edges of a setup's stock box and the three arms of its work origin (`X`, `Y`, `Z`).
pub fn setup_lines(view: &SetupView) -> (Vec<(Vec3, Vec3)>, [(Vec3, Vec3); 3]) {
    let [o, x, y, z] = view.frame;
    let world = |q: Vec3| o + x * q.x + y * q.y + z * q.z;
    let mut edges = Vec::new();
    let mut size = 20.0;
    if let Some((lo, hi)) = view.stock {
        let corner = |i: usize| {
            world(Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z }))
        };
        for i in 0..8usize {
            for bit in [1usize, 2, 4] {
                if i & bit == 0 {
                    edges.push((corner(i), corner(i | bit)));
                }
            }
        }
        size = (hi - lo).len() * 0.15;
    }
    (edges, [(o, o + x * size), (o, o + y * size), (o, o + z * size)])
}

/// The screen segments of the toolpaths: `(from, to, cutting)`.
pub fn segments(app: &SolveApp, proj: &Proj) -> Vec<(egui::Pos2, egui::Pos2, bool)> {
    let mut out = Vec::new();
    if !(app.manufacture.open && app.manufacture.show_paths) {
        return out;
    }
    for path in app.manufacture.cache.iter().flat_map(|(_, p, _)| p) {
        for (a, b) in path.iter().zip(path.iter().skip(1)) {
            if out.len() >= MAX_SEGMENTS {
                return out;
            }
            if let (Some(from), Some(to)) = (proj.to_screen(a.0), proj.to_screen(b.0)) {
                out.push((from, to, b.1));
            }
        }
    }
    out
}

/// Draws the toolpaths over the model: cuts in the accent colour, rapids thin and dim.
pub fn draw(app: &SolveApp, painter: &egui::Painter, proj: &Proj) {
    let t = Tokens::get();
    if app.manufacture.open && app.manufacture.show_setups {
        for view in app.manufacture.cache.iter().flat_map(|(_, _, v)| v) {
            let (edges, arms) = setup_lines(view);
            for (a, b) in edges {
                if let (Some(a), Some(b)) = (proj.to_screen(a), proj.to_screen(b)) {
                    painter.line_segment([a, b], Stroke::new(1.0, t.text_dim));
                }
            }
            for ((a, b), (label, colour)) in arms.into_iter().zip([("X", t.error), ("Y", t.sketch_accent), ("Z", t.accent)]) {
                if let (Some(a), Some(b)) = (proj.to_screen(a), proj.to_screen(b)) {
                    painter.line_segment([a, b], Stroke::new(2.0, colour));
                    painter.text(b, egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(12.0), colour);
                }
            }
        }
    }
    for (from, to, cutting) in segments(app, proj) {
        let stroke = if cutting { Stroke::new(1.5, t.accent) } else { Stroke::new(1.0, t.warning.gamma_multiply(0.7)) };
        painter.line_segment([from, to], stroke);
    }
}

/// A labelled text box in a form's grid; `true` when its text changed this frame.
fn field(ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str) -> bool {
    ui.label(label);
    let r = ui.add(egui::TextEdit::singleline(value).hint_text(hint).desired_width(140.0));
    let changed = r.changed() | crate::params_dialog::complete(ui, &r, value);
    ui.end_row();
    changed
}

/// A number as forms show it: at most `decimals` decimals, none wasted.
fn shown(v: f64, decimals: usize) -> String {
    let s = format!("{v:.decimals$}");
    if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s }
}

fn parsed(s: &str) -> Option<f64> {
    s.trim().parse::<f64>().ok().filter(|v| v.is_finite() && *v > 0.0)
}

/// Keeps feed and chip load in step: feed = speed × flutes × chip load. A changed chip load,
/// speed or flute count works the feed out; a changed feed works the chip load out.
fn link_chip_load(rpm: &str, flutes: Option<f64>, feed: &mut String, chip: &mut String, speed_changed: bool, feed_changed: bool, chip_changed: bool) {
    let (Some(rpm), Some(flutes)) = (parsed(rpm), flutes.filter(|z| *z >= 1.0)) else { return };
    if feed_changed {
        *chip = parsed(feed).map(|f| shown(f / (rpm * flutes), 4)).unwrap_or_default();
    } else if chip_changed || speed_changed {
        if let Some(c) = parsed(chip) {
            *feed = shown(rpm * flutes * c, 1);
        } else if chip_changed {
            feed.clear();
        }
    }
}

fn choice(ui: &mut egui::Ui, id: &str, label: &str, index: &mut usize, options: &[&str]) {
    ui.label(label);
    egui::ComboBox::from_id_salt(id).selected_text(options.get(*index).copied().unwrap_or_default()).show_ui(ui, |ui| {
        for (i, o) in options.iter().enumerate() {
            ui.selectable_value(index, i, *o);
        }
    });
    ui.end_row();
}

/// A text value chosen from a list (`(value, label)`); a value not in the list is shown as it is.
fn pick(ui: &mut egui::Ui, id: &str, label: &str, value: &mut String, options: &[(String, String)]) {
    ui.label(label);
    let shown = options.iter().find(|(v, _)| v == value).map_or(value.clone(), |(_, l)| l.clone());
    egui::ComboBox::from_id_salt(id).selected_text(shown).width(140.0).show_ui(ui, |ui| {
        for (v, l) in options {
            ui.selectable_value(value, v.clone(), l);
        }
    });
    ui.end_row();
}

/// A tool's cutting data, as forms fill it in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ToolData {
    diameter: f64,
    flutes: u32,
    spindle_rpm: Option<f64>,
    feed: Option<f64>,
    plunge_feed: Option<f64>,
    stepdown: Option<f64>,
    stepover: Option<f64>,
}

/// What an operation form chooses from.
#[derive(Default)]
struct Choices {
    /// Sketch names, newest first.
    sketches: Vec<(String, String)>,
    /// Tool numbers with what they are: this design's tools, then the library's others.
    tools: Vec<(String, String)>,
    /// Each tool's cutting data, by number.
    data: Vec<(String, ToolData)>,
}

fn choices(app: &SolveApp, listed: &Value, library: &Value) -> Choices {
    let sketches = app
        .session
        .doc
        .features
        .iter()
        .rev()
        .filter(|f| matches!(f.kind, solidcraft_engine::doc::FeatureKind::Sketch { .. }))
        .map(|f| (f.name.clone(), f.name.clone()))
        .collect();
    let (mut tools, mut data) = (Vec::new(), Vec::new());
    // The design's own copy of a tool is the one its operations cut with.
    for t in listed["tools"].as_array().into_iter().flatten().chain(library["tools"].as_array().into_iter().flatten()) {
        let (Some(n), Some(diameter)) = (t["number"].as_u64(), t["diameter"].as_f64()) else { continue };
        let n = n.to_string();
        if tools.iter().any(|(number, _): &(String, String)| *number == n) {
            continue;
        }
        let name = text(t, "description");
        let what = if name.is_empty() { text(t, "kind").replace('_', " ") } else { name };
        tools.push((n.clone(), format!("T{n}  D{diameter} {what}")));
        let value = |k: &str| t[k].as_f64();
        data.push((
            n,
            ToolData {
                diameter,
                flutes: t["flutes"].as_u64().unwrap_or(0) as u32,
                spindle_rpm: value("spindle_rpm"),
                feed: value("feed"),
                plunge_feed: value("plunge_feed"),
                stepdown: value("stepdown"),
                stepover: value("stepover"),
            },
        ));
    }
    Choices { sketches, tools, data }
}

fn form_ui(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices) {
    egui::Grid::new("sc_cam_form").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| match form {
        Form::Tool(f) => {
            if f.editing {
                ui.label("Number");
                ui.label(format!("T{}", f.number));
                ui.end_row();
            } else {
                field(ui, "Number", &mut f.number, "1");
            }
            choice(ui, "sc_cam_tool_kind", "Kind", &mut f.kind, &TOOL_KINDS);
            field(ui, "Diameter", &mut f.diameter, "6 mm");
            field(ui, "Description", &mut f.description, "");
            field(ui, "Flute length", &mut f.flute_length, "3 × diameter");
            // Speed, flutes, chip load and feed belong together: feed = speed × flutes × chip load.
            let flutes_changed = field(ui, "Flutes", &mut f.flutes, "2");
            let speed_changed = field(ui, "Spindle (rev/min)", &mut f.spindle_rpm, "none");
            let chip_changed = field(ui, "Chip load (mm/tooth)", &mut f.chip_load, "feed ÷ (speed × flutes)");
            let feed_changed = field(ui, "Feed (mm/min)", &mut f.feed, "speed × flutes × chip load");
            let flutes = parsed(&f.flutes);
            link_chip_load(&f.spindle_rpm, flutes, &mut f.feed, &mut f.chip_load, speed_changed || flutes_changed, feed_changed, chip_changed);
            field(ui, "Plunge feed (mm/min)", &mut f.plunge_feed, "feed / 3");
            field(ui, "Stepdown (mm)", &mut f.stepdown, "half the diameter");
            field(ui, "Stepover (mm)", &mut f.stepover, "by operation");
        }
        Form::Setup(f) => {
            field(ui, "Name", &mut f.name, "automatic");
            choice(ui, "sc_cam_setup_axis", "Tool axis", &mut f.axis, &AXES);
            field(ui, "Stock on the sides", &mut f.side, "0 mm");
            field(ui, "Stock on top", &mut f.top, "0 mm");
            field(ui, "Stock below", &mut f.bottom, "0 mm");
            choice(ui, "sc_cam_setup_origin", "Origin on", &mut f.origin, &ORIGINS);
            if f.origin == 2 {
                let [x, y, z] = &mut f.point;
                field(ui, "Origin X", x, "0");
                field(ui, "Origin Y", y, "0");
                field(ui, "Origin Z", z, "0");
            } else {
                // Where on the box, along each of the setup's own axes.
                let [x, y, z] = &mut f.anchors;
                choice(ui, "sc_cam_setup_x", "Origin X", x, &["min", "center", "max"]);
                choice(ui, "sc_cam_setup_y", "Origin Y", y, &["min", "center", "max"]);
                choice(ui, "sc_cam_setup_z", "Origin Z", z, &["bottom", "middle", "top"]);
            }
            choice(ui, "sc_cam_setup_post", "Post", &mut f.post, solidcraft_engine::cam::Post::BUILTIN);
        }
        Form::Op(f) => {
            field(ui, "Name", &mut f.name, "automatic");
            if f.faces.is_empty() {
                // Facing covers a box: the stock's, the part's, or the one round a sketch's profiles.
                if f.kind == OpKind::Face {
                    choice(ui, "sc_cam_face_boundary", "Boundary", &mut f.boundary, &FACE_BOUNDS);
                }
                if f.kind != OpKind::Face || f.boundary == 2 {
                    pick(ui, "sc_cam_sketch", "Sketch", &mut f.sketch, &choices.sketches);
                }
            } else {
                ui.label("Faces");
                ui.horizontal(|ui| {
                    ui.label(format!("{} picked", f.faces.len()));
                    if ui.small_button("Use a sketch").clicked() {
                        f.faces.clear();
                    }
                });
                ui.end_row();
            }
            if !selected.is_empty() && selected != f.faces.as_slice() {
                ui.label("");
                if ui.small_button(format!("Cut the {} selected face(s)", selected.len())).clicked() {
                    f.faces = selected.to_vec();
                }
                ui.end_row();
            }
            let from_faces = !f.faces.is_empty();
            pick(ui, "sc_cam_tool", "Tool", &mut f.tool, &choices.tools);
            // Another tool brings its own speed, feeds, stepdown and stepover; all stay editable here.
            prefill(f, choices);
            let speed_changed = field(ui, "Spindle (rev/min)", &mut f.spindle_rpm, "the tool's");
            let chip_changed = field(ui, "Chip load (mm/tooth)", &mut f.chip_load, "feed ÷ (speed × flutes)");
            let feed_changed = field(ui, "Feed (mm/min)", &mut f.feed, "the tool's");
            let flutes = Some(f64::from(f.flutes));
            link_chip_load(&f.spindle_rpm, flutes, &mut f.feed, &mut f.chip_load, speed_changed, feed_changed, chip_changed);
            field(ui, "Plunge feed (mm/min)", &mut f.plunge_feed, "the tool's, or feed / 3");
            field(ui, "Top", &mut f.top, if from_faces { "from the faces" } else { "the sketch plane" });
            field(ui, "Bottom", &mut f.bottom, if from_faces { "from the faces" } else { "-5 mm" });
            if f.kind != OpKind::Drill {
                field(ui, "Stepdown", &mut f.stepdown, "half the tool diameter");
            }
            match f.kind {
                OpKind::Face => {
                    field(ui, "Stepover", &mut f.stepover, "the tool's, or half its diameter");
                    choice(ui, "sc_cam_face_limit", "Cut", &mut f.limit, &["over the boundary", "inside the boundary"]);
                    field(
                        ui,
                        if f.limit == 0 { "Over by" } else { "Inside by" },
                        &mut f.margin,
                        if f.limit == 0 { "the tool's radius" } else { "0 mm" },
                    );
                }
                OpKind::Contour => {
                    choice(ui, "sc_cam_side", "Side", &mut f.side, &SIDES);
                    field(ui, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                }
                OpKind::Pocket => {
                    field(ui, "Stepover", &mut f.stepover, "the tool's, or 40% of its diameter");
                    field(ui, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                }
                OpKind::Drill => {
                    field(ui, "Peck depth", &mut f.peck, "one pass");
                    field(ui, "Dwell (s)", &mut f.dwell, "0");
                }
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket) {
                field(ui, "Ramp angle", &mut f.ramp_angle, "3 deg (0: plunge)");
                ui.label("Direction");
                ui.checkbox(&mut f.climb, "Climb");
                ui.end_row();
            }
            ui.label("Coolant");
            ui.checkbox(&mut f.coolant, "On");
            ui.end_row();
        }
    });
}

/// The form's title, grid and OK / Cancel; `true` when OK was pressed.
fn form_block(ui: &mut egui::Ui, cam: &mut CamUi, selected: &[(String, Vec3)], lists: &Choices) -> bool {
    let Some(form) = &mut cam.form else { return false };
    ui.add_space(6.0);
    let title = match form {
        Form::Tool(f) if f.editing => "EDIT TOOL",
        Form::Tool(_) => "NEW TOOL",
        Form::Setup(f) if f.editing.is_some() => "EDIT SETUP",
        Form::Setup(_) => "NEW SETUP",
        Form::Op(f) if f.editing.is_some() => "EDIT OPERATION",
        Form::Op(f) => f.kind.title(),
    };
    let mut ok = false;
    let mut cancel = false;
    egui::Frame::group(ui.style()).inner_margin(10.0).show(ui, |ui| {
        ui.label(RichText::new(title).strong());
        ui.add_space(4.0);
        form_ui(ui, form, selected, lists);
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            ok = ui.button("OK").clicked();
            cancel = ui.button("Cancel").clicked();
        });
    });
    if cancel {
        cam.form = None;
        cam.error = None;
    }
    ok
}

/// What a window asks for once it has been drawn.
#[derive(Default)]
struct Asked {
    run: Vec<(&'static str, Value)>,
    new_tool: bool,
    form: Option<Form>,
    /// An operation to open for editing: its setup and itself, as listed.
    edit_op: Option<(Value, Value)>,
    apply: bool,
    simulate: Option<u64>,
    /// Write a program: the setup, one operation of it or all (`None`), and the setup's post.
    post: Option<(u64, Option<(u64, String)>, String)>,
    import: bool,
    export: bool,
}

/// The tool table's columns.
const TOOL_HEADS: [&str; 7] = ["Tool", "Kind", "Diameter", "Flutes", "Spindle", "Chip load", "Feed"];

/// A table of tools, each row picked by clicking its name. `library`: these are the machine library's tools.
fn tool_table(ui: &mut egui::Ui, id: &str, tools: &[Value], library: bool, picked: &mut Option<(bool, u64)>) {
    let t = Tokens::get();
    egui::ScrollArea::both().id_salt(id).max_height(220.0).auto_shrink([false, true]).show(ui, |ui| {
        egui::Grid::new(id).num_columns(TOOL_HEADS.len() + 1).spacing([18.0, 6.0]).striped(true).show(ui, |ui| {
            for head in TOOL_HEADS {
                ui.label(RichText::new(head).color(t.text_dim));
            }
            ui.end_row();
            for tool in tools {
                let number = tool["number"].as_u64().unwrap_or(0);
                let data = |k: &str, decimals: usize| tool[k].as_f64().map_or("—".to_string(), |v| shown(v, decimals));
                let name = text(tool, "description");
                let label = if name.is_empty() { format!("T{number}") } else { format!("T{number}  {name}") };
                if ui.selectable_label(*picked == Some((library, number)), label).clicked() {
                    *picked = if *picked == Some((library, number)) { None } else { Some((library, number)) };
                }
                ui.label(text(tool, "kind").replace('_', " "));
                ui.label(format!("{} mm", data("diameter", 3)));
                ui.label(data("flutes", 0));
                ui.label(data("spindle_rpm", 0));
                ui.label(data("chip_load", 4));
                ui.label(data("feed", 1));
                // How this design's copy of a library tool stands.
                match tool["design"].as_str() {
                    Some("same") => ui.label(RichText::new("in this design").color(t.text_dim)),
                    Some("differs") => ui.label(RichText::new("changed since this design copied it").color(t.warning)),
                    _ => ui.label(""),
                };
                ui.end_row();
            }
        });
    });
}

/// The Tool Library window: the machine's tools, which every design can use. Pick a tool to
/// change or remove it; a form appears only for New Tool and Edit.
fn tools_window(ctx: &egui::Context, cam: &mut CamUi, library: &Value, lists: &Choices, corner: egui::Pos2, asked: &mut Asked) {
    let t = Tokens::get();
    let mut open = true;
    crate::frame::window(ctx, RichText::new("TOOL LIBRARY").strong().size(13.0), crate::frame::Width::Wide)
        .id(egui::Id::new("sc_cam_tools"))
        // From the top of the viewport down, beside the Manufacture window where there is room.
        .pivot(egui::Align2::LEFT_TOP)
        .default_pos(corner)
        .default_width(crate::frame::WINDOW_WIDE_MAX_W - 40.0)
        .open(&mut open)
        .resizable(true)
        .show(ctx, |ui| {
            // Everything is as wide as the window, never wider: tables scroll inside it.
            ui.set_max_width(ui.available_width());
            if let Some(e) = cam.error.as_ref().filter(|_| !cam.open) {
                ui.label(RichText::new(e).color(t.error));
            }
            if let Some(e) = library["error"].as_str() {
                ui.label(RichText::new(e).color(t.error));
            }
            let tools = library["tools"].as_array().cloned().unwrap_or_default();
            let own = library["only_in_design"].as_array().cloned().unwrap_or_default();
            // A pick that is no longer in either list is forgotten.
            let listed = |in_library: bool, number: u64| if in_library { &tools } else { &own }.iter().find(|x| x["number"].as_u64() == Some(number));
            if cam.selected_tool.is_some_and(|(in_library, number)| listed(in_library, number).is_none()) {
                cam.selected_tool = None;
            }
            egui::Frame::group(ui.style()).inner_margin(10.0).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("This machine's tools").strong());
                    ui.label(RichText::new("shared by every design").color(t.text_dim)).on_hover_text(text(library, "path"));
                });
                ui.add_space(4.0);
                if tools.is_empty() {
                    ui.label(RichText::new("No tools yet. New Tool adds the first.").color(t.text_dim));
                } else {
                    tool_table(ui, "sc_cam_tool_table", &tools, true, &mut cam.selected_tool);
                }
            });
            if !own.is_empty() {
                ui.add_space(6.0);
                egui::Frame::group(ui.style()).inner_margin(10.0).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new("Only in this design").strong());
                        ui.label(RichText::new("not in the machine's library").color(t.text_dim));
                    });
                    ui.add_space(4.0);
                    tool_table(ui, "sc_cam_design_tools", &own, false, &mut cam.selected_tool);
                });
            }
            ui.add_space(6.0);
            // What can be done: always New Tool and the library file; with a tool picked, to that tool.
            ui.horizontal_wrapped(|ui| {
                if ui.button("New Tool").clicked() {
                    asked.new_tool = true;
                }
                if let Some((in_library, number)) = cam.selected_tool
                    && let Some(tool) = listed(in_library, number)
                {
                    ui.separator();
                    ui.label(RichText::new(format!("T{number}")).strong());
                    if ui.button("Edit").clicked() {
                        asked.form = tool_form(tool, in_library);
                    }
                    if ui.button("Remove").clicked() {
                        let id = if in_library { "cam.library.remove" } else { "cam.tool.remove" };
                        asked.run.push((id, json!({ "number": number })));
                    }
                    if !in_library && ui.button("Add to library").clicked() {
                        asked.run.push(("cam.library.add", json!({ "number": number })));
                    }
                    if tool["design"].as_str() == Some("differs")
                        && ui.button("Update this design").on_hover_text("Replace this design's older copy of the tool with the library's").clicked()
                    {
                        asked.run.push(("cam.tool.update", json!({ "number": number })));
                    }
                } else {
                    ui.label(RichText::new("Pick a tool to change or remove it").color(t.text_dim));
                }
                ui.separator();
                asked.import = ui.button("Import…").on_hover_text("Add the tools of a library file to this machine's library").clicked();
                asked.export = ui.button("Export…").on_hover_text("Write this machine's library to a file").clicked();
            });
            if matches!(cam.form, Some(Form::Tool(_))) {
                asked.apply |= form_block(ui, cam, &[], lists);
            }
        });
    cam.tools_open = open;
}

/// The Manufacture window: setups with their operations, what is drawn, and the open setup or operation form.
fn manufacture_window(
    ctx: &egui::Context,
    cam: &mut CamUi,
    listed: &Value,
    lists: &Choices,
    selected: &[(String, Vec3)],
    corner: egui::Pos2,
    asked: &mut Asked,
) {
    let t = Tokens::get();
    let mut open = true;
    crate::frame::window(ctx, RichText::new("MANUFACTURE").strong().size(13.0), crate::frame::Width::Normal)
        .id(egui::Id::new("sc_cam_window"))
        // Beside the model, not over it.
        .pivot(egui::Align2::LEFT_TOP)
        .default_pos(corner)
        .open(&mut open)
        .resizable(true)
        .show(ctx, |ui| {
            if let Some(e) = &cam.error {
                ui.label(RichText::new(e).color(t.error));
            }
            ui.horizontal(|ui| {
                if ui.button("Tool Library…").clicked() {
                    cam.tools_open = true;
                }
                ui.checkbox(&mut cam.show_paths, "Toolpaths");
                ui.checkbox(&mut cam.show_setups, "Stock and origin");
                if cam.stock.is_some() {
                    ui.checkbox(&mut cam.show_stock, "Simulated stock");
                }
            });
            ui.add_space(4.0);
            let setups = listed["setups"].as_array().cloned().unwrap_or_default();
            if setups.is_empty() {
                ui.label(RichText::new("No setup yet").color(t.text_dim));
            }
            for setup in &setups {
                let id = setup["id"].as_u64().unwrap_or(0);
                // Each setup in a frame of its own, with its operations.
                egui::Frame::group(ui.style()).inner_margin(8.0).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(text(setup, "name")).strong());
                        ui.label(RichText::new(format!("post: {}", text(setup, "post"))).color(t.text_dim));
                        if ui.small_button("Edit").clicked() {
                            asked.form = setup_form(setup);
                        }
                        let has_ops = setup["operations"].as_array().is_some_and(|o| !o.is_empty());
                        if ui.add_enabled(has_ops, egui::Button::new("Simulate").small()).clicked() {
                            asked.simulate = Some(id);
                        }
                        if ui.small_button("Remove").clicked() {
                            asked.run.push(("cam.setup.remove", json!({ "setup": id })));
                        }
                    });
                    if let Some(e) = setup.get("error").and_then(Value::as_str) {
                        ui.label(RichText::new(e).color(t.error));
                    }
                    for op in setup["operations"].as_array().into_iter().flatten() {
                        ui.horizontal(|ui| {
                            ui.add_space(12.0);
                            ui.label(format!("{}  ({}, T{})", text(op, "name"), text(op, "kind"), op["tool"].as_u64().unwrap_or(0)));
                            match op.get("error").and_then(Value::as_str) {
                                Some(e) => {
                                    ui.label(RichText::new(e).color(t.error));
                                }
                                None => {
                                    ui.label(
                                        RichText::new(format!("{:.1} min", op["toolpath"]["cut_minutes"].as_f64().unwrap_or(0.0))).color(t.text_dim),
                                    );
                                }
                            }
                            for (label, by) in [("Up", -1), ("Down", 1)] {
                                if ui.small_button(label).clicked() {
                                    asked.run.push(("cam.op.move", json!({ "setup": id, "op": op["id"], "by": by })));
                                }
                            }
                            if ui.small_button("Edit").clicked() {
                                asked.edit_op = Some((setup.clone(), op.clone()));
                            }
                            if ui
                                .add_enabled(op.get("error").is_none(), egui::Button::new("Post").small())
                                .on_hover_text("Write this operation alone as G-code")
                                .clicked()
                                && let Some(op_id) = op["id"].as_u64()
                            {
                                asked.post = Some((id, Some((op_id, text(op, "name"))), text(setup, "post")));
                            }
                            if ui.small_button("Remove").clicked() {
                                asked.run.push(("cam.op.remove", json!({ "setup": id, "op": op["id"] })));
                            }
                        });
                    }
                    // The setup's whole program: the one thing everything here leads to.
                    ui.add_space(6.0);
                    let has_ops = setup["operations"].as_array().is_some_and(|o| !o.is_empty());
                    let label = RichText::new(format!("Post Process  ({})", text(setup, "post"))).strong().size(14.0).color(t.text);
                    let button = egui::Button::new(label).fill(t.accent_soft).min_size(egui::vec2(ui.available_width(), 32.0));
                    if ui.add_enabled(has_ops, button).on_hover_text("Write every operation of this setup, in order, as one G-code program").clicked()
                    {
                        asked.post = Some((id, None, text(setup, "post")));
                    }
                });
                ui.add_space(4.0);
            }
            if let Some((found, warnings)) = &cam.simulation {
                ui.separator();
                for line in found {
                    ui.label(RichText::new(line).color(t.text_dim));
                }
                for line in warnings {
                    ui.label(RichText::new(line).color(t.warning));
                }
            }
            if matches!(cam.form, Some(Form::Setup(_) | Form::Op(_))) {
                asked.apply |= form_block(ui, cam, selected, lists);
            }
        });
    cam.open = open;
}

/// Draw the windows that are open, and keep what the viewport shows up to date. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    if !app.manufacture.open && !app.manufacture.tools_open {
        return;
    }
    if app.manufacture.open && (app.manufacture.show_paths || app.manufacture.show_setups) {
        refresh(app);
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let selected = selected_faces(app);
    let library = library_listing(app, false);
    let lists = choices(app, &listed, &library);
    let corner = app.viewport.rect.map_or(egui::pos2(16.0, 160.0), |r| r.left_top() + egui::vec2(12.0, 12.0));
    let mut asked = Asked::default();
    let mut cam = std::mem::take(&mut app.manufacture);
    if cam.open {
        manufacture_window(ctx, &mut cam, &listed, &lists, &selected, corner, &mut asked);
    }
    if cam.tools_open {
        let beside = app.viewport.rect.map_or(corner, |r| {
            let x = r.left() + 12.0 + crate::frame::WINDOW_MAX_W + 12.0;
            let fits = x + crate::frame::WINDOW_WIDE_MAX_W <= r.right();
            egui::pos2(if fits { x } else { r.left() + 36.0 }, r.top() + if fits { 12.0 } else { 36.0 })
        });
        tools_window(ctx, &mut cam, &library, &lists, beside, &mut asked);
    }
    app.manufacture = cam;
    for (id, p) in std::mem::take(&mut asked.run) {
        app.manufacture.error = run(app, id, p).err();
    }
    if asked.new_tool {
        start(app, "cam.tool.define");
    }
    if let Some((setup, op)) = asked.edit_op {
        asked.form = edit_form(app, &setup, &op);
    }
    if let Some(form) = asked.form {
        app.manufacture.form = Some(form);
        app.manufacture.error = None;
    }
    if asked.apply {
        apply(app);
    }
    if let Some(setup) = asked.simulate {
        simulate_setup(app, setup);
    }
    if asked.import
        && let Some(path) = app.services.pick_open.as_ref().and_then(|f| f())
    {
        app.manufacture.error = run(app, "cam.library.import", json!({ "path": path })).err();
    }
    if asked.export
        && let Some(path) = app.services.pick_save.as_ref().and_then(|f| f("tools.json", &["json"]))
    {
        app.manufacture.error = run(app, "cam.library.export", json!({ "path": path })).err();
    }
    if let Some((setup, op, post)) = asked.post {
        let extension = solidcraft_engine::cam::Post::builtin(&post).map(|p| p.extension).unwrap_or_else(|| "nc".into());
        // A file named after what is in it.
        let stem: String =
            op.as_ref().map_or("program", |(_, name)| name.as_str()).chars().filter(|c| c.is_ascii_alphanumeric() || "-_ ".contains(*c)).collect();
        let stem = if stem.trim().is_empty() { "program".to_string() } else { stem };
        if let Some(path) = app.services.pick_save.as_ref().and_then(|f| f(&format!("{stem}.{extension}"), &[extension.as_str()])) {
            post_to(app, setup, op.map(|(id, _)| id), &path);
        }
    }
}

/// Writes a setup's program, or one operation of it, to `path`.
pub fn post_to(app: &mut SolveApp, setup: u64, op: Option<u64>, path: &str) {
    let mut p = json!({ "setup": setup, "path": path });
    if let Some(op) = op {
        p["op"] = json!(op);
    }
    match app.run("cam.post", p) {
        Ok(v) => {
            app.manufacture.error = None;
            app.set_status(format!("Wrote {} lines of G-code", v["lines"].as_u64().unwrap_or(0)), false);
        }
        Err(e) => app.manufacture.error = Some(e),
    }
}

#[cfg(test)]
#[path = "cam_ui_tests.rs"]
mod tests;
