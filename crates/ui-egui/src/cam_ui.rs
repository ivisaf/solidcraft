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
    Tool { number: String, kind: usize, diameter: String, flute_length: String, description: String },
    Setup { name: String, origin: [String; 3], post: usize },
    Op(Box<OpForm>),
}

/// The operation a form edits, and the geometry it had when the form opened.
#[derive(Clone, Debug, PartialEq)]
pub struct Editing {
    pub setup: u64,
    pub op: u64,
    pub sketch: String,
    pub faces: Vec<(String, Vec3)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpForm {
    pub kind: OpKind,
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
        (nx.checked_mul(ny) == Some(z.len()) && (deviation.is_empty() || deviation.len() == z.len())).then_some(StockField {
            origin: [v["origin"][0].as_f64()?, v["origin"][1].as_f64()?],
            cell: v["cell"].as_f64()?,
            nx,
            ny,
            floor: v["floor"].as_f64()?,
            z,
            deviation,
        })
    }

    /// The top of column `(i, j)`: its centre at its height, and its deviation from the part.
    fn column(&self, i: usize, j: usize) -> Option<(Vec3, f64)> {
        let k = j.checked_mul(self.nx)?.checked_add(i)?;
        let z = *self.z.get(k).filter(|_| i < self.nx && j < self.ny)?;
        let p = Vec3::new(self.origin[0] + (i as f64 + 0.5) * self.cell, self.origin[1] + (j as f64 + 0.5) * self.cell, z);
        Some((p, self.deviation.get(k).copied().unwrap_or(0.0)))
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
            out.push(([a.0, b.0, c.0], mark([a.1, b.1, c.1])));
            out.push(([a.0, c.0, d.0], mark([a.1, c.1, d.1])));
        }
    }
    // Walls round the edge, outward-facing: along each side, from the top's rim down to the floor.
    let down = |p: Vec3| Vec3::new(p.x, p.y, field.floor);
    let mut wall = |a: Option<(Vec3, f64)>, b: Option<(Vec3, f64)>| {
        if let (Some((a, _)), Some((b, _))) = (a, b) {
            out.push(([a, down(a), down(b)], StockMark::Fine));
            out.push(([a, down(b), b], StockMark::Fine));
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

pub struct CamUi {
    pub open: bool,
    pub form: Option<Form>,
    pub show_paths: bool,
    /// Index into the engine's post list.
    pub post: usize,
    pub error: Option<String>,
    /// The last simulation: what it found, and its warnings.
    pub simulation: Option<(Vec<String>, Vec<String>)>,
    /// The stock as the last simulation left it, drawn in place of the bodies.
    pub stock: Option<StockField>,
    pub show_stock: bool,
    /// Counts changes of `stock` (the viewport redraws when it moves on).
    stock_stamp: u64,
    /// The toolpaths of every setup, for the design revision they were worked out at.
    cache: Option<(u64, Vec<Path>)>,
}

impl Default for CamUi {
    fn default() -> Self {
        CamUi {
            open: false,
            form: None,
            show_paths: true,
            post: 0,
            error: None,
            simulation: None,
            stock: None,
            show_stock: true,
            stock_stamp: 0,
            cache: None,
        }
    }
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
    let first_tool = listed["tools"].get(0).and_then(|t| t["number"].as_u64()).map(|n| n.to_string()).unwrap_or_default();
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
            editing: None,
            name: String::new(),
            bottom: if faces.is_empty() { "-5 mm".into() } else { String::new() },
            faces,
            sketch,
            tool: first_tool.clone(),
            spindle_rpm: "10000".into(),
            feed: "600".into(),
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
    let next_number = (1u64..).find(|n| !listed["tools"].as_array().is_some_and(|a| a.iter().any(|t| t["number"].as_u64() == Some(*n)))).unwrap_or(1);
    let form = match id {
        "cam.tool.define" => Some(Form::Tool {
            number: next_number.to_string(),
            kind: 0,
            diameter: "6 mm".into(),
            flute_length: String::new(),
            description: String::new(),
        }),
        "cam.setup.create" => Some(Form::Setup { name: String::new(), origin: ["0".into(), "0".into(), "0".into()], post: 0 }),
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
    app.manufacture.open = true;
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
        Form::Tool { number: n, kind, diameter, flute_length, description } => {
            optional(&mut p, "diameter", diameter);
            optional(&mut p, "flute_length", flute_length);
            optional(&mut p, "description", description);
            p.insert("number".into(), json!(number("The tool number", n)? as u64));
            p.insert("kind".into(), json!(TOOL_KINDS.get(*kind).copied().unwrap_or("flat")));
            Ok(("cam.tool.define", Value::Object(p)))
        }
        Form::Setup { name, origin, post } => {
            optional(&mut p, "name", name);
            let [x, y, z] = origin;
            p.insert("origin".into(), json!([number("The origin's X", x)?, number("The origin's Y", y)?, number("The origin's Z", z)?]));
            p.insert("post".into(), json!(solidcraft_engine::cam::Post::BUILTIN.get(*post).copied().unwrap_or("rs274")));
            Ok(("cam.setup.create", Value::Object(p)))
        }
        Form::Op(f) => {
            optional(&mut p, "name", &f.name);
            optional(&mut p, "stepdown", &f.stepdown);
            match &f.editing {
                // A new operation: what is empty is left to the command.
                None => {
                    if f.faces.is_empty() {
                        optional(&mut p, "sketch", &f.sketch);
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
                    if f.faces.is_empty() && (f.sketch.trim() != e.sketch || !e.faces.is_empty()) {
                        p.insert("sketch".into(), json!(f.sketch.trim()));
                    }
                    p.insert("op".into(), json!(e.op));
                    p.insert("setup".into(), json!(e.setup));
                }
            }
            match f.kind {
                OpKind::Face => optional(&mut p, "stepover", &f.stepover),
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
            p.insert("spindle_rpm".into(), json!(number("The spindle speed", &f.spindle_rpm)?));
            p.insert("feed".into(), json!(number("The feed", &f.feed)?));
            if !f.plunge_feed.trim().is_empty() {
                p.insert("plunge_feed".into(), json!(number("The plunge feed", &f.plunge_feed)?));
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
    let sketch = op["geometry"]["sketch"].as_u64().and_then(|id| app.session.doc.feature(id)).map(|f| f.name.clone()).unwrap_or_default();
    let faces: Vec<(String, Vec3)> = op["geometry"]["faces"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|f| Some((f["body"].as_str()?.to_string(), Vec3::new(f["at"]["x"].as_f64()?, f["at"]["y"].as_f64()?, f["at"]["z"].as_f64()?))))
        .collect();
    let inputs = &op["inputs"];
    Some(Form::Op(Box::new(OpForm {
        kind,
        editing: Some(Editing { setup: setup["id"].as_u64()?, op: op["id"].as_u64()?, sketch: sketch.clone(), faces: faces.clone() }),
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

/// Runs the open form's command; the form closes when the command succeeds.
pub fn apply(app: &mut SolveApp) {
    let Some(form) = app.manufacture.form.clone() else { return };
    match command(&form).and_then(|(id, p)| app.run(id, p)) {
        Ok(_) => {
            app.manufacture.form = None;
            app.manufacture.error = None;
        }
        Err(e) => app.manufacture.error = Some(e),
    }
}

/// Runs `cam.simulate` on the first setup and keeps what it found for the window.
pub fn simulate(app: &mut SolveApp) {
    let Some(setup) = app.session.doc.manufacture.setups.first().map(|s| s.id) else {
        app.manufacture.error = Some("There is no setup to simulate".into());
        return;
    };
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

/// The toolpaths to draw, worked out again when the design has changed.
pub fn paths(app: &mut SolveApp) -> &[Path] {
    let revision = app.session.revision;
    if app.manufacture.cache.as_ref().is_none_or(|(r, _)| *r != revision) {
        let setups: Vec<u64> = app.session.doc.manufacture.setups.iter().map(|s| s.id).collect();
        let mut all = Vec::new();
        for setup in setups {
            // A setup with an operation that cannot be cut draws nothing; the window says why.
            let Ok(v) = app.session.execute("cam.toolpath", &json!({ "setup": setup })) else { continue };
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
        // `cam.toolpath` is not a change, but keep the key the revision as it is now.
        app.manufacture.cache = Some((app.session.revision, all));
    }
    app.manufacture.cache.as_ref().map_or(&[], |(_, p)| p.as_slice())
}

/// The screen segments of the toolpaths: `(from, to, cutting)`.
pub fn segments(app: &SolveApp, proj: &Proj) -> Vec<(egui::Pos2, egui::Pos2, bool)> {
    let mut out = Vec::new();
    if !(app.manufacture.open && app.manufacture.show_paths) {
        return out;
    }
    for path in app.manufacture.cache.iter().flat_map(|(_, p)| p) {
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
    for (from, to, cutting) in segments(app, proj) {
        let stroke = if cutting { Stroke::new(1.5, t.accent) } else { Stroke::new(1.0, t.warning.gamma_multiply(0.7)) };
        painter.line_segment([from, to], stroke);
    }
}

fn field(ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str) {
    ui.label(label);
    let r = ui.add(egui::TextEdit::singleline(value).hint_text(hint).desired_width(140.0));
    crate::params_dialog::complete(ui, &r, value);
    ui.end_row();
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

/// What an operation form chooses from.
#[derive(Default)]
struct Choices {
    /// Sketch names, newest first.
    sketches: Vec<(String, String)>,
    /// Tool numbers with what they are.
    tools: Vec<(String, String)>,
}

fn choices(app: &SolveApp, listed: &Value) -> Choices {
    let sketches = app
        .session
        .doc
        .features
        .iter()
        .rev()
        .filter(|f| matches!(f.kind, solidcraft_engine::doc::FeatureKind::Sketch { .. }))
        .map(|f| (f.name.clone(), f.name.clone()))
        .collect();
    let tools = listed["tools"]
        .as_array()
        .into_iter()
        .flatten()
        .filter_map(|t| {
            let n = t["number"].as_u64()?;
            Some((n.to_string(), format!("T{n}  D{} {}", t["diameter"].as_f64()?, text(t, "kind").replace('_', " "))))
        })
        .collect();
    Choices { sketches, tools }
}

fn form_ui(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices) {
    egui::Grid::new("sc_cam_form").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| match form {
        Form::Tool { number, kind, diameter, flute_length, description } => {
            field(ui, "Number", number, "1");
            choice(ui, "sc_cam_tool_kind", "Kind", kind, &TOOL_KINDS);
            field(ui, "Diameter", diameter, "6 mm");
            field(ui, "Flute length", flute_length, "3 × diameter");
            field(ui, "Description", description, "");
        }
        Form::Setup { name, origin, post } => {
            field(ui, "Name", name, "Setup1");
            let [x, y, z] = origin;
            field(ui, "Origin X", x, "0");
            field(ui, "Origin Y", y, "0");
            field(ui, "Origin Z", z, "0");
            choice(ui, "sc_cam_setup_post", "Post", post, solidcraft_engine::cam::Post::BUILTIN);
        }
        Form::Op(f) => {
            field(ui, "Name", &mut f.name, "automatic");
            if f.faces.is_empty() {
                pick(ui, "sc_cam_sketch", "Sketch", &mut f.sketch, &choices.sketches);
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
            field(ui, "Spindle (rev/min)", &mut f.spindle_rpm, "10000");
            field(ui, "Feed (mm/min)", &mut f.feed, "600");
            field(ui, "Plunge feed (mm/min)", &mut f.plunge_feed, "feed / 3");
            field(ui, "Top", &mut f.top, if from_faces { "from the faces" } else { "the sketch plane" });
            field(ui, "Bottom", &mut f.bottom, if from_faces { "from the faces" } else { "-5 mm" });
            if f.kind != OpKind::Drill {
                field(ui, "Stepdown", &mut f.stepdown, "half the tool diameter");
            }
            match f.kind {
                OpKind::Face => field(ui, "Stepover", &mut f.stepover, "5 mm"),
                OpKind::Contour => {
                    choice(ui, "sc_cam_side", "Side", &mut f.side, &SIDES);
                    field(ui, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                }
                OpKind::Pocket => {
                    field(ui, "Stepover", &mut f.stepover, "40% of the tool diameter");
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

/// Draw the window when it is open, and keep the toolpaths up to date. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    if !app.manufacture.open {
        return;
    }
    if app.manufacture.show_paths {
        paths(app);
    }
    let t = Tokens::get();
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let mut open = true;
    let mut run: Vec<(&'static str, Value)> = Vec::new();
    let mut apply_form = false;
    let mut run_simulation = false;
    let mut edit: Option<(Value, Value)> = None;
    let mut post_to: Option<String> = None;
    let selected = selected_faces(app);
    let lists = choices(app, &listed);
    let corner = app.viewport.rect.map_or(egui::pos2(16.0, 160.0), |r| r.left_top() + egui::vec2(12.0, 12.0));
    let mut cam = std::mem::take(&mut app.manufacture);
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
            ui.label(RichText::new("Tools").strong());
            let tools = listed["tools"].as_array().cloned().unwrap_or_default();
            if tools.is_empty() {
                ui.label(RichText::new("No tools yet").color(t.text_dim));
            }
            for tool in &tools {
                ui.horizontal(|ui| {
                    let number = tool["number"].as_u64().unwrap_or(0);
                    ui.label(format!(
                        "T{number}  D{} mm  {}  {}",
                        tool["diameter"].as_f64().unwrap_or(0.0),
                        text(tool, "kind").replace('_', " "),
                        text(tool, "description")
                    ));
                    if ui.small_button("Remove").clicked() {
                        run.push(("cam.tool.remove", json!({ "number": number })));
                    }
                });
            }
            ui.separator();
            let setups = listed["setups"].as_array().cloned().unwrap_or_default();
            if setups.is_empty() {
                ui.label(RichText::new("No setup yet").color(t.text_dim));
            }
            for setup in &setups {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(text(setup, "name")).strong());
                    ui.label(RichText::new(format!("post: {}", text(setup, "post"))).color(t.text_dim));
                    if ui.small_button("Remove").clicked() {
                        run.push(("cam.setup.remove", json!({ "setup": setup["id"] })));
                    }
                });
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
                                run.push(("cam.op.move", json!({ "setup": setup["id"], "op": op["id"], "by": by })));
                            }
                        }
                        if ui.small_button("Edit").clicked() {
                            edit = Some((setup.clone(), op.clone()));
                        }
                        if ui.small_button("Remove").clicked() {
                            run.push(("cam.op.remove", json!({ "setup": setup["id"], "op": op["id"] })));
                        }
                    });
                }
            }
            ui.separator();
            ui.horizontal(|ui| {
                ui.checkbox(&mut cam.show_paths, "Show toolpaths");
                let posts = solidcraft_engine::cam::Post::BUILTIN;
                egui::ComboBox::from_id_salt("sc_cam_post").selected_text(posts.get(cam.post).copied().unwrap_or_default()).show_ui(ui, |ui| {
                    for (i, name) in posts.iter().enumerate() {
                        ui.selectable_value(&mut cam.post, i, *name);
                    }
                });
                if ui.add_enabled(!setups.is_empty(), egui::Button::new("Post Process…")).clicked() {
                    post_to = posts.get(cam.post).map(|p| p.to_string());
                }
                run_simulation = ui.add_enabled(!setups.is_empty(), egui::Button::new("Simulate")).clicked();
            });
            if cam.stock.is_some() {
                ui.checkbox(&mut cam.show_stock, "Show simulated stock");
            }
            if let Some((found, warnings)) = &cam.simulation {
                for line in found {
                    ui.label(RichText::new(line).color(t.text_dim));
                }
                for line in warnings {
                    ui.label(RichText::new(line).color(t.warning));
                }
            }
            if let Some(form) = &mut cam.form {
                ui.separator();
                let title = match form {
                    Form::Tool { .. } => "DEFINE TOOL",
                    Form::Setup { .. } => "NEW SETUP",
                    Form::Op(f) if f.editing.is_some() => "EDIT OPERATION",
                    Form::Op(f) => f.kind.title(),
                };
                ui.label(RichText::new(title).strong());
                form_ui(ui, form, &selected, &lists);
                ui.horizontal(|ui| {
                    apply_form = ui.button("OK").clicked();
                    if ui.button("Cancel").clicked() {
                        cam.form = None;
                        cam.error = None;
                    }
                });
            }
        });
    cam.open = open;
    app.manufacture = cam;
    for (id, p) in run {
        app.manufacture.error = app.run(id, p).err();
    }
    if let Some((setup, op)) = edit {
        app.manufacture.form = edit_form(app, &setup, &op);
        app.manufacture.error = None;
    }
    if apply_form {
        apply(app);
    }
    if run_simulation {
        simulate(app);
    }
    if let Some(post) = post_to {
        let extension = solidcraft_engine::cam::Post::builtin(&post).map(|p| p.extension).unwrap_or_else(|| "nc".into());
        if let Some(path) = app.services.pick_save.as_ref().and_then(|f| f(&format!("program.{extension}"), &[extension.as_str()])) {
            match app.run("cam.post", json!({ "post": post, "path": path })) {
                Ok(v) => {
                    app.manufacture.error = None;
                    app.set_status(format!("Wrote {} lines of G-code", v["lines"].as_u64().unwrap_or(0)), false);
                }
                Err(e) => app.manufacture.error = Some(e),
            }
        }
    }
}

#[cfg(test)]
#[path = "cam_ui_tests.rs"]
mod tests;
