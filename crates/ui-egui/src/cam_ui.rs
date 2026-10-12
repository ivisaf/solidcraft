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

#[derive(Clone, Debug, PartialEq)]
pub struct OpForm {
    pub kind: OpKind,
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
    pub peck: String,
    pub dwell: String,
    pub coolant: bool,
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
    /// The toolpaths of every setup, for the design revision they were worked out at.
    cache: Option<(u64, Vec<Path>)>,
}

impl Default for CamUi {
    fn default() -> Self {
        CamUi { open: false, form: None, show_paths: true, post: 0, error: None, cache: None }
    }
}

fn text(v: &Value, k: &str) -> String {
    v.get(k).and_then(Value::as_str).unwrap_or_default().to_string()
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
        Form::Op(Box::new(OpForm {
            kind,
            sketch,
            tool: first_tool.clone(),
            spindle_rpm: "10000".into(),
            feed: "600".into(),
            plunge_feed: String::new(),
            top: String::new(),
            bottom: "-5 mm".into(),
            stepdown: String::new(),
            stepover: if kind == OpKind::Face { "5 mm".into() } else { String::new() },
            side: 0,
            climb: true,
            stock_to_leave: String::new(),
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
    app.manufacture.open = true;
    app.manufacture.form = form;
    app.manufacture.error = None;
    true
}

/// The command a form stands for. Fields left empty are left to the command's defaults.
pub fn command(form: &Form) -> Result<(&'static str, Value), String> {
    let number = |what: &str, s: &str| s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| format!("{what} must be a number"));
    let mut p = serde_json::Map::new();
    let mut optional = |k: &str, s: &str| {
        if !s.trim().is_empty() {
            p.insert(k.into(), json!(s.trim()));
        }
    };
    match form {
        Form::Tool { number: n, kind, diameter, flute_length, description } => {
            optional("diameter", diameter);
            optional("flute_length", flute_length);
            optional("description", description);
            p.insert("number".into(), json!(number("The tool number", n)? as u64));
            p.insert("kind".into(), json!(TOOL_KINDS.get(*kind).copied().unwrap_or("flat")));
            Ok(("cam.tool.define", Value::Object(p)))
        }
        Form::Setup { name, origin, post } => {
            optional("name", name);
            let [x, y, z] = origin;
            p.insert("origin".into(), json!([number("The origin's X", x)?, number("The origin's Y", y)?, number("The origin's Z", z)?]));
            p.insert("post".into(), json!(solidcraft_engine::cam::Post::BUILTIN.get(*post).copied().unwrap_or("rs274")));
            Ok(("cam.setup.create", Value::Object(p)))
        }
        Form::Op(f) => {
            optional("sketch", &f.sketch);
            optional("top", &f.top);
            optional("bottom", &f.bottom);
            optional("stepdown", &f.stepdown);
            match f.kind {
                OpKind::Face => optional("stepover", &f.stepover),
                OpKind::Contour => optional("stock_to_leave", &f.stock_to_leave),
                OpKind::Pocket => {
                    optional("stepover", &f.stepover);
                    optional("stock_to_leave", &f.stock_to_leave);
                }
                OpKind::Drill => optional("peck", &f.peck),
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
            Ok((f.kind.command(), Value::Object(p)))
        }
    }
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

fn form_ui(ui: &mut egui::Ui, form: &mut Form) {
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
            field(ui, "Sketch", &mut f.sketch, "name");
            field(ui, "Tool", &mut f.tool, "number");
            field(ui, "Spindle (rev/min)", &mut f.spindle_rpm, "10000");
            field(ui, "Feed (mm/min)", &mut f.feed, "600");
            field(ui, "Plunge feed (mm/min)", &mut f.plunge_feed, "feed / 3");
            field(ui, "Top", &mut f.top, "the sketch plane");
            field(ui, "Bottom", &mut f.bottom, "-5 mm");
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
    let mut post_to: Option<String> = None;
    let mut cam = std::mem::take(&mut app.manufacture);
    crate::frame::window(ctx, RichText::new("MANUFACTURE").strong().size(13.0), crate::frame::Width::Normal)
        .id(egui::Id::new("sc_cam_window"))
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
            });
            if let Some(form) = &mut cam.form {
                ui.separator();
                let title = match form {
                    Form::Tool { .. } => "DEFINE TOOL",
                    Form::Setup { .. } => "NEW SETUP",
                    Form::Op(f) => f.kind.title(),
                };
                ui.label(RichText::new(title).strong());
                form_ui(ui, form);
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
    if apply_form {
        apply(app);
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
