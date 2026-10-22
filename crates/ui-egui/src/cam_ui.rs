//! The Manufacture window and the toolpaths drawn in the viewport.
//!
//! A window over the engine's `cam.*` commands: the tool library, the setups with their
//! operations (each with its cutting time, or the error that stops it), a form for whichever
//! command the toolbar started, and Post Process. Toolpaths come from `cam.toolpath` and are
//! redrawn when the design changes.

use crate::ui_kit::{self, Button, button};
use egui::{RichText, Stroke};
use serde_json::{Value, json};
use solidcraft_engine::geom::Vec3;

use crate::SolveApp;
use crate::theme::Tokens;
use crate::viewport::Proj;

const TOOL_KINDS: [&str; 4] = ["flat", "ball", "drill", "vbit"];
/// What each of [`TOOL_KINDS`] is called in the tool assistant.
const TOOL_KIND_NAMES: [&str; 4] = ["Flat end mill", "Ball end mill", "Drill", "V-bit"];
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
/// Toolpath line widths: a name and the cutting moves' width in points (rapids are thinner).
const PATH_WIDTHS: [(&str, f32); 3] = [("thin", 1.0), ("normal", 1.5), ("thick", 2.5)];
/// Toolpath colours, from the theme (rapids keep the warning colour, so it is not offered).
const PATH_COLOURS: [&str; 3] = ["blue", "green", "white"];

/// The colour of cutting moves.
fn path_colour(index: usize) -> egui::Color32 {
    let t = Tokens::get();
    match index {
        1 => t.sketch_accent,
        2 => t.text,
        _ => t.accent,
    }
}

/// A small drop-down of `options` after a label; `true` when the choice changed.
fn small_choice(ui: &mut egui::Ui, id: &str, label: &str, help_label: &str, index: &mut usize, options: &[&str]) -> bool {
    let before = *index;
    explained(ui.label(label), help_label, None);
    // Only as wide as its longest option: the kit's drop-down otherwise fills the row.
    let width = ui_kit::ContentWidth::dropdown(ui, options) + ui_kit::ContentWidth::label(ui, " ");
    ui.allocate_ui(egui::vec2(width, ui_kit::row_height()), |ui| crate::ui_kit::dropdown(ui, id, index, options));
    *index != before
}

/// The simulation's detail: a name and how many columns along the stock's longer side.
const SIM_DETAIL: [(&str, u32); 3] = [("coarse", 150), ("normal", 300), ("fine", 700)];
/// Most toolpath segments drawn.
const MAX_SEGMENTS: usize = 400_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpKind {
    Face,
    Contour,
    Pocket,
    Adaptive,
    Chamfer,
    Bore,
    Drill,
}

impl OpKind {
    fn command(self) -> &'static str {
        match self {
            OpKind::Face => "cam.op.face",
            OpKind::Contour => "cam.op.contour",
            OpKind::Pocket => "cam.op.pocket",
            OpKind::Adaptive => "cam.op.adaptive",
            OpKind::Chamfer => "cam.op.chamfer",
            OpKind::Bore => "cam.op.bore",
            OpKind::Drill => "cam.op.drill",
        }
    }
    fn title(self) -> &'static str {
        match self {
            OpKind::Face => "FACE",
            OpKind::Contour => "2D CONTOUR",
            OpKind::Pocket => "2D POCKET",
            OpKind::Adaptive => "ADAPTIVE",
            OpKind::Chamfer => "CHAMFER",
            OpKind::Bore => "BORE",
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
    /// The point (a V-bit's, a drill's: its included angle in degrees; a V-bit's flat in mm),
    /// and the shank's diameter and the length out of the holder (mm). Empty: the defaults.
    pub tip_angle: String,
    pub tip_diameter: String,
    pub shank_diameter: String,
    pub length: String,
    /// The assistant's step: 0 the kind, 1 the shape, 2 the cutting data.
    pub step: usize,
    /// The material whose cutting data the rows show ("": the tool's own, for every other material).
    pub material: String,
    /// The cutting data the rows do not show: other materials', and the tool's own ("") while
    /// they show a material's.
    pub others: Vec<(String, CutText)>,
}

/// A tool's cutting data as a form shows it: for one material, or the tool's own.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CutText {
    pub spindle_rpm: String,
    pub feed: String,
    pub plunge_feed: String,
    pub stepdown: String,
    pub stepover: String,
    pub chip_load: String,
}

impl CutText {
    /// Whether nothing is typed in it.
    fn is_empty_text(&self) -> bool {
        [&self.spindle_rpm, &self.feed, &self.plunge_feed, &self.stepdown, &self.stepover, &self.chip_load].iter().all(|v| v.trim().is_empty())
    }
}

impl ToolForm {
    /// The cutting data the rows show.
    fn shown(&self) -> CutText {
        CutText {
            spindle_rpm: self.spindle_rpm.clone(),
            feed: self.feed.clone(),
            plunge_feed: self.plunge_feed.clone(),
            stepdown: self.stepdown.clone(),
            stepover: self.stepover.clone(),
            chip_load: self.chip_load.clone(),
        }
    }
    /// The cutting data kept for `material` ("": the tool's own).
    fn kept(&self, material: &str) -> CutText {
        if same_material(&self.material, material) {
            return self.shown();
        }
        self.others.iter().find(|(m, _)| same_material(m, material)).map(|(_, c)| c.clone()).unwrap_or_default()
    }
    /// Shows `material`'s cutting data in the rows ("": the tool's own), keeping what they showed.
    pub fn show_material(&mut self, material: &str) {
        if same_material(&self.material, material) {
            return;
        }
        let was = (std::mem::take(&mut self.material), self.shown());
        self.others.retain(|(m, _)| !same_material(m, &was.0));
        self.others.push(was);
        // Kept under the spelling it already has.
        let (name, c) =
            self.others.iter().find(|(m, _)| same_material(m, material)).cloned().unwrap_or_else(|| (material.to_string(), CutText::default()));
        self.others.retain(|(m, _)| !same_material(m, material));
        (self.spindle_rpm, self.feed, self.plunge_feed) = (c.spindle_rpm, c.feed, c.plunge_feed);
        (self.stepdown, self.stepover, self.chip_load) = (c.stepdown, c.stepover, c.chip_load);
        self.material = name;
    }
    /// Every material's cutting data, as typed (the tool's own left out).
    fn by_material(&self) -> Vec<(String, CutText)> {
        let mut out: Vec<(String, CutText)> = self.others.iter().filter(|(m, _)| !m.is_empty()).cloned().collect();
        if !self.material.is_empty() {
            out.push((self.material.clone(), self.shown()));
        }
        out
    }
}

/// Whether two material names are the same material (case and surrounding spaces aside).
fn same_material(a: &str, b: &str) -> bool {
    solidcraft_engine::cam::same_material(a, b)
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
    /// What the stock is made of ("": not said).
    pub material: String,
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
    /// Where the tool travels between cuts, and where its rapid approach stops (empty: automatic).
    pub clearance: String,
    pub retract: String,
    pub stepdown: String,
    pub stepover: String,
    pub side: usize,
    pub climb: bool,
    pub stock_to_leave: String,
    pub ramp_angle: String,
    /// How many depth passes instead of the stepdown (empty: by the stepdown).
    pub passes: String,
    /// A contour's tabs: whether it has any, how many on each loop, how wide and how high.
    pub tabs: bool,
    pub tab_count: String,
    pub tab_width: String,
    pub tab_height: String,
    /// What the depth passes leave on the walls for one finishing pass (empty: none).
    pub finish: String,
    /// How far a bore's helix goes down in one turn.
    pub pitch: String,
    /// A pocket's rest machining: the larger tool (its number) that cleared it before (empty: the whole pocket).
    pub rest_from: String,
    /// Adaptive clearing: the heaviest cut (percent of the tool's diameter), the entry helix's
    /// slope and its diameter at most and at least (percent of the tool's).
    pub stepover_percent: String,
    pub helix_angle: String,
    pub helix_max: String,
    pub helix_min: String,
    /// Adaptive clearing: how fast light steps may go, percent of the feed (empty: never faster).
    pub feed_up: String,
    /// A chamfer's width across the top, and how much deeper than it the tool's point goes.
    pub width: String,
    pub tip_offset: String,
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

    /// A height field in work coordinates with the work frame that puts it in the world; no deviations.
    fn of(frame: [Vec3; 4], field: &solidcraft_engine::cam::HeightField) -> StockField {
        StockField {
            frame,
            origin: [field.origin.x, field.origin.y],
            cell: field.cell,
            nx: field.nx,
            ny: field.ny,
            floor: field.floor,
            z: field.z.clone(),
            deviation: Vec::new(),
        }
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
    /// The program being shown running, when it is.
    pub play: Option<Box<Play>>,
    pub open: bool,
    /// The Tool Library window.
    pub tools_open: bool,
    /// Whether each panel is folded to its header, and where the Manufacture panel was last drawn.
    pub folded: bool,
    pub tools_folded: bool,
    panel: Option<egui::Rect>,
    /// How far each panel was dragged from its docked place by its header (zero: docked).
    pub offset: egui::Vec2,
    pub tools_offset: egui::Vec2,
    /// The setup, or the operation of a setup, picked in the browser.
    pub picked: Option<(u64, Option<u64>)>,
    /// The browser's eye keeps the toolpaths drawn while the Manufacture panel is closed.
    pub pinned_paths: bool,
    /// `cam.list` as the browser last read it, and the design revision it was read at.
    browser_listing: Option<(u64, Value)>,
    /// The tool picked in the Tool Library window: whether it is the machine library's, and its number.
    pub selected_tool: Option<(bool, u64)>,
    /// The tool library file to use instead of the machine's own (tests).
    pub library: Option<String>,
    /// A material's name as typed in the Tool Library window, to add to the library.
    pub new_material: String,
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
    /// How finely the simulation looks at the stock: index into [`SIM_DETAIL`]. Kept with the
    /// preferences, like the toolpaths' width and colour (indices into [`PATH_WIDTHS`] and
    /// [`PATH_COLOURS`]).
    pub sim_detail: usize,
    pub path_width: usize,
    pub path_colour: usize,
    /// How fast the machine's rapids move (mm/min), for playing a program; kept with the preferences.
    pub rapid_rate: f64,
    /// How the Manufacture and Tool Library panels are shown (kept with the preferences).
    pub look: PanelLook,
    pub tools_look: PanelLook,
    /// The setup last simulated (simulated again when the detail changes).
    simulated: Option<u64>,
    /// The design revision a program was last written at, and whether that is the revision now.
    posted_at: Option<u64>,
    posted_now: bool,
    /// The title of the form last shown (one that opens is scrolled into view).
    form_seen: Option<&'static str>,
    /// What the last tool import took and left out, shown in the Tool Library window.
    pub import_notes: Vec<String>,
    /// Setups and operations whose toolpaths are not drawn (the browser's eyes).
    pub hidden: std::collections::BTreeSet<u64>,
    /// The setup and the operation of each toolpath in `cache`, in the same order.
    path_ids: Vec<(u64, u64)>,
    /// The open operation form's toolpath before OK (or why there is none), for the design
    /// revision and the command it was worked out for.
    preview: Option<(u64, Value, Result<solidcraft_engine::CamPreview, String>)>,
    /// The toolpaths of every setup, for the design revision they were worked out at.
    cache: Option<(u64, Vec<Path>, Vec<SetupView>)>,
    /// The open operation form's own heights, for the design revision, the command and the geometry asked about.
    heights: Option<(u64, &'static str, Value, Heights)>,
    /// How high the selection reaches, for the design revision and the setup and selection asked about.
    selection_heights: Option<(u64, Value, Option<(f64, f64)>)>,
}

/// A setup's program shown running: the tool moving along the toolpaths and the stock cut as
/// far as it has got.
pub struct Play {
    data: solidcraft_engine::CamPlayback,
    /// The stock cut as far as `t`.
    field: solidcraft_engine::cam::HeightField,
    /// Minutes into the program, and where the controls asked to go.
    pub t: f64,
    seek: Option<f64>,
    pub playing: bool,
    /// Index into [`PLAY_SPEEDS`].
    pub speed: usize,
    /// When the drawn stock was last brought up to date (seconds of the app's clock).
    drawn_at: f64,
    /// How many times a rapid has run into material so far, and the last time: where (world) and when.
    pub collisions: usize,
    flash: Option<(Vec3, f64)>,
    /// The simulation's own stock (with its deviations from the part), shown again at the end.
    finished: Option<StockField>,
}

/// How much faster than the machine a program plays: a name and the factor.
const PLAY_SPEEDS: [(&str, f64); 4] = [("1×", 1.0), ("5×", 5.0), ("20×", 20.0), ("100×", 100.0)];
/// How often the drawn stock follows a playing program (per second).
const PLAY_REDRAWS: f64 = 15.0;
/// How long a rapid into material stays marked (seconds).
const FLASH_SECONDS: f64 = 1.5;

/// Minutes as the clock shows them: m:ss.
fn clock(minutes: f64) -> String {
    let s = (minutes.max(0.0) * 60.0).round() as u64;
    format!("{}:{:02}", s / 60, s % 60)
}

impl Default for CamUi {
    fn default() -> Self {
        CamUi {
            rapid_rate: solidcraft_engine::cam::DEFAULT_RAPID_RATE,
            play: None,
            open: false,
            tools_open: false,
            folded: false,
            tools_folded: false,
            panel: None,
            offset: egui::Vec2::ZERO,
            tools_offset: egui::Vec2::ZERO,
            picked: None,
            pinned_paths: false,
            browser_listing: None,
            selected_tool: None,
            new_material: String::new(),
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
            sim_detail: 1,
            path_width: 1,
            path_colour: 0,
            look: PanelLook { width: 0.0, fade: 1, resizing: false },
            tools_look: PanelLook { width: 0.0, fade: 1, resizing: false },
            simulated: None,
            posted_at: None,
            posted_now: false,
            form_seen: None,
            import_notes: Vec::new(),
            hidden: Default::default(),
            path_ids: Vec::new(),
            preview: None,
            cache: None,
            heights: None,
            selection_heights: None,
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
        let has_bodies = !app.session.world_state().bodies.is_empty();
        Form::Op(Box::new(OpForm {
            kind,
            filled_for: String::new(),
            flutes: 0,
            chip_load: String::new(),
            // Facing covers the stock when there is a body to have stock round, else a sketch.
            boundary: if has_bodies { 0 } else { 2 },
            limit: 0,
            margin: String::new(),
            editing: None,
            name: String::new(),
            // Faces, and the stock a facing operation covers, give the heights themselves.
            // With a body, heights come from it and the sketch; without one, a first depth to change.
            bottom: if faces.is_empty() && !has_bodies { "-5 mm".into() } else { String::new() },
            faces,
            sketch,
            tool: first_tool.clone(),
            spindle_rpm: String::new(),
            feed: String::new(),
            plunge_feed: String::new(),
            top: String::new(),
            clearance: String::new(),
            retract: String::new(),
            stepdown: String::new(),
            stepover: if kind == OpKind::Face { "5 mm".into() } else { String::new() },
            side: 0,
            climb: true,
            stock_to_leave: String::new(),
            ramp_angle: String::new(),
            passes: String::new(),
            tabs: false,
            tab_count: String::new(),
            tab_width: String::new(),
            tab_height: String::new(),
            finish: String::new(),
            pitch: String::new(),
            rest_from: String::new(),
            stepover_percent: String::new(),
            helix_angle: String::new(),
            helix_max: String::new(),
            helix_min: String::new(),
            feed_up: String::new(),
            width: String::new(),
            tip_offset: String::new(),
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
                tip_angle: String::new(),
                tip_diameter: String::new(),
                shank_diameter: String::new(),
                length: String::new(),
                step: 0,
                material: String::new(),
                others: Vec::new(),
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
            material: String::new(),
        }))),
        "cam.op.face" => Some(op(OpKind::Face)),
        "cam.op.contour" => Some(op(OpKind::Contour)),
        "cam.op.pocket" => Some(op(OpKind::Pocket)),
        "cam.op.adaptive" => Some(op(OpKind::Adaptive)),
        "cam.op.chamfer" => Some(op(OpKind::Chamfer)),
        "cam.op.bore" => Some(op(OpKind::Bore)),
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
            let own = f.kept("");
            for (k, what, text) in [
                ("spindle_rpm", "The spindle speed", &own.spindle_rpm),
                ("feed", "The feed", &own.feed),
                ("plunge_feed", "The plunge feed", &own.plunge_feed),
                ("stepdown", "The stepdown", &own.stepdown),
                ("stepover", "The stepover", &own.stepover),
            ] {
                if !text.trim().is_empty() {
                    p.insert(k.into(), json!(number(what, text)?));
                } else if f.editing {
                    p.insert(k.into(), Value::Null);
                }
            }
            // The point and the picture's shank and length; units typed after a number are dropped.
            for (k, what, text) in [
                ("tip_angle", "The point angle", &f.tip_angle),
                ("tip_diameter", "The tip diameter", &f.tip_diameter),
                ("shank_diameter", "The shank diameter", &f.shank_diameter),
                ("length", "The length", &f.length),
            ] {
                let plain = text.trim().trim_end_matches("mm").trim_end_matches("deg").trim_end_matches('°').trim();
                if !plain.is_empty() {
                    p.insert(k.into(), json!(number(what, plain)?));
                } else if f.editing {
                    p.insert(k.into(), Value::Null);
                }
            }
            // A chip load without a feed: the engine works the feed out.
            if own.feed.trim().is_empty() && !own.chip_load.trim().is_empty() {
                p.insert("feed".into(), Value::Null);
                p.insert("chip_load".into(), json!(number("The chip load", &own.chip_load)?));
            }
            // The data in particular materials; a material with nothing typed is left out.
            let mut cutting = Vec::new();
            for (material, c) in f.by_material() {
                let mut e = serde_json::Map::new();
                for (k, what, text) in [
                    ("spindle_rpm", "The spindle speed", &c.spindle_rpm),
                    ("feed", "The feed", &c.feed),
                    ("plunge_feed", "The plunge feed", &c.plunge_feed),
                    ("stepdown", "The stepdown", &c.stepdown),
                    ("stepover", "The stepover", &c.stepover),
                ] {
                    if !text.trim().is_empty() {
                        e.insert(k.into(), json!(number(&format!("{what} in {material}"), text)?));
                    }
                }
                if c.feed.trim().is_empty() && !c.chip_load.trim().is_empty() {
                    e.insert("chip_load".into(), json!(number(&format!("The chip load in {material}"), &c.chip_load)?));
                }
                if !e.is_empty() {
                    e.insert("material".into(), json!(material));
                    cutting.push(Value::Object(e));
                }
            }
            if !cutting.is_empty() || f.editing {
                p.insert("cutting".into(), json!(cutting));
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
            p.insert("material".into(), if f.material.trim().is_empty() { Value::Null } else { json!(f.material.trim()) });
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
                    for (k, text) in [("top", &f.top), ("bottom", &f.bottom), ("clearance", &f.clearance), ("retract", &f.retract)] {
                        optional(&mut p, k, text);
                    }
                }
                // An edit: an emptied height goes back to automatic; geometry is sent only when it changed.
                Some(e) => {
                    for (k, text) in [("top", &f.top), ("bottom", &f.bottom), ("clearance", &f.clearance), ("retract", &f.retract)] {
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
                    // An emptied rest tool on an edit means the whole pocket again.
                    match f.rest_from.trim().parse::<u64>() {
                        Ok(n) => {
                            p.insert("rest_from".into(), json!(n));
                        }
                        Err(_) if f.editing.is_some() => {
                            p.insert("rest_from".into(), Value::Null);
                        }
                        Err(_) => {}
                    }
                    optional(&mut p, "stepover", &f.stepover);
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    optional(&mut p, "ramp_angle", &f.ramp_angle);
                }
                OpKind::Adaptive => {
                    for (k, what, text) in [
                        ("stepover_percent", "The stepover", &f.stepover_percent),
                        ("helix_max", "The largest helix", &f.helix_max),
                        ("helix_min", "The smallest helix", &f.helix_min),
                    ] {
                        let plain = text.trim().trim_end_matches('%').trim();
                        if !plain.is_empty() {
                            p.insert(k.into(), json!(number(what, plain)?));
                        }
                    }
                    let feed_up = f.feed_up.trim().trim_end_matches('%').trim();
                    if !feed_up.is_empty() {
                        p.insert("feed_up".into(), json!(number("The feed on light cuts", feed_up)?));
                    } else if f.editing.is_some() {
                        p.insert("feed_up".into(), Value::Null);
                    }
                    optional(&mut p, "helix_angle", &f.helix_angle);
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    match f.rest_from.trim().parse::<u64>() {
                        Ok(n) => {
                            p.insert("rest_from".into(), json!(n));
                        }
                        Err(_) if f.editing.is_some() => {
                            p.insert("rest_from".into(), Value::Null);
                        }
                        Err(_) => {}
                    }
                }
                OpKind::Chamfer => {
                    optional(&mut p, "width", &f.width);
                    optional(&mut p, "tip_offset", &f.tip_offset);
                    optional(&mut p, "ramp_angle", &f.ramp_angle);
                }
                OpKind::Bore => {
                    optional(&mut p, "pitch", &f.pitch);
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                }
                OpKind::Drill => optional(&mut p, "peck", &f.peck),
            }
            if !f.faces.is_empty() && f.editing.as_ref().is_none_or(|e| e.faces != f.faces) {
                let faces: Vec<Value> = f.faces.iter().map(|(body, at)| json!({"body": body, "point": [at.x, at.y, at.z]})).collect();
                p.insert("faces".into(), json!(faces));
            }
            // A number of passes instead of the stepdown; on an edit an emptied one goes back to the stepdown.
            if !matches!(f.kind, OpKind::Drill | OpKind::Bore | OpKind::Chamfer) {
                match f.passes.trim().parse::<u64>() {
                    Ok(n) => {
                        p.insert("passes".into(), json!(n));
                    }
                    Err(_) if !f.passes.trim().is_empty() => return Err("The number of passes must be a whole number".into()),
                    Err(_) if f.editing.is_some() => {
                        p.insert("passes".into(), Value::Null);
                    }
                    Err(_) => {}
                }
            }
            if f.kind == OpKind::Contour {
                if f.tabs {
                    let mut tabs = serde_json::Map::new();
                    if !f.tab_count.trim().is_empty() {
                        let n = f.tab_count.trim().parse::<u64>().map_err(|_| "The number of tabs must be a whole number".to_string())?;
                        tabs.insert("count".into(), json!(n));
                    }
                    optional(&mut tabs, "width", &f.tab_width);
                    optional(&mut tabs, "height", &f.tab_height);
                    p.insert("tabs".into(), Value::Object(tabs));
                } else if f.editing.is_some() {
                    p.insert("tabs".into(), Value::Null);
                }
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket | OpKind::Adaptive) {
                if !f.finish.trim().is_empty() {
                    p.insert("finish".into(), json!(f.finish.trim()));
                } else if f.editing.is_some() {
                    p.insert("finish".into(), Value::Null);
                }
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
            // A chamfer is outside or inside an edge, never on it.
            if f.kind == OpKind::Chamfer {
                p.insert("side".into(), json!(if f.side == 1 { "inside" } else { "outside" }));
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket | OpKind::Bore | OpKind::Chamfer | OpKind::Adaptive) {
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
        "v_bit" => 3,
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
        tip_angle: number("tip_angle"),
        tip_diameter: number("tip_diameter"),
        shank_diameter: number("shank_diameter"),
        length: number("length"),
        // A tool that exists opens on its shape.
        step: 1,
        material: String::new(),
        others: tool["cutting"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|c| {
                let value = |k: &str| c[k].as_f64().map(|x| x.to_string()).unwrap_or_default();
                // The chip load its feed makes at its speed (else the tool's own).
                let rpm = c["spindle_rpm"].as_f64().or(tool["spindle_rpm"].as_f64());
                let flutes = tool["flutes"].as_f64().filter(|z| *z >= 1.0);
                let chip_load = match (c["feed"].as_f64(), rpm, flutes) {
                    (Some(feed), Some(rpm), Some(z)) if rpm > 0.0 => shown(feed / (rpm * z), 4),
                    _ => String::new(),
                };
                let cut = CutText {
                    spindle_rpm: value("spindle_rpm"),
                    feed: value("feed"),
                    plunge_feed: value("plunge_feed"),
                    stepdown: value("stepdown"),
                    stepover: value("stepover"),
                    chip_load,
                };
                Some((c["material"].as_str()?.to_string(), cut))
            })
            .collect(),
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
        material: text(setup, "material"),
    })))
}

/// A form filled in from an operation as `cam.list` reports it.
pub fn edit_form(app: &SolveApp, setup: &Value, op: &Value) -> Option<Form> {
    let kind = match op["kind"].as_str()? {
        "face" => OpKind::Face,
        "contour" => OpKind::Contour,
        "pocket" => OpKind::Pocket,
        "adaptive" => OpKind::Adaptive,
        "chamfer" => OpKind::Chamfer,
        "bore" => OpKind::Bore,
        "drill" => OpKind::Drill,
        _ => return None,
    };
    let number = |v: &Value| v.as_f64().map(|x| x.to_string()).unwrap_or_default();
    let flutes = op["tool"].as_u64().and_then(|n| app.session.doc.manufacture.tool(n as u32)).map_or(0, |t| t.flutes);
    let sketch = op["geometry"]["sketch"].as_u64().and_then(|id| app.session.doc.feature(id)).map(|f| f.name.clone()).unwrap_or_default();
    let faces: Vec<(String, Vec3)> =
        op["geometry"]["faces"].as_array().into_iter().flatten().filter_map(|f| Some((f["body"].as_str()?.to_string(), point(&f["at"])?))).collect();
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
        clearance: text(op, "clearance"),
        retract: text(op, "retract"),
        stepdown: text(op, "stepdown"),
        stepover: text(inputs, "stepover"),
        side: SIDES.iter().position(|s| Some(*s) == inputs["side"].as_str()).unwrap_or(0),
        climb: inputs["climb"].as_bool().unwrap_or(true),
        stock_to_leave: text(inputs, "stock_to_leave"),
        ramp_angle: text(inputs, "ramp_angle"),
        passes: op["passes"].as_u64().map(|n| n.to_string()).unwrap_or_default(),
        tabs: inputs["tabs"].is_object(),
        tab_count: inputs["tabs"]["count"].as_u64().map(|n| n.to_string()).unwrap_or_default(),
        tab_width: text(&inputs["tabs"], "width"),
        tab_height: text(&inputs["tabs"], "height"),
        finish: text(inputs, "finish"),
        pitch: text(inputs, "pitch"),
        rest_from: inputs["rest_from"].as_u64().map(|n| n.to_string()).unwrap_or_default(),
        stepover_percent: number(&inputs["stepover_percent"]),
        helix_angle: text(inputs, "helix_angle"),
        helix_max: number(&inputs["helix_max"]),
        helix_min: number(&inputs["helix_min"]),
        feed_up: number(&inputs["feed_up"]),
        width: text(inputs, "width"),
        tip_offset: text(inputs, "tip_offset"),
        peck: text(inputs, "peck"),
        dwell: inputs["dwell"].as_f64().filter(|d| *d > 0.0).map(|d| d.to_string()).unwrap_or_default(),
        coolant: op["coolant"].as_bool().unwrap_or(false),
    })))
}

/// Says in the Tool Library window what an import took and what it left out.
pub fn imported(app: &mut SolveApp, result: Result<Value, String>) {
    match result {
        Ok(v) => {
            let count = |k: &str| v[k].as_array().map_or(0, Vec::len);
            let mut lines = vec![format!("Imported {} tool(s)", count("added") + count("replaced"))];
            if count("materials") > 0 {
                lines[0].push_str(&format!(" and {} material(s)", count("materials")));
            }
            if count("skipped") > 0 {
                let numbers: Vec<String> = v["skipped"].as_array().into_iter().flatten().map(|n| format!("T{n}")).collect();
                lines.push(format!("Skipped {}: the library has these numbers already", numbers.join(", ")));
            }
            lines.extend(v["notes"].as_array().into_iter().flatten().filter_map(Value::as_str).map(|n| format!("Left out {n}")));
            app.manufacture.error = None;
            app.manufacture.import_notes = lines;
        }
        Err(e) => {
            app.manufacture.error = Some(e);
            app.manufacture.import_notes.clear();
        }
    }
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

/// What the engine asks about an operation form's heights: its setup and its geometry, as its
/// command names them (`None` while the form names no geometry).
fn heights_query(f: &OpForm) -> Option<Value> {
    let mut p = serde_json::Map::new();
    if let Some(e) = &f.editing {
        p.insert("setup".into(), json!(e.setup));
    }
    if !f.faces.is_empty() {
        let faces: Vec<Value> = f.faces.iter().map(|(body, at)| json!({"body": body, "point": [at.x, at.y, at.z]})).collect();
        p.insert("faces".into(), json!(faces));
    } else if f.kind == OpKind::Face && f.boundary < 2 {
        p.insert("boundary".into(), json!(FACE_BOUNDS.get(f.boundary).copied().unwrap_or("stock")));
    } else if f.sketch.trim().is_empty() {
        return None;
    } else {
        p.insert("sketch".into(), json!(f.sketch.trim()));
    }
    Some(Value::Object(p))
}

/// The open operation form's own heights, worked out again when the design or its geometry changes.
fn form_heights(app: &mut SolveApp) -> Option<Heights> {
    let Some(Form::Op(f)) = &app.manufacture.form else { return None };
    let query = heights_query(f)?;
    let (cmd, revision) = (f.kind.command(), app.session.revision);
    if let Some((r, c, q, h)) = &app.manufacture.heights
        && (*r, *c) == (revision, cmd)
        && *q == query
    {
        return Some(h.clone());
    }
    let h = solidcraft_engine::cam_heights(&app.session, cmd, &query).map_err(|e| e.to_string());
    app.manufacture.heights = Some((revision, cmd, query, h.clone()));
    Some(h)
}

/// How high the selection reaches in the open operation form's setup, worked out again when the
/// design or the selection changes.
fn picked_heights(app: &mut SolveApp) -> Option<(f64, f64)> {
    let Some(Form::Op(f)) = &app.manufacture.form else { return None };
    if app.session.selection.is_empty() {
        return None;
    }
    let setup = f.editing.as_ref().map_or(json!({}), |e| json!({ "setup": e.setup }));
    let key = json!({ "setup": setup, "selection": app.session.selection });
    let revision = app.session.revision;
    if let Some((r, k, h)) = &app.manufacture.selection_heights
        && *r == revision
        && *k == key
    {
        return *h;
    }
    let h = solidcraft_engine::cam_selection_heights(&app.session, &setup, &app.session.selection).ok().flatten();
    app.manufacture.selection_heights = Some((revision, key, h));
    h
}

/// Works out the open operation form's toolpath again when its command or the design changed;
/// none while the form still misses something.
fn update_preview(app: &mut SolveApp, lists: &Choices) {
    let wanted = match &app.manufacture.form {
        Some(form @ Form::Op(_)) if missing(form, lists).is_empty() => command(form).ok().map(|(id, p)| (id, json!({ "command": id, "params": p }))),
        _ => None,
    };
    let Some((id, key)) = wanted else {
        app.manufacture.preview = None;
        return;
    };
    let revision = app.session.revision;
    if app.manufacture.preview.as_ref().is_some_and(|(r, k, _)| *r == revision && *k == key) {
        return;
    }
    let path = solidcraft_engine::cam_preview(&app.session, id, &key["params"]).map_err(|e| e.to_string());
    app.manufacture.preview = Some((revision, key, path));
}

/// The operation an open form edits, whose own toolpath the preview stands in for.
fn previewed_op(cam: &CamUi) -> Option<u64> {
    match (&cam.form, &cam.preview) {
        (Some(Form::Op(f)), Some((_, _, Ok(_)))) => f.editing.as_ref().map(|e| e.op),
        _ => None,
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
    app.manufacture.play = None;
    app.manufacture.stock_stamp += 1;
    app.manufacture.simulated = Some(setup);
    let columns = SIM_DETAIL.get(app.manufacture.sim_detail).map_or(300, |d| d.1);
    match app.session.execute("cam.simulate", &json!({ "setup": setup, "field": true, "columns": columns })) {
        Ok(v) => {
            let number = |k: &str| v[k].as_f64();
            let mut found = vec![format!("Removed {:.2} cm³", number("removed_volume").unwrap_or(0.0) / 1000.0)];
            if let (Some(gouge), Some(left), Some(volume)) = (number("max_gouge"), number("max_leftover"), number("leftover_volume")) {
                found.push(if gouge > 1e-3 { format!("Cuts up to {gouge:.3} mm into the part") } else { "Does not cut into the part".into() });
                found.push(format!("Leaves up to {left:.2} mm on the part ({:.2} cm³)", volume / 1000.0));
            }
            found.extend(v["notes"].as_array().into_iter().flatten().filter_map(Value::as_str).map(str::to_string));
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

/// Starts playing a setup's program from its beginning, on the simulation's columns.
pub fn start_play(app: &mut SolveApp, setup: u64) {
    let columns = SIM_DETAIL.get(app.manufacture.sim_detail).map_or(300, |d| d.1);
    let rapid_rate = app.manufacture.rapid_rate;
    match solidcraft_engine::cam_playback(&app.session, &json!({ "setup": setup, "columns": columns, "rapid_rate": rapid_rate })) {
        Ok(data) => {
            let speed = app.manufacture.play.as_ref().map_or(1, |p| p.speed);
            let finished = app.manufacture.play.take().map_or_else(|| app.manufacture.stock.clone(), |p| p.finished);
            let field = data.stock.clone();
            app.manufacture.play = Some(Box::new(Play {
                data,
                field,
                t: 0.0,
                seek: Some(0.0),
                playing: true,
                speed,
                drawn_at: f64::NEG_INFINITY,
                collisions: 0,
                flash: None,
                finished,
            }));
            app.manufacture.show_stock = true;
            app.manufacture.error = None;
        }
        Err(e) => app.manufacture.error = Some(e.to_string()),
    }
}

/// Moves a playing program on by the time since the last frame (or to where the controls asked
/// to go), cutting the stock as it goes; the drawn stock follows a few times a second.
fn tick(app: &mut SolveApp, ctx: &egui::Context) {
    let (now, dt) = ctx.input(|i| (i.time, f64::from(i.stable_dt).clamp(0.0, 0.1)));
    let Some(play) = app.manufacture.play.as_mut() else { return };
    let total = play.data.playback.minutes;
    let seek = play.seek.take();
    let to = match seek {
        Some(t) if t.is_finite() => t.clamp(0.0, total),
        Some(_) => return,
        None if play.playing => (play.t + dt / 60.0 * PLAY_SPEEDS.get(play.speed).map_or(1.0, |s| s.1)).min(total),
        None => {
            if play.flash.is_some_and(|(_, when)| now - when < FLASH_SECONDS) {
                ctx.request_repaint();
            }
            return;
        }
    };
    // Back in time: from the uncut block again.
    if to < play.t {
        play.field = play.data.stock.clone();
        (play.t, play.collisions) = (0.0, 0);
    }
    let mut failed = None;
    match play.data.playback.cut(&mut play.field, &play.data.tools, play.t, to) {
        Ok(report) if report.rapid_collisions > 0 => {
            play.collisions += report.rapid_collisions;
            if let Some((at, _)) = play.data.playback.at(to) {
                let [o, x, y, z] = play.data.frame;
                play.flash = Some((o + x * at.x + y * at.y + z * at.z, now));
            }
        }
        Ok(_) => {}
        Err(e) => failed = Some(e.to_string()),
    }
    play.t = to;
    let end = to >= total;
    if end || failed.is_some() {
        play.playing = false;
    }
    let redraw = end || seek.is_some() || now - play.drawn_at >= 1.0 / PLAY_REDRAWS;
    let stock = if end && play.finished.is_some() { play.finished.clone() } else { Some(StockField::of(play.data.frame, &play.field)) };
    if redraw {
        play.drawn_at = now;
    }
    if play.playing || play.flash.is_some_and(|(_, when)| now - when < FLASH_SECONDS) {
        ctx.request_repaint();
    }
    if redraw {
        app.manufacture.stock = stock;
        app.manufacture.stock_stamp += 1;
    }
    if let Some(e) = failed {
        app.manufacture.error = Some(e);
    }
}

/// Under a simulation: play its program, and while it plays the controls, where it is and what it does.
fn play_controls(ui: &mut egui::Ui, cam: &mut CamUi, asked: &mut Asked) {
    let t = Tokens::get();
    let Some(setup) = cam.simulated else { return };
    ui.add_space(4.0);
    let Some(play) = cam.play.as_mut() else {
        if button(ui, "Play the program", Button::Normal).on_hover_text("Show the tool moving along the toolpaths and cutting the stock").clicked() {
            asked.play = Some(setup);
        }
        return;
    };
    let total = play.data.playback.minutes;
    let steps = &play.data.playback.steps;
    let k = play.data.playback.step_at(play.t).unwrap_or(0);
    let mut stop = false;
    ui.horizontal_wrapped(|ui| {
        if button(ui, "|<", Button::Small).on_hover_text("To the start").clicked() {
            play.seek = Some(0.0);
        }
        if button(ui, "<", Button::Small).on_hover_text("Back one move").clicked() {
            let back = match steps.get(k) {
                Some(s) if play.t > s.start + 1e-9 => s.start,
                _ => k.checked_sub(1).and_then(|k| steps.get(k)).map_or(0.0, |s| s.start),
            };
            (play.seek, play.playing) = (Some(back), false);
        }
        let label = if play.playing {
            "Pause"
        } else if play.t >= total {
            "Again"
        } else {
            "Play"
        };
        if button(ui, label, Button::Small).clicked() {
            if play.t >= total {
                (play.seek, play.playing) = (Some(0.0), true);
            } else {
                play.playing = !play.playing;
            }
        }
        if button(ui, ">", Button::Small).on_hover_text("On one move").clicked() {
            let on = steps.get(k).map_or(total, |s| s.start + s.minutes);
            (play.seek, play.playing) = (Some(on), false);
        }
        if button(ui, ">|", Button::Small).on_hover_text("To the end").clicked() {
            (play.seek, play.playing) = (Some(total), false);
        }
        let speeds = PLAY_SPEEDS.map(|s| s.0);
        small_choice(ui, "sc_cam_play_speed", "Speed", "Playback speed", &mut play.speed, &speeds);
        if button(ui, "Stop", Button::Small).on_hover_text("Stop playing and show the simulation's result").clicked() {
            stop = true;
        }
    });
    // The machine's own rapid rate: the times follow it, the tool stays where it is.
    ui.horizontal(|ui| {
        explained(ui.label("Rapids"), "Rapid rate", None);
        let mut rate = cam.rapid_rate;
        let r = ui_kit::number_field(ui, &mut rate, 100.0..=100_000.0, 50.0, " mm/min");
        if r.changed() && rate != cam.rapid_rate {
            match play.data.playback.retime(rate, play.t) {
                Ok(t) => {
                    cam.rapid_rate = rate;
                    play.t = t;
                }
                Err(e) => cam.error = Some(e.to_string()),
            }
        }
    });
    let mut at = play.t;
    let r = ui.scope(|ui| {
        ui.set_max_width(ui.available_width().min(420.0));
        ui_kit::slider(ui, &mut at, 0.0..=total.max(1e-9), ui_kit::Slider { format: clock, ..Default::default() })
    });
    if r.inner.changed() {
        (play.seek, play.playing) = (Some(at), false);
    }
    if let Some((p, step)) = play.data.playback.at(play.t) {
        let op = play.data.ops.get(step.op).map_or("", String::as_str);
        let tool = play.data.tools.get(step.op).map_or(0, |t| t.number);
        let feed = match step.feed {
            _ if step.dwell => format!("dwell {} s", shown(step.minutes * 60.0, 1)),
            Some(f) => format!("feed {} mm/min", shown(f, 0)),
            None => "rapid".to_string(),
        };
        let mm = |v: f64| shown((v * 1e3).round() / 1e3 + 0.0, 3);
        ui.label(RichText::new(format!("{op}  T{tool}   X {}  Y {}  Z {}   {feed}", mm(p.x), mm(p.y), mm(p.z))).color(t.text));
        ui.label(RichText::new(format!("{} of {}", clock(play.t), clock(total))).color(t.text_dim));
    }
    if play.collisions > 0 {
        ui.label(RichText::new(format!("{} rapid move(s) ran into material: marked red where it happened", play.collisions)).color(t.error));
    }
    if stop && let Some(play) = cam.play.take() {
        cam.stock = play.finished;
        cam.stock_stamp += 1;
    }
}

/// Works the toolpaths and the setups' frames out again when the design has changed.
fn refresh(app: &mut SolveApp) {
    let revision = app.session.revision;
    if app.manufacture.cache.as_ref().is_some_and(|(r, ..)| *r == revision) {
        return;
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let (mut all, mut views, mut ids) = (Vec::new(), Vec::new(), Vec::new());
    for setup in listed["setups"].as_array().into_iter().flatten() {
        let f = &setup["frame"];
        if let (Some(o), Some(x), Some(y), Some(z)) = (point(&f["origin"]), point(&f["x"]), point(&f["y"]), point(&f["z"])) {
            let b = &setup["stock_box"];
            views.push(SetupView { frame: [o, x, y, z], stock: point(&b["min"]).zip(point(&b["max"])) });
        }
        // A setup with an operation that cannot be cut draws no toolpath; the window says why.
        let Ok(v) = app.session.execute("cam.toolpath", &json!({ "setup": setup["id"] })) else { continue };
        for op in v["operations"].as_array().into_iter().flatten() {
            ids.push((setup["id"].as_u64().unwrap_or(0), op["id"].as_u64().unwrap_or(0)));
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
    app.manufacture.play = None;
    app.manufacture.simulation = None;
    app.manufacture.stock = None;
    app.manufacture.stock_stamp += 1;
    // These commands are not changes, but keep the key the revision as it is now.
    app.manufacture.cache = Some((app.session.revision, all, views));
    app.manufacture.path_ids = ids;
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
/// Whether the toolpaths are drawn: while the Manufacture panel is open, or kept on by the browser's eye.
pub fn paths_drawn(app: &SolveApp) -> bool {
    app.manufacture.show_paths && (app.manufacture.open || app.manufacture.pinned_paths)
}

pub fn segments(app: &SolveApp, proj: &Proj) -> Vec<(egui::Pos2, egui::Pos2, bool)> {
    let mut out = Vec::new();
    if !paths_drawn(app) {
        return out;
    }
    let cam = &app.manufacture;
    let paths = cam.cache.iter().flat_map(|(_, p, _)| p).zip(&cam.path_ids);
    let edited = previewed_op(cam);
    let shown = |(setup, op): &(u64, u64)| !cam.hidden.contains(setup) && !cam.hidden.contains(op) && edited != Some(*op);
    for (path, _) in paths.filter(|(_, ids)| shown(ids)) {
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
                    painter.text(b, egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(t.font_caption), colour);
                }
            }
        }
    }
    // While a form shows its toolpath, the others step back.
    let preview = match &app.manufacture.preview {
        Some((_, _, Ok(p))) if paths_drawn(app) => Some(&p.points),
        _ => None,
    };
    let back = if preview.is_some() { 0.3 } else { 1.0 };
    let prefs = &app.preferences;
    let (width, colour) = (PATH_WIDTHS.get(prefs.cam_path_width).map_or(1.5, |w| w.1), path_colour(prefs.cam_path_colour));
    // Rapids stay thin and in the warning colour.
    let rapid = (width * 0.66).max(1.0);
    for (from, to, cutting) in segments(app, proj) {
        let stroke = if cutting { Stroke::new(width, colour.gamma_multiply(back)) } else { Stroke::new(rapid, t.warning.gamma_multiply(0.7 * back)) };
        painter.line_segment([from, to], stroke);
    }
    for (a, b) in preview.into_iter().flat_map(|path| path.iter().zip(path.iter().skip(1))).take(MAX_SEGMENTS) {
        if let (Some(from), Some(to)) = (proj.to_screen(a.0), proj.to_screen(b.0)) {
            let stroke = if b.1 { Stroke::new(width + 1.0, colour) } else { Stroke::new(rapid + 0.2, t.warning) };
            painter.line_segment([from, to], stroke);
        }
    }
    // A playing program: its tool where it is, and where a rapid last ran into material.
    if let Some(play) = app.manufacture.play.as_ref().filter(|_| app.manufacture.open) {
        if let Some((at, step)) = play.data.playback.at(play.t)
            && let Some(tool) = play.data.tools.get(step.op)
        {
            draw_tool(painter, proj, play.data.frame, at, tool);
        }
        let now = painter.ctx().input(|i| i.time);
        if let Some((at, when)) = play.flash.filter(|(_, when)| now - when < FLASH_SECONDS)
            && let Some(p) = proj.to_screen(at)
        {
            let fade = (1.0 - (now - when) / FLASH_SECONDS) as f32;
            painter.circle(p, 14.0, t.error.gamma_multiply(0.35 * fade), Stroke::new(2.0, t.error.gamma_multiply(fade)));
        }
    }
}

/// The tool at `tip` (work coordinates in `frame`): its cutting part and its shank, each drawn
/// as the outline of its rings round the setup's Z.
fn draw_tool(painter: &egui::Painter, proj: &Proj, frame: [Vec3; 4], tip: Vec3, tool: &solidcraft_engine::cam::Tool) {
    use solidcraft_engine::cam::ToolKind;
    let t = Tokens::get();
    let [o, x, y, z] = frame;
    let r = tool.radius();
    let flute = tool.flute_length;
    let length = tool.length.unwrap_or(flute + 2.0 * tool.diameter).max(flute);
    let shank = tool.shank_diameter.unwrap_or(tool.diameter) * 0.5;
    // Rings as (height over the tip, radius).
    let cutting: Vec<(f64, f64)> = match tool.kind {
        ToolKind::FlatEndmill => vec![(0.0, r), (flute, r)],
        ToolKind::BallEndmill => {
            (0..=6).map(|k| f64::from(k) / 6.0 * std::f64::consts::FRAC_PI_2).map(|a| (r - r * a.cos(), r * a.sin())).chain([(flute, r)]).collect()
        }
        ToolKind::Drill | ToolKind::VBit => {
            let point = tool.tip() * 0.5;
            let cone = ((r - point) / (tool.point_angle() * 0.5).to_radians().tan()).min(flute);
            vec![(0.0, point), (cone, r), (flute, r)]
        }
    };
    let outline = |rings: &[(f64, f64)]| -> Vec<egui::Pos2> {
        let mut pts = Vec::new();
        for &(h, radius) in rings {
            for k in 0..24 {
                let a = f64::from(k) / 24.0 * std::f64::consts::TAU;
                let q = Vec3::new(tip.x + radius * a.cos(), tip.y + radius * a.sin(), tip.z + h);
                if let Some(p) = proj.to_screen(o + x * q.x + y * q.y + z * q.z) {
                    pts.push(p);
                }
            }
        }
        convex_hull(pts)
    };
    let shank_pts = outline(&[(flute, shank), (length, shank)]);
    if shank_pts.len() >= 3 {
        painter.add(egui::Shape::convex_polygon(shank_pts, t.text_dim.gamma_multiply(0.8), Stroke::new(1.0, t.text)));
    }
    let cut_pts = outline(&cutting);
    if cut_pts.len() >= 3 {
        painter.add(egui::Shape::convex_polygon(cut_pts, t.accent.gamma_multiply(0.85), Stroke::new(1.0, t.text)));
    }
}

/// The convex hull of points on the screen, anticlockwise.
fn convex_hull(mut pts: Vec<egui::Pos2>) -> Vec<egui::Pos2> {
    pts.retain(|p| p.x.is_finite() && p.y.is_finite());
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: egui::Pos2, a: egui::Pos2, b: egui::Pos2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut hull: Vec<egui::Pos2> = Vec::with_capacity(pts.len() * 2);
    for pass in [false, true] {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &egui::Pos2>> = if pass { Box::new(pts.iter().rev()) } else { Box::new(pts.iter()) };
        for &p in ordered {
            while hull.len() >= start + 2 && hull.get(hull.len() - 2).zip(hull.last()).is_some_and(|(a, b)| cross(*a, *b, p) <= 0.0) {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// A value a form still needs before OK can run its command: the row it belongs to (empty: the
/// form as a whole) and what is wanted there.
#[derive(Clone, Debug, PartialEq)]
struct Need {
    field: &'static str,
    why: String,
}

/// A row label without its unit: "Feed (mm/min)" is "Feed".
fn base(label: &str) -> &str {
    label.split(" (").next().unwrap_or(label)
}

/// What a form's row is for, shown when the pointer rests on it.
fn help(label: &str) -> Option<&'static str> {
    Some(match base(label) {
        "Number" => "The tool's number: the T word in the program and the pocket of a tool changer.",
        "Kind" => "Flat: a flat end mill. Ball: a rounded end. Drill: only plunges straight down.",
        "Diameter" => "How wide the tool cuts.",
        "Description" => "Your own words for the tool, shown in the lists.",
        "Length" => "How far the tool reaches out of its holder: shank and cutting part together.",
        "Shank diameter" => "How thick the part the holder grips is. Empty: as thick as the tool cuts.",
        "Point angle" => "The angle between the point's two flanks: a V-bit's (often 60° or 90°) or a drill's (118°).",
        "Tip diameter" => "A V-bit's small flat at its point. Empty: a sharp point.",
        "Flute length" => "How deep the cutting edges reach. Deeper cuts need a longer tool.",
        "Flutes" => "How many cutting edges the tool has. Together with the speed it sets the feed.",
        "Spindle" => "How fast the spindle turns. Empty: the tool's own speed.",
        "Chip load" => "How thick a chip each edge takes. Feed = speed × flutes × chip load: change one and the feed follows.",
        "Feed" => "How fast the tool moves sideways while cutting. Empty: the tool's own feed.",
        "Plunge feed" => "How fast the tool moves down into the material.",
        "Stepdown" => "How deep the tool goes per pass. Deep cuts are made in several passes.",
        "Passes" => "How many depth passes, instead of a stepdown: the depth is divided evenly. Empty: by the stepdown.",
        "Finish allowance" => {
            "What the depth passes leave on the walls; one more pass at the full depth then takes it off, for a clean wall. Empty: none."
        }
        "Tabs" => "Small bridges left standing at the bottom, so that a part cut out of a sheet stays in place. Cut them free by hand afterwards.",
        "Tab count" => "How many tabs on each outline, spaced evenly along it.",
        "Tab width" => "How long each tab is along the part's edge.",
        "Tab height" => "How high each tab stands over the bottom of the cut.",
        "Stepover" => "How far apart the side-by-side passes are. Smaller is cleaner but slower.",
        "Name" => "Your own name for it. Empty: one is made up.",
        "Tool axis" => "The direction the tool comes from. +z: from above.",
        "Stock on the sides" | "Stock on top" | "Stock below" => "Extra material round the part: the raw block is the part's box plus these.",
        "Origin on" => "What the work zero sits on: the raw block, the part, or a point you give.",
        "Origin X" | "Origin Y" | "Origin Z" => "Where on it the work zero sits, the point the machine is zeroed on.",
        "Post" => "Which machine dialect the G-code is written in.",
        "Material" => {
            "What the stock is made of. A setup's material picks each tool's cutting data for it (a tool without any cuts with its own); in a tool, the rows below are for the material chosen here."
        }
        "Boundary" => "The area to face: the raw block, the part's outline, or a sketch's outline.",
        "Sketch" => "The sketch whose closed profiles are cut.",
        "Faces" => "Picked faces of the part: they give the outline and the heights.",
        "Tool" => "The tool that cuts. Its speed and feeds fill the rows below.",
        "Clearance" => "Where the tool travels between cuts, clear of the part and its clamps. Empty: 10 mm over the top, the part and the stock.",
        "Retract" => "Where the tool's fast approach stops and it feeds down into the material. Empty: 2 mm over the top.",
        "Top" => "Where cutting starts, in the setup's Z (zero at the work zero).",
        "Toolpath colour" => "The colour toolpaths' cutting moves are drawn in; rapids stay orange and thin. Kept for the next start.",
        "Rest of tool" => {
            "Rest machining: a larger tool cleared this pocket before; this one only cuts what it could not reach, in the corners and narrow places."
        }
        "Stepover (%)" => {
            "The heaviest cut, as a percentage of the tool's diameter. The tool steps through the material so that no step cuts wider, in corners and narrow places too: a steady load, so the full flute length can be used."
        }
        "Helix angle" => "How steeply the tool goes down into the material on its entry helix (or ramp, where a helix has no room).",
        "Helix largest (%)" | "Helix smallest (%)" => {
            "The entry helix's diameter, as a percentage of the tool's: as large as there is room for, at most the largest; with less room than the smallest, a ramp instead."
        }
        "Light-cut feed (%)" => {
            "How fast a step that cuts narrower than the stepover may go, in percent of the feed: its chips are thinner, so it goes faster until they are as thick as at the full stepover, at most this. Empty: always the feed."
        }
        "Chamfer width" => "How wide the chamfer is across the top. Its angle is the tool's; its depth follows from the two.",
        "Tip offset" => "How much deeper than the chamfer the tool's point goes, so that the sharp tip does not do the cutting and wear.",
        "Depth per turn" => "How far the helix goes down in one turn round the hole. Smaller: gentler on the tool, longer to cut.",
        "Rapid rate" => "How fast your machine's rapids move (mm/min): the played program's times follow it. Kept for the next start.",
        "Panel fade" => {
            "How see-through these panels' background gets while the pointer is in the 3D view; over a panel, or typing in it, it is solid again. Kept for the next start."
        }
        "Toolpath width" => "How thick toolpaths are drawn. Kept for the next start.",
        "Simulation detail" => "How finely the simulation looks at the stock. Fine shows the cut closer to the toolpaths and takes longer.",
        "Bottom" => "How deep it cuts: the lowest height, in the setup's Z (zero at the work zero; below it is negative).",
        "Cut" => "Whether the tool may run over the boundary or stays inside it.",
        "Over by" | "Inside by" => "How far over or inside the boundary the tool goes.",
        "Side" => "Outside: the profile is the part. Inside: it is a hole. On: the tool's centre follows the line.",
        "Stock to leave" => "Material left on the walls, for a finishing pass later.",
        "Ramp angle" => "How steeply the tool goes down into each pass. 0: straight down.",
        "Direction" => "Climb milling cuts with the tool's turning. Off: conventional milling.",
        "Coolant" => "Switch the coolant on for this operation.",
        "Peck depth" => "Drill this deep, pull back to clear the chips, and go on. Empty: in one go.",
        "Dwell" => "Seconds to wait at the bottom of each hole.",
        _ => return None,
    })
}

/// What `field` of a form still needs, if anything.
fn needed<'a>(need: &'a [Need], label: &str) -> Option<&'a Need> {
    need.iter().find(|n| n.field == base(label))
}

/// A form row's label: in the error colour while its value is still needed, with what the row
/// is for (and what it needs) on hover.
fn row_label(ui: &mut egui::Ui, label: &str, need: Option<&Need>) {
    let t = Tokens::get();
    let text = if need.is_some() { RichText::new(label).color(t.error) } else { RichText::new(label) };
    let r = ui.label(text);
    explained(r, label, need);
}

/// `r` with what its row is for, and what it needs, on hover.
fn explained(r: egui::Response, label: &str, need: Option<&Need>) -> egui::Response {
    let mut tip = help(label).unwrap_or_default().to_string();
    if let Some(n) = need {
        if !tip.is_empty() {
            tip.push_str("\n\n");
        }
        tip.push_str(&format!("Needed: {}", n.why));
    }
    if tip.is_empty() {
        return r;
    }
    // Wrapped: a panel at the screen's edge must not push its tips off it.
    r.on_hover_ui(|ui| {
        ui.set_max_width(320.0);
        ui.add(egui::Label::new(tip).wrap());
    })
}

/// A frame in the error colour round a row's control while its value is still needed.
fn mark(ui: &egui::Ui, r: &egui::Response, need: Option<&Need>) {
    if need.is_some() {
        ui.painter().rect_stroke(r.rect.expand(1.0), 2.0, Stroke::new(1.5, Tokens::get().error), egui::StrokeKind::Outside);
    }
}

/// A form's text box, 140 pt wide; `invalid` draws it in the error colour (its value is still needed).
fn text_box(ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str, invalid: bool) -> egui::Response {
    ui.scope(|ui| {
        ui.set_max_width(140.0);
        crate::ui_kit::cell_editor(ui, ui.make_persistent_id(("cam_field", label)), value, hint, invalid)
    })
    .inner
}

/// `add` with its buttons filled with `fill` and outlined with `stroke` (where set).
fn tinted<R>(ui: &mut egui::Ui, fill: Option<egui::Color32>, stroke: Option<Stroke>, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    ui.scope(|ui| {
        let w = &mut ui.visuals_mut().widgets;
        for v in [&mut w.inactive, &mut w.hovered, &mut w.active] {
            if let Some(fill) = fill {
                v.weak_bg_fill = fill;
            }
            if let Some(stroke) = stroke {
                v.bg_stroke = stroke;
            }
        }
        add(ui)
    })
    .inner
}

/// [`field`] for a height, with a button beside it that takes `picked`: the selection's height
/// (its highest point for a top, its lowest for a bottom; `None` with nothing selected).
/// `true` while the row is pointed at or typed in (the heights picture lights its line).
fn height_field(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str, picked: Option<f64>) -> bool {
    let need = needed(need, label);
    let mut lit = false;
    row_label(ui, label, need);
    ui.horizontal(|ui| {
        let r = text_box(ui, label, value, hint, need.is_some());
        lit |= r.hovered() || r.has_focus();
        let r = explained(r, label, need);
        crate::params_dialog::complete(ui, &r, value);
        let mm = |z: f64| format!("{} mm", shown((z * 1e3).round() / 1e3 + 0.0, 3));
        let name = format!("{label} from the selection");
        let side = crate::ui_kit::row_height();
        let b = ui
            .add_enabled_ui(picked.is_some(), |ui| {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(side, side), egui::Sense::hover());
                crate::ui_kit::icon_button_at(ui, rect, "select", &name)
            })
            .inner;
        let tip = match picked {
            Some(z) => format!("Take the height from the selection: {}", mm(z)),
            None => String::new(),
        };
        let b = b
            .on_hover_text(tip)
            .on_disabled_hover_text("Click an edge, a point or a face of the model first (the bottom circle of a hole, say); this takes its height.");
        lit |= b.hovered();
        if let (true, Some(z)) = (b.clicked(), picked) {
            *value = mm(z);
        }
    });
    ui.end_row();
    lit
}

/// [`field`], also saying whether its row is pointed at or typed in.
fn field_seen(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str) -> (bool, bool) {
    field_unless(ui, need, label, value, hint, None)
}

/// [`field_seen`], greyed out with `unused` saying why while it is `Some`.
fn field_unless(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str, unused: Option<&str>) -> (bool, bool) {
    let need = needed(need, label);
    row_label(ui, label, need);
    let r = ui.add_enabled_ui(unused.is_none(), |ui| text_box(ui, label, value, hint, need.is_some())).inner;
    let r = r.on_disabled_hover_text(unused.unwrap_or_default());
    let lit = r.hovered() || r.has_focus();
    let r = explained(r, label, need);
    let changed = r.changed() | crate::params_dialog::complete(ui, &r, value);
    ui.end_row();
    (lit, changed)
}

/// Which line of the heights picture a form row stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HeightLine {
    Clearance,
    Retract,
    Top,
    Bottom,
    Passes,
}

/// The heights of an operation as a side view: the material with the cut in it, its depth
/// passes, the tool above it, and a line at each height with its value. `lit` is drawn stronger.
fn heights_picture(ui: &mut egui::Ui, h: &solidcraft_engine::PreviewHeights, lit: Option<HeightLine>, height: f32) {
    let t = Tokens::get();
    let (rect, _) = ui.allocate_exact_size(egui::vec2(PICTURE_W, height), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    // Z from a little below the bottom to a little above the clearance, onto the picture's height.
    let depth = (h.top - h.bottom).max(1e-3);
    let (lo, hi) = (h.bottom - depth * 0.35, h.clearance.max(h.retract).max(h.top) + depth * 0.1);
    let span = (hi - lo).max(1e-3);
    let y = |z: f64| rect.bottom() - 6.0 - ((z - lo) / span) as f32 * (rect.height() - 12.0);
    // The material: a block from the top down past the bottom, with the cut taken out of its left half.
    let (block_l, block_r, cut_r) = (rect.left() + 4.0, rect.left() + 70.0, rect.left() + 36.0);
    let (top_y, bottom_y, floor_y) = (y(h.top), y(h.bottom), rect.bottom() - 2.0);
    let material = t.text_dim.gamma_multiply(0.35);
    painter.rect_filled(egui::Rect::from_min_max(egui::pos2(cut_r, top_y), egui::pos2(block_r, floor_y)), 0.0, material);
    painter.rect_filled(egui::Rect::from_min_max(egui::pos2(block_l, bottom_y), egui::pos2(cut_r, floor_y)), 0.0, material);
    // The depth passes: where each one cuts, in the cut.
    let passes = lit == Some(HeightLine::Passes);
    for z in &h.levels {
        let stroke = Stroke::new(if passes { 1.6 } else { 1.0 }, t.accent.gamma_multiply(if passes { 1.0 } else { 0.55 }));
        painter.add(egui::Shape::dashed_line(&[egui::pos2(block_l, y(*z)), egui::pos2(cut_r, y(*z))], stroke, 4.0, 3.0));
    }
    // The tool, waiting at the retract height over the cut.
    let tool_x = (block_l + cut_r) * 0.5;
    let tool = egui::Rect::from_min_max(egui::pos2(tool_x - 5.0, y(h.retract) - 22.0), egui::pos2(tool_x + 5.0, y(h.retract)));
    painter.rect_filled(tool, 2.0, t.text_dim);
    // A line at each height across the picture, with its name and value on the right; labels are
    // kept apart where heights lie close together.
    let lines = [
        (HeightLine::Clearance, "Clearance", h.clearance, t.text_dim),
        (HeightLine::Retract, "Retract", h.retract, t.warning),
        (HeightLine::Top, "Top", h.top, t.accent),
        (HeightLine::Bottom, "Bottom", h.bottom, t.accent),
    ];
    let mut label_y = f32::NEG_INFINITY;
    let mut labels: Vec<(f32, String, egui::Color32, bool)> = Vec::new();
    for (line, name, z, colour) in lines.iter().rev() {
        let on = lit == Some(*line);
        let ly = y(*z);
        let stroke = Stroke::new(if on { 2.0 } else { 1.0 }, if on { t.text } else { *colour });
        painter.add(egui::Shape::dashed_line(&[egui::pos2(rect.left(), ly), egui::pos2(block_r + 4.0, ly)], stroke, 5.0, 3.0));
        // Bottom first, upwards: each label at least a line above the last.
        let place = if label_y.is_finite() { ly.min(label_y - 13.0) } else { ly };
        label_y = place;
        labels.push((place, format!("{name} {}", shown(*z, 2)), if on { t.text } else { *colour }, on));
    }
    for (ly, text, colour, on) in labels {
        let font = egui::FontId::proportional(if on { t.font_caption } else { t.font_small });
        painter.text(egui::pos2(block_r + 8.0, ly), egui::Align2::LEFT_CENTER, text, font, colour);
    }
}

/// A labelled text box in a form's grid; `true` when its text changed this frame.
fn field(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str) -> bool {
    let need = needed(need, label);
    row_label(ui, label, need);
    let r = text_box(ui, label, value, hint, need.is_some());
    let r = explained(r, label, need);
    let changed = r.changed() | crate::params_dialog::complete(ui, &r, value);
    ui.end_row();
    changed
}

/// A number as forms show it: at most `decimals` decimals, none wasted.
fn shown(v: f64, decimals: usize) -> String {
    let s = format!("{v:.decimals$}");
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    // A value that rounds to nothing has no sign.
    if s == "-0" { "0".to_string() } else { s }
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

fn choice(ui: &mut egui::Ui, need: &[Need], id: &str, label: &str, index: &mut usize, options: &[&str]) {
    row_label(ui, label, needed(need, label));
    let r = crate::ui_kit::dropdown(ui, id, index, options);
    explained(r, label, None);
    ui.end_row();
}

/// A text value chosen from a list (`(value, label)`); a value not in the list is shown as it is.
fn pick(ui: &mut egui::Ui, need: &[Need], id: &str, label: &str, value: &mut String, options: &[(String, String)]) {
    let need = needed(need, label);
    row_label(ui, label, need);
    // Indexed for the kit's drop-down; a value not in the list shows as it is, chosen by none.
    let mut labels: Vec<&str> = options.iter().map(|(_, l)| l.as_str()).collect();
    let mut index = options.iter().position(|(v, _)| v == value).unwrap_or_else(|| {
        labels.push(value.as_str());
        labels.len() - 1
    });
    let before = index;
    let r = crate::ui_kit::dropdown(ui, id, &mut index, &labels);
    if index != before
        && let Some((v, _)) = options.get(index)
    {
        *value = v.clone();
    }
    mark(ui, &r, need);
    explained(r, label, need);
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
    /// The top and bottom the open operation form's geometry gives by itself (`None`: to be
    /// given), or why there are none; not known while the form names no geometry.
    heights: Option<Heights>,
    /// How high the selection reaches in the open operation form's setup: lowest and highest point.
    picked: Option<(f64, f64)>,
    /// The heights the open operation form's preview cuts at.
    cut_heights: Option<solidcraft_engine::PreviewHeights>,
    /// The materials stock is made of: the machine library's, then those only this design names.
    materials: Vec<String>,
    /// The material of the setup an operation form adds to (its tools' data is for it).
    material: Option<String>,
}

type Heights = Result<(Option<f64>, Option<f64>), String>;

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
    let mut materials: Vec<String> = Vec::new();
    for m in
        library["materials"].as_array().into_iter().flatten().chain(listed["materials"].as_array().into_iter().flatten()).filter_map(Value::as_str)
    {
        if !materials.iter().any(|n| same_material(n, m)) {
            materials.push(m.to_string());
        }
    }
    // The operation form's setup: the one it edits in, else the only one.
    let setups = listed["setups"].as_array().cloned().unwrap_or_default();
    let editing = match &app.manufacture.form {
        Some(Form::Op(f)) => f.editing.as_ref().map(|e| e.setup),
        _ => None,
    };
    let setup = match editing {
        Some(id) => setups.iter().find(|x| x["id"].as_u64() == Some(id)),
        None if setups.len() == 1 => setups.first(),
        None => None,
    };
    let material = setup.and_then(|x| x["material"].as_str()).map(str::to_string);
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
        // Its data in the setup's material, where it has some, else its own.
        let cut = material
            .as_deref()
            .and_then(|m| t["cutting"].as_array().into_iter().flatten().find(|c| c["material"].as_str().is_some_and(|n| same_material(n, m))));
        let value = |k: &str| cut.and_then(|c| c[k].as_f64()).or(t[k].as_f64());
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
    Choices { sketches, tools, data, heights: None, picked: None, cut_heights: None, materials, material }
}

fn form_ui(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices, need: &[Need]) {
    // A form's labels stay on one line each.
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
    if let Form::Tool(f) = form {
        tool_assistant(ui, f, need, &choices.materials);
        return;
    }
    // An operation form's heights, as a picture beside the rows (or under them in a narrow panel).
    let mut picture: Option<Picture> = None;
    let below = ui
        .horizontal_top(|ui| {
            let top = ui.cursor().top();
            form_grid(ui, form, selected, choices, need, &mut picture);
            let Some(p) = &picture else { return false };
            if ui.available_width() < PICTURE_W + 12.0 {
                return true;
            }
            ui.add_space(12.0);
            // Level with the height rows, from Clearance down to the last of them.
            let (from, to) = p.rows;
            ui.vertical(|ui| {
                ui.add_space((from - top).max(0.0));
                heights_or_note(ui, p.seen.as_ref(), p.lit, (to - from).max(120.0));
            });
            false
        })
        .inner;
    if below && let Some(p) = &picture {
        heights_or_note(ui, p.seen.as_ref(), p.lit, 160.0);
    }
}

/// An operation form's heights picture: what it shows, the row lit, and where the height rows are (their top and bottom).
struct Picture {
    seen: Option<solidcraft_engine::PreviewHeights>,
    lit: Option<HeightLine>,
    rows: (f32, f32),
}

/// A dimension of a tool's picture, lit while its row is pointed at or typed in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dim {
    Diameter,
    Flutes,
    Length,
    Shank,
    Angle,
    Tip,
}

/// A tool's shape for its picture: from a form's text (what is not typed takes its default).
#[derive(Clone, Copy, Debug, PartialEq)]
struct ToolShape {
    kind: usize,
    diameter: f64,
    flute_length: f64,
    shank: f64,
    length: f64,
    angle: f64,
    tip: f64,
}

impl ToolShape {
    fn of(f: &ToolForm) -> ToolShape {
        let mm = |s: &str| {
            s.trim()
                .trim_end_matches("mm")
                .trim_end_matches("deg")
                .trim_end_matches('°')
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite() && *v > 0.0)
        };
        ToolShape::with(f.kind, mm(&f.diameter), mm(&f.flute_length), mm(&f.shank_diameter), mm(&f.length), mm(&f.tip_angle), mm(&f.tip_diameter))
    }
    /// A shape with `kind`'s defaults wherever a value is left out (or makes no tool).
    fn with(kind: usize, d: Option<f64>, f: Option<f64>, s: Option<f64>, l: Option<f64>, a: Option<f64>, tip: Option<f64>) -> ToolShape {
        let diameter = d.unwrap_or(6.0).min(1000.0);
        let angle = a.filter(|a| *a < 179.0).unwrap_or(if kind == 2 { 118.0 } else { 90.0 });
        let tip_d = tip.filter(|t| *t < diameter).unwrap_or(0.0);
        // A V-bit cuts with its cone (as the engine takes it when left out).
        let cone = ((diameter - tip_d) * 0.5 / (angle * 0.5).to_radians().tan()).max(0.01);
        let flute_length = f.unwrap_or(if kind == 3 { cone } else { diameter * 3.0 }).min(10_000.0);
        ToolShape {
            kind,
            diameter,
            flute_length,
            shank: s.unwrap_or(diameter).min(1000.0),
            length: l.filter(|l| *l >= flute_length).unwrap_or(flute_length + diameter.max(4.0) * 2.0),
            angle,
            tip: tip_d,
        }
    }
}

/// A tool drawn upright, its point down, in `size`: shank, cutting part and point, and with
/// `dims` its dimensions D, F, L, S (and α, d for a point), `lit` drawn stronger. Returns its response.
fn tool_picture(ui: &mut egui::Ui, shape: &ToolShape, lit: Option<Dim>, size: egui::Vec2, dims: bool) -> egui::Response {
    let t = Tokens::get();
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let room = if dims { egui::vec2(40.0, 38.0) } else { egui::vec2(8.0, 8.0) };
    let inner = rect.shrink2(room);
    let widest = shape.diameter.max(shape.shank);
    let scale = (f64::from(inner.height()) / shape.length).min(f64::from(inner.width()) / widest) as f32;
    let (cx, top) = (inner.center().x, inner.center().y - (shape.length as f32 * scale) * 0.5);
    let tip_y = top + shape.length as f32 * scale;
    let cut_top = tip_y - shape.flute_length as f32 * scale;
    let (r, rs) = ((shape.diameter * 0.5) as f32 * scale, (shape.shank * 0.5) as f32 * scale);
    let ink = Stroke::new(1.2, t.text);
    // The shank, then the cutting part with its point: a polygon down one side and up the other.
    painter.add(egui::Shape::convex_polygon(
        vec![egui::pos2(cx - rs, top), egui::pos2(cx + rs, top), egui::pos2(cx + rs, cut_top), egui::pos2(cx - rs, cut_top)],
        t.text_dim.gamma_multiply(0.45),
        ink,
    ));
    let half_angle = (shape.angle * 0.5).to_radians();
    // How tall a point is over `half_width` (both in points).
    let point = |half_width: f32| (f64::from(half_width) / half_angle.tan()) as f32;
    let mut side: Vec<egui::Pos2> = vec![egui::pos2(cx + r, cut_top)];
    match shape.kind {
        // Ball: down the side to a radius above the end, round the end.
        1 => {
            side.push(egui::pos2(cx + r, tip_y - r));
            for k in 1..=16 {
                let a = std::f32::consts::FRAC_PI_2 * k as f32 / 16.0;
                side.push(egui::pos2(cx + r * a.cos(), tip_y - r + r * a.sin()));
            }
        }
        // Drill: down to the point's cone.
        2 => {
            let h = point(r).min(tip_y - cut_top);
            side.push(egui::pos2(cx + r, tip_y - h));
            side.push(egui::pos2(cx, tip_y));
        }
        // V-bit: the cone narrows to the tip's flat.
        3 => {
            let tip = (shape.tip * 0.5) as f32 * scale;
            let h = point(r - tip).min(tip_y - cut_top);
            side.push(egui::pos2(cx + r, tip_y - h));
            side.push(egui::pos2(cx + tip, tip_y));
        }
        _ => side.push(egui::pos2(cx + r, tip_y)),
    }
    let mirrored: Vec<egui::Pos2> = side.iter().rev().map(|p| egui::pos2(2.0 * cx - p.x, p.y)).collect();
    let outline: Vec<egui::Pos2> = side.iter().copied().chain(mirrored).collect();
    painter.add(egui::Shape::convex_polygon(outline, t.accent.gamma_multiply(0.45), ink));
    if !dims {
        return response;
    }
    // Dimensions: thin lines with ticks, the lit one in the accent colour.
    let colour = |d: Dim| if lit == Some(d) { t.accent } else { t.text_dim };
    let width = |d: Dim| if lit == Some(d) { 2.0 } else { 1.0 };
    let font = |d: Dim| egui::FontId::proportional(if lit == Some(d) { t.font_heading } else { t.font_body });
    let dim_line = |a: egui::Pos2, b: egui::Pos2, d: Dim, label: &str, at: egui::Pos2| {
        let stroke = Stroke::new(width(d), colour(d));
        painter.line_segment([a, b], stroke);
        let along = (b - a).normalized();
        let tick = egui::vec2(-along.y, along.x) * 4.0;
        painter.line_segment([a - tick, a + tick], stroke);
        painter.line_segment([b - tick, b + tick], stroke);
        painter.text(at, egui::Align2::CENTER_CENTER, label, font(d), colour(d));
    };
    let below = tip_y + 16.0;
    dim_line(egui::pos2(cx - r, below), egui::pos2(cx + r, below), Dim::Diameter, "D", egui::pos2(cx, below + 11.0));
    let above = top - 12.0;
    dim_line(egui::pos2(cx - rs, above), egui::pos2(cx + rs, above), Dim::Shank, "S", egui::pos2(cx + rs + 12.0, above));
    let left = cx - r.max(rs) - 16.0;
    dim_line(egui::pos2(left, top), egui::pos2(left, tip_y), Dim::Length, "L", egui::pos2(left - 12.0, (top + tip_y) * 0.5));
    let right = cx + r.max(rs) + 16.0;
    dim_line(egui::pos2(right, cut_top), egui::pos2(right, tip_y), Dim::Flutes, "F", egui::pos2(right + 12.0, (cut_top + tip_y) * 0.5));
    if matches!(shape.kind, 2 | 3) {
        // The point's angle between its two flanks, below the point.
        let d = Dim::Angle;
        let stroke = Stroke::new(width(d), colour(d));
        let reach = (r * 0.9).max(10.0);
        let (dl, dr) =
            (egui::vec2(-half_angle.sin() as f32, -half_angle.cos() as f32), egui::vec2(half_angle.sin() as f32, -half_angle.cos() as f32));
        let apex = egui::pos2(cx, tip_y);
        painter.add(egui::Shape::dashed_line(&[apex, apex + dl * reach * 1.4], stroke, 3.0, 2.0));
        painter.add(egui::Shape::dashed_line(&[apex, apex + dr * reach * 1.4], stroke, 3.0, 2.0));
        // Inside the point, just over its tip.
        let inside = if lit == Some(d) { t.accent } else { t.text };
        painter.text(apex + egui::vec2(0.0, -(r * 0.45).clamp(10.0, 26.0)), egui::Align2::CENTER_CENTER, "α", font(d), inside);
    }
    if shape.kind == 3 {
        let d = Dim::Tip;
        let tip = ((shape.tip * 0.5) as f32 * scale).max(2.0);
        dim_line(egui::pos2(cx - tip, tip_y + 5.0), egui::pos2(cx + tip, tip_y + 5.0), d, "d", egui::pos2(cx + tip + 12.0, tip_y + 5.0));
    }
    response
}

/// The tool assistant: the kind (tiles), the shape (with its dimensioned picture), the cutting data.
fn tool_assistant(ui: &mut egui::Ui, f: &mut ToolForm, need: &[Need], materials: &[String]) {
    let t = Tokens::get();
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
    // Where it is: three steps, each a button back to it.
    crate::ui_kit::segmented(ui, &mut f.step, &["1  Kind", "2  Shape", "3  Cutting data"], crate::ui_kit::Segments::default());
    ui.add_space(6.0);
    match f.step {
        0 => {
            ui.label(RichText::new("What kind of tool is it?").color(t.text_dim));
            ui.horizontal_wrapped(|ui| {
                for (k, name) in TOOL_KIND_NAMES.iter().enumerate() {
                    // Stubby, so that each kind's end shows: (diameter, cutting length, shank, length).
                    let (d, fl, sh, l) = [(10.0, 14.0, 8.0, 24.0), (10.0, 14.0, 8.0, 24.0), (8.0, 18.0, 8.0, 26.0), (14.0, 7.0, 7.0, 22.0)]
                        .get(k)
                        .copied()
                        .unwrap_or((10.0, 14.0, 8.0, 24.0));
                    let shape = ToolShape::with(k, Some(d), Some(fl), Some(sh), Some(l), None, if k == 3 { Some(0.6) } else { None });
                    // A tile of its own size: the tool drawn, its name under it; the whole tile is the button.
                    let (rect, r) = ui.allocate_exact_size(egui::vec2(112.0, 132.0), egui::Sense::click());
                    let chosen = f.kind == k;
                    let edge = if chosen || r.hovered() { t.accent } else { t.border };
                    ui.painter().rect_filled(rect, 4.0, if r.hovered() { t.hover } else { egui::Color32::TRANSPARENT });
                    ui.painter().rect_stroke(rect, 4.0, Stroke::new(if chosen { 1.5 } else { 1.0 }, edge), egui::StrokeKind::Inside);
                    let mut inside =
                        ui.new_child(egui::UiBuilder::new().max_rect(rect.shrink(6.0)).layout(egui::Layout::top_down(egui::Align::Center)));
                    tool_picture(&mut inside, &shape, None, egui::vec2(96.0, 96.0), false);
                    // Not selectable: a click on the name is a click on the tile.
                    inside.add(egui::Label::new(*name).selectable(false));
                    r.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, *name));
                    if r.clicked() {
                        f.kind = k;
                        f.step = 1;
                    }
                }
            });
        }
        1 => {
            let mut lit = None;
            ui.horizontal_top(|ui| {
                egui::Grid::new("sc_cam_tool_shape").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                    if f.editing {
                        row_label(ui, "Number", None);
                        ui.label(format!("T{}", f.number));
                        ui.end_row();
                    } else {
                        field(ui, need, "Number", &mut f.number, "1");
                    }
                    row_label(ui, "Kind", None);
                    ui.label(TOOL_KIND_NAMES.get(f.kind).copied().unwrap_or_default());
                    ui.end_row();
                    field(ui, need, "Description", &mut f.description, "");
                    let mut row = |ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str, d: Dim| {
                        if field_seen(ui, need, label, value, hint).0 {
                            lit = Some(d);
                        }
                    };
                    row(ui, "Diameter (D)", &mut f.diameter, "6 mm", Dim::Diameter);
                    row(ui, "Flute length (F)", &mut f.flute_length, if f.kind == 3 { "the cone's height" } else { "3 × D" }, Dim::Flutes);
                    row(ui, "Length (L)", &mut f.length, "F + 2 × D", Dim::Length);
                    row(ui, "Shank diameter (S)", &mut f.shank_diameter, "D", Dim::Shank);
                    if matches!(f.kind, 2 | 3) {
                        row(ui, "Point angle (α)", &mut f.tip_angle, if f.kind == 2 { "118°" } else { "90°" }, Dim::Angle);
                    }
                    if f.kind == 3 {
                        row(ui, "Tip diameter (d)", &mut f.tip_diameter, "0 mm (a sharp point)", Dim::Tip);
                    }
                    field(ui, need, "Flutes", &mut f.flutes, "2");
                });
                ui.add_space(12.0);
                tool_picture(ui, &ToolShape::of(f), lit, egui::vec2(170.0, 250.0), true);
            });
        }
        _ => {
            // Which material's data the rows show: the library's materials, and any the tool has data for.
            let mut options: Vec<(String, String)> = vec![(String::new(), "Every material".into())];
            for m in materials.iter().map(String::as_str).chain(f.others.iter().map(|(m, _)| m.as_str())).chain([f.material.as_str()]) {
                if !m.is_empty() && !options.iter().any(|(n, _)| same_material(n, m)) {
                    let has = !f.kept(m).is_empty_text();
                    options.push((m.to_string(), if has { format!("{m}  (has data)") } else { m.to_string() }));
                }
            }
            if options.len() > 1 {
                options[0].1 = "Any other material".into();
            }
            let own = f.kept("");
            let in_material = !f.material.is_empty();
            // In a material, an empty row is the tool's own value: its hint says what that is.
            let hint = |own: &str, usual: &'static str| -> String {
                match (in_material, own.trim()) {
                    (false, _) => usual.to_string(),
                    (true, "") => "the tool's: none".to_string(),
                    (true, v) => format!("the tool's: {v}"),
                }
            };
            egui::Grid::new("sc_cam_tool_cutting").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| {
                let mut chosen = f.material.clone();
                pick(ui, need, "sc_cam_tool_material", "Material", &mut chosen, &options);
                f.show_material(&chosen);
                // Speed, flutes, chip load and feed belong together: feed = speed × flutes × chip load.
                let speed_changed = field(ui, need, "Spindle (rev/min)", &mut f.spindle_rpm, &hint(&own.spindle_rpm, "none"));
                let chip_changed = field(ui, need, "Chip load (mm/tooth)", &mut f.chip_load, &hint(&own.chip_load, "feed ÷ (speed × flutes)"));
                let feed_changed = field(ui, need, "Feed (mm/min)", &mut f.feed, &hint(&own.feed, "speed × flutes × chip load"));
                let flutes = parsed(&f.flutes);
                // A material without a speed of its own cuts at the tool's.
                let rpm = if f.spindle_rpm.trim().is_empty() { own.spindle_rpm.clone() } else { f.spindle_rpm.clone() };
                link_chip_load(&rpm, flutes, &mut f.feed, &mut f.chip_load, speed_changed, feed_changed, chip_changed);
                field(ui, need, "Plunge feed (mm/min)", &mut f.plunge_feed, &hint(&own.plunge_feed, "feed / 3"));
                field(ui, need, "Stepdown (mm)", &mut f.stepdown, &hint(&own.stepdown, "half the diameter"));
                field(ui, need, "Stepover (mm)", &mut f.stepover, &hint(&own.stepover, "by operation"));
            });
            let note = if in_material {
                "Rows left empty take the tool's own value. A setup whose stock is of this material cuts with these."
            } else if options.len() > 1 {
                "The tool's own data: for setups of any material it has no data of its own for, and for setups without a material."
            } else {
                "Materials are added in the Tool Library; a tool can then have data for each."
            };
            ui.add(egui::Label::new(RichText::new(note).color(Tokens::get().text_dim)).wrap());
        }
    }
    ui.add_space(4.0);
    ui.horizontal(|ui| {
        if button(ui, "< Back", ui_kit::Button::Enabled(f.step > 0)).clicked() {
            f.step -= 1;
        }
        if button(ui, "Next >", ui_kit::Button::Enabled(f.step < 2)).clicked() {
            f.step += 1;
        }
    });
}

/// The heights picture, or what it waits for.
fn heights_or_note(ui: &mut egui::Ui, seen: Option<&solidcraft_engine::PreviewHeights>, lit: Option<HeightLine>, height: f32) {
    match seen {
        Some(h) => heights_picture(ui, h, lit, height),
        None => {
            ui.set_max_width(PICTURE_W);
            ui.add(egui::Label::new(RichText::new("A picture of the heights shows once nothing is missing").color(Tokens::get().text_dim)).wrap());
        }
    }
}

/// The heights picture's width (points).
const PICTURE_W: f32 = 170.0;

/// The form's rows; an operation form leaves its heights in `picture` for beside them.
fn form_grid(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices, need: &[Need], picture: &mut Option<Picture>) {
    egui::Grid::new("sc_cam_form").num_columns(2).spacing([12.0, 6.0]).show(ui, |ui| match form {
        // Tools have an assistant of their own (`tool_assistant`).
        Form::Tool(_) => {}
        Form::Setup(f) => {
            field(ui, need, "Name", &mut f.name, "automatic");
            choice(ui, need, "sc_cam_setup_axis", "Tool axis", &mut f.axis, &AXES);
            field(ui, need, "Stock on the sides", &mut f.side, "0 mm");
            field(ui, need, "Stock on top", &mut f.top, "0 mm");
            field(ui, need, "Stock below", &mut f.bottom, "0 mm");
            choice(ui, need, "sc_cam_setup_origin", "Origin on", &mut f.origin, &ORIGINS);
            if f.origin == 2 {
                let [x, y, z] = &mut f.point;
                field(ui, need, "Origin X", x, "0");
                field(ui, need, "Origin Y", y, "0");
                field(ui, need, "Origin Z", z, "0");
            } else {
                // Where on the box, along each of the setup's own axes.
                let [x, y, z] = &mut f.anchors;
                choice(ui, need, "sc_cam_setup_x", "Origin X", x, &["min", "center", "max"]);
                choice(ui, need, "sc_cam_setup_y", "Origin Y", y, &["min", "center", "max"]);
                choice(ui, need, "sc_cam_setup_z", "Origin Z", z, &["bottom", "middle", "top"]);
            }
            choice(ui, need, "sc_cam_setup_post", "Post", &mut f.post, solidcraft_engine::cam::Post::BUILTIN);
            let mut materials: Vec<(String, String)> = vec![(String::new(), "not said".into())];
            materials.extend(choices.materials.iter().map(|m| (m.clone(), m.clone())));
            pick(ui, need, "sc_cam_setup_material", "Material", &mut f.material, &materials);
        }
        Form::Op(f) => {
            field(ui, need, "Name", &mut f.name, "automatic");
            if f.faces.is_empty() {
                // Facing covers a box: the stock's, the part's, or the one round a sketch's profiles.
                if f.kind == OpKind::Face {
                    choice(ui, need, "sc_cam_face_boundary", "Boundary", &mut f.boundary, &FACE_BOUNDS);
                }
                if f.kind != OpKind::Face || f.boundary == 2 {
                    pick(ui, need, "sc_cam_sketch", "Sketch", &mut f.sketch, &choices.sketches);
                }
            } else {
                row_label(ui, "Faces", None);
                ui.horizontal(|ui| {
                    ui.label(format!("{} picked", f.faces.len()));
                    if button(ui, "Use a sketch", Button::Small).clicked() {
                        f.faces.clear();
                    }
                });
                ui.end_row();
            }
            if !selected.is_empty() && selected != f.faces.as_slice() {
                ui.label("");
                if button(ui, format!("Cut the {} selected face(s)", selected.len()), Button::Small).clicked() {
                    f.faces = selected.to_vec();
                }
                ui.end_row();
            }
            let from_faces = !f.faces.is_empty();
            pick(ui, need, "sc_cam_tool", "Tool", &mut f.tool, &choices.tools);
            // Another tool brings its own speed, feeds, stepdown and stepover; all stay editable here.
            prefill(f, choices);
            // Filled from the tool's data in the setup's material, when the setup says one.
            let tools = match &choices.material {
                Some(m) => format!("the tool's for {m}"),
                None => "the tool's".to_string(),
            };
            let speed_changed = field(ui, need, "Spindle (rev/min)", &mut f.spindle_rpm, &tools);
            let chip_changed = field(ui, need, "Chip load (mm/tooth)", &mut f.chip_load, "feed ÷ (speed × flutes)");
            let feed_changed = field(ui, need, "Feed (mm/min)", &mut f.feed, &tools);
            let flutes = Some(f64::from(f.flutes));
            link_chip_load(&f.spindle_rpm, flutes, &mut f.feed, &mut f.chip_load, speed_changed, feed_changed, chip_changed);
            field(ui, need, "Plunge feed (mm/min)", &mut f.plunge_feed, "the tool's, or feed / 3");
            let on_stock = f.kind == OpKind::Face && !from_faces && f.boundary < 2;
            let (top, bottom) = match (from_faces, on_stock) {
                (true, _) => ("from the faces", "from the faces"),
                (_, true) => ("the stock's top", "the part's top"),
                _ if f.kind == OpKind::Pocket => ("the sketch plane", "the pocket's floor"),
                _ => ("the sketch, or the material's top above it", "the sketch, or through the part"),
            };
            // Heights the geometry gives are shown as they are; one it does not give is asked for.
            let auto = |what: &str, z: Option<Option<f64>>| match z {
                Some(Some(z)) => format!("{} mm: {what}", shown(z, 3)),
                Some(None) => "required: type it in".to_string(),
                None => what.to_string(),
            };
            let given = lists_heights(choices);
            // Beside each, a button that takes the height from what is selected in the model.
            let (low, high) = (choices.picked.map(|p| p.0), choices.picked.map(|p| p.1));
            // The heights the preview cuts at fill the automatic ones in.
            let seen = choices.cut_heights.as_ref();
            // Known from the preview: the value; else how it is found.
            let automatic = |z: Option<f64>, what: &str| z.map_or_else(|| what.to_string(), |z| format!("{} mm (automatic)", shown(z, 3)));
            let mut lit = None;
            // Where the height rows are: the picture beside them stands as tall as they are.
            let first_row = ui.cursor().top();
            if height_field(ui, need, "Clearance", &mut f.clearance, &automatic(seen.map(|h| h.clearance), "10 mm over all"), high) {
                lit = Some(HeightLine::Clearance);
            }
            if height_field(ui, need, "Retract", &mut f.retract, &automatic(seen.map(|h| h.retract), "2 mm over the top"), high) {
                lit = Some(HeightLine::Retract);
            }
            if height_field(ui, need, "Top", &mut f.top, &auto(top, given.map(|h| h.0)), high) {
                lit = Some(HeightLine::Top);
            }
            // A chamfer's depth comes from its width.
            if f.kind != OpKind::Chamfer && height_field(ui, need, "Bottom", &mut f.bottom, &auto(bottom, given.map(|h| h.1)), low) {
                lit = Some(HeightLine::Bottom);
            }
            // Drilling has no stepdown (its peck depth is further down).
            // Not used while a number of passes divides the depth.
            let unused =
                (!f.passes.trim().is_empty()).then_some("Not used: the passes divide the depth evenly. Empty Passes to cut by the stepdown.");
            // A bore goes down on its helix (its depth per turn is further down).
            let levels = !matches!(f.kind, OpKind::Drill | OpKind::Bore | OpKind::Chamfer);
            if levels && field_unless(ui, need, "Stepdown", &mut f.stepdown, "half the tool diameter", unused).0 {
                lit = Some(HeightLine::Passes);
            }
            // Or how many passes: the depth is then divided evenly.
            if levels && field_seen(ui, need, "Passes", &mut f.passes, "by the stepdown").0 {
                lit = Some(HeightLine::Passes);
            }
            let last_row = ui.cursor().top() - ui.spacing().item_spacing.y;
            // The picture of these heights goes beside the form.
            *picture = Some(Picture { seen: seen.cloned(), lit, rows: (first_row, last_row) });
            match f.kind {
                OpKind::Face => {
                    field(ui, need, "Stepover", &mut f.stepover, "the tool's, or half its diameter");
                    choice(ui, need, "sc_cam_face_limit", "Cut", &mut f.limit, &["over the boundary", "inside the boundary"]);
                    field(
                        ui,
                        need,
                        if f.limit == 0 { "Over by" } else { "Inside by" },
                        &mut f.margin,
                        if f.limit == 0 { "the tool's radius" } else { "0 mm" },
                    );
                }
                OpKind::Contour => {
                    choice(ui, need, "sc_cam_side", "Side", &mut f.side, &SIDES);
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    // On the line there is no wall to finish.
                    if SIDES.get(f.side) != Some(&"on") {
                        field(ui, need, "Finish allowance", &mut f.finish, "none");
                    }
                    row_label(ui, "Tabs", None);
                    let r = crate::ui_kit::checkbox(ui, "Hold the part", &mut f.tabs);
                    explained(r, "Tabs", None);
                    ui.end_row();
                    if f.tabs {
                        field(ui, need, "Tab count", &mut f.tab_count, "4 on each loop");
                        field(ui, need, "Tab width", &mut f.tab_width, "6 mm");
                        field(ui, need, "Tab height", &mut f.tab_height, "2 mm");
                    }
                }
                OpKind::Pocket => {
                    field(ui, need, "Stepover", &mut f.stepover, "the tool's, or 40% of its diameter");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    field(ui, need, "Finish allowance", &mut f.finish, "none");
                    // Rest machining: what a larger tool left, in the corners and narrow places.
                    let mut options: Vec<(String, String)> = vec![(String::new(), "none: the whole pocket".into())];
                    options.extend(choices.tools.iter().filter(|(n, _)| *n != f.tool).cloned());
                    pick(ui, need, "sc_cam_rest_from", "Rest of tool", &mut f.rest_from, &options);
                }
                OpKind::Adaptive => {
                    field(ui, need, "Stepover (%)", &mut f.stepover_percent, "20");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    field(ui, need, "Finish allowance", &mut f.finish, "none");
                    field(ui, need, "Helix angle", &mut f.helix_angle, "5 deg");
                    field(ui, need, "Helix largest (%)", &mut f.helix_max, "100");
                    field(ui, need, "Helix smallest (%)", &mut f.helix_min, "10");
                    field(ui, need, "Light-cut feed (%)", &mut f.feed_up, "100: the feed");
                    let mut options: Vec<(String, String)> = vec![(String::new(), "none: the whole pocket".into())];
                    options.extend(choices.tools.iter().filter(|(n, _)| *n != f.tool).cloned());
                    pick(ui, need, "sc_cam_adaptive_rest", "Rest of tool", &mut f.rest_from, &options);
                }
                OpKind::Chamfer => {
                    f.side = f.side.min(1);
                    choice(ui, need, "sc_cam_chamfer_side", "Side", &mut f.side, &["outside", "inside"]);
                    field(ui, need, "Chamfer width", &mut f.width, "1 mm");
                    field(ui, need, "Tip offset", &mut f.tip_offset, "0.5 mm");
                }
                OpKind::Bore => {
                    field(ui, need, "Depth per turn", &mut f.pitch, "the tool's stepdown, or a quarter of its diameter");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                }
                OpKind::Drill => {
                    field(ui, need, "Peck depth", &mut f.peck, "one pass");
                    field(ui, need, "Dwell (s)", &mut f.dwell, "0");
                }
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket | OpKind::Chamfer) {
                field(ui, need, "Ramp angle", &mut f.ramp_angle, "3 deg (0: plunge)");
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket | OpKind::Bore | OpKind::Chamfer | OpKind::Adaptive) {
                row_label(ui, "Direction", None);
                crate::ui_kit::checkbox(ui, "Climb", &mut f.climb);
                ui.end_row();
            }
            row_label(ui, "Coolant", None);
            crate::ui_kit::checkbox(ui, "On", &mut f.coolant);
            ui.end_row();
        }
    });
}

/// The heights the open operation form's geometry gives, when they are known.
fn lists_heights(choices: &Choices) -> Option<(Option<f64>, Option<f64>)> {
    choices.heights.as_ref().and_then(|h| h.as_ref().ok()).copied()
}

/// What a form still needs before its command can run: rows left empty that have no value of
/// their own to fall back on, and what stops the operation's geometry. What the command could
/// still refuse (a value out of range, say) it says when OK is pressed.
fn missing(form: &Form, lists: &Choices) -> Vec<Need> {
    let mut need = Vec::new();
    let mut want = |field: &'static str, why: &str| need.push(Need { field, why: why.into() });
    let empty = |s: &str| s.trim().is_empty();
    match form {
        Form::Tool(f) => {
            if !f.editing && empty(&f.number) {
                want("Number", "the tool's number");
            }
            if empty(&f.diameter) {
                want("Diameter", "the tool's diameter");
            }
        }
        Form::Setup(f) => {
            if f.origin == 2 {
                for (field, value) in [("Origin X", &f.point[0]), ("Origin Y", &f.point[1]), ("Origin Z", &f.point[2])] {
                    if empty(value) {
                        want(field, "where the work zero is, in the world");
                    }
                }
            }
        }
        Form::Op(f) => {
            if f.faces.is_empty() && (f.kind != OpKind::Face || f.boundary == 2) && empty(&f.sketch) {
                if lists.sketches.is_empty() {
                    want("Sketch", "a sketch of what to cut: draw one first");
                } else {
                    want("Sketch", "the sketch whose profiles are cut");
                }
            }
            let tool = lists.data.iter().find(|(n, _)| *n == f.tool.trim()).map(|(_, d)| *d);
            if lists.tools.is_empty() {
                want("Tool", "a tool: add one in the Tool Library first");
            } else if empty(&f.tool) {
                want("Tool", "the tool that cuts");
            }
            // A new operation takes speed and feed from its tool; an edit keeps what it had.
            if let (Some(tool), None) = (tool, &f.editing) {
                if empty(&f.spindle_rpm) && tool.spindle_rpm.is_none() {
                    want("Spindle", "the spindle speed: this tool has none of its own");
                }
                let from_chip = parsed(&f.chip_load).is_some() && parsed(&f.spindle_rpm).is_some() && f.flutes > 0;
                if empty(&f.feed) && !from_chip && tool.feed.is_none() {
                    want("Feed", "the feed: this tool has none of its own");
                }
            }
            match &lists.heights {
                Some(Ok((top, bottom))) => {
                    if empty(&f.top) && top.is_none() {
                        want("Top", "where cutting starts");
                    }
                    if empty(&f.bottom) && bottom.is_none() {
                        want("Bottom", "how deep it cuts (the geometry does not say)");
                    }
                }
                Some(Err(e)) => want("", e),
                None => {}
            }
        }
    }
    need
}

/// What is still missing, in words: under OK, and on it while it is off.
fn need_text(need: &[Need]) -> String {
    let rows: Vec<&str> = need.iter().filter(|n| !n.field.is_empty()).map(|n| n.field).collect();
    let mut lines = Vec::new();
    if !rows.is_empty() {
        lines.push(format!("Still needed: {} (marked red; point at a row for more)", rows.join(", ")));
    }
    lines.extend(need.iter().filter(|n| n.field.is_empty()).map(|n| n.why.clone()));
    lines.join("\n")
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
    let need = missing(form, lists);
    let t = Tokens::get();
    let mut ok = false;
    let mut cancel = false;
    let shown = egui::Frame::group(ui.style()).inner_margin(10.0).show(ui, |ui| {
        ui.label(RichText::new(title).strong());
        ui.add_space(4.0);
        form_ui(ui, form, selected, lists, &need);
        ui.add_space(6.0);
        // OK stays off until nothing is missing; what is missing is said under it.
        let wanted = need_text(&need);
        ui.horizontal(|ui| {
            ok = button(ui, "OK", ui_kit::Button::Enabled(need.is_empty())).on_disabled_hover_text(&wanted).clicked();
            cancel = button(ui, "Cancel", Button::Normal).clicked();
        });
        if !need.is_empty() {
            ui.add(egui::Label::new(RichText::new(wanted).color(t.error)).wrap());
        }
        // What stops the toolpath, said before OK.
        match (&*form, &cam.preview) {
            (Form::Op(_), Some((_, _, Err(e)))) => {
                ui.add(egui::Label::new(RichText::new(format!("Cannot be cut: {e}")).color(t.error)).wrap());
            }
            (Form::Op(_), Some((_, _, Ok(solidcraft_engine::CamPreview { warning: Some(w), .. })))) => {
                ui.add(egui::Label::new(RichText::new(format!("This toolpath {w}: check Top, Bottom and the geometry")).color(t.warning)).wrap());
            }
            _ => {}
        }
    });
    // A form that has just opened is scrolled into view, its title at the panel's top.
    if cam.form_seen != Some(title) {
        shown.response.scroll_to_me_animation(Some(egui::Align::TOP), egui::style::ScrollAnimation::none());
        cam.form_seen = Some(title);
    }
    if cancel {
        cam.form = None;
        cam.error = None;
    }
    ok
}

/// Where a panel docks, and how tall it may get.
#[derive(Clone, Copy)]
struct Dock {
    /// The panel's top right corner.
    anchor: egui::Pos2,
    max_height: f32,
    /// The viewport, which a dragged panel stays inside.
    viewport: egui::Rect,
}

/// Let `band` (a panel's header) drag the panel: the drag adds to `offset`, its distance from
/// its docked place; `applied` of it is where the panel is now, after keeping it on screen. A
/// double-click docks it again. Called before the header's buttons, so they keep their clicks.
fn drag_header(ui: &egui::Ui, id: &str, band: egui::Rect, offset: &mut egui::Vec2, applied: egui::Vec2) {
    let response = ui.interact(band, egui::Id::new((id, "drag")), egui::Sense::click_and_drag());
    crate::scenario::publish_handle(&format!("{id}:title"), band.center());
    // The drag runs from where the press started, so no movement is lost while egui decides it is a drag.
    let base = egui::Id::new((id, "drag_base"));
    if response.drag_started()
        && let Some(origin) = ui.input(|i| i.pointer.press_origin())
    {
        ui.ctx().data_mut(|d| d.insert_temp(base, (applied, origin)));
    }
    if response.double_clicked() {
        *offset = egui::Vec2::ZERO;
    } else if response.dragged() || response.drag_stopped() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        let start = ui.ctx().data(|d| d.get_temp::<(egui::Vec2, egui::Pos2)>(base));
        *offset = match (start, ui.input(|i| i.pointer.latest_pos())) {
            (Some((start, from)), Some(now)) => start + (now - from),
            _ => applied + response.drag_delta(),
        };
    } else {
        if response.hovered() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
        }
        // Kept on screen: remember where it is, not where it was dragged off to.
        *offset = applied;
    }
}

/// A panel docked at the right of the viewport, as the app's command dialogs are: no title
/// bar, a header row with "−" to fold it, its name and "×" to close it, and a body that scrolls
/// when it is taller than the viewport. Its header drags it away from the dock (`offset`).
/// → whether it stays open, and where it was drawn.
#[allow(clippy::too_many_arguments)]
fn docked_panel(
    ctx: &egui::Context,
    id: &str,
    title: &str,
    dock: Dock,
    folded: &mut bool,
    offset: &mut egui::Vec2,
    look: &mut PanelLook,
    body: impl FnOnce(&mut egui::Ui),
) -> (bool, Option<egui::Rect>) {
    use crate::frame::Side;
    let t = Tokens::get();
    let mut frame = crate::frame::dock_frame(ctx, Side::Right);
    // As wide as dragged (or as a wide dialog), and never wider than the viewport has room for.
    let widest = (dock.viewport.width() - 40.0).max(PANEL_MIN_W);
    let w = if look.width > 0.0 { look.width.clamp(PANEL_MIN_W, widest) } else { DIALOG_WIDE_MAX_W.min(widest) };
    // The background fades while the pointer is elsewhere and nothing in the panel is being typed into.
    let last = ctx.memory(|m| m.area_rect(egui::Id::new(id)));
    let over = ctx.pointer_latest_pos().is_some_and(|p| last.is_some_and(|r| r.expand(6.0).contains(p)));
    let solid = ctx.animate_bool(egui::Id::new((id, "solid")), over || ctx.egui_wants_keyboard_input() || look.resizing);
    let faded = PANEL_FADES.get(look.fade).map_or(1.0, |f| f.1);
    let alpha = faded + (1.0 - faded) * solid;
    frame.fill = frame.fill.gamma_multiply(alpha);
    frame.shadow.color = frame.shadow.color.gamma_multiply(alpha);
    let margin = frame.inner_margin.leftf() + frame.inner_margin.rightf() + 2.0 * frame.stroke.width;
    // Kept inside the viewport, as wide as it was last drawn (its top right corner is the anchor).
    let width = ctx.memory(|m| m.area_rect(egui::Id::new(id))).map_or(crate::frame::DIALOG_MIN_W, |r| r.width());
    let vp = dock.viewport;
    let p = dock.anchor + *offset;
    // The kit's panels keep `space_8` off the viewport's right edge.
    let right = vp.right() - t.space_8;
    let x = if vp.left() + width <= right { p.x.clamp(vp.left() + width, right) } else { dock.anchor.x };
    let y = if vp.top() <= vp.bottom() - 40.0 { p.y.clamp(vp.top(), vp.bottom() - 40.0) } else { dock.anchor.y };
    let anchor = egui::pos2(x, y);
    let max_height = dock.max_height - (anchor.y - dock.anchor.y);
    let mut keep = true;
    let response = crate::ui_kit::Panel::new(title, egui::Id::new(id), anchor, dock.viewport, Side::Right, frame).width(w).window().show(ctx, |ui| {
        // Long texts (errors, notes) wrap instead of widening the panel.
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Wrap);
        let min_w = if look.width > 0.0 { w - margin } else { 480.0 - margin };
        ui.set_min_width(min_w);
        ui.set_max_width(w - margin);
        // What it was last frame, but never more than the panel's width now (once dragged narrower).
        let total = content_width(ui, egui::Id::new((id, "w")), min_w).min(w - margin);
        let band = egui::Rect::from_min_size(ui.cursor().min, egui::vec2(total, ui.spacing().interact_size.y + 2.0));
        drag_header(ui, id, band, offset, anchor - dock.anchor);
        ui.horizontal(|ui| {
            if crate::ui_kit::compact_button(ui, if *folded { "+" } else { "−" }, if *folded { "Expand" } else { "Collapse" }, true, false).clicked()
            {
                *folded = !*folded;
            }
            // Not selectable: a drag on the title moves the panel.
            ui.add(egui::Label::new(RichText::new(title).strong().size(t.font_label)).selectable(false))
                .on_hover_text("Drag to move, double-click to dock again");
            let width = ui.spacing().interact_size.x;
            right_aligned(ui, total, width, |ui| {
                if crate::ui_kit::compact_button(ui, "×", "Close", true, false).clicked() {
                    keep = false;
                }
            });
        });
        if *folded {
            return;
        }
        ui.add_space(2.0);
        let height = (max_height - ui.min_rect().height() - 28.0).max(80.0);
        // A sizing pass offers almost no height to an auto-sized window: give the body the viewport's.
        egui::ScrollArea::vertical().id_salt((id, "body")).min_scrolled_height(height).max_height(height).show(ui, body);
        remember_width(ui, egui::Id::new((id, "w")));
    });
    let rect = response.map(|r| r.response.rect);
    // A grip along the left edge: drag to make the panel wider or narrower, double-click for its usual width.
    look.resizing = false;
    if let Some(rect) = rect {
        let grip = egui::Rect::from_min_max(egui::pos2(rect.left() - 4.0, rect.top()), egui::pos2(rect.left() + 3.0, rect.bottom()));
        egui::Area::new(egui::Id::new((id, "grip"))).order(egui::Order::Foreground).fixed_pos(grip.min).show(ctx, |ui| {
            let (_, r) = ui.allocate_exact_size(grip.size(), egui::Sense::click_and_drag());
            if r.hovered() || r.dragged() {
                ctx.set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
            }
            if r.dragged() {
                look.width = (rect.width() - r.drag_delta().x).clamp(PANEL_MIN_W, widest);
                look.resizing = true;
            }
            if r.double_clicked() {
                look.width = 0.0;
            }
            r.on_hover_text("Drag to make the panel wider or narrower; double-click for its usual width");
        });
    }
    (keep, rect)
}

/// How wide a CAM panel is when docked (and the widest it gets without being dragged wider).
const DIALOG_WIDE_MAX_W: f32 = 560.0;

/// How wide a panel's content was last frame (at least `min`).
fn content_width(ui: &egui::Ui, key: egui::Id, min: f32) -> f32 {
    ui.ctx().data(|d| d.get_temp::<f32>(key)).unwrap_or(min).max(min)
}

/// Remembers a panel's content width for the next frame (at the end of its content).
fn remember_width(ui: &egui::Ui, key: egui::Id) {
    let w = ui.min_rect().width();
    ui.ctx().data_mut(|d| d.insert_temp(key, w));
}

/// In a row: pushes what `add` draws (`width` wide) to the right edge of a content `total` wide.
fn right_aligned<R>(ui: &mut egui::Ui, total: f32, width: f32, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let used = ui.min_rect().width();
    let gap = (total - used - width - ui.spacing().item_spacing.x).max(0.0);
    ui.add_space(gap);
    add(ui)
}

/// How a panel is shown: its width as dragged (0: its usual width), how far its background
/// fades while the pointer is elsewhere (index into [`PANEL_FADES`]), and whether it is being resized.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PanelLook {
    pub width: f32,
    pub fade: usize,
    resizing: bool,
}

/// How far a panel's background fades while the pointer is elsewhere: a name and its opacity.
const PANEL_FADES: [(&str, f32); 3] = [("off", 1.0), ("light", 0.5), ("strong", 0.25)];
/// Narrowest a panel is dragged to (points).
const PANEL_MIN_W: f32 = 400.0;

/// What a window asks for once it has been drawn.
#[derive(Default)]
struct Asked {
    run: Vec<(&'static str, Value)>,
    /// A toolbar command to start (its form opens).
    start: Option<&'static str>,
    new_tool: bool,
    form: Option<Form>,
    /// An operation to open for editing: its setup and itself, as listed.
    edit_op: Option<(Value, Value)>,
    apply: bool,
    simulate: Option<u64>,
    /// A setup whose simulated program to play.
    play: Option<u64>,
    /// Write a program: the setup, one operation of it or all (`None`), and the setup's post.
    post: Option<(u64, Option<(u64, String)>, String)>,
    import: bool,
    export: bool,
}

/// Why Simulate and Post Process are off on a setup without operations.
const NO_OPS: &str = "This setup has no operation yet: add one from the toolbar (Face, 2D Contour, 2D Pocket, Adaptive, Chamfer, Bore or Drill)";

/// The operations the toolbar offers, by their command and label.
const OPERATIONS: [(&str, &str); 7] = [
    ("cam.op.face", "Face"),
    ("cam.op.contour", "2D Contour"),
    ("cam.op.pocket", "2D Pocket"),
    ("cam.op.adaptive", "Adaptive"),
    ("cam.op.chamfer", "Chamfer"),
    ("cam.op.bore", "Bore"),
    ("cam.op.drill", "Drill"),
];

/// How far along the way to a program the job is: a setup, an operation, a simulation, a
/// program written (until the design changes). Each step is a button that does it; the line
/// under them says what comes next.
fn steps(ui: &mut egui::Ui, cam: &CamUi, setups: &[Value], asked: &mut Asked) {
    let t = Tokens::get();
    let has_ops = |s: &&Value| s["operations"].as_array().is_some_and(|o| !o.is_empty());
    // The setup the steps work on: the first with operations, else the first.
    let target = setups.iter().find(has_ops).or(setups.first());
    let id = target.and_then(|s| s["id"].as_u64());
    let done = [target.is_some(), target.is_some_and(|s| has_ops(&s)), cam.simulation.is_some(), cam.posted_now];
    let next = done.iter().position(|d| !d);
    const NAMES: [&str; 4] = ["Setup", "Operation", "Simulate", "G-code"];
    const NEXT: [&str; 4] = [
        "Next: make a setup: where the raw block is, and the zero the machine is set to.",
        "Next: add an operation: what to cut, and with which tool.",
        "Next: simulate, to check the toolpaths against the part.",
        "Next: write the G-code for the machine.",
    ];
    ui.horizontal(|ui| {
        for (k, name) in NAMES.iter().enumerate() {
            if k > 0 {
                ui.label(RichText::new("→").color(t.text_dim));
            }
            let is_done = done.get(k).copied().unwrap_or(false);
            let is_next = next == Some(k);
            // Each step needs the ones before it.
            let open = done.iter().take(k).all(|d| *d);
            let caption = RichText::new(format!("{}  {name}", k + 1));
            let (caption, fill, stroke) = match (is_done, is_next) {
                (true, _) => (caption.color(t.text), Some(t.accent_soft), None),
                (_, true) => (caption.strong().color(t.text), None, Some(Stroke::new(1.5, t.accent))),
                _ => (caption.color(t.text_dim), None, None),
            };
            let why = ["", "Make a setup first", "Add an operation first", "Add an operation first"];
            let tip = [
                "The raw block and the work zero. Click to make one, or to change it.",
                "Face, 2D Contour, 2D Pocket, Adaptive, Chamfer, Bore or Drill: click to choose one.",
                "Cut the raw block with every operation and see what is left, or cut into the part.",
                "Write the setup's program for the machine.",
            ];
            let r = ui
                .add_enabled_ui(open, |ui| tinted(ui, fill, stroke, |ui| button(ui, caption, Button::Normal)))
                .inner
                .on_hover_text(tip.get(k).copied().unwrap_or_default())
                .on_disabled_hover_text(why.get(k).copied().unwrap_or_default());
            if k == 1 {
                // The operations to choose from, under the step.
                egui::Popup::menu(&r).show(|ui| {
                    for (cmd, label) in OPERATIONS {
                        if button(ui, label, Button::Normal).clicked() {
                            asked.start = Some(cmd);
                        }
                    }
                });
                continue;
            }
            if r.clicked() {
                match k {
                    0 => match target {
                        Some(setup) => asked.form = setup_form(setup),
                        None => asked.start = Some("cam.setup.create"),
                    },
                    2 => asked.simulate = id,
                    _ => {
                        if let (Some(id), Some(setup)) = (id, target) {
                            asked.post = Some((id, None, text(setup, "post")));
                        }
                    }
                }
            }
        }
    });
    let line = match next {
        Some(k) => NEXT.get(k).copied().unwrap_or_default(),
        None => "Done: the G-code is written. A change to the design or the operations starts the checks again.",
    };
    ui.label(RichText::new(line).color(if next.is_some() { t.text } else { t.text_dim }));
}

/// The tool table's columns.
const TOOL_HEADS: [&str; 8] = ["Tool", "Kind", "Diameter", "Flutes", "Spindle", "Chip load", "Feed", "Design"];

/// A table of tools, each row picked by clicking its name. `library`: these are the machine library's tools.
fn tool_table(ui: &mut egui::Ui, id: &str, tools: &[Value], library: bool, picked: &mut Option<(bool, u64)>) {
    let t = Tokens::get();
    egui::ScrollArea::horizontal().id_salt(id).auto_shrink([false, true]).show(ui, |ui| {
        // A table's cells are never wrapped: it scrolls sideways instead.
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
        egui::Grid::new(id).num_columns(TOOL_HEADS.len()).spacing([18.0, 6.0]).striped(true).show(ui, |ui| {
            for head in TOOL_HEADS {
                ui.label(RichText::new(head).color(t.text_dim));
            }
            ui.end_row();
            for tool in tools {
                let number = tool["number"].as_u64().unwrap_or(0);
                let data = |k: &str, decimals: usize| tool[k].as_f64().map(|v| shown(v, decimals));
                // Cutting data a tool has none of: each operation is given it instead.
                let cell = |ui: &mut egui::Ui, v: Option<String>| match v {
                    Some(v) => ui.label(v),
                    None => ui.label(RichText::new("—").color(t.text_dim)).on_hover_text("Not set: operations with this tool ask for it"),
                };
                let name = text(tool, "description");
                let label = if name.is_empty() { format!("T{number}") } else { format!("T{number}  {name}") };
                if crate::ui_kit::list_picker(ui, &format!("cam_tool_{library}_{number}"), &label, *picked == Some((library, number))).clicked() {
                    *picked = if *picked == Some((library, number)) { None } else { Some((library, number)) };
                }
                ui.label(text(tool, "kind").replace('_', " "));
                cell(ui, data("diameter", 3).map(|d| format!("{d} mm")));
                cell(ui, data("flutes", 0));
                cell(ui, data("spindle_rpm", 0));
                cell(ui, data("chip_load", 4));
                cell(ui, data("feed", 1));
                // How this design's copy of a library tool stands.
                match tool["design"].as_str() {
                    Some("same") => ui.label(RichText::new("in use").color(t.text_dim)).on_hover_text("This design uses it as the library has it"),
                    Some("differs") => {
                        ui.label(RichText::new("changed").color(t.warning)).on_hover_text("Changed in the library since this design copied it")
                    }
                    _ => ui.label(""),
                };
                ui.end_row();
            }
        });
        // Room under the last row for the sideways scroll bar, which would otherwise cover it.
        ui.add_space(ui.spacing().scroll.bar_width + ui.spacing().scroll.bar_outer_margin + 4.0);
    });
}

/// The Tool Library window: the machine's tools, which every design can use. Pick a tool to
/// change or remove it; a form appears only for New Tool and Edit.
fn tools_window(ctx: &egui::Context, cam: &mut CamUi, library: &Value, lists: &Choices, dock: Dock, asked: &mut Asked) {
    let t = Tokens::get();
    let mut folded = cam.tools_folded;
    let mut offset = cam.tools_offset;
    let mut look = cam.tools_look;
    let (open, _) = docked_panel(ctx, "sc_cam_tools", "TOOL LIBRARY", dock, &mut folded, &mut offset, &mut look, |ui| {
        {
            // Everything is as wide as the panel, never wider: tables scroll inside it.
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
                if button(ui, "New Tool", Button::Normal).clicked() {
                    asked.new_tool = true;
                }
                if let Some((in_library, number)) = cam.selected_tool
                    && let Some(tool) = listed(in_library, number)
                {
                    ui.separator();
                    ui.label(RichText::new(format!("T{number}")).strong());
                    if button(ui, "Edit", Button::Normal).clicked() {
                        asked.form = tool_form(tool, in_library);
                    }
                    if button(ui, "Remove", Button::Normal).clicked() {
                        let id = if in_library { "cam.library.remove" } else { "cam.tool.remove" };
                        asked.run.push((id, json!({ "number": number })));
                    }
                    if !in_library && button(ui, "Add to library", Button::Normal).clicked() {
                        asked.run.push(("cam.library.add", json!({ "number": number })));
                    }
                    if tool["design"].as_str() == Some("differs")
                        && button(ui, "Update this design", Button::Normal)
                            .on_hover_text("Replace this design's older copy of the tool with the library's")
                            .clicked()
                    {
                        asked.run.push(("cam.tool.update", json!({ "number": number })));
                    }
                } else {
                    ui.label(RichText::new("Pick a tool to change or remove it").color(t.text_dim));
                }
                ui.separator();
                asked.import = button(ui, "Import…", Button::Normal)
                    .on_hover_text("Add the tools of a library file to this machine's library: SolidCraft's own, or an RP23CNC toollibrary.json")
                    .clicked();
                asked.export = button(ui, "Export…", Button::Normal).on_hover_text("Write this machine's library to a file").clicked();
            });
            for (k, line) in cam.import_notes.iter().enumerate() {
                let colour = if k == 0 { t.text_dim } else { t.warning };
                ui.add(egui::Label::new(RichText::new(line).color(colour)).wrap());
            }
            if matches!(cam.form, Some(Form::Tool(_))) {
                asked.apply |= form_block(ui, cam, &[], lists);
            }
            ui.add_space(6.0);
            materials_block(ui, cam, library, asked);
        }
    });
    cam.tools_folded = folded;
    cam.tools_offset = offset;
    (cam.tools_look.width, cam.tools_look.resizing) = (look.width, look.resizing);
    cam.tools_open = open;
}

/// The Manufacture window: setups with their operations, what is drawn, and the open setup or operation form.
fn manufacture_window(
    ctx: &egui::Context,
    cam: &mut CamUi,
    listed: &Value,
    lists: &Choices,
    selected: &[(String, Vec3)],
    dock: Dock,
    asked: &mut Asked,
) {
    let t = Tokens::get();
    let mut folded = cam.folded;
    let mut offset = cam.offset;
    let mut look = cam.look;
    let (open, rect) = docked_panel(ctx, "sc_cam_window", "MANUFACTURE", dock, &mut folded, &mut offset, &mut look, |ui| {
        {
            if let Some(e) = &cam.error {
                ui.label(RichText::new(e).color(t.error));
            }
            ui.horizontal_wrapped(|ui| {
                if button(ui, "Tool Library…", Button::Normal).clicked() {
                    cam.tools_open = true;
                }
                crate::ui_kit::checkbox(ui, "Toolpaths", &mut cam.show_paths);
                let widths = PATH_WIDTHS.map(|w| w.0);
                small_choice(ui, "sc_cam_path_colour", "Colour", "Toolpath colour", &mut cam.path_colour, &PATH_COLOURS);
                small_choice(ui, "sc_cam_path_width", "Width", "Toolpath width", &mut cam.path_width, &widths);
                let fades = PANEL_FADES.map(|f| f.0);
                small_choice(ui, "sc_cam_panel_fade", "Fade", "Panel fade", &mut cam.look.fade, &fades);
                crate::ui_kit::checkbox(ui, "Stock and origin", &mut cam.show_setups);
                if cam.stock.is_some() {
                    crate::ui_kit::checkbox(ui, "Simulated stock", &mut cam.show_stock);
                    // Finer shows the cut stock closer to the toolpaths, and takes longer.
                    let detail = SIM_DETAIL.map(|d| d.0);
                    if small_choice(ui, "sc_cam_sim_detail", "Detail", "Simulation detail", &mut cam.sim_detail, &detail) {
                        asked.simulate = cam.simulated;
                    }
                }
            });
            ui.add_space(4.0);
            let setups = listed["setups"].as_array().cloned().unwrap_or_default();
            steps(ui, cam, &setups, asked);
            ui.add_space(4.0);
            if setups.is_empty() {
                ui.label(RichText::new("No setup yet: New Setup on the toolbar makes the first").color(t.text_dim));
            }
            for setup in &setups {
                let id = setup["id"].as_u64().unwrap_or(0);
                // Each setup in a frame of its own, with its operations.
                egui::Frame::group(ui.style()).inner_margin(8.0).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(RichText::new(text(setup, "name")).strong());
                        ui.label(RichText::new(format!("post: {}", text(setup, "post"))).color(t.text_dim));
                        if button(ui, "Edit", Button::Small).clicked() {
                            asked.form = setup_form(setup);
                        }
                        let has_ops = setup["operations"].as_array().is_some_and(|o| !o.is_empty());
                        if ui
                            .add_enabled_ui(has_ops, |ui| button(ui, "Simulate", Button::Small))
                            .inner
                            .on_hover_text("Cut the raw block with every operation and show what is left")
                            .on_disabled_hover_text(NO_OPS)
                            .clicked()
                        {
                            asked.simulate = Some(id);
                        }
                        if button(ui, "Remove", Button::Small).clicked() {
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
                                if button(ui, label, Button::Small).clicked() {
                                    asked.run.push(("cam.op.move", json!({ "setup": id, "op": op["id"], "by": by })));
                                }
                            }
                            if button(ui, "Edit", Button::Small).clicked() {
                                asked.edit_op = Some((setup.clone(), op.clone()));
                            }
                            if ui
                                .add_enabled_ui(op.get("error").is_none(), |ui| button(ui, "Post", Button::Small))
                                .inner
                                .on_hover_text("Write this operation alone as G-code")
                                .on_disabled_hover_text("This operation cannot be cut: fix what it says first (Edit)")
                                .clicked()
                                && let Some(op_id) = op["id"].as_u64()
                            {
                                asked.post = Some((id, Some((op_id, text(op, "name"))), text(setup, "post")));
                            }
                            if button(ui, "Remove", Button::Small).clicked() {
                                asked.run.push(("cam.op.remove", json!({ "setup": id, "op": op["id"] })));
                            }
                        });
                        // On a line of its own, under the operation's buttons.
                        if let Some(w) = op.get("warning").and_then(Value::as_str) {
                            ui.horizontal(|ui| {
                                ui.add_space(24.0);
                                ui.label(RichText::new(w).color(t.warning)).on_hover_text(
                                    "The toolpath runs into the part. Simulate shows where; change the operation's heights or geometry.",
                                );
                            });
                        }
                    }
                    // The setup's whole program: the one thing everything here leads to.
                    ui.add_space(6.0);
                    let has_ops = setup["operations"].as_array().is_some_and(|o| !o.is_empty());
                    let label = RichText::new(format!("Post Process  ({})", text(setup, "post"))).strong().size(t.font_heading).color(t.text);
                    let post = ui.add_enabled_ui(has_ops, |ui| {
                        ui.vertical_centered_justified(|ui| {
                            ui.spacing_mut().button_padding.y = t.space_8;
                            tinted(ui, Some(t.accent_soft), None, |ui| button(ui, label, Button::Normal))
                        })
                        .inner
                    });
                    if post
                        .inner
                        .on_hover_text("Write every operation of this setup, in order, as one G-code program")
                        .on_disabled_hover_text(NO_OPS)
                        .clicked()
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
                play_controls(ui, cam, asked);
            }
            if matches!(cam.form, Some(Form::Setup(_) | Form::Op(_))) {
                asked.apply |= form_block(ui, cam, selected, lists);
            }
        }
    });
    cam.folded = folded;
    cam.offset = offset;
    // The panel's own changes (its width); the fade is chosen inside it, on `cam` itself.
    (cam.look.width, cam.look.resizing) = (look.width, look.resizing);
    cam.panel = rect;
    cam.open = open;
}

/// Draw the windows that are open, and keep what the viewport shows up to date. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    if !app.manufacture.open && !app.manufacture.tools_open {
        return;
    }
    tick(app, ctx);
    if app.manufacture.open && (app.manufacture.show_paths || app.manufacture.show_setups) {
        refresh(app);
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let selected = selected_faces(app);
    let library = library_listing(app, false);
    let mut lists = choices(app, &listed, &library);
    lists.heights = form_heights(app);
    lists.picked = picked_heights(app);
    update_preview(app, &lists);
    lists.cut_heights = match &app.manufacture.preview {
        Some((_, _, Ok(p))) => p.heights.clone(),
        _ => None,
    };
    // Docked like the app's command dialogs: at the right edge of their area, under the view
    // cube; left of a command dialog that is open, and the Tool Library left of the Manufacture panel.
    let vp = crate::shell::content_rect(app, app.viewport.rect.unwrap_or_else(|| ctx.content_rect()));
    let top = crate::shell::panel_top(app, vp, vp.bottom(), crate::frame::Side::Right);
    let right = app.viewport.dialog_rect.filter(|_| app.dialog.is_some()).map_or(vp.right(), |r| r.left() - 6.0);
    let max_height = vp.bottom() - top - 8.0;
    let mut asked = Asked::default();
    app.manufacture.posted_now = app.manufacture.posted_at == Some(app.session.revision);
    if app.manufacture.form.is_none() {
        app.manufacture.form_seen = None;
    }
    // The display choices are preferences, kept between runs.
    let prefs = &app.preferences;
    (app.manufacture.sim_detail, app.manufacture.path_width, app.manufacture.path_colour) =
        (prefs.cam_sim_detail, prefs.cam_path_width, prefs.cam_path_colour);
    (app.manufacture.look.width, app.manufacture.tools_look.width) = (prefs.cam_panel_width, prefs.cam_tools_width);
    (app.manufacture.look.fade, app.manufacture.tools_look.fade) = (prefs.cam_panel_fade, prefs.cam_panel_fade);
    app.manufacture.rapid_rate = f64::from(prefs.cam_rapid_rate);
    let mut cam = std::mem::take(&mut app.manufacture);
    if cam.open {
        manufacture_window(ctx, &mut cam, &listed, &lists, &selected, Dock { anchor: egui::pos2(right, top), max_height, viewport: vp }, &mut asked);
    }
    if cam.tools_open {
        // Beside where the Manufacture panel docks, wherever it was dragged to.
        // The kit's panels keep `space_8` off the viewport's right edge.
        let docked = right.min(vp.right() - Tokens::get().space_8);
        let beside = if cam.open { docked - cam.panel.map_or(DIALOG_WIDE_MAX_W, |r| r.width()) - 6.0 } else { right };
        tools_window(ctx, &mut cam, &library, &lists, Dock { anchor: egui::pos2(beside, top), max_height, viewport: vp }, &mut asked);
    }
    (app.preferences.cam_sim_detail, app.preferences.cam_path_width, app.preferences.cam_path_colour) =
        (cam.sim_detail, cam.path_width, cam.path_colour);
    (app.preferences.cam_panel_width, app.preferences.cam_tools_width, app.preferences.cam_panel_fade) =
        (cam.look.width, cam.tools_look.width, cam.look.fade);
    app.preferences.cam_rapid_rate = cam.rapid_rate as f32;
    app.manufacture = cam;
    answer(app, asked);
}

/// The machine library's materials: each with a button to remove it, and a box to add one.
fn materials_block(ui: &mut egui::Ui, cam: &mut CamUi, library: &Value, asked: &mut Asked) {
    let t = Tokens::get();
    let materials: Vec<&str> = library["materials"].as_array().into_iter().flatten().filter_map(Value::as_str).collect();
    egui::Frame::group(ui.style()).inner_margin(10.0).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Materials").strong());
            ui.label(RichText::new("what stock is made of: a tool can have cutting data for each").color(t.text_dim));
        });
        ui.add_space(4.0);
        if materials.is_empty() {
            ui.label(RichText::new("None yet. Add one, then give tools their data for it (Edit, Cutting data).").color(t.text_dim));
        }
        ui.horizontal_wrapped(|ui| {
            for m in &materials {
                ui.label(*m);
                if ui_kit::compact_button(ui, "×", &format!("Remove {m}"), true, false)
                    .on_hover_text("Tools in the library lose their data for it; designs keep theirs")
                    .clicked()
                {
                    asked.run.push(("cam.material.remove", json!({ "name": m })));
                }
                ui.add_space(6.0);
            }
        });
        ui.horizontal(|ui| {
            let (enter, _) = ui
                .scope(|ui| {
                    ui.set_max_width(180.0);
                    ui_kit::text_field(ui, &mut cam.new_material, "Plywood, MDF, Aluminium…")
                })
                .inner;
            let named = !cam.new_material.trim().is_empty();
            if (button(ui, "Add material", ui_kit::Button::Enabled(named)).clicked() || enter) && named {
                asked.run.push(("cam.material.define", json!({ "name": cam.new_material.trim() })));
                cam.new_material.clear();
            }
        });
    });
}

/// Does what a panel, the browser or a menu asked for.
fn answer(app: &mut SolveApp, mut asked: Asked) {
    if let Some(id) = asked.start.take() {
        start(app, id);
    }
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
    if let Some(setup) = asked.play {
        start_play(app, setup);
    }
    // File pickers answer later; the command runs when a file is chosen.
    if asked.import {
        app.pick_open(|app, path| {
            if let Some(path) = path {
                let result = run(app, "cam.library.import", json!({ "path": path }));
                imported(app, result);
            }
        });
    }
    if asked.export {
        app.pick_save("tools.json", &["json"], |app, path| {
            if let Some(path) = path {
                app.manufacture.error = run(app, "cam.library.export", json!({ "path": path })).err();
            }
        });
    }
    if let Some((setup, op, post)) = asked.post {
        let extension = solidcraft_engine::cam::Post::builtin(&post).map(|p| p.extension).unwrap_or_else(|| "nc".into());
        // A file named after what is in it.
        let stem: String =
            op.as_ref().map_or("program", |(_, name)| name.as_str()).chars().filter(|c| c.is_ascii_alphanumeric() || "-_ ".contains(*c)).collect();
        let stem = if stem.trim().is_empty() { "program".to_string() } else { stem };
        let op = op.map(|(id, _)| id);
        app.pick_save(&format!("{stem}.{extension}"), &[extension.as_str()], move |app, path| {
            if let Some(path) = path {
                post_to(app, setup, op, &path);
            }
        });
    }
}

/// The design's setups and their operations in the browser, under a Manufacture folder (none
/// while there is no setup). Its eye shows and hides the toolpaths; a double-click edits a setup
/// or an operation, and the right button opens their menu.
pub fn browser_rows(app: &mut SolveApp, ui: &mut egui::Ui, depth: usize) {
    use crate::browser::{Row, draw_row, is_open, toggle};
    if app.session.doc.manufacture.setups.is_empty() {
        app.manufacture.picked = None;
        app.manufacture.browser_listing = None;
        return;
    }
    // Read again once the design changed, but never while its model waits to be rebuilt (a
    // sketch drag in progress): listing would rebuild it under the drag.
    let revision = app.session.revision;
    if app.manufacture.browser_listing.as_ref().is_none_or(|(r, _)| *r != revision) && !app.session.model.is_stale() {
        let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
        app.manufacture.browser_listing = Some((revision, listed));
    }
    let setups = app.manufacture.browser_listing.as_ref().and_then(|(_, l)| l["setups"].as_array().cloned()).unwrap_or_default();
    if setups.is_empty() {
        return;
    }
    let key = "c0/manufacture";
    let open = is_open(app, key, true);
    let shown = paths_drawn(app);
    if shown {
        refresh(app);
    }
    let r = draw_row(
        ui,
        ui.id().with(key),
        &Row { depth, fold: Some(open), eye: Some(shown), icon: "folder", label: "Manufacture", dim: !shown, ..Default::default() },
    );
    if r.eye {
        app.manufacture.show_paths = !shown;
        app.manufacture.pinned_paths = !shown;
    } else if r.fold || r.clicked {
        toggle(app, key, true);
    }
    if !open {
        return;
    }
    let mut asked = Asked::default();
    for setup in &setups {
        let Some(id) = setup["id"].as_u64() else { continue };
        let skey = format!("c0/manufacture/{id}");
        let sopen = is_open(app, &skey, true);
        let name = text(setup, "name");
        let ops = setup["operations"].as_array().cloned().unwrap_or_default();
        let r = draw_row(
            ui,
            ui.id().with(("cam_setup", id)),
            &Row {
                depth: depth + 1,
                fold: (!ops.is_empty()).then_some(sopen),
                eye: Some(!app.manufacture.hidden.contains(&id)),
                dim: app.manufacture.hidden.contains(&id),
                icon: "cam_setup",
                label: &name,
                name: true,
                selected: app.manufacture.picked == Some((id, None)),
                ..Default::default()
            },
        );

        if let Some(resp) = &r.resp {
            let hover = setup.get("error").and_then(Value::as_str).map_or_else(|| format!("post: {}", text(setup, "post")), str::to_string);
            resp.clone().on_hover_text(hover);
        }
        if r.eye {
            toggle_hidden(app, id);
        } else if r.fold {
            toggle(app, &skey, true);
        } else if r.clicked {
            app.manufacture.picked = Some((id, None));
        }
        if r.double {
            asked.form = setup_form(setup);
            app.manufacture.open = true;
        }
        if let Some(at) = r.secondary {
            app.manufacture.picked = Some((id, None));
            crate::context_menu::open_for(app, at, crate::context_menu::Target::Cam { setup: id, op: None });
        }
        if !sopen {
            continue;
        }
        for op in &ops {
            let Some(op_id) = op["id"].as_u64() else { continue };
            let error = op.get("error").and_then(Value::as_str);
            let label = text(op, "name");
            // A bore is a hole too: it shares the drill's icon.
            let icon = match text(op, "kind").as_str() {
                "bore" => "cam_drill".to_string(),
                "chamfer" => "cam_contour".to_string(),
                "adaptive" => "cam_pocket".to_string(),
                kind => format!("cam_{kind}"),
            };
            let r = draw_row(
                ui,
                ui.id().with(("cam_op", id, op_id)),
                &Row {
                    depth: depth + 2,
                    eye: Some(!app.manufacture.hidden.contains(&op_id)),
                    dim: app.manufacture.hidden.contains(&op_id) || app.manufacture.hidden.contains(&id),
                    icon: &icon,
                    label: &label,
                    name: true,
                    selected: app.manufacture.picked == Some((id, Some(op_id))),
                    color: error.map(|_| Tokens::get().error),
                    ..Default::default()
                },
            );

            if let Some(resp) = &r.resp {
                let hover = error.map_or_else(
                    || {
                        format!(
                            "{}, T{}, {:.1} min",
                            text(op, "kind"),
                            op["tool"].as_u64().unwrap_or(0),
                            op["toolpath"]["cut_minutes"].as_f64().unwrap_or(0.0)
                        )
                    },
                    str::to_string,
                );
                resp.clone().on_hover_text(hover);
            }
            if r.eye {
                toggle_hidden(app, op_id);
            } else if r.clicked {
                app.manufacture.picked = Some((id, Some(op_id)));
            }
            if r.double {
                asked.edit_op = Some((setup.clone(), op.clone()));
                app.manufacture.open = true;
            }
            if let Some(at) = r.secondary {
                app.manufacture.picked = Some((id, Some(op_id)));
                crate::context_menu::open_for(app, at, crate::context_menu::Target::Cam { setup: id, op: Some(op_id) });
            }
        }
    }
    answer(app, asked);
}

/// Shows or hides the toolpaths of a setup or an operation (by its id).
fn toggle_hidden(app: &mut SolveApp, id: u64) {
    let hidden = &mut app.manufacture.hidden;
    if !hidden.remove(&id) {
        hidden.insert(id);
    }
}

/// A menu entry that `menu_action` does: `what` to the setup, or to one operation of it.
fn menu_item(label: &str, icon: &str, what: &str, setup: u64, op: Option<u64>) -> crate::context_menu::Item {
    let mut item = crate::context_menu::Item::action("ui.cam", label, icon);
    item.params = Some(json!({ "do": what, "setup": setup, "op": op }));
    item
}

/// The menu of a setup, or of one operation of it, in the browser.
pub fn menu_items(app: &SolveApp, setup: u64, op: Option<u64>) -> Vec<crate::context_menu::Item> {
    use crate::context_menu::Item;
    let Some(s) = app.session.doc.manufacture.setups.iter().find(|s| s.id == setup) else { return Vec::new() };
    let ops: Vec<u64> = s.operations.iter().map(|o| o.id).collect();
    let disable = |mut item: Item, on: bool| {
        item.enabled = on;
        item
    };
    match op {
        None => vec![
            menu_item("Edit Setup", "edit", "edit", setup, None),
            disable(menu_item("Simulate", "cam_simulate", "simulate", setup, None), !ops.is_empty()),
            disable(menu_item("Post Process", "cam_post", "post", setup, None), !ops.is_empty()),
            Item::sep(),
            menu_item("Remove Setup", "delete", "remove", setup, None),
        ],
        Some(op) => {
            let at = ops.iter().position(|o| *o == op);
            vec![
                menu_item("Edit Operation", "edit", "edit", setup, Some(op)),
                menu_item("Post Process", "cam_post", "post", setup, Some(op)),
                Item::sep(),
                disable(menu_item("Move Up", "", "up", setup, Some(op)), at.is_some_and(|i| i > 0)),
                disable(menu_item("Move Down", "", "down", setup, Some(op)), at.is_some_and(|i| i + 1 < ops.len())),
                Item::sep(),
                menu_item("Remove Operation", "delete", "remove", setup, Some(op)),
            ]
        }
    }
}

/// Does a browser menu's entry (`ui.cam`): `p` is `{do, setup, op?}`.
pub fn menu_action(app: &mut SolveApp, p: &Value) {
    let (Some(setup), op) = (p["setup"].as_u64(), p["op"].as_u64()) else { return };
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    let Some(s) = listed["setups"].as_array().into_iter().flatten().find(|s| s["id"].as_u64() == Some(setup)).cloned() else { return };
    let o = op.and_then(|op| s["operations"].as_array().into_iter().flatten().find(|o| o["id"].as_u64() == Some(op)).cloned());
    let mut asked = Asked::default();
    match (p["do"].as_str().unwrap_or(""), &o) {
        ("edit", None) => asked.form = setup_form(&s),
        ("edit", Some(o)) => asked.edit_op = Some((s.clone(), o.clone())),
        ("simulate", None) => asked.simulate = Some(setup),
        ("post", None) => asked.post = Some((setup, None, text(&s, "post"))),
        ("post", Some(o)) => asked.post = Some((setup, o["id"].as_u64().map(|id| (id, text(o, "name"))), text(&s, "post"))),
        ("up" | "down", Some(o)) => {
            let by = if p["do"] == "up" { -1 } else { 1 };
            asked.run.push(("cam.op.move", json!({ "setup": setup, "op": o["id"], "by": by })));
        }
        ("remove", None) => asked.run.push(("cam.setup.remove", json!({ "setup": setup }))),
        ("remove", Some(o)) => asked.run.push(("cam.op.remove", json!({ "setup": setup, "op": o["id"] }))),
        _ => {}
    }
    // What is edited or simulated shows in the Manufacture panel.
    app.manufacture.open = true;
    answer(app, asked);
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
            // A whole setup's program finishes the steps (until the design changes).
            if op.is_none() {
                app.manufacture.posted_at = Some(app.session.revision);
            }
            app.set_status(format!("Wrote {} lines of G-code", v["lines"].as_u64().unwrap_or(0)), false);
        }
        Err(e) => app.manufacture.error = Some(e),
    }
}

#[cfg(test)]
#[path = "cam_ui_tests.rs"]
mod tests;
