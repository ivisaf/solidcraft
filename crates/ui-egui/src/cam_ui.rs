//! The Manufacture window and the toolpaths drawn in the viewport.
//!
//! A window over the engine's `cam.*` commands: the tool library, the setups with their
//! operations (each with its cutting time, or the error that stops it), a form for whichever
//! command the toolbar started, and Post Process. Toolpaths come from `cam.toolpath` and are
//! redrawn when the design changes.

use crate::ui_kit::{self, Button, StatusTone, button};
use egui::{RichText, Stroke};
use serde_json::{Value, json};
use solidcraft_engine::geom::Vec3;

use crate::SolveApp;
use crate::i18n::{format as tf, tr_manufacture as tm};
use crate::theme::Tokens;
use crate::viewport::Proj;

const TOOL_KINDS: [&str; 7] = ["flat", "ball", "drill", "vbit", "bull", "chamfer", "engraver"];
/// What each of [`TOOL_KINDS`] is called in the tool assistant.
const TOOL_KIND_NAMES: [&str; 7] = ["Flat end mill", "Ball end mill", "Drill", "V-bit", "Bull-nose end mill", "Chamfer mill", "Engraving tool"];

/// The kind of tool [`TOOL_KINDS`]`[index]` is.
fn tool_kind(index: usize) -> solidcraft_engine::cam::ToolKind {
    use solidcraft_engine::cam::ToolKind;
    match index {
        1 => ToolKind::BallEndmill,
        2 => ToolKind::Drill,
        3 => ToolKind::VBit,
        4 => ToolKind::BullNose,
        5 => ToolKind::ChamferMill,
        6 => ToolKind::Engraver,
        _ => ToolKind::FlatEndmill,
    }
}
const SIDES: [&str; 3] = ["outside", "inside", "on"];
/// What a facing operation's boundary is, when no faces are picked.
const FACE_BOUNDS: [&str; 3] = ["stock", "part", "sketch"];
/// Whether a facing operation cuts over its boundary or stays inside it.
const FACE_LIMITS: [&str; 2] = ["over", "inside"];
/// Tool axes a setup form offers; the last stands for a direction set by command, kept as it is.
const AXES: [&str; 7] = ["+z", "-z", "+x", "-x", "+y", "-y", "custom"];
const ANCHORS: [&str; 3] = ["min", "center", "max"];
/// What the work origin is on.
const ORIGINS: [&str; 4] = ["stock box", "part box", "world point", "point of the model"];
/// Which point of a picked edge the work origin is at, as `origin`'s `on` names it, and as forms say it.
const EDGE_POINTS: [(&str, &str); 3] = [("vertex", "corner"), ("middle", "middle of the edge"), ("center", "centre of the circle")];
/// Where a work origin on the model is along the tool axis: on a box (`of`, `z`), or the point's own height.
const ORIGIN_ZS: [(Option<(&str, &str)>, &str); 5] = [
    (Some(("stock", "max")), "stock top"),
    (Some(("stock", "min")), "stock bottom"),
    (Some(("part", "max")), "part top"),
    (Some(("part", "min")), "part bottom"),
    (None, "the point"),
];
/// Toolpath line widths: a name and the cutting moves' width in points (rapids are thinner).
const PATH_WIDTHS: [(&str, f32); 3] = [("thin", 1.0), ("normal", 1.5), ("thick", 2.5)];
/// From how thick material left on the part is shown in the simulated stock (mm): finer than a
/// finishing pass's cusps, it is drawn as the part.
const LEFTOVER_SHOWN: [(&str, f64); 5] = [("0.001 mm", 0.001), ("0.01 mm", 0.01), ("0.05 mm", 0.05), ("0.1 mm", 0.1), ("0.2 mm", 0.2)];
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
    explained(ui.label(tm(label)), help_label, None);
    let options: Vec<&str> = options.iter().map(|o| tm(o)).collect();
    fitted_dropdown(ui, id, index, &options, 0.0);
    *index != before
}

/// The kit's drop-down, only as wide as its longest option (and at least `least`): it
/// otherwise fills the row, and a form's grid then leaves no room for the heights picture.
fn fitted_dropdown(ui: &mut egui::Ui, id: &str, index: &mut usize, options: &[&str], least: f32) -> egui::Response {
    let width = (ui_kit::ContentWidth::dropdown(ui, options) + ui_kit::ContentWidth::label(ui, " ")).max(least);
    ui.allocate_ui(egui::vec2(width, ui_kit::row_height()), |ui| crate::ui_kit::dropdown(ui, id, index, options)).inner
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
    /// Adaptive roughing down to the model in 3D.
    Adaptive3d,
    /// Finishing the model in 3D with passes side by side.
    Parallel3d,
    /// Finishing the model's steep walls in 3D level by level.
    ZLevel3d,
    Chamfer,
    VCarve,
    Bore,
    Thread,
    Drill,
}

impl OpKind {
    /// Whether it follows the model's shape in 3D, so that faces of the part can have roles in it.
    fn follows_the_model(self) -> bool {
        matches!(self, OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d)
    }
    fn command(self) -> &'static str {
        match self {
            OpKind::Face => "cam.op.face",
            OpKind::Contour => "cam.op.contour",
            OpKind::Pocket => "cam.op.pocket",
            OpKind::Adaptive => "cam.op.adaptive",
            OpKind::Adaptive3d => "cam.op.adaptive3d",
            OpKind::Parallel3d => "cam.op.parallel3d",
            OpKind::ZLevel3d => "cam.op.zlevel3d",
            OpKind::Chamfer => "cam.op.chamfer",
            OpKind::VCarve => "cam.op.vcarve",
            OpKind::Thread => "cam.op.thread",
            OpKind::Bore => "cam.op.bore",
            OpKind::Drill => "cam.op.drill",
        }
    }
    fn title(self) -> &'static str {
        match self {
            OpKind::Face => "FACING",
            OpKind::Contour => "2D CONTOUR",
            OpKind::Pocket => "2D POCKET",
            OpKind::Adaptive => "ADAPTIVE",
            OpKind::Adaptive3d => "3D ADAPTIVE",
            OpKind::Parallel3d => "3D PARALLEL",
            OpKind::ZLevel3d => "3D Z-LEVEL",
            OpKind::Chamfer => "CHAMFER",
            OpKind::VCarve => "V-CARVE",
            OpKind::Thread => "THREAD",
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
    /// The speed at the cutting edge (m/min), kept in step with the spindle speed: speed =
    /// surface speed × 1000 / (π × diameter).
    pub surface_speed: String,
    /// Which of each pair the tool keeps: the surface speed (else the spindle speed), the chip
    /// load (else the feed); the other is worked out from it. The last one typed is kept.
    pub by_surface_speed: bool,
    pub by_chip_load: bool,
    /// The point (a V-bit's, a drill's: its included angle in degrees; a V-bit's flat in mm),
    /// and the shank's diameter and the length out of the holder (mm). Empty: the defaults.
    pub tip_angle: String,
    pub tip_diameter: String,
    /// A bull nose's corner radius (mm).
    pub corner_radius: String,
    /// An end mill's taper, each side (degrees). Empty: straight.
    pub taper_angle: String,
    pub shank_diameter: String,
    pub length: String,
    /// The holder above the tool (mm): its diameter and length. Empty: not known.
    pub holder_diameter: String,
    pub holder_length: String,
    /// The holder by its shape: one of the library's by name ("": the plain one above), and the
    /// one the tool sat in when the form opened (its copy is kept unless another is picked).
    pub holder: String,
    pub holder_was: String,
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
    pub surface_speed: String,
    /// As [`ToolForm::by_surface_speed`] and [`ToolForm::by_chip_load`].
    pub by_surface_speed: bool,
    pub by_chip_load: bool,
}

impl CutText {
    /// Whether nothing is typed in it.
    fn is_empty_text(&self) -> bool {
        [&self.spindle_rpm, &self.feed, &self.plunge_feed, &self.stepdown, &self.stepover, &self.chip_load, &self.surface_speed]
            .iter()
            .all(|v| v.trim().is_empty())
    }
    /// What the command is sent for its speed and feed: of each pair, the one kept (an empty one
    /// is sent as nothing).
    fn rates(&self) -> [(&'static str, &'static str, &String); 2] {
        [
            match self.by_surface_speed {
                true => ("surface_speed", "The surface speed", &self.surface_speed),
                false => ("spindle_rpm", "The spindle speed", &self.spindle_rpm),
            },
            match self.by_chip_load {
                true => ("chip_load", "The chip load", &self.chip_load),
                false => ("feed", "The feed", &self.feed),
            },
        ]
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
            surface_speed: self.surface_speed.clone(),
            by_surface_speed: self.by_surface_speed,
            by_chip_load: self.by_chip_load,
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
        self.show(c);
        self.material = name;
    }
    /// Puts `c` in the rows.
    fn show(&mut self, c: CutText) {
        (self.spindle_rpm, self.feed, self.plunge_feed) = (c.spindle_rpm, c.feed, c.plunge_feed);
        (self.stepdown, self.stepover, self.chip_load) = (c.stepdown, c.stepover, c.chip_load);
        (self.surface_speed, self.by_surface_speed, self.by_chip_load) = (c.surface_speed, c.by_surface_speed, c.by_chip_load);
    }
    /// Works out the speed and feed rows from the ones each kept data has, the tool's own shown.
    fn work_out_rates(&mut self) {
        let (diameter, flutes) = (self.diameter_mm(), parsed(&self.flutes));
        let mut own = self.shown();
        link_tool_rates(&mut own, Typed::default(), diameter, flutes, None);
        let own_rpm = rpm_of(&own, diameter);
        self.show(own);
        for (_, c) in &mut self.others {
            link_tool_rates(c, Typed::default(), diameter, flutes, own_rpm);
        }
    }
    /// The diameter typed, in mm (none for an expression).
    fn diameter_mm(&self) -> Option<f64> {
        parsed(self.diameter.trim().trim_end_matches("mm"))
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
    /// A work origin on the model: the body and the point its edge (or vertex) was picked at,
    /// which point of the edge ([`EDGE_POINTS`]) and its height ([`ORIGIN_ZS`]).
    pub edge: Option<(String, Vec3)>,
    pub edge_on: usize,
    pub edge_z: usize,
    /// Index into [`STOCK_SHAPES`]; a set box's size along X, Y and Z, and a round bar's diameter.
    pub stock_shape: usize,
    pub stock_size: [String; 3],
    pub diameter: String,
    /// Stock offsets.
    pub side: String,
    pub top: String,
    pub bottom: String,
    /// The bodies that are the part, as typed: names apart by commas (empty: every body).
    pub bodies: String,
    /// The bodies that hold the stock, as typed: names apart by commas (empty: none).
    pub fixtures: String,
    pub post: usize,
    /// What the stock is made of ("": not said).
    pub material: String,
    /// Index into [`WORK_OFFSETS`].
    pub work_offset: usize,
    /// The earlier setup whose result is the stock (its id; "": the stock box).
    pub stock_from: String,
    /// Clamps no tool may enter, each as typed: X from, Y from, X to, Y to, top (work coordinates).
    pub clamps: Vec<[String; 5]>,
    /// Copies of the part as typed: how many along X and Y, and how far apart (empty: one part).
    pub copies: [String; 4],
    /// Index into [`X_DIRECTIONS`].
    pub x_axis: usize,
}

/// The shapes a setup's stock can have, as `cam.setup.create` names them, and as forms say them.
const STOCK_SHAPES: [(&str, &str); 3] = [("offset", "box round the part"), ("box", "box of a set size"), ("round", "round bar")];

/// A setup's X directions: worked out from the axis, or along a world axis.
const X_DIRECTIONS: [&str; 5] = ["automatic", "+x", "-x", "+y", "-y"];

/// The work offsets a setup's programs can run in.
const WORK_OFFSETS: [&str; 6] = ["G54", "G55", "G56", "G57", "G58", "G59"];

/// The operation a form edits, and the geometry it had when the form opened.
#[derive(Clone, Debug, PartialEq)]
pub struct Editing {
    pub setup: u64,
    pub op: u64,
    /// Index into [`FACE_BOUNDS`] (facing only).
    pub boundary: usize,
    pub sketch: String,
    pub faces: Vec<(String, Vec3)>,
    pub face_roles: Vec<FaceRoleRow>,
}

/// A face with a role in a 3D operation form: the face (its body and a point on it), and what it
/// is to the operation, with how far the tool keeps off it (avoid) or what is left on it (leave), mm.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRoleRow {
    pub body: String,
    pub at: Vec3,
    pub role: FaceRoleKind,
    pub mm: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceRoleKind {
    Ignore,
    Avoid,
    Leave,
}

impl FaceRoleRow {
    /// As `face_roles` takes it.
    fn json(&self) -> Value {
        let mut v = json!({ "body": self.body, "point": [self.at.x, self.at.y, self.at.z] });
        match self.role {
            FaceRoleKind::Ignore => v["role"] = json!("ignore"),
            FaceRoleKind::Avoid => (v["role"], v["clearance"]) = (json!("avoid"), json!(self.mm)),
            FaceRoleKind::Leave => (v["role"], v["stock_to_leave"]) = (json!("leave"), json!(self.mm)),
        }
        v
    }
    /// One of an operation's `face_roles` as `cam.list` gives them.
    fn of(v: &Value) -> Option<FaceRoleRow> {
        let (role, mm) = match v["role"]["role"].as_str()? {
            "ignore" => (FaceRoleKind::Ignore, 0.0),
            "avoid" => (FaceRoleKind::Avoid, v["role"]["clearance"].as_f64()?),
            "leave" => (FaceRoleKind::Leave, v["role"]["stock_to_leave"].as_f64()?),
            _ => return None,
        };
        Some(FaceRoleRow { body: v["face"]["body"].as_str()?.to_string(), at: point(&v["face"]["at"])?, role, mm })
    }
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
    /// 3D operations: faces of the part with a role other than being machined, and the clearance
    /// and allowance the next ones given a role get (empty: 1 mm to avoid; none to leave yet).
    pub face_roles: Vec<FaceRoleRow>,
    pub role_clearance: String,
    pub role_leave: String,
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
    /// A contour's lead-in radius (empty: none), its onion skin (empty: none) and whether a last
    /// pass takes the skin off; dogbones in sharp inside corners (contour and pocket).
    pub lead: String,
    pub skin: String,
    pub skin_last: bool,
    pub dogbones: bool,
    /// How far a bore's helix goes down in one turn.
    pub pitch: String,
    /// The deepest a V-carve goes (empty: as deep as its shape asks).
    pub max_depth: String,
    /// A thread's major diameter (empty: from the drawn circle), whether it is round a boss
    /// rather than in a hole, and its passes out to the full depth (empty: one).
    pub thread_diameter: String,
    pub thread_outside: bool,
    pub thread_passes: String,
    /// A left-hand thread.
    pub thread_left: bool,
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
    /// 3D roughing: how far apart its levels in between on slopes are at most (empty: a quarter
    /// of the stepdown).
    pub step_up: String,
    /// 3D finishing: which way its passes run (degrees from X; empty: 0), and back and forth.
    pub pass_angle: String,
    pub zigzag: bool,
    /// 3D Z-level finishing: how far apart its levels are (empty: a twelfth of the tool's
    /// diameter), and from how steep a wall it takes (degrees; empty: 30).
    pub level_step: String,
    pub steep: String,
    /// 3D finishing: only where earlier operations left material on the model.
    pub rest3d: bool,
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
    /// Where each column's material starts (empty: on the floor everywhere).
    pub bottom: Vec<f64>,
    /// Per column: below 0 cut into the part, above 0 left on it. Empty when there was no part to compare with.
    pub deviation: Vec<f64>,
}

impl StockField {
    fn from_json(v: &Value) -> Option<StockField> {
        let numbers = |k: &str| v[k].as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(0.0)).collect::<Vec<f64>>());
        let (nx, ny) = (v["nx"].as_u64()? as usize, v["ny"].as_u64()? as usize);
        let z = numbers("z")?;
        let deviation = numbers("deviation").unwrap_or_default();
        let bottom = numbers("bottom").unwrap_or_default();
        let f = &v["frame"];
        let fits = |v: &Vec<f64>| v.is_empty() || v.len() == z.len();
        (nx.checked_mul(ny) == Some(z.len()) && fits(&deviation) && fits(&bottom)).then_some(StockField {
            frame: [point(&f["origin"])?, point(&f["x"])?, point(&f["y"])?, point(&f["z"])?],
            origin: [v["origin"][0].as_f64()?, v["origin"][1].as_f64()?],
            cell: v["cell"].as_f64()?,
            nx,
            ny,
            floor: v["floor"].as_f64()?,
            z,
            bottom,
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
            bottom: field.bottom.clone(),
            deviation: Vec::new(),
        }
    }

    /// A point in work coordinates, in the world.
    fn world(&self, q: Vec3) -> Vec3 {
        let [o, x, y, z] = self.frame;
        o + x * q.x + y * q.y + z * q.z
    }

    /// Column `(i, j)` in the world: the top of it at its centre, the bottom of its material
    /// under that, its deviation from the part, and whether it holds material.
    fn column(&self, i: usize, j: usize) -> Option<(Vec3, Vec3, f64, bool)> {
        let k = j.checked_mul(self.nx)?.checked_add(i)?;
        let z = *self.z.get(k).filter(|_| i < self.nx && j < self.ny)?;
        let base = self.bottom.get(k).map_or(self.floor, |b| b.max(self.floor));
        let (cx, cy) = (self.origin[0] + (i as f64 + 0.5) * self.cell, self.origin[1] + (j as f64 + 0.5) * self.cell);
        let (top, under) = (self.world(Vec3::new(cx, cy, z)), self.world(Vec3::new(cx, cy, base)));
        Some((top, under, self.deviation.get(k).copied().unwrap_or(0.0), z > base + 1e-9))
    }
}

/// Cuts into the part smaller than this are drawn as the part (mm).
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

/// Whether the body called `name` holds the stock in a setup: it stays drawn beside a simulated stock.
pub fn is_fixture(app: &SolveApp, name: &str) -> bool {
    app.session.doc.manufacture.setups.iter().any(|s| s.fixtures.iter().any(|f| f.eq_ignore_ascii_case(name)))
}

/// Changes when what the viewport draws of the stock changes.
pub fn stock_key(app: &SolveApp) -> (bool, u64, usize) {
    (stock_shown(app), app.manufacture.stock_stamp, app.preferences.cam_leftover_shown)
}

/// The simulated stock as triangles: its top through the column centres, its underside where
/// it was cut from below, and walls down round the edge, each with what it is. Where all of a
/// triangle's columns are cut through there is none: a hole. Material left on the part is
/// marked from `leftover` thick (mm), a cut into it from the smallest.
pub fn stock_triangles(field: &StockField, leftover: f64) -> Vec<([Vec3; 3], StockMark)> {
    let mut out = Vec::new();
    let mark = |d: [f64; 3]| {
        if d.iter().any(|d| *d < -SHOWN_DEVIATION) {
            StockMark::Gouge
        } else if d.iter().any(|d| *d > leftover.max(SHOWN_DEVIATION)) {
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
            for [p, q, r] in [[a, b, c], [a, c, d]].into_iter().filter(|t| t.iter().any(|x| x.3)) {
                out.push(([p.0, q.0, r.0], mark([p.2, q.2, r.2])));
                if !field.bottom.is_empty() {
                    out.push(([p.1, r.1, q.1], StockMark::Fine));
                }
            }
        }
    }
    // Walls round the edge, outward-facing: along each side, from the top's rim down to the bottom.
    let mut wall = |a: Option<(Vec3, Vec3, f64, bool)>, b: Option<(Vec3, Vec3, f64, bool)>| {
        if let (Some((a, a_floor, _, _)), Some((b, b_floor, _, _))) = (a, b) {
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
    let leftover = LEFTOVER_SHOWN.get(app.preferences.cam_leftover_shown).map_or(0.05, |l| l.1);
    for (tri, mark) in stock_triangles(field, leftover) {
        let [a, b, c] = tri;
        let n = (b - a).cross(c - a).normalized().unwrap_or(Vec3::new(0.0, 0.0, 1.0)).to_f32();
        let colour = sc.color(rgba(match mark {
            StockMark::Fine => t.body,
            StockMark::Gouge => t.error,
            StockMark::Leftover => t.warning,
        }));
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
    /// A round stock's centre and radius across, in work coordinates (its box is `stock`).
    pub round: Option<(Vec3, f64)>,
    /// Its clamps as boxes in work coordinates: from the stock's bottom up to each one's top.
    pub clamps: Vec<(Vec3, Vec3)>,
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
    /// A holder as typed in the Tool Library window, to add to the library or change there.
    pub new_holder: HolderDraft,
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
    /// How the next program is written: an optional stop before each operation after the
    /// first, and an air cut (every Z raised by `air_height` mm).
    pub optional_stops: bool,
    pub air_cut: bool,
    pub air_height: f64,
    /// A program waiting to be written once its rapids into material are confirmed.
    pub confirm_post: Option<PendingPost>,
    /// The setup last simulated (simulated again when the detail changes), and what of it.
    simulated: Option<u64>,
    scope: SimScope,
    /// The design revision a program was last written at, and whether that is the revision now.
    posted_at: Option<u64>,
    posted_now: bool,
    /// The title of the form last shown (one that opens is scrolled into view).
    form_seen: Option<&'static str>,
    /// The open form as it opened: when it differs, the form has changes a click in the
    /// browser does not throw away.
    form_opened: Option<Form>,
    /// What the last tool import took and left out, shown in the Tool Library window.
    pub import_notes: Vec<String>,
    /// Setups and operations whose toolpaths are not drawn (the browser's eyes).
    pub hidden: std::collections::BTreeSet<u64>,
    /// The setup and the operation of each toolpath in `cache`, in the same order.
    path_ids: Vec<(u64, u64)>,
    /// The open operation form's toolpath before OK (or why there is none), for the design
    /// revision and the command it was worked out for.
    preview: Option<(u64, Value, Result<solidcraft_engine::CamPreview, String>)>,
    /// A 3D roughing form's preview being worked out off the window thread: the revision and
    /// command it is for, and where it arrives.
    #[cfg(not(target_arch = "wasm32"))]
    preview_job: Option<(u64, Value, std::sync::mpsc::Receiver<Result<solidcraft_engine::CamPreview, String>>)>,
    /// `cam.list` worked out on a copy off the window thread (for the design revision given):
    /// 3D roughing toolpaths take seconds; the copy shares the engine's cache of them.
    #[cfg(not(target_arch = "wasm32"))]
    listing_job: Option<(u64, std::sync::mpsc::Receiver<()>)>,
    /// The last `cam.list` read: shown while a newer one is worked out.
    last_listing: Value,
    /// To ask for a frame when work done off the window thread arrives.
    repaint: Option<egui::Context>,
    /// The toolpaths of every setup, for the design revision they were worked out at.
    cache: Option<(u64, Vec<Path>, Vec<SetupView>)>,
    /// The open operation form's own heights, for the design revision, the command and the geometry asked about.
    heights: Option<(u64, &'static str, Value, Heights)>,
    /// How high the selection reaches, for the design revision and the setup and selection asked about.
    selection_heights: Option<(u64, Value, Option<(f64, f64)>)>,
    /// The corner or edge a setup's origin can be put on, for the design revision and selection asked about.
    origin_pick: Option<(u64, Value, Option<(String, Vec3, solidcraft_engine::doc::manufacture::EdgePoint)>)>,
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
            optional_stops: false,
            air_cut: false,
            air_height: 20.0,
            confirm_post: None,
            play: None,
            open: false,
            tools_open: false,
            folded: false,
            tools_folded: false,
            panel: None,
            picked: None,
            pinned_paths: false,
            browser_listing: None,
            selected_tool: None,
            new_material: String::new(),
            new_holder: HolderDraft::default(),
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
            simulated: None,
            scope: SimScope::default(),
            posted_at: None,
            posted_now: false,
            form_seen: None,
            form_opened: None,
            import_notes: Vec::new(),
            hidden: Default::default(),
            path_ids: Vec::new(),
            preview: None,
            #[cfg(not(target_arch = "wasm32"))]
            preview_job: None,
            #[cfg(not(target_arch = "wasm32"))]
            listing_job: None,
            last_listing: Value::Null,
            repaint: None,
            cache: None,
            heights: None,
            selection_heights: None,
            origin_pick: None,
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
    let listed = listed(app).0;
    let lists = {
        let library = library_listing(app, true);
        choices(app, &listed, &library)
    };
    let first_tool = lists.tools.first().map(|t| t.0.clone()).unwrap_or_default();
    let op = |kind: OpKind| {
        // The newest sketch, except for 3D roughing, which roughs the whole stock unless told otherwise.
        let sketch = if matches!(kind, OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d) {
            String::new()
        } else {
            app.session
                .doc
                .features
                .iter()
                .rev()
                .find(|f| matches!(f.kind, solidcraft_engine::doc::FeatureKind::Sketch { .. }))
                .map(|f| f.name.clone())
                .unwrap_or_default()
        };
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
            face_roles: Vec::new(),
            role_clearance: String::new(),
            role_leave: String::new(),
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
            max_depth: String::new(),
            thread_diameter: String::new(),
            thread_outside: false,
            thread_passes: String::new(),
            thread_left: false,
            lead: String::new(),
            skin: String::new(),
            skin_last: false,
            dogbones: false,
            pitch: String::new(),
            rest_from: String::new(),
            stepover_percent: String::new(),
            helix_angle: String::new(),
            helix_max: String::new(),
            helix_min: String::new(),
            feed_up: String::new(),
            step_up: String::new(),
            pass_angle: String::new(),
            zigzag: true,
            level_step: String::new(),
            steep: String::new(),
            rest3d: false,
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
                surface_speed: String::new(),
                by_surface_speed: false,
                by_chip_load: false,
                tip_angle: String::new(),
                tip_diameter: String::new(),
                corner_radius: String::new(),
                taper_angle: String::new(),
                shank_diameter: String::new(),
                length: String::new(),
                holder_diameter: String::new(),
                holder_length: String::new(),
                holder: String::new(),
                holder_was: String::new(),
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
            edge: None,
            edge_on: 0,
            edge_z: 0,
            stock_shape: 0,
            stock_size: Default::default(),
            diameter: String::new(),
            side: String::new(),
            top: String::new(),
            bottom: String::new(),
            bodies: String::new(),
            fixtures: String::new(),
            post: 0,
            material: String::new(),
            work_offset: 0,
            stock_from: String::new(),
            clamps: Vec::new(),
            copies: Default::default(),
            x_axis: 0,
        }))),
        "cam.op.face" => Some(op(OpKind::Face)),
        "cam.op.contour" => Some(op(OpKind::Contour)),
        "cam.op.pocket" => Some(op(OpKind::Pocket)),
        "cam.op.adaptive" => Some(op(OpKind::Adaptive)),
        "cam.op.adaptive3d" => Some(op(OpKind::Adaptive3d)),
        "cam.op.parallel3d" => Some(op(OpKind::Parallel3d)),
        "cam.op.zlevel3d" => Some(op(OpKind::ZLevel3d)),
        "cam.op.chamfer" => Some(op(OpKind::Chamfer)),
        "cam.op.vcarve" => Some(op(OpKind::VCarve)),
        "cam.op.thread" => Some(op(OpKind::Thread)),
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
        if let Some(saved) = app.preferences.cam_op_defaults.get(id) {
            recall(f, saved, &lists);
        }
        prefill(f, &lists);
    }
    app.manufacture.form_opened = form.clone();
    app.manufacture.form = form;
    app.manufacture.error = None;
    true
}

/// The command a form stands for. Fields left empty are left to the command's defaults.
pub fn command(form: &Form) -> Result<(&'static str, Value), String> {
    // A decimal point or a decimal comma, whatever the language (#1347).
    let number = |what: &str, s: &str| crate::prefs::parse_number(s).ok_or_else(|| tf("manufacture.must_be_a_number", &[("what", tm(what))]));
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
            let [speed, feed] = own.rates();
            for (k, what, text) in [
                speed,
                feed,
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
                ("corner_radius", "The corner radius", &f.corner_radius),
                ("taper_angle", "The taper angle", &f.taper_angle),
                ("shank_diameter", "The shank diameter", &f.shank_diameter),
                ("length", "The length", &f.length),
                ("holder_diameter", "The holder's diameter", &f.holder_diameter),
                ("holder_length", "The holder's length", &f.holder_length),
            ] {
                let plain = text.trim().trim_end_matches("mm").trim_end_matches("deg").trim_end_matches('°').trim();
                if !plain.is_empty() {
                    p.insert(k.into(), json!(number(what, plain)?));
                } else if f.editing {
                    p.insert(k.into(), Value::Null);
                }
            }
            // The holder by its shape: named only when another was picked, so the tool keeps its copy.
            if f.holder != f.holder_was {
                p.insert("holder".into(), if f.holder.is_empty() { Value::Null } else { json!(f.holder) });
            }
            // The data in particular materials; a material with nothing typed is left out.
            let mut cutting = Vec::new();
            for (material, c) in f.by_material() {
                let mut e = serde_json::Map::new();
                let [speed, feed] = c.rates();
                for (k, what, text) in [
                    speed,
                    feed,
                    ("plunge_feed", "The plunge feed", &c.plunge_feed),
                    ("stepdown", "The stepdown", &c.stepdown),
                    ("stepover", "The stepover", &c.stepover),
                ] {
                    if !text.trim().is_empty() {
                        e.insert(k.into(), json!(number(&tf("manufacture.in_material", &[("what", tm(what)), ("material", &material)]), text)?));
                    }
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
                3 => {
                    let (body, at) = f.edge.as_ref().ok_or("Pick a corner or an edge of the model for the work origin")?;
                    let on = EDGE_POINTS.get(f.edge_on).map_or("vertex", |e| e.0);
                    let z = match ORIGIN_ZS.get(f.edge_z).map_or(ORIGIN_ZS[0].0, |z| z.0) {
                        Some((of, z)) => json!({"of": of, "z": z}),
                        None => json!("point"),
                    };
                    json!({"edge": at, "body": body, "on": on, "z": z})
                }
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
            let shape = STOCK_SHAPES.get(f.stock_shape).map_or("offset", |(k, _)| *k);
            if shape != "offset" || f.editing.is_some() {
                stock.insert("shape".into(), json!(shape));
            }
            match shape {
                "box" => {
                    stock.insert("size".into(), json!(f.stock_size.iter().map(|t| t.trim()).collect::<Vec<_>>()));
                }
                "round" => {
                    stock.insert("diameter".into(), json!(f.diameter.trim()));
                }
                _ => {}
            }
            if !stock.is_empty() {
                p.insert("stock".into(), Value::Object(stock));
            }
            // Every body when none is named.
            let fixtures: Vec<&str> = f.fixtures.split(',').map(str::trim).filter(|b| !b.is_empty()).collect();
            if !fixtures.is_empty() || f.editing.is_some() {
                p.insert("fixtures".into(), if fixtures.is_empty() { Value::Null } else { json!(fixtures) });
            }
            let bodies: Vec<&str> = f.bodies.split(',').map(str::trim).filter(|b| !b.is_empty()).collect();
            if !bodies.is_empty() || f.editing.is_some() {
                p.insert("bodies".into(), if bodies.is_empty() { Value::Null } else { json!(bodies) });
            }
            p.insert("post".into(), json!(solidcraft_engine::cam::Post::BUILTIN.get(f.post).copied().unwrap_or("rs274")));
            p.insert("material".into(), if f.material.trim().is_empty() { Value::Null } else { json!(f.material.trim()) });
            p.insert("work_offset".into(), json!(WORK_OFFSETS.get(f.work_offset).copied().unwrap_or("G54")));
            p.insert("stock_from".into(), f.stock_from.trim().parse::<u64>().map_or(Value::Null, |id| json!(id)));
            let mut clamps = Vec::new();
            for (k, c) in f.clamps.iter().enumerate() {
                if c.iter().all(|t| t.trim().is_empty()) {
                    continue;
                }
                let what = tf("manufacture.clamp_number", &[("number", &(k + 1).to_string())]);
                let v: Vec<f64> = c.iter().map(|t| number(&what, t)).collect::<Result<_, _>>()?;
                clamps.push(json!({ "min": [v[0].min(v[2]), v[1].min(v[3])], "max": [v[0].max(v[2]), v[1].max(v[3])], "top": v[4] }));
            }
            p.insert("clamps".into(), json!(clamps));
            // Copies: counts as whole numbers, an empty count is one.
            let copies = if f.copies.iter().all(|t| t.trim().is_empty()) {
                Value::Null
            } else {
                let count = |t: &str| -> Result<u64, String> {
                    if t.trim().is_empty() {
                        Ok(1)
                    } else {
                        t.trim().parse::<u64>().map_err(|_| tm("The number of copies must be a whole number").to_string())
                    }
                };
                let gap = |t: &str| if t.trim().is_empty() { Ok(0.0) } else { number("The copies' spacing", t) };
                json!({ "nx": count(&f.copies[0])?, "ny": count(&f.copies[1])?, "dx": gap(&f.copies[2])?, "dy": gap(&f.copies[3])? })
            };
            p.insert("copies".into(), copies);
            p.insert("x_axis".into(), X_DIRECTIONS.get(f.x_axis).filter(|d| **d != "automatic").map_or(Value::Null, |d| json!(d)));
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
                    // Facing chooses its boundary; 3D roughing without a sketch roughs the stock.
                    let on_sketch = match f.kind {
                        OpKind::Face => f.boundary == 2,
                        OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d => !f.sketch.trim().is_empty(),
                        _ => true,
                    };
                    let was_on_sketch = match f.kind {
                        OpKind::Face => e.boundary == 2,
                        OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d => !e.sketch.is_empty(),
                        _ => true,
                    };
                    if f.faces.is_empty() && on_sketch && (f.sketch.trim() != e.sketch || !e.faces.is_empty() || !was_on_sketch) {
                        p.insert("sketch".into(), json!(f.sketch.trim()));
                    } else if f.faces.is_empty() && !on_sketch && (f.boundary != e.boundary || !e.faces.is_empty() || was_on_sketch) {
                        // Facing chooses; 3D finishing covers the model's box, 3D roughing the stock.
                        let boundary = match f.kind {
                            OpKind::Face => FACE_BOUNDS.get(f.boundary).copied(),
                            OpKind::Parallel3d | OpKind::ZLevel3d => Some("part"),
                            _ => None,
                        };
                        p.insert("boundary".into(), json!(boundary.unwrap_or("stock")));
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
                OpKind::Adaptive | OpKind::Adaptive3d => {
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
                    if f.kind == OpKind::Adaptive3d {
                        if !f.step_up.trim().is_empty() {
                            p.insert("step_up".into(), json!(f.step_up.trim()));
                        } else if f.editing.is_some() {
                            p.insert("step_up".into(), Value::Null);
                        }
                    }
                    // 3D roughing has no rest machining (yet).
                    match f.rest_from.trim().parse::<u64>().ok().filter(|_| f.kind == OpKind::Adaptive) {
                        Some(n) => {
                            p.insert("rest_from".into(), json!(n));
                        }
                        None if f.editing.is_some() && f.kind == OpKind::Adaptive => {
                            p.insert("rest_from".into(), Value::Null);
                        }
                        None => {}
                    }
                }
                OpKind::ZLevel3d => {
                    p.insert("rest".into(), json!(f.rest3d));
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    if !f.level_step.trim().is_empty() {
                        p.insert("step".into(), json!(f.level_step.trim()));
                    }
                    let steep = f.steep.trim().trim_end_matches("deg").trim_end_matches('°').trim();
                    if !steep.is_empty() {
                        p.insert("steep".into(), json!(number("The steepness", steep)?));
                    } else if f.editing.is_some() {
                        p.insert("steep".into(), json!(30.0));
                    }
                }
                OpKind::Parallel3d => {
                    optional(&mut p, "stepover", &f.stepover);
                    optional(&mut p, "stock_to_leave", &f.stock_to_leave);
                    let angle = f.pass_angle.trim().trim_end_matches("deg").trim_end_matches('°').trim();
                    if !angle.is_empty() {
                        p.insert("angle".into(), json!(number("The pass angle", angle)?));
                    } else if f.editing.is_some() {
                        p.insert("angle".into(), json!(0.0));
                    }
                    p.insert("zigzag".into(), json!(f.zigzag));
                    p.insert("rest".into(), json!(f.rest3d));
                }
                OpKind::Thread => {
                    optional(&mut p, "pitch", &f.pitch);
                    if !f.thread_diameter.trim().is_empty() {
                        p.insert("diameter".into(), json!(f.thread_diameter.trim()));
                    } else if f.editing.is_some() {
                        p.insert("diameter".into(), Value::Null);
                    }
                    p.insert("side".into(), json!(if f.thread_outside { "outside" } else { "inside" }));
                    p.insert("hand".into(), json!(if f.thread_left { "left" } else { "right" }));
                    if let Ok(n) = f.thread_passes.trim().parse::<u64>() {
                        p.insert("thread_passes".into(), json!(n));
                    } else if !f.thread_passes.trim().is_empty() {
                        return Err(tm("The number of thread passes must be a whole number").into());
                    }
                }
                OpKind::VCarve => {
                    optional(&mut p, "stepover", &f.stepover);
                    if !f.max_depth.trim().is_empty() {
                        p.insert("max_depth".into(), json!(f.max_depth.trim()));
                    } else if f.editing.is_some() {
                        p.insert("max_depth".into(), Value::Null);
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
            // Face roles: a new operation's when it has any, an edited one's when they changed.
            let changed = match &f.editing {
                Some(e) => e.face_roles != f.face_roles,
                None => !f.face_roles.is_empty(),
            };
            if changed && f.kind.follows_the_model() {
                p.insert("face_roles".into(), Value::Array(f.face_roles.iter().map(FaceRoleRow::json).collect()));
            }
            // A number of passes instead of the stepdown; on an edit an emptied one goes back to the stepdown.
            if !matches!(f.kind, OpKind::Drill | OpKind::Bore | OpKind::Chamfer | OpKind::VCarve | OpKind::Thread) {
                match f.passes.trim().parse::<u64>() {
                    Ok(n) => {
                        p.insert("passes".into(), json!(n));
                    }
                    Err(_) if !f.passes.trim().is_empty() => return Err(tm("The number of passes must be a whole number").into()),
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
                        let n = f.tab_count.trim().parse::<u64>().map_err(|_| tm("The number of tabs must be a whole number").to_string())?;
                        tabs.insert("count".into(), json!(n));
                    }
                    optional(&mut tabs, "width", &f.tab_width);
                    optional(&mut tabs, "height", &f.tab_height);
                    p.insert("tabs".into(), Value::Object(tabs));
                } else if f.editing.is_some() {
                    p.insert("tabs".into(), Value::Null);
                }
            }
            if f.kind == OpKind::Contour {
                for (k, text) in [("lead", &f.lead), ("skin", &f.skin)] {
                    if !text.trim().is_empty() {
                        p.insert(k.into(), json!(text.trim()));
                    } else if f.editing.is_some() {
                        p.insert(k.into(), Value::Null);
                    }
                }
                p.insert("skin_last".into(), json!(f.skin_last));
            }
            if matches!(f.kind, OpKind::Contour | OpKind::Pocket) {
                p.insert("dogbones".into(), json!(f.dogbones));
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
            if matches!(
                f.kind,
                OpKind::Contour
                    | OpKind::Pocket
                    | OpKind::Bore
                    | OpKind::Chamfer
                    | OpKind::Adaptive
                    | OpKind::Adaptive3d
                    | OpKind::ZLevel3d
                    | OpKind::VCarve
                    | OpKind::Thread
            ) {
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
        "bull_nose" => 4,
        "chamfer_mill" => 5,
        "engraver" => 6,
        _ => 0,
    };
    let mut form = ToolForm {
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
        surface_speed: number("surface_speed"),
        // The tool keeps a surface speed or chip load when it has one and not the speed or feed.
        by_surface_speed: tool["surface_speed"].is_f64(),
        by_chip_load: tool["chip_load"].is_f64() && !tool["feed"].is_f64(),
        tip_angle: number("tip_angle"),
        tip_diameter: number("tip_diameter"),
        corner_radius: number("corner_radius"),
        taper_angle: number("taper_angle"),
        shank_diameter: number("shank_diameter"),
        length: number("length"),
        holder_diameter: number("holder_diameter"),
        holder_length: number("holder_length"),
        holder: text(&tool["holder"], "name"),
        holder_was: text(&tool["holder"], "name"),
        // A tool that exists opens on its shape.
        step: 1,
        material: String::new(),
        others: tool["cutting"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|c| {
                let value = |k: &str| c[k].as_f64().map(|x| x.to_string()).unwrap_or_default();
                // The rows worked out from the ones kept follow when the form is shown.
                let cut = CutText {
                    spindle_rpm: value("spindle_rpm"),
                    feed: value("feed"),
                    plunge_feed: value("plunge_feed"),
                    stepdown: value("stepdown"),
                    stepover: value("stepover"),
                    chip_load: c["chip_load"].as_f64().map(|v| shown(v, 4)).unwrap_or_default(),
                    surface_speed: value("surface_speed"),
                    by_surface_speed: c["surface_speed"].is_f64(),
                    by_chip_load: c["chip_load"].is_f64(),
                };
                Some((c["material"].as_str()?.to_string(), cut))
            })
            .collect(),
    };
    form.work_out_rates();
    Some(Form::Tool(Box::new(form)))
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
            (Some("edge"), _) => 3,
            _ => 2,
        },
        anchors: [anchor("x"), anchor("y"), anchor("z")],
        point: [at.x.to_string(), at.y.to_string(), at.z.to_string()],
        edge: (origin["type"].as_str() == Some("edge")).then(|| (text(origin, "body"), at)),
        edge_on: EDGE_POINTS.iter().position(|e| Some(e.0) == origin["on"].as_str()).unwrap_or(0),
        edge_z: {
            let z = &origin["z"];
            let wanted = (z["type"].as_str() == Some("box")).then(|| (z["of"].as_str().unwrap_or("stock"), z["z"].as_str().unwrap_or("max")));
            ORIGIN_ZS.iter().position(|o| o.0 == wanted).unwrap_or(0)
        },
        stock_shape: STOCK_SHAPES.iter().position(|(k, _)| Some(*k) == setup["stock"]["shape"]["type"].as_str()).unwrap_or(0),
        stock_size: ["x", "y", "z"].map(|k| text(&setup["stock"]["shape"], k)),
        diameter: text(&setup["stock"]["shape"], "diameter"),
        side: offset("side"),
        top: offset("top"),
        bottom: offset("bottom"),
        bodies: setup["bodies"].as_array().into_iter().flatten().filter_map(Value::as_str).collect::<Vec<_>>().join(", "),
        fixtures: setup["fixtures"].as_array().into_iter().flatten().filter_map(Value::as_str).collect::<Vec<_>>().join(", "),
        post: solidcraft_engine::cam::Post::BUILTIN.iter().position(|p| Some(*p) == setup["post"].as_str()).unwrap_or(0),
        material: text(setup, "material"),
        work_offset: setup["work_offset"]
            .as_u64()
            .and_then(|n| usize::try_from(n.saturating_sub(54)).ok())
            .filter(|i| *i < WORK_OFFSETS.len())
            .unwrap_or(0),
        stock_from: setup["stock_from"].as_u64().map(|id| id.to_string()).unwrap_or_default(),
        clamps: setup["clamps"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|c| {
                let n = |v: &Value| v.as_f64().map(|x| x.to_string()).unwrap_or_default();
                [n(&c["min"][0]), n(&c["min"][1]), n(&c["max"][0]), n(&c["max"][1]), n(&c["top"])]
            })
            .collect(),
        copies: {
            let c = &setup["copies"];
            let n = |v: &Value| v.as_f64().map(|x| x.to_string()).unwrap_or_default();
            if c.is_object() { [n(&c["nx"]), n(&c["ny"]), n(&c["dx"]), n(&c["dy"])] } else { Default::default() }
        },
        x_axis: {
            let v: Vec<f64> = setup["x_axis"].as_array().into_iter().flatten().filter_map(Value::as_f64).collect();
            match v.as_slice() {
                [x, _, _] if *x > 0.5 => 1,
                [x, _, _] if *x < -0.5 => 2,
                [_, y, _] if *y > 0.5 => 3,
                [_, y, _] if *y < -0.5 => 4,
                _ => 0,
            }
        },
    })))
}

/// 3D finishing's rest row: only where earlier operations of the setup left material.
fn rest_row(ui: &mut egui::Ui, rest: &mut bool) {
    check_row(ui, "Rest", "Only what earlier operations left", rest);
}

/// A form row with a check box after its label, and what the row is for on hover.
fn check_row(ui: &mut egui::Ui, label: &str, text: &str, on: &mut bool) {
    labelled(ui, label, |ui| {
        let r = crate::ui_kit::checkbox(ui, tm(text), on);
        explained(r, label, None);
    });
}

/// A form filled in from an operation as `cam.list` reports it.
pub fn edit_form(app: &SolveApp, setup: &Value, op: &Value) -> Option<Form> {
    let kind = match op["kind"].as_str()? {
        "face" => OpKind::Face,
        "contour" => OpKind::Contour,
        "pocket" => OpKind::Pocket,
        "adaptive" => OpKind::Adaptive,
        "adaptive_3d" => OpKind::Adaptive3d,
        "parallel_3d" => OpKind::Parallel3d,
        "zlevel_3d" => OpKind::ZLevel3d,
        "chamfer" => OpKind::Chamfer,
        "vcarve" => OpKind::VCarve,
        "thread" => OpKind::Thread,
        "bore" => OpKind::Bore,
        "drill" => OpKind::Drill,
        _ => return None,
    };
    let number = |v: &Value| v.as_f64().map(|x| x.to_string()).unwrap_or_default();
    let flutes = op["tool"].as_u64().and_then(|n| app.session.doc.manufacture.tool(n as u32)).map_or(0, |t| t.flutes);
    let sketch = op["geometry"]["sketch"].as_u64().and_then(|id| app.session.doc.feature(id)).map(|f| f.name.clone()).unwrap_or_default();
    let faces: Vec<(String, Vec3)> =
        op["geometry"]["faces"].as_array().into_iter().flatten().filter_map(|f| Some((f["body"].as_str()?.to_string(), point(&f["at"])?))).collect();
    let face_roles: Vec<FaceRoleRow> = op["face_roles"].as_array().into_iter().flatten().filter_map(FaceRoleRow::of).collect();
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
        editing: Some(Editing {
            setup: setup["id"].as_u64()?,
            op: op["id"].as_u64()?,
            boundary,
            sketch: sketch.clone(),
            faces: faces.clone(),
            face_roles: face_roles.clone(),
        }),
        name: text(op, "name"),
        faces,
        face_roles,
        role_clearance: String::new(),
        role_leave: String::new(),
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
        max_depth: text(inputs, "max_depth"),
        thread_diameter: text(inputs, "diameter"),
        thread_outside: inputs["internal"].as_bool() == Some(false),
        thread_passes: inputs["passes"].as_u64().map(|n| n.to_string()).unwrap_or_default(),
        thread_left: inputs["left_hand"].as_bool().unwrap_or(false),
        lead: text(inputs, "lead"),
        skin: text(inputs, "skin"),
        skin_last: inputs["skin_last"].as_bool().unwrap_or(false),
        dogbones: inputs["dogbones"].as_bool().unwrap_or(false),
        pitch: text(inputs, "pitch"),
        rest_from: inputs["rest_from"].as_u64().map(|n| n.to_string()).unwrap_or_default(),
        stepover_percent: number(&inputs["stepover_percent"]),
        helix_angle: text(inputs, "helix_angle"),
        helix_max: number(&inputs["helix_max"]),
        helix_min: number(&inputs["helix_min"]),
        feed_up: number(&inputs["feed_up"]),
        step_up: text(inputs, "step_up"),
        pass_angle: inputs["angle"].as_f64().filter(|a| *a != 0.0).map(|a| a.to_string()).unwrap_or_default(),
        zigzag: inputs["zigzag"].as_bool().unwrap_or(true),
        level_step: text(inputs, "step"),
        rest3d: inputs["rest"].as_bool().unwrap_or(false),
        steep: inputs["steep"].as_f64().filter(|a| *a != 30.0).map(|a| a.to_string()).unwrap_or_default(),
        width: text(inputs, "width"),
        tip_offset: text(inputs, "tip_offset"),
        peck: text(inputs, "peck"),
        dwell: inputs["dwell"].as_f64().filter(|d| *d > 0.0).map(|d| d.to_string()).unwrap_or_default(),
        coolant: op["coolant"].as_bool().unwrap_or(false),
    })))
}

/// Says in the Tool Library window what an import took and what it left out.
pub fn imported(app: &mut SolveApp, result: Result<Value, String>) {
    let _language = crate::i18n::scope(&app.preferences.language);
    match result {
        Ok(v) => {
            let count = |k: &str| v[k].as_array().map_or(0, Vec::len);
            let tools = (count("added") + count("replaced")).to_string();
            let mut lines = vec![if count("materials") > 0 {
                tf("manufacture.imported_tools_and_materials", &[("tools", &tools), ("materials", &count("materials").to_string())])
            } else {
                tf("manufacture.imported_tools", &[("tools", &tools)])
            }];
            if count("skipped") > 0 {
                let numbers: Vec<String> = v["skipped"].as_array().into_iter().flatten().map(|n| format!("T{n}")).collect();
                lines.push(tf("manufacture.skipped_numbers", &[("numbers", &numbers.join(", "))]));
            }
            lines.extend(v["notes"].as_array().into_iter().flatten().filter_map(Value::as_str).map(|n| tf("manufacture.left_out", &[("what", n)])));
            app.manufacture.error = None;
            app.manufacture.import_notes = lines;
        }
        Err(e) => {
            app.manufacture.error = Some(e);
            app.manufacture.import_notes.clear();
        }
    }
}

/// Gives `p` this window's tool library, if it has one of its own.
fn with_library(library: &Option<String>, p: &mut Value) {
    if let (Some(library), Some(o)) = (library, p.as_object_mut()) {
        o.insert("library".into(), json!(library));
    }
}

/// Runs a command with this window's tool library, if it has one of its own.
fn run(app: &mut SolveApp, id: &str, mut p: Value) -> Result<Value, String> {
    with_library(&app.manufacture.library, &mut p);
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

/// The rows of an operation form a default keeps: how it cuts, not what or how deep.
macro_rules! default_rows {
    ($m:ident) => {
        $m!(text: tool, spindle_rpm, feed, plunge_feed, stepdown, stepover, stock_to_leave, ramp_angle, passes, tab_count, tab_width,
            tab_height, finish, pitch, stepover_percent, helix_angle, helix_max, helix_min, feed_up, width, tip_offset, peck, dwell, margin,
            lead, skin);
        $m!(flag: climb, tabs, coolant, dogbones, skin_last);
        $m!(index: side, boundary, limit);
    };
}

/// An operation form's rows a default keeps, by name.
fn remembered(f: &OpForm) -> std::collections::BTreeMap<String, String> {
    let mut out = std::collections::BTreeMap::new();
    macro_rules! keep {
        (text: $($n:ident),*) => { $(out.insert(stringify!($n).to_string(), f.$n.clone());)* };
        (flag: $($n:ident),*) => { $(out.insert(stringify!($n).to_string(), f.$n.to_string());)* };
        (index: $($n:ident),*) => { $(out.insert(stringify!($n).to_string(), f.$n.to_string());)* };
    }
    default_rows!(keep);
    out
}

/// A new operation form filled from a kept default: a tool the library no longer has, and its
/// speeds, are left to the form's own.
fn recall(f: &mut OpForm, saved: &std::collections::BTreeMap<String, String>, lists: &Choices) {
    let get = |k: &str| saved.get(k).filter(|v| v.len() <= MAX_NAME_TEXT);
    let tool_known = get("tool").is_some_and(|t| lists.tools.iter().any(|(n, _)| n == t));
    let speeds = ["tool", "spindle_rpm", "feed", "plunge_feed"];
    macro_rules! take {
        (text: $($n:ident),*) => { $(if let Some(v) = get(stringify!($n)).filter(|_| tool_known || !speeds.contains(&stringify!($n))) { f.$n = v.clone(); })* };
        (flag: $($n:ident),*) => { $(if let Some(v) = get(stringify!($n)).and_then(|v| v.parse::<bool>().ok()) { f.$n = v; })* };
        (index: $($n:ident),*) => { $(if let Some(v) = get(stringify!($n)).and_then(|v| v.parse::<usize>().ok()).filter(|v| *v < 8) { f.$n = v; })* };
    }
    default_rows!(take);
    // The kept speeds stand: the tool's own do not fill them over.
    if tool_known {
        f.filled_for = f.tool.clone();
    }
}

/// Longest text a kept default row may have.
const MAX_NAME_TEXT: usize = 200;

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
    } else if f.kind == OpKind::Adaptive3d && f.sketch.trim().is_empty() {
        p.insert("boundary".into(), json!("stock"));
    } else if matches!(f.kind, OpKind::Parallel3d | OpKind::ZLevel3d) && f.sketch.trim().is_empty() {
        p.insert("boundary".into(), json!("part"));
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

/// The selected corner or edge a setup form can put the work origin on, worked out again when
/// the design or the selection changes.
fn origin_pick(app: &mut SolveApp) -> Option<(String, Vec3, solidcraft_engine::doc::manufacture::EdgePoint)> {
    let Some(Form::Setup(f)) = &app.manufacture.form else { return None };
    if f.origin != 3 || app.session.selection.is_empty() {
        return None;
    }
    let key = json!(app.session.selection);
    let revision = app.session.revision;
    if let Some((r, k, pick)) = &app.manufacture.origin_pick
        && *r == revision
        && *k == key
    {
        return pick.clone();
    }
    let pick = solidcraft_engine::cam_origin_pick(&app.session, &app.session.selection);
    app.manufacture.origin_pick = Some((revision, key, pick.clone()));
    pick
}

/// Works out the open operation form's toolpath again when its command or the design changed;
/// none while the form still misses something.
fn update_preview(app: &mut SolveApp, lists: &Choices) {
    let wanted = match &app.manufacture.form {
        Some(form @ Form::Op(_)) if missing(form, lists).is_empty() => command(form).ok().map(|(id, mut p)| {
            with_library(&app.manufacture.library, &mut p);
            (id, json!({ "command": id, "params": p }))
        }),
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
    // 3D roughing takes seconds: worked out off the window thread, which shows none meanwhile.
    #[cfg(not(target_arch = "wasm32"))]
    if !cfg!(test) && solidcraft_engine::cam_slow(&app.session, id, &key["params"]) {
        if let Some((r, k, rx)) = &app.manufacture.preview_job
            && *r == revision
            && *k == key
        {
            match rx.try_recv() {
                Ok(path) => {
                    app.manufacture.preview = Some((revision, key, path));
                    app.manufacture.preview_job = None;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => app.manufacture.preview = None,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => app.manufacture.preview_job = None,
            }
            return;
        }
        let (tx, rx) = std::sync::mpsc::channel();
        let (copy, params, cmd, repaint) = (app.session.scratch(), key["params"].clone(), id.to_string(), app.manufacture.repaint.clone());
        let spawned = std::thread::Builder::new().name("cam-preview".into()).spawn(move || {
            let _ = tx.send(solidcraft_engine::cam_preview(&copy, &cmd, &params).map_err(|e| e.to_string()));
            if let Some(ctx) = repaint {
                ctx.request_repaint();
            }
        });
        if spawned.is_ok() {
            app.manufacture.preview_job = Some((revision, key, rx));
            app.manufacture.preview = None;
            return;
        }
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
    let _language = crate::i18n::scope(&app.preferences.language);
    match app.session.doc.manufacture.setups.first().map(|s| s.id) {
        Some(setup) => simulate_setup(app, setup),
        None => app.manufacture.error = Some(tm("There is no setup to simulate").into()),
    }
}

/// What a simulation covers: the setup's operations it cuts and shows (`None`: every one),
/// those it cuts first for the stock they leave, and an open form's operation as it stands
/// (its command, run on a copy of the design: the form is not applied); `title` says which.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SimScope {
    pub ops: Option<Vec<u64>>,
    pub first: Vec<u64>,
    pub draft: Option<(&'static str, Value)>,
    pub title: String,
}

impl SimScope {
    /// `cam.simulate`'s and `cam_playback`'s parameters for it.
    fn params(&self, setup: u64, columns: u32) -> Value {
        let mut p = json!({ "setup": setup, "columns": columns, "first": self.first });
        if let Some(ops) = &self.ops {
            p["ops"] = json!(ops);
        }
        p
    }

    /// Runs `f` on the design, or on a copy with the form's operation applied.
    fn run<T>(
        &self,
        session: &mut solidcraft_engine::Session,
        f: impl FnOnce(&mut solidcraft_engine::Session) -> Result<T, String>,
    ) -> Result<T, String> {
        match &self.draft {
            Some((cmd, p)) => {
                let mut copy = session.scratch();
                copy.execute(cmd, p).map_err(|e| e.to_string())?;
                f(&mut copy)
            }
            None => f(session),
        }
    }
}

/// Simulates a setup: the operations shown in the browser (a hidden one is left out), or every one.
pub fn simulate_setup(app: &mut SolveApp, setup: u64) {
    let _language = crate::i18n::scope(&app.preferences.language);
    let Some(s) = app.session.doc.manufacture.setups.iter().find(|s| s.id == setup) else { return };
    let ids: Vec<u64> = s.operations.iter().map(|o| o.id).collect();
    let shown: Vec<u64> = ids.iter().copied().filter(|id| !app.manufacture.hidden.contains(id)).collect();
    let scope = if shown.is_empty() || shown.len() == ids.len() {
        SimScope { title: tf("manufacture.simulated_setup", &[("setup", &s.name)]), ..Default::default() }
    } else {
        let (count, total) = (shown.len().to_string(), ids.len().to_string());
        SimScope {
            title: tf("manufacture.simulated_some", &[("setup", &s.name), ("count", &count), ("total", &total)]),
            ops: Some(shown),
            ..Default::default()
        }
    };
    simulate_scope(app, setup, scope);
}

/// Simulates one operation on the stock the operations before it leave.
pub fn simulate_op(app: &mut SolveApp, setup: u64, op: u64) {
    let _language = crate::i18n::scope(&app.preferences.language);
    let Some(s) = app.session.doc.manufacture.setups.iter().find(|s| s.id == setup) else { return };
    let Some(at) = s.operations.iter().position(|o| o.id == op) else { return };
    let first: Vec<u64> = s.operations.iter().take(at).map(|o| o.id).collect();
    let name = s.operations.get(at).map(|o| o.name.clone()).unwrap_or_default();
    let title = if first.is_empty() { "manufacture.simulated_op" } else { "manufacture.simulated_op_after" };
    let scope = SimScope { ops: Some(vec![op]), first, draft: None, title: tf(title, &[("op", &name)]) };
    simulate_scope(app, setup, scope);
}

/// Simulates the open operation form as it stands, on a copy of the design (the form stays
/// open, not applied), on the stock the operations before it leave.
pub fn simulate_form(app: &mut SolveApp) {
    let _language = crate::i18n::scope(&app.preferences.language);
    let Some(form) = app.manufacture.form.clone() else { return };
    let Form::Op(f) = &form else { return };
    let made = command(&form).and_then(|(cmd, p)| {
        let mut copy = app.session.scratch();
        let made = copy.execute(cmd, &p).map_err(|e| e.to_string())?;
        let (setup, op) = match &f.editing {
            Some(e) => (e.setup, e.op),
            None => (made["setup"].as_u64().unwrap_or(0), made["operation"].as_u64().unwrap_or(0)),
        };
        let s = copy.doc.manufacture.setups.iter().find(|s| s.id == setup).ok_or_else(|| tm("There is no setup to simulate").to_string())?;
        let at = s.operations.iter().position(|o| o.id == op).ok_or_else(|| tm("There is no setup to simulate").to_string())?;
        let first: Vec<u64> = s.operations.iter().take(at).map(|o| o.id).collect();
        let name = s.operations.get(at).map(|o| o.name.clone()).unwrap_or_default();
        Ok((setup, SimScope { ops: Some(vec![op]), first, draft: Some((cmd, p)), title: tf("manufacture.simulated_draft", &[("op", &name)]) }))
    });
    match made {
        Ok((setup, scope)) => simulate_scope(app, setup, scope),
        Err(e) => app.manufacture.error = Some(e),
    }
}

/// Runs `cam.simulate` for what `scope` covers and keeps what it found for the window.
fn simulate_scope(app: &mut SolveApp, setup: u64, scope: SimScope) {
    app.manufacture.play = None;
    app.manufacture.stock_stamp += 1;
    app.manufacture.simulated = Some(setup);
    let columns = SIM_DETAIL.get(app.manufacture.sim_detail).map_or(300, |d| d.1);
    let mut p = scope.params(setup, columns);
    p["field"] = json!(true);
    let result = scope.run(&mut app.session, |s| s.execute("cam.simulate", &p).map_err(|e| e.to_string()));
    let title = scope.title.clone();
    app.manufacture.scope = scope;
    match result {
        Ok(v) => {
            let number = |k: &str| v[k].as_f64();
            let removed = format!("{:.2}", number("removed_volume").unwrap_or(0.0) / 1000.0);
            let mut found = vec![title, tf("manufacture.removed", &[("volume", &removed)])];
            if let (Some(gouge), Some(left), Some(volume)) = (number("max_gouge"), number("max_leftover"), number("leftover_volume")) {
                found.push(if gouge > 1e-3 {
                    tf("manufacture.cuts_into_part", &[("depth", &format!("{gouge:.3}"))])
                } else {
                    tm("Does not cut into the part").into()
                });
                let (left, volume) = (format!("{left:.2}"), format!("{:.2}", volume / 1000.0));
                found.push(tf("manufacture.leaves_on_part", &[("thickness", &left), ("volume", &volume)]));
            }
            found.extend(v["notes"].as_array().into_iter().flatten().filter_map(Value::as_str).map(|n| tm(n).to_string()));
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
    // What the simulation covered is what plays.
    let scope = app.manufacture.scope.clone();
    let mut p = scope.params(setup, columns);
    p["rapid_rate"] = json!(app.manufacture.rapid_rate);
    match scope.run(&mut app.session, |s| solidcraft_engine::cam_playback(s, &p).map_err(|e| e.to_string())) {
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
    ui.add_space(t.space_4);
    let Some(play) = cam.play.as_mut() else {
        if button(ui, tm("Play the program"), Button::Normal)
            .on_hover_text(tm("Show the tool moving along the toolpaths and cutting the stock"))
            .clicked()
        {
            asked.play = Some(setup);
        }
        return;
    };
    let total = play.data.playback.minutes;
    let steps = &play.data.playback.steps;
    let k = play.data.playback.step_at(play.t).unwrap_or(0);
    let mut stop = false;
    ui.horizontal_wrapped(|ui| {
        if button(ui, "|<", Button::Small).on_hover_text(tm("To the start")).clicked() {
            play.seek = Some(0.0);
        }
        if button(ui, "<", Button::Small).on_hover_text(tm("Back one move")).clicked() {
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
        if button(ui, tm(label), Button::Small).clicked() {
            if play.t >= total {
                (play.seek, play.playing) = (Some(0.0), true);
            } else {
                play.playing = !play.playing;
            }
        }
        if button(ui, ">", Button::Small).on_hover_text(tm("On one move")).clicked() {
            let on = steps.get(k).map_or(total, |s| s.start + s.minutes);
            (play.seek, play.playing) = (Some(on), false);
        }
        // Where the cut it is in ends: the next rapid up.
        if button(ui, ">R", Button::Small).on_hover_text(tm("On to the next retract: where the cut being played ends")).clicked() {
            let up = steps.iter().skip(k + 1).find(|s| s.feed.is_none() && s.to.z > s.from.z + 1e-9).map_or(total, |s| s.start);
            (play.seek, play.playing) = (Some(up), false);
        }
        if button(ui, ">|", Button::Small).on_hover_text(tm("To the end")).clicked() {
            (play.seek, play.playing) = (Some(total), false);
        }
        let speeds = PLAY_SPEEDS.map(|s| s.0);
        small_choice(ui, "sc_cam_play_speed", "Speed", "Playback speed", &mut play.speed, &speeds);
        if button(ui, tm("Stop"), Button::Small).on_hover_text(tm("Stop playing and show the simulation's result")).clicked() {
            stop = true;
        }
    });
    // The machine's own rapid rate: the times follow it, the tool stays where it is.
    ui.horizontal(|ui| {
        explained(ui.label(tm("Rapids")), "Rapid rate", None);
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
            _ if step.dwell => tf("manufacture.dwell_seconds", &[("seconds", &shown(step.minutes * 60.0, 1))]),
            Some(f) => tf("manufacture.feed_rate", &[("feed", &shown(f, 0))]),
            None => tm("rapid").to_string(),
        };
        let mm = |v: f64| shown((v * 1e3).round() / 1e3 + 0.0, 3);
        ui.label(RichText::new(format!("{op}  T{tool}   X {}  Y {}  Z {}   {feed}", mm(p.x), mm(p.y), mm(p.z))).color(t.text));
        ui.label(RichText::new(tf("manufacture.time_of_total", &[("time", &clock(play.t)), ("total", &clock(total))])).color(t.text_dim));
    }
    if play.collisions > 0 {
        ui.label(RichText::new(tf("manufacture.rapids_into_material", &[("count", &play.collisions.to_string())])).color(t.error));
    }
    if stop && let Some(play) = cam.play.take() {
        cam.stock = play.finished;
        cam.stock_stamp += 1;
    }
}

/// `cam.list` for the design as it is, and whether it is (`false`: the last one read, while 3D
/// roughing toolpaths it needs are worked out off the window thread, which then asks for a
/// frame). Without 3D roughing, or once its toolpaths are known, it is read at once.
fn listed(app: &mut SolveApp) -> (Value, bool) {
    #[cfg(not(target_arch = "wasm32"))]
    if !cfg!(test) {
        let revision = app.session.revision;
        let cam = &mut app.manufacture;
        let done_here = match &cam.listing_job {
            Some((r, rx)) => match rx.try_recv() {
                Err(std::sync::mpsc::TryRecvError::Empty) if *r == revision => return (cam.last_listing.clone(), false),
                // Worked out for this revision: what it could not cache (a failing operation) is quick now.
                _ => *r == revision,
            },
            None => false,
        };
        if !done_here && !solidcraft_engine::cam_list_ready(&app.session) {
            let (tx, rx) = std::sync::mpsc::channel();
            let mut copy = app.session.scratch();
            let repaint = app.manufacture.repaint.clone();
            let spawned = std::thread::Builder::new().name("cam-list".into()).spawn(move || {
                let _ = copy.execute("cam.list", &json!({}));
                let _ = tx.send(());
                if let Some(ctx) = repaint {
                    ctx.request_repaint();
                }
            });
            if spawned.is_ok() {
                app.manufacture.listing_job = Some((revision, rx));
                return (app.manufacture.last_listing.clone(), false);
            }
        }
        app.manufacture.listing_job = None;
    }
    let listed = app.session.execute("cam.list", &json!({})).unwrap_or_default();
    app.manufacture.last_listing = listed.clone();
    (listed, true)
}

/// Works the toolpaths and the setups' frames out again when the design has changed.
fn refresh(app: &mut SolveApp) {
    let revision = app.session.revision;
    if app.manufacture.cache.as_ref().is_some_and(|(r, ..)| *r == revision) {
        return;
    }
    // While 3D roughing is worked out, the toolpaths drawn stay as they were.
    let (listed, fresh) = listed(app);
    if !fresh {
        return;
    }
    let (mut all, mut views, mut ids) = (Vec::new(), Vec::new(), Vec::new());
    for setup in listed["setups"].as_array().into_iter().flatten() {
        let f = &setup["frame"];
        if let (Some(o), Some(x), Some(y), Some(z)) = (point(&f["origin"]), point(&f["x"]), point(&f["y"]), point(&f["z"])) {
            let b = &setup["stock_box"];
            let stock = point(&b["min"]).zip(point(&b["max"]));
            let floor = stock.map_or(0.0, |(lo, _)| lo.z);
            let clamps = setup["clamps"]
                .as_array()
                .into_iter()
                .flatten()
                .filter_map(|c| {
                    let at = |k: &str, i: usize| c[k][i].as_f64();
                    let top = c["top"].as_f64()?;
                    Some((Vec3::new(at("min", 0)?, at("min", 1)?, floor.min(top)), Vec3::new(at("max", 0)?, at("max", 1)?, top)))
                })
                .collect();
            let round = setup["stock_round"]["centre"]
                .as_array()
                .and_then(|c| Some(Vec3::new(c.first()?.as_f64()?, c.get(1)?.as_f64()?, 0.0)))
                .zip(setup["stock_round"]["radius"].as_f64());
            views.push(SetupView { frame: [o, x, y, z], stock, round, clamps });
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
    let mut edges = Vec::new();
    let mut size = 20.0;
    if let Some((lo, hi)) = view.stock {
        edges = match view.round {
            Some((c, r)) => round_edges(view, c, r, lo.z, hi.z),
            None => box_edges(view, lo, hi),
        };
        size = (hi - lo).len() * 0.15;
    }
    (edges, [(o, o + x * size), (o, o + y * size), (o, o + z * size)])
}

/// The twelve edges, in the world, of a box from `lo` to `hi` in a setup's work coordinates.
fn box_edges(view: &SetupView, lo: Vec3, hi: Vec3) -> Vec<(Vec3, Vec3)> {
    let [o, x, y, z] = view.frame;
    let world = |q: Vec3| o + x * q.x + y * q.y + z * q.z;
    let corner =
        |i: usize| world(Vec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z }));
    let mut edges = Vec::with_capacity(12);
    for i in 0..8usize {
        for bit in [1usize, 2, 4] {
            if i & bit == 0 {
                edges.push((corner(i), corner(i | bit)));
            }
        }
    }
    edges
}

/// The edges, in the world, of a round bar about `c` (`r` across) from `bottom` to `top` in a
/// setup's work coordinates: its two end circles and four lines along it.
fn round_edges(view: &SetupView, c: Vec3, r: f64, bottom: f64, top: f64) -> Vec<(Vec3, Vec3)> {
    const SIDES: usize = 64;
    let [o, x, y, z] = view.frame;
    let world = |a: f64, h: f64| o + x * (c.x + r * a.cos()) + y * (c.y + r * a.sin()) + z * h;
    let angle = |k: usize| k as f64 * std::f64::consts::TAU / SIDES as f64;
    let mut edges = Vec::with_capacity(2 * SIDES + 4);
    for k in 0..SIDES {
        for h in [bottom, top] {
            edges.push((world(angle(k), h), world(angle(k + 1), h)));
        }
        if k % (SIDES / 4) == 0 {
            edges.push((world(angle(k), bottom), world(angle(k), top)));
        }
    }
    edges
}

/// A setup's clamps' edges in the world.
pub fn clamp_lines(view: &SetupView) -> Vec<(Vec3, Vec3)> {
    view.clamps.iter().flat_map(|(lo, hi)| box_edges(view, *lo, *hi)).collect()
}

/// The screen segments of the toolpaths: `(from, to, cutting)`.
/// Whether the toolpaths are drawn: while the Manufacture panel is open, or kept on by the browser's eye.
pub fn paths_drawn(app: &SolveApp) -> bool {
    paths_drawn_by(&app.manufacture)
}

/// [`paths_drawn`], from the Manufacture state alone.
fn paths_drawn_by(cam: &CamUi) -> bool {
    cam.show_paths && (cam.open || cam.pinned_paths)
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

/// Most triangles tinted to show faces with roles.
const MAX_TINTED: usize = 200_000;

/// The faces an open 3D operation form gives roles, tinted over the model by role: ignored
/// dimmed, avoided in the warning colour, those left material on in the accent colour.
fn draw_face_roles(app: &SolveApp, painter: &egui::Painter, proj: &Proj, roles: &[FaceRoleRow]) {
    use solidcraft_engine::doc::naming;
    if roles.is_empty() {
        return;
    }
    let t = Tokens::get();
    let state = app.session.world_state();
    let mut mesh = egui::Mesh::default();
    for r in roles {
        let colour = match r.role {
            FaceRoleKind::Ignore => t.text_dim,
            FaceRoleKind::Avoid => t.warning,
            FaceRoleKind::Leave => t.accent,
        }
        .gamma_multiply(0.45);
        let Some(body) = state.bodies.iter().find(|b| b.name == r.body) else { continue };
        let Some((index, _, _)) = naming::face_name_at_body(body, r.at).and_then(|name| naming::face_on_body(body, &name, r.at)) else { continue };
        let m = body.mesh();
        for (k, tri) in m.triangles.iter().enumerate() {
            if m.tri_face.get(k).map(|f| *f as usize) != Some(index) {
                continue;
            }
            if mesh.vertices.len() >= MAX_TINTED * 3 {
                break;
            }
            let corners: Option<Vec<egui::Pos2>> = tri.iter().map(|v| m.positions.get(*v as usize).and_then(|p| proj.to_screen(*p))).collect();
            let Some(corners) = corners else { continue };
            let first = mesh.vertices.len() as u32;
            for p in corners {
                mesh.colored_vertex(p, colour);
            }
            mesh.add_triangle(first, first + 1, first + 2);
        }
    }
    painter.add(egui::Shape::mesh(mesh));
}

/// Draws the toolpaths over the model: cuts in the accent colour, rapids thin and dim.
pub fn draw(app: &SolveApp, painter: &egui::Painter, proj: &Proj) {
    let t = Tokens::get();
    if let Some(Form::Op(f)) = app.manufacture.form.as_ref().filter(|_| app.manufacture.open) {
        draw_face_roles(app, painter, proj, &f.face_roles);
    }
    if app.manufacture.open && app.manufacture.show_setups {
        for view in app.manufacture.cache.iter().flat_map(|(_, _, v)| v) {
            let (edges, arms) = setup_lines(view);
            for (a, b) in edges {
                if let (Some(a), Some(b)) = (proj.to_screen(a), proj.to_screen(b)) {
                    painter.line_segment([a, b], Stroke::new(1.0, t.text_dim));
                }
            }
            for (a, b) in clamp_lines(view) {
                if let (Some(a), Some(b)) = (proj.to_screen(a), proj.to_screen(b)) {
                    painter.line_segment([a, b], Stroke::new(1.5, t.warning));
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
    let t = Tokens::get();
    let [o, x, y, z] = frame;
    let flute = tool.flute_length;
    let length = tool.length.unwrap_or(flute + 2.0 * tool.diameter).max(flute);
    let shank = tool.shank_diameter.unwrap_or(tool.diameter) * 0.5;
    // Rings as (height over the tip, radius).
    // As it is shaped: its end, cone or round, a tapered side widening up.
    let cutting: Vec<(f64, f64)> = tool_side(tool).into_iter().step_by(4).chain([(0.0, 0.0)]).map(|(d, h)| (h, d)).collect();
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
        "Face roles" => {
            "What particular faces of the part are to this operation: ignored, kept clear of, or left material on. Every other face is machined. Select faces in the model, then press one of the buttons; the faces are tinted while the form is open."
        }
        "Holder" => "What the tool sits in: one of the library's holders by its shape, else a plain one by its diameter and length.",
        "Kind" => "Flat: a flat end mill. Ball: a rounded end. Drill: only plunges straight down.",
        "Diameter" => "How wide the tool cuts.",
        "Description" => "Your own words for the tool, shown in the lists.",
        "Length" => "How far the tool reaches out of its holder: shank and cutting part together.",
        "Shank diameter" => "How thick the part the holder grips is. Empty: as thick as the tool cuts.",
        "Point angle" => "The angle between the point's two flanks: a V-bit's (often 60° or 90°) or a drill's (118°).",
        "Tip diameter" => "A V-bit's small flat at its point. Empty: a sharp point.",
        "Taper angle" => {
            "How much a tapered end mill widens up its flutes, each side, in degrees (as catalogues give it). The diameter is at its end. Empty or 0: straight."
        }
        "Corner radius" => "How far a bull nose's flat end rounds into its side: at most half the diameter (all the way round is a ball).",
        "Flute length" => "How deep the cutting edges reach. Deeper cuts need a longer tool.",
        "Flutes" => "How many cutting edges the tool has. Together with the speed it sets the feed.",
        "Spindle" => "How fast the spindle turns. Empty: the tool's own speed.",
        "Chip load" => "How thick a chip each edge takes. Feed = speed × flutes × chip load: change one and the feed follows.",
        "Feed" => "How fast the tool moves sideways while cutting. Empty: the tool's own feed.",
        "Plunge feed" => "How fast the tool moves down into the material.",
        "Stepdown" => "How deep the tool goes per pass. Deep cuts are made in several passes.",
        "Passes" => "How many depth passes, instead of a stepdown: the depth is divided evenly. Empty: by the stepdown.",
        "Pitch" => "How far the thread rises in one turn: 1.25 mm for M8, 1.5 mm for M10, say.",
        "Thread diameter" => {
            "The thread's major (nominal) diameter: 8 mm for M8. Empty: a hole drawn at its core diameter plus its depth, a boss as drawn."
        }
        "Thread side" => "Inside a hole (a nut's thread) or round a boss (a bolt's).",
        "Hand" => {
            "Which way the thread winds: right-hand (the usual) or left-hand. A right-hand thread is cut from the bottom up, a left-hand one from the top down."
        }
        "Thread passes" => "How many passes out to the full thread depth: more are gentler on a small thread mill. Empty: one.",
        "Deepest" => {
            "The deepest the V-bit's point goes below the top: wider parts of the shape are carved flat at this depth. Empty: as deep as the shape asks."
        }
        "Onion skin" => "Material left on the bottom of a cut-out, so that the part stays held by a thin floor. Empty: none.",
        "Skin" => "Take the onion skin off in one light last pass of this operation; otherwise it is left for another operation, or by hand.",
        "Lead-in radius" => {
            "Each pass goes down beside the wall, off the part, and arcs onto the wall and off it again: no mark where it starts. Empty: it ramps down along the loop."
        }
        "Dogbones" => "The round tool leaves a radius in sharp inside corners; a dogbone runs in to the corner, so that square parts fit into it.",
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
        "Stock on the sides" | "Stock on top" | "Stock below" => {
            "Extra material round the part: the raw block is the part's box plus these. A set box and a round bar sit this far over the part's top (a round bar also this far below its foot)."
        }
        "Stock" => {
            "The raw block: the part's box with extra material round it, a box of a set size (a sawn plate), or a round bar standing along the tool axis. Both of the last are centred on the part across."
        }
        "Stock size X" | "Stock size Y" | "Stock size Z" => "How big the raw block is along the setup's X, Y and Z.",
        "Bar diameter" => "How thick the round bar is. It stands along the tool axis, centred on the part.",
        "Fixtures" => {
            "Which bodies hold the stock (a vise, its jaws, a plate), by name, apart by commas. They are never the part; operations are warned when a tool or its holder comes into one, and 3D operations keep 1 mm clear of them. Empty: none."
        }
        "Part bodies" => {
            "Which bodies are the part, by name, apart by commas. Others (a fixture, a modelled stock) are neither cut round nor checked against. Empty: every body."
        }
        "Origin on" => "What the work zero sits on: the raw block, the part, or a point you give.",
        "Origin point" => {
            "The corner or edge of the model the work zero is on. Click it in the view, then take it here. It moves with the model when the design changes."
        }
        "Origin at" => {
            "Which point of the picked edge: its corner nearest where you clicked, its middle, or the centre of its circle (a hole, a round boss)."
        }
        "Origin X" | "Origin Y" | "Origin Z" => "Where on it the work zero sits, the point the machine is zeroed on.",
        "Post" => "Which machine dialect the G-code is written in.",
        "X direction" => {
            "Which way the setup's X runs, across the tool axis. Automatic: worked out from the axis. From below (-z), +x means the part is turned over about X, -x about Y."
        }
        "Holder diameter" => {
            "How wide the holder (collet nut, chuck) the tool sits in is. With its length, and the tool's length out of it, operations check that the holder stays clear of the clamps. Empty: not checked."
        }
        "Holder length" => "How long the holder is, from where the tool comes out of it up. Empty: not checked.",
        "Copies" => {
            "Copies of the part side by side on the stock, in rows along X and columns along Y: each operation cuts every copy before the next operation. Empty: one part."
        }
        "Spacing" => "How far apart the copies are along X and Y, from one part's place to the next's (mm).",
        "Clamps" => {
            "What holds the stock and no tool may touch: a box from-to across, in the setup's work coordinates, up to its top. Operations whose tool comes into one are warned about."
        }
        "Stock from" => {
            "Where this setup's stock comes from: its own box round the part, or what an earlier setup leaves of it (the part turned over for its other side, say: the earlier setup's tool axis turned over)."
        }
        "Work offset" => "Which of the machine's work offsets the program runs in: the one you zeroed on this stock. G54 is the usual one.",
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
        "Leftover shown from" => {
            "From how thick material left on the part shows in the simulated stock (the warning colour). Finer than a finishing pass's cusps, it is drawn as the part. Cuts into the part show however small."
        }
        "Rest" => {
            "Only where the setup's earlier operations left material on the model: with a smaller tool after a larger one, it cleans the corners and steps the larger could not reach, and goes nowhere else."
        }
        "Level step" => "How far apart the levels are that the tool goes round the walls at: smaller is cleaner but slower.",
        "Walls from" => {
            "How steep, in degrees, a wall has to be to be finished level by level; flatter areas are left to 3D Parallel. 0: wherever the tool touches."
        }
        "Pass angle" => "Which way the finishing passes run, in degrees from X: across the slopes that matter most, they finish them best.",
        "Zigzag" => "Back and forth, the tool going on to the next pass along the part; otherwise every pass the same way, the tool up between them.",
        "Step-up height" => {
            "On slopes and rounds a level leaves steps as high as the stepdown. Levels in between, at most this far apart, take them where they stand. Empty: a quarter of the stepdown; 0: none."
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

/// A form row as the app's dialogs have it (`ui_kit::row`): `label` in the form's label
/// column and what `add` draws in its control column.
fn labelled<R>(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    crate::ui_kit::row(ui, tm(label), add)
}

/// The form's label column and the gap after it, as [`form_body`] chose it for this frame.
const LABEL_COLUMN: &str = "sc_cam_label_column";

/// A line in a form's control column under the row before it, with no label of its own (a
/// button that fills that row in).
fn beneath<R>(ui: &mut egui::Ui, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let indent = ui.ctx().data(|d| d.get_temp::<f32>(egui::Id::new(LABEL_COLUMN))).unwrap_or(0.0);
    ui.horizontal(|ui| {
        ui.add_space((indent - ui.spacing().item_spacing.x).max(0.0));
        add(ui)
    })
    .inner
}

/// `r` with what its row is for, and what it needs, on hover.
fn explained(r: egui::Response, label: &str, need: Option<&Need>) -> egui::Response {
    let mut tip = help(label).map(tm).unwrap_or_default().to_string();
    if let Some(n) = need {
        if !tip.is_empty() {
            tip.push_str("\n\n");
        }
        tip.push_str(&tf("manufacture.needed", &[("why", tm(&n.why))]));
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

/// A frame in the error colour inside a row's control while its value is still needed, as a
/// text box the kit draws invalid has.
fn mark(ui: &egui::Ui, r: &egui::Response, need: Option<&Need>) {
    if need.is_some() {
        let t = Tokens::get();
        ui.painter().rect_stroke(r.rect, t.control_radius, Stroke::new(1.0, t.form_error), egui::StrokeKind::Inside);
    }
}

/// A form's text box, as wide as its row leaves; `invalid` draws it in the error colour (its value is still needed).
fn text_box(ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str, invalid: bool) -> egui::Response {
    crate::ui_kit::cell_editor(ui, ui.make_persistent_id(("cam_field", label)), value, tm(hint), invalid)
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
fn height_field(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str, picked: Option<f64>) -> (bool, f32) {
    let need = needed(need, label);
    let middle = ui.cursor().top() + crate::ui_kit::row_height() / 2.0;
    let mut lit = ui.rect_contains_pointer(egui::Rect::from_min_size(ui.cursor().min, egui::vec2(ui.available_width(), crate::ui_kit::row_height())));
    labelled(ui, label, |ui| {
        let side = crate::ui_kit::row_height();
        let r = ui
            .scope(|ui| {
                ui.set_max_width((ui.available_width() - side - ui.spacing().item_spacing.x).max(0.0));
                text_box(ui, label, value, hint, need.is_some())
            })
            .inner;
        lit |= r.hovered() || r.has_focus();
        let r = explained(r, label, need);
        crate::params_dialog::complete(ui, &r, value);
        let mm = |z: f64| format!("{} mm", shown((z * 1e3).round() / 1e3 + 0.0, 3));
        let name = tf("manufacture.from_the_selection", &[("label", tm(label))]);
        let b = ui
            .add_enabled_ui(picked.is_some(), |ui| {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(side, side), egui::Sense::hover());
                crate::ui_kit::icon_button_at(ui, rect, "select", &name)
            })
            .inner;
        let tip = match picked {
            Some(z) => tf("manufacture.take_the_height", &[("height", &mm(z))]),
            None => String::new(),
        };
        let b = b.on_hover_text(tip).on_disabled_hover_text(tm(
            "Click an edge, a point or a face of the model first (the bottom circle of a hole, say); this takes its height.",
        ));
        lit |= b.hovered();
        if let (true, Some(z)) = (b.clicked(), picked) {
            *value = mm(z);
        }
    });
    (lit, middle)
}

/// [`field`], also saying whether its row is pointed at or typed in.
fn field_seen(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str) -> (bool, bool) {
    field_unless(ui, need, label, value, hint, None)
}

/// [`field_seen`], greyed out with `unused` saying why while it is `Some`.
fn field_unless(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str, unused: Option<&str>) -> (bool, bool) {
    let need = needed(need, label);
    labelled(ui, label, |ui| {
        let r = ui.add_enabled_ui(unused.is_none(), |ui| text_box(ui, label, value, hint, need.is_some())).inner;
        let r = r.on_disabled_hover_text(tm(unused.unwrap_or_default()));
        let lit = r.hovered() || r.has_focus();
        let r = explained(r, label, need);
        let changed = r.changed() | crate::params_dialog::complete(ui, &r, value);
        (lit, changed)
    })
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

/// The heights of an operation as a side view, not to scale: each height's line level with
/// its row and running to its label, the material from Top down with the cut to Bottom in it,
/// its depth passes, and the tool waiting at Retract. The line of the row pointed at, or typed
/// in, is drawn stronger.
fn heights_picture(painter: &egui::Painter, p: &Picture) {
    let t = Tokens::get();
    let at = |line: HeightLine| p.rows.iter().find(|r| r.0 == line).map(|r| r.1);
    let Some(top_y) = at(HeightLine::Top) else { return };
    // A chamfer has no Bottom row: its cut is shown a little under the top.
    let bottom_y = at(HeightLine::Bottom).unwrap_or(top_y + 18.0);
    let (left, right) = (p.left, p.left + SCHEME_W - 8.0);
    let (block_l, cut_r, block_r) = (left + 4.0, left + 30.0, left + 52.0);
    let floor_y = bottom_y + 14.0;
    let material = t.text_dim.gamma_multiply(0.35);
    painter.rect_filled(egui::Rect::from_min_max(egui::pos2(cut_r, top_y), egui::pos2(block_r, floor_y)), 0.0, material);
    painter.rect_filled(egui::Rect::from_min_max(egui::pos2(block_l, bottom_y), egui::pos2(cut_r, floor_y)), 0.0, material);
    // The depth passes, evenly between Top and Bottom (the last is Bottom's own line).
    let lit_passes = p.lit == Some(HeightLine::Passes);
    let stroke = Stroke::new(if lit_passes { 1.6 } else { 1.0 }, t.accent.gamma_multiply(if lit_passes { 1.0 } else { 0.55 }));
    for k in 1..p.passes.max(1) {
        let y = top_y + (bottom_y - top_y) * k as f32 / p.passes as f32;
        painter.add(egui::Shape::dashed_line(&[egui::pos2(block_l, y), egui::pos2(cut_r, y)], stroke, 4.0, 3.0));
    }
    // The tool, its tip at Retract over the cut.
    if let Some(y) = at(HeightLine::Retract) {
        let x = (block_l + cut_r) * 0.5;
        painter.rect_filled(egui::Rect::from_min_max(egui::pos2(x - 5.0, y - 20.0), egui::pos2(x + 5.0, y)), 2.0, t.text_dim);
    }
    for (line, colour) in
        [(HeightLine::Clearance, t.text_dim), (HeightLine::Retract, t.warning), (HeightLine::Top, t.accent), (HeightLine::Bottom, t.accent)]
    {
        let Some(y) = at(line) else { continue };
        let on = p.lit == Some(line);
        let stroke = Stroke::new(if on { 2.0 } else { 1.0 }, if on { t.text } else { colour });
        painter.add(egui::Shape::dashed_line(&[egui::pos2(left, y), egui::pos2(right, y)], stroke, 5.0, 3.0));
        crate::scenario::publish_handle(&format!("cam_height:{line:?}"), egui::pos2(right, y));
    }
}

/// A labelled text box in a form; `true` when its text changed this frame.
fn field(ui: &mut egui::Ui, need: &[Need], label: &str, value: &mut String, hint: &str) -> bool {
    let need = needed(need, label);
    labelled(ui, label, |ui| {
        let r = text_box(ui, label, value, hint, need.is_some());
        let r = explained(r, label, need);
        r.changed() | crate::params_dialog::complete(ui, &r, value)
    })
}

/// A number as forms show it: at most `decimals` decimals, none wasted.
fn shown(v: f64, decimals: usize) -> String {
    let s = format!("{v:.decimals$}");
    let s = if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.').to_string() } else { s };
    // A value that rounds to nothing has no sign.
    if s == "-0" { "0".to_string() } else { s }
}

fn parsed(s: &str) -> Option<f64> {
    crate::prefs::parse_number(s).filter(|v| *v > 0.0)
}

/// Which of a tool's speed and feed rows were typed in.
#[derive(Clone, Copy, Debug, Default)]
struct Typed {
    speed: bool,
    surface: bool,
    chip: bool,
    feed: bool,
}

/// The spindle speed cutting data gives a tool `diameter` mm across: its surface speed's, else its own.
fn rpm_of(c: &CutText, diameter: Option<f64>) -> Option<f64> {
    match (c.by_surface_speed, parsed(&c.surface_speed), diameter) {
        (true, Some(v), Some(d)) => Some(solidcraft_engine::cam::rpm_from_surface_speed(v, d)),
        (true, ..) => None,
        (false, ..) => parsed(&c.spindle_rpm),
    }
}

/// Keeps a tool's speed rows and feed rows in step: of each pair (spindle speed and surface speed,
/// feed and chip load) the one typed last is kept, and the other worked out from it for the
/// tool's `diameter` (mm) and `flutes`. `own_rpm`: the speed rows left empty cut at.
fn link_tool_rates(c: &mut CutText, typed: Typed, diameter: Option<f64>, flutes: Option<f64>, own_rpm: Option<f64>) {
    if typed.surface {
        c.by_surface_speed = true;
    } else if typed.speed {
        c.by_surface_speed = false;
    }
    if typed.chip {
        c.by_chip_load = true;
    } else if typed.feed {
        c.by_chip_load = false;
    }
    let diameter = diameter.filter(|d| *d > 0.0);
    if c.by_surface_speed {
        c.spindle_rpm = rpm_of(c, diameter).map(|r| shown(r, 0)).unwrap_or_default();
    } else {
        c.surface_speed = match (parsed(&c.spindle_rpm), diameter) {
            (Some(r), Some(d)) => shown(solidcraft_engine::cam::surface_speed_from_rpm(r, d), 1),
            _ => String::new(),
        };
    }
    let (rpm, flutes) = (parsed(&c.spindle_rpm).or(own_rpm), flutes.filter(|z| *z >= 1.0));
    if c.by_chip_load {
        c.feed = match (parsed(&c.chip_load), rpm, flutes) {
            (Some(chip), Some(r), Some(z)) => shown(r * z * chip, 1),
            _ => String::new(),
        };
    } else {
        c.chip_load = match (parsed(&c.feed), rpm, flutes) {
            (Some(feed), Some(r), Some(z)) => shown(feed / (r * z), 4),
            _ => String::new(),
        };
    }
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
    let need = needed(need, label);
    labelled(ui, label, |ui| {
        let options: Vec<&str> = options.iter().map(|o| tm(o)).collect();
        let r = crate::ui_kit::dropdown(ui, id, index, &options);
        mark(ui, &r, need);
        explained(r, label, need);
    });
}

/// A text value chosen from a list (`(value, label)`); a value not in the list is shown as it is.
fn pick(ui: &mut egui::Ui, need: &[Need], id: &str, label: &str, value: &mut String, options: &[(String, String)]) {
    let need = needed(need, label);
    labelled(ui, label, |ui| {
        // Indexed for the kit's drop-down; a value not in the list shows as it is, chosen by none.
        let mut labels: Vec<&str> = options.iter().map(|(_, l)| tm(l)).collect();
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
    });
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
    /// The machine library's holders, by name.
    holders: Vec<String>,
    /// The material of the setup an operation form adds to (its tools' data is for it).
    material: Option<String>,
    /// The design's setups in order, by id and name: what a setup can take its stock from.
    setups: Vec<(String, String)>,
    /// The bodies the selection is on, each once: what a setup can take as its part.
    bodies: Vec<String>,
    /// The selected corner or edge a setup's work origin can be put on (see [`solidcraft_engine::cam_origin_pick`]).
    origin_pick: Option<(String, Vec3, solidcraft_engine::doc::manufacture::EdgePoint)>,
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
        let what = if name.is_empty() { kind_name("tool_kind", &text(t, "kind")) } else { name };
        tools.push((n.clone(), format!("T{n}  D{diameter} {what}")));
        // Its data in the setup's material, where it has some, else its own.
        let cut = material
            .as_deref()
            .and_then(|m| t["cutting"].as_array().into_iter().flatten().find(|c| c["material"].as_str().is_some_and(|n| same_material(n, m))));
        let value = |k: &str| cut.and_then(|c| c[k].as_f64()).or(t[k].as_f64());
        let flutes = t["flutes"].as_u64().unwrap_or(0) as u32;
        let (spindle_rpm, feed) = speeds(t, cut, app.preferences.cam_max_rpm);
        data.push((
            n,
            ToolData {
                diameter,
                flutes,
                spindle_rpm,
                feed,
                plunge_feed: value("plunge_feed"),
                stepdown: value("stepdown"),
                stepover: value("stepover"),
            },
        ));
    }
    let setups = listed["setups"].as_array().into_iter().flatten().filter_map(|x| Some((x["id"].as_u64()?.to_string(), text(x, "name")))).collect();
    let mut bodies: Vec<String> = Vec::new();
    for item in &app.session.selection {
        let name = match item {
            solidcraft_engine::Sel::Body { name } => name,
            solidcraft_engine::Sel::Face { body, .. } | solidcraft_engine::Sel::Edge { body, .. } => body,
            _ => continue,
        };
        if !bodies.contains(name) {
            bodies.push(name.clone());
        }
    }
    let holders = library["holders"].as_array().into_iter().flatten().filter_map(|h| h["name"].as_str()).map(str::to_string).collect();
    Choices { sketches, tools, data, heights: None, picked: None, cut_heights: None, materials, holders, material, setups, bodies, origin_pick: None }
}

/// The spindle speed (rev/min) and feed (mm/min) a listed tool cuts at: in a material, `cut`'s
/// where it gives them; a surface speed worked out for the tool's diameter and cut down to the
/// machine's `max_rpm` (0: no limit); a chip load fed at that speed. As the engine works them out.
fn speeds(tool: &Value, cut: Option<&Value>, max_rpm: f32) -> (Option<f64>, Option<f64>) {
    // Of each pair, the material's when it gives either.
    let pair = |a: &str, b: &str| {
        let from = cut.filter(|c| c[a].is_f64() || c[b].is_f64()).unwrap_or(tool);
        (from[a].as_f64(), from[b].as_f64())
    };
    let (rpm, surface_speed) = pair("spindle_rpm", "surface_speed");
    let (feed, chip_load) = pair("feed", "chip_load");
    let rpm = match (surface_speed, tool["diameter"].as_f64()) {
        (Some(v), Some(d)) if d > 0.0 => Some(solidcraft_engine::cam::rpm_from_surface_speed(v, d)),
        (Some(_), _) => None,
        (None, _) => rpm,
    };
    let rpm = rpm.map(|r| if max_rpm > 0.0 { r.min(f64::from(max_rpm)) } else { r });
    // A listing reports the chip load a feed makes, so a feed given is the one kept.
    let feed = match (feed, chip_load, rpm, tool["flutes"].as_f64()) {
        (Some(f), ..) => Some(f),
        (None, Some(c), Some(r), Some(z)) => Some((r * z * c * 10.0).round() / 10.0),
        _ => None,
    };
    (rpm, feed)
}

fn form_ui(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices, need: &[Need]) {
    if let Form::Tool(f) = form {
        tool_assistant(ui, f, need, &choices.materials, &choices.holders);
        return;
    }
    // An operation form's heights, as a picture left of their rows' labels.
    let mut picture: Option<Picture> = None;
    form_grid(ui, form, selected, choices, need, &mut picture);
    if let Some(p) = &picture {
        heights_picture(ui.painter(), p);
    }
}

/// An operation form's heights picture: what it shows, the row lit, and where the height rows are (their top and bottom).
struct Picture {
    /// Where the label column starts: the picture stands there, left of the height rows' labels.
    left: f32,
    /// Each height's row, by the middle of its line.
    rows: Vec<(HeightLine, f32)>,
    /// How many depth passes the preview cuts (none known yet: a few, to show what they are).
    passes: usize,
    lit: Option<HeightLine>,
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
    Corner,
    Taper,
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
    /// A bull nose's corner radius.
    corner: f64,
    /// An end mill's taper each side (degrees; 0: straight).
    taper: f64,
}

impl ToolShape {
    fn of(f: &ToolForm) -> ToolShape {
        let mm =
            |s: &str| crate::prefs::parse_number(s.trim().trim_end_matches("mm").trim_end_matches("deg").trim_end_matches('°')).filter(|v| *v > 0.0);
        let shape = ToolShape::with(
            f.kind,
            mm(&f.diameter),
            mm(&f.flute_length),
            mm(&f.shank_diameter),
            mm(&f.length),
            mm(&f.tip_angle),
            mm(&f.tip_diameter),
        );
        let shape = ToolShape { taper: mm(&f.taper_angle).filter(|a| *a < 45.0).unwrap_or(0.0), ..shape };
        match mm(&f.corner_radius) {
            Some(c) => ToolShape { corner: c.min(shape.diameter * 0.5), ..shape },
            None => shape,
        }
    }
    /// A shape with `kind`'s defaults wherever a value is left out (or makes no tool).
    fn with(kind: usize, d: Option<f64>, f: Option<f64>, s: Option<f64>, l: Option<f64>, a: Option<f64>, tip: Option<f64>) -> ToolShape {
        let diameter = d.unwrap_or(6.0).min(1000.0);
        let angle = a.filter(|a| *a < 179.0).unwrap_or(match kind {
            2 => 118.0,
            6 => 30.0,
            _ => 90.0,
        });
        let tip_d = tip.filter(|t| *t < diameter).unwrap_or(0.0);
        // A V-bit cuts with its cone (as the engine takes it when left out).
        let cone = ((diameter - tip_d) * 0.5 / (angle * 0.5).to_radians().tan()).max(0.01);
        let flute_length = f.unwrap_or(if tool_kind(kind).is_cone() { cone } else { diameter * 3.0 }).min(10_000.0);
        ToolShape {
            kind,
            diameter,
            flute_length,
            shank: s.unwrap_or(diameter).min(1000.0),
            length: l.filter(|l| *l >= flute_length).unwrap_or(flute_length + diameter.max(4.0) * 2.0),
            angle,
            tip: tip_d,
            // A bull nose's corner: a sixth of the diameter when not given.
            corner: diameter / 6.0,
            taper: 0.0,
        }
    }
    /// The tool it draws, for its outline.
    fn tool(&self) -> solidcraft_engine::cam::Tool {
        let kind = tool_kind(self.kind);
        solidcraft_engine::cam::Tool {
            tip_angle: Some(self.angle),
            tip_diameter: Some(self.tip),
            corner_radius: Some(self.corner),
            taper_angle: Some(self.taper),
            ..solidcraft_engine::cam::Tool::new(0, kind, self.diameter, self.flute_length)
        }
    }
}

/// One side of a tool's cutting part, from the top of its flutes down to its tip: points as
/// (distance from the axis, height over the tip), from its shape.
fn tool_side(tool: &solidcraft_engine::cam::Tool) -> Vec<(f64, f64)> {
    let (widest, flute) = (tool.widest(), tool.flute_length);
    const STEPS: u32 = 48;
    let mut side = vec![(widest, flute)];
    for k in (0..=STEPS).rev() {
        let d = widest * f64::from(k) / f64::from(STEPS);
        if let Some(h) = tool.rise(d) {
            side.push((d, h.min(flute)));
        }
    }
    side
}

/// A tool drawn upright, its point down, in `size`: shank, cutting part and point, and with
/// `dims` its dimensions D, F, L, S (and α, d for a point), `lit` drawn stronger. Returns its response.
fn tool_picture(ui: &mut egui::Ui, shape: &ToolShape, lit: Option<Dim>, size: egui::Vec2, dims: bool) -> egui::Response {
    let t = Tokens::get();
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let room = if dims { egui::vec2(40.0, 38.0) } else { egui::vec2(8.0, 8.0) };
    let inner = rect.shrink2(room);
    // A tapered tool is widest at the top of its flutes.
    let widest = (shape.tool().widest() * 2.0).max(shape.shank);
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
    // Down one side as the tool is shaped: its cone or round end, a tapered side widening up.
    let tool = shape.tool();
    let side: Vec<egui::Pos2> = tool_side(&tool).into_iter().map(|(d, h)| egui::pos2(cx + d as f32 * scale, tip_y - h as f32 * scale)).collect();
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
    let pointed = shape.kind == 2 || tool_kind(shape.kind).is_cone();
    if pointed {
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
    if shape.kind == 4 {
        // The corner's radius: from its centre out through the round, at 45°.
        let d = Dim::Corner;
        let c = (shape.corner as f32 * scale).min(r).max(3.0);
        let centre = egui::pos2(cx + r - c, tip_y - c);
        let out = centre + egui::vec2(1.0, 1.0).normalized() * c;
        let stroke = Stroke::new(width(d), colour(d));
        painter.line_segment([centre, out], stroke);
        painter.circle_filled(centre, 1.5, colour(d));
        painter.text(centre + egui::vec2(-8.0, -6.0), egui::Align2::CENTER_CENTER, "r", font(d), colour(d));
    }
    if shape.taper > 0.0 && tool_kind(shape.kind).is_end_mill() {
        // The taper: its angle beside the widening side, from upright.
        let d = Dim::Taper;
        let stroke = Stroke::new(width(d), colour(d));
        let (start, top_at) = (egui::pos2(cx + r, tip_y - r.min(12.0)), egui::pos2(cx + r, cut_top));
        painter.add(egui::Shape::dashed_line(&[start, top_at], stroke, 3.0, 2.0));
        painter.text(egui::pos2(cx + r + 8.0, (start.y + top_at.y) * 0.5), egui::Align2::LEFT_CENTER, "β", font(d), colour(d));
    }
    if tool_kind(shape.kind).is_cone() {
        let d = Dim::Tip;
        let tip = ((shape.tip * 0.5) as f32 * scale).max(2.0);
        dim_line(egui::pos2(cx - tip, tip_y + 5.0), egui::pos2(cx + tip, tip_y + 5.0), d, "d", egui::pos2(cx + tip + 12.0, tip_y + 5.0));
    }
    response
}

/// The tool assistant: the kind (tiles), the shape (with its dimensioned picture), the cutting data.
fn tool_assistant(ui: &mut egui::Ui, f: &mut ToolForm, need: &[Need], materials: &[String], holders: &[String]) {
    let t = Tokens::get();
    // Where it is: three steps, each a button back to it.
    let steps = [tm("1  Kind"), tm("2  Shape"), tm("3  Cutting data")];
    crate::ui_kit::segmented(ui, &mut f.step, &steps, crate::ui_kit::Segments::default());
    ui.add_space(t.space_8);
    match f.step {
        0 => {
            ui.label(RichText::new(tm("What kind of tool is it?")).color(t.text_dim));
            ui.horizontal_wrapped(|ui| {
                for (k, name) in TOOL_KIND_NAMES.iter().enumerate() {
                    // Stubby, so that each kind's end shows: (diameter, cutting length, shank, length).
                    let (d, fl, sh, l) = [
                        (10.0, 14.0, 8.0, 24.0),
                        (10.0, 14.0, 8.0, 24.0),
                        (8.0, 18.0, 8.0, 26.0),
                        (14.0, 7.0, 7.0, 22.0),
                        (10.0, 14.0, 8.0, 24.0),
                        (14.0, 5.0, 8.0, 22.0),
                        (6.0, 11.0, 6.0, 24.0),
                    ]
                    .get(k)
                    .copied()
                    .unwrap_or((10.0, 14.0, 8.0, 24.0));
                    // A V-bit with a small flat; a chamfer mill's flat is wider.
                    let tip = match k {
                        3 => Some(0.6),
                        5 => Some(3.0),
                        6 => Some(0.4),
                        _ => None,
                    };
                    let shape = ToolShape::with(k, Some(d), Some(fl), Some(sh), Some(l), None, tip);
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
                    inside.add(egui::Label::new(tm(name)).selectable(false));
                    r.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Button, true, tm(name)));
                    if r.clicked() {
                        f.kind = k;
                        f.step = 1;
                    }
                }
            });
        }
        1 => {
            let mut lit = None;
            // The rows, and the dimensioned picture right of them.
            let picture = egui::vec2(170.0, 250.0);
            ui.horizontal_top(|ui| {
                let rows = (ui.available_width() - picture.x - t.space_12).max(0.0);
                ui.allocate_ui_with_layout(egui::vec2(rows, 0.0), egui::Layout::top_down(egui::Align::Min), |ui| {
                    ui.set_width(rows);
                    if f.editing {
                        labelled(ui, "Number", |ui| ui.label(format!("T{}", f.number)));
                    } else {
                        field(ui, need, "Number", &mut f.number, "1");
                    }
                    labelled(ui, "Kind", |ui| ui.label(tm(TOOL_KIND_NAMES.get(f.kind).copied().unwrap_or_default())));
                    field(ui, need, "Description", &mut f.description, "");
                    let mut row = |ui: &mut egui::Ui, label: &str, value: &mut String, hint: &str, d: Dim| {
                        if field_seen(ui, need, label, value, hint).0 {
                            lit = Some(d);
                        }
                    };
                    row(ui, "Diameter (D)", &mut f.diameter, "6 mm", Dim::Diameter);
                    let cone = tool_kind(f.kind).is_cone();
                    row(ui, "Flute length (F)", &mut f.flute_length, if cone { "the cone's height" } else { "3 × D" }, Dim::Flutes);
                    row(ui, "Length (L)", &mut f.length, "F + 2 × D", Dim::Length);
                    row(ui, "Shank diameter (S)", &mut f.shank_diameter, "D", Dim::Shank);
                    if f.kind == 2 || cone {
                        let hint = match f.kind {
                            2 => "118°",
                            6 => "30°",
                            _ => "90°",
                        };
                        row(ui, "Point angle (α)", &mut f.tip_angle, hint, Dim::Angle);
                    }
                    if cone {
                        row(ui, "Tip diameter (d)", &mut f.tip_diameter, "0 mm (a sharp point)", Dim::Tip);
                    }
                    if f.kind == 4 {
                        row(ui, "Corner radius (r)", &mut f.corner_radius, "1 mm", Dim::Corner);
                    }
                    if tool_kind(f.kind).is_end_mill() {
                        row(ui, "Taper angle (β)", &mut f.taper_angle, "0° (straight)", Dim::Taper);
                    }
                    field(ui, need, "Flutes", &mut f.flutes, "2");
                    // The holder: checked against clamps and fixtures, from where the tool comes out
                    // of it. One of the library's by its shape, else a plain one.
                    let mut options: Vec<(String, String)> = vec![(String::new(), tm("Plain (diameter and length)").into())];
                    for h in holders.iter().chain([&f.holder_was, &f.holder]) {
                        if !h.is_empty() && !options.iter().any(|(n, _)| n.eq_ignore_ascii_case(h)) {
                            options.push((h.clone(), h.clone()));
                        }
                    }
                    pick(ui, need, "sc_cam_tool_holder", "Holder", &mut f.holder, &options);
                    if f.holder.is_empty() {
                        field(ui, need, "Holder diameter", &mut f.holder_diameter, "not known");
                        field(ui, need, "Holder length", &mut f.holder_length, "not known");
                    }
                });
                ui.add_space(t.space_12 - ui.spacing().item_spacing.x);
                tool_picture(ui, &ToolShape::of(f), lit, picture, true);
            });
        }
        _ => {
            // Which material's data the rows show: the library's materials, and any the tool has data for.
            let mut options: Vec<(String, String)> = vec![(String::new(), tm("Every material").into())];
            for m in materials.iter().map(String::as_str).chain(f.others.iter().map(|(m, _)| m.as_str())).chain([f.material.as_str()]) {
                if !m.is_empty() && !options.iter().any(|(n, _)| same_material(n, m)) {
                    let has = !f.kept(m).is_empty_text();
                    options.push((m.to_string(), if has { tf("manufacture.has_data", &[("material", m)]) } else { m.to_string() }));
                }
            }
            if options.len() > 1 {
                options[0].1 = tm("Any other material").into();
            }
            let own = f.kept("");
            let in_material = !f.material.is_empty();
            // In a material, an empty row is the tool's own value: its hint says what that is.
            let hint = |own: &str, usual: &'static str| -> String {
                match (in_material, own.trim()) {
                    (false, _) => tm(usual).to_string(),
                    (true, "") => tm("the tool's: none").to_string(),
                    (true, v) => tf("manufacture.the_tools_value", &[("value", v)]),
                }
            };
            {
                let mut chosen = f.material.clone();
                pick(ui, need, "sc_cam_tool_material", "Material", &mut chosen, &options);
                f.show_material(&chosen);
                // Speed and surface speed belong together through the diameter, and speed, flutes,
                // chip load and feed through feed = speed × flutes × chip load.
                let typed = Typed {
                    speed: field(ui, need, "Spindle (rev/min)", &mut f.spindle_rpm, &hint(&own.spindle_rpm, "none")),
                    surface: field(ui, need, "Surface speed (m/min)", &mut f.surface_speed, &hint(&own.surface_speed, "speed × π × diameter ÷ 1000")),
                    chip: field(ui, need, "Chip load (mm/tooth)", &mut f.chip_load, &hint(&own.chip_load, "feed ÷ (speed × flutes)")),
                    feed: field(ui, need, "Feed (mm/min)", &mut f.feed, &hint(&own.feed, "speed × flutes × chip load")),
                };
                let (diameter, flutes) = (f.diameter_mm(), parsed(&f.flutes));
                // A material without a speed of its own cuts at the tool's.
                let own_rpm = rpm_of(&own, diameter);
                let mut c = f.shown();
                link_tool_rates(&mut c, typed, diameter, flutes, own_rpm.filter(|_| in_material));
                f.show(c);
                field(ui, need, "Plunge feed (mm/min)", &mut f.plunge_feed, &hint(&own.plunge_feed, "feed / 3"));
                field(ui, need, "Stepdown (mm)", &mut f.stepdown, &hint(&own.stepdown, "half the diameter"));
                field(ui, need, "Stepover (mm)", &mut f.stepover, &hint(&own.stepover, "by operation"));
            }
            let note = if in_material {
                "Rows left empty take the tool's own value. A setup whose stock is of this material cuts with these."
            } else if options.len() > 1 {
                "The tool's own data: for setups of any material it has no data of its own for, and for setups without a material."
            } else {
                "Materials are added in the Tool Library; a tool can then have data for each."
            };
            crate::ui_kit::status_line(ui, tm(note), crate::ui_kit::StatusTone::Dim);
        }
    }
    ui.add_space(t.space_8);
    ui.horizontal(|ui| {
        if button(ui, tm("< Back"), ui_kit::Button::Enabled(f.step > 0)).clicked() {
            f.step -= 1;
        }
        if button(ui, tm("Next >"), ui_kit::Button::Enabled(f.step < 2)).clicked() {
            f.step += 1;
        }
    });
}

/// The heights picture's width (points).
/// How wide the heights picture is, left of the height rows' labels.
const SCHEME_W: f32 = 64.0;

/// A 3D operation form's face roles: what they are, a button to machine every face again, and
/// buttons giving the selected faces a role.
fn face_roles_rows(ui: &mut egui::Ui, f: &mut OpForm, selected: &[(String, Vec3)]) {
    let t = Tokens::get();
    labelled(ui, "Face roles", |ui| {
        if f.face_roles.is_empty() {
            ui.label(RichText::new(tm("none: every face is machined")).color(t.text_dim));
        } else {
            let count = |k: FaceRoleKind| f.face_roles.iter().filter(|r| r.role == k).count().to_string();
            let summary = tf(
                "manufacture.face_roles_summary",
                &[("ignored", &count(FaceRoleKind::Ignore)), ("avoided", &count(FaceRoleKind::Avoid)), ("left", &count(FaceRoleKind::Leave))],
            );
            ui.label(summary);
            if ui_kit::compact_button(ui, "×", tm("Machine every face again"), true, false).clicked() {
                f.face_roles.clear();
            }
        }
    });
    if selected.is_empty() {
        return;
    }
    let given = |text: &str| crate::prefs::parse_number(text.trim().trim_end_matches("mm").trim()).filter(|v| *v >= 0.0 && *v <= 100.0);
    let clearance = if f.role_clearance.trim().is_empty() { Some(1.0) } else { given(&f.role_clearance) };
    let leave = given(&f.role_leave);
    let mut chosen: Option<(FaceRoleKind, f64)> = None;
    let n = selected.len().to_string();
    // One role a line, so the form keeps its width with faces selected.
    beneath(ui, |ui| {
        if button(ui, tf("manufacture.ignore_selected", &[("count", &n)]), Button::Small)
            .on_hover_text(tm("Leave them out of the part this operation follows: the tool may cut through them"))
            .clicked()
        {
            chosen = Some((FaceRoleKind::Ignore, 0.0));
        }
    });
    // A role's amount: a short box after its button.
    let amount = t.form_label_width;
    beneath(ui, |ui| {
        if button(ui, tf("manufacture.avoid_selected", &[("count", &n)]), ui_kit::Button::Enabled(clearance.is_some()))
            .on_hover_text(tm("Keep the tool at least this far off them"))
            .clicked()
            && let Some(c) = clearance
        {
            chosen = Some((FaceRoleKind::Avoid, c));
        }
        ui.scope(|ui| {
            ui.set_max_width(amount);
            ui_kit::text_field(ui, &mut f.role_clearance, "1 mm");
        });
    });
    beneath(ui, |ui| {
        if button(ui, tf("manufacture.leave_on_selected", &[("count", &n)]), ui_kit::Button::Enabled(leave.is_some()))
            .on_hover_text(tm("Leave this much on them instead of the operation's own allowance"))
            .clicked()
            && let Some(l) = leave
        {
            chosen = Some((FaceRoleKind::Leave, l));
        }
        ui.scope(|ui| {
            ui.set_max_width(amount);
            ui_kit::text_field(ui, &mut f.role_leave, "0.3 mm");
        });
    });
    if let Some((role, mm)) = chosen {
        // A face given a role again takes the new one.
        f.face_roles.retain(|r| !selected.iter().any(|(body, at)| *body == r.body && *at == r.at));
        f.face_roles.extend(selected.iter().map(|(body, at)| FaceRoleRow { body: body.clone(), at: *at, role, mm }));
    }
}

/// The form's rows; an operation form leaves its heights in `picture` for beside them.
fn form_grid(ui: &mut egui::Ui, form: &mut Form, selected: &[(String, Vec3)], choices: &Choices, need: &[Need], picture: &mut Option<Picture>) {
    match form {
        // Tools have an assistant of their own (`tool_assistant`).
        Form::Tool(_) => {}
        Form::Setup(f) => {
            field(ui, need, "Name", &mut f.name, "automatic");
            choice(ui, need, "sc_cam_setup_axis", "Tool axis", &mut f.axis, &AXES);
            // The part: every body, or those named.
            field(ui, need, "Part bodies", &mut f.bodies, "every body");
            if !choices.bodies.is_empty() && choices.bodies.join(", ") != f.bodies {
                beneath(ui, |ui| {
                    if button(ui, tf("manufacture.part_the_selected_bodies", &[("count", &choices.bodies.len().to_string())]), Button::Small)
                        .clicked()
                    {
                        f.bodies = choices.bodies.join(", ");
                    }
                });
            }
            // What holds the stock: bodies that are never the part, kept clear of.
            field(ui, need, "Fixtures", &mut f.fixtures, "none");
            if !choices.bodies.is_empty() && choices.bodies.join(", ") != f.fixtures {
                beneath(ui, |ui| {
                    if button(ui, tf("manufacture.fixtures_the_selected_bodies", &[("count", &choices.bodies.len().to_string())]), Button::Small)
                        .clicked()
                    {
                        f.fixtures = choices.bodies.join(", ");
                    }
                });
            }
            let shapes = STOCK_SHAPES.map(|(_, label)| label);
            choice(ui, need, "sc_cam_setup_stock", "Stock", &mut f.stock_shape, &shapes);
            match f.stock_shape {
                1 => {
                    let [x, y, z] = &mut f.stock_size;
                    field(ui, need, "Stock size X", x, "required");
                    field(ui, need, "Stock size Y", y, "required");
                    field(ui, need, "Stock size Z", z, "required");
                    field(ui, need, "Stock on top", &mut f.top, "0 mm");
                }
                2 => {
                    field(ui, need, "Bar diameter", &mut f.diameter, "required");
                    field(ui, need, "Stock on top", &mut f.top, "0 mm");
                    field(ui, need, "Stock below", &mut f.bottom, "0 mm");
                }
                _ => {
                    field(ui, need, "Stock on the sides", &mut f.side, "0 mm");
                    field(ui, need, "Stock on top", &mut f.top, "0 mm");
                    field(ui, need, "Stock below", &mut f.bottom, "0 mm");
                }
            }
            choice(ui, need, "sc_cam_setup_origin", "Origin on", &mut f.origin, &ORIGINS);
            if f.origin == 3 {
                // What was picked, and a button that takes what is selected now.
                let wanted = needed(need, "Origin point");
                labelled(ui, "Origin point", |ui| {
                    let t = Tokens::get();
                    let shown = match &f.edge {
                        Some((body, _)) => RichText::new(tf("manufacture.on_the_body", &[("body", body)])),
                        None => RichText::new(tm("nothing picked yet")).color(if wanted.is_some() { t.error } else { t.text_dim }),
                    };
                    explained(ui.label(shown), "Origin point", wanted);
                });
                let now = choices.origin_pick.as_ref().map(|(body, at, _)| (body.clone(), *at));
                if let Some((body, at, on)) = choices.origin_pick.as_ref().filter(|_| now != f.edge) {
                    beneath(ui, |ui| {
                        if button(ui, tm("Take the selected corner or edge"), Button::Small).clicked() {
                            f.edge = Some((body.clone(), *at));
                            f.edge_on = match on {
                                solidcraft_engine::doc::manufacture::EdgePoint::Vertex => 0,
                                solidcraft_engine::doc::manufacture::EdgePoint::Middle => 1,
                                solidcraft_engine::doc::manufacture::EdgePoint::Center => 2,
                            };
                        }
                    });
                }
                choice(ui, need, "sc_cam_setup_edge_on", "Origin at", &mut f.edge_on, &EDGE_POINTS.map(|e| e.1));
                choice(ui, need, "sc_cam_setup_edge_z", "Origin Z", &mut f.edge_z, &ORIGIN_ZS.map(|z| z.1));
            } else if f.origin == 2 {
                let [x, y, z] = &mut f.point;
                field(ui, need, "Origin X", x, "0");
                field(ui, need, "Origin Y", y, "0");
                field(ui, need, "Origin Z", z, "0");
            } else {
                // Where on the box, along each of the setup's own axes.
                let [x, y, z] = &mut f.anchors;
                choice(ui, need, "sc_cam_setup_x", "Origin X", x, &["min", "centre", "max"]);
                choice(ui, need, "sc_cam_setup_y", "Origin Y", y, &["min", "centre", "max"]);
                choice(ui, need, "sc_cam_setup_z", "Origin Z", z, &["bottom", "middle", "top"]);
            }
            choice(ui, need, "sc_cam_setup_post", "Post", &mut f.post, solidcraft_engine::cam::Post::BUILTIN);
            choice(ui, need, "sc_cam_setup_offset", "Work offset", &mut f.work_offset, &WORK_OFFSETS);
            choice(ui, need, "sc_cam_setup_x", "X direction", &mut f.x_axis, &X_DIRECTIONS);
            // The setups before this one.
            let mut earlier: Vec<(String, String)> = vec![(String::new(), tm("its own stock box").into())];
            let here = f.editing.map(|id| id.to_string());
            earlier.extend(choices.setups.iter().take_while(|(id, _)| Some(id) != here.as_ref()).cloned());
            pick(ui, need, "sc_cam_setup_stock_from", "Stock from", &mut f.stock_from, &earlier);
            // Clamps: from-to across and their top, each on a row of its own.
            let mut gone = None;
            for (k, c) in f.clamps.iter_mut().enumerate() {
                let label = tf("manufacture.clamp_number", &[("number", &(k + 1).to_string())]);
                crate::ui_kit::row(ui, &label, |ui| {
                    // Five boxes sharing the row with the remove button.
                    let gap = ui.spacing().item_spacing.x;
                    let each = ((ui.available_width() - crate::ui_kit::row_height() - gap * 5.0) / 5.0).max(0.0);
                    for (i, (text, hint)) in c.iter_mut().zip(["X from", "Y from", "X to", "Y to", "top Z"]).enumerate() {
                        ui.scope(|ui| {
                            ui.set_max_width(each);
                            let r = crate::ui_kit::cell_editor(ui, ui.make_persistent_id(("cam_clamp", k, i)), text, tm(hint), false);
                            explained(r, "Clamps", None);
                        });
                    }
                    if ui_kit::compact_button(ui, "×", tm("Remove this clamp"), true, false).clicked() {
                        gone = Some(k);
                    }
                });
            }
            if let Some(k) = gone {
                f.clamps.remove(k);
            }
            // Copies of the part: how many along X and Y, then how far apart.
            for (label, range, hints) in [("Copies", 0..2, ["along X", "along Y"]), ("Spacing", 2..4, ["X apart", "Y apart"])] {
                labelled(ui, label, |ui| {
                    let each = ((ui.available_width() - ui.spacing().item_spacing.x) / 2.0).max(0.0);
                    for (i, hint) in range.zip(hints) {
                        ui.scope(|ui| {
                            ui.set_max_width(each);
                            if let Some(text) = f.copies.get_mut(i) {
                                let r = crate::ui_kit::cell_editor(ui, ui.make_persistent_id(("cam_copies", i)), text, tm(hint), false);
                                explained(r, label, None);
                            }
                        });
                    }
                });
            }
            beneath(ui, |ui| {
                if button(ui, tm("Add Clamp"), Button::Small).on_hover_text(tm(help("Clamps").unwrap_or_default())).clicked() && f.clamps.len() < 64 {
                    f.clamps.push(Default::default());
                }
            });
            let mut materials: Vec<(String, String)> = vec![(String::new(), tm("not said").into())];
            materials.extend(choices.materials.iter().map(|m| (m.clone(), m.clone())));
            pick(ui, need, "sc_cam_setup_material", "Material", &mut f.material, &materials);
        }
        Form::Op(f) => {
            field(ui, need, "Name", &mut f.name, "automatic");
            if f.faces.is_empty() {
                // Facing covers a box: the stock's, the part's, or the one round a sketch's profiles.
                if f.kind == OpKind::Face {
                    choice(
                        ui,
                        need,
                        "sc_cam_face_boundary",
                        "Boundary",
                        &mut f.boundary,
                        &["the raw block", "the part's outline", "a sketch's outline"],
                    );
                }
                if matches!(f.kind, OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d) {
                    // Without a sketch, the whole stock (finishing: the model's box); with one, only inside its profiles.
                    let none = if f.kind == OpKind::Adaptive3d { "none: the whole stock" } else { "none: the part's box" };
                    let mut options: Vec<(String, String)> = vec![(String::new(), tm(none).into())];
                    options.extend(choices.sketches.iter().cloned());
                    pick(ui, need, "sc_cam_sketch", "Sketch", &mut f.sketch, &options);
                } else if f.kind != OpKind::Face || f.boundary == 2 {
                    pick(ui, need, "sc_cam_sketch", "Sketch", &mut f.sketch, &choices.sketches);
                }
            } else {
                labelled(ui, "Faces", |ui| {
                    ui.label(tf("manufacture.faces_picked", &[("count", &f.faces.len().to_string())]));
                    if button(ui, tm("Use a sketch"), Button::Small).clicked() {
                        f.faces.clear();
                    }
                });
            }
            if !selected.is_empty() && selected != f.faces.as_slice() {
                beneath(ui, |ui| {
                    if button(ui, tf("manufacture.cut_the_selected_faces", &[("count", &selected.len().to_string())]), Button::Small).clicked() {
                        f.faces = selected.to_vec();
                    }
                });
            }
            if f.kind.follows_the_model() {
                face_roles_rows(ui, f, selected);
            }
            let from_faces = !f.faces.is_empty();
            pick(ui, need, "sc_cam_tool", "Tool", &mut f.tool, &choices.tools);
            // Another tool brings its own speed, feeds, stepdown and stepover; all stay editable here.
            prefill(f, choices);
            // Filled from the tool's data in the setup's material, when the setup says one.
            let tools = match &choices.material {
                Some(m) => tf("manufacture.the_tools_for", &[("material", m)]),
                None => tm("the tool's").to_string(),
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
                Some(Some(z)) => tf("manufacture.height_given", &[("height", &shown(z, 3)), ("what", tm(what))]),
                Some(None) => tm("required: type it in").to_string(),
                None => tm(what).to_string(),
            };
            let given = lists_heights(choices);
            // Beside each, a button that takes the height from what is selected in the model.
            let (low, high) = (choices.picked.map(|p| p.0), choices.picked.map(|p| p.1));
            // The heights the preview cuts at fill the automatic ones in.
            let seen = choices.cut_heights.as_ref();
            // Known from the preview: the value; else how it is found.
            let automatic = |z: Option<f64>, what: &str| {
                z.map_or_else(|| tm(what).to_string(), |z| tf("manufacture.height_automatic", &[("height", &shown(z, 3))]))
            };
            let mut lit = None;
            // Where the height rows are: the picture left of them draws each line level with its row.
            let (mut rows, mut left) = (Vec::new(), 0.0);
            // The picture takes the start of the label column; the rows' own, narrower label
            // column ends where the form's does, so their boxes line up with the other rows'.
            let indent = ui.ctx().data(|d| d.get_temp::<f32>(egui::Id::new(LABEL_COLUMN))).unwrap_or(0.0);
            ui.horizontal_top(|ui| {
                left = ui.cursor().left();
                ui.allocate_exact_size(egui::vec2(SCHEME_W, 0.0), egui::Sense::hover());
                let labels = (indent - Tokens::get().space_14 - SCHEME_W - ui.spacing().item_spacing.x).max(0.0);
                ui.vertical(|ui| {
                    crate::ui_kit::settings_form(ui, labels, |ui| {
                        let mut row = |ui: &mut egui::Ui, line: HeightLine, label: &str, value: &mut String, hint: &str, picked: Option<f64>| {
                            let (on, middle) = height_field(ui, need, label, value, hint, picked);
                            rows.push((line, middle));
                            if on {
                                lit = Some(line);
                            }
                        };
                        row(ui, HeightLine::Clearance, "Clearance", &mut f.clearance, &automatic(seen.map(|h| h.clearance), "10 mm over all"), high);
                        row(ui, HeightLine::Retract, "Retract", &mut f.retract, &automatic(seen.map(|h| h.retract), "2 mm over the top"), high);
                        row(ui, HeightLine::Top, "Top", &mut f.top, &auto(top, given.map(|h| h.0)), high);
                        // A chamfer's depth comes from its width, a V-carve's from its shape.
                        if !matches!(f.kind, OpKind::Chamfer | OpKind::VCarve) {
                            row(ui, HeightLine::Bottom, "Bottom", &mut f.bottom, &auto(bottom, given.map(|h| h.1)), low);
                        }
                    });
                });
            });
            // Drilling has no stepdown (its peck depth is further down).
            // Not used while a number of passes divides the depth.
            let unused =
                (!f.passes.trim().is_empty()).then_some("Not used: the passes divide the depth evenly. Empty Passes to cut by the stepdown.");
            // A bore goes down on its helix (its depth per turn is further down).
            // 3D finishing follows the surface: one layer.
            let levels = !matches!(
                f.kind,
                OpKind::Drill | OpKind::Bore | OpKind::Chamfer | OpKind::VCarve | OpKind::Thread | OpKind::Parallel3d | OpKind::ZLevel3d
            );
            if levels && field_unless(ui, need, "Stepdown", &mut f.stepdown, "half the tool diameter", unused).0 {
                lit = Some(HeightLine::Passes);
            }
            // Or how many passes: the depth is then divided evenly.
            if levels && field_seen(ui, need, "Passes", &mut f.passes, "by the stepdown").0 {
                lit = Some(HeightLine::Passes);
            }
            // The passes the preview cuts; until there is one, a few to show what the line is.
            let passes = if levels { seen.map_or(3, |h| h.levels.len().max(1)) } else { 1 };
            *picture = Some(Picture { left, rows, passes, lit });
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
                    check_row(ui, "Tabs", "Hold the part", &mut f.tabs);
                    if f.tabs {
                        field(ui, need, "Tab count", &mut f.tab_count, "4 on each loop");
                        field(ui, need, "Tab width", &mut f.tab_width, "6 mm");
                        field(ui, need, "Tab height", &mut f.tab_height, "2 mm");
                    }
                    field(ui, need, "Onion skin", &mut f.skin, "none");
                    if !f.skin.trim().is_empty() {
                        check_row(ui, "Skin", "Cut it off last", &mut f.skin_last);
                    }
                    // Arcs onto the wall, from off the part.
                    field(ui, need, "Lead-in radius", &mut f.lead, "none: ramps along the loop");
                    check_row(ui, "Dogbones", "Into sharp inside corners", &mut f.dogbones);
                }
                OpKind::Pocket => {
                    field(ui, need, "Stepover", &mut f.stepover, "the tool's, or 40% of its diameter");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    field(ui, need, "Finish allowance", &mut f.finish, "none");
                    // Rest machining: what a larger tool left, in the corners and narrow places.
                    let mut options: Vec<(String, String)> = vec![(String::new(), tm("none: the whole pocket").into())];
                    options.extend(choices.tools.iter().filter(|(n, _)| *n != f.tool).cloned());
                    pick(ui, need, "sc_cam_rest_from", "Rest of tool", &mut f.rest_from, &options);
                    check_row(ui, "Dogbones", "Into sharp inside corners", &mut f.dogbones);
                }
                OpKind::ZLevel3d => {
                    field(ui, need, "Level step", &mut f.level_step, "a twelfth of the tool's diameter");
                    field(ui, need, "Walls from", &mut f.steep, "30° (0: everywhere)");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    rest_row(ui, &mut f.rest3d);
                }
                OpKind::Parallel3d => {
                    field(ui, need, "Stepover", &mut f.stepover, "a tenth of the tool's diameter");
                    field(ui, need, "Pass angle", &mut f.pass_angle, "0° (along X)");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    labelled(ui, "Direction", |ui| {
                        let r = crate::ui_kit::checkbox(ui, tm("Both ways (zigzag)"), &mut f.zigzag);
                        explained(r, "Zigzag", None);
                    });
                    rest_row(ui, &mut f.rest3d);
                }
                OpKind::Adaptive3d => {
                    field(ui, need, "Stepover (%)", &mut f.stepover_percent, "20");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm, sides and floors");
                    field(ui, need, "Step-up height", &mut f.step_up, "a quarter of the stepdown (0: none)");
                    field(ui, need, "Helix angle", &mut f.helix_angle, "5 deg");
                    field(ui, need, "Helix largest (%)", &mut f.helix_max, "100");
                    field(ui, need, "Helix smallest (%)", &mut f.helix_min, "10");
                    field(ui, need, "Light-cut feed (%)", &mut f.feed_up, "100: the feed");
                }
                OpKind::Adaptive => {
                    field(ui, need, "Stepover (%)", &mut f.stepover_percent, "20");
                    field(ui, need, "Stock to leave", &mut f.stock_to_leave, "0 mm");
                    field(ui, need, "Finish allowance", &mut f.finish, "none");
                    field(ui, need, "Helix angle", &mut f.helix_angle, "5 deg");
                    field(ui, need, "Helix largest (%)", &mut f.helix_max, "100");
                    field(ui, need, "Helix smallest (%)", &mut f.helix_min, "10");
                    field(ui, need, "Light-cut feed (%)", &mut f.feed_up, "100: the feed");
                    let mut options: Vec<(String, String)> = vec![(String::new(), tm("none: the whole pocket").into())];
                    options.extend(choices.tools.iter().filter(|(n, _)| *n != f.tool).cloned());
                    pick(ui, need, "sc_cam_adaptive_rest", "Rest of tool", &mut f.rest_from, &options);
                }
                OpKind::Chamfer => {
                    f.side = f.side.min(1);
                    choice(ui, need, "sc_cam_chamfer_side", "Side", &mut f.side, &["outside", "inside"]);
                    field(ui, need, "Chamfer width", &mut f.width, "1 mm");
                    field(ui, need, "Tip offset", &mut f.tip_offset, "0.5 mm");
                }
                OpKind::Thread => {
                    field(ui, need, "Pitch", &mut f.pitch, "1.25 mm for M8");
                    field(ui, need, "Thread diameter", &mut f.thread_diameter, "from the drawn circle");
                    let mut side = usize::from(f.thread_outside);
                    choice(ui, need, "sc_cam_thread_side", "Thread side", &mut side, &["inside a hole", "round a boss"]);
                    f.thread_outside = side == 1;
                    let mut hand = usize::from(f.thread_left);
                    choice(ui, need, "sc_cam_thread_hand", "Hand", &mut hand, &["right-hand", "left-hand"]);
                    f.thread_left = hand == 1;
                    field(ui, need, "Thread passes", &mut f.thread_passes, "1");
                }
                OpKind::VCarve => {
                    field(ui, need, "Stepover", &mut f.stepover, "2% of the tool's diameter");
                    field(ui, need, "Deepest", &mut f.max_depth, "as deep as the shape asks");
                    field(ui, need, "Stepdown", &mut f.stepdown, "the tool's");
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
            if matches!(
                f.kind,
                OpKind::Contour
                    | OpKind::Pocket
                    | OpKind::Bore
                    | OpKind::Chamfer
                    | OpKind::Adaptive
                    | OpKind::Adaptive3d
                    | OpKind::ZLevel3d
                    | OpKind::VCarve
                    | OpKind::Thread
            ) {
                check_row(ui, "Direction", "Climb", &mut f.climb);
            }
            check_row(ui, "Coolant", "On", &mut f.coolant);
        }
    }
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
            if f.kind == 4 && empty(&f.corner_radius) {
                want("Corner radius (r)", "the bull nose's corner radius");
            }
        }
        Form::Setup(f) => {
            if f.origin == 3 && f.edge.is_none() {
                want("Origin point", "a corner or an edge of the model: click it, then take it");
            }
            if f.origin == 2 {
                for (field, value) in [("Origin X", &f.point[0]), ("Origin Y", &f.point[1]), ("Origin Z", &f.point[2])] {
                    if empty(value) {
                        want(field, "where the work zero is, in the world");
                    }
                }
            }
            match f.stock_shape {
                1 => {
                    for (field, value) in [("Stock size X", &f.stock_size[0]), ("Stock size Y", &f.stock_size[1]), ("Stock size Z", &f.stock_size[2])]
                    {
                        if empty(value) {
                            want(field, "how big the raw block is");
                        }
                    }
                }
                2 if empty(&f.diameter) => want("Bar diameter", "how thick the round bar is"),
                _ => {}
            }
        }
        Form::Op(f) => {
            // Facing chooses a box instead, and 3D roughing roughs the whole stock without one.
            if f.faces.is_empty()
                && (f.kind != OpKind::Face || f.boundary == 2)
                && !matches!(f.kind, OpKind::Adaptive3d | OpKind::Parallel3d | OpKind::ZLevel3d)
                && empty(&f.sketch)
            {
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
            if f.kind == OpKind::Thread && empty(&f.pitch) {
                want("Pitch", "the thread's pitch");
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
        let rows: Vec<&str> = rows.into_iter().map(tm).collect();
        lines.push(tf("manufacture.still_needed", &[("rows", &rows.join(", "))]));
    }
    lines.extend(need.iter().filter(|n| n.field.is_empty()).map(|n| tm(&n.why).to_string()));
    lines.join("\n")
}

/// Which form is open, as its heading in the catalog says it: a form that changes kind (or
/// opens anew) starts scrolled to its top.
fn form_heading(form: &Form) -> &'static str {
    match form {
        Form::Tool(f) if f.editing => "EDIT TOOL",
        Form::Tool(_) => "NEW TOOL",
        Form::Setup(f) if f.editing.is_some() => "EDIT SETUP",
        Form::Setup(_) => "NEW SETUP",
        Form::Op(f) if f.editing.is_some() => "EDIT OPERATION",
        Form::Op(f) => f.kind.title(),
    }
}

/// The card's icon and title, as the app's command dialogs have them: the command's own icon
/// and name, and "Edit …" for a form that changes what is there.
fn form_title(form: &Form) -> (&'static str, String) {
    let command = match form {
        Form::Tool(_) => "cam.tool.define",
        Form::Setup(_) => "cam.setup.create",
        Form::Op(f) => f.kind.command(),
    };
    let spec = solidcraft_engine::find_command(command);
    let icon = spec.as_ref().map_or("", |s| s.icon);
    let title = match form {
        Form::Tool(f) if !f.editing => tm("New Tool").to_string(),
        Form::Setup(f) if f.editing.is_none() => {
            spec.as_ref().map_or_else(|| tm(form_heading(form)).to_string(), |s| crate::i18n::tr_label(s.label).to_string())
        }
        Form::Op(f) if f.editing.is_none() => {
            spec.as_ref().map_or_else(|| tm(form_heading(form)).to_string(), |s| crate::i18n::tr_label(s.label).to_string())
        }
        _ => tm(form_heading(form)).to_string(),
    };
    (icon, title)
}

/// The open form as the app's command dialogs are (`ui_kit::Panel::feature`): docked at the
/// right under the view cube and dragged with them, its header, the rows scrolling between it
/// and the footer with OK and Cancel (or OK and Cancel in the header, with Preferences ›
/// Interface › Dialog buttons at the top). Enter, with nothing typed in, is OK; Esc (when no
/// command dialog took it, `esc_taken`) is Cancel. → where it was drawn, and whether it took Esc.
#[allow(clippy::too_many_arguments)]
fn form_card(
    ctx: &egui::Context,
    cam: &mut CamUi,
    selected: &[(String, Vec3)],
    lists: &Choices,
    docked: egui::Pos2,
    bounds: egui::Rect,
    layout: crate::shell::LayoutMode,
    esc_taken: bool,
    asked: &mut Asked,
) -> (Option<egui::Rect>, bool) {
    let Some(form) = &cam.form else { return (None, false) };
    let heading = form_heading(form);
    let (icon, title) = form_title(form);
    let need = missing(form, lists);
    let t = Tokens::get();
    let side = crate::frame::Side::Right;
    let drag_key = side.dialog_offset_key();
    let moved = crate::frame::dock_offset(ctx, drag_key);
    let anchor = crate::frame::offset_anchor(docked, moved, bounds, side);
    let frame = crate::ui_kit::Panel::feature_frame(ctx, layout, side);
    // Two cards wide: the rows, the heights picture and the tool's picture need the room.
    let width = (t.form_card_width * 2.0).min(bounds.width());
    let panel = crate::ui_kit::Panel::new(&title, egui::Id::new(FORM_CARD), anchor, bounds, side, frame)
        .width(width)
        .reset_scroll(cam.form_seen != Some(heading));
    cam.form_seen = Some(heading);
    let mut answer = (false, crate::ui_kit::Action::None);
    let response = panel.window(ctx).show(ctx, |ui| {
        let options = crate::ui_kit::FeatureOptions { icon, title: &title, drag_key, moved, docked, apply_label: None };
        answer = panel.feature(ui, options, false, need.is_empty(), |ui| {
            form_body(ui, cam, selected, lists, &need, asked);
            crate::ui_kit::ComputeStatus { text: None, busy: false, applying: false }
        });
    });
    let esc = ctx.input(|i| i.key_pressed(egui::Key::Escape));
    let (closed, action) = answer;
    let mut took_esc = false;
    match action {
        crate::ui_kit::Action::Ok => asked.apply = true,
        // Esc a command dialog answered first is not this form's too.
        crate::ui_kit::Action::Cancel if !(esc && esc_taken) => {
            (cam.form, cam.error) = (None, None);
            took_esc = esc;
        }
        _ if closed => (cam.form, cam.error) = (None, None),
        _ => {}
    }
    (response.map(|r| r.response.rect), took_esc)
}

/// Closes the open form without running its command, as its Cancel does (Esc).
pub fn cancel_form(app: &mut SolveApp) {
    (app.manufacture.form, app.manufacture.error) = (None, None);
}

/// What the form's card holds: what went wrong, the rows, what an operation form also offers
/// (its defaults, a simulation of it), what the form still needs and what stops its toolpath.
fn form_body(ui: &mut egui::Ui, cam: &mut CamUi, selected: &[(String, Vec3)], lists: &Choices, need: &[Need], asked: &mut Asked) {
    let t = Tokens::get();
    if let Some(e) = &cam.error {
        crate::ui_kit::status_line(ui, e, StatusTone::Error);
    }
    let Some(form) = &mut cam.form else { return };
    // One label column for every row: as wide as the kit lets a wide card's grow, or wider
    // where a height's label beside the heights picture needs it; never more than half the row.
    let heights = ["Clearance", "Retract", "Top", "Bottom"].map(|l| crate::ui_kit::ContentWidth::label(ui, tm(l))).into_iter().fold(0.0, f32::max);
    let kit = (t.form_label_width + t.space_24 + t.space_10) * LABEL_GROWTH;
    let column = kit.max(SCHEME_W + ui.spacing().item_spacing.x + heights + t.space_4).min(ui.available_width() * 0.5);
    ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new(LABEL_COLUMN), column + t.space_14));
    crate::ui_kit::settings_form(ui, column, |ui| form_ui(ui, form, selected, lists, need));
    let wanted = need_text(need);
    if matches!(form, Form::Op(_)) {
        ui.add_space(t.space_8);
        ui.horizontal_wrapped(|ui| {
            // An operation is simulated as the form has it, before OK.
            asked.simulate_form = button(ui, tm("Simulate"), ui_kit::Button::Enabled(need.is_empty()))
                .on_hover_text(tm("Cut this operation, with the values in the form, on what the operations before it leave; OK still applies it"))
                .on_disabled_hover_text(&wanted)
                .clicked();
            asked.save_default = button(ui, tm("Save as Default"), Button::Small)
                .on_hover_text(tm(
                    "New operations of this kind start with this form's tool, speeds, stepdown and the like (not its geometry or heights)",
                ))
                .clicked();
            asked.forget_default = button(ui, tm("Built-in Defaults"), Button::Small)
                .on_hover_text(tm("New operations of this kind start with SolidCraft's own defaults again"))
                .clicked();
        });
    }
    // OK stays off until nothing is missing; what is missing is said here.
    if !need.is_empty() {
        crate::ui_kit::status_line(ui, &wanted, StatusTone::Error);
    }
    // What stops the toolpath, said before OK.
    let op = matches!(form, Form::Op(_));
    match (op, &cam.preview) {
        (true, Some((_, _, Err(e)))) => {
            crate::ui_kit::status_line(ui, &tf("manufacture.cannot_be_cut", &[("why", e)]), StatusTone::Error);
        }
        (true, Some((_, _, Ok(solidcraft_engine::CamPreview { warning: Some(w), .. })))) => {
            crate::ui_kit::status_line(ui, &tf("manufacture.toolpath_warning", &[("warning", &engine_text(w))]), StatusTone::Warning);
        }
        _ => {}
    }
    if op {
        simulation_block(ui, cam, asked);
    }
}

/// The form card's id.
const FORM_CARD: &str = "sc_cam_form";
/// How far the kit lets a wide card's label column grow past its basis.
const LABEL_GROWTH: f32 = 1.6;

/// Where a panel docks: its top right corner, the room it is dragged in, and the window's
/// layout, which the kit's frame follows.
#[derive(Clone, Copy)]
struct Dock {
    anchor: egui::Pos2,
    bounds: egui::Rect,
    layout: crate::shell::LayoutMode,
}

/// A panel as the app's managers are (`ui_kit::Panel::feature_collapsible`, as Parameters):
/// docked at the right, dragged by its header on its own (`drag_key`), folded to its header,
/// closed by its × or the footer's Close. Esc leaves it open: it is no dialog to cancel.
/// → whether it stays open, and where it was drawn.
#[allow(clippy::too_many_arguments)]
fn overview_card(
    ctx: &egui::Context,
    id: &str,
    drag_key: &'static str,
    icon: &str,
    title: &str,
    dock: Dock,
    folded: &mut bool,
    body: impl FnOnce(&mut egui::Ui),
) -> (bool, Option<egui::Rect>) {
    let t = Tokens::get();
    let side = crate::frame::Side::Right;
    let moved = crate::frame::dock_offset(ctx, drag_key);
    let anchor = crate::frame::offset_anchor(dock.anchor, moved, dock.bounds, side);
    let frame = crate::ui_kit::Panel::feature_frame(ctx, dock.layout, side);
    let width = (t.form_card_width * 2.0).min(dock.bounds.width());
    let panel = crate::ui_kit::Panel::new(title, egui::Id::new(id), anchor, dock.bounds, side, frame).width(width).actions("", "Close");
    let mut answer = (false, crate::ui_kit::Action::None);
    let response = panel.window(ctx).show(ctx, |ui| {
        let options = crate::ui_kit::FeatureOptions { icon, title, drag_key, moved, docked: dock.anchor, apply_label: None };
        answer = panel.feature_collapsible(ui, options, false, true, folded, |ui| {
            body(ui);
            crate::ui_kit::ComputeStatus { text: None, busy: false, applying: false }
        });
    });
    // Its own handles beside the kit's shared ones, so automation can tell the panels apart.
    if let Some(header) = crate::scenario::rect_of("dialog:header") {
        crate::scenario::publish_handle(&format!("{id}:title"), header.center());
    }
    let esc = ctx.input(|i| i.key_pressed(egui::Key::Escape));
    let (closed, action) = answer;
    let close = closed || (action == crate::ui_kit::Action::Cancel && !esc);
    (!close, response.map(|r| r.response.rect))
}

/// A command's icon and its name in this language, for the card that shows it.
fn command_heading(command: &str) -> (&'static str, String) {
    let spec = solidcraft_engine::find_command(command);
    (spec.as_ref().map_or("", |s| s.icon), spec.as_ref().map_or_else(String::new, |s| crate::i18n::tr_label(s.label).to_string()))
}

/// The Manufacture panel's id.
const MANUFACTURE_PANEL: &str = "sc_cam_window";

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
    /// The form was picked in the browser: it does not replace an open form with changes.
    from_browser: bool,
    apply: bool,
    simulate: Option<u64>,
    /// Simulate what was simulated last again (its detail changed).
    resimulate: bool,
    /// One operation to simulate on the stock the ones before it leave: its setup and itself.
    simulate_op: Option<(u64, u64)>,
    /// Simulate the open operation form as it stands.
    simulate_form: bool,
    /// Keep the open operation form's rows as the default of its kind, or forget that default.
    save_default: bool,
    forget_default: bool,
    /// A setup whose simulated program to play.
    play: Option<u64>,
    /// Write a program: the setup, one operation of it or all (`None`), and the setup's post.
    post: Option<(u64, Option<(u64, String)>, String)>,
    import: bool,
    export: bool,
}

/// Why Simulate and Post Process are off on a setup without operations.
const NO_OPS: &str = "This setup has no operation yet: add one from the toolbar (Facing, 2D Contour, 2D Pocket, Adaptive, Chamfer, Bore or Drill)";

/// The engine's warnings about a toolpath in this window's language: how deep a program or an
/// operation cuts into the part; other messages as the engine words them.
fn engine_text(w: &str) -> String {
    // Several warnings of one operation, one after the other.
    if w.contains("; ") {
        return w.split("; ").map(engine_text).collect::<Vec<_>>().join("; ");
    }
    let between = |s: &str, a: &str, b: &str| s.strip_prefix(a).and_then(|r| r.strip_suffix(b)).map(str::to_string);
    if let Some(clamp) = w.strip_prefix("the holder runs into clamp ") {
        return tf("manufacture.the_holder_runs_into_clamp", &[("clamp", clamp)]);
    }
    if let Some(clamp) = w.strip_prefix("runs into clamp ") {
        return tf("manufacture.runs_into_named_clamp", &[("clamp", clamp)]);
    }
    if let Some((depth, reach)) = between(w, "goes ", " mm out of its holder")
        .and_then(|r| r.split_once(" mm deep but the tool reaches ").map(|(d, r)| (d.to_string(), r.to_string())))
    {
        return tf("manufacture.deeper_than_the_tool_reaches", &[("depth", &depth), ("reach", &reach)]);
    }
    if let Some(depth) = between(w, "cuts up to ", " mm into the part") {
        return tf("manufacture.op_cuts_into_part", &[("depth", &depth)]);
    }
    if let Some((depth, area)) = between(w, "the program cuts up to ", " mm²")
        .and_then(|r| r.split_once(" mm into the part, over ").map(|(d, a)| (d.to_string(), a.to_string())))
    {
        return tf("manufacture.program_cuts_into_part", &[("depth", &depth), ("area", &area)]);
    }
    tm(w).to_string()
}

/// A kind the engine names (`pocket`, `ball_endmill`) as people read it: `group` is
/// `op_kind` or `tool_kind`, since a drill is a tool as well as an operation.
fn kind_name(group: &str, kind: &str) -> String {
    let id = format!("manufacture.{group}_{kind}");
    match crate::i18n::tr(&id) {
        known if known != id => known.to_string(),
        _ => kind.replace('_', " "),
    }
}

/// The operations the toolbar offers, by their command.
const OPERATIONS: &[&str] = &[
    "cam.op.face",
    "cam.op.contour",
    "cam.op.pocket",
    "cam.op.adaptive",
    "cam.op.adaptive3d",
    "cam.op.parallel3d",
    "cam.op.zlevel3d",
    "cam.op.chamfer",
    "cam.op.vcarve",
    "cam.op.bore",
    "cam.op.thread",
    "cam.op.drill",
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
            let caption = RichText::new(format!("{}  {}", k + 1, tm(name)));
            let (caption, fill, stroke) = match (is_done, is_next) {
                (true, _) => (caption.color(t.text), Some(t.accent_soft), None),
                (_, true) => (caption.strong().color(t.text), None, Some(Stroke::new(1.5, t.accent))),
                _ => (caption.color(t.text_dim), None, None),
            };
            let why = ["", "Make a setup first", "Add an operation first", "Add an operation first"];
            let tip = [
                "The raw block and the work zero. Click to make one, or to change it.",
                "Facing, 2D Contour, 2D Pocket, Adaptive, Chamfer, V-Carve, Bore, Thread or Drill: click to choose one.",
                "Cut the raw block with every operation and see what is left, or cut into the part.",
                "Write the setup's program for the machine.",
            ];
            let r = ui
                .add_enabled_ui(open, |ui| tinted(ui, fill, stroke, |ui| button(ui, caption, Button::Normal)))
                .inner
                .on_hover_text(tm(tip.get(k).copied().unwrap_or_default()))
                .on_disabled_hover_text(tm(why.get(k).copied().unwrap_or_default()));
            if k == 1 {
                // The operations to choose from, under the step.
                crate::ui_kit::Container::new(ui.ctx(), crate::ui_kit::Surface::Feature).menu(&r).show(|ui| {
                    for cmd in OPERATIONS {
                        // The toolbar's own name for it.
                        if button(ui, crate::i18n::tr(&format!("cmd.{}", cmd.replace('.', "_"))), Button::Normal).clicked() {
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
    ui.label(RichText::new(tm(line)).color(if next.is_some() { t.text } else { t.text_dim }));
}

/// The tool table's columns.
const TOOL_HEADS: [&str; 9] = ["Tool", "Kind", "Diameter", "Flutes", "Spindle", "Surface speed", "Chip load", "Feed", "Design"];

/// A table of tools, each row picked by clicking its name. `library`: these are the machine library's tools.
fn tool_table(ui: &mut egui::Ui, id: &str, tools: &[Value], library: bool, picked: &mut Option<(bool, u64)>) {
    let t = Tokens::get();
    egui::ScrollArea::horizontal().id_salt(id).auto_shrink([false, true]).show(ui, |ui| {
        // A table's cells are never wrapped: it scrolls sideways instead.
        ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
        egui::Grid::new(id).num_columns(TOOL_HEADS.len()).spacing([18.0, 6.0]).striped(true).show(ui, |ui| {
            for head in TOOL_HEADS {
                ui.label(RichText::new(tm(head)).color(t.text_dim));
            }
            ui.end_row();
            for tool in tools {
                let number = tool["number"].as_u64().unwrap_or(0);
                let data = |k: &str, decimals: usize| tool[k].as_f64().map(|v| shown(v, decimals));
                // Cutting data a tool has none of: each operation is given it instead.
                let cell = |ui: &mut egui::Ui, v: Option<String>| match v {
                    Some(v) => ui.label(v),
                    None => ui.label(RichText::new("—").color(t.text_dim)).on_hover_text(tm("Not set: operations with this tool ask for it")),
                };
                let name = text(tool, "description");
                let label = if name.is_empty() { format!("T{number}") } else { format!("T{number}  {name}") };
                if crate::ui_kit::list_picker(ui, &format!("cam_tool_{library}_{number}"), &label, *picked == Some((library, number))).clicked() {
                    *picked = if *picked == Some((library, number)) { None } else { Some((library, number)) };
                }
                // A tapered end mill says how much.
                let kind = kind_name("tool_kind", &text(tool, "kind"));
                ui.label(match tool["taper_angle"].as_f64() {
                    Some(a) => tf("manufacture.kind_with_taper", &[("kind", &kind), ("angle", &shown(a, 2))]),
                    None => kind,
                });
                cell(ui, data("diameter", 3).map(|d| format!("{d} mm")));
                cell(ui, data("flutes", 0));
                // Each pair as the tool gives it or as it works out (without the machine's limit).
                let (rpm, feed) = speeds(tool, None, 0.0);
                let surface_speed = match (tool["surface_speed"].as_f64(), rpm, tool["diameter"].as_f64()) {
                    (Some(v), ..) => Some(v),
                    (None, Some(r), Some(d)) => Some(solidcraft_engine::cam::surface_speed_from_rpm(r, d)),
                    _ => None,
                };
                cell(ui, rpm.map(|r| shown(r, 0)));
                cell(ui, surface_speed.map(|v| shown(v, 1)));
                cell(ui, data("chip_load", 4));
                cell(ui, feed.map(|f| shown(f, 1)));
                // How this design's copy of a library tool stands.
                match tool["design"].as_str() {
                    Some("same") => {
                        ui.label(RichText::new(tm("in use")).color(t.text_dim)).on_hover_text(tm("This design uses it as the library has it"))
                    }
                    Some("differs") => ui
                        .label(RichText::new(tm("changed")).color(t.warning))
                        .on_hover_text(tm("Changed in the library since this design copied it")),
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
fn tools_window(ctx: &egui::Context, cam: &mut CamUi, library: &Value, dock: Dock, asked: &mut Asked) {
    let t = Tokens::get();
    let mut folded = cam.tools_folded;
    let (icon, title) = command_heading("cam.library.list");
    let (open, _) = overview_card(ctx, "sc_cam_tools", "CAM_TOOLS", icon, &title, dock, &mut folded, |ui| {
        {
            // Everything is as wide as the panel, never wider: tables scroll inside it.
            ui.set_max_width(ui.available_width());
            if let Some(e) = cam.error.as_ref().filter(|_| !cam.open && cam.form.is_none()) {
                crate::ui_kit::status_line(ui, e, StatusTone::Error);
            }
            if let Some(e) = library["error"].as_str() {
                crate::ui_kit::status_line(ui, e, StatusTone::Error);
            }
            let tools = library["tools"].as_array().cloned().unwrap_or_default();
            let own = library["only_in_design"].as_array().cloned().unwrap_or_default();
            // A pick that is no longer in either list is forgotten.
            let listed = |in_library: bool, number: u64| if in_library { &tools } else { &own }.iter().find(|x| x["number"].as_u64() == Some(number));
            if cam.selected_tool.is_some_and(|(in_library, number)| listed(in_library, number).is_none()) {
                cam.selected_tool = None;
            }
            crate::ui_kit::section_count(ui, "sc_cam_tools_machine", tm("This machine's tools"), true, Some(tools.len()), |ui| {
                crate::ui_kit::status_line(ui, tm("shared by every design"), StatusTone::Dim).on_hover_text(text(library, "path"));
                if tools.is_empty() {
                    crate::ui_kit::status_line(ui, tm("No tools yet. New Tool adds the first."), StatusTone::Dim);
                } else {
                    tool_table(ui, "sc_cam_tool_table", &tools, true, &mut cam.selected_tool);
                }
            });
            if !own.is_empty() {
                crate::ui_kit::section_count(ui, "sc_cam_tools_design", tm("Only in this design"), true, Some(own.len()), |ui| {
                    crate::ui_kit::status_line(ui, tm("not in the machine's library"), StatusTone::Dim);
                    tool_table(ui, "sc_cam_design_tools", &own, false, &mut cam.selected_tool);
                });
            }
            ui.add_space(t.space_8);
            // What can be done: always New Tool and the library file; with a tool picked, to that tool.
            ui.horizontal_wrapped(|ui| {
                if button(ui, tm("New Tool"), Button::Normal).clicked() {
                    asked.new_tool = true;
                }
                if let Some((in_library, number)) = cam.selected_tool
                    && let Some(tool) = listed(in_library, number)
                {
                    ui.separator();
                    ui.label(RichText::new(format!("T{number}")).strong());
                    if button(ui, tm("Edit"), Button::Normal).clicked() {
                        asked.form = tool_form(tool, in_library);
                    }
                    if button(ui, tm("Remove"), Button::Normal).clicked() {
                        let id = if in_library { "cam.library.remove" } else { "cam.tool.remove" };
                        asked.run.push((id, json!({ "number": number })));
                    }
                    if !in_library && button(ui, tm("Add to library"), Button::Normal).clicked() {
                        asked.run.push(("cam.library.add", json!({ "number": number })));
                    }
                    if tool["design"].as_str() == Some("differs")
                        && button(ui, tm("Update this design"), Button::Normal)
                            .on_hover_text(tm("Replace this design's older copy of the tool with the library's"))
                            .clicked()
                    {
                        asked.run.push(("cam.tool.update", json!({ "number": number })));
                    }
                } else {
                    ui.label(RichText::new(tm("Pick a tool to change or remove it")).color(t.text_dim));
                }
                ui.separator();
                asked.import = button(ui, tm("Import…"), Button::Normal)
                    .on_hover_text(tm("Add the tools of a library file to this machine's library: SolidCraft's own, or an RP23CNC toollibrary.json"))
                    .clicked();
                asked.export = button(ui, tm("Export…"), Button::Normal).on_hover_text(tm("Write this machine's library to a file")).clicked();
            });
            for (k, line) in cam.import_notes.iter().enumerate() {
                crate::ui_kit::status_line(ui, line, if k == 0 { StatusTone::Dim } else { StatusTone::Warning });
            }
            ui.add_space(t.space_8);
            materials_block(ui, cam, library, asked);
            holders_block(ui, cam, library, asked);
        }
    });
    cam.tools_folded = folded;
    cam.tools_open = open;
}

/// The Manufacture window: setups with their operations, what is drawn, and the open setup or operation form.
fn manufacture_window(ctx: &egui::Context, cam: &mut CamUi, listed: &Value, dock: Dock, asked: &mut Asked) {
    let t = Tokens::get();
    let mut folded = cam.folded;
    let (icon, title) = command_heading("cam.list");
    let (open, rect) = overview_card(ctx, MANUFACTURE_PANEL, "CAM_MANUFACTURE", icon, &title, dock, &mut folded, |ui| {
        {
            if let Some(e) = &cam.error {
                crate::ui_kit::status_line(ui, e, StatusTone::Error);
            }
            let setups = listed["setups"].as_array().cloned().unwrap_or_default();
            steps(ui, cam, &setups, asked);
            ui.add_space(t.space_4);
            if setups.is_empty() {
                ui.label(RichText::new(tm("No setup yet: New Setup on the toolbar makes the first")).color(t.text_dim));
            }
            for setup in &setups {
                let id = setup["id"].as_u64().unwrap_or(0);
                let ops = setup["operations"].as_array().cloned().unwrap_or_default();
                ui.add_space(t.space_4);
                ui.horizontal(|ui| {
                    ui.label(RichText::new(text(setup, "name")).strong());
                    if !ops.is_empty() {
                        let total: f64 = ops.iter().filter_map(|o| o["toolpath"]["cut_minutes"].as_f64()).sum();
                        ui.label(RichText::new(minutes_text(total)).color(t.text_dim));
                        let loaded = setup["tool_changes"].as_u64().unwrap_or(0).to_string();
                        ui.label(RichText::new(tf("manufacture.tools_loaded", &[("count", &loaded)])).color(t.text_dim));
                    }
                });
                if let Some(e) = setup.get("error").and_then(Value::as_str) {
                    crate::ui_kit::status_line(ui, e, StatusTone::Error);
                }
                let to_check = ops.iter().filter(|o| o.get("error").is_some() || o.get("warning").is_some()).count();
                if to_check > 0 {
                    ui.label(RichText::new(tf("manufacture.ops_to_check", &[("count", &to_check.to_string())])).color(t.warning));
                }
                // How the program is written this time.
                ui.add_space(t.space_4);
                ui.horizontal_wrapped(|ui| {
                    crate::ui_kit::checkbox(ui, tm("Stop before each operation (M1)"), &mut cam.optional_stops).on_hover_text(tm(
                        "An optional stop before every operation after the first: the machine pauses there when its optional stop switch is on",
                    ));
                    crate::ui_kit::checkbox(ui, tm("Air cut"), &mut cam.air_cut)
                        .on_hover_text(tm("Writes the program with every Z raised, to run it above the stock first and watch where the tool goes"));
                    if cam.air_cut {
                        crate::ui_kit::number_field(ui, &mut cam.air_height, 1.0..=1000.0, 1.0, " mm");
                    }
                });
                // The setup's whole program: the one thing everything here leads to.
                ui.add_space(t.space_4);
                let label = RichText::new(format!("{}  ({})", tm("Post Process"), text(setup, "post"))).strong().size(t.font_heading).color(t.text);
                let post = ui.add_enabled_ui(!ops.is_empty(), |ui| {
                    ui.vertical_centered_justified(|ui| {
                        ui.spacing_mut().button_padding.y = t.space_8;
                        tinted(ui, Some(t.accent_soft), None, |ui| button(ui, label, Button::Normal))
                    })
                    .inner
                });
                if post
                    .inner
                    .on_hover_text(tm("Write every operation of this setup, in order, as one G-code program"))
                    .on_disabled_hover_text(tm(NO_OPS))
                    .clicked()
                {
                    asked.post = Some((id, None, text(setup, "post")));
                }
            }
            simulation_block(ui, cam, asked);
        }
    });
    cam.folded = folded;
    cam.panel = rect;
    cam.open = open;
}

/// What the last simulation found (its first line says what it covered), and its playback.
fn simulation_block(ui: &mut egui::Ui, cam: &mut CamUi, asked: &mut Asked) {
    let t = Tokens::get();
    let Some((found, warnings)) = &cam.simulation else { return };
    ui.separator();
    for (k, line) in found.iter().enumerate() {
        let text = RichText::new(line);
        ui.label(if k == 0 { text.strong() } else { text.color(t.text_dim) });
    }
    for line in warnings {
        ui.label(RichText::new(engine_text(line)).color(t.warning));
    }
    ui.horizontal_wrapped(|ui| {
        crate::ui_kit::checkbox(ui, tm("Simulated stock"), &mut cam.show_stock);
        // Finer shows the cut stock closer to the toolpaths, and takes longer.
        let detail = SIM_DETAIL.map(|d| d.0);
        if small_choice(ui, "sc_cam_sim_detail", "Detail", "Simulation detail", &mut cam.sim_detail, &detail) {
            asked.resimulate = true;
        }
    });
    play_controls(ui, cam, asked);
}

/// Draw the windows that are open, and keep what the viewport shows up to date. Called once per frame.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    confirm::post_confirmation(app, ctx);
    if !app.manufacture.open && !app.manufacture.tools_open {
        return;
    }
    tick(app, ctx);
    app.manufacture.repaint = Some(ctx.clone());
    if app.manufacture.open && (app.manufacture.show_paths || app.manufacture.show_setups) {
        refresh(app);
    }
    let listed = listed(app).0;
    let selected = selected_faces(app);
    let library = library_listing(app, false);
    let mut lists = choices(app, &listed, &library);
    lists.heights = form_heights(app);
    lists.picked = picked_heights(app);
    lists.origin_pick = origin_pick(app);
    update_preview(app, &lists);
    lists.cut_heights = match &app.manufacture.preview {
        Some((_, _, Ok(p))) => p.heights.clone(),
        _ => None,
    };
    // Docked like the app's command dialogs: at the right edge of their area, under the view
    // cube; left of a command dialog that is open, and the Tool Library left of the Manufacture panel.
    let vp = crate::shell::content_rect(app, app.viewport.rect.unwrap_or_else(|| ctx.content_rect()));
    let top = crate::shell::panel_top(app, vp, vp.bottom(), crate::frame::Side::Right);
    let gap = Tokens::get().space_8;
    let right = app.viewport.dialog_rect.filter(|_| app.dialog.is_some()).map_or(vp.right(), |r| r.left() - gap);
    let mut asked = Asked::default();
    app.manufacture.posted_now = app.manufacture.posted_at == Some(app.session.revision);
    if app.manufacture.form.is_none() {
        app.manufacture.form_seen = None;
    }
    // The display choices are preferences, kept between runs.
    let prefs = &app.preferences;
    (app.manufacture.sim_detail, app.manufacture.path_width, app.manufacture.path_colour) =
        (prefs.cam_sim_detail, prefs.cam_path_width, prefs.cam_path_colour);
    app.manufacture.rapid_rate = f64::from(prefs.cam_rapid_rate);
    let layout = app.ui.layout;
    let bounds = crate::shell::tool_drag_rect(app, vp);
    let mut cam = std::mem::take(&mut app.manufacture);
    // An open form is a card of its own where the Manufacture panel docks; the panel waits
    // under it (the browser lists the setups and operations meanwhile).
    if cam.form.is_some() {
        let (rect, took_esc) = form_card(ctx, &mut cam, &selected, &lists, egui::pos2(right, top), bounds, layout, app.esc_handled, &mut asked);
        app.esc_handled |= took_esc;
        cam.panel = rect;
    } else if cam.open {
        manufacture_window(ctx, &mut cam, &listed, Dock { anchor: egui::pos2(right, top), bounds, layout }, &mut asked);
    }
    if cam.tools_open {
        // Beside where the Manufacture panel docks, wherever it was dragged to.
        // The kit's panels keep `space_8` off the viewport's right edge.
        let docked = right.min(vp.right() - gap);
        let beside = match cam.panel.filter(|_| cam.open || cam.form.is_some()) {
            Some(panel) => docked - panel.width() - gap,
            None => right,
        };
        tools_window(ctx, &mut cam, &library, Dock { anchor: egui::pos2(beside, top), bounds, layout }, &mut asked);
    }
    (app.preferences.cam_sim_detail, app.preferences.cam_path_width, app.preferences.cam_path_colour) =
        (cam.sim_detail, cam.path_width, cam.path_colour);
    app.preferences.cam_rapid_rate = cam.rapid_rate as f32;
    // A simulation of a form as it stood goes when the form closes.
    if cam.form.is_none() && cam.scope.draft.is_some() {
        (cam.simulation, cam.stock, cam.play, cam.scope) = (None, None, None, SimScope::default());
    }
    // A form opened some other way (an automation setting it) counts as opened as first drawn.
    if cam.form.is_none() {
        cam.form_opened = None;
    } else if cam.form_opened.is_none() {
        cam.form_opened = cam.form.clone();
    }
    app.manufacture = cam;
    answer(app, asked);
}

/// The machine library's materials: each with a button to remove it, and a box to add one.
fn materials_block(ui: &mut egui::Ui, cam: &mut CamUi, library: &Value, asked: &mut Asked) {
    let t = Tokens::get();
    let materials: Vec<&str> = library["materials"].as_array().into_iter().flatten().filter_map(Value::as_str).collect();
    crate::ui_kit::section_count(ui, "sc_cam_materials", tm("Materials"), true, Some(materials.len()), |ui| {
        crate::ui_kit::status_line(ui, tm("what stock is made of: a tool can have cutting data for each"), StatusTone::Dim);
        if materials.is_empty() {
            crate::ui_kit::status_line(ui, tm("None yet. Add one, then give tools their data for it (Edit, Cutting data)."), StatusTone::Dim);
        }
        ui.horizontal_wrapped(|ui| {
            for m in &materials {
                ui.label(*m);
                if ui_kit::compact_button(ui, "×", &tf("manufacture.remove_material", &[("material", m)]), true, false)
                    .on_hover_text(tm("Tools in the library lose their data for it; designs keep theirs"))
                    .clicked()
                {
                    asked.run.push(("cam.material.remove", json!({ "name": m })));
                }
                ui.add_space(t.space_8);
            }
        });
        ui.horizontal(|ui| {
            let (enter, _) = ui
                .scope(|ui| {
                    ui.set_max_width(t.form_card_width);
                    ui_kit::text_field(ui, &mut cam.new_material, tm("Plywood, MDF, Aluminium…"))
                })
                .inner;
            let named = !cam.new_material.trim().is_empty();
            if (button(ui, tm("Add material"), ui_kit::Button::Enabled(named)).clicked() || enter) && named {
                asked.run.push(("cam.material.define", json!({ "name": cam.new_material.trim() })));
                cam.new_material.clear();
            }
        });
    });
}

/// A holder as typed in the Tool Library window: its name, and its sections from where the tool
/// comes out of it upward, each its length, its diameter and its diameter at its top (empty: the same).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HolderDraft {
    pub name: String,
    pub sections: Vec<[String; 3]>,
}

impl HolderDraft {
    /// One of the library's holders, to change.
    fn of(holder: &Value) -> HolderDraft {
        let mm = |v: &Value| v.as_f64().map(|x| shown(x, 3)).unwrap_or_default();
        let sections = holder["sections"].as_array().into_iter().flatten();
        HolderDraft {
            name: text(holder, "name"),
            sections: sections.map(|s| [mm(&s["length"]), mm(&s["diameter"]), mm(&s["top_diameter"])]).collect(),
        }
    }
    /// What `cam.holder.define` takes, once it has a name and every section its length and diameter.
    fn command(&self) -> Option<Value> {
        let name = self.name.trim();
        if name.is_empty() || self.sections.is_empty() {
            return None;
        }
        // A unit typed after a number is dropped.
        let mm = |s: &str| parsed(s.trim().trim_end_matches("mm").trim());
        let mut sections = Vec::new();
        for [length, diameter, top] in &self.sections {
            let mut s = json!({ "length": mm(length)?, "diameter": mm(diameter)? });
            if !top.trim().is_empty() {
                s["top_diameter"] = json!(mm(top)?);
            }
            sections.push(s);
        }
        Some(json!({ "name": name, "sections": sections }))
    }
}

/// The machine library's holders by their shape, each with buttons to change or remove it, and
/// rows to type one in: its name and its sections.
fn holders_block(ui: &mut egui::Ui, cam: &mut CamUi, library: &Value, asked: &mut Asked) {
    let t = Tokens::get();
    let holders = library["holders"].as_array().cloned().unwrap_or_default();
    crate::ui_kit::section_count(ui, "sc_cam_holders", tm("Holders"), true, Some(holders.len()), |ui| {
        crate::ui_kit::status_line(ui, tm("what tools sit in, by their shape: checked against clamps and fixtures"), StatusTone::Dim);
        if holders.is_empty() {
            crate::ui_kit::status_line(ui, tm("None yet. Type one in below, then pick it in a tool's shape."), StatusTone::Dim);
        }
        for h in &holders {
            let name = text(h, "name");
            let sections = h["sections"].as_array().cloned().unwrap_or_default();
            let long: f64 = sections.iter().filter_map(|s| s["length"].as_f64()).sum();
            let wide = sections.iter().flat_map(|s| [s["diameter"].as_f64(), s["top_diameter"].as_f64()]).flatten().fold(0.0, f64::max);
            ui.horizontal(|ui| {
                ui.label(RichText::new(&name).strong());
                let summary = tf(
                    "manufacture.holder_summary",
                    &[("sections", &sections.len().to_string()), ("length", &shown(long, 1)), ("diameter", &shown(wide, 1))],
                );
                ui.label(RichText::new(summary).color(t.text_dim));
                if button(ui, tm("Change"), Button::Normal).clicked() {
                    cam.new_holder = HolderDraft::of(h);
                }
                if ui_kit::compact_button(ui, "×", &tf("manufacture.remove_holder", &[("holder", &name)]), true, false)
                    .on_hover_text(tm("Tools keep their copy of its shape"))
                    .clicked()
                {
                    asked.run.push(("cam.holder.remove", json!({ "name": name })));
                }
            });
        }
        ui.add_space(t.space_8);
        let d = &mut cam.new_holder;
        if d.sections.is_empty() {
            d.sections.push(Default::default());
        }
        crate::ui_kit::row(ui, tm("Name"), |ui| ui_kit::text_field(ui, &mut d.name, tm("ER20 collet chuck…")));
        egui::Grid::new("sc_cam_holder_sections").num_columns(5).spacing([t.space_8, t.space_4]).show(ui, |ui| {
            ui.label("");
            for heading in ["Length", "Diameter", "Top diameter"] {
                ui.label(RichText::new(tm(heading)).color(t.text_dim));
            }
            ui.label("");
            ui.end_row();
            let (many, mut gone) = (d.sections.len() > 1, None);
            for (k, row) in d.sections.iter_mut().enumerate() {
                ui.label((k + 1).to_string());
                for (cell, hint) in row.iter_mut().zip(["20 mm", "35 mm", tm("the same")]) {
                    ui.scope(|ui| {
                        ui.set_max_width(t.form_label_width);
                        ui_kit::text_field(ui, cell, hint);
                    });
                }
                if many && ui_kit::compact_button(ui, "×", tm("Remove this section"), true, false).clicked() {
                    gone = Some(k);
                } else if !many {
                    ui.label("");
                }
                ui.end_row();
            }
            if let Some(k) = gone {
                d.sections.remove(k);
            }
        });
        crate::ui_kit::status_line(
            ui,
            tm("Sections from where the tool comes out of the holder, upward. A top diameter makes a cone, as a collet nut is."),
            StatusTone::Dim,
        );
        ui.horizontal(|ui| {
            if button(ui, tm("Add section"), ui_kit::Button::Enabled(d.sections.len() < solidcraft_engine::cam::MAX_HOLDER_SECTIONS)).clicked() {
                d.sections.push(Default::default());
            }
            let command = d.command();
            let known = holders.iter().any(|h| text(h, "name").trim().eq_ignore_ascii_case(d.name.trim()));
            let label = if known { tm("Save holder") } else { tm("Add holder") };
            if button(ui, label, ui_kit::Button::Enabled(command.is_some())).clicked()
                && let Some(p) = command
            {
                asked.run.push(("cam.holder.define", p));
                *d = HolderDraft::default();
            }
            let typed = !d.name.trim().is_empty() || d.sections.len() > 1 || d.sections.iter().flatten().any(|c| !c.trim().is_empty());
            if typed && button(ui, tm("Clear"), Button::Normal).clicked() {
                *d = HolderDraft::default();
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
        let cam = &app.manufacture;
        let changed = cam.form.is_some() && cam.form_opened.is_some() && cam.form != cam.form_opened;
        if asked.from_browser && changed && cam.form.as_ref() != Some(&form) {
            app.set_status(tm("The open form has changes: OK keeps them, Cancel drops them"), false);
        } else {
            app.manufacture.form_opened = Some(form.clone());
            app.manufacture.form = Some(form);
            app.manufacture.error = None;
        }
    }
    if asked.apply {
        apply(app);
    }
    if let Some(setup) = asked.simulate {
        simulate_setup(app, setup);
    }
    if let (true, Some(setup)) = (asked.resimulate, app.manufacture.simulated) {
        simulate_scope(app, setup, app.manufacture.scope.clone());
    }
    if let Some((setup, op)) = asked.simulate_op {
        simulate_op(app, setup, op);
    }
    if asked.simulate_form {
        simulate_form(app);
    }
    if let Some(Form::Op(f)) = &app.manufacture.form
        && (asked.save_default || asked.forget_default)
    {
        let kind = f.kind.command();
        let title = crate::i18n::tr(&format!("cmd.{}", kind.replace('.', "_"))).to_string();
        if asked.save_default {
            app.preferences.cam_op_defaults.insert(kind.to_string(), remembered(f));
            app.set_status(tf("manufacture.default_saved", &[("kind", &title)]), false);
        } else {
            app.preferences.cam_op_defaults.remove(kind);
            app.set_status(tf("manufacture.default_forgotten", &[("kind", &title)]), false);
        }
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
        // Rapids that run into material drive the tool through the stock at full speed: confirmed first.
        let collisions = rapid_collisions(app, setup, op.as_ref().map(|o| o.0));
        let clamped = clamped_ops(app, setup, op.as_ref().map(|o| o.0));
        if collisions > 0 || !clamped.is_empty() {
            app.manufacture.confirm_post = Some(PendingPost { setup, op, post, collisions, clamped });
        } else {
            pick_post(app, setup, op, post);
        }
    }
}

/// A program to write: its setup, one operation of it or all, the setup's post, and how many
/// of its rapid moves run into material.
#[derive(Clone, Debug, PartialEq)]
pub struct PendingPost {
    pub setup: u64,
    pub op: Option<(u64, String)>,
    pub post: String,
    pub collisions: u64,
    /// Its operations that run into a clamp.
    pub clamped: Vec<String>,
}

/// The operations of a setup (or the one given) that run into one of its clamps.
fn clamped_ops(app: &mut SolveApp, setup: u64, op: Option<u64>) -> Vec<String> {
    let listed = listed(app).0;
    listed["setups"]
        .as_array()
        .into_iter()
        .flatten()
        .filter(|x| x["id"].as_u64() == Some(setup))
        .flat_map(|x| x["operations"].as_array().cloned().unwrap_or_default())
        .filter(|o| op.is_none_or(|id| o["id"].as_u64() == Some(id)))
        .filter(|o| o["warning"].as_str().is_some_and(|w| w.contains("runs into clamp")))
        .map(|o| text(&o, "name"))
        .collect()
}

/// How many rapid moves of a setup's program (or of one operation of it, on what the ones
/// before it leave) run into material, as a quick simulation finds; none when it cannot tell.
fn rapid_collisions(app: &mut SolveApp, setup: u64, op: Option<u64>) -> u64 {
    let Some(s) = app.session.doc.manufacture.setups.iter().find(|s| s.id == setup) else { return 0 };
    let mut p = json!({ "setup": setup, "columns": 150 });
    if let Some(op) = op {
        let first: Vec<u64> = s.operations.iter().take_while(|o| o.id != op).map(|o| o.id).collect();
        (p["ops"], p["first"]) = (json!([op]), json!(first));
    }
    app.session
        .execute("cam.simulate", &p)
        .ok()
        .map_or(0, |v| v["operations"].as_array().into_iter().flatten().filter_map(|o| o["rapid_collisions"].as_u64()).sum())
}

/// Asks where to write a program, then writes it.
fn pick_post(app: &mut SolveApp, setup: u64, op: Option<(u64, String)>, post: String) {
    {
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
        app.manufacture.repaint = Some(ui.ctx().clone());
        // While 3D roughing is worked out, the rows stay as they were.
        if let (listed, true) = listed(app) {
            app.manufacture.browser_listing = Some((revision, listed));
        }
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
    if let Some(at) = r.secondary {
        crate::context_menu::open_for(app, at, crate::context_menu::Target::CamFolder);
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
        let total: f64 = ops.iter().filter_map(|o| o["toolpath"]["cut_minutes"].as_f64()).sum();
        let total = if ops.is_empty() { String::new() } else { minutes_text(total) };
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
                secondary: &total,
                ..Default::default()
            },
        );

        if let Some(resp) = &r.resp {
            let hover = setup
                .get("error")
                .and_then(Value::as_str)
                .map_or_else(|| tf("manufacture.post_name", &[("post", &text(setup, "post"))]), str::to_string);
            resp.clone().on_hover_text(hover);
        }
        if r.eye {
            toggle_hidden(app, id);
        } else if r.fold {
            toggle(app, &skey, true);
        } else if r.clicked || r.double {
            // The setup picked is the one the panel shows.
            app.manufacture.picked = Some((id, None));
            (asked.form, asked.from_browser) = (setup_form(setup), true);
            app.manufacture.open = true;
        }
        if let Some(at) = r.secondary {
            app.manufacture.picked = Some((id, None));
            crate::context_menu::open_for(app, at, crate::context_menu::Target::Cam { setup: id, op: None });
        }
        if !sopen {
            continue;
        }
        for (at, op) in ops.iter().enumerate() {
            let Some(op_id) = op["id"].as_u64() else { continue };
            let error = op.get("error").and_then(Value::as_str);
            let warning = op.get("warning").and_then(Value::as_str);
            let minutes = op["toolpath"]["cut_minutes"].as_f64().map(minutes_text).unwrap_or_default();
            let label = text(op, "name");
            // A bore is a hole too: it shares the drill's icon.
            let icon = match text(op, "kind").as_str() {
                "bore" => "cam_drill".to_string(),
                "chamfer" | "vcarve" => "cam_contour".to_string(),
                "thread" => "cam_drill".to_string(),
                "adaptive" | "adaptive_3d" => "cam_pocket".to_string(),
                "parallel_3d" | "zlevel_3d" => "cam_contour".to_string(),
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
                    color: if error.is_some() { Some(Tokens::get().error) } else { warning.map(|_| Tokens::get().warning) },
                    secondary: &minutes,
                    ..Default::default()
                },
            );

            if let Some(resp) = &r.resp {
                let hover = error.map_or_else(
                    || {
                        let tool = op["tool"].as_u64().unwrap_or(0).to_string();
                        let minutes = format!("{:.1}", op["toolpath"]["cut_minutes"].as_f64().unwrap_or(0.0));
                        let summary = tf(
                            "manufacture.op_summary",
                            &[("kind", &kind_name("op_kind", &text(op, "kind"))), ("tool", &tool), ("minutes", &minutes)],
                        );
                        match warning {
                            Some(w) => format!("{summary}\n\n{}", tf("manufacture.toolpath_warning", &[("warning", &engine_text(w))])),
                            None => summary,
                        }
                    },
                    str::to_string,
                );
                resp.clone().on_hover_text(hover);
                // Dragged onto another operation of its setup, it is cut before that one (or after it, from its lower half).
                if resp.drag_started() && !r.on_control {
                    resp.dnd_set_drag_payload(OpDrag { setup: id, op: op_id, at });
                }
                let after = resp.hover_pos().or_else(|| ui.input(|i| i.pointer.hover_pos())).is_some_and(|p| p.y > resp.rect.center().y);
                let mark = if after { crate::browser::DropMark::Below } else { crate::browser::DropMark::Above };
                if let Some(d) = crate::browser::drop_target(ui, resp, |d: &OpDrag| d.setup == id && d.op != op_id, mark) {
                    let to = if d.at < at { at - usize::from(!after) } else { at + usize::from(after) };
                    asked.run.push(("cam.op.move", json!({ "setup": id, "op": d.op, "to": to })));
                }
            }
            if r.eye {
                toggle_hidden(app, op_id);
            } else if r.clicked || r.double {
                // The operation picked is the one the panel shows.
                app.manufacture.picked = Some((id, Some(op_id)));
                (asked.edit_op, asked.from_browser) = (Some((setup.clone(), op.clone())), true);
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

/// An operation dragged in the browser: its setup, its id and where it is in the setup's order.
struct OpDrag {
    setup: u64,
    op: u64,
    at: usize,
}

/// A cutting time for the browser, as "1.5 min".
fn minutes_text(minutes: f64) -> String {
    tf("manufacture.minutes", &[("minutes", &format!("{minutes:.1}"))])
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
    let mut item = crate::context_menu::Item::action("ui.cam", tm(label), icon);
    item.params = Some(json!({ "do": what, "setup": setup, "op": op }));
    item
}

/// The menu of the browser's Manufacture folder: what the 3D view shows of the toolpaths, the
/// stock and the setups, and how toolpaths are drawn (as the Preferences' Manufacture section).
pub fn folder_menu_items(app: &SolveApp) -> Vec<crate::context_menu::Item> {
    use crate::context_menu::Item;
    let cam = &app.manufacture;
    let prefs = &app.preferences;
    let entry = |label: &str, what: &str, index: Option<usize>, on: bool| {
        let mut item = Item::action("ui.cam", tm(label), "");
        item.params = Some(json!({ "do": what, "index": index }));
        if on {
            item.shortcut = "current".into();
        }
        item
    };
    let choices = |label: &str, what: &str, names: &[&str], current: usize| {
        let mut item = Item::action("ui.cam", tm(label), "");
        item.children = names.iter().enumerate().map(|(i, name)| entry(name, what, Some(i), i == current)).collect();
        item
    };
    let mut items = vec![entry("Toolpaths", "paths", None, paths_drawn_by(cam)), entry("Stock and origin", "setups", None, cam.show_setups)];
    if cam.stock.is_some() {
        items.push(entry("Simulated stock", "stock", None, cam.show_stock));
    }
    items.extend([
        Item::sep(),
        choices("Toolpath colour", "colour", &PATH_COLOURS, prefs.cam_path_colour),
        choices("Toolpath width", "width", &PATH_WIDTHS.map(|w| w.0), prefs.cam_path_width),
        choices("Leftover shown from", "leftover", &LEFTOVER_SHOWN.map(|l| l.0), prefs.cam_leftover_shown),
    ]);
    items
}

/// The Manufacture section of the Preferences' Graphics page: how toolpaths are drawn and
/// simulated, and the machine's rapid rate and spindle.
pub fn preferences(app: &mut SolveApp, ui: &mut egui::Ui) {
    let prefs = &mut app.preferences;
    crate::ui_kit::section(ui, "cam_preferences", tm("Manufacture"), true, |ui| {
        let choice = |ui: &mut egui::Ui, id: &str, label: &str, index: &mut usize, names: &[&str]| {
            let names: Vec<&str> = names.iter().map(|n| tm(n)).collect();
            crate::ui_kit::row(ui, tm(label), |ui| crate::ui_kit::dropdown(ui, id, index, &names))
                .on_hover_text(help(label).map(tm).unwrap_or_default());
        };
        choice(ui, "pref_cam_colour", "Toolpath colour", &mut prefs.cam_path_colour, &PATH_COLOURS);
        choice(ui, "pref_cam_width", "Toolpath width", &mut prefs.cam_path_width, &PATH_WIDTHS.map(|w| w.0));
        choice(ui, "pref_cam_detail", "Simulation detail", &mut prefs.cam_sim_detail, &SIM_DETAIL.map(|d| d.0));
        choice(ui, "pref_cam_leftover", "Leftover shown from", &mut prefs.cam_leftover_shown, &LEFTOVER_SHOWN.map(|l| l.0));
        crate::ui_kit::row(ui, tm("Rapid rate"), |ui| crate::ui_kit::number_field(ui, &mut prefs.cam_rapid_rate, 100.0..=100_000.0, 50.0, " mm/min"))
            .on_hover_text(help("Rapid rate").map(tm).unwrap_or_default());
        crate::ui_kit::row(ui, tm("Spindle maximum"), |ui| crate::ui_kit::number_field(ui, &mut prefs.cam_max_rpm, 0.0..=200_000.0, 100.0, " rev/min"))
            .on_hover_text(tm("The fastest this machine's spindle turns: a speed worked out from a tool's surface speed is cut down to it, and its chip load then feeds slower. 0: no limit"));
        crate::ui_kit::row(ui, tm("Spindle spin-up"), |ui| crate::ui_kit::number_field(ui, &mut prefs.cam_spindle_delay, 0.0..=60.0, 0.5, " s"))
            .on_hover_text(tm("How long programs wait after the spindle starts or changes speed, so that it is up to speed before it cuts (a G4 after M3). 0: no wait"));
    });
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
            disable(menu_item("Setup Sheet…", "cam_post", "sheet", setup, None), !ops.is_empty()),
            menu_item("New Setup, Turned Over about X", "cam_setup", "turn", setup, None),
            menu_item("New Setup, Turned Over about Y", "cam_setup", "turn_y", setup, None),
            disable(menu_item("Order by Tool", "", "order", setup, None), ops.len() > 1),
            disable(menu_item("Save as Template…", "", "template_save", setup, None), !ops.is_empty()),
            menu_item("Apply Template…", "", "template_apply", setup, None),
            Item::sep(),
            menu_item("Remove Setup", "delete", "remove", setup, None),
        ],
        Some(op) => {
            let at = ops.iter().position(|o| *o == op);
            vec![
                menu_item("Edit Operation", "edit", "edit", setup, Some(op)),
                menu_item("Simulate", "cam_simulate", "simulate", setup, Some(op)),
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
    let index = p["index"].as_u64().and_then(|i| usize::try_from(i).ok()).unwrap_or(0);
    let (cam, prefs) = (&mut app.manufacture, &mut app.preferences);
    match p["do"].as_str().unwrap_or("") {
        "paths" => {
            let shown = !paths_drawn_by(cam);
            (cam.show_paths, cam.pinned_paths) = (shown, shown);
            return;
        }
        "setups" => return cam.show_setups = !cam.show_setups,
        "stock" => return cam.show_stock = !cam.show_stock,
        "colour" => return prefs.cam_path_colour = index.min(PATH_COLOURS.len() - 1),
        "width" => return prefs.cam_path_width = index.min(PATH_WIDTHS.len() - 1),
        "leftover" => return prefs.cam_leftover_shown = index.min(LEFTOVER_SHOWN.len() - 1),
        _ => {}
    }
    let (Some(setup), op) = (p["setup"].as_u64(), p["op"].as_u64()) else { return };
    let listed = listed(app).0;
    let Some(s) = listed["setups"].as_array().into_iter().flatten().find(|s| s["id"].as_u64() == Some(setup)).cloned() else { return };
    let o = op.and_then(|op| s["operations"].as_array().into_iter().flatten().find(|o| o["id"].as_u64() == Some(op)).cloned());
    let mut asked = Asked::default();
    match (p["do"].as_str().unwrap_or(""), &o) {
        ("edit", None) => asked.form = setup_form(&s),
        ("edit", Some(o)) => asked.edit_op = Some((s.clone(), o.clone())),
        ("simulate", None) => asked.simulate = Some(setup),
        ("simulate", Some(o)) => asked.simulate_op = o["id"].as_u64().map(|op| (setup, op)),
        ("post", None) => asked.post = Some((setup, None, text(&s, "post"))),
        // The part turned over: the tool from the other side, the stock what this setup leaves.
        // The part turned over about the setup's X (its Y reversed) or about its Y (its X reversed).
        (turn @ ("turn" | "turn_y"), None) => {
            let axis: Vec<f64> = s["axis"].as_array().into_iter().flatten().filter_map(Value::as_f64).map(|v| -v).collect();
            let sign = if turn == "turn_y" { -1.0 } else { 1.0 };
            let x_axis: Vec<f64> = s["frame"]["x"].as_array().into_iter().flatten().filter_map(Value::as_f64).map(|v| v * sign).collect();
            // The origin as the command takes it: a point, or where on a box.
            let o = &s["origin"];
            let origin = match o["type"].as_str() {
                Some("point") => o["at"].clone(),
                _ => json!({ "of": o["of"], "x": o["x"], "y": o["y"], "z": o["z"] }),
            };
            // The same block turned over: what was on top is below it now.
            let st = &s["stock"];
            let stock = json!({ "side": st["side"], "top": st["bottom"], "bottom": st["top"] });
            let mut p = json!({
                "axis": axis,
                "x_axis": x_axis,
                "origin": origin,
                "stock": stock,
                "post": s["post"],
                "work_offset": s["work_offset"],
                "stock_from": setup,
            });
            if let Some(material) = s["material"].as_str() {
                p["material"] = json!(material);
            }
            asked.run.push(("cam.setup.create", p));
        }
        ("order", None) => match run(app, "cam.setup.order", json!({ "setup": setup })) {
            Ok(v) => {
                let n = |i: usize| v["tool_changes"][i].as_u64().unwrap_or(0).to_string();
                app.set_status(tf("manufacture.ordered_by_tool", &[("before", &n(0)), ("after", &n(1))]), false);
            }
            Err(e) => app.manufacture.error = Some(e),
        },
        ("template_save", None) => {
            let name: String = text(&s, "name").chars().filter(|c| c.is_ascii_alphanumeric() || "-_ ".contains(*c)).collect();
            app.pick_save(&format!("{name} template.json"), &["json"], move |app, path| {
                if let Some(path) = path {
                    app.manufacture.error = run(app, "cam.template.save", json!({ "setup": setup, "path": path })).err();
                }
            });
        }
        ("template_apply", None) => app.pick_open_for(&["json"], move |app, path| {
            if let Some(path) = path {
                app.manufacture.error = run(app, "cam.template.apply", json!({ "setup": setup, "path": path })).err();
            }
        }),
        ("sheet", None) => {
            let name: String = format!("{} sheet", text(&s, "name")).chars().filter(|c| c.is_ascii_alphanumeric() || "-_ ".contains(*c)).collect();
            app.pick_save(&format!("{name}.md"), &["md"], move |app, path| {
                if let Some(path) = path {
                    app.manufacture.error = run(app, "cam.sheet", json!({ "setup": setup, "path": path })).err();
                }
            });
        }
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
    let _language = crate::i18n::scope(&app.preferences.language);
    let cam = &app.manufacture;
    let air = if cam.air_cut { cam.air_height } else { 0.0 };
    let mut p = json!({
        "setup": setup,
        "path": path,
        "spindle_delay": app.preferences.cam_spindle_delay,
        "optional_stops": cam.optional_stops,
        "air_cut": air,
    });
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
            app.set_status(tf("manufacture.wrote_lines", &[("lines", &v["lines"].as_u64().unwrap_or(0).to_string())]), false);
        }
        Err(e) => app.manufacture.error = Some(e),
    }
}

#[path = "cam_ui_confirm.rs"]
mod confirm;

#[cfg(test)]
#[path = "cam_ui_tests.rs"]
mod tests;
