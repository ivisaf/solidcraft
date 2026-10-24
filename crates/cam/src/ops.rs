//! 2.5D operations: facing, contours, pockets and drilling. Each one turns planar geometry
//! and cutting data into a [`Toolpath`] and checks it with [`verify`] before returning it.

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::check::verify;
use crate::offset::{Shape, reversed};
use crate::path::{MAX_MOVES, Move, Toolpath};
use crate::piece::{Piece, turn};
use crate::sim::HeightField;
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most depth passes of one operation.
const MAX_LEVELS: usize = 10_000;
/// Most offset rings of one pocket.
const MAX_RINGS: usize = 2_000;
/// Most passes across a faced area.
const MAX_ROWS: usize = 100_000;
/// Most holes of one drilling operation.
const MAX_HOLES: usize = 100_000;
/// Most times a ramp goes round its loop; a loop too short for that is plunged into instead.
const MAX_RAMP_LAPS: f64 = 2000.0;
/// Largest coordinate or height (mm).
const MAX_COORD: f64 = 1.0e6;
/// Most tabs on one loop of a contour.
pub const MAX_TABS: u32 = 100;

/// The Z levels of an operation (work coordinates, mm).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Heights {
    /// Where the tool travels between cuts.
    pub clearance: f64,
    /// Where rapid motion towards the material stops and feed begins.
    pub retract: f64,
    /// Top of the material to cut.
    pub top: f64,
    /// Final depth.
    pub bottom: f64,
}

impl Heights {
    pub fn validate(&self) -> Result<()> {
        let all = [self.clearance, self.retract, self.top, self.bottom];
        if !all.iter().all(|z| z.is_finite() && z.abs() <= MAX_COORD) {
            return invalid("the heights must be finite numbers");
        }
        if self.clearance < self.retract || self.retract < self.top || self.top < self.bottom {
            return invalid("the heights must be ordered: clearance ≥ retract ≥ top ≥ bottom");
        }
        Ok(())
    }
}

/// Feed rates (mm/min).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Feeds {
    pub cut: f64,
    pub plunge: f64,
}

impl Feeds {
    pub fn validate(&self) -> Result<()> {
        if [self.cut, self.plunge].iter().all(|f| *f > 0.0 && *f <= 1.0e6) { Ok(()) } else { invalid("the feed rates must be above 0") }
    }
}

/// What every operation needs to cut.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cut {
    pub heights: Heights,
    pub feeds: Feeds,
    /// Most material taken per depth pass (mm).
    pub stepdown: f64,
}

impl Cut {
    fn validate(&self) -> Result<()> {
        self.heights.validate()?;
        self.feeds.validate()?;
        if !(self.stepdown > 0.0) || !self.stepdown.is_finite() {
            return invalid("the stepdown must be above 0");
        }
        Ok(())
    }

    /// Depth passes from the top down to the bottom, evenly spaced and at most a stepdown apart.
    pub fn levels(&self) -> Result<Vec<f64>> {
        let (top, bottom) = (self.heights.top, self.heights.bottom);
        let n = ((top - bottom) / self.stepdown - 1e-9).ceil().max(1.0);
        if !(n <= MAX_LEVELS as f64) {
            return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} depth passes")));
        }
        let n = n as usize;
        Ok((1..=n).map(|k| if k == n { bottom } else { top - (top - bottom) * k as f64 / n as f64 }).collect())
    }
}

/// Which side of a profile the tool runs on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    /// Around the regions: the regions are the part that stays.
    #[default]
    Outside,
    /// Inside the regions' boundary: the regions are the opening.
    Inside,
    /// Tool centre on the boundary.
    On,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ContourParams {
    pub side: Side,
    /// Climb milling (with a clockwise spindle the wall is on the right of the tool's travel);
    /// otherwise conventional.
    pub climb: bool,
    /// Material left on the wall (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent into each depth pass, along the profile (radians); 0 plunges straight down.
    pub ramp_angle: f64,
    /// Tabs left standing on each loop, holding a part that is cut out.
    pub tabs: Option<Tabs>,
    /// Material the depth passes leave on the wall besides `stock_to_leave` (mm), taken off by
    /// one more pass round each loop at the full depth; 0: none.
    pub finish: f64,
    /// Radius of a quarter-circle lead-in and lead-out (mm): each pass goes down beside the wall,
    /// off the part, and arcs onto it and off it again; 0: none (it ramps along the loop).
    #[serde(default)]
    pub lead: f64,
    /// Dogbones: at a sharp inside corner the round tool cannot reach, it runs in along the
    /// corner's bisector until its edge touches the corner, and back.
    #[serde(default)]
    pub dogbones: bool,
    /// An onion skin (mm): the depth passes stop this far above the bottom, so that a part cut
    /// out stays held by a thin floor; `skin_last` takes it off in one light last pass.
    #[serde(default)]
    pub skin: f64,
    #[serde(default)]
    pub skin_last: bool,
}

/// Tabs that hold a part being cut out: `count` on each loop, spaced evenly along it, each
/// `width` long along the part's edge and `height` high over the bottom (mm). Where a depth
/// pass is below a tab's top the tool goes up over it and down again after it.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tabs {
    pub count: u32,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PocketParams {
    /// Distance between passes (mm), at most the tool diameter.
    pub stepover: f64,
    pub climb: bool,
    /// Material left on the walls (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent into uncut material, along the first pass (radians); 0 plunges straight down.
    pub ramp_angle: f64,
    /// Material the depth passes leave on the walls besides `stock_to_leave` (mm), taken off by
    /// one more pass along the walls at the full depth; 0: none.
    pub finish: f64,
    /// Rest machining: only what a larger tool this wide (mm) could not reach (corners, narrow
    /// places) is cut, the larger tool having cleared the rest to the same depth and walls.
    pub rest_diameter: Option<f64>,
    /// Dogbones in the pocket's sharp corners (see [`ContourParams::dogbones`]).
    #[serde(default)]
    pub dogbones: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrillParams {
    /// Depth per peck (mm); `None` drills in one pass.
    pub peck: Option<f64>,
    /// Pause at the bottom (seconds).
    pub dwell: f64,
}

/// How shapes are V-carved with a V-bit: the walls come out at the tool's angle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VCarveParams {
    /// Distance between the rings (mm): smaller carves the walls smoother.
    pub stepover: f64,
    /// Deepest the point goes below the top (mm); none: as deep as the shape asks.
    pub max_depth: Option<f64>,
    pub climb: bool,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
}

/// Most rings a V-carve cuts.
const MAX_VCARVE_RINGS: usize = 5000;

/// V-carves `regions` (letters, a logo) with a V-bit: rings inside the outlines, each where the
/// tool's cone just touches the walls, so that it goes deeper the farther the ring is from them
/// and the walls come out at the tool's angle; down to `max_depth`, below which the rings stay
/// at that depth. Each ring is cut in depth passes no deeper than the stepdown.
pub fn vcarve(regions: &[Region2], tool: &Tool, params: &VCarveParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if tool.kind != crate::tool::ToolKind::VBit {
        return invalid("a V-carve is cut with a V-bit");
    }
    if !(params.stepover > 0.0 && params.stepover <= tool.diameter) {
        return invalid("the stepover must be above 0 and at most the tool diameter");
    }
    if params.max_depth.is_some_and(|d| !(d > 0.0 && d <= MAX_COORD)) {
        return invalid("the deepest a V-carve goes must be above 0");
    }
    if !(cut.stepdown > 0.0) || !cut.stepdown.is_finite() {
        return invalid("the stepdown must be above 0");
    }
    let slope = (tool.point_angle() * 0.5).to_radians().tan();
    let tip = tool.tip() * 0.5;
    let top = cut.heights.top;
    let shape = Shape::from_regions(regions, params.tolerance)?;
    // Down as far as the limit (or the shape) asks: the cut's own bottom is its top edge.
    let heights = Heights { bottom: top - params.max_depth.unwrap_or(MAX_COORD), ..cut.heights };
    let cut = Cut { heights, ..*cut };
    let mut b = Builder::new(&cut);
    for k in 1.. {
        if k > MAX_VCARVE_RINGS {
            return Err(CamError::TooLarge(format!("the V-carve needs more than {MAX_VCARVE_RINGS} rings; use a larger stepover")));
        }
        let away = tip + params.stepover * k as f64;
        let rings = shape.offset(away)?;
        if rings.is_empty() {
            break;
        }
        // Where the cone, its axis `away` from the walls, touches them at the top.
        let depth = ((away - tip) / slope).min(params.max_depth.unwrap_or(f64::INFINITY));
        let passes = ((depth / cut.stepdown) - 1e-9).ceil().max(1.0) as usize;
        for lp in rings {
            // Offset loops keep the inside on their left: the wall is on the right, as climb wants.
            let lp = if params.climb { lp } else { reversed(&lp) };
            let Some(first) = lp.first() else { continue };
            b.travel_to(first.start())?;
            b.plunge_to(top)?;
            for pass in 1..=passes {
                let level = top - depth * pass as f64 / passes as f64;
                b.plunge_to(level)?;
                b.cut_loop(&lp, level)?;
                b.cut_to(first.start(), level)?;
            }
            b.retract()?;
        }
    }
    b.finish()
}

/// How a top edge is chamfered with a pointed tool (a V-bit, a drill): the chamfer's angle is the tool's.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChamferParams {
    /// Outside: the regions' edges round a part; inside: the edges of openings. `On` is not a chamfer.
    pub side: Side,
    pub climb: bool,
    /// How wide the chamfer is, across the top (mm).
    pub width: f64,
    /// How much deeper than the chamfer the tool's point goes, so that it does not cut with its very tip (mm).
    pub tip_offset: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent along the edge into the cut (radians); 0 plunges straight down.
    pub ramp_angle: f64,
}

/// Chamfers the top edges of `regions` (at the cut's top) with a V-bit or a drill in one pass
/// along each loop: its cone cuts the chamfer from `width` in from the edge on the top down to
/// the wall, its point `tip_offset` lower.
pub fn chamfer(regions: &[Region2], tool: &Tool, params: &ChamferParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    check_ramp(params.ramp_angle)?;
    if !matches!(tool.kind, crate::tool::ToolKind::VBit | crate::tool::ToolKind::Drill) {
        return invalid("a chamfer is cut with a pointed tool: a V-bit or a drill");
    }
    if params.side == Side::On {
        return invalid("a chamfer is on the outside or the inside of an edge");
    }
    if !(params.width > 0.0 && params.width <= MAX_COORD) {
        return invalid("the chamfer's width must be above 0");
    }
    if !(params.tip_offset >= 0.0 && params.tip_offset <= MAX_COORD) {
        return invalid("the tip offset must be 0 or more");
    }
    let slope = (tool.point_angle() * 0.5).to_radians().tan();
    // The chamfer's depth, the point's depth, and how far the axis stays off the wall.
    let depth = params.width / slope;
    let point = cut.heights.top - depth - params.tip_offset;
    let off = tool.tip() * 0.5 + params.tip_offset * slope;
    if off + params.width > tool.radius() + 1e-9 {
        return invalid(format!(
            "the {} mm tool is too narrow for a {} mm chamfer with a {} mm tip offset",
            tool.diameter, params.width, params.tip_offset
        ));
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let loops = match params.side {
        Side::Inside => shape.offset(off)?,
        _ => shape.offset(-off)?,
    };
    if loops.is_empty() {
        return invalid("the tool does not fit the profile");
    }
    let heights = Heights { bottom: point.min(cut.heights.top), ..cut.heights };
    let cut = Cut { heights, ..*cut };
    let natural_is_climb = params.side != Side::Outside;
    let mut b = Builder::new(&cut);
    for lp in &loops {
        let lp = if params.climb == natural_is_climb { lp.clone() } else { reversed(lp) };
        let Some(first) = lp.first() else { continue };
        b.travel_to(first.start())?;
        b.plunge_to(heights.top)?;
        b.ramp_loop(&lp, point, params.ramp_angle)?;
        b.cut_loop(&lp, point)?;
        b.cut_to(first.start(), point)?;
        b.retract()?;
    }
    b.finish()
}

/// How a hole larger than the tool is milled on a helix.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BoreParams {
    /// How far the helix goes down in one turn (mm).
    pub pitch: f64,
    /// Climb milling (counter-clockwise inside a hole with a clockwise spindle); otherwise conventional.
    pub climb: bool,
    /// Material left on the wall (mm).
    pub stock_to_leave: f64,
}

/// Collects moves, keeping the tool's position.
pub(crate) struct Builder {
    moves: Vec<Move>,
    pub(crate) pos: Option<Vec3>,
    h: Heights,
    feeds: Feeds,
}

impl Builder {
    pub(crate) fn new(cut: &Cut) -> Builder {
        Builder { moves: Vec::new(), pos: None, h: cut.heights, feeds: cut.feeds }
    }

    pub(crate) fn push(&mut self, m: Move) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        if let Some(to) = m.target() {
            self.pos = Some(to);
        }
        self.moves.push(m);
        Ok(())
    }

    /// How many moves so far.
    pub(crate) fn len(&self) -> usize {
        self.moves.len()
    }

    /// The moves from the `k`-th on.
    pub(crate) fn moves_from(&self, k: usize) -> &[Move] {
        self.moves.get(k..).unwrap_or(&[])
    }

    fn at(&self, p: Vec2) -> bool {
        self.pos.is_some_and(|q| Vec2::new(q.x, q.y).dist(p) <= 1e-9)
    }

    /// Up to the clearance height (if below it), then across to `p`.
    pub(crate) fn travel_to(&mut self, p: Vec2) -> Result<()> {
        if let Some(q) = self.pos {
            if self.at(p) && q.z >= self.h.clearance {
                return Ok(());
            }
            if q.z < self.h.clearance {
                self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) })?;
            }
        }
        self.push(Move::Rapid { to: p.extend(self.h.clearance) })
    }

    /// Straight down to `z`: rapid to the retract height, then at the plunge feed.
    pub(crate) fn plunge_to(&mut self, z: f64) -> Result<()> {
        let Some(q) = self.pos else { return Err(CamError::Unsafe("a plunge before the first positioning move".into())) };
        if q.z > self.h.retract {
            self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.retract) })?;
        }
        if z < self.pos.map_or(z, |p| p.z) {
            self.push(Move::Feed { to: Vec3::new(q.x, q.y, z), feed: self.feeds.plunge })?;
        }
        Ok(())
    }

    pub(crate) fn cut_to(&mut self, p: Vec2, z: f64) -> Result<()> {
        if self.at(p) && self.pos.is_some_and(|q| (q.z - z).abs() <= 1e-9) {
            return Ok(());
        }
        self.push(Move::Feed { to: p.extend(z), feed: self.feeds.cut })
    }

    /// Cuts along `piece` from wherever the tool is (within the offsetting's joining tolerance
    /// of the piece's start), going evenly from depth `z0` to `z1`.
    pub(crate) fn cut_piece(&mut self, piece: &Piece, z0: f64, z1: f64) -> Result<()> {
        match *piece {
            Piece::Line { b, .. } => self.cut_to(b, z1),
            Piece::Arc { c, sweep, .. } => {
                // Halves, so that no arc move comes near a full turn.
                let parts = if sweep.abs() > PI { 2 } else { 1 };
                for k in 1..=parts {
                    let t = k as f64 / parts as f64;
                    let to = piece.point_at(t).extend(z0 + (z1 - z0) * t);
                    self.push(Move::Arc { to, center: c, ccw: sweep > 0.0, feed: self.feeds.cut })?;
                }
                Ok(())
            }
        }
    }

    fn cut_loop(&mut self, lp: &[Piece], z: f64) -> Result<()> {
        lp.iter().try_for_each(|piece| self.cut_piece(piece, z, z))
    }

    /// Goes down to `z` along `lp`, no steeper than `angle`, round as often as that takes; the
    /// tool ends where the loop starts. Straight down when there is no angle or the loop is too
    /// short to ramp along.
    fn ramp_loop(&mut self, lp: &[Piece], z: f64, angle: f64) -> Result<()> {
        let Some(from) = self.pos.map(|p| p.z) else { return Err(CamError::Unsafe("a ramp before the first positioning move".into())) };
        let drop = from - z;
        if !(drop > 1e-9) {
            return Ok(());
        }
        let length: f64 = lp.iter().map(Piece::length).sum();
        let laps = (drop / angle.tan() / length - 1e-9).ceil().max(1.0);
        if !(angle > 0.0) || !(length > 1e-3) || !(laps <= MAX_RAMP_LAPS) {
            return self.plunge_to(z);
        }
        let total = laps * length;
        let mut done = 0.0;
        for _ in 0..laps as usize {
            for piece in lp {
                let z0 = from - drop * done / total;
                done += piece.length();
                self.cut_piece(piece, z0, from - drop * (done / total).min(1.0))?;
            }
        }
        // Exactly at depth, whatever the rounding of the last step.
        match self.pos {
            Some(p) => self.cut_to(Vec2::new(p.x, p.y), z),
            None => Ok(()),
        }
    }

    /// Cuts along `segs` (pieces, each saying whether it runs over a tab) going evenly from
    /// depth `z0` to `z1`, never below `tab_z` over a tab: up onto a tab where the tool is
    /// below its top, and down again after it.
    fn cut_segments(&mut self, segs: &[(Piece, bool)], z0: f64, z1: f64, tab_z: f64) -> Result<()> {
        let total: f64 = segs.iter().map(|(p, _)| p.length()).sum();
        let at = |done: f64| if total > 1e-12 { z0 + (z1 - z0) * (done / total).min(1.0) } else { z1 };
        let mut done = 0.0;
        for (piece, tab) in segs {
            let za = at(done);
            done += piece.length();
            let zb = at(done);
            let (za, zb) = if *tab { (za.max(tab_z), zb.max(tab_z)) } else { (za, zb) };
            if let Some(q) = self.pos {
                if q.z < za - 1e-9 {
                    self.push(Move::Feed { to: Vec3::new(q.x, q.y, za), feed: self.feeds.cut })?;
                } else if q.z > za + 1e-9 {
                    self.push(Move::Feed { to: Vec3::new(q.x, q.y, za), feed: self.feeds.plunge })?;
                }
            }
            self.cut_piece(piece, za, zb)?;
        }
        Ok(())
    }

    /// [`Builder::ramp_loop`] and a whole lap at `z` along a loop split at its tabs, staying over them.
    fn tabbed_lap(&mut self, segs: &[(Piece, bool)], z: f64, angle: f64, tab_z: f64) -> Result<()> {
        let Some(from) = self.pos.map(|p| p.z) else { return Err(CamError::Unsafe("a ramp before the first positioning move".into())) };
        let drop = from - z;
        let length: f64 = segs.iter().map(|(p, _)| p.length()).sum();
        let laps = if drop > 1e-9 { (drop / angle.tan() / length - 1e-9).ceil().max(1.0) } else { 0.0 };
        if drop > 1e-9 && (!(angle > 0.0) || !(length > 1e-3) || !(laps <= MAX_RAMP_LAPS)) {
            // Straight down where the loop starts (never on a tab).
            self.plunge_to(z)?;
        } else {
            for k in 0..laps as usize {
                let (za, zb) = (from - drop * k as f64 / laps, from - drop * (k + 1) as f64 / laps);
                self.cut_segments(segs, za, zb, tab_z)?;
            }
        }
        self.cut_segments(segs, z, z, tab_z)
    }

    pub(crate) fn retract(&mut self) -> Result<()> {
        match self.pos {
            Some(q) if q.z < self.h.clearance => self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) }),
            _ => Ok(()),
        }
    }

    pub(crate) fn finish(mut self) -> Result<Toolpath> {
        self.retract()?;
        let path = Toolpath { moves: self.moves };
        verify(&path, &self.h)?;
        Ok(path)
    }
}

fn check_point(p: Vec2) -> Result<()> {
    if p.is_finite() && p.x.abs() <= MAX_COORD && p.y.abs() <= MAX_COORD { Ok(()) } else { invalid("a coordinate is not a usable number") }
}

fn check_ramp(angle: f64) -> Result<()> {
    if (0.0..=1.5).contains(&angle) { Ok(()) } else { invalid("the ramp angle must be between 0 and 85 degrees") }
}

fn check_leave(leave: f64) -> Result<()> {
    if leave.is_finite() && (0.0..=MAX_COORD).contains(&leave) { Ok(()) } else { invalid("the stock to leave must be 0 or more") }
}

fn check_finish(finish: f64) -> Result<()> {
    if finish.is_finite() && (0.0..=MAX_COORD).contains(&finish) { Ok(()) } else { invalid("the finishing allowance must be 0 or more") }
}

/// Where the tabs are along a loop `length` long, as stretches of the tool's path (from, to):
/// each a tab's width and the tool's diameter long, spaced evenly and none at the loop's
/// start. None on a loop too short to carry them.
fn tab_spans(length: f64, tabs: &Tabs, diameter: f64) -> Vec<(f64, f64)> {
    let span = tabs.width + diameter;
    let n = tabs.count as f64;
    if n * span > 0.8 * length {
        return Vec::new();
    }
    (0..tabs.count).map(|k| length * (k as f64 + 0.5) / n).map(|c| (c - span * 0.5, c + span * 0.5)).collect()
}

/// A loop split where its tabs start and end; each piece says whether it runs over a tab.
fn tab_segments(lp: &[Piece], spans: &[(f64, f64)]) -> Vec<(Piece, bool)> {
    let mut out = Vec::new();
    let mut s0 = 0.0;
    for piece in lp {
        let len = piece.length();
        let s1 = s0 + len;
        // Where along this piece (0..1) a tab starts or ends.
        let mut cuts: Vec<f64> =
            spans.iter().flat_map(|(a, b)| [*a, *b]).filter(|s| *s > s0 + 1e-9 && *s < s1 - 1e-9).map(|s| (s - s0) / len).collect();
        cuts.sort_by(f64::total_cmp);
        let mut t0 = 0.0;
        for t1 in cuts.into_iter().chain([1.0]) {
            let mid = s0 + len * (t0 + t1) * 0.5;
            let on_tab = spans.iter().any(|(a, b)| mid > *a && mid < *b);
            out.push((if t0 == 0.0 && t1 == 1.0 { *piece } else { piece.sub(t0, t1) }, on_tab));
            t0 = t1;
        }
        s0 = s1;
    }
    out
}

/// The loop turned to start at its corner nearest to `p`.
fn start_near(lp: &[Piece], p: Vec2) -> Vec<Piece> {
    let first = lp.iter().enumerate().min_by(|a, b| a.1.start().dist(p).total_cmp(&b.1.start().dist(p))).map_or(0, |(i, _)| i);
    lp.iter().cycle().skip(first).take(lp.len()).copied().collect()
}

/// How a facing operation treats its boundary.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceParams {
    /// Distance between passes (mm), at most the tool diameter.
    pub stepover: f64,
    /// `true`: the tool stays inside the boundary, `margin` away from it (the corners keep the
    /// tool's radius). `false`: everything up to `margin` beyond the boundary is cut.
    pub inside: bool,
    /// How far inside or beyond the boundary (mm, 0 or more).
    pub margin: f64,
}

/// Faces the rectangle `min`–`max` with passes along X. Over the boundary, the area out to the
/// margin is cut whole: passes run from one end of it to the other (the tool's nose a radius
/// past) and the outer passes have the tool's edge on the area's edge. Inside it, the tool
/// never leaves the area.
pub fn face(min: Vec2, max: Vec2, tool: &Tool, params: &FaceParams, cut: &Cut) -> Result<Toolpath> {
    let rows = face_rows(min, max, tool, params, cut)?;
    let mut b = Builder::new(cut);
    for z in cut.levels()? {
        b.travel_to(Vec2::new(rows.x0, rows.y(0)))?;
        b.plunge_to(z)?;
        for k in 0..rows.count {
            let y = rows.y(k);
            let (from, to) = if k % 2 == 0 { (rows.x0, rows.x1) } else { (rows.x1, rows.x0) };
            b.cut_to(Vec2::new(from, y), z)?;
            b.cut_to(Vec2::new(to, y), z)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// Where a facing operation's passes run: along X from `x0` to `x1`, `count` rows from `y0` up.
struct FaceRows {
    x0: f64,
    x1: f64,
    y0: f64,
    height: f64,
    count: usize,
}

impl FaceRows {
    fn y(&self, k: usize) -> f64 {
        self.y0 + self.height * k as f64 / (self.count - 1).max(1) as f64
    }
}

/// [`face`] staying out of a part: `part` holds its top seen from above. On each depth pass the
/// tool's axis keeps out of the columns that rise above the pass within the tool's radius (and
/// a column's width more), lifting over them along a pass and going round them, or over them
/// at the clearance height, between passes. Where the part is nowhere above the bottom, this is
/// [`face`].
pub fn face_around(min: Vec2, max: Vec2, tool: &Tool, params: &FaceParams, cut: &Cut, part: &HeightField) -> Result<Toolpath> {
    let rows = face_rows(min, max, tool, params, cut)?;
    let keep_out = part.dilated(tool.radius() + part.cell)?;
    const ABOVE: f64 = 1e-4;
    let bottom = cut.heights.bottom;
    // Seen along the passes' area: does the part rise above the bottom anywhere?
    let lo = Vec2::new(rows.x0.min(rows.x1), rows.y0);
    let hi = Vec2::new(rows.x0.max(rows.x1), rows.y0 + rows.height);
    let (is, js) = keep_out.range(lo, hi);
    let intrudes = js.into_iter().any(|j| is.clone().any(|i| keep_out.get(i, j).is_some_and(|z| z > bottom + ABOVE)));
    if !intrudes {
        return face(min, max, tool, params, cut);
    }
    let step = (part.cell * 0.5).max(1e-3);
    let clear = |p: Vec2, z: f64| keep_out.at(p) <= z + ABOVE;
    let line_clear = |a: Vec2, b: Vec2, z: f64| {
        let n = ((a.dist(b) / step).ceil()).clamp(1.0, 1.0e5) as usize;
        (0..=n).all(|k| clear(a.lerp(b, k as f64 / n as f64), z))
    };
    let span = rows.x1 - rows.x0;
    let samples = ((span.abs() / step).ceil()).clamp(1.0, 1.0e5) as usize;
    let mut b = Builder::new(cut);
    for z in cut.levels()? {
        let mut down = false;
        for k in 0..rows.count {
            let y = rows.y(k);
            // The stretches of this pass the tool may cut, in the pass's direction.
            let mut stretches: Vec<(f64, f64)> = Vec::new();
            let mut open: Option<f64> = None;
            for n in 0..=samples {
                let x = rows.x0 + span * n as f64 / samples as f64;
                match (clear(Vec2::new(x, y), z), open) {
                    (true, None) => open = Some(x),
                    (false, Some(start)) => {
                        // Back to the last clear sample.
                        let end = x - span / samples as f64;
                        if (end - start) * span.signum() >= 0.0 {
                            stretches.push((start, end));
                        }
                        open = None;
                    }
                    _ => {}
                }
            }
            if let Some(start) = open {
                stretches.push((start, rows.x1));
            }
            if k % 2 == 1 {
                stretches = stretches.into_iter().rev().map(|(start, end)| (end, start)).collect();
            }
            for (from, to) in stretches {
                let (from, to) = (Vec2::new(from, y), Vec2::new(to, y));
                let joined = down && b.pos.is_some_and(|q| line_clear(Vec2::new(q.x, q.y), from, z));
                if !joined {
                    b.retract()?;
                    b.travel_to(from)?;
                    b.plunge_to(z)?;
                }
                b.cut_to(from, z)?;
                b.cut_to(to, z)?;
                down = true;
            }
        }
        b.retract()?;
    }
    b.finish()
}

fn face_rows(min: Vec2, max: Vec2, tool: &Tool, params: &FaceParams, cut: &Cut) -> Result<FaceRows> {
    tool.validate()?;
    cut.validate()?;
    check_point(min)?;
    check_point(max)?;
    if !(max.x > min.x) || !(max.y > min.y) {
        return invalid("the area to face is empty");
    }
    if !(params.stepover > 0.0) || !(params.stepover <= tool.diameter) {
        return invalid("the stepover must be above 0 and at most the tool diameter");
    }
    if !(0.0..=MAX_COORD).contains(&params.margin) {
        return invalid("the distance inside or beyond the boundary must be 0 or more");
    }
    let r = tool.radius();
    let grow = if params.inside { -params.margin } else { params.margin };
    let (lo, hi) = (Vec2::new(min.x - grow, min.y - grow), Vec2::new(max.x + grow, max.y + grow));
    // Where the tool's centre goes: along the passes, and from the first pass to the last.
    let (x0, x1) = if params.inside { (lo.x + r, hi.x - r) } else { (lo.x, hi.x) };
    let (mut y0, mut y1) = (lo.y + r, hi.y - r);
    if params.inside && (x0 > x1 || y0 > y1) {
        return invalid("the tool does not fit inside the boundary");
    }
    if y0 > y1 {
        // An area narrower than the tool: one pass down its middle.
        (y0, y1) = ((lo.y + hi.y) * 0.5, (lo.y + hi.y) * 0.5);
    }
    let height = y1 - y0;
    let rows = if height > 1e-9 { (height / params.stepover - 1e-9).ceil().max(1.0) + 1.0 } else { 1.0 };
    if !(rows <= MAX_ROWS as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_ROWS} passes")));
    }
    Ok(FaceRows { x0, x1, y0, height, count: rows as usize })
}

/// Follows the boundaries of `regions` on the chosen side, one loop after the other, each
/// down through every depth pass before the next.
pub fn contour(regions: &[Region2], tool: &Tool, params: &ContourParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_leave(params.stock_to_leave)?;
    check_finish(params.finish)?;
    check_ramp(params.ramp_angle)?;
    if params.side == Side::On && params.finish > 0.0 {
        return invalid("a contour on the line has no finishing pass: it leaves nothing on a wall");
    }
    let depth = cut.heights.top - cut.heights.bottom;
    if !(params.lead.is_finite() && (0.0..=10.0 * tool.diameter).contains(&params.lead)) {
        return invalid("the lead-in radius must be 0 to ten tool diameters");
    }
    if !(params.skin.is_finite() && params.skin >= 0.0 && params.skin < depth) {
        return invalid("the onion skin must be 0 or more and thinner than the depth cut");
    }
    if let Some(t) = &params.tabs {
        if !(1..=MAX_TABS).contains(&t.count) {
            return invalid(format!("a contour has 1 to {MAX_TABS} tabs on each loop"));
        }
        if !(t.width > 0.0 && t.width <= MAX_COORD) {
            return invalid("the tabs' width must be above 0");
        }
        if !(t.height > 0.0 && t.height < depth) {
            return invalid("the tabs must be above 0 high and lower than the depth cut");
        }
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let loops_at = |leave: f64| -> Result<Vec<Vec<Piece>>> {
        let away = tool.radius() + leave;
        Ok(match params.side {
            Side::Outside => shape.offset(-away)?,
            Side::Inside => shape.offset(away)?,
            Side::On => shape.loops().to_vec(),
        })
    };
    // The depth passes leave the finishing allowance; one pass more at full depth takes it off.
    let dogbones = |loops: Vec<Vec<Piece>>| if params.dogbones { loops.iter().map(|lp| dogboned(lp, tool.radius())).collect() } else { loops };
    let loops = dogbones(loops_at(params.stock_to_leave + params.finish)?);
    if loops.is_empty() {
        return invalid("the tool does not fit the profile");
    }
    let finish = if params.finish > 0.0 { dogbones(loops_at(params.stock_to_leave)?) } else { Vec::new() };
    // Loops come with the regions on their left. Around a part that puts the wall on the left
    // (conventional), inside an opening on the right (climb).
    let natural_is_climb = params.side != Side::Outside;
    // An onion skin: the depth passes stop above the bottom; one light pass takes the skin off when asked.
    let skinned = Cut { heights: Heights { bottom: cut.heights.bottom + params.skin, ..cut.heights }, ..*cut };
    let mut levels = skinned.levels()?;
    if params.skin > 0.0 && params.skin_last {
        levels.push(cut.heights.bottom);
    }
    let floor = if params.skin > 0.0 && !params.skin_last { skinned.heights.bottom } else { cut.heights.bottom };
    let tab_z = params.tabs.map_or(f64::NEG_INFINITY, |t| cut.heights.bottom + t.height);
    let mut b = Builder::new(cut);
    let passes = loops.iter().map(|lp| (lp, levels.as_slice())).chain(finish.iter().map(|lp| (lp, std::slice::from_ref(&floor))));
    for (lp, levels) in passes {
        let lp = if params.climb == natural_is_climb { lp.clone() } else { reversed(lp) };
        let Some(first) = lp.first() else { continue };
        let segs = match &params.tabs {
            Some(t) => tab_segments(&lp, &tab_spans(lp.iter().map(Piece::length).sum(), t, tool.diameter)),
            None => Vec::new(),
        };
        if params.lead > 0.0 {
            lead_laps(&mut b, &shape, &lp, levels, params.lead, (&segs, tab_z))?;
            continue;
        }
        b.travel_to(first.start())?;
        // Down to the last pass (through what it cut), then into the material along the loop.
        b.plunge_to(cut.heights.top)?;
        for &z in levels {
            if z < tab_z - 1e-9 && segs.iter().any(|(_, tab)| *tab) {
                b.tabbed_lap(&segs, z, params.ramp_angle, tab_z)?;
            } else {
                b.ramp_loop(&lp, z, params.ramp_angle)?;
                b.cut_loop(&lp, z)?;
            }
            b.cut_to(first.start(), z)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// Smallest change of direction (radians) a dogbone is put into.
const DOGBONE_TURN: f64 = 0.1;

/// `lp` with a dogbone at each sharp corner: from the corner, along the bisector away from the
/// turn, until the edge of a tool `r` round reaches the corner the wall makes there, and back.
/// Offset loops have sharp corners only where the material is on the outside of the turn.
pub(crate) fn dogboned(lp: &[Piece], r: f64) -> Vec<Piece> {
    let mut out = Vec::with_capacity(lp.len() * 3);
    for (piece, next) in lp.iter().zip(lp.iter().cycle().skip(1)) {
        out.push(*piece);
        let (v, t1, t2) = (piece.end(), piece.tangent_at(1.0), next.tangent_at(0.0));
        let turn_by = turn(t1, t2);
        if (next.start() - v).len() > 1e-6 || turn_by.abs() < DOGBONE_TURN || turn_by.abs() > PI - DOGBONE_TURN {
            continue;
        }
        let Some(away) = (t1 - t2).normalized() else { continue };
        let reach = r * (1.0 / (turn_by / 2.0).cos() - 1.0);
        let tip = v + away * reach;
        out.push(Piece::Line { a: v, b: tip });
        out.push(Piece::Line { a: tip, b: v });
    }
    out
}

/// Each depth pass of a contour loop with a lead-in and lead-out: down beside the wall where
/// the tool is off the part (the side farther from the profile), a quarter circle `radius`
/// round onto the loop's start, round the loop, a quarter circle off it, and straight back to
/// where it went down. Below the tabs' top (`tabs`: the loop split at them, and that height)
/// the lap goes up over each tab; the loop never starts on one.
fn lead_laps(b: &mut Builder, shape: &Shape, lp: &[Piece], levels: &[f64], radius: f64, tabs: (&[(Piece, bool)], f64)) -> Result<()> {
    let (segs, tab_z) = tabs;
    let Some(first) = lp.first() else { return Ok(()) };
    let (p0, t) = (first.start(), first.tangent_at(0.0));
    let left = Vec2::new(-t.y, t.x);
    // Off the part: the side whose point is farther from the profile.
    let away = if shape.dist(p0 + left * radius) >= shape.dist(p0 - left * radius) { left } else { -left };
    let c = p0 + away * radius;
    let (s, e) = (c - t * radius, c + t * radius);
    let angle = |p: Vec2| (p.y - c.y).atan2(p.x - c.x);
    let into = Piece::Arc { c, r: radius, a0: angle(s), sweep: turn(s - c, p0 - c) };
    let out_of = Piece::Arc { c, r: radius, a0: angle(p0), sweep: turn(p0 - c, e - c) };
    b.travel_to(s)?;
    for &z in levels {
        b.plunge_to(z)?;
        b.cut_piece(&into, z, z)?;
        if z < tab_z - 1e-9 && segs.iter().any(|(_, tab)| *tab) {
            b.cut_segments(segs, z, z, tab_z)?;
        } else {
            b.cut_loop(lp, z)?;
        }
        b.cut_piece(&out_of, z, z)?;
        b.cut_to(s, z)?;
    }
    b.retract()
}

/// Clears `regions` (holes are islands) with rings parallel to the boundary, from the inside out.
pub fn pocket(regions: &[Region2], tool: &Tool, params: &PocketParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_leave(params.stock_to_leave)?;
    check_finish(params.finish)?;
    check_ramp(params.ramp_angle)?;
    let r = tool.radius();
    if !(params.stepover > 0.0) || !(params.stepover <= tool.diameter) {
        return invalid("the stepover must be above 0 and at most the tool diameter");
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    // The depth passes leave the finishing allowance on the walls; a last pass along them takes it off.
    let wall = r + params.stock_to_leave + params.finish;
    let mut rings: Vec<Vec<Vec<Piece>>> = Vec::new();
    loop {
        let ring = shape.offset(wall + params.stepover * rings.len() as f64)?;
        if ring.is_empty() {
            break;
        }
        rings.push(ring);
        if rings.len() > MAX_RINGS {
            return Err(CamError::TooLarge(format!("the pocket needs more than {MAX_RINGS} passes; use a larger stepover")));
        }
    }
    if rings.is_empty() {
        return invalid("the tool does not fit the pocket");
    }
    if let Some(big) = params.rest_diameter {
        return rest(&shape, &rings, tool, params, cut, big);
    }
    // Dogbones in the wall ring's corners.
    if params.dogbones
        && let Some(wall_ring) = rings.first_mut()
    {
        *wall_ring = wall_ring.iter().map(|lp| dogboned(lp, r)).collect();
    }
    // Rings more than a radius apart leave material wherever the next ring in does not run
    // within a radius: the middle of a part of the pocket it no longer fits in, a passage that
    // narrows. A ring a radius further in clears it; it is cut where some stretch of it is out
    // of the next ring's reach (corners between rings are cleared by the spurs below).
    if params.stepover > r + 1e-9 {
        let mut passes = Vec::new();
        for (k, ring) in rings.iter().enumerate() {
            passes.push(ring.clone());
            let next: Vec<&Piece> = rings.get(k + 1).into_iter().flatten().flatten().collect();
            let reached = |p: Vec2| next.iter().any(|piece| piece.dist(p) <= r + 1e-6);
            let extra: Vec<Vec<Piece>> = shape
                .offset(wall + params.stepover * k as f64 + r)?
                .into_iter()
                .filter(|lp| lp.iter().any(|piece| [0.25, 0.5, 0.75].iter().any(|t| !reached(piece.point_at(*t)))))
                .collect();
            if !extra.is_empty() {
                passes.push(extra);
            }
        }
        rings = passes;
    }
    let mut b = Builder::new(cut);
    // The depth already cut wherever this pass's rings run.
    let mut cleared = cut.heights.top;
    for z in cut.levels()? {
        // Whether the tool is down in this level's cut.
        let mut down: Option<Vec2> = None;
        for lp in rings.iter().rev().flatten() {
            // Offset loops keep the pocket on their left, so the wall is on the right: climb.
            let lp = if params.climb { lp.clone() } else { reversed(lp) };
            let lp = start_near(&lp, down.unwrap_or_default());
            let Some(first) = lp.first() else { continue };
            let start = first.start();
            match down {
                Some(from) if link_is_clear(&shape, from, start, wall, r) => b.cut_to(start, z)?,
                _ => {
                    b.travel_to(start)?;
                    b.plunge_to(cleared)?;
                    b.ramp_loop(&lp, z, params.ramp_angle)?;
                }
            }
            // Rings far apart do not reach into a sharp corner between them: there the tool
            // steps towards the next ring's corner and back.
            let inward = if params.climb { 1.0 } else { -1.0 };
            for (piece, next) in lp.iter().zip(lp.iter().cycle().skip(1)) {
                b.cut_piece(piece, z, z)?;
                if let Some(to) = corner_spur(piece, next, inward, params.stepover, r).filter(|to| link_is_clear(&shape, piece.end(), *to, wall, r)) {
                    b.cut_to(to, z)?;
                    b.cut_to(piece.end(), z)?;
                }
            }
            b.cut_to(start, z)?;
            down = Some(start);
        }
        cleared = z;
        b.retract()?;
    }
    if params.finish > 0.0 {
        let z = cut.heights.bottom;
        for lp in shape.offset(r + params.stock_to_leave)? {
            let lp = if params.dogbones { dogboned(&lp, r) } else { lp };
            let lp = if params.climb { lp } else { reversed(&lp) };
            let Some(first) = lp.first() else { continue };
            // Down where the depth passes cleared, all but the allowance itself.
            b.travel_to(first.start())?;
            b.plunge_to(z)?;
            b.cut_loop(&lp, z)?;
            b.cut_to(first.start(), z)?;
            b.retract()?;
        }
    }
    b.finish()
}

/// Most cells of the grid rest machining finds the material left on.
const MAX_REST_CELLS: usize = 2_000_000;

/// [`pocket`]'s rings, cut only where they come within reach of what a tool `big` wide left:
/// the pocket's points that no position of the larger tool (as far from the walls as this
/// pocket's) came within its radius of. The stretches of the rings near such material are cut
/// on each depth pass; the tool goes up and over between them.
fn rest(shape: &Shape, rings: &[Vec<Vec<Piece>>], tool: &Tool, params: &PocketParams, cut: &Cut, big: f64) -> Result<Toolpath> {
    let (r, leave) = (tool.radius(), params.stock_to_leave + params.finish);
    if !(big > tool.diameter + 1e-9) || !(big <= MAX_COORD) {
        return invalid("rest machining follows a larger tool: its diameter must be above this tool's");
    }
    let big_r = big * 0.5;
    // The pocket on a grid a quarter of this tool's radius fine.
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(Piece::bounds) {
        (lo, hi) = (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)));
    }
    let cell = (r * 0.25).max(0.01);
    if !lo.is_finite() || !hi.is_finite() || !(((hi.x - lo.x) / cell + 1.0) * ((hi.y - lo.y) / cell + 1.0) <= MAX_REST_CELLS as f64) {
        return Err(CamError::TooLarge("the pocket is too large for rest machining with so small a tool".into()));
    }
    let flat = |z: f64| HeightField::block(lo.extend(z - 1.0), hi.extend(z), cell);
    let (mut centres, mut inside) = (flat(0.0)?, flat(0.0)?);
    for j in 0..centres.ny {
        for i in 0..centres.nx {
            let q = centres.centre(i, j);
            let d = shape.dist(q);
            let k = j * centres.nx + i;
            let is_in = shape.inside(q, d.max(1e-6));
            // Where the larger tool's axis could be, and where the pocket's material is (not what stays on the walls).
            if let Some(z) = centres.z.get_mut(k) {
                *z = if is_in && d >= big_r + leave - 1e-9 { 1.0 } else { 0.0 };
            }
            if let Some(z) = inside.z.get_mut(k) {
                *z = if is_in && d > leave + cell * 0.5 { 1.0 } else { 0.0 };
            }
        }
    }
    // A column more: the grid's own rounding is no material left (a sliver thinner than a column is not cut again).
    let reached = centres.dilated(big_r + cell)?;
    let mut left = inside.clone();
    for (z, reach) in left.z.iter_mut().zip(&reached.z) {
        if *reach > 0.5 {
            *z = 0.0;
        }
    }
    if left.z.iter().all(|z| *z < 0.5) {
        return invalid(format!("the {big} mm tool leaves nothing for this tool: it reached the whole pocket"));
    }
    // Where this tool's axis touches what was left (a column more, so that no corner is missed).
    let near = left.dilated(r + cell)?;
    let touches = |p: Vec2| near.at(p) > 0.5;
    // Each ring's stretches near what was left, joined while they follow on; a ring near it all
    // the way round is one whole loop (ramped into: nothing of it was cleared).
    let mut stretches: Vec<Vec<Piece>> = Vec::new();
    let mut whole: Vec<Vec<Piece>> = Vec::new();
    for lp in rings.iter().rev().flatten() {
        let lp = if params.climb { lp.clone() } else { reversed(lp) };
        // Started where it is clear of what was left, so that no stretch begins in it.
        let clear = lp.iter().enumerate().find_map(|(i, piece)| {
            let n = ((piece.length() / (cell * 0.5)).ceil() as usize).clamp(1, 100_000);
            (0..n).map(|k| k as f64 / n as f64).find(|t| !touches(piece.point_at(*t))).map(|t| (i, t))
        });
        let Some((i, t)) = clear else {
            whole.push(lp);
            continue;
        };
        let mut turned: Vec<Piece> = Vec::with_capacity(lp.len() + 1);
        if let Some(piece) = lp.get(i) {
            turned.push(if t > 0.0 { piece.sub(t, 1.0) } else { *piece });
        }
        turned.extend(lp.iter().cycle().skip(i + 1).take(lp.len().saturating_sub(1)).copied());
        if let Some(piece) = lp.get(i).filter(|_| t > 0.0) {
            turned.push(piece.sub(0.0, t));
        }
        let lp = turned;
        let mut open: Vec<Piece> = Vec::new();
        for piece in &lp {
            let n = ((piece.length() / (cell * 0.5)).ceil() as usize).clamp(1, 100_000);
            let mut from: Option<usize> = None;
            for k in 0..=n {
                let on = touches(piece.point_at(k as f64 / n as f64));
                match (on, from) {
                    (true, None) => from = Some(k),
                    (false, Some(f)) => {
                        open.push(piece.sub(f as f64 / n as f64, k as f64 / n as f64));
                        stretches.push(std::mem::take(&mut open));
                        from = None;
                    }
                    _ => {}
                }
            }
            if let Some(f) = from {
                open.push(if f == 0 { *piece } else { piece.sub(f as f64 / n as f64, 1.0) });
            }
        }
        if !open.is_empty() {
            stretches.push(open);
        }
    }
    stretches.retain(|s| s.iter().map(Piece::length).sum::<f64>() > 1e-6);
    // Each one after the one nearest the last's end.
    let mut ordered: Vec<Vec<Piece>> = Vec::with_capacity(stretches.len());
    let mut at = stretches.first().and_then(|s| s.first()).map_or(Vec2::ZERO, Piece::start);
    while !stretches.is_empty() {
        let next = stretches
            .iter()
            .enumerate()
            .min_by(|a, b| {
                let d = |s: &Vec<Piece>| s.first().map_or(f64::INFINITY, |p| p.start().dist(at));
                d(a.1).total_cmp(&d(b.1))
            })
            .map_or(0, |(i, _)| i);
        let s = stretches.swap_remove(next);
        at = s.last().map_or(at, Piece::end);
        ordered.push(s);
    }
    let stretches = ordered;
    let mut b = Builder::new(cut);
    let mut cleared = cut.heights.top;
    for z in cut.levels()? {
        for lp in &whole {
            let Some(first) = lp.first() else { continue };
            b.retract()?;
            b.travel_to(first.start())?;
            b.plunge_to(cleared)?;
            b.ramp_loop(lp, z, params.ramp_angle)?;
            b.cut_loop(lp, z)?;
            b.cut_to(first.start(), z)?;
        }
        cleared = z;
        for stretch in &stretches {
            let Some(first) = stretch.first() else { continue };
            // Within the pocket, over at the retract height (the tool kept inside its walls); else by the clearance height.
            match b.pos {
                Some(q) if link_is_clear(shape, Vec2::new(q.x, q.y), first.start(), r, r) => {
                    if q.z < cut.heights.retract {
                        b.push(Move::Rapid { to: Vec3::new(q.x, q.y, cut.heights.retract) })?;
                    }
                    b.push(Move::Rapid { to: first.start().extend(cut.heights.retract) })?;
                }
                _ => {
                    b.retract()?;
                    b.travel_to(first.start())?;
                }
            }
            // Down where the larger tool cleared: the tool's edge only touches what is left.
            b.plunge_to(z)?;
            for piece in stretch {
                b.cut_to(piece.start(), z)?;
                b.cut_piece(piece, z, z)?;
            }
        }
        b.retract()?;
    }
    b.finish()
}

/// Where to step to from the corner between `piece` and `next` so that the corner between
/// this ring and the next one in (`gap` away) is cleared: along the corner's bisector, to within
/// a radius of the next ring's corner. `inward`: 1 with the pocket on the loop's left, −1 on
/// its right. `None` where the rings' own passes already clear the corner.
fn corner_spur(piece: &Piece, next: &Piece, inward: f64, gap: f64, r: f64) -> Option<Vec2> {
    let (t_in, t_out) = (piece.tangent_at(1.0), next.tangent_at(0.0));
    let bend = crate::piece::turn(t_in, t_out);
    // Half the corner's inside angle has this sine.
    let sharp = (bend.abs() * 0.5).cos();
    if bend * inward <= 1e-6 || sharp < 0.05 || gap <= r * (1.0 + sharp) + 1e-9 {
        return None;
    }
    let dir = (t_out - t_in).normalized()?;
    Some(piece.end() + dir * (gap / sharp - r))
}

/// Whether the tool can feed straight from `a` to `b` without touching a wall.
fn link_is_clear(shape: &Shape, a: Vec2, b: Vec2, wall: f64, r: f64) -> bool {
    let n = ((a.dist(b) / (r * 0.25).max(1e-3)).ceil()).clamp(1.0, 2000.0) as usize;
    (0..=n).all(|i| shape.clear(a.lerp(b, i as f64 / n as f64), wall))
}

/// Drills at each point from the top to the bottom height, optionally in pecks that lift the
/// drill to the retract height to clear chips.
/// Mills round holes (centre, diameter) larger than the tool on a helix down the wall, one turn
/// for each `pitch` of depth, then a whole turn at the bottom for a clean wall and floor. A hole
/// more than twice the tool's diameter keeps a core in its middle: a through hole drops it, a
/// blind one needs a pocket.
pub fn bore(holes: &[(Vec2, f64)], tool: &Tool, params: &BoreParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    check_leave(params.stock_to_leave)?;
    if holes.is_empty() {
        return invalid("there is no hole to bore");
    }
    if holes.len() > MAX_HOLES {
        return Err(CamError::TooLarge(format!("more than {MAX_HOLES} holes")));
    }
    if !(params.pitch > 0.0) || !params.pitch.is_finite() {
        return invalid("the depth per turn must be above 0");
    }
    let h = cut.heights;
    if !(h.top > h.bottom) {
        return invalid("the hole has no depth");
    }
    let turns = ((h.top - h.bottom) / params.pitch - 1e-9).ceil().max(1.0);
    if !(turns <= MAX_LEVELS as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} turns")));
    }
    let turns = turns as usize;
    let mut b = Builder::new(cut);
    for &(c, d) in holes {
        check_point(c)?;
        // Where the tool's axis goes round: the hole's radius less the tool's and what stays on the wall.
        let r = d * 0.5 - tool.radius() - params.stock_to_leave;
        if !(r > 1e-3) || !d.is_finite() {
            return invalid(format!("a {} mm tool cannot bore a hole {} mm across: it needs a larger hole or a drill", tool.diameter, d));
        }
        let start = c + Vec2::new(r, 0.0);
        b.travel_to(start)?;
        b.plunge_to(h.top)?;
        // Half turns, so that no arc move comes near a full turn; the last turn stays at the bottom.
        let half = |k: usize| if k.is_multiple_of(2) { start } else { c - Vec2::new(r, 0.0) };
        let drop = (h.top - h.bottom) / turns as f64;
        for k in 1..=2 * (turns + 1) {
            let z = (h.top - drop * 0.5 * k as f64).max(h.bottom);
            b.push(Move::Arc { to: half(k).extend(z), center: c, ccw: params.climb, feed: cut.feeds.cut })?;
        }
        // Away from the wall before going up, where the middle is cut.
        if r <= tool.radius() {
            b.cut_to(c, h.bottom)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// How a thread is milled round a hole or a boss with a single-profile thread mill.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThreadParams {
    /// The thread's pitch (mm): how far the helix rises in one turn.
    pub pitch: f64,
    /// The thread's major (nominal) diameter (mm).
    pub diameter: f64,
    /// Inside a hole (a nut's thread); otherwise round a boss (a bolt's).
    pub internal: bool,
    /// Climb milling; otherwise conventional.
    pub climb: bool,
    /// Passes out to the full thread depth, evenly apart.
    pub passes: u32,
    /// A left-hand thread (it rises turning clockwise); otherwise right-hand.
    #[serde(default)]
    pub left_hand: bool,
}

/// Most radial passes of a thread.
const MAX_THREAD_PASSES: u32 = 20;

/// Mills a thread at each centre: in passes out to the thread's full depth (ISO metric: the
/// minor diameter 1.0825 pitches less than the major inside a hole, 1.2269 round a boss), each
/// on a helix one pitch a turn, in from the middle (inside) or from outside the boss and away
/// again before it goes up. The helix turns the way climb or conventional milling wants: a
/// right-hand thread is cut from the bottom up, a left-hand one from the top down.
pub fn thread_mill(centres: &[Vec2], tool: &Tool, params: &ThreadParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if centres.is_empty() {
        return invalid("there is no hole or boss to thread");
    }
    if centres.len() > MAX_HOLES {
        return Err(CamError::TooLarge(format!("more than {MAX_HOLES} threads")));
    }
    if !(params.pitch > 0.0 && params.pitch <= tool.diameter) {
        return invalid("the thread's pitch must be above 0 and at most the tool diameter");
    }
    if !(params.diameter > 0.0 && params.diameter <= MAX_COORD) {
        return invalid("the thread's diameter must be above 0");
    }
    if !(1..=MAX_THREAD_PASSES).contains(&params.passes) {
        return invalid(format!("a thread is milled in 1 to {MAX_THREAD_PASSES} passes"));
    }
    let h = cut.heights;
    if !(h.top > h.bottom) {
        return invalid("the thread has no length");
    }
    let turns = ((h.top - h.bottom) / params.pitch).ceil();
    if !(turns <= MAX_LEVELS as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} turns")));
    }
    let r = tool.radius();
    let major = params.diameter * 0.5;
    // The tool's axis: where each pass runs, out to the full depth.
    let (first, last) = if params.internal {
        let minor = major - 1.0825 * params.pitch * 0.5;
        (minor - r, major - r)
    } else {
        let minor = major - 1.2269 * params.pitch * 0.5;
        (major + r, minor + r)
    };
    if params.internal && !(first > 1e-3) {
        return invalid(format!("a {} mm thread mill does not fit a {} mm thread", tool.diameter, params.diameter));
    }
    // Going up, a right-hand thread turns counter-clockwise seen from above; climb inside a hole is counter-clockwise too.
    let ccw = params.climb == params.internal;
    let half_turns = (turns as usize) * 2;
    let mut b = Builder::new(cut);
    for &c in centres {
        check_point(c)?;
        for pass in 1..=params.passes {
            let at = first + (last - first) * f64::from(pass) / f64::from(params.passes);
            // In and out where the tool is free: the hole's middle, or beyond the boss.
            let free = if params.internal { c } else { c + Vec2::new(at + r + 1.0, 0.0) };
            let start = c + Vec2::new(at, 0.0);
            // Turning the same way, a right-hand thread rises and a left-hand one falls.
            let (from, by) = if params.left_hand { (h.top, -params.pitch) } else { (h.bottom, params.pitch) };
            b.travel_to(free)?;
            b.plunge_to(from)?;
            b.cut_to(start, from)?;
            for k in 1..=half_turns {
                let z = (from + by * 0.5 * k as f64).clamp(h.bottom, h.top);
                let to = if k % 2 == 0 { start } else { c - Vec2::new(at, 0.0) };
                b.push(Move::Arc { to: to.extend(z), center: c, ccw, feed: cut.feeds.cut })?;
            }
            let end = b.pos.map_or(start, |p| Vec2::new(p.x, p.y));
            let away = if params.internal { c } else { c + (end - c) * ((at + r + 1.0) / at.max(1e-9)) };
            let top = b.pos.map_or(h.top, |p| p.z);
            b.cut_to(away, top)?;
            b.retract()?;
        }
    }
    b.finish()
}

pub fn drill(points: &[Vec2], tool: &Tool, params: &DrillParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if points.is_empty() {
        return invalid("there is no point to drill");
    }
    if points.len() > MAX_HOLES {
        return Err(CamError::TooLarge(format!("more than {MAX_HOLES} holes")));
    }
    if !(params.dwell >= 0.0) || !(params.dwell <= 3600.0) {
        return invalid("the dwell must be between 0 and 3600 seconds");
    }
    let h = cut.heights;
    if !(h.top > h.bottom) {
        return invalid("the hole has no depth");
    }
    let pecks = match params.peck {
        None => vec![h.bottom],
        Some(peck) => {
            if !(peck > 0.0) || !peck.is_finite() {
                return invalid("the peck depth must be above 0");
            }
            Cut { stepdown: peck, ..*cut }.levels()?
        }
    };
    // How far above the last peck's depth the rapid back into the hole stops.
    let back_off = params.peck.map_or(0.0, |p| (p * 0.5).min(0.5));
    let mut b = Builder::new(cut);
    for &p in points {
        check_point(p)?;
        b.travel_to(p)?;
        b.push(Move::Rapid { to: p.extend(h.retract) })?;
        let mut last: Option<f64> = None;
        for &z in &pecks {
            if let Some(prev) = last {
                b.push(Move::Rapid { to: p.extend(h.retract) })?;
                b.push(Move::Rapid { to: p.extend(prev + back_off) })?;
            }
            b.push(Move::Feed { to: p.extend(z), feed: cut.feeds.plunge })?;
            last = Some(z);
        }
        if params.dwell > 0.0 {
            b.push(Move::Dwell { seconds: params.dwell })?;
        }
        b.retract()?;
    }
    b.finish()
}
