//! 2.5D operations: facing, contours, pockets and drilling. Each one turns planar geometry
//! and cutting data into a [`Toolpath`] and checks it with [`verify`] before returning it.

use std::f64::consts::PI;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::check::verify;
use crate::offset::{Shape, reversed};
use crate::path::{MAX_MOVES, Move, Toolpath};
use crate::piece::Piece;
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
/// Largest coordinate or height (mm).
const MAX_COORD: f64 = 1.0e6;

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
    fn levels(&self) -> Result<Vec<f64>> {
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
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    /// Around the regions: the regions are the part that stays.
    Outside,
    /// Inside the regions' boundary: the regions are the opening.
    Inside,
    /// Tool centre on the boundary.
    On,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContourParams {
    pub side: Side,
    /// Climb milling (with a clockwise spindle the wall is on the right of the tool's travel);
    /// otherwise conventional.
    pub climb: bool,
    /// Material left on the wall (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PocketParams {
    /// Distance between passes (mm), at most the tool radius.
    pub stepover: f64,
    pub climb: bool,
    /// Material left on the walls (mm).
    pub stock_to_leave: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DrillParams {
    /// Depth per peck (mm); `None` drills in one pass.
    pub peck: Option<f64>,
    /// Pause at the bottom (seconds).
    pub dwell: f64,
}

/// Collects moves, keeping the tool's position.
struct Builder {
    moves: Vec<Move>,
    pos: Option<Vec3>,
    h: Heights,
    feeds: Feeds,
}

impl Builder {
    fn new(cut: &Cut) -> Builder {
        Builder { moves: Vec::new(), pos: None, h: cut.heights, feeds: cut.feeds }
    }

    fn push(&mut self, m: Move) -> Result<()> {
        if self.moves.len() >= MAX_MOVES {
            return Err(CamError::TooLarge(format!("the toolpath has more than {MAX_MOVES} moves")));
        }
        if let Some(to) = m.target() {
            self.pos = Some(to);
        }
        self.moves.push(m);
        Ok(())
    }

    fn at(&self, p: Vec2) -> bool {
        self.pos.is_some_and(|q| Vec2::new(q.x, q.y).dist(p) <= 1e-9)
    }

    /// Up to the clearance height (if below it), then across to `p`.
    fn travel_to(&mut self, p: Vec2) -> Result<()> {
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
    fn plunge_to(&mut self, z: f64) -> Result<()> {
        let Some(q) = self.pos else { return Err(CamError::Unsafe("a plunge before the first positioning move".into())) };
        if q.z > self.h.retract {
            self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.retract) })?;
        }
        if z < self.pos.map_or(z, |p| p.z) {
            self.push(Move::Feed { to: Vec3::new(q.x, q.y, z), feed: self.feeds.plunge })?;
        }
        Ok(())
    }

    fn cut_to(&mut self, p: Vec2, z: f64) -> Result<()> {
        if self.at(p) && self.pos.is_some_and(|q| (q.z - z).abs() <= 1e-9) {
            return Ok(());
        }
        self.push(Move::Feed { to: p.extend(z), feed: self.feeds.cut })
    }

    /// Cuts along `piece` at depth `z` from wherever the tool is (within the offsetting's joining tolerance of the piece's start).
    fn cut_piece(&mut self, piece: &Piece, z: f64) -> Result<()> {
        match *piece {
            Piece::Line { b, .. } => self.cut_to(b, z),
            Piece::Arc { c, sweep, .. } => {
                // Halves, so that no arc move comes near a full turn.
                let parts = if sweep.abs() > PI { 2 } else { 1 };
                for k in 1..=parts {
                    let to = piece.point_at(k as f64 / parts as f64);
                    self.push(Move::Arc { to: to.extend(z), center: c, ccw: sweep > 0.0, feed: self.feeds.cut })?;
                }
                Ok(())
            }
        }
    }

    fn cut_loop(&mut self, lp: &[Piece], z: f64) -> Result<()> {
        lp.iter().try_for_each(|piece| self.cut_piece(piece, z))
    }

    fn retract(&mut self) -> Result<()> {
        match self.pos {
            Some(q) if q.z < self.h.clearance => self.push(Move::Rapid { to: Vec3::new(q.x, q.y, self.h.clearance) }),
            _ => Ok(()),
        }
    }

    fn finish(mut self) -> Result<Toolpath> {
        self.retract()?;
        let path = Toolpath { moves: self.moves };
        verify(&path, &self.h)?;
        Ok(path)
    }
}

fn check_point(p: Vec2) -> Result<()> {
    if p.is_finite() && p.x.abs() <= MAX_COORD && p.y.abs() <= MAX_COORD { Ok(()) } else { invalid("a coordinate is not a usable number") }
}

fn check_leave(leave: f64) -> Result<()> {
    if leave.is_finite() && (0.0..=MAX_COORD).contains(&leave) { Ok(()) } else { invalid("the stock to leave must be 0 or more") }
}

/// The loop turned to start at its corner nearest to `p`.
fn start_near(lp: &[Piece], p: Vec2) -> Vec<Piece> {
    let first = lp.iter().enumerate().min_by(|a, b| a.1.start().dist(p).total_cmp(&b.1.start().dist(p))).map_or(0, |(i, _)| i);
    lp.iter().cycle().skip(first).take(lp.len()).copied().collect()
}

/// Faces the rectangle `min`–`max` with passes along X, the tool running off the material at both ends.
pub fn face(min: Vec2, max: Vec2, tool: &Tool, stepover: f64, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_point(min)?;
    check_point(max)?;
    if !(max.x > min.x) || !(max.y > min.y) {
        return invalid("the area to face is empty");
    }
    if !(stepover > 0.0) || !(stepover <= tool.diameter) {
        return invalid("the stepover must be above 0 and at most the tool diameter");
    }
    let r = tool.radius();
    let height = max.y - min.y;
    let rows = (height / stepover - 1e-9).ceil().max(1.0) + 1.0;
    if !(rows <= MAX_ROWS as f64) {
        return Err(CamError::TooLarge(format!("more than {MAX_ROWS} passes")));
    }
    let rows = rows as usize;
    // Start and end with the whole tool off the material.
    let (x0, x1) = (min.x - r - 1.0, max.x + r + 1.0);
    let mut b = Builder::new(cut);
    for z in cut.levels()? {
        b.travel_to(Vec2::new(x0, min.y))?;
        b.plunge_to(z)?;
        for k in 0..rows {
            let y = min.y + height * k as f64 / (rows - 1).max(1) as f64;
            let (from, to) = if k % 2 == 0 { (x0, x1) } else { (x1, x0) };
            b.cut_to(Vec2::new(from, y), z)?;
            b.cut_to(Vec2::new(to, y), z)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// Follows the boundaries of `regions` on the chosen side, one loop after the other, each
/// down through every depth pass before the next.
pub fn contour(regions: &[Region2], tool: &Tool, params: &ContourParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_leave(params.stock_to_leave)?;
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let away = tool.radius() + params.stock_to_leave;
    let loops = match params.side {
        Side::Outside => shape.offset(-away)?,
        Side::Inside => shape.offset(away)?,
        Side::On => shape.loops().to_vec(),
    };
    if loops.is_empty() {
        return invalid("the tool does not fit the profile");
    }
    // Loops come with the regions on their left. Around a part that puts the wall on the left
    // (conventional), inside an opening on the right (climb).
    let natural_is_climb = params.side != Side::Outside;
    let levels = cut.levels()?;
    let mut b = Builder::new(cut);
    for lp in &loops {
        let lp = if params.climb == natural_is_climb { lp.clone() } else { reversed(lp) };
        let Some(first) = lp.first() else { continue };
        b.travel_to(first.start())?;
        for &z in &levels {
            b.plunge_to(z)?;
            b.cut_loop(&lp, z)?;
            b.cut_to(first.start(), z)?;
        }
        b.retract()?;
    }
    b.finish()
}

/// Clears `regions` (holes are islands) with rings parallel to the boundary, from the inside out.
pub fn pocket(regions: &[Region2], tool: &Tool, params: &PocketParams, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.validate()?;
    check_leave(params.stock_to_leave)?;
    let r = tool.radius();
    if !(params.stepover > 0.0) || !(params.stepover <= r) {
        return invalid("the stepover must be above 0 and at most the tool radius");
    }
    let shape = Shape::from_regions(regions, params.tolerance)?;
    let wall = r + params.stock_to_leave;
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
    let mut b = Builder::new(cut);
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
                    b.plunge_to(z)?;
                }
            }
            b.cut_loop(&lp, z)?;
            b.cut_to(start, z)?;
            down = Some(start);
        }
        b.retract()?;
    }
    b.finish()
}

/// Whether the tool can feed straight from `a` to `b` without touching a wall.
fn link_is_clear(shape: &Shape, a: Vec2, b: Vec2, wall: f64, r: f64) -> bool {
    let n = ((a.dist(b) / (r * 0.25).max(1e-3)).ceil()).clamp(1.0, 2000.0) as usize;
    (0..=n).all(|i| shape.clear(a.lerp(b, i as f64 / n as f64), wall))
}

/// Drills at each point from the top to the bottom height, optionally in pecks that lift the
/// drill to the retract height to clear chips.
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
