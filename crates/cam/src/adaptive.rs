//! Adaptive clearing: a pocket cleared at a steady load on the tool. The tool steps through the
//! material and at each step takes the direction in which it removes as much new area as a
//! straight cut `stepover` wide would over the step's length, so that inner corners and narrow
//! places never bury it in a full slot. A wall counts as material: the tool turns away from it
//! as from too much engagement.
//!
//! The cleared area is kept on a grid of the pocket, a twentieth of the tool's diameter fine: a
//! step's new area is the count of material cells, not yet cleared, under the tool. Each depth
//! pass enters on a helix where there is the most room (else on a ramp, else straight down),
//! then cuts run one after the other; between them the tool links inside the cleared area just
//! above the floor where it can, else over the retract height. Every pass ends with a lap along
//! the walls, and a finishing allowance with one more lap at the full depth.

use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::offset::{Shape, reversed};
use crate::ops::{Builder, Cut};
use crate::path::{Move, Toolpath};
use crate::piece::Piece;
use crate::tool::Tool;
use crate::{CamError, Result, invalid};

/// Most grid cells of one pocket.
const MAX_GRID: usize = 4_000_000;
/// Most steps of all the cuts of one pocket.
const MAX_STEPS: usize = 2_000_000;
/// Most cuts of one pocket.
const MAX_CUTS: usize = 20_000;
/// A cut ends where no direction removes more than this share of a step's target area.
const MIN_SHARE: f64 = 0.15;
/// Most a cut starts engaged, as a share of a step's target area.
const START_SHARE: f64 = 1.0;
/// Points round the tool's edge its engagement is measured at.
const ROUND: usize = 144;
/// Points across the tool's leading edge the width of its cut is measured at.
const ACROSS: usize = 96;
/// How high over the floor links inside the cleared area run (mm).
pub(crate) const LINK_LIFT: f64 = 0.5;

/// How an adaptive pocket is cleared.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AdaptiveParams {
    /// The load: how wide a straight cut would take, as a share of the tool's diameter (0.02–0.5).
    pub stepover: f64,
    /// Climb milling (material on the right of the tool's travel); otherwise conventional.
    pub climb: bool,
    /// Material left on the walls (mm).
    pub stock_to_leave: f64,
    /// More left on the walls by the passes, taken off by one lap at the full depth (mm; 0: none).
    pub finish: f64,
    /// Chord error allowed on spline boundaries (mm).
    pub tolerance: f64,
    /// Steepest descent of the entry helix or ramp (radians).
    pub helix_angle: f64,
    /// The entry helix's diameter, at most and at least, as shares of the tool's diameter.
    pub helix_max: f64,
    pub helix_min: f64,
    /// Rest machining: only what a larger tool this wide (mm) could not reach is cut, the larger
    /// tool having cleared the pocket before to the same depth and walls.
    pub rest_diameter: Option<f64>,
    /// How much faster than the cutting feed a lighter step may go (1: never): a step that cuts
    /// narrower than the target makes thinner chips, and its feed goes up until the chips are as
    /// thick as the target cut's, at most this many times the feed.
    pub feed_up: f64,
}

/// Points of a cut that lie within this of a circle are cut as one arc (mm).
const ARC_FIT: f64 = 0.01;
/// Feed factors are rounded to this step, so that steps alike share one feed (and one arc).
const FEED_STEP: f64 = 0.05;

/// How a depth pass enters the material at the start of the plan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Entry {
    /// A helix round `c`, `r` from it.
    Helix { c: Vec2, r: f64 },
    /// Back and forth from `a` to `b`.
    Ramp { a: Vec2, b: Vec2 },
    /// Straight down at a point (no room to ramp).
    Plunge(Vec2),
}

/// One cut of the plan, and how the tool gets to its start from where the last one ended:
/// along `link` just above the floor, or (empty) over the retract height.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Run {
    /// The way through the cleared area to the cut's start (empty: the start is where the tool is).
    pub(crate) link: Vec<Vec2>,
    pub(crate) points: Vec<Vec2>,
    /// Each step's feed, as a factor of the cutting feed (the first point's is not used).
    pub(crate) feeds: Vec<f64>,
    /// An entry of its own (a part of the pocket the cleared area does not reach).
    pub(crate) entry: Option<Entry>,
    /// The first cut without an entry (after a larger tool): the tool comes down to it.
    pub(crate) down: bool,
}

/// The pocket on a grid, and what is cleared of it.
pub(crate) struct Grid {
    pub(crate) lo: Vec2,
    pub(crate) cell: f64,
    pub(crate) nx: usize,
    pub(crate) ny: usize,
    /// Where the tool's axis may be, how far from the walls it has room to move, what is to
    /// be cut, and what has been.
    pub(crate) allowed: Vec<bool>,
    pub(crate) room: Vec<f64>,
    pub(crate) material: Vec<bool>,
    pub(crate) cleared: Vec<bool>,
    /// The tool's radius.
    pub(crate) r: f64,
    /// The pocket, and how far from its boundary the tool's axis stays: positions near the
    /// walls are checked against it exactly. Without it (the walls of a part in 3D, known on
    /// the grid only), `allowed` alone says where the axis may go.
    pub(crate) shape: Option<Shape>,
    pub(crate) reach: f64,
}

impl Grid {
    pub(crate) fn centre(&self, i: usize, j: usize) -> Vec2 {
        self.lo + Vec2::new((i as f64 + 0.5) * self.cell, (j as f64 + 0.5) * self.cell)
    }

    pub(crate) fn index(&self, p: Vec2) -> Option<usize> {
        let (fi, fj) = ((p.x - self.lo.x) / self.cell, (p.y - self.lo.y) / self.cell);
        if !(fi >= 0.0 && fj >= 0.0) {
            return None;
        }
        let (i, j) = (fi as usize, fj as usize);
        (i < self.nx && j < self.ny).then_some(j * self.nx + i)
    }

    fn allowed_at(&self, p: Vec2) -> bool {
        let Some(shape) = &self.shape else {
            return self.index(p).and_then(|k| self.allowed.get(k).copied()).unwrap_or(false);
        };
        let Some(room) = self.index(p).and_then(|k| self.room.get(k).copied()) else { return false };
        let inside = self.index(p).and_then(|k| self.allowed.get(k).copied()).unwrap_or(false) || room > f64::MIN;
        // A cell's centre says it for the whole cell only well away from the walls.
        if room > self.cell {
            return true;
        }
        if room < -self.cell || !inside {
            return false;
        }
        shape.clear(p, self.reach)
    }

    /// The cells within `r` of `p`.
    pub(crate) fn disc(&self, p: Vec2, r: f64) -> impl Iterator<Item = usize> + '_ {
        let cell = self.cell;
        let span = |c: f64, lo: f64, n: usize| {
            let a = ((c - r - lo) / cell).floor().max(0.0) as usize;
            let b = (((c + r - lo) / cell).ceil().max(0.0) as usize).min(n);
            a..b
        };
        let (is, js) = (span(p.x, self.lo.x, self.nx), span(p.y, self.lo.y, self.ny));
        js.flat_map(move |j| is.clone().map(move |i| (i, j))).filter(move |&(i, j)| self.centre(i, j).dist(p) <= r).map(move |(i, j)| j * self.nx + i)
    }

    /// The material not yet cut under the tool at `p` (mm²).
    fn area(&self, p: Vec2) -> f64 {
        let n =
            self.disc(p, self.r).filter(|&k| self.material.get(k).copied().unwrap_or(false) && !self.cleared.get(k).copied().unwrap_or(true)).count();
        n as f64 * self.cell * self.cell
    }

    /// How much of the tool's edge at `p` is in material not yet cut (radians, 0 to 2π): its
    /// engagement angle.
    fn engagement(&self, p: Vec2) -> f64 {
        let n = (0..ROUND)
            .filter(|&k| {
                let q = p + Vec2::from_angle(TAU * k as f64 / ROUND as f64) * self.r;
                self.index(q).is_some_and(|c| self.material.get(c).copied().unwrap_or(false) && !self.cleared.get(c).copied().unwrap_or(true))
            })
            .count();
        TAU * n as f64 / ROUND as f64
    }

    /// How wide a cut the tool takes moving from `p` to `q` (mm): of its width across the
    /// travel, the part whose leading edge passes through material not yet cut on the way.
    fn width(&self, p: Vec2, q: Vec2) -> f64 {
        let d = q - p;
        let len = d.len();
        if !(len > 1e-12) {
            return 0.0;
        }
        let u = d * (1.0 / len);
        let n = Vec2::new(-u.y, u.x);
        let samples = ((len / (self.cell * 0.5)).ceil() as usize).clamp(1, 1000);
        let uncut =
            |x: Vec2| self.index(x).is_some_and(|c| self.material.get(c).copied().unwrap_or(false) && !self.cleared.get(c).copied().unwrap_or(true));
        let hits = (0..ACROSS)
            .filter(|&k| {
                let w = self.r * ((k as f64 + 0.5) / ACROSS as f64 * 2.0 - 1.0);
                let edge = u * (self.r * self.r - w * w).max(0.0).sqrt() + n * w;
                (0..=samples).any(|t| uncut(p + edge + d * (t as f64 / samples as f64)))
            })
            .count();
        2.0 * self.r * hits as f64 / ACROSS as f64
    }

    /// Marks what the tool sweeps from `a` to `b` as cut.
    fn sweep(&mut self, a: Vec2, b: Vec2) {
        let (r, cell) = (self.r, self.cell);
        let (lo, hi) = (Vec2::new(a.x.min(b.x) - r, a.y.min(b.y) - r), Vec2::new(a.x.max(b.x) + r, a.y.max(b.y) + r));
        let i0 = ((lo.x - self.lo.x) / cell).floor().max(0.0) as usize;
        let i1 = (((hi.x - self.lo.x) / cell).ceil().max(0.0) as usize).min(self.nx);
        let j0 = ((lo.y - self.lo.y) / cell).floor().max(0.0) as usize;
        let j1 = (((hi.y - self.lo.y) / cell).ceil().max(0.0) as usize).min(self.ny);
        let d = b - a;
        let len2 = d.len2();
        for j in j0..j1 {
            for i in i0..i1 {
                let c = self.centre(i, j);
                let t = if len2 > 0.0 { ((c - a).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
                if c.dist(a + d * t) <= r
                    && let Some(x) = self.cleared.get_mut(j * self.nx + i)
                {
                    *x = true;
                }
            }
        }
    }

    /// Whether the straight move from `a` to `b` keeps the tool inside the walls and over cut material only.
    fn clear_link(&self, a: Vec2, b: Vec2) -> bool {
        let n = ((a.dist(b) / (self.cell * 0.5)).ceil() as usize).clamp(1, 1_000_000);
        (0..=n).all(|k| {
            let p = a.lerp(b, k as f64 / n as f64);
            self.allowed_at(p) && self.area(p) <= 0.0
        })
    }

    /// The material the tool can reach that is still standing (mm²).
    fn left(&self, reachable: &[bool]) -> f64 {
        let n = (0..self.material.len())
            .filter(|&k| {
                self.material.get(k).copied().unwrap_or(false)
                    && !self.cleared.get(k).copied().unwrap_or(true)
                    && reachable.get(k).copied().unwrap_or(false)
            })
            .count();
        n as f64 * self.cell * self.cell
    }
}

/// Clears `regions` (holes are islands) with `tool` at a steady engagement.
pub fn adaptive(regions: &[Region2], tool: &Tool, params: &AdaptiveParams, cut: &Cut) -> Result<Toolpath> {
    check(tool, params, cut)?;
    let shape = Shape::from_regions(regions, params.tolerance)?;
    adaptive_in(&shape, tool, params, cut)
}

/// Whether `p` can clear with `tool` within `cut`.
pub(crate) fn check(tool: &Tool, p: &AdaptiveParams, cut: &Cut) -> Result<()> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if !(0.02..=0.5).contains(&p.stepover) {
        return invalid("the adaptive stepover must be 2% to 50% of the tool's diameter");
    }
    for (v, what) in [(p.stock_to_leave, "the stock to leave"), (p.finish, "the finishing allowance")] {
        if !(v.is_finite() && (0.0..=1.0e6).contains(&v)) {
            return invalid(format!("{what} must be 0 or more"));
        }
    }
    if !(p.helix_angle > 0.0 && p.helix_angle <= 1.5) {
        return invalid("the entry angle must be above 0 and at most 85 degrees");
    }
    if !(p.helix_min > 0.0 && p.helix_min <= p.helix_max && p.helix_max <= 10.0) {
        return invalid("the entry helix's diameters must be above 0, the smallest no larger than the largest");
    }
    if !(p.feed_up >= 1.0 && p.feed_up <= 5.0) {
        return invalid("the feed on light cuts may go up 1 to 5 times, no more");
    }
    if let Some(big) = p.rest_diameter
        && (!(big > tool.diameter + 1e-9) || !(big <= 1.0e6))
    {
        return invalid("rest machining follows a larger tool: its diameter must be above this tool's");
    }
    Ok(())
}

/// [`adaptive`] of a checked `shape`.
fn adaptive_in(shape: &Shape, tool: &Tool, p: &AdaptiveParams, cut: &Cut) -> Result<Toolpath> {
    let (r, wall) = (tool.radius(), p.stock_to_leave + p.finish);
    let mut grid = grid(shape, r, wall)?;
    // After a larger tool: what it reached (as far from the walls) is cut already.
    if let Some(big) = p.rest_diameter {
        let big_r = big * 0.5;
        let mut reached = vec![false; grid.cleared.len()];
        for k in 0..grid.room.len() {
            let room = grid.room.get(k).copied().unwrap_or(f64::MIN);
            if room + r >= big_r {
                let c = grid.centre(k % grid.nx, k / grid.nx);
                for x in grid.disc(c, big_r + grid.cell).collect::<Vec<_>>() {
                    if let Some(v) = reached.get_mut(x) {
                        *v = true;
                    }
                }
            }
        }
        grid.cleared = reached;
        if !grid.material.iter().zip(&grid.cleared).any(|(m, c)| *m && !*c) {
            return invalid(format!("the {big} mm tool leaves nothing for this tool: it reached the whole pocket"));
        }
    }
    if !grid.allowed.iter().any(|a| *a) {
        return invalid("the tool does not fit the pocket");
    }
    // Every material cell some position of the tool covers: what is left of the rest is out of reach (corners).
    let mut reachable = vec![false; grid.allowed.len()];
    for k in 0..grid.allowed.len() {
        if grid.allowed.get(k).copied().unwrap_or(false) {
            let c = grid.centre(k % grid.nx, k / grid.nx);
            let cells: Vec<usize> = grid.disc(c, r + grid.cell * 0.5).collect();
            for x in cells {
                if let Some(v) = reachable.get_mut(x) {
                    *v = true;
                }
            }
        }
    }
    let (entry, runs) = plan(&mut grid, &reachable, tool, p)?;
    // The walls: a lap along them on each pass, and the finishing lap at the full depth.
    let walls = oriented(shape.offset(r + wall)?, p.climb);
    let finish = if p.finish > 0.0 { oriented(shape.offset(r + p.stock_to_leave)?, p.climb) } else { Vec::new() };
    let mut b = Builder::new(cut);
    let mut cleared = cut.heights.top;
    for z in cut.levels()? {
        pass(&mut b, &grid, entry, &runs, z, &|_| cleared, cut, p)?;
        lap(&mut b, &walls, z)?;
        cleared = z;
    }
    lap(&mut b, &finish, cut.heights.bottom)?;
    b.finish()
}

/// A depth pass at `z`: its entry and its cuts, each reached along its link through the cleared
/// area or entering on its own; an entry starts `from` the height it gives for it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn pass(
    b: &mut Builder,
    grid: &Grid,
    entry: Option<Entry>,
    runs: &[Run],
    z: f64,
    from: &dyn Fn(Entry) -> f64,
    cut: &Cut,
    p: &AdaptiveParams,
) -> Result<()> {
    if let Some(entry) = entry {
        enter(b, entry, from(entry), z, cut, p)?;
    }
    for run in runs {
        match (&run.entry, run.down) {
            (Some(e), _) => {
                b.retract()?;
                enter(b, *e, from(*e), z, cut, p)?;
            }
            (None, false) => {
                // Up a little, along the cleared floor, and down again.
                let lift = (z + LINK_LIFT).min(cut.heights.retract);
                if let Some(q) = b.pos {
                    b.push(Move::Feed { to: Vec3::new(q.x, q.y, lift), feed: cut.feeds.cut })?;
                }
                for &l in &run.link {
                    b.push(Move::Feed { to: l.extend(lift), feed: cut.feeds.cut })?;
                }
                if let Some(first) = run.points.first() {
                    b.push(Move::Feed { to: first.extend(lift), feed: cut.feeds.cut })?;
                    b.push(Move::Feed { to: first.extend(z), feed: cut.feeds.plunge })?;
                }
            }
            (None, true) => {
                if let Some(first) = run.points.first() {
                    b.retract()?;
                    b.travel_to(*first)?;
                    b.plunge_to(z)?;
                }
            }
        }
        cut_run(b, grid, &run.points, &run.feeds, z, cut.feeds.cut)?;
    }
    Ok(())
}

/// The pocket's grid: where the tool's axis may go (`r` and `wall` off the boundary), and the
/// material (what is more than `wall` inside it).
fn grid(shape: &Shape, r: f64, wall: f64) -> Result<Grid> {
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(Piece::bounds) {
        (lo, hi) = (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)));
    }
    let cell = (r / 10.0).max(0.01);
    let (fx, fy) = (((hi.x - lo.x) / cell).ceil(), ((hi.y - lo.y) / cell).ceil());
    if !lo.is_finite() || !hi.is_finite() || !(fx * fy <= MAX_GRID as f64) {
        return Err(CamError::TooLarge("the pocket is too large for adaptive clearing with so small a tool".into()));
    }
    let (nx, ny) = (fx.max(1.0) as usize, fy.max(1.0) as usize);
    let mut g = Grid {
        lo,
        cell,
        nx,
        ny,
        allowed: vec![false; nx * ny],
        room: vec![f64::MIN; nx * ny],
        material: vec![false; nx * ny],
        cleared: vec![false; nx * ny],
        r,
        shape: Some(shape.clone()),
        reach: r + wall,
    };
    for j in 0..ny {
        for i in 0..nx {
            let q = g.centre(i, j);
            let d = shape.dist(q);
            if !shape.inside(q, d.max(1e-6)) {
                continue;
            }
            let k = j * nx + i;
            let free = d - r - wall;
            if let (Some(a), Some(m), Some(room)) = (g.allowed.get_mut(k), g.material.get_mut(k), g.room.get_mut(k)) {
                *a = free >= 0.0;
                *room = free;
                *m = d > wall;
            }
        }
    }
    Ok(g)
}

/// The loops as the tool runs them: climb along the walls of a pocket.
fn oriented(loops: Vec<Vec<Piece>>, climb: bool) -> Vec<Vec<Piece>> {
    loops.into_iter().map(|lp| if climb { lp } else { reversed(&lp) }).collect()
}

/// A lap along each loop at `z`, getting to it over the retract height.
fn lap(b: &mut Builder, loops: &[Vec<Piece>], z: f64) -> Result<()> {
    for lp in loops {
        let Some(first) = lp.first() else { continue };
        b.retract()?;
        b.travel_to(first.start())?;
        b.plunge_to(z)?;
        for piece in lp {
            b.cut_to(piece.start(), z)?;
            b.cut_piece(piece, z, z)?;
        }
        b.cut_to(first.start(), z)?;
    }
    b.retract()
}

/// Gets the tool down from `from` (already cut to) to `z` at the entry.
pub(crate) fn enter(b: &mut Builder, entry: Entry, from: f64, z: f64, cut: &Cut, p: &AdaptiveParams) -> Result<()> {
    let slope = p.helix_angle.tan();
    match entry {
        Entry::Helix { c, r } => {
            let start = c + Vec2::new(r, 0.0);
            b.retract()?;
            b.travel_to(start)?;
            b.plunge_to(from)?;
            let turn = TAU * r * slope;
            let turns = ((from - z) / turn - 1e-9).ceil().max(1.0) as usize;
            let drop = (from - z) / turns as f64;
            let other = c - Vec2::new(r, 0.0);
            // Half turns down, then a whole turn at the depth.
            for k in 1..=2 * (turns + 1) {
                let at = if k % 2 == 0 { start } else { other };
                let to_z = (from - drop * 0.5 * k as f64).max(z);
                b.push(Move::Arc { to: at.extend(to_z), center: c, ccw: p.climb, feed: cut.feeds.cut })?;
            }
        }
        Entry::Ramp { a, b: end } => {
            b.retract()?;
            b.travel_to(a)?;
            b.plunge_to(from)?;
            let leg = a.dist(end);
            let per_leg = (leg * slope).max(1e-6);
            let legs = ((from - z) / per_leg - 1e-9).ceil().max(1.0) as usize;
            let drop = (from - z) / legs as f64;
            for k in 1..=legs {
                let at = if k % 2 == 1 { end } else { a };
                b.cut_to(at, from - drop * k as f64)?;
            }
            // Once along at the depth, back to where it is.
            let (here, there) = if legs % 2 == 1 { (end, a) } else { (a, end) };
            b.cut_to(there, z)?;
            b.cut_to(here, z)?;
        }
        Entry::Plunge(q) => {
            b.retract()?;
            b.travel_to(q)?;
            b.plunge_to(z)?;
        }
    }
    Ok(())
}

/// The plan of a depth pass: where it enters, then the cuts, each with its link.
pub(crate) fn plan(g: &mut Grid, reachable: &[bool], tool: &Tool, p: &AdaptiveParams) -> Result<(Option<Entry>, Vec<Run>)> {
    // Material smaller than this is left to the lap along the walls.
    let crumbs = p.stepover * tool.diameter * g.cell * 2.0;
    plan_to(g, reachable, tool, p, crumbs, usize::MAX)
}

/// [`plan`], cutting until no more than `crumbs` (mm²) of the material in reach is left, or
/// `patience` starts in a row cut nothing (what is left is beyond the tool).
pub(crate) fn plan_to(
    g: &mut Grid,
    reachable: &[bool],
    tool: &Tool,
    p: &AdaptiveParams,
    crumbs: f64,
    patience: usize,
) -> Result<(Option<Entry>, Vec<Run>)> {
    let ae = p.stepover * tool.diameter;
    // The engagement angle of a straight cut `ae` wide.
    let phi = (1.0 - 2.0 * p.stepover).clamp(-1.0, 1.0).acos();
    let s = (tool.diameter / 8.0).max(g.cell * 2.0);
    // After a larger tool the first cut starts beside what it left, down where it cleared;
    // otherwise the pass enters where there is the most room.
    let (first, mut at) = if p.rest_diameter.is_some() {
        let k = (0..g.allowed.len())
            .filter(|&k| g.allowed.get(k).copied().unwrap_or(false))
            .min_by(|&a, &b| {
                let near = |k: usize| {
                    let c = g.centre(k % g.nx, k / g.nx);
                    let e = g.engagement(c);
                    if e > 0.0 { e } else { f64::INFINITY }
                };
                near(a).total_cmp(&near(b))
            })
            .ok_or_else(|| CamError::Invalid("the tool does not fit the pocket".into()))?;
        (None, g.centre(k % g.nx, k / g.nx))
    } else {
        let e = entry_at(g, tool, p, None).ok_or_else(|| CamError::Invalid("the tool does not fit the pocket".into()))?;
        (Some(e), entry_end(e))
    };
    let mut runs: Vec<Run> = Vec::new();
    let mut steps = 0usize;
    // Starts from which nothing could be cut: not tried again.
    let mut dead = vec![false; g.allowed.len()];
    let mut tries = 0usize;
    let mut idle = 0usize;
    while runs.len() < MAX_CUTS && tries < MAX_CUTS * 4 && idle < patience && g.left(reachable) > crumbs {
        tries += 1;
        let (start, link, entry) = match next_start(g, at, phi, &dead) {
            Some((q, link)) => (q, link, None),
            None => {
                // A part of the pocket the cleared area does not reach: an entry of its own.
                let Some(e) = entry_at(g, tool, p, Some(reachable)) else { break };
                (entry_end(e), Vec::new(), Some(e))
            }
        };
        let (points, widths) = cut_from(g, start, ae, s, p.climb, &mut steps)?;
        let feeds = widths.iter().map(|w| feed_factor(*w, ae, tool.diameter, p.feed_up)).collect();
        let moved = points.len() > 1;
        if !moved && entry.is_none() {
            // Nothing could be cut from here: another start, if there is one.
            if let Some(d) = g.index(start).and_then(|k| dead.get_mut(k)) {
                *d = true;
            }
            idle += 1;
            continue;
        }
        idle = 0;
        at = points.last().copied().unwrap_or(start);
        // Without an entry the tool is not down yet at the first cut: it comes down to it.
        let down = first.is_none() && runs.is_empty();
        runs.push(Run { link, points, feeds, entry, down });
    }
    Ok((first, runs))
}

/// Where the tool is after entering.
pub(crate) fn entry_end(e: Entry) -> Vec2 {
    match e {
        Entry::Helix { c, r } => c + Vec2::new(r, 0.0),
        Entry::Ramp { a, .. } => a,
        Entry::Plunge(q) => q,
    }
}

/// An entry where there is the most room (among positions over material still standing, when
/// `fresh` is given: with material not yet cut under the tool there), clearing what it cuts on
/// the grid.
fn entry_at(g: &mut Grid, tool: &Tool, p: &AdaptiveParams, fresh: Option<&[bool]>) -> Option<Entry> {
    let mut open: Vec<usize> = (0..g.allowed.len())
        .filter(|&k| g.allowed.get(k).copied().unwrap_or(false))
        .filter(|&k| fresh.is_none_or(|reach| reach.get(k).copied().unwrap_or(false) && !g.cleared.get(k).copied().unwrap_or(true)))
        .collect();
    open.sort_by(|&a, &b| g.room.get(b).copied().unwrap_or(0.0).total_cmp(&g.room.get(a).copied().unwrap_or(0.0)));
    let best = match fresh {
        Some(_) => open.into_iter().find(|&k| g.area(g.centre(k % g.nx, k / g.nx)) > 0.0)?,
        None => open.first().copied()?,
    };
    let c = g.centre(best % g.nx, best / g.nx);
    let room = g.room.get(best).copied().unwrap_or(0.0);
    let rh = (p.helix_max * tool.diameter * 0.5).min(room);
    if rh >= p.helix_min * tool.diameter * 0.5 {
        let cells: Vec<usize> = g.disc(c, rh + g.r).collect();
        for k in cells {
            if let Some(x) = g.cleared.get_mut(k) {
                *x = true;
            }
        }
        return Some(Entry::Helix { c, r: rh });
    }
    // The longest straight line through the point that the tool may follow, both ways.
    let reach = |u: Vec2| {
        let mut d = 0.0;
        while d < tool.diameter * 4.0 && g.allowed_at(c + u * (d + g.cell)) {
            d += g.cell;
        }
        d
    };
    let (u, len) =
        (0..36).map(|k| Vec2::from_angle(PI * k as f64 / 36.0)).map(|u| (u, reach(u) + reach(u * -1.0))).max_by(|a, b| a.1.total_cmp(&b.1))?;
    if len >= tool.diameter * 0.5 {
        let (a, b) = (c - u * reach(u * -1.0), c + u * reach(u));
        g.sweep(a, b);
        return Some(Entry::Ramp { a, b });
    }
    g.sweep(c, c);
    Some(Entry::Plunge(c))
}

/// Where the next cut starts: the nearest position, reached through the cleared area from `at`,
/// where the tool touches material but takes at most a step's share; and the way there.
fn next_start(g: &Grid, at: Vec2, target: f64, dead: &[bool]) -> Option<(Vec2, Vec<Vec2>)> {
    let k0 = g.index(at)?;
    let mut came: Vec<usize> = vec![usize::MAX; g.allowed.len()];
    let mut queue = VecDeque::from([k0]);
    *came.get_mut(k0)? = k0;
    while let Some(k) = queue.pop_front() {
        let (i, j) = (k % g.nx, k / g.nx);
        let a = g.engagement(g.centre(i, j));
        if a > 0.0 && a <= target * START_SHARE && !dead.get(k).copied().unwrap_or(false) && g.allowed_at(g.centre(i, j)) {
            // The way back, as a few straight moves between the cells it went through.
            let mut way = vec![g.centre(i, j)];
            let mut c = k;
            while let Some(&prev) = came.get(c).filter(|&&p| p != c) {
                way.push(g.centre(prev % g.nx, prev / g.nx));
                c = prev;
            }
            way.reverse();
            let link = straighten(g, at, &way);
            return Some((g.centre(i, j), link));
        }
        // Only through what is cut: the link must not cut.
        if (a > 0.0 || g.area(g.centre(i, j)) > 0.0) && k != k0 {
            continue;
        }
        for (di, dj) in [(1i64, 0i64), (-1, 0), (0, 1), (0, -1)] {
            let (ni, nj) = (i as i64 + di, j as i64 + dj);
            if ni < 0 || nj < 0 || ni as usize >= g.nx || nj as usize >= g.ny {
                continue;
            }
            let n = nj as usize * g.nx + ni as usize;
            if g.allowed.get(n).copied().unwrap_or(false) && came.get(n).copied() == Some(usize::MAX) {
                if let Some(c) = came.get_mut(n) {
                    *c = k;
                }
                queue.push_back(n);
            }
        }
    }
    None
}

/// The cells of a way through the cleared area as a few straight moves: from `at`, on to the
/// farthest cell still in sight, and so on.
fn straighten(g: &Grid, at: Vec2, way: &[Vec2]) -> Vec<Vec2> {
    let mut out = Vec::new();
    let mut from = at;
    let mut k = 0;
    while k + 1 < way.len() {
        let mut far = k + 1;
        for (m, q) in way.iter().enumerate().skip(k + 1) {
            if g.clear_link(from, *q) {
                far = m;
            }
        }
        if let Some(q) = way.get(far) {
            out.push(*q);
            from = *q;
        }
        k = far;
    }
    out
}

/// One cut from `start`: steps `s` long (shorter where the full step would take too much), each
/// in the direction where the tool's cut comes to `ae` wide, the material on the right when
/// climbing. Ends where nothing more is in reach.
fn cut_from(g: &mut Grid, start: Vec2, ae: f64, s: f64, climb: bool, steps: &mut usize) -> Result<(Vec<Vec2>, Vec<f64>)> {
    let into = if climb { -1.0 } else { 1.0 };
    let mut pts = vec![start];
    // How wide each step cuts (the start's: none).
    let mut widths = vec![0.0];
    let mut p = start;
    // The first direction: of all round, the one that takes nearest the target with the
    // material on the cutting side; where there is none climbing, the other way round.
    let Some((u0, into)) = [into, -into].into_iter().find_map(|side| first_direction(g, start, ae, s, side).map(|u| (u, side))) else {
        return Ok((pts, widths));
    };
    let mut dirs: VecDeque<Vec2> = VecDeque::from([u0]);
    loop {
        if *steps >= MAX_STEPS {
            return Err(CamError::TooLarge(format!("the adaptive toolpath needs more than {MAX_STEPS} steps; use a larger tool or stepover")));
        }
        let pred = dirs.iter().fold(Vec2::ZERO, |a, d| a + *d);
        let pred = if pred.len() > 1e-12 { pred * (1.0 / pred.len()) } else { dirs.back().copied().unwrap_or(u0) };
        let Some((q, u, w)) = step(g, p, pred, ae, s, into) else { break };
        g.sweep(p, q);
        pts.push(q);
        widths.push(w);
        p = q;
        *steps += 1;
        dirs.push_back(u);
        if dirs.len() > 3 {
            dirs.pop_front();
        }
    }
    Ok((pts, widths))
}

/// The feed of a step that cuts `w` wide, as a factor of the feed for a cut `ae` wide (tool
/// `d`): up by as much as the chips get thinner, at most `most` times, never down.
fn feed_factor(w: f64, ae: f64, d: f64, most: f64) -> f64 {
    // A chip's thickness against the feed per tooth: 2·√(x(1−x)) for a cut x·d wide, the full
    // feed from half the diameter on.
    let chip = |w: f64| {
        let x = (w / d).clamp(0.0, 0.5);
        2.0 * (x * (1.0 - x)).sqrt()
    };
    let (thin, nominal) = (chip(w), chip(ae));
    if !(most > 1.0) {
        return 1.0;
    }
    if !(thin > 1e-9) {
        return most;
    }
    let f = (nominal / thin).clamp(1.0, most);
    ((f / FEED_STEP).floor() * FEED_STEP).max(1.0)
}

/// Cuts through `points` at `z` from the first (where the tool is), each step at its feed
/// factor of `feed`; runs of points alike in feed that lie on a circle go as one arc, where the
/// arc keeps clear of the walls as the steps do.
pub(crate) fn cut_run(b: &mut Builder, g: &Grid, points: &[Vec2], feeds: &[f64], z: f64, feed: f64) -> Result<()> {
    let clear = |q: Vec2| match &g.shape {
        Some(shape) => shape.clear(q, g.reach),
        None => g.allowed_at(q),
    };
    let mut i = 0;
    while i + 1 < points.len() {
        let f = feeds.get(i + 1).copied().unwrap_or(1.0);
        // As many points on as fit one arc, at the same feed.
        let mut best: Option<(usize, Vec2, bool)> = None;
        let mut j = i + 2;
        while j < points.len() && feeds.get(j).copied().unwrap_or(1.0) == f {
            match points.get(i..=j).and_then(|run| arc_through(run, &clear)) {
                Some((c, ccw)) => best = Some((j, c, ccw)),
                None => break,
            }
            j += 1;
        }
        match (best, points.get(i + 1)) {
            (Some((j, center, ccw)), _) => {
                let to = points.get(j).copied().unwrap_or_default();
                b.push(Move::Arc { to: to.extend(z), center, ccw, feed: feed * f })?;
                i = j;
            }
            (None, Some(&q)) => {
                b.push(Move::Feed { to: q.extend(z), feed: feed * f })?;
                i += 1;
            }
            (None, None) => break,
        }
    }
    Ok(())
}

/// The circle the points lie on, within [`ARC_FIT`], in order and turning one way through less
/// than half a turn, its bulge between them `clear` of the walls: its centre and whether it goes
/// anticlockwise. `None` for a straight run (better a line) or points off any circle.
fn arc_through(pts: &[Vec2], clear: &impl Fn(Vec2) -> bool) -> Option<(Vec2, bool)> {
    let (&a, &z) = (pts.first()?, pts.last()?);
    let m = *pts.get(pts.len() / 2)?;
    // The circle through the ends and the middle point.
    let (ab, ac) = (m - a, z - a);
    let cross = ab.x * ac.y - ab.y * ac.x;
    if cross.abs() < 1e-12 {
        return None;
    }
    let (b2, c2) = (ab.len2(), ac.len2());
    let c = a + Vec2::new(ac.y * b2 - ab.y * c2, ab.x * c2 - ac.x * b2) * (0.5 / cross);
    let r = c.dist(a);
    // Nearly straight: lines are the plainer program.
    if !(r.is_finite()) || r > 1000.0 {
        return None;
    }
    let ccw = cross > 0.0;
    let mut turned = 0.0;
    for w in pts.windows(2) {
        let (&p, &q) = (w.first()?, w.get(1)?);
        if (c.dist(q) - r).abs() > ARC_FIT {
            return None;
        }
        let (u, v) = (p - c, q - c);
        let t = (u.x * v.y - u.y * v.x).atan2(u.dot(v));
        // Every step turns the same way, and the chord's middle stays near the circle.
        if (t > 0.0) != ccw || t.abs() < 1e-12 || (c.dist(p.lerp(q, 0.5)) - r).abs() > ARC_FIT {
            return None;
        }
        // The arc's farthest point from the chord.
        let mid = (p.lerp(q, 0.5) - c).normalized().map(|n| c + n * r)?;
        if !clear(mid) {
            return None;
        }
        turned += t.abs();
    }
    (turned < PI * 0.9).then_some((c, ccw))
}

/// The direction to start a cut from `p` in: of 36 round, the one whose step of `s` takes nearest
/// `target` without more, the material on the `into` side (more of it when turning that way).
fn first_direction(g: &Grid, p: Vec2, target: f64, s: f64, into: f64) -> Option<Vec2> {
    let take = |u: Vec2| {
        let q = p + u * s;
        if g.allowed_at(q) { g.width(p, q) } else { f64::INFINITY }
    };
    (0..36)
        .map(|k| Vec2::from_angle(TAU * k as f64 / 36.0))
        .filter_map(|u| {
            let a = take(u);
            let (towards, away) = (take(turned(u, into * 0.35)), take(turned(u, -into * 0.35)));
            (a <= target && a >= target * MIN_SHARE && towards >= away).then_some((u, a))
        })
        .max_by(|x, y| x.1.total_cmp(&y.1))
        .map(|(u, _)| u)
}

/// `u` turned by `a` (radians, anticlockwise).
fn turned(u: Vec2, a: f64) -> Vec2 {
    let (sin, cos) = a.sin_cos();
    Vec2::new(u.x * cos - u.y * sin, u.x * sin + u.y * cos)
}

/// The next step from `p` near the direction `pred`: its end and direction. The search turns
/// from away from the material (`into` the other way) towards it until the new area reaches the
/// target; a position the tool may not take counts as too much. Shorter steps where even the
/// gentlest turn takes too much; none where nothing near takes enough.
fn step(g: &Grid, p: Vec2, pred: Vec2, target: f64, s: f64, into: f64) -> Option<(Vec2, Vec2, f64)> {
    for len in [s, s * 0.5, s * 0.25] {
        let at = |t: f64| {
            let u = turned(pred, t * into);
            let q = p + u * len;
            let a = if g.allowed_at(q) { g.width(p, q) } else { f64::INFINITY };
            (q, u, a)
        };
        // From 100° away from the material to 100° into it.
        let n = 20;
        let samples: Vec<(f64, (Vec2, Vec2, f64))> =
            (0..=n).map(|k| -100.0 + 200.0 * k as f64 / n as f64).map(|deg: f64| (deg.to_radians(), at(deg.to_radians()))).collect();
        let first_over = samples.iter().position(|(_, (_, _, a))| *a > target);
        match first_over {
            // Even turning away takes too much: try a shorter step.
            Some(0) => continue,
            Some(k) => {
                let (mut lo, mut hi) = (samples.get(k - 1)?.0, samples.get(k)?.0);
                for _ in 0..8 {
                    let mid = (lo + hi) * 0.5;
                    if at(mid).2 > target {
                        hi = mid;
                    } else {
                        lo = mid;
                    }
                }
                let (q, u, a) = at(lo);
                // Something must be cut, or the tool would run on through what is cleared.
                if a >= target * MIN_SHARE && a.is_finite() && g.area(q) > 0.0 {
                    return Some((q, u, a));
                }
                return None;
            }
            // Nowhere near enough: into the material as far as it goes, if that takes something.
            None => {
                let (_, (q, u, a)) = samples.into_iter().filter(|(_, (_, _, a))| a.is_finite()).max_by(|x, y| x.1.2.total_cmp(&y.1.2))?;
                return (a >= target * MIN_SHARE && g.area(q) > 0.0).then_some((q, u, a));
            }
        }
    }
    None
}
