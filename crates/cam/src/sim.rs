//! Material removal on a height field: the stock as a grid of column heights that each cutting
//! move lowers under the tool. It answers what a program removes, whether a rapid runs into
//! material, how deep the tool is buried, and, against a height field of the part, where the
//! program cuts into the part or leaves material on it.
//!
//! A height field knows the top of each column and, for stock already cut from the other side,
//! its bottom; undercuts do not exist in it. A column cut through is empty: top and bottom on the
//! floor. Along a sloped move a column takes the lowest the tool reaches over it near where the
//! move passes nearest (a few positions within the tool's radius either way).

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Vec2, Vec3};

use crate::path::Toolpath;
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most columns of a height field.
pub const MAX_CELLS: usize = 4_000_000;
/// Most column updates of one simulated toolpath.
const MAX_WORK: u64 = 4_000_000_000;
/// How many positions either way along a sloped move a column looks at for the lowest the tool
/// reaches over it.
const SLOPE_LOOKS: u32 = 8;
/// Most columns a dilation keeps maxima of (one field per width).
const MAX_DILATION: usize = 16_000_000;

/// Column heights over a rectangle. Column `(i, j)` is centred at `origin + (i + ½, j + ½) × cell`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HeightField {
    pub origin: Vec2,
    pub cell: f64,
    pub nx: usize,
    pub ny: usize,
    /// Row by row (`j × nx + i`).
    pub z: Vec<f64>,
    /// Columns are never cut below this.
    pub floor: f64,
    /// Where each column's material starts, as `z` (empty: on the floor everywhere): above the
    /// floor where the stock was cut from below.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bottom: Vec<f64>,
}

impl HeightField {
    /// A block from `min` to `max` in columns `cell` wide.
    pub fn block(min: Vec3, max: Vec3, cell: f64) -> Result<HeightField> {
        if !min.is_finite() || !max.is_finite() || !(max.x > min.x) || !(max.y > min.y) || !(max.z > min.z) {
            return invalid("the stock must be a box with a size");
        }
        if !(cell > 0.0) || !cell.is_finite() {
            return invalid("the simulation's cell size must be above 0");
        }
        let (nx, ny) = (((max.x - min.x) / cell).ceil(), ((max.y - min.y) / cell).ceil());
        if !(nx * ny <= MAX_CELLS as f64) {
            return Err(CamError::TooLarge(format!("the simulation would need more than {MAX_CELLS} columns; use a larger cell")));
        }
        let (nx, ny) = (nx.max(1.0) as usize, ny.max(1.0) as usize);
        Ok(HeightField { origin: Vec2::new(min.x, min.y), cell, nx, ny, z: vec![max.z; nx * ny], floor: min.z, bottom: Vec::new() })
    }

    pub fn centre(&self, i: usize, j: usize) -> Vec2 {
        self.origin + Vec2::new((i as f64 + 0.5) * self.cell, (j as f64 + 0.5) * self.cell)
    }

    pub fn get(&self, i: usize, j: usize) -> Option<f64> {
        if i < self.nx && j < self.ny { self.z.get(j * self.nx + i).copied() } else { None }
    }

    /// Where column `k`'s material starts (`j × nx + i`).
    pub fn base(&self, k: usize) -> f64 {
        self.bottom.get(k).map_or(self.floor, |b| b.max(self.floor))
    }

    /// Where the material of the column under `p` starts; the floor off the field.
    pub fn base_at(&self, p: Vec2) -> f64 {
        self.column(p).map_or(self.floor, |(i, j)| self.base(j * self.nx + i))
    }

    /// Columns whose centres lie in `lo`–`hi`: index ranges, clipped to the field.
    pub(crate) fn range(&self, lo: Vec2, hi: Vec2) -> (std::ops::Range<usize>, std::ops::Range<usize>) {
        let index = |v: f64, n: usize| ((v / self.cell).floor().clamp(0.0, n as f64)) as usize;
        let (i0, i1) = (index(lo.x - self.origin.x, self.nx), index(hi.x - self.origin.x + self.cell, self.nx));
        let (j0, j1) = (index(lo.y - self.origin.y, self.ny), index(hi.y - self.origin.y + self.cell, self.ny));
        (i0..i1, j0..j1)
    }

    /// Volume of the columns' material.
    pub fn volume(&self) -> f64 {
        self.z.iter().enumerate().map(|(k, z)| (z - self.base(k)).max(0.0)).sum::<f64>() * self.cell * self.cell
    }

    /// The top of `triangles` seen from above, on columns `cell` wide over `min`–`max`; `floor`
    /// where none is. A column takes the highest triangle over its centre; with `edges`, the
    /// corners and edges of every triangle also mark the columns they pass through, so that walls
    /// seen edge-on are not lost between centres (what to keep a tool out of, rather than what
    /// a column's centre sees).
    pub fn of_triangles(min: Vec2, max: Vec2, cell: f64, floor: f64, triangles: &[[Vec3; 3]], edges: bool) -> Result<HeightField> {
        let mut f = HeightField::block(min.extend(floor), max.extend(floor + 1.0), cell)?;
        f.z.iter_mut().for_each(|z| *z = floor);
        if !(triangles.len() as f64 * 64.0 <= MAX_WORK as f64) {
            return Err(CamError::TooLarge("the part has too many triangles to look at from above".into()));
        }
        let mut work: u64 = 0;
        for tri in triangles {
            if !tri.iter().all(|v| v.is_finite()) {
                continue;
            }
            let [a, b, c] = *tri;
            let lo = Vec2::new(a.x.min(b.x).min(c.x), a.y.min(b.y).min(c.y));
            let hi = Vec2::new(a.x.max(b.x).max(c.x), a.y.max(b.y).max(c.y));
            let (is, js) = f.range(lo - Vec2::new(cell, cell), hi);
            work = work.saturating_add((is.len() * js.len()) as u64);
            if work > MAX_WORK {
                return Err(CamError::TooLarge("the part is too detailed to look at from above at this cell size".into()));
            }
            // Over each column centre inside the triangle: its height there.
            let d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
            if d.abs() > 1e-12 {
                for j in js.clone() {
                    for i in is.clone() {
                        let p = f.centre(i, j);
                        let l1 = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / d;
                        let l2 = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / d;
                        let l3 = 1.0 - l1 - l2;
                        if l1 >= -1e-9 && l2 >= -1e-9 && l3 >= -1e-9 {
                            f.raise(i, j, l1 * a.z + l2 * b.z + l3 * c.z);
                        }
                    }
                }
            }
            // Along each edge, at half a column: the columns it passes through.
            for (p, q) in [(a, b), (b, c), (c, a)].into_iter().filter(|_| edges) {
                let n = ((Vec2::new(p.x, p.y).dist(Vec2::new(q.x, q.y)) / (cell * 0.5)).ceil()).clamp(1.0, 1.0e5) as usize;
                for k in 0..=n {
                    let t = k as f64 / n as f64;
                    let v = p + (q - p) * t;
                    if let Some((i, j)) = f.column(Vec2::new(v.x, v.y)) {
                        f.raise(i, j, v.z);
                    }
                }
                work = work.saturating_add(n as u64);
            }
        }
        Ok(f)
    }

    /// The column whose square holds `p`.
    pub fn column(&self, p: Vec2) -> Option<(usize, usize)> {
        let (u, v) = ((p.x - self.origin.x) / self.cell, (p.y - self.origin.y) / self.cell);
        (u >= 0.0 && v >= 0.0 && u < self.nx as f64 && v < self.ny as f64).then_some((u as usize, v as usize))
    }

    /// The height of the column under `p`; the floor off the field.
    pub fn at(&self, p: Vec2) -> f64 {
        self.column(p).and_then(|(i, j)| self.get(i, j)).unwrap_or(self.floor)
    }

    fn raise(&mut self, i: usize, j: usize, z: f64) {
        if let Some(c) = self.z.get_mut(j * self.nx + i) {
            *c = c.max(z);
        }
    }

    /// Each column raised to the highest column within `radius` of it: where a tool's axis
    /// may not go below to stay out of what the field holds.
    pub fn dilated(&self, radius: f64) -> Result<HeightField> {
        let reach = (radius / self.cell).ceil().max(0.0);
        // A row of maxima per width is kept: at most this many columns in all.
        if !(reach * (self.nx * self.ny) as f64 <= MAX_DILATION as f64) {
            return Err(CamError::TooLarge("the tool is too large for this cell size".into()));
        }
        let reach = reach as usize;
        // The disc's half-width in columns on the rows `dj` away.
        let half: Vec<usize> = (0..=reach)
            .map(|dj| {
                let dy = (dj as f64 - 1.0).max(0.0) * self.cell;
                (((radius * radius - dy * dy).max(0.0)).sqrt() / self.cell).ceil() as usize
            })
            .collect();
        // Each row's highest within w columns either side, for every w the disc needs, built one
        // width from the last: the window w + 1 wide is the windows w wide one column either side.
        let widest = half.iter().copied().max().unwrap_or(0);
        let (nx, ny) = (self.nx, self.ny);
        let mut rows: Vec<Vec<f64>> = vec![self.z.clone()];
        for _ in 0..widest {
            let last = rows.last().cloned().unwrap_or_default();
            let mut next = last.clone();
            for j in 0..ny {
                for i in 0..nx {
                    let left = if i > 0 { last.get(j * nx + i - 1).copied() } else { None };
                    let right = last.get(j * nx + i + 1).filter(|_| i + 1 < nx).copied();
                    if let Some(c) = next.get_mut(j * nx + i) {
                        *c = c.max(left.unwrap_or(f64::NEG_INFINITY)).max(right.unwrap_or(f64::NEG_INFINITY));
                    }
                }
            }
            rows.push(next);
        }
        let mut out = self.clone();
        for j in 0..ny {
            for i in 0..nx {
                let mut top = self.floor;
                for (dj, w) in half.iter().enumerate() {
                    let Some(row) = rows.get(*w) else { continue };
                    for jj in [j.checked_sub(dj), Some(j + dj).filter(|jj| *jj < ny)].into_iter().flatten() {
                        if let Some(z) = row.get(jj * nx + i) {
                            top = top.max(*z);
                        }
                    }
                }
                if let Some(c) = out.z.get_mut(j * nx + i) {
                    *c = top;
                }
            }
        }
        Ok(out)
    }
}

/// How deep `path` cuts into the part `part` holds (mm; 0 when it never does): the deepest a
/// cutting move's tool end goes below the part's top within its radius (a rounded end: below
/// the part seen through its shape), less a few columns' width (the field's own rounding of a
/// wall, either way) so that a tool running along a wall is not counted.
pub fn gouge(path: &Toolpath, tool: &Tool, part: &HeightField) -> f64 {
    let r = (tool.radius() - part.cell * 2.5).max(0.0);
    // A rounded end clears a wall its tip goes below: the part seen through the tool's shape.
    let reach = if tool.corner() > 0.0 && r > 0.0 {
        let corner = (tool.corner() - part.cell * 2.5).max(0.0);
        let slim = match tool.kind {
            ToolKind::BullNose if corner > 0.0 => Tool { diameter: r * 2.0, corner_radius: Some(corner.min(r)), ..tool.clone() },
            ToolKind::BullNose => Tool { diameter: r * 2.0, ..Tool::new(tool.number, ToolKind::FlatEndmill, r * 2.0, tool.flute_length) },
            _ => Tool { diameter: r * 2.0, ..tool.clone() },
        };
        tool_floor(part, &slim, 0.0)
    } else {
        part.dilated(r)
    };
    let Ok(reach) = reach else { return 0.0 };
    let step = (part.cell * 0.5).max(1e-3);
    let mut deepest: f64 = 0.0;
    let mut samples: u64 = 0;
    let points = path.points(part.cell * 0.25);
    for pair in points.windows(2) {
        let &[(a, _), (b, true)] = pair else { continue };
        let n = ((Vec2::new(a.x, a.y).dist(Vec2::new(b.x, b.y)) / step).ceil()).clamp(1.0, 1.0e5) as usize;
        for k in 0..=n {
            samples += 1;
            if samples > 2_000_000 {
                return deepest;
            }
            let q = a + (b - a) * (k as f64 / n as f64);
            let top = reach.at(Vec2::new(q.x, q.y));
            deepest = deepest.max(top - q.z);
        }
    }
    deepest
}

/// The lowest the tip of `tool` (a flat, ball or bull-nose end mill) may go over each column of
/// the `part` and stay `allowance` clear of it: the part's top seen through the tool's shape,
/// the tool grown by the allowance all round (its corner too). Columns without part get the
/// part's floor.
pub fn tool_floor(part: &HeightField, tool: &Tool, allowance: f64) -> Result<HeightField> {
    let radius = tool.radius();
    if !(radius > 0.0 && radius.is_finite()) || !(allowance >= 0.0 && allowance.is_finite()) {
        return invalid("the tool's radius must be above 0 and the allowance 0 or more");
    }
    let (kind, r) = (tool.kind, radius + allowance);
    let corner = tool.corner() + allowance;
    // How far above its tip the grown tool's surface is at `d` from its axis.
    let rise = |d: f64| match kind {
        ToolKind::BallEndmill | ToolKind::BullNose => rounded(r, corner, d),
        _ => 0.0,
    };
    if !matches!(kind, ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::BullNose) {
        return Err(CamError::Unsupported("following a part in 3D takes a flat, ball or bull-nose end mill".into()));
    }
    if kind == ToolKind::FlatEndmill {
        let mut out = part.dilated(r)?;
        out.z.iter_mut().for_each(|z| *z += allowance);
        return Ok(out);
    }
    // The disc's columns and how far the tool's surface rises at each.
    let reach = (r / part.cell).ceil() as i64;
    let mut disc: Vec<(i64, i64, f64)> = Vec::new();
    for dj in -reach..=reach {
        for di in -reach..=reach {
            let d = ((di * di + dj * dj) as f64).sqrt() * part.cell;
            if d <= r {
                disc.push((di, dj, rise(d)));
            }
        }
    }
    if !((disc.len() as f64) * (part.z.len() as f64) <= MAX_WORK as f64) {
        return Err(CamError::TooLarge("the tool is too large for this cell size".into()));
    }
    let mut out = part.clone();
    for j in 0..part.ny {
        for i in 0..part.nx {
            let mut low = part.floor;
            for &(di, dj, up) in &disc {
                let (ii, jj) = (i as i64 + di, j as i64 + dj);
                if ii < 0 || jj < 0 {
                    continue;
                }
                if let Some(h) = part.get(ii as usize, jj as usize).filter(|h| *h > part.floor) {
                    low = low.max(h - up);
                }
            }
            if let Some(c) = out.z.get_mut(j * part.nx + i) {
                *c = low + allowance;
            }
        }
    }
    Ok(out)
}

/// What one toolpath did to the stock.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CutReport {
    /// Material removed (mm³).
    pub removed_volume: f64,
    /// Rapid moves that ran into material (the stock is left as it was there).
    pub rapid_collisions: usize,
    /// Deepest the tool was buried in material in one move (mm): to compare with its flute length.
    pub max_engagement: f64,
}

/// How far above its tip a rounded end `corner` round, on a tool `r` in radius, is at distance
/// `d` from its axis: flat out to `r - corner`, then round (a ball: `corner` = `r`).
pub(crate) fn rounded(r: f64, corner: f64, d: f64) -> f64 {
    let flat = r - corner;
    if d <= flat { 0.0 } else { corner - (corner * corner - (d - flat).powi(2)).max(0.0).sqrt() }
}

/// How far above its tip the tool's surface is at distance `d` from its axis.
fn profile(tool: &Tool, d: f64) -> f64 {
    let r = tool.radius();
    match tool.kind {
        ToolKind::FlatEndmill => 0.0,
        ToolKind::BallEndmill | ToolKind::BullNose => rounded(r, tool.corner(), d),
        ToolKind::Drill => d / (tool.point_angle() * 0.5).to_radians().tan(),
        ToolKind::VBit => (d - tool.tip() * 0.5).max(0.0) / (tool.point_angle() * 0.5).to_radians().tan(),
    }
}

/// Runs `path` with `tool` through the stock.
pub fn simulate(field: &mut HeightField, path: &Toolpath, tool: &Tool) -> Result<CutReport> {
    tool.validate()?;
    let mut report = CutReport::default();
    let mut work = 0u64;
    let mut from: Option<Vec3> = None;
    // Arcs as chords well inside a column.
    for (to, cutting) in path.points((field.cell * 0.1).max(1e-4)) {
        let Some(a) = from.replace(to) else { continue };
        sweep(field, a, to, cutting, tool, &mut report, &mut work)?;
    }
    Ok(report)
}

/// The tool moved from `a` to `b` through the stock: at feed it cuts; at rapid running into
/// material counts as a collision and leaves the stock as it was. `work` counts column updates.
fn sweep(field: &mut HeightField, a: Vec3, to: Vec3, cutting: bool, tool: &Tool, report: &mut CutReport, work: &mut u64) -> Result<()> {
    if !a.is_finite() || !to.is_finite() {
        return Err(CamError::Unsafe("a coordinate is not a finite number".into()));
    }
    let r = tool.radius();
    let (a2, b2) = (Vec2::new(a.x, a.y), Vec2::new(to.x, to.y));
    let lo = Vec2::new(a2.x.min(b2.x) - r, a2.y.min(b2.y) - r);
    let hi = Vec2::new(a2.x.max(b2.x) + r, a2.y.max(b2.y) + r);
    let (is, js) = field.range(lo, hi);
    *work += (is.len() as u64) * (js.len() as u64);
    if *work > MAX_WORK {
        return Err(CamError::TooLarge("the simulation is too long; use a larger cell".into()));
    }
    let d = b2 - a2;
    let len2 = d.len2();
    let mut collided = false;
    for j in js {
        for i in is.clone() {
            let c = field.centre(i, j);
            let t = if len2 > 0.0 { ((c - a2).dot(d) / len2).clamp(0.0, 1.0) } else { 0.0 };
            let off = c.dist(a2 + d * t);
            if off > r {
                continue;
            }
            // Straight down or up, the lowest point of the move is what cuts.
            let tip_at = |t: f64| if len2 > 0.0 { a.z + (to.z - a.z) * t } else { a.z.min(to.z) };
            let mut z = (tip_at(t) + profile(tool, off)).max(field.floor);
            // Along a slope the tool reaches lowest over the column a little down the move from
            // where it passes nearest: positions within its radius either way are looked at too.
            if len2 > 0.0 && (to.z - a.z).abs() > 1e-9 {
                let window = r / len2.sqrt();
                for k in 1..=SLOPE_LOOKS {
                    for side in [-1.0, 1.0] {
                        let tt = (t + side * window * f64::from(k) / f64::from(SLOPE_LOOKS)).clamp(0.0, 1.0);
                        let off = c.dist(a2 + d * tt);
                        if off <= r {
                            z = z.min((tip_at(tt) + profile(tool, off)).max(field.floor));
                        }
                    }
                }
            }
            let k = j * field.nx + i;
            let base = field.base(k);
            let Some(h) = field.z.get_mut(k) else { continue };
            if z < *h - 1e-9 {
                if cutting {
                    let to = z.max(base);
                    report.max_engagement = report.max_engagement.max(*h - to);
                    report.removed_volume += (*h - to) * field.cell * field.cell;
                    // Down to where the material starts: the column is cut through.
                    if z <= base + 1e-9 {
                        *h = field.floor;
                        if let Some(b) = field.bottom.get_mut(k) {
                            *b = field.floor;
                        }
                    } else {
                        *h = z;
                    }
                } else {
                    collided = true;
                }
            }
        }
    }
    report.rapid_collisions += usize::from(collided);
    Ok(())
}

/// The rate rapids move at in a [`Playback`] unless told otherwise (mm/min): a typical hobby
/// router's; machines differ.
pub const DEFAULT_RAPID_RATE: f64 = 5000.0;
/// Most steps of a playback.
const MAX_STEPS: usize = 20_000_000;

/// One straight step of a [`Playback`]: from `from` to `to`, at feed or rapid, starting
/// `start` minutes into the program and taking `minutes`; `op` is its toolpath's index. A
/// dwell is a step that stays where it is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Step {
    pub from: Vec3,
    pub to: Vec3,
    /// The feed (mm/min); `None` at rapid.
    pub feed: Option<f64>,
    pub dwell: bool,
    pub start: f64,
    pub minutes: f64,
    pub op: usize,
}

/// Toolpaths run one after the other in time, to show the tool moving and the stock being cut
/// as far as the program has got. Arcs are straight steps well inside a column `cell` wide.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Playback {
    pub steps: Vec<Step>,
    /// How long the whole program takes (minutes).
    pub minutes: f64,
    /// The rate rapids move at (mm/min).
    pub rapid_rate: f64,
}

fn check_rapid(rate: f64) -> Result<()> {
    if rate > 0.0 && rate <= 1.0e6 { Ok(()) } else { invalid("the rapid rate must be above 0 and at most 1000000 mm/min") }
}

impl Playback {
    /// `paths` one after the other, rapids at `rapid_rate` (mm/min).
    pub fn new(paths: &[&Toolpath], cell: f64, rapid_rate: f64) -> Result<Playback> {
        check_rapid(rapid_rate)?;
        let mut out = Playback { rapid_rate, ..Playback::default() };
        let mut at: Option<Vec3> = None;
        // Each step starts where the program has got to.
        let push = |out: &mut Playback, step: Step| {
            if out.steps.len() >= MAX_STEPS {
                return Err(CamError::TooLarge(format!("the program has more than {MAX_STEPS} steps to show")));
            }
            out.steps.push(Step { start: out.minutes, ..step });
            out.minutes += step.minutes;
            Ok(())
        };
        for (op, path) in paths.iter().enumerate() {
            for (to, feed, dwell) in path.timed_points((cell * 0.1).max(1e-4)) {
                if !to.is_finite() {
                    return Err(CamError::Unsafe("a coordinate is not a finite number".into()));
                }
                if let Some(from) = at.replace(to) {
                    let rate = feed.filter(|f| *f > 0.0).unwrap_or(rapid_rate);
                    push(&mut out, Step { from, to, feed, dwell: false, start: 0.0, minutes: from.dist(to) / rate, op })?;
                }
                if dwell > 0.0 {
                    push(&mut out, Step { from: to, to, feed, dwell: true, start: 0.0, minutes: dwell, op })?;
                }
            }
        }
        Ok(out)
    }

    /// Rapids at `rate` from now on: every step's time is worked out again. → where `t` minutes
    /// into the program was, in the new times.
    pub fn retime(&mut self, rate: f64, t: f64) -> Result<f64> {
        check_rapid(rate)?;
        let (k, f) = match self.step_at(t).and_then(|k| Some((k, self.steps.get(k)?))) {
            Some((k, s)) => (k, if s.minutes > 0.0 { ((t - s.start) / s.minutes).clamp(0.0, 1.0) } else { 1.0 }),
            None => (0, 0.0),
        };
        let mut start = 0.0;
        for s in &mut self.steps {
            if s.feed.is_none() && !s.dwell {
                s.minutes = s.from.dist(s.to) / rate;
            }
            s.start = start;
            start += s.minutes;
        }
        (self.minutes, self.rapid_rate) = (start, rate);
        Ok(self.steps.get(k).map_or(0.0, |s| s.start + s.minutes * f))
    }

    /// The step under way `t` minutes into the program (the last one after the end).
    pub fn step_at(&self, t: f64) -> Option<usize> {
        if self.steps.is_empty() {
            return None;
        }
        let k = self.steps.partition_point(|s| s.start + s.minutes <= t);
        Some(k.min(self.steps.len() - 1))
    }

    /// Where the tool is `t` minutes into the program, and the step it is on.
    pub fn at(&self, t: f64) -> Option<(Vec3, &Step)> {
        let step = self.steps.get(self.step_at(t)?)?;
        let f = if step.minutes > 0.0 { ((t - step.start) / step.minutes).clamp(0.0, 1.0) } else { 1.0 };
        Some((step.from + (step.to - step.from) * f, step))
    }

    /// Cuts the stock as the program goes from `t0` to `t1` minutes (`t1` after `t0`); `tools`
    /// holds each toolpath's tool.
    pub fn cut(&self, field: &mut HeightField, tools: &[Tool], t0: f64, t1: f64) -> Result<CutReport> {
        tools.iter().try_for_each(Tool::validate)?;
        let mut report = CutReport::default();
        let mut work = 0u64;
        let (Some(first), Some(last)) = (self.step_at(t0), self.step_at(t1)) else { return Ok(report) };
        for step in self.steps.get(first..=last).into_iter().flatten() {
            let tool = tools.get(step.op).ok_or_else(|| CamError::Invalid("a toolpath without its tool".into()))?;
            let along = |t: f64| {
                let f = if step.minutes > 0.0 { ((t - step.start) / step.minutes).clamp(0.0, 1.0) } else { 1.0 };
                step.from + (step.to - step.from) * f
            };
            sweep(field, along(t0), along(t1), step.feed.is_some(), tool, &mut report, &mut work)?;
        }
        Ok(report)
    }
}

/// The stock after cutting against the part.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Comparison {
    /// Deepest cut into the part (mm) and the area cut into (mm²).
    pub max_gouge: f64,
    pub gouge_area: f64,
    /// Thickest material left on the part (mm) and its volume (mm³).
    pub max_leftover: f64,
    pub leftover_volume: f64,
}

/// How each column of the cut `stock` stands against the `part` (both over the same columns):
/// negative, the depth cut into the part; positive, the thickness left on it; zero, neither.
/// A column is compared with the part's columns around it too, so an edge that falls one column
/// over is neither a gouge nor leftover; past the field's border there is no part (its floor),
/// so a part edge on the border is treated the same.
pub fn deviations(stock: &HeightField, part: &HeightField) -> Result<Vec<f64>> {
    if stock.nx != part.nx || stock.ny != part.ny || stock.z.len() != part.z.len() {
        return invalid("the stock and the part are not on the same columns");
    }
    let tol = 1e-6;
    let mut out = Vec::with_capacity(stock.z.len());
    for j in 0..stock.ny {
        for i in 0..stock.nx {
            let h = stock.get(i, j).unwrap_or(stock.floor);
            let (mut lowest, mut highest) = (f64::INFINITY, f64::NEG_INFINITY);
            for (di, dj) in (-1i64..=1).flat_map(|di| (-1i64..=1).map(move |dj| (di, dj))) {
                let (ni, nj) = (i as i64 + di, j as i64 + dj);
                let p = if ni < 0 || nj < 0 { None } else { part.get(ni as usize, nj as usize) };
                match p {
                    Some(p) => (lowest, highest) = (lowest.min(p), highest.max(p)),
                    None => lowest = lowest.min(part.floor),
                }
            }
            out.push(if h < lowest - tol {
                h - lowest
            } else if h > highest + tol {
                h - highest
            } else {
                0.0
            });
        }
    }
    // Stock standing round the part's outline (the corners a round part leaves in a square block)
    // is waste, not material left on the part; its holes and pockets are the part's own.
    for (k, outside) in outside_the_part(part).into_iter().enumerate() {
        if let Some(d) = out.get_mut(k).filter(|d| outside && **d > 0.0) {
            *d = 0.0;
        }
    }
    Ok(out)
}

/// The columns outside the part's outline: those without part that the field's border reaches
/// without crossing the part (so not its holes, which the part surrounds).
fn outside_the_part(part: &HeightField) -> Vec<bool> {
    let (nx, ny) = (part.nx, part.ny);
    let empty = |i: usize, j: usize| part.get(i, j).is_none_or(|z| z <= part.floor + 1e-6);
    let mut outside = vec![false; nx * ny];
    let mut todo: Vec<(usize, usize)> = Vec::new();
    for i in 0..nx {
        todo.extend([(i, 0), (i, ny.saturating_sub(1))]);
    }
    for j in 0..ny {
        todo.extend([(0, j), (nx.saturating_sub(1), j)]);
    }
    while let Some((i, j)) = todo.pop() {
        let k = j * nx + i;
        if i >= nx || j >= ny || outside.get(k).copied().unwrap_or(true) || !empty(i, j) {
            continue;
        }
        if let Some(o) = outside.get_mut(k) {
            *o = true;
        }
        todo.extend([(i + 1, j), (i, j + 1)]);
        if i > 0 {
            todo.push((i - 1, j));
        }
        if j > 0 {
            todo.push((i, j - 1));
        }
    }
    outside
}

/// Compares the cut `stock` with the `part` (see [`deviations`]).
pub fn compare(stock: &HeightField, part: &HeightField) -> Result<Comparison> {
    let area = stock.cell * stock.cell;
    let mut out = Comparison::default();
    for d in deviations(stock, part)? {
        if d < 0.0 {
            out.max_gouge = out.max_gouge.max(-d);
            out.gouge_area += area;
        } else if d > 0.0 {
            out.max_leftover = out.max_leftover.max(d);
            out.leftover_volume += d * area;
        }
    }
    Ok(out)
}
