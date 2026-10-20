//! Material removal on a height field: the stock as a grid of column heights that each cutting
//! move lowers under the tool. It answers what a program removes, whether a rapid runs into
//! material, how deep the tool is buried, and, against a height field of the part, where the
//! program cuts into the part or leaves material on it.
//!
//! A height field only knows the top of each column, so undercuts do not exist in it. Along a
//! sloped move a column takes the tool's height at the nearest point of the move.

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Vec2, Vec3};

use crate::path::Toolpath;
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most columns of a height field.
pub const MAX_CELLS: usize = 4_000_000;
/// Most column updates of one simulated toolpath.
const MAX_WORK: u64 = 4_000_000_000;
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
        Ok(HeightField { origin: Vec2::new(min.x, min.y), cell, nx, ny, z: vec![max.z; nx * ny], floor: min.z })
    }

    pub fn centre(&self, i: usize, j: usize) -> Vec2 {
        self.origin + Vec2::new((i as f64 + 0.5) * self.cell, (j as f64 + 0.5) * self.cell)
    }

    pub fn get(&self, i: usize, j: usize) -> Option<f64> {
        if i < self.nx && j < self.ny { self.z.get(j * self.nx + i).copied() } else { None }
    }

    /// Columns whose centres lie in `lo`–`hi`: index ranges, clipped to the field.
    pub(crate) fn range(&self, lo: Vec2, hi: Vec2) -> (std::ops::Range<usize>, std::ops::Range<usize>) {
        let index = |v: f64, n: usize| ((v / self.cell).floor().clamp(0.0, n as f64)) as usize;
        let (i0, i1) = (index(lo.x - self.origin.x, self.nx), index(hi.x - self.origin.x + self.cell, self.nx));
        let (j0, j1) = (index(lo.y - self.origin.y, self.ny), index(hi.y - self.origin.y + self.cell, self.ny));
        (i0..i1, j0..j1)
    }

    /// Volume between the columns and the floor.
    pub fn volume(&self) -> f64 {
        self.z.iter().map(|z| (z - self.floor).max(0.0)).sum::<f64>() * self.cell * self.cell
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
/// cutting move's tool end goes below the part's top within its radius, less a few columns'
/// width (the field's own rounding of a wall, either way) so that a tool running along a wall
/// is not counted.
pub fn gouge(path: &Toolpath, tool: &Tool, part: &HeightField) -> f64 {
    let r = (tool.radius() - part.cell * 2.5).max(0.0);
    let Ok(reach) = part.dilated(r) else { return 0.0 };
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

/// How far above its tip the tool's surface is at distance `d` from its axis.
fn profile(tool: &Tool, d: f64) -> f64 {
    let r = tool.radius();
    match tool.kind {
        ToolKind::FlatEndmill => 0.0,
        ToolKind::BallEndmill => r - (r * r - d * d).max(0.0).sqrt(),
        ToolKind::Drill => d / (tool.point_angle() * 0.5).to_radians().tan(),
        ToolKind::VBit => (d - tool.tip() * 0.5).max(0.0) / (tool.point_angle() * 0.5).to_radians().tan(),
    }
}

/// Runs `path` with `tool` through the stock.
pub fn simulate(field: &mut HeightField, path: &Toolpath, tool: &Tool) -> Result<CutReport> {
    tool.validate()?;
    let r = tool.radius();
    let mut report = CutReport::default();
    let mut work = 0u64;
    let mut from: Option<Vec3> = None;
    // Arcs as chords well inside a column.
    for (to, cutting) in path.points((field.cell * 0.1).max(1e-4)) {
        let Some(a) = from.replace(to) else { continue };
        if !a.is_finite() || !to.is_finite() {
            return Err(CamError::Unsafe("a coordinate is not a finite number".into()));
        }
        let (a2, b2) = (Vec2::new(a.x, a.y), Vec2::new(to.x, to.y));
        let lo = Vec2::new(a2.x.min(b2.x) - r, a2.y.min(b2.y) - r);
        let hi = Vec2::new(a2.x.max(b2.x) + r, a2.y.max(b2.y) + r);
        let (is, js) = field.range(lo, hi);
        work += (is.len() as u64) * (js.len() as u64);
        if work > MAX_WORK {
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
                let tip = if len2 > 0.0 { a.z + (to.z - a.z) * t } else { a.z.min(to.z) };
                let z = (tip + profile(tool, off)).max(field.floor);
                let Some(h) = field.z.get_mut(j * field.nx + i) else { continue };
                if z < *h - 1e-9 {
                    if cutting {
                        report.max_engagement = report.max_engagement.max(*h - z);
                        report.removed_volume += (*h - z) * field.cell * field.cell;
                        *h = z;
                    } else {
                        collided = true;
                    }
                }
            }
        }
        report.rapid_collisions += usize::from(collided);
    }
    Ok(report)
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
    Ok(out)
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
