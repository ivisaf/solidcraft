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
/// Half the point angle of a drill (118°), measured from its axis.
const DRILL_HALF_ANGLE: f64 = 59.0;

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
    fn range(&self, lo: Vec2, hi: Vec2) -> (std::ops::Range<usize>, std::ops::Range<usize>) {
        let index = |v: f64, n: usize| ((v / self.cell).floor().clamp(0.0, n as f64)) as usize;
        let (i0, i1) = (index(lo.x - self.origin.x, self.nx), index(hi.x - self.origin.x + self.cell, self.nx));
        let (j0, j1) = (index(lo.y - self.origin.y, self.ny), index(hi.y - self.origin.y + self.cell, self.ny));
        (i0..i1, j0..j1)
    }

    /// Volume between the columns and the floor.
    pub fn volume(&self) -> f64 {
        self.z.iter().map(|z| (z - self.floor).max(0.0)).sum::<f64>() * self.cell * self.cell
    }
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
        ToolKind::Drill => d / DRILL_HALF_ANGLE.to_radians().tan(),
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

/// Compares the cut `stock` with the `part` (both over the same columns). A column is compared
/// with the part's columns around it too, so an edge that falls one column over is neither a
/// gouge nor leftover.
pub fn compare(stock: &HeightField, part: &HeightField) -> Result<Comparison> {
    if stock.nx != part.nx || stock.ny != part.ny || stock.z.len() != part.z.len() {
        return invalid("the stock and the part are not on the same columns");
    }
    let tol = 1e-6;
    let mut out = Comparison::default();
    let area = stock.cell * stock.cell;
    for j in 0..stock.ny {
        for i in 0..stock.nx {
            let Some(h) = stock.get(i, j) else { continue };
            let (mut lowest, mut highest) = (f64::INFINITY, f64::NEG_INFINITY);
            for (di, dj) in (-1i64..=1).flat_map(|di| (-1i64..=1).map(move |dj| (di, dj))) {
                let (ni, nj) = (i as i64 + di, j as i64 + dj);
                if ni < 0 || nj < 0 {
                    continue;
                }
                if let Some(p) = part.get(ni as usize, nj as usize) {
                    (lowest, highest) = (lowest.min(p), highest.max(p));
                }
            }
            if h < lowest - tol {
                out.max_gouge = out.max_gouge.max(lowest - h);
                out.gouge_area += area;
            } else if h > highest + tol {
                out.max_leftover = out.max_leftover.max(h - highest);
                out.leftover_volume += (h - highest) * area;
            }
        }
    }
    Ok(out)
}
