//! Parallel finishing in 3D: passes side by side across the area at a stepover, each following
//! the part's shape seen from above (no undercuts) with a flat, ball or bull-nose end mill.
//!
//! The part is looked at from above on fine columns (its triangles' edges marking the columns
//! they cross, so that walls are not lost between centres), each as high as its neighbours: what
//! lies between column centres is never missed, at the cost of a little left on slopes. Along each pass the tool's tip goes
//! as low as it may over every point without coming nearer the part than the allowance: the
//! highest of the part under the tool less how far the tool's end rises at that distance from
//! its axis. The passes run back and forth; the tool goes on to the next along the part when it
//! is near, else over the clearance height. Points on a straight run are left out where the line
//! between the others stays within the tolerance above them, never below.

use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::offset::Shape;
use crate::ops::{Builder, Cut};
use crate::path::{Move, Toolpath};
use crate::piece::Piece;
use crate::sim::{self, HeightField};
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most columns of the part's field.
const MAX_GRID: usize = 6_000_000;
/// Most points along all the passes.
const MAX_POINTS: usize = 4_000_000;
/// Most passes.
const MAX_PASSES: usize = 20_000;

/// How 3D parallel finishing cuts, besides the tool and its cut.
#[derive(Clone, Copy, Debug)]
pub struct Finish3d<'a> {
    /// Where the tool's axis stays (holes are islands).
    pub boundary: &'a [Region2],
    /// The part as triangles, in work coordinates.
    pub part: &'a [[Vec3; 3]],
    /// How far apart the passes are (mm).
    pub stepover: f64,
    /// Which way the passes run: degrees from the X axis.
    pub angle: f64,
    /// Material left on the part all round (mm).
    pub stock_to_leave: f64,
    /// How far a straight run may stray from the surface between the points kept (mm).
    pub tolerance: f64,
    /// Back and forth (otherwise every pass the same way, the tool up between them).
    pub zigzag: bool,
}

/// The passes of `job` with `tool`, down to `cut`'s bottom at most.
pub fn parallel_3d(job: &Finish3d, tool: &Tool, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if !matches!(tool.kind, ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::BullNose) {
        return Err(CamError::Unsupported("3D finishing with anything but a flat, ball or bull-nose end mill".into()));
    }
    let r = tool.radius();
    if !(job.stepover > 0.0 && job.stepover <= tool.diameter) {
        return invalid("the stepover must be above 0 and at most the tool's diameter");
    }
    if !(job.stock_to_leave >= 0.0 && job.stock_to_leave.is_finite()) {
        return invalid("the stock to leave must be 0 or more");
    }
    if !(job.tolerance > 0.0 && job.tolerance <= 1.0) || !job.angle.is_finite() {
        return invalid("the tolerance must be above 0 and at most 1 mm, the angle a number");
    }
    let shape = Shape::from_regions(job.boundary, job.tolerance.min(0.01))?;
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(Piece::bounds) {
        (lo, hi) = (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)));
    }
    if !lo.is_finite() || !hi.is_finite() {
        return invalid("the area has no size");
    }
    // The part round the area as far as the tool reaches, on columns fine enough for the tolerance.
    let s = job.stock_to_leave;
    let reach = r + s;
    let cell = (r / 20.0).clamp(0.05, 0.5);
    let (flo, fhi) = (lo - Vec2::new(reach + cell, reach + cell), hi + Vec2::new(reach + cell, reach + cell));
    if !(((fhi.x - flo.x) / cell).ceil() * ((fhi.y - flo.y) / cell).ceil() <= MAX_GRID as f64) {
        return Err(CamError::TooLarge("the area is too large to finish this finely; use a larger tolerance".into()));
    }
    let floor = cut.heights.bottom - 1.0;
    // Each column as high as the part over its square reaches (its neighbours' centres taken
    // in): between centres the surface is never missed, so the tool never goes below it.
    let part = HeightField::of_triangles(flo, fhi, cell, floor, job.part, true)?.dilated(cell)?;
    // The tool grown by the allowance: the columns under it and how far its end rises at each.
    let corner = tool.corner() + s;
    let span = (reach / cell).ceil() as i64;
    let mut disc: Vec<(i64, i64, f64)> = Vec::new();
    for dj in -span..=span {
        for di in -span..=span {
            let d = ((di * di + dj * dj) as f64).sqrt() * cell;
            if d <= reach + cell * 0.5 {
                disc.push((di, dj, sim::rounded(reach, corner, d.min(reach))));
            }
        }
    }
    // The lowest the tip may go over `p`: the part under the tool, less its rise, plus the allowance.
    let tip = |p: Vec2| -> f64 {
        let Some((i, j)) = part.column(p) else { return cut.heights.bottom };
        let mut low = f64::NEG_INFINITY;
        for &(di, dj, up) in &disc {
            let (ii, jj) = (i as i64 + di, j as i64 + dj);
            if ii < 0 || jj < 0 {
                continue;
            }
            if let Some(h) = part.get(ii as usize, jj as usize).filter(|h| *h > part.floor) {
                low = low.max(h - up);
            }
        }
        (low + s).max(cut.heights.bottom)
    };
    // The passes: lines `stepover` apart across the area, the way `angle` says, each cut into
    // runs where the tool's axis is inside the boundary.
    let along = Vec2::from_angle(job.angle.to_radians());
    let across = Vec2::new(-along.y, along.x);
    let corners = [lo, Vec2::new(hi.x, lo.y), hi, Vec2::new(lo.x, hi.y)];
    let (mut a0, mut a1, mut c0, mut c1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for q in corners {
        (a0, a1) = (a0.min(q.dot(along)), a1.max(q.dot(along)));
        (c0, c1) = (c0.min(q.dot(across)), c1.max(q.dot(across)));
    }
    let passes = ((c1 - c0) / job.stepover).ceil().max(0.0) as usize + 1;
    if passes > MAX_PASSES {
        return Err(CamError::TooLarge(format!("more than {MAX_PASSES} passes; use a larger stepover")));
    }
    let step = cell;
    let samples = ((a1 - a0) / step).ceil().max(1.0) as usize;
    if passes.saturating_mul(samples + 1) > MAX_POINTS {
        return Err(CamError::TooLarge("the passes would have too many points; use a larger tolerance or stepover".into()));
    }
    let mut runs: Vec<Vec<Vec3>> = Vec::new();
    for k in 0..passes {
        let c = (c0 + job.stepover * k as f64).min(c1);
        let mut run: Vec<Vec3> = Vec::new();
        for t in 0..=samples {
            let a = (a0 + step * t as f64).min(a1);
            let p = along * a + across * c;
            if shape.inside(p, shape.dist(p).max(1e-6)) {
                run.push(p.extend(tip(p)));
            } else if run.len() > 1 {
                runs.push(std::mem::take(&mut run));
            } else {
                run.clear();
            }
        }
        if run.len() > 1 {
            runs.push(run);
        }
    }
    // Back and forth: every other pass the other way round.
    if job.zigzag {
        let mut flip = false;
        let mut last_c: Option<f64> = None;
        for run in &mut runs {
            let c = run.first().map(|p| Vec2::new(p.x, p.y).dot(across));
            if let (Some(c), Some(l)) = (c, last_c)
                && (c - l).abs() > 1e-9
            {
                flip = !flip;
            }
            last_c = c.or(last_c);
            if flip {
                run.reverse();
            }
        }
    }
    let mut b = Builder::new(cut);
    for run in &runs {
        let kept = thin(run, job.tolerance);
        let Some(&first) = kept.first() else { continue };
        let from = b.pos;
        let near = from.is_some_and(|q| Vec2::new(q.x, q.y).dist(Vec2::new(first.x, first.y)) <= job.stepover * 2.0 + 1e-9);
        match from {
            // On to the next pass along the part (it is near): over the surface between them.
            Some(q) if near && job.zigzag => {
                let (a, z) = (Vec2::new(q.x, q.y), Vec2::new(first.x, first.y));
                let n = ((a.dist(z) / step).ceil() as usize).max(1);
                for m in 1..=n {
                    let p = a.lerp(z, m as f64 / n as f64);
                    let h = if m == n { first.z } else { tip(p) };
                    b.push(Move::Feed { to: p.extend(h), feed: cut.feeds.cut })?;
                }
            }
            _ => {
                b.retract()?;
                b.travel_to(Vec2::new(first.x, first.y))?;
                b.plunge_to(first.z)?;
            }
        }
        for p in kept.iter().skip(1) {
            b.push(Move::Feed { to: *p, feed: cut.feeds.cut })?;
        }
    }
    b.finish()
}

/// The points of `run` to cut through: a point is left out where the line between its
/// neighbours kept stays above every point it passes, and no more than `tol` above any.
fn thin(run: &[Vec3], tol: f64) -> Vec<Vec3> {
    let mut out: Vec<Vec3> = Vec::new();
    let Some(&first) = run.first() else { return out };
    out.push(first);
    let mut start = 0;
    while start + 1 < run.len() {
        // As far on as one straight move covers.
        let mut end = start + 1;
        let (Some(&a), mut best) = (run.get(start), start + 1) else { break };
        while end < run.len() {
            let Some(&z) = run.get(end) else { break };
            let fits = (start + 1..end).all(|k| {
                let Some(&q) = run.get(k) else { return false };
                let t = (k - start) as f64 / (end - start) as f64;
                let line = a.z + (z.z - a.z) * t;
                line >= q.z - 1e-9 && line - q.z <= tol
            });
            if !fits {
                break;
            }
            best = end;
            end += 1;
        }
        if let Some(&q) = run.get(best) {
            out.push(q);
        }
        start = best;
    }
    out
}
