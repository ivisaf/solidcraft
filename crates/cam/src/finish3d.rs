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

use crate::drop::BallDrop;
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
/// Rest machining cuts where more than this is left on the part over the allowance (mm):
/// thicker than a finishing pass's cusps.
const REST_MIN: f64 = 0.05;

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
    /// What earlier operations left of the stock (its columns' tops), where known: the tool
    /// comes down fast to just over it.
    pub left: Option<&'a HeightField>,
    /// Rest machining: the tool only goes where it reaches material `left` leaves on the part.
    pub rest: bool,
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
    let near = rest_zone(job.left.filter(|_| job.rest), job.part, flo, fhi, cell, floor, s, reach)?;
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
    // A ball is dropped onto the triangles themselves: exact, nothing kept back for between columns.
    let ball = (tool.kind == ToolKind::BallEndmill).then(|| BallDrop::new(job.part, r + s)).flatten();
    // The lowest the tip may go over `p`: the part under the tool, less its rise, plus the allowance.
    let tip = |p: Vec2| -> f64 {
        if let Some(ball) = &ball {
            return ball.centre(p).map_or(cut.heights.bottom, |c| c - r).max(cut.heights.bottom);
        }
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
    // Rest machining keeps the points where the tool takes something off, and a tool's
    // radius of run-up either side.
    let rest = job.left.filter(|_| job.rest).map(|left| Ok::<_, CamError>((left, part_on(left, job.part, floor)?))).transpose()?;
    let spread = (r / step).ceil() as usize;
    for k in 0..passes {
        let c = (c0 + job.stepover * k as f64).min(c1);
        let points: Vec<Option<Vec3>> = (0..=samples)
            .map(|t| {
                let p = along * (a0 + step * t as f64).min(a1) + across * c;
                (shape.inside(p, shape.dist(p).max(1e-6)) && near(p)).then(|| p.extend(tip(p)))
            })
            .collect();
        let keep = match &rest {
            Some((left, seen)) => spread_along(
                &points.iter().map(|q| q.is_some_and(|q| bites(left, seen, tool, Vec2::new(q.x, q.y), q.z, s))).collect::<Vec<_>>(),
                &points.iter().map(|q| q.map_or(f64::INFINITY, |q| q.z)).collect::<Vec<_>>(),
                spread,
                r,
            ),
            None => vec![true; points.len()],
        };
        let mut run: Vec<Vec3> = Vec::new();
        for (q, kept) in points.iter().zip(keep) {
            match q.filter(|_| kept) {
                Some(q) => run.push(q),
                None if run.len() > 1 => runs.push(std::mem::take(&mut run)),
                None => run.clear(),
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
    // Where the passes break up (round an island, or only where rest is), on to whichever run
    // starts nearest (either end when going both ways) rather than back across the part.
    if runs.len() <= MAX_ORDERED {
        let mut left = std::mem::take(&mut runs);
        let mut at = left.first().and_then(|r| r.first()).map_or(Vec2::new(0.0, 0.0), |p| Vec2::new(p.x, p.y));
        while !left.is_empty() {
            let dist = |p: Option<&Vec3>| p.map_or(f64::INFINITY, |p| Vec2::new(p.x, p.y).dist(at));
            let (pick, flip, _) = left.iter().enumerate().fold((0, false, f64::INFINITY), |best, (k, run)| {
                let (head, tail) = (dist(run.first()), if job.zigzag { dist(run.last()) } else { f64::INFINITY });
                if head.min(tail) < best.2 { (k, tail < head, head.min(tail)) } else { best }
            });
            let mut run = left.swap_remove(pick);
            if flip {
                run.reverse();
            }
            at = run.last().map_or(at, |p| Vec2::new(p.x, p.y));
            runs.push(run);
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
                approach(&mut b, job.left, Vec2::new(first.x, first.y), first.z, tool, cut)?;
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

/// How 3D Z-level finishing cuts, besides the tool and its cut.
#[derive(Clone, Copy, Debug)]
pub struct ZLevel3d<'a> {
    /// Where the tool's axis stays (holes are islands).
    pub boundary: &'a [Region2],
    /// The part as triangles, in work coordinates.
    pub part: &'a [[Vec3; 3]],
    /// How far apart the levels are (mm).
    pub step: f64,
    /// Material left on the part all round (mm).
    pub stock_to_leave: f64,
    /// How far a straight run may stray from the contour between the points kept (mm).
    pub tolerance: f64,
    /// Only where the part is at least this steep (degrees; 0: wherever it is touched).
    pub steep: f64,
    /// Climb: the part on the right of the tool's travel (otherwise on its left).
    pub climb: bool,
    /// What is left of the stock and rest machining, as for [`Finish3d`].
    pub left: Option<&'a HeightField>,
    pub rest: bool,
}

/// Level by level from `cut`'s top down to its bottom, the tool runs round the part along the
/// line where it touches it at that height (the lowest its tip may go over each point, as for
/// [`parallel_3d`], equal to the level), where the part there is at least `steep`.
pub fn zlevel_3d(job: &ZLevel3d, tool: &Tool, cut: &Cut) -> Result<Toolpath> {
    tool.validate()?;
    cut.heights.validate()?;
    cut.feeds.validate()?;
    if !matches!(tool.kind, ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::BullNose) {
        return Err(CamError::Unsupported("3D finishing with anything but a flat, ball or bull-nose end mill".into()));
    }
    let r = tool.radius();
    if !(job.step > 0.0 && job.step.is_finite()) {
        return invalid("the level step must be above 0");
    }
    if !(job.stock_to_leave >= 0.0 && job.stock_to_leave.is_finite()) {
        return invalid("the stock to leave must be 0 or more");
    }
    if !(job.tolerance > 0.0 && job.tolerance <= 1.0) || !(0.0..90.0).contains(&job.steep) {
        return invalid("the tolerance must be above 0 and at most 1 mm, the steepness 0 to under 90 degrees");
    }
    let shape = Shape::from_regions(job.boundary, job.tolerance.min(0.01))?;
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(Piece::bounds) {
        (lo, hi) = (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)));
    }
    if !lo.is_finite() || !hi.is_finite() {
        return invalid("the area has no size");
    }
    let (top, bottom) = (cut.heights.top, cut.heights.bottom);
    let n_levels = ((top - bottom) / job.step - 1e-9).ceil().max(1.0);
    if !(n_levels <= 20_000.0) {
        return Err(CamError::TooLarge("more than 20000 levels; use a larger level step".into()));
    }
    // The lowest the tip may go over each column, on columns fine enough for the tolerance, the
    // part taken as high as each column's neighbours (nothing between centres missed).
    let s = job.stock_to_leave;
    let cell = (r / 20.0).clamp(0.05, 0.5);
    let margin = Vec2::new(r + s + cell * 2.0, r + s + cell * 2.0);
    let (flo, fhi) = (lo - margin, hi + margin);
    if !(((fhi.x - flo.x) / cell).ceil() * ((fhi.y - flo.y) / cell).ceil() <= MAX_GRID as f64) {
        return Err(CamError::TooLarge("the area is too large to finish this finely".into()));
    }
    let part = HeightField::of_triangles(flo, fhi, cell, bottom - 1.0, job.part, true)?.dilated(cell)?;
    let near = rest_zone(job.left.filter(|_| job.rest), job.part, flo, fhi, cell, bottom - 1.0, s, r + s)?;
    let rest = job.left.filter(|_| job.rest).map(|left| Ok::<_, CamError>((left, part_on(left, job.part, bottom - 1.0)?))).transpose()?;
    let reach = sim::tool_floor(&part, tool, s)?;
    let (nx, ny) = (reach.nx, reach.ny);
    let at = |i: usize, j: usize| reach.get(i, j).unwrap_or(bottom - 1.0);
    // A ball's contours are found on the columns, then put exactly where it touches the part at
    // the level: along the way up the slope, by halving.
    let ball = (tool.kind == ToolKind::BallEndmill).then(|| BallDrop::new(job.part, r + s)).flatten();
    let exact = |p: Vec2| ball.as_ref().map(|b| b.centre(p).map_or(f64::NEG_INFINITY, |c| c - r));
    let uphill = |p: Vec2| -> Option<Vec2> {
        let (i, j) = reach.column(p)?;
        let (i0, j0) = (i.saturating_sub(1), j.saturating_sub(1));
        let (i1, j1) = ((i + 1).min(nx - 1), (j + 1).min(ny - 1));
        let gx = (at(i1, j) - at(i0, j)) / ((i1 - i0).max(1) as f64 * cell);
        let gy = (at(i, j1) - at(i, j0)) / ((j1 - j0).max(1) as f64 * cell);
        Vec2::new(gx, gy).normalized()
    };
    let settle = |p: Vec2, z: f64| -> Option<Vec2> {
        let (Some(here), Some(up)) = (exact(p), uphill(p)) else { return Some(p) };
        let over = |t: f64| exact(p + up * t).unwrap_or(here) - z;
        // A span from where the tool clears the part at the level to where it would not.
        let (mut lo, mut hi) = if here <= z { (0.0, cell * 3.0) } else { (-cell * 3.0, 0.0) };
        let mut grow = 0;
        while over(lo) > 0.0 && grow < 8 {
            lo -= cell * 3.0;
            grow += 1;
        }
        if over(lo) > 0.0 {
            return None;
        }
        if over(hi) <= 0.0 {
            return Some(p + up * hi);
        }
        for _ in 0..40 {
            let mid = (lo + hi) * 0.5;
            if over(mid) > 0.0 {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        Some(p + up * lo)
    };
    let slope = job.steep.to_radians().tan();
    let mut b = Builder::new(cut);
    let n = n_levels as usize;
    for k in 1..=n {
        let z = if k == n { bottom } else { top - (top - bottom) * k as f64 / n as f64 };
        let mut runs = contours(&reach, z, job.climb, |i, j| {
            // Steep enough in this square: the tool's floor rises that much across it.
            let (a, b2, c, d) = (at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1));
            let gx = ((b2 - a) + (c - d)) / (2.0 * cell);
            let gy = ((d - a) + (c - b2)) / (2.0 * cell);
            (gx * gx + gy * gy).sqrt() >= slope
        });
        // Only inside the boundary, exactly at the part (a ball), and nothing shorter than the tool's radius.
        runs = runs
            .into_iter()
            .map(|run| if ball.is_some() { run.into_iter().filter_map(|p| settle(p, z)).collect() } else { run })
            .flat_map(|run: Vec<Vec2>| split_inside(&run, &shape, &near))
            // Rest machining: where the tool takes something off at this level, and a tool's
            // radius of run-up either side.
            .flat_map(|run: Vec<Vec2>| match &rest {
                Some((left, seen)) => {
                    let along =
                        run.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(p, q)| p.dist(*q))).sum::<f64>() / run.len().max(1) as f64;
                    let keep = spread_along(
                        &run.iter().map(|p| bites(left, seen, tool, *p, z, s)).collect::<Vec<_>>(),
                        &vec![z; run.len()],
                        (r / along.max(1e-3)).ceil() as usize,
                        r,
                    );
                    let mut out: Vec<Vec<Vec2>> = Vec::new();
                    let mut part: Vec<Vec2> = Vec::new();
                    for (p, kept) in run.iter().zip(keep) {
                        if kept {
                            part.push(*p);
                        } else if part.len() > 1 {
                            out.push(std::mem::take(&mut part));
                        } else {
                            part.clear();
                        }
                    }
                    if part.len() > 1 {
                        out.push(part);
                    }
                    out
                }
                None => vec![run],
            })
            .filter(|run| run.windows(2).map(|w| w.first().zip(w.get(1)).map_or(0.0, |(p, q)| p.dist(*q))).sum::<f64>() >= r * 0.5)
            .collect();
        // Nearest first, from where the tool is.
        while !runs.is_empty() {
            let from = b.pos.map_or(lo, |q| Vec2::new(q.x, q.y));
            let pick = (0..runs.len())
                .min_by(|&x, &y| {
                    let d = |k: usize| runs.get(k).and_then(|r| r.first()).map_or(f64::INFINITY, |p| p.dist(from));
                    d(x).total_cmp(&d(y))
                })
                .unwrap_or(0);
            let run = runs.swap_remove(pick);
            let kept = thin_xy(&run, job.tolerance * 0.5, job.climb);
            let Some(&first) = kept.first() else { continue };
            approach(&mut b, job.left, first, z, tool, cut)?;
            for p in kept.iter().skip(1) {
                b.push(Move::Feed { to: p.extend(z), feed: cut.feeds.cut })?;
            }
        }
        let nx_ny = nx.saturating_mul(ny);
        if b.len() > MAX_POINTS.max(nx_ny) {
            return Err(CamError::TooLarge("the levels have too many moves; use a larger level step".into()));
        }
    }
    b.finish()
}

/// The lines where `f` (the tool's floor) equals `z`, joined into runs, each with the higher
/// side (the part) on the right when `climb`, else on the left; only through squares `keep`
/// keeps. Marching squares over the column centres, crossings found linearly along the edges.
fn contours(f: &HeightField, z: f64, climb: bool, keep: impl Fn(usize, usize) -> bool) -> Vec<Vec<Vec2>> {
    use std::collections::HashMap;
    // An edge by its lower-left column and whether it runs along X (0) or Y (1).
    type Edge = (usize, usize, u8);
    let (nx, ny) = (f.nx, f.ny);
    let val = |i: usize, j: usize| f.get(i, j).map_or(-1.0, |h| h - z);
    let point = |e: Edge| -> Vec2 {
        let (i, j, dir) = e;
        let (i2, j2) = if dir == 0 { (i + 1, j) } else { (i, j + 1) };
        let (a, b) = (val(i, j), val(i2, j2));
        let t = if (a - b).abs() > 1e-12 { (a / (a - b)).clamp(0.0, 1.0) } else { 0.5 };
        f.centre(i, j).lerp(f.centre(i2, j2), t)
    };
    // Each segment from one edge to another, oriented.
    let mut next: HashMap<Edge, Edge> = HashMap::new();
    for j in 0..ny.saturating_sub(1) {
        for i in 0..nx.saturating_sub(1) {
            let (a, b, c, d) = (val(i, j), val(i + 1, j), val(i + 1, j + 1), val(i, j + 1));
            let high = [a > 0.0, b > 0.0, c > 0.0, d > 0.0];
            let count = high.iter().filter(|h| **h).count();
            if count == 0 || count == 4 || !keep(i, j) {
                continue;
            }
            // The square's edges in order round it: bottom, right, top, left.
            let edges: [Edge; 4] = [(i, j, 0), (i + 1, j, 1), (i, j + 1, 0), (i, j, 1)];
            let crossed: Vec<Edge> = (0..4)
                .filter(|&k| {
                    let (p, q) = (high.get(k).copied().unwrap_or(false), high.get((k + 1) % 4).copied().unwrap_or(false));
                    p != q
                })
                .filter_map(|k| edges.get(k).copied())
                .collect();
            let corners = [f.centre(i, j), f.centre(i + 1, j), f.centre(i + 1, j + 1), f.centre(i, j + 1)];
            // Two crossings: one segment, the part towards its high corners; four (a saddle):
            // two, each cutting a corner off (by the square's middle), the part towards that
            // corner when it is high, away from it when low.
            let highs: Vec<Vec2> = corners.iter().zip(high).filter(|(_, h)| *h).map(|(c, _)| *c).collect();
            let towards_highs = highs.iter().fold(Vec2::ZERO, |s, c| s + *c) * (1.0 / highs.len().max(1) as f64);
            let cut_off = |k: usize| {
                let corner = corners.get(k).copied().unwrap_or(towards_highs);
                if high.get(k).copied().unwrap_or(false) { corner } else { corners.iter().fold(Vec2::ZERO, |s, c| s + *c) * 0.5 - corner }
            };
            let pairs: Vec<(Edge, Edge, Vec2)> = match crossed.as_slice() {
                [e0, e1] => vec![(*e0, *e1, towards_highs)],
                [e0, e1, e2, e3] => {
                    let middle_high = (a + b + c + d) * 0.25 > 0.0;
                    // Edges 0 and 1 meet at corner 1, 1 and 2 at corner 2, and so on round.
                    if middle_high == (a > 0.0) {
                        vec![(*e0, *e1, cut_off(1)), (*e2, *e3, cut_off(3))]
                    } else {
                        vec![(*e1, *e2, cut_off(2)), (*e3, *e0, cut_off(0))]
                    }
                }
                _ => Vec::new(),
            };
            for (e0, e1, mid) in pairs {
                let (p, q) = (point(e0), point(e1));
                // Which side of p → q the part is on.
                let (d, v) = (q - p, mid - p);
                let left = d.x * v.y - d.y * v.x > 0.0;
                // Climb: the part on the right.
                let (s, e) = if left == climb { (e1, e0) } else { (e0, e1) };
                next.insert(s, e);
            }
        }
    }
    // Chains of segments, each from an edge no segment ends at (or round a loop).
    let ends: std::collections::HashSet<Edge> = next.values().copied().collect();
    let mut starts: Vec<Edge> = next.keys().copied().filter(|e| !ends.contains(e)).collect();
    starts.sort_unstable();
    let mut loops: Vec<Edge> = next.keys().copied().filter(|e| ends.contains(e)).collect();
    loops.sort_unstable();
    starts.extend(loops);
    let mut out = Vec::new();
    for s in starts {
        if !next.contains_key(&s) {
            continue;
        }
        let mut run = vec![point(s)];
        let mut at = s;
        while let Some(e) = next.remove(&at) {
            run.push(point(e));
            at = e;
            if at == s {
                break;
            }
        }
        if run.len() > 1 {
            out.push(run);
        }
    }
    out
}

/// The parts of `run` inside `shape` where `near` keeps them.
fn split_inside(run: &[Vec2], shape: &Shape, near: &impl Fn(Vec2) -> bool) -> Vec<Vec<Vec2>> {
    let mut out = Vec::new();
    let mut part: Vec<Vec2> = Vec::new();
    for p in run {
        if shape.inside(*p, shape.dist(*p).max(1e-6)) && near(*p) {
            part.push(*p);
        } else if part.len() > 1 {
            out.push(std::mem::take(&mut part));
        } else {
            part.clear();
        }
    }
    if part.len() > 1 {
        out.push(part);
    }
    out
}

/// Whether `tool` with its tip at `p`, `z` takes more than [`REST_MIN`] off what is `left` of
/// the stock: a column under it standing that much over both the tool's surface there and the
/// part (`part`, on `left`'s columns) with its `allowance` (what stands no higher is the part).
fn bites(left: &HeightField, part: &HeightField, tool: &Tool, p: Vec2, z: f64, allowance: f64) -> bool {
    let (r, corner) = (tool.radius(), tool.corner());
    let over = |d: f64, q: Vec2| left.at(q) - (z + sim::rounded(r, corner, d)).max(part.at(q) + allowance) > REST_MIN;
    if over(0.0, p) {
        return true;
    }
    (1..=4).any(|ring| {
        let d = r * f64::from(ring) / 4.0;
        (0..16).any(|k| over(d, p + Vec2::from_angle(std::f64::consts::TAU * f64::from(k) / 16.0) * d))
    })
}

/// The part seen from above on `left`'s own columns.
fn part_on(left: &HeightField, part: &[[Vec3; 3]], floor: f64) -> Result<HeightField> {
    let far = left.origin + Vec2::new(left.nx as f64 * left.cell, left.ny as f64 * left.cell);
    HeightField::of_triangles(left.origin, far - Vec2::new(left.cell * 0.5, left.cell * 0.5), left.cell, floor, part, false)
}

/// `marks` with each true one spread `by` places either way, onto places whose height in
/// `z` is within `rise` of its own (a run-up does not climb a wall).
fn spread_along(marks: &[bool], z: &[f64], by: usize, rise: f64) -> Vec<bool> {
    let mut out = vec![false; marks.len()];
    for (k, _) in marks.iter().enumerate().filter(|(_, m)| **m) {
        let here = z.get(k).copied().unwrap_or(0.0);
        for j in k.saturating_sub(by)..(k + by + 1).min(marks.len()) {
            if (j == k || z.get(j).is_some_and(|h| (h - here).abs() <= rise))
                && let Some(x) = out.get_mut(j)
            {
                *x = true;
            }
        }
    }
    out
}

/// Over the clearance height to `p`, then down to `z`: at the cutting feed to just over what is
/// `left` of the stock under the tool (where that is known and lower than the retract height:
/// through air, not at the slow plunge feed; a rapid there would be taken for one into
/// material), then at the plunge feed.
fn approach(b: &mut Builder, left: Option<&HeightField>, p: Vec2, z: f64, tool: &Tool, cut: &Cut) -> Result<()> {
    if let Some(left) = left {
        let over = clear_of(left, p, tool);
        // From near by, over what is left between the two rather than up to the retract height
        // and down again, when that is the quicker way.
        if let Some(q) = b.pos.filter(|q| q.z < cut.heights.retract) {
            let from = Vec2::new(q.x, q.y);
            let gap = from.dist(p);
            let steps = ((gap / (tool.radius() * 0.5)).ceil() as usize).clamp(1, MAX_LINK_STEPS);
            let low = (0..=steps).map(|k| clear_of(left, from.lerp(p, k as f64 / steps as f64), tool)).fold(q.z.max(z), f64::max);
            let retract = cut.heights.retract;
            let (rapid, feed, plunge) = (sim::DEFAULT_RAPID_RATE, cut.feeds.cut, cut.feeds.plunge);
            let up_and_over = (retract - q.z) / rapid + gap / rapid + (retract - over.max(z)).max(0.0) / feed + (over.max(z) - z) / plunge;
            let low_over = (low - q.z) / feed + gap / feed + (low - z) / plunge;
            if low < retract && steps < MAX_LINK_STEPS && low_over < up_and_over {
                if low > q.z {
                    b.push(Move::Feed { to: Vec3::new(q.x, q.y, low), feed })?;
                }
                b.push(Move::Feed { to: p.extend(low), feed })?;
                return b.plunge_to(z);
            }
        }
        b.retract()?;
        b.travel_to(p)?;
        if over > z && over < cut.heights.retract && b.pos.is_some_and(|q| q.z > over) {
            if let Some(q) = b.pos.filter(|q| q.z > cut.heights.retract) {
                b.push(Move::Rapid { to: Vec3::new(q.x, q.y, cut.heights.retract) })?;
            }
            b.push(Move::Feed { to: p.extend(over), feed: cut.feeds.cut })?;
        }
        return b.plunge_to(z);
    }
    b.retract()?;
    b.travel_to(p)?;
    b.plunge_to(z)
}

/// How low the tip may come over `p` and stay [`APPROACH_GAP`] clear of what is `left` of the
/// stock: each column less how far the tool's end rises at its distance, its width more for
/// rounding.
fn clear_of(left: &HeightField, p: Vec2, tool: &Tool) -> f64 {
    let (r, corner) = (tool.radius(), tool.corner());
    let mut high = left.at(p);
    for ring in 1..=4 {
        let d = r * f64::from(ring) / 4.0;
        for k in 0..16 {
            let q = p + Vec2::from_angle(std::f64::consts::TAU * f64::from(k) / 16.0) * d;
            high = high.max(left.at(q) - sim::rounded(r, corner, d));
        }
    }
    high + APPROACH_GAP + left.cell
}

/// Most runs put in nearest-first order (more stay in pass order).
const MAX_ORDERED: usize = 5000;
/// Most samples along a link that stays low over what is left (longer ones go up and over).
const MAX_LINK_STEPS: usize = 400;

/// How far over what is left of the stock the tool slows to the plunge feed (mm).
const APPROACH_GAP: f64 = 1.0;

/// Where rest machining may go: within `reach` of a column (over `flo`–`fhi`, `cell` wide or an
/// eighth of the reach)
/// where `left` (what earlier operations left of the stock) stands more than [`REST_MIN`] over
/// the part and its `allowance` (beside the part, what is left is waste, not rest). Everywhere
/// without `left`.
#[allow(clippy::too_many_arguments)]
fn rest_zone(
    left: Option<&HeightField>,
    part: &[[Vec3; 3]],
    flo: Vec2,
    fhi: Vec2,
    cell: f64,
    floor: f64,
    allowance: f64,
    reach: f64,
) -> Result<impl Fn(Vec2) -> bool> {
    let zone = match left {
        None => None,
        Some(left) => {
            // The part on the stock's own columns, so that each is compared where it stands
            // (beside a slope, a column over would differ by the slope alone).
            let seen = part_on(left, part, floor)?;
            // Marked on columns coarser than the finishing ones: it only says whether rest is near.
            let cell = cell.max(reach / 8.0);
            let mut mark = HeightField::block(flo.extend(-1.0), fhi.extend(0.0), cell)?;
            mark.floor = 0.0;
            mark.z.iter_mut().for_each(|z| *z = 0.0);
            for j in 0..left.ny.min(seen.ny) {
                for i in 0..left.nx.min(seen.nx) {
                    let (Some(h), Some(p)) = (left.get(i, j), seen.get(i, j).filter(|p| *p > seen.floor)) else { continue };
                    if h - p - allowance > REST_MIN
                        && let Some((mi, mj)) = mark.column(left.centre(i, j))
                        && let Some(m) = mark.z.get_mut(mj * mark.nx + mi)
                    {
                        *m = 1.0;
                    }
                }
            }
            Some(mark.dilated(reach + cell)?)
        }
    };
    Ok(move |p: Vec2| zone.as_ref().is_none_or(|z| z.at(p) > 0.5))
}

/// The points of `run` to keep: one is left out where it lies within `tol` of the line between
/// the points kept either side of it, on the part's side of that line (on the right of the travel
/// when `part_right`), so that the line never runs nearer the part than the points.
fn thin_xy(run: &[Vec2], tol: f64, part_right: bool) -> Vec<Vec2> {
    let mut out: Vec<Vec2> = Vec::new();
    let Some(&first) = run.first() else { return out };
    out.push(first);
    let mut start = 0;
    while start + 1 < run.len() {
        let Some(&a) = run.get(start) else { break };
        let mut best = start + 1;
        let mut end = start + 2;
        while end < run.len() {
            let Some(&z) = run.get(end) else { break };
            let fits = (start + 1..end).all(|k| {
                run.get(k).is_some_and(|q| {
                    let side = (z - a).cross(*q - a);
                    let towards_part = if part_right { side <= 1e-12 } else { side >= -1e-12 };
                    towards_part && q.extend(0.0).dist_to_segment(a.extend(0.0), z.extend(0.0)) <= tol
                })
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
