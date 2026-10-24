//! Adaptive roughing of a part in 3D: the material between the stock and the part cleared level
//! by level at the steady load of [`crate::adaptive`], a flat end mill following the part's
//! shape seen from above (no undercuts). A ball's cut narrows away from its axis, which the
//! planner, clearing discs, does not see: ball and bull-nose roughing are for later.
//!
//! Everything lives on one grid of columns over the area, a tenth of the tool's radius fine: the
//! part's top, the lowest the tool's tip may go over each column without coming nearer the part
//! than the allowance ([`crate::sim::tool_floor`]), and the stock, cut as the levels go. At each
//! level the tool's axis may go where its tip may reach that low inside the boundary, and the
//! material is the stock standing above the level that is not part; the 2.5D planner clears it
//! there, entering on a helix from the top of the stock under it. Levels are a stepdown apart at
//! most, and fall exactly on the part's flat floors so that those come out level.

use solidcraft_geom::{Region2, Vec2, Vec3};

use crate::adaptive::{self, AdaptiveParams, Entry, Grid};
use crate::offset::Shape;
use crate::ops::{Builder, Cut};
use crate::path::{Move, Toolpath};
use crate::piece::Piece;
use crate::sim::{self, HeightField};
use crate::tool::{Tool, ToolKind};
use crate::{CamError, Result, invalid};

/// Most columns of the grid.
const MAX_GRID: usize = 4_000_000;
/// Most levels.
const MAX_LEVELS: usize = 2_000;
/// A flat of the part at least this share of the tool's own area gets a level of its own.
const FLOOR_SHARE: f64 = 1.0;

/// Roughs the material of the stock box `stock` (corners in work coordinates) down to the part
/// `part` (triangles in work coordinates) inside `boundary`, where the tool's axis stays.
/// `params.stock_to_leave` is left on the part all round; a finishing lap and rest machining
/// are not offered here.
pub fn adaptive_3d(
    boundary: &[Region2],
    part: &[[Vec3; 3]],
    stock: (Vec3, Vec3),
    tool: &Tool,
    params: &AdaptiveParams,
    cut: &Cut,
) -> Result<Toolpath> {
    adaptive::check(tool, params, cut)?;
    let p = params;
    if tool.kind != ToolKind::FlatEndmill {
        return Err(CamError::Unsupported("3D adaptive roughing with anything but a flat end mill".into()));
    }
    if p.finish > 0.0 || p.rest_diameter.is_some() {
        return Err(CamError::Unsupported("a finishing lap or rest machining in 3D adaptive roughing".into()));
    }
    if !(cut.stepdown > 0.0 && cut.stepdown.is_finite()) {
        return invalid("the stepdown must be above 0");
    }
    let (lo_s, hi_s) = stock;
    if !lo_s.is_finite() || !hi_s.is_finite() || !(hi_s.x > lo_s.x && hi_s.y > lo_s.y && hi_s.z > lo_s.z) {
        return invalid("the stock must be a box with a size");
    }
    let shape = Shape::from_regions(boundary, p.tolerance)?;
    let (r, s) = (tool.radius(), p.stock_to_leave);
    let (top, bottom) = (cut.heights.top, cut.heights.bottom);
    // The grid: the boundary and a tool's reach (with the allowance) round it.
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(Piece::bounds) {
        (lo, hi) = (Vec2::new(lo.x.min(a.x), lo.y.min(a.y)), Vec2::new(hi.x.max(b.x), hi.y.max(b.y)));
    }
    let around = Vec2::new(r + s, r + s);
    let (lo, hi) = (lo - around, hi + around);
    let cell = (r / 10.0).max(0.01);
    if !lo.is_finite() || !hi.is_finite() || !(((hi.x - lo.x) / cell).ceil() * ((hi.y - lo.y) / cell).ceil() <= MAX_GRID as f64) {
        return Err(CamError::TooLarge("the area is too large for 3D adaptive roughing with so small a tool".into()));
    }
    let floor = bottom.min(lo_s.z) - 1.0;
    let part_top = HeightField::of_triangles(lo, hi, cell, floor, part, true)?;
    let reach = sim::tool_floor(&part_top, tool.kind, r, s)?;
    // The stock over the grid: its box, nothing beside it. A column partly over the box counts
    // as stock, so that the box's edge is cleared on every level, not left as a thin skin.
    let mut stock_top = HeightField::block(lo.extend(floor), hi.extend(hi_s.z.max(floor + 1e-3)), cell)?;
    let (nx, ny) = (stock_top.nx, stock_top.ny);
    if part_top.nx != nx || part_top.ny != ny || reach.z.len() != nx * ny {
        return Err(CamError::Invalid("the part and the stock are not on the same columns".into()));
    }
    for j in 0..ny {
        for i in 0..nx {
            let c = stock_top.centre(i, j);
            let half = cell * 0.5;
            let inside_box = c.x + half > lo_s.x && c.x - half < hi_s.x && c.y + half > lo_s.y && c.y - half < hi_s.y;
            if let Some(z) = stock_top.z.get_mut(j * nx + i) {
                *z = if inside_box { hi_s.z } else { floor };
            }
        }
    }
    // Where the tool's axis may be at all: inside the boundary.
    let inside: Vec<bool> = (0..nx * ny)
        .map(|k| {
            let q = stock_top.centre(k % nx, k / nx);
            shape.inside(q, shape.dist(q).max(1e-6))
        })
        .collect();
    let levels = levels(&part_top, &inside, top, bottom, cut.stepdown, s, r)?;
    let mut b = Builder::new(cut);
    for z in levels {
        let allowed: Vec<bool> =
            (0..nx * ny).map(|k| inside.get(k).copied().unwrap_or(false) && reach.z.get(k).is_some_and(|c| *c <= z + 1e-9)).collect();
        if !allowed.iter().any(|a| *a) {
            continue;
        }
        let material: Vec<bool> = (0..nx * ny)
            .map(|k| stock_top.z.get(k).is_some_and(|h| *h > z + 1e-6) && part_top.z.get(k).is_some_and(|h| *h + s <= z + 1e-6))
            .collect();
        let mut grid = Grid {
            lo,
            cell,
            nx,
            ny,
            room: room(&allowed, nx, ny, cell),
            allowed,
            material,
            cleared: vec![false; nx * ny],
            r,
            shape: None,
            reach: r + s,
        };
        // Every material cell some position of the tool covers.
        let mut reachable = vec![false; nx * ny];
        for k in (0..nx * ny).filter(|&k| grid.allowed.get(k).copied().unwrap_or(false)) {
            for x in grid.disc(grid.centre(k % nx, k / nx), r + cell * 0.5).collect::<Vec<_>>() {
                if let Some(v) = reachable.get_mut(x) {
                    *v = true;
                }
            }
        }
        if !grid.material.iter().zip(&reachable).any(|(m, r)| *m && *r) {
            continue;
        }
        // No lap along the walls takes what a level leaves: it cuts down to a cell's worth.
        let (entry, runs) = match adaptive::plan_to(&mut grid, &reachable, tool, p, cell * cell) {
            Ok(planned) => planned,
            // No room for the tool at this level: the next one down may have some.
            Err(CamError::Invalid(_)) => continue,
            Err(e) => return Err(e),
        };
        let (start, before) = (b.len(), b.pos);
        // An entry starts at the top of the stock under it, so it comes down through all of it.
        let from = |e: Entry| entry_top(&grid, &stock_top, e).min(top).max(z);
        adaptive::pass(&mut b, &grid, entry, &runs, z, &from, cut, p)?;
        // The stock as this level leaves it, for the levels below.
        let mut moves: Vec<Move> = before.map(|q| Move::Rapid { to: q }).into_iter().collect();
        moves.extend_from_slice(b.moves_from(start));
        sim::simulate(&mut stock_top, &Toolpath { moves }, tool)?;
    }
    b.finish()
}

/// The levels from `top` down to `bottom`: on each flat of the part (as large as the tool's own
/// area, inside the boundary) `allowance` above it, and between those evenly, `stepdown` apart
/// at most.
fn levels(part: &HeightField, inside: &[bool], top: f64, bottom: f64, stepdown: f64, allowance: f64, r: f64) -> Result<Vec<f64>> {
    // How much of the area lies at each height (to a thousandth of a millimetre).
    let mut flats: std::collections::BTreeMap<i64, usize> = std::collections::BTreeMap::new();
    for (k, z) in part.z.iter().enumerate() {
        if inside.get(k).copied().unwrap_or(false) && *z > part.floor {
            *flats.entry((z * 1000.0).round() as i64).or_default() += 1;
        }
    }
    let least = (FLOOR_SHARE * std::f64::consts::PI * r * r / (part.cell * part.cell)).ceil() as usize;
    let mut fixed: Vec<f64> = flats
        .iter()
        .filter(|(_, n)| **n >= least)
        .map(|(z, _)| *z as f64 / 1000.0 + allowance)
        .filter(|z| *z > bottom + 1e-6 && *z < top - 1e-6)
        .collect();
    fixed.push(bottom);
    fixed.sort_by(|a, b| b.total_cmp(a));
    let mut out = Vec::new();
    let mut above = top;
    for z in fixed {
        let n = ((above - z) / stepdown - 1e-9).ceil().max(1.0);
        if !(n + out.len() as f64 <= MAX_LEVELS as f64) {
            return Err(CamError::TooLarge(format!("more than {MAX_LEVELS} levels")));
        }
        let n = n as usize;
        out.extend((1..=n).map(|k| if k == n { z } else { above - (above - z) * k as f64 / n as f64 }));
        above = z;
    }
    Ok(out)
}

/// The top of the stock under an entry: where it has to start to come down through all of it.
fn entry_top(g: &Grid, stock: &HeightField, e: Entry) -> f64 {
    let cells: Vec<usize> = match e {
        Entry::Helix { c, r } => g.disc(c, r + g.r).collect(),
        Entry::Ramp { a, b } => {
            let n = ((a.dist(b) / g.r).ceil() as usize).clamp(1, 1000);
            (0..=n).flat_map(|k| g.disc(a.lerp(b, k as f64 / n as f64), g.r).collect::<Vec<_>>()).collect()
        }
        Entry::Plunge(q) => g.disc(q, g.r).collect(),
    };
    cells.into_iter().filter_map(|k| stock.z.get(k).copied()).fold(stock.floor, f64::max)
}

/// How far each allowed cell's centre is from the nearest cell that is not (mm, less half a
/// cell: the room the tool's axis has to move); negative off the allowed cells. Each cell takes
/// the nearest blocked cell its neighbours know of, in two sweeps.
fn room(allowed: &[bool], nx: usize, ny: usize, cell: f64) -> Vec<f64> {
    const NONE: (i64, i64) = (i64::MIN / 4, i64::MIN / 4);
    let mut near: Vec<(i64, i64)> =
        (0..nx * ny).map(|k| if allowed.get(k).copied().unwrap_or(false) { NONE } else { ((k % nx) as i64, (k / nx) as i64) }).collect();
    let d2 = |i: i64, j: i64, (a, b): (i64, i64)| if (a, b) == NONE { i64::MAX } else { (i - a).pow(2) + (j - b).pow(2) };
    let mut relax = |i: i64, j: i64, from: &[(i64, i64)]| {
        let k = j as usize * nx + i as usize;
        let mut best = near.get(k).copied().unwrap_or(NONE);
        for &(di, dj) in from {
            let (a, b) = (i + di, j + dj);
            if a < 0 || b < 0 || a >= nx as i64 || b >= ny as i64 {
                // Off the grid is no room: the border counts as blocked.
                if d2(i, j, (a, b)) < d2(i, j, best) {
                    best = (a, b);
                }
                continue;
            }
            if let Some(&n) = near.get(b as usize * nx + a as usize)
                && d2(i, j, n) < d2(i, j, best)
            {
                best = n;
            }
        }
        if let Some(x) = near.get_mut(k) {
            *x = best;
        }
    };
    for j in 0..ny as i64 {
        for i in 0..nx as i64 {
            relax(i, j, &[(-1, 0), (-1, -1), (0, -1), (1, -1)]);
        }
    }
    for j in (0..ny as i64).rev() {
        for i in (0..nx as i64).rev() {
            relax(i, j, &[(1, 0), (1, 1), (0, 1), (-1, 1)]);
        }
    }
    (0..nx * ny)
        .map(|k| {
            if !allowed.get(k).copied().unwrap_or(false) {
                return -cell;
            }
            let (i, j) = ((k % nx) as i64, (k / nx) as i64);
            let n = near.get(k).copied().unwrap_or(NONE);
            if n == NONE { f64::MAX / 4.0 } else { ((d2(i, j, n) as f64).sqrt() - 0.5) * cell }
        })
        .collect()
}
