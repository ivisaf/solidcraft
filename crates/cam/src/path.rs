//! Toolpaths: the moves of the tool tip's centre in work coordinates.

use std::f64::consts::TAU;

use serde::{Deserialize, Serialize};
use solidcraft_geom::{Vec2, Vec3};

use crate::piece::wrap;

/// Most moves in one toolpath.
pub const MAX_MOVES: usize = 2_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Move {
    /// Positioning at the machine's rapid rate; never through material.
    Rapid { to: Vec3 },
    /// Straight cut at `feed` mm/min.
    Feed { to: Vec3, feed: f64 },
    /// Circular cut about `center` in XY (a helix when Z changes), always less than a full
    /// turn: an arc that ends where it starts is no move at all.
    Arc { to: Vec3, center: Vec2, ccw: bool, feed: f64 },
    /// Pause with the spindle running.
    Dwell { seconds: f64 },
}

impl Move {
    /// Where the tool is after the move (`None`: it stays).
    pub fn target(&self) -> Option<Vec3> {
        match *self {
            Move::Rapid { to } | Move::Feed { to, .. } | Move::Arc { to, .. } => Some(to),
            Move::Dwell { .. } => None,
        }
    }

    /// The same move moved `dx`, `dy` across (an arc's centre with it).
    pub fn shifted(&self, dx: f64, dy: f64) -> Move {
        let by = Vec3::new(dx, dy, 0.0);
        match *self {
            Move::Arc { to, center, ccw, feed } => Move::Arc { to: to + by, center: center + Vec2::new(dx, dy), ccw, feed },
            other => other.lifted(by),
        }
    }

    /// The same move shifted by `by` (an arc's centre stays in XY, so `by` should only lift it).
    pub fn lifted(&self, by: Vec3) -> Move {
        match *self {
            Move::Rapid { to } => Move::Rapid { to: to + by },
            Move::Feed { to, feed } => Move::Feed { to: to + by, feed },
            Move::Arc { to, center, ccw, feed } => Move::Arc { to: to + by, center, ccw, feed },
            Move::Dwell { seconds } => Move::Dwell { seconds },
        }
    }
}

/// Signed angle (positive = counter-clockwise) of an arc move from `from` to `to`; zero when
/// the two coincide in XY.
pub fn arc_sweep(from: Vec3, to: Vec3, center: Vec2, ccw: bool) -> f64 {
    let (a, b) = (Vec2::new(from.x, from.y) - center, Vec2::new(to.x, to.y) - center);
    if a.dist(b) <= 1e-9 {
        return 0.0;
    }
    let delta = wrap(b.angle() - a.angle());
    if ccw { delta } else { delta - TAU }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Toolpath {
    pub moves: Vec<Move>,
}

/// Lengths and time of a toolpath.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PathStats {
    /// Distance travelled at feed (mm).
    pub cut_length: f64,
    /// Distance travelled at rapid (mm).
    pub rapid_length: f64,
    /// Time at feed, including dwells (minutes).
    pub cut_minutes: f64,
    /// Lowest Z reached (mm); `None` for a path without moves.
    pub lowest_z: Option<f64>,
}

impl Toolpath {
    pub fn stats(&self) -> PathStats {
        let mut s = PathStats::default();
        let mut pos: Option<Vec3> = None;
        for m in &self.moves {
            match *m {
                Move::Rapid { to } => s.rapid_length += pos.map_or(0.0, |p| p.dist(to)),
                Move::Feed { to, feed } => {
                    let len = pos.map_or(0.0, |p| p.dist(to));
                    s.cut_length += len;
                    s.cut_minutes += if feed > 0.0 { len / feed } else { 0.0 };
                }
                Move::Arc { to, center, ccw, feed } => {
                    let len = pos.map_or(0.0, |p| {
                        let r = Vec2::new(p.x, p.y).dist(center);
                        (r * arc_sweep(p, to, center, ccw)).hypot(to.z - p.z)
                    });
                    s.cut_length += len;
                    s.cut_minutes += if feed > 0.0 { len / feed } else { 0.0 };
                }
                Move::Dwell { seconds } => s.cut_minutes += seconds.max(0.0) / 60.0,
            }
            if let Some(to) = m.target() {
                s.lowest_z = Some(s.lowest_z.map_or(to.z, |z: f64| z.min(to.z)));
                pos = Some(to);
            }
        }
        s
    }

    /// The path as points for display and checking: `(point, cutting)` where `cutting` says
    /// whether the tool reached the point at feed. Arcs are followed within `tol`.
    pub fn points(&self, tol: f64) -> Vec<(Vec3, bool)> {
        self.timed_points(tol).into_iter().map(|(p, feed, _)| (p, feed.is_some())).collect()
    }

    /// [`Toolpath::points`] with the feed (mm/min) the tool reached each point at (`None` at
    /// rapid) and how long it then dwells there (minutes).
    pub fn timed_points(&self, tol: f64) -> Vec<(Vec3, Option<f64>, f64)> {
        let mut out = Vec::new();
        let mut pos: Option<Vec3> = None;
        for m in &self.moves {
            match *m {
                Move::Rapid { to } => out.push((to, None, 0.0)),
                Move::Feed { to, feed } => out.push((to, Some(feed), 0.0)),
                Move::Arc { to, center, ccw, feed } => {
                    if let Some(p) = pos {
                        let start = Vec2::new(p.x, p.y) - center;
                        let (r, sweep) = (start.len(), arc_sweep(p, to, center, ccw));
                        let tol = tol.max(1e-6);
                        let step = if tol < r { 2.0 * (1.0 - tol / r).acos() } else { TAU / 8.0 };
                        let n = ((sweep.abs() / step.max(1e-3)).ceil() as usize).clamp(1, 4096);
                        for i in 1..n {
                            let t = i as f64 / n as f64;
                            let q = center + Vec2::from_angle(start.angle() + sweep * t) * r;
                            out.push((q.extend(p.z + (to.z - p.z) * t), Some(feed), 0.0));
                        }
                    }
                    out.push((to, Some(feed), 0.0));
                }
                Move::Dwell { seconds } => {
                    if let Some(last) = out.last_mut() {
                        last.2 += seconds.max(0.0) / 60.0;
                    }
                }
            }
            if let Some(to) = m.target() {
                pos = Some(to);
            }
        }
        out
    }
}
