//! Dropping a sphere onto a part's triangles: how high its centre has to stay over a point for
//! the sphere not to reach into any of them. A ball end mill grown by an allowance is such a
//! sphere, so its tip goes exactly as low as it may, with nothing missed between columns of a
//! grid. A sphere meets a triangle first at its face (the plane lifted by the radius), at an
//! edge (a cylinder round it), or at a corner (a sphere round it); the highest over all the
//! triangles near the point is where it stops.

use solidcraft_geom::{Vec2, Vec3};

/// Most triangles kept for dropping onto.
const MAX_TRIANGLES: usize = 4_000_000;
/// Most buckets of the index.
const MAX_BUCKETS: usize = 4_000_000;
/// A share of the radius squared under which a touch is only grazing: the sphere passes it.
const GRAZE: f64 = 1e-9;

/// The part's triangles, indexed by squares of the plane for quick lookups round a point.
pub(crate) struct BallDrop {
    tris: Vec<[Vec3; 3]>,
    lo: Vec2,
    size: f64,
    nx: usize,
    ny: usize,
    /// Per square, the triangles whose box (grown by the sphere's radius) reaches into it.
    buckets: Vec<Vec<u32>>,
    radius: f64,
}

impl BallDrop {
    /// The index of `tris` for a sphere of `radius`; `None` when there is nothing to drop onto
    /// or too much to index.
    pub(crate) fn new(tris: &[[Vec3; 3]], radius: f64) -> Option<BallDrop> {
        let tris: Vec<[Vec3; 3]> = tris.iter().filter(|t| t.iter().all(|v| v.is_finite())).copied().collect();
        if tris.is_empty() || tris.len() > MAX_TRIANGLES || !(radius > 0.0 && radius.is_finite()) {
            return None;
        }
        let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for v in tris.iter().flatten() {
            (lo, hi) = (Vec2::new(lo.x.min(v.x), lo.y.min(v.y)), Vec2::new(hi.x.max(v.x), hi.y.max(v.y)));
        }
        let (lo, hi) = (lo - Vec2::new(radius, radius), hi + Vec2::new(radius, radius));
        let size = (radius * 2.0).max(((hi.x - lo.x) * (hi.y - lo.y) / MAX_BUCKETS as f64).sqrt()).max(1e-3);
        let (nx, ny) = (((hi.x - lo.x) / size).ceil().max(1.0) as usize, ((hi.y - lo.y) / size).ceil().max(1.0) as usize);
        let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx.checked_mul(ny)?];
        for (k, t) in tris.iter().enumerate() {
            let (a, b) = (
                Vec2::new(t[0].x.min(t[1].x).min(t[2].x) - radius, t[0].y.min(t[1].y).min(t[2].y) - radius),
                Vec2::new(t[0].x.max(t[1].x).max(t[2].x) + radius, t[0].y.max(t[1].y).max(t[2].y) + radius),
            );
            let (i0, j0) = (((a.x - lo.x) / size).floor().max(0.0) as usize, ((a.y - lo.y) / size).floor().max(0.0) as usize);
            let (i1, j1) =
                ((((b.x - lo.x) / size).floor().max(0.0) as usize).min(nx - 1), (((b.y - lo.y) / size).floor().max(0.0) as usize).min(ny - 1));
            for j in j0..=j1 {
                for i in i0..=i1 {
                    if let Some(bucket) = buckets.get_mut(j * nx + i) {
                        bucket.push(k as u32);
                    }
                }
            }
        }
        Some(BallDrop { tris, lo, size, nx, ny, buckets, radius })
    }

    /// The lowest the sphere's centre may be over `p` without reaching into the part; `None`
    /// where no triangle is within its radius.
    pub(crate) fn centre(&self, p: Vec2) -> Option<f64> {
        let (fi, fj) = ((p.x - self.lo.x) / self.size, (p.y - self.lo.y) / self.size);
        if !(fi >= 0.0 && fj >= 0.0) || fi >= self.nx as f64 || fj >= self.ny as f64 {
            return None;
        }
        let bucket = self.buckets.get(fj as usize * self.nx + fi as usize)?;
        let r = self.radius;
        let mut high: Option<f64> = None;
        let mut up = |z: f64| high = Some(high.map_or(z, |h: f64| h.max(z)));
        for &k in bucket {
            let Some(&[a, b, c]) = self.tris.get(k as usize) else { continue };
            // The face: its plane lifted by the radius, where the touching point lies inside it.
            let n = (b - a).cross(c - a);
            if let Some(n) = n.normalized().map(|n| if n.z < 0.0 { n * -1.0 } else { n })
                && n.z > 1e-9
            {
                // The centre (p, zc) is r from the plane, and touches it at centre − r n.
                let zc = (r + n.dot(a) - n.x * p.x - n.y * p.y) / n.z;
                let touch = Vec3::new(p.x, p.y, zc) - n * r;
                if inside(touch, a, b, c, n) {
                    up(zc);
                }
            }
            // The corners: a sphere round each (a corner only grazed at the sphere's equator lets
            // it pass on down).
            for v in [a, b, c] {
                let d2 = (p.x - v.x).powi(2) + (p.y - v.y).powi(2);
                if d2 < r * r * (1.0 - GRAZE) {
                    up(v.z + (r * r - d2).sqrt());
                }
            }
            // The edges: a cylinder round each, where the touching point is on the edge.
            for (s, e) in [(a, b), (b, c), (c, a)] {
                if let Some(zc) = edge(p, s, e, r) {
                    up(zc);
                }
            }
        }
        high
    }
}

/// Whether `q` (on the triangle's plane, its normal `n`) lies inside the triangle `a b c`.
fn inside(q: Vec3, a: Vec3, b: Vec3, c: Vec3, n: Vec3) -> bool {
    let side = |p: Vec3, s: Vec3| (s - p).cross(q - p).dot(n) >= -1e-12;
    side(a, b) && side(b, c) && side(c, a)
}

/// The height of the centre of a sphere `r` round over `p` that touches the segment `s e`
/// between its ends; `None` where it cannot.
fn edge(p: Vec2, s: Vec3, e: Vec3, r: f64) -> Option<f64> {
    let d = e - s;
    let len2 = d.len2();
    if !(len2 > 1e-18) {
        return None;
    }
    let u = d * (1.0 / len2.sqrt());
    // The centre C = (p, s.z + h) is r from the edge's line: with w = C − s = (wx, wy, h),
    // |w|² − (w·u)² = wx² + wy² + h² − (wx ux + wy uy + h uz)² = r², a quadratic in h.
    let (wx, wy) = (p.x - s.x, p.y - s.y);
    let k = wx * u.x + wy * u.y;
    let qa = 1.0 - u.z * u.z;
    let qb = -2.0 * k * u.z;
    let qc = wx * wx + wy * wy - k * k - r * r;
    // A vertical edge: the sphere stops on its top end, which the corners see.
    if qa.abs() < 1e-12 {
        return None;
    }
    // Only grazed (the line just touches the sphere's equator) the sphere passes on down.
    let disc = qb * qb - 4.0 * qa * qc;
    if disc <= 4.0 * qa * r * r * GRAZE {
        return None;
    }
    let h = (-qb + disc.sqrt()) / (2.0 * qa);
    // The touching point along the edge must lie between its ends.
    let t = (wx * u.x + wy * u.y + h * u.z) / len2.sqrt();
    (0.0..=1.0).contains(&t).then_some(s.z + h)
}
