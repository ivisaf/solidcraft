//! Edge nodes: every edge of a body sampled once, so the faces on either side share the very
//! same boundary points (watertight by construction).

use solidcraft_geom::Vec3;
use truck_meshalgo::prelude::*;
use truck_modeling as mt;

use super::size::Rules;
use crate::body::from_p3;

/// Most nodes on one edge.
pub(crate) const MAX_EDGE_NODES: usize = 20_000;
/// Deepest bisection of one edge segment.
const MAX_DEPTH: u32 = 24;

/// Nodes of `edge` from its absolute front to its absolute back: every chord within the
/// tolerance of the curve (checked at the quarter points, so an S-bend in one segment is not
/// missed), at most `h` long, and within the rules' facet angle. The end nodes are the
/// vertices' own points, so edges meeting at a vertex meet exactly.
pub(crate) fn sample_edge(edge: &mt::Edge, rules: &Rules, h: f64) -> Vec<Vec3> {
    let tol = rules.tol;
    let front = from_p3(edge.absolute_front().point());
    let back = from_p3(edge.absolute_back().point());
    let curve = edge.curve();
    if matches!(curve, mt::Curve::Line(_)) {
        // Evenly, as few segments as keep each at most `h`. The ends are the vertices' own
        // points: `front + (back − front) · 1` is not `back` to the last bit, and the face that
        // runs the edge backwards starts its loop with that node, where the face running it
        // forwards takes the next edge's first node (the vertex) — a crack.
        let n = if h > 0.0 { (front.dist(back) / h).ceil().clamp(1.0, (MAX_EDGE_NODES - 1) as f64) as usize } else { 1 };
        let mut pts: Vec<Vec3> = (0..=n).map(|i| front + (back - front) * (i as f64 / n as f64)).collect();
        if let Some(p) = pts.last_mut() {
            *p = back;
        }
        return pts;
    }
    let (t0, t1) = curve.range_tuple();
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
        return vec![front, back];
    }
    // Our evaluator where the curve has a form it knows (no allocation per point); truck's
    // for intersection curves.
    let fast = super::surface::eval_curve(&curve);
    let at = |t: f64| {
        match &fast {
            Some(c) => c.eval(t).map(|(p, _)| p),
            None => Some(from_p3(curve.subs(t))),
        }
        .filter(|p| p.is_finite())
    };
    // A closed curve's chord between its ends is empty: start it from a few segments. An open
    // one starts whole, so an arc the tolerance leaves flat stays one segment.
    let initial: u32 = if edge.absolute_front().id() == edge.absolute_back().id() { 4 } else { 1 };
    let t_at = |i: u32| t0 + (t1 - t0) * f64::from(i) / f64::from(initial);
    let mut ends: Vec<Vec3> = Vec::with_capacity(initial as usize + 1);
    for i in 0..=initial {
        let Some(p) = at(t_at(i)) else { return vec![front, back] };
        ends.push(p);
    }
    // Depth-first bisection, segment by segment, keeping the output ordered. Each segment
    // carries its end points and middle, so every parameter is evaluated once (an
    // intersection curve's point is a Newton solve).
    struct Seg {
        a: f64,
        b: f64,
        pa: Vec3,
        pb: Vec3,
        mid: Option<Vec3>,
        depth: u32,
    }
    let mut out: Vec<Vec3> = vec![front];
    let mut stack: Vec<Seg> = Vec::with_capacity(64);
    for i in (0..initial).rev() {
        let (Some(pa), Some(pb)) = (<[Vec3]>::get(&ends, i as usize).copied(), <[Vec3]>::get(&ends, i as usize + 1).copied()) else {
            return vec![front, back];
        };
        stack.push(Seg { a: t_at(i), b: t_at(i + 1), pa, pb, mid: None, depth: 0 });
    }
    while let Some(sg) = stack.pop() {
        let (a, b, pa, pb) = (sg.a, sg.b, sg.pa, sg.pb);
        let m = 0.5 * (a + b);
        let pm = match sg.mid {
            Some(p) => Some(p),
            None => at(m),
        };
        let (q1, q3) = (at(a + 0.25 * (b - a)), at(a + 0.75 * (b - a)));
        let done = sg.depth >= MAX_DEPTH || out.len() + stack.len() >= MAX_EDGE_NODES;
        let fine = || {
            let sag = [q1, pm, q3].iter().flatten().map(|q| q.dist_to_segment(pa, pb)).fold(0.0, f64::max);
            let short = !(pa.dist(pb) > h * (1.0 + 1e-9));
            // The two half chords of an arc turning θ differ by θ/2.
            let straight = match (rules.turn, pm) {
                (Some(t), Some(q)) => !t.exceeds(2.0 * turn(q - pa, pb - q), sag),
                _ => true,
            };
            short && straight && sag <= tol
        };
        match pm {
            Some(pm) if !done && !fine() => {
                stack.push(Seg { a: m, b, pa: pm, pb, mid: q3, depth: sg.depth + 1 });
                stack.push(Seg { a, b: m, pa, pb: pm, mid: q1, depth: sg.depth + 1 });
            }
            _ => out.push(pb),
        }
    }
    if out.len() < 2 {
        return vec![front, back];
    }
    if let Some(p) = out.last_mut() {
        *p = back;
    }
    out
}

/// Angle (radians) between two directions; zero when either is empty.
fn turn(a: Vec3, b: Vec3) -> f64 {
    let (la, lb) = (a.len(), b.len());
    if !(la > 0.0 && lb > 0.0) {
        return 0.0;
    }
    (a.dot(b) / (la * lb)).clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_edges_are_within_tolerance_and_end_on_vertices() {
        let Ok(cyl) = crate::cylinder(Vec3::ZERO, Vec3::Z, 10.0, 5.0) else { panic!("cylinder") };
        for tol in [0.1, 0.01, 0.001] {
            for e in cyl.solid.edge_iter() {
                let pts = sample_edge(&e, &Rules { tol, max_len: f64::INFINITY, turn: None }, f64::INFINITY);
                assert!(pts.len() >= 2);
                assert_eq!(pts.first().copied(), Some(from_p3(e.absolute_front().point())));
                assert_eq!(pts.last().copied(), Some(from_p3(e.absolute_back().point())));
                if matches!(e.curve(), mt::Curve::Line(_)) {
                    assert_eq!(pts.len(), 2);
                    continue;
                }
                // Circle of radius 10: every chord's middle sags at most `tol` (plus rounding).
                for w in pts.windows(2) {
                    let m = (w[0] + w[1]) * 0.5;
                    let r = (m.x * m.x + m.y * m.y).sqrt();
                    assert!(10.0 - r <= tol * 1.0001, "sag {} at tol {tol}", 10.0 - r);
                }
            }
        }
    }

    #[test]
    fn edges_are_split_to_the_side_length_and_turn() {
        let Ok(cyl) = crate::cylinder(Vec3::ZERO, Vec3::Z, 10.0, 5.0) else { panic!("cylinder") };
        for e in cyl.solid.edge_iter() {
            let pts = sample_edge(&e, &Rules { tol: 1.0, max_len: f64::INFINITY, turn: None }, 1.2);
            for w in pts.windows(2) {
                assert!(w[0].dist(w[1]) <= 1.2 + 1e-9, "side {}", w[0].dist(w[1]));
            }
            if matches!(e.curve(), mt::Curve::Line(_)) {
                // 5 mm in sides of at most 1.2: five equal sides.
                assert_eq!(pts.len(), 6);
            }
            // The ends are the vertices' own points, to the last bit.
            assert_eq!(pts.first().copied(), Some(from_p3(e.absolute_front().point())));
            assert_eq!(pts.last().copied(), Some(from_p3(e.absolute_back().point())));
        }
        // A coarse tolerance alone would leave a few long chords; a 30° facet angle keeps every
        // side within 30° of arc: at least 12 around each of the two circles.
        let thirty =
            Rules { tol: 5.0, max_len: f64::INFINITY, turn: Some(crate::tess::size::Turn { angle: std::f64::consts::PI / 6.0, floor: 0.0 }) };
        let mut seen = std::collections::HashSet::new();
        let sides: usize = cyl
            .solid
            .edge_iter()
            .filter(|e| seen.insert(e.id()) && !matches!(e.curve(), mt::Curve::Line(_)))
            .map(|e| sample_edge(&e, &thirty, f64::INFINITY).len() - 1)
            .sum();
        assert!(sides >= 24, "{sides}");
    }
}
