use std::f64::consts::PI;

use solidcraft_geom::{Loop2, Region2, Vec2, Vec3};

use super::*;
use crate::offset::Shape;

fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

fn polygon(pts: &[(f64, f64)]) -> Region2 {
    Region2 { outer: Loop2::polygon(&pts.iter().map(|p| v(p.0, p.1)).collect::<Vec<_>>()), holes: vec![] }
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Region2 {
    polygon(&[(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
}

fn area(loops: &[Loop2]) -> f64 {
    loops.iter().map(Loop2::signed_area).sum()
}

fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} is not within {tol} of {b}");
}

/// Every point of the offset loops is `d` from the boundary, and every loop is closed.
fn check_offset(regions: &[Region2], d: f64, loops: &[Loop2]) {
    let shape = Shape::from_regions(regions, 0.01).unwrap();
    for lp in loops {
        assert!(lp.is_closed(1e-3), "open loop");
        for p in lp.polyline(0.001) {
            close(shape.dist(p), d.abs(), 1e-3);
        }
    }
}

fn tool(d: f64) -> Tool {
    Tool::new(1, ToolKind::FlatEndmill, d, 20.0)
}

fn cut(top: f64, bottom: f64, stepdown: f64) -> Cut {
    Cut { heights: Heights { clearance: 10.0, retract: 2.0, top, bottom }, feeds: Feeds { cut: 600.0, plunge: 200.0 }, stepdown }
}

fn cutting_points(path: &Toolpath, below: f64) -> Vec<Vec3> {
    path.points(0.005).into_iter().filter(|(p, cutting)| *cutting && p.z < below).map(|(p, _)| p).collect()
}

/// Distance in XY from `q` to the nearest cut made entirely below `below`.
fn dist_to_cuts(path: &Toolpath, below: f64, q: Vec2) -> f64 {
    let flat = |p: Vec3| Vec3::new(p.x, p.y, 0.0);
    let pts = path.points(0.005);
    pts.windows(2)
        .filter(|w| w[1].1 && w[0].0.z < below && w[1].0.z < below)
        .map(|w| q.extend(0.0).dist_to_segment(flat(w[0].0), flat(w[1].0)))
        .fold(f64::INFINITY, f64::min)
}

#[test]
fn square_offsets_to_a_smaller_square_and_to_a_rounded_larger_one() {
    let r = [rect(0.0, 0.0, 40.0, 30.0)];
    let inner = offset_regions(&r, 5.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), 30.0 * 20.0, 1e-6);
    check_offset(&r, 5.0, &inner);
    // Grown by d: area + perimeter × d + π d² (four quarter circles at the corners).
    let outer = offset_regions(&r, -5.0, 0.01).unwrap();
    assert_eq!(outer.len(), 1);
    close(area(&outer), 1200.0 + 140.0 * 5.0 + PI * 25.0, 1e-6);
    check_offset(&r, 5.0, &outer);
}

#[test]
fn inside_corner_of_an_l_shape_is_rounded() {
    // Two 30×10 arms that share a 10×10 square, plus the part of the 5×5 square at the inside
    // corner that is at least 5 from the corner.
    let l = [polygon(&[(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (20.0, 20.0), (20.0, 40.0), (0.0, 40.0)])];
    let inner = offset_regions(&l, 5.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), 500.0 + 25.0 - 25.0 * PI / 4.0, 1e-6);
    check_offset(&l, 5.0, &inner);
}

#[test]
fn circles_and_holes_offset_exactly() {
    let disc = [Region2 { outer: Loop2::circle(v(3.0, -2.0), 10.0), holes: vec![] }];
    close(area(&offset_regions(&disc, 3.0, 0.01).unwrap()), 49.0 * PI, 1e-6);
    close(area(&offset_regions(&disc, -3.0, 0.01).unwrap()), 169.0 * PI, 1e-6);
    // A hole grows as its region shrinks.
    let plate = [Region2 { outer: rect(0.0, 0.0, 40.0, 40.0).outer, holes: vec![Loop2::circle(v(20.0, 20.0), 5.0).reversed()] }];
    let inner = offset_regions(&plate, 3.0, 0.01).unwrap();
    assert_eq!(inner.len(), 2);
    close(area(&inner), 34.0 * 34.0 - 64.0 * PI, 1e-6);
    check_offset(&plate, 3.0, &inner);
    // Hole loops given counter-clockwise are turned round.
    let ccw_hole = [Region2 { outer: rect(0.0, 0.0, 40.0, 40.0).outer, holes: vec![Loop2::circle(v(20.0, 20.0), 5.0)] }];
    close(area(&offset_regions(&ccw_hole, 3.0, 0.01).unwrap()), 34.0 * 34.0 - 64.0 * PI, 1e-6);
}

#[test]
fn narrow_neck_splits_the_offset_and_a_small_region_vanishes() {
    let dumbbell = [polygon(&[
        (0.0, 0.0),
        (20.0, 0.0),
        (20.0, 8.0),
        (30.0, 8.0),
        (30.0, 0.0),
        (50.0, 0.0),
        (50.0, 20.0),
        (30.0, 20.0),
        (30.0, 12.0),
        (20.0, 12.0),
        (20.0, 20.0),
        (0.0, 20.0),
    ])];
    let inner = offset_regions(&dumbbell, 3.0, 0.01).unwrap();
    assert_eq!(inner.len(), 2, "the 4 mm neck is too narrow for a 3 mm offset");
    check_offset(&dumbbell, 3.0, &inner);
    for lp in &inner {
        // A 14×14 square plus a small bulge towards the neck.
        let a = lp.signed_area();
        assert!(a > 196.0 && a < 200.0, "{a}");
    }
    // At 1.5 the neck stays open.
    assert_eq!(offset_regions(&dumbbell, 1.5, 0.01).unwrap().len(), 1);
    assert!(offset_regions(&[rect(0.0, 0.0, 10.0, 10.0)], 6.0, 0.01).unwrap().is_empty());
}

#[test]
fn polygon_with_many_shallow_corners_offsets_to_one_loop() {
    // A regular 720-gon: neighbouring sides turn by half a degree.
    let (n, radius) = (720usize, 25.0);
    let pts: Vec<(f64, f64)> =
        (0..n).map(|i| (radius * (i as f64 * 2.0 * PI / n as f64).cos(), radius * (i as f64 * 2.0 * PI / n as f64).sin())).collect();
    let gon = [polygon(&pts)];
    let half = PI / n as f64;
    let (apothem, perimeter) = (radius * half.cos(), 2.0 * n as f64 * radius * half.sin());
    let inner = offset_regions(&gon, 4.0, 0.01).unwrap();
    assert_eq!(inner.len(), 1);
    close(area(&inner), n as f64 * (apothem - 4.0).powi(2) * half.tan(), 1e-3);
    let outer = offset_regions(&gon, -4.0, 0.01).unwrap();
    assert_eq!(outer.len(), 1);
    close(area(&outer), n as f64 * apothem.powi(2) * half.tan() + perimeter * 4.0 + PI * 16.0, 1e-3);
}

#[test]
fn pocket_stays_inside_the_walls_and_clears_everything_the_tool_can_reach() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let params = PocketParams {
        stepover: 2.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -5.0, 2.0)).unwrap();
    let stats = path.stats();
    assert_eq!(stats.lowest_z, Some(-5.0));
    // Three passes of 5/3 mm.
    let mut levels: Vec<i64> = cutting_points(&path, 0.0).iter().map(|p| (p.z * 1e6).round() as i64).collect();
    levels.sort();
    levels.dedup();
    assert!(levels.len() >= 3 && levels.contains(&-5_000_000) && levels.contains(&-1_666_667), "{levels:?}");
    for p in cutting_points(&path, 0.0) {
        assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "gouge at {p:?}");
    }
    for i in 0..=34 {
        for j in 0..=24 {
            let q = v(3.0 + i as f64, 3.0 + j as f64);
            let d = dist_to_cuts(&path, -5.0 + 1e-6, q);
            assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
        }
    }
}

#[test]
fn pocket_goes_round_an_island() {
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams {
        stepover: 3.0,
        climb: false,
        stock_to_leave: 0.5,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
    let shape = Shape::from_regions(&regions, 0.01).unwrap();
    let pts = cutting_points(&path, 0.0);
    for p in &pts {
        // Tool radius 3 plus 0.5 left on the walls.
        assert!(shape.clear(v(p.x, p.y), 3.5 - 1e-4), "gouge at {p:?}");
    }
    for i in 0..=60 {
        for j in 0..=40 {
            let q = v(i as f64, j as f64);
            if shape.clear(q, 3.5 + 1e-3) {
                let d = dist_to_cuts(&path, 0.0, q);
                assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
            }
        }
    }
}

#[test]
fn contour_runs_round_the_part_with_the_wall_on_the_chosen_side() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let c = cut(0.0, -6.0, 3.0);
    let run = |climb: bool| {
        let params = ContourParams {
            side: Side::Outside,
            climb,
            stock_to_leave: 0.0,
            tolerance: 0.01,
            ramp_angle: 0.0,
            tabs: None,
            finish: 0.0,
            ..Default::default()
        };
        contour(&regions, &tool(6.0), &params, &c).unwrap()
    };
    let climb = run(true);
    // Two passes round the rounded rectangle 3 mm outside the part, and 8 mm of plunging
    // (retract 2 → −3 → −6).
    close(climb.stats().cut_length, 2.0 * (140.0 + 6.0 * PI) + 8.0, 1e-6);
    close(climb.stats().cut_minutes, 2.0 * (140.0 + 6.0 * PI) / 600.0 + 8.0 / 200.0, 1e-9);
    let turns = |path: &Toolpath| {
        let pts = cutting_points(path, -5.9);
        pts.iter().zip(pts.iter().cycle().skip(1)).map(|(a, b)| a.x * b.y - a.y * b.x).sum::<f64>()
    };
    // Climb round the outside of a part is clockwise (wall on the right of travel).
    assert!(turns(&climb) < 0.0);
    assert!(turns(&run(false)) > 0.0);
    for p in cutting_points(&climb, 0.0) {
        let outside = (p.x.clamp(0.0, 40.0) - p.x).hypot(p.y.clamp(0.0, 30.0) - p.y);
        close(outside, 3.0, 1e-6);
    }
    // Inside an opening the tool must fit.
    let inside = ContourParams {
        side: Side::Inside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: None,
        finish: 0.0,
        ..Default::default()
    };
    assert!(contour(&[rect(0.0, 0.0, 5.0, 5.0)], &tool(6.0), &inside, &c).is_err());
    let inner = contour(&regions, &tool(6.0), &inside, &c).unwrap();
    close(inner.stats().cut_length, 2.0 * 2.0 * (34.0 + 24.0) + 8.0, 1e-6);
    // Climb inside an opening is counter-clockwise.
    assert!(turns(&inner) > 0.0);
}

#[test]
fn tabs_stand_on_a_contour_and_a_finishing_pass_takes_off_the_allowance() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let loop_length = 140.0 + 6.0 * PI;
    // Four tabs 5 mm wide and 3 mm high; passes at −2.5, −5, −7.5 and −10.
    let c = cut(0.0, -10.0, 2.5);
    let tabs = Tabs { count: 4, width: 5.0, height: 3.0 };
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        tabs: Some(tabs),
        finish: 0.0,
        ..Default::default()
    };
    let path = contour(&regions, &tool(6.0), &params, &c).unwrap();
    // Four laps; 12 mm down to −10, and over each tab 0.5 mm up and down at −7.5, 3 mm at −10.
    close(path.stats().cut_length, 4.0 * loop_length + 12.0 + 4.0 * (1.0 + 6.0), 1e-6);
    // Below the tabs' top (−7) the tool leaves each tab's width and its own diameter uncut.
    let pts = path.points(0.005);
    let below: f64 =
        pts.windows(2).filter(|w| w[1].1 && w[0].0.z < -7.001 && w[1].0.z < -7.001).map(|w| (w[0].0.x - w[1].0.x).hypot(w[0].0.y - w[1].0.y)).sum();
    close(below, 2.0 * (loop_length - 4.0 * 11.0), 0.05);
    assert!(pts.iter().all(|(p, _)| p.z >= -10.0 - 1e-9));
    // Tabs as high as the depth, or none at all on a loop: refused; a loop too short for them has none.
    for bad in [Tabs { height: 10.0, ..tabs }, Tabs { count: 0, ..tabs }, Tabs { width: 0.0, ..tabs }] {
        assert!(contour(&regions, &tool(6.0), &ContourParams { tabs: Some(bad), ..params }, &c).is_err());
    }
    let crowded = contour(&regions, &tool(6.0), &ContourParams { tabs: Some(Tabs { count: 50, ..tabs }), ..params }, &c).unwrap();
    close(crowded.stats().cut_length, 4.0 * loop_length + 12.0, 1e-6);

    // A 0.5 mm finishing allowance: the depth passes run 3.5 mm out, one more lap 3 mm out at the full depth.
    let c = cut(0.0, -6.0, 3.0);
    let finished = ContourParams { tabs: None, finish: 0.5, ..params };
    let path = contour(&regions, &tool(6.0), &finished, &c).unwrap();
    close(path.stats().cut_length, 2.0 * (140.0 + 7.0 * PI) + loop_length + 16.0, 1e-6);
    let off = |p: &Vec3| (p.x.clamp(0.0, 40.0) - p.x).hypot(p.y.clamp(0.0, 30.0) - p.y);
    assert!(cutting_points(&path, -5.9).iter().any(|p| (off(p) - 3.0).abs() < 1e-6));
    assert!(cutting_points(&path, 0.0).iter().filter(|p| p.z > -5.9).all(|p| (off(p) - 3.5).abs() < 1e-6));
    let on_line = ContourParams { side: Side::On, ..finished };
    assert!(contour(&regions, &tool(6.0), &on_line, &c).is_err());

    // A pocket's depth passes keep the allowance off its walls; the last pass along them takes it.
    let pocketed = PocketParams {
        stepover: 2.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.5,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &pocketed, &c).unwrap();
    let left = |pts: Vec<Vec3>| pts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    close(left(cutting_points(&path, -2.9).into_iter().filter(|p| p.z > -3.1).collect()), 3.5, 1e-6);
    close(left(cutting_points(&path, -5.9)), 3.0, 1e-6);
}

#[test]
fn dogbones_reach_the_corners_lead_arcs_start_off_the_part_and_an_onion_skin_holds_it() {
    let hole = [rect(0.0, 0.0, 40.0, 20.0)];
    let corners = [v(0.0, 0.0), v(40.0, 0.0), v(40.0, 20.0), v(0.0, 20.0)];
    let inside = ContourParams { side: Side::Inside, climb: true, tolerance: 0.01, ..Default::default() };
    let c = cut(0.0, -6.0, 3.0);
    // Without dogbones the 6 mm tool stays √2 × its radius from each corner; with them its edge reaches it.
    let plain = contour(&hole, &tool(6.0), &inside, &c).unwrap();
    let boned = contour(&hole, &tool(6.0), &ContourParams { dogbones: true, ..inside }, &c).unwrap();
    for corner in corners {
        close(dist_to_cuts(&plain, -5.9, corner), 3.0 * 2f64.sqrt(), 0.02);
        close(dist_to_cuts(&boned, -5.9, corner), 3.0, 0.02);
    }
    // A pocket's wall ring too.
    let pocketed = PocketParams { stepover: 2.5, climb: true, tolerance: 0.01, dogbones: true, ..Default::default() };
    let p = pocket(&hole, &tool(6.0), &pocketed, &c).unwrap();
    close(dist_to_cuts(&p, -5.9, v(0.0, 0.0)), 3.0, 0.02);
    // Lead arcs: round a part, the tool goes down off it and never comes nearer than its radius.
    let part = [rect(0.0, 0.0, 40.0, 20.0)];
    let outside = ContourParams { side: Side::Outside, climb: true, tolerance: 0.01, lead: 4.0, ..Default::default() };
    let led = contour(&part, &tool(6.0), &outside, &c).unwrap();
    let off = |q: Vec3| {
        let (dx, dy) = ((-q.x).max(q.x - 40.0).max(0.0), (-q.y).max(q.y - 20.0).max(0.0));
        dx.hypot(dy)
    };
    assert!(cutting_points(&led, -0.01).iter().all(|q| off(*q) > 3.0 - 0.02), "into the part");
    let plunges: Vec<Vec3> = led
        .moves
        .windows(2)
        .filter_map(|w| match (w[0], w[1]) {
            (_, Move::Feed { to, .. }) if w[0].target().is_some_and(|a| (a.x - to.x).abs() < 1e-9 && (a.y - to.y).abs() < 1e-9 && to.z < a.z) => {
                Some(to)
            }
            _ => None,
        })
        .collect();
    assert!(!plunges.is_empty() && plunges.iter().all(|q| off(*q) > 3.0 + 1.0), "down off the wall: {plunges:?}");
    // An onion skin: the passes stop 0.5 above the bottom; taken off last when asked.
    let skin = ContourParams { side: Side::Outside, climb: true, tolerance: 0.01, skin: 0.5, ..Default::default() };
    let held = contour(&part, &tool(6.0), &skin, &c).unwrap();
    close(held.stats().lowest_z.unwrap(), -5.5, 1e-9);
    let freed = contour(&part, &tool(6.0), &ContourParams { skin_last: true, ..skin }, &c).unwrap();
    close(freed.stats().lowest_z.unwrap(), -6.0, 1e-9);
    assert!(cutting_points(&freed, -5.4).iter().any(|q| (q.z + 5.5).abs() < 1e-9), "the skin's level first");
    // What it cannot do is refused.
    // With tabs too: down off the part, over each tab in the passes below its top, never into the part.
    let tabs = Some(Tabs { count: 2, width: 4.0, height: 1.0 });
    let held = contour(&part, &tool(6.0), &ContourParams { tabs, ..outside }, &c).unwrap();
    assert!(cutting_points(&held, -0.01).iter().all(|q| off(*q) > 3.0 - 0.02), "into the part");
    let bottom = cutting_points(&held, -5.99);
    let over = cutting_points(&held, -4.99).iter().filter(|q| (q.z + 5.0).abs() < 1e-9).count();
    assert!(!bottom.is_empty() && over > 0, "the last pass at the bottom, rising to the tabs' top over them");
    for bad in [ContourParams { skin: 6.0, ..skin }, ContourParams { skin: -1.0, ..skin }, ContourParams { lead: f64::NAN, ..outside }] {
        assert!(contour(&part, &tool(6.0), &bad, &c).is_err(), "{bad:?}");
    }
}

#[test]
fn a_v_carve_goes_deeper_away_from_the_walls_down_to_its_limit() {
    let mut vbit = Tool::new(5, ToolKind::VBit, 10.0, 6.0);
    vbit.tip_angle = Some(90.0);
    // A 10 mm wide strip: at 90° the middle is 5 mm from the walls, so 5 mm deep.
    let strip = [rect(0.0, 0.0, 40.0, 10.0)];
    let params = VCarveParams { stepover: 0.25, max_depth: None, climb: true, tolerance: 0.01 };
    let c = cut(0.0, -20.0, 10.0);
    let path = vcarve(&strip, &vbit, &params, &c).unwrap();
    let lowest = path.stats().lowest_z.unwrap();
    assert!((lowest + 5.0).abs() < 0.3, "{lowest}");
    // Each cutting point is as deep as it is far from the nearer long wall (45°): never past it.
    for q in cutting_points(&path, -0.01) {
        let wall = q.y.min(10.0 - q.y).min(q.x).min(40.0 - q.x);
        assert!(-q.z <= wall + 1e-6, "{q:?} deeper than the wall allows");
    }
    // Held to 2 mm deep, and in passes no deeper than the stepdown.
    let held = vcarve(&strip, &vbit, &VCarveParams { max_depth: Some(2.0), ..params }, &cut(0.0, -20.0, 0.75)).unwrap();
    assert!((held.stats().lowest_z.unwrap() + 2.0).abs() < 1e-9);
    // A flat end mill is no V-bit.
    assert!(vcarve(&strip, &tool(6.0), &params, &c).is_err());
    assert!(vcarve(&strip, &vbit, &VCarveParams { stepover: 0.0, ..params }, &c).is_err());
}

#[test]
fn a_thread_rises_one_pitch_a_turn_out_to_its_full_depth() {
    // M8 × 1.25 in a 6.8 mm hole, with a 6 mm thread mill: from the bottom up, in two passes.
    let mill = tool(6.0);
    let params = ThreadParams { pitch: 1.25, diameter: 8.0, internal: true, climb: true, passes: 2, left_hand: false };
    let c = cut(0.0, -10.0, 10.0);
    let path = thread_mill(&[v(20.0, 20.0)], &mill, &params, &c).unwrap();
    let arcs: Vec<(Vec3, bool)> = path
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Arc { to, ccw, .. } => Some((*to, *ccw)),
            _ => None,
        })
        .collect();
    assert!(!arcs.is_empty() && arcs.iter().all(|(_, ccw)| *ccw), "climb inside: counter-clockwise going up");
    // Each half turn rises half a pitch; the last pass runs at the major diameter less the tool's radius.
    for w in arcs.windows(2).filter(|w| w[1].0.z > w[0].0.z + 1e-9) {
        assert!((w[1].0.z - w[0].0.z - 0.625).abs() < 1e-6 || (w[1].0.z + 0.0).abs() < 1e-9, "{w:?}");
    }
    let farthest = arcs.iter().map(|(p, _)| (p.x - 20.0).hypot(p.y - 20.0)).fold(0.0, f64::max);
    assert!((farthest - (4.0 - 3.0)).abs() < 1e-6, "{farthest}");
    assert!((path.stats().lowest_z.unwrap() + 10.0).abs() < 1e-9);
    // Round a boss: outside it, and conventional turns clockwise going up.
    let bolt = thread_mill(&[v(0.0, 0.0)], &mill, &ThreadParams { internal: false, climb: false, ..params }, &c).unwrap();
    let nearest = bolt
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Arc { to, .. } => Some(to.x.hypot(to.y)),
            _ => None,
        })
        .fold(f64::INFINITY, f64::min);
    assert!(nearest >= 4.0 - 1.2269 * 0.625 + 3.0 - 1e-6, "{nearest}");
    // Left-hand: turning the same way, it goes from the top down, so the helix winds the other way.
    let left = thread_mill(&[v(20.0, 20.0)], &mill, &ThreadParams { left_hand: true, ..params }, &c).unwrap();
    let zs: Vec<f64> = left
        .moves
        .iter()
        .filter_map(|m| match m {
            Move::Arc { to, ccw, .. } => Some((to.z, *ccw)),
            _ => None,
        })
        .map(|(z, ccw)| {
            assert!(ccw);
            z
        })
        .collect();
    // Down in each of the two passes: up again only where the second starts.
    assert!(zs.windows(2).filter(|w| w[1] > w[0] + 1e-9).count() == 1 && (zs.last().copied().unwrap_or(0.0) + 10.0).abs() < 1e-9, "{zs:?}");
    // A tool too large, a pitch past the tool, no passes: refused.
    assert!(thread_mill(&[v(0.0, 0.0)], &tool(7.0), &params, &c).is_err());
    assert!(thread_mill(&[v(0.0, 0.0)], &mill, &ThreadParams { pitch: 7.0, ..params }, &c).is_err());
    assert!(thread_mill(&[v(0.0, 0.0)], &mill, &ThreadParams { passes: 0, ..params }, &c).is_err());
}

/// The steepest descent of a cutting move (radians), and whether any goes straight down below `top`.
fn steepest(path: &Toolpath, top: f64) -> (f64, bool) {
    let pts = path.points(0.002);
    let (mut angle, mut plunges) = (0.0f64, false);
    for w in pts.windows(2) {
        let (a, b) = (w[0].0, w[1].0);
        if !w[1].1 || b.z >= a.z - 1e-9 || a.z > top + 1e-9 {
            continue;
        }
        let run = (b.x - a.x).hypot(b.y - a.y);
        plunges |= run < 1e-9;
        angle = angle.max((a.z - b.z).atan2(run));
    }
    (angle, plunges)
}

#[test]
fn ramps_replace_plunges_into_material() {
    let ramp = 3.0f64.to_radians();
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    // Round the part: each pass ramps once round the 140 + 6π loop, then cuts it flat.
    let c = cut(0.0, -6.0, 3.0);
    let params = ContourParams {
        side: Side::Outside,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: ramp,
        tabs: None,
        finish: 0.0,
        ..Default::default()
    };
    let path = contour(&regions, &tool(6.0), &params, &c).unwrap();
    let lap = 140.0 + 6.0 * PI;
    close(path.stats().cut_length, 2.0 * (lap.hypot(3.0) + lap) + 2.0, 1e-6);
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-6.0));

    // In a pocket the first ring is short, so the ramp goes round it several times.
    let params = PocketParams {
        stepover: 2.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: ramp,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -5.0, 2.5)).unwrap();
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-5.0));
    for p in cutting_points(&path, 0.0) {
        assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "gouge at {p:?}");
    }
    // The floor is still cleared: the flat pass follows the ramp.
    for i in 0..=34 {
        for j in 0..=24 {
            let q = v(3.0 + i as f64, 3.0 + j as f64);
            assert!(dist_to_cuts(&path, -5.0 + 1e-6, q) <= 3.0 + 0.01, "{q:?}");
        }
    }
    // A ring too short to ramp along is plunged into rather than refused.
    let tight = pocket(&[rect(0.0, 0.0, 6.004, 6.004)], &tool(6.0), &params, &cut(0.0, -5.0, 2.5)).unwrap();
    assert_eq!(tight.stats().lowest_z, Some(-5.0));
    assert!(steepest(&tight, 0.0).1);
    assert!(pocket(&regions, &tool(6.0), &PocketParams { ramp_angle: 2.0, ..params }, &c).is_err());
}

/// A 60 × 40 plate with its top at 0 and a boss of radius 12 at (30, 20) rising to 12, seen from above.
fn plate_with_boss(cell: f64) -> HeightField {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let mut tris = vec![[p(0.0, 0.0, 0.0), p(60.0, 0.0, 0.0), p(60.0, 40.0, 0.0)], [p(0.0, 0.0, 0.0), p(60.0, 40.0, 0.0), p(0.0, 40.0, 0.0)]];
    let n = 64;
    for k in 0..n {
        let at = |k: usize| {
            let a = 2.0 * PI * k as f64 / n as f64;
            p(30.0 + 12.0 * a.cos(), 20.0 + 12.0 * a.sin(), 12.0)
        };
        tris.push([p(30.0, 20.0, 12.0), at(k), at(k + 1)]);
    }
    HeightField::of_triangles(v(-15.0, -15.0), v(75.0, 55.0), cell, -1.0, &tris, true).unwrap()
}

#[test]
fn facing_goes_round_what_rises_above_its_passes() {
    let part = plate_with_boss(0.25);
    assert_eq!(part.at(v(30.0, 20.0)), 12.0);
    assert_eq!(part.at(v(5.0, 5.0)), 0.0);
    assert_eq!(part.at(v(70.0, 5.0)), -1.0);
    // From a stock top at 13 down to the plate's top, passes 4.33 apart.
    let c = Cut {
        heights: Heights { clearance: 25.0, retract: 15.0, top: 13.0, bottom: 0.0 },
        feeds: Feeds { cut: 800.0, plunge: 300.0 },
        stepdown: 5.0,
    };
    let params = FaceParams { stepover: 4.0, inside: false, margin: 3.0, round: None };
    let t = tool(6.0);
    // Facing alone runs through the boss; round it, the tool's edge stays off it.
    let straight = face(v(0.0, 0.0), v(60.0, 40.0), &t, &params, &c).unwrap();
    assert!(gouge(&straight, &t, &part) > 8.0);
    let around = face_around(v(0.0, 0.0), v(60.0, 40.0), &t, &params, &c, &part).unwrap();
    assert_eq!(gouge(&around, &t, &part), 0.0);
    let near = cutting_points(&around, 12.0).iter().map(|q| v(q.x, q.y).dist(v(30.0, 20.0))).fold(f64::INFINITY, f64::min);
    assert!((15.0..16.0).contains(&near), "{near}");
    // The plate round the boss is cut down to its top, out to the area's edges.
    assert_eq!(around.stats().lowest_z, Some(0.0));
    let floor: Vec<Vec3> = cutting_points(&around, 0.1).into_iter().filter(|q| q.z == 0.0).collect();
    assert!(floor.iter().any(|q| q.x <= -2.9) && floor.iter().any(|q| q.x >= 62.9), "{}", floor.len());
    // With the part below the bottom, it is facing as it was.
    let level = Cut { heights: Heights { top: 13.0, bottom: 12.0, ..c.heights }, ..c };
    assert_eq!(
        face_around(v(0.0, 0.0), v(60.0, 40.0), &t, &params, &level, &part).unwrap(),
        face(v(0.0, 0.0), v(60.0, 40.0), &t, &params, &level).unwrap()
    );
}

#[test]
fn face_cuts_over_the_boundary_or_stays_inside_it() {
    let c =
        Cut { heights: Heights { clearance: 10.0, retract: 3.0, top: 1.0, bottom: 0.0 }, feeds: Feeds { cut: 800.0, plunge: 300.0 }, stepdown: 1.0 };
    let span = |path: &Toolpath| {
        let pts = cutting_points(path, 0.5);
        pts.iter().fold((v(f64::INFINITY, f64::INFINITY), v(f64::NEG_INFINITY, f64::NEG_INFINITY)), |m, p| {
            (v(m.0.x.min(p.x), m.0.y.min(p.y)), v(m.1.x.max(p.x), m.1.y.max(p.y)))
        })
    };
    // Over by the tool's radius: the centre runs from 10 before the 100 × 50 area to 10 past it,
    // in passes from Y 0 to Y 50: ceil(50/15) + 1 = 5 of them 12.5 apart, 120 long, and one 3 mm plunge.
    let over = FaceParams { stepover: 15.0, inside: false, margin: 10.0, round: None };
    let path = face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), &over, &c).unwrap();
    close(path.stats().cut_length, 5.0 * 120.0 + 4.0 * 12.5 + 3.0, 1e-9);
    assert_eq!((span(&path), path.stats().lowest_z), ((v(-10.0, 0.0), v(110.0, 50.0)), Some(0.0)));
    // Over by nothing: the area itself is cut whole, the tool's nose a radius past its ends.
    let path = face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), &FaceParams { margin: 0.0, ..over }, &c).unwrap();
    assert_eq!(span(&path), (v(0.0, 10.0), v(100.0, 40.0)));
    // Inside by 2: the tool's edge stays 2 from the boundary all round.
    let inside = FaceParams { stepover: 15.0, inside: true, margin: 2.0, round: None };
    let path = face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), &inside, &c).unwrap();
    assert_eq!(span(&path), (v(12.0, 12.0), v(88.0, 38.0)));
    // An area narrower than the tool is one pass down its middle; inside, the tool must fit.
    let path = face(v(0.0, 0.0), v(100.0, 10.0), &tool(20.0), &FaceParams { margin: 0.0, ..over }, &c).unwrap();
    assert_eq!(span(&path), (v(0.0, 5.0), v(100.0, 5.0)));
    assert!(face(v(0.0, 0.0), v(100.0, 10.0), &tool(20.0), &inside, &c).is_err());
    assert!(face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), &FaceParams { stepover: 25.0, ..over }, &c).is_err(), "stepover above the tool diameter");
    assert!(face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), &FaceParams { margin: -1.0, ..over }, &c).is_err());
    // A round boundary (a bar 80 across in its 80 × 80 box): each pass runs across the circle only,
    // the tool's centre at most the margin beyond it, or the margin and its radius inside it.
    let reach = |path: &Toolpath| cutting_points(path, 0.5).iter().map(|p| v(p.x, p.y).dist(v(50.0, 50.0))).fold(0.0, f64::max);
    let round = FaceParams { round: Some((v(50.0, 50.0), 40.0)), ..over };
    let path = face(v(10.0, 10.0), v(90.0, 90.0), &tool(20.0), &round, &c).unwrap();
    let boxed = face(v(10.0, 10.0), v(90.0, 90.0), &tool(20.0), &over, &c).unwrap();
    assert!(reach(&path) <= 50.0 + 1e-6 && reach(&boxed) > 60.0, "{} {}", reach(&path), reach(&boxed));
    assert!(path.stats().cut_length < boxed.stats().cut_length);
    let path = face(v(10.0, 10.0), v(90.0, 90.0), &tool(20.0), &FaceParams { round: Some((v(50.0, 50.0), 40.0)), ..inside }, &c).unwrap();
    assert!(reach(&path) <= 28.0 + 1e-6, "{}", reach(&path));
    assert!(face(v(10.0, 10.0), v(90.0, 90.0), &tool(20.0), &FaceParams { round: Some((v(50.0, 50.0), 11.0)), ..inside }, &c).is_err());
}

#[test]
fn pocket_with_passes_up_to_a_diameter_apart_still_clears_the_middle() {
    // A 40 × 30 pocket, a 6 mm tool, passes 5 apart: rings 3, 8 and 13 from the walls. The last
    // leaves a strip down the middle (15 from the long walls) more than a radius from it.
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    for (stepover, name) in [(5.0, "wide"), (6.0, "a diameter"), (4.0, "moderate")] {
        let params = PocketParams {
            stepover,
            climb: true,
            stock_to_leave: 0.0,
            tolerance: 0.01,
            ramp_angle: 0.0,
            finish: 0.0,
            rest_diameter: None,
            ..Default::default()
        };
        let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
        for p in cutting_points(&path, 0.0) {
            assert!(p.x >= 3.0 - 1e-6 && p.x <= 37.0 + 1e-6 && p.y >= 3.0 - 1e-6 && p.y <= 27.0 + 1e-6, "{name}: gouge at {p:?}");
        }
        for i in 0..=68 {
            for j in 0..=48 {
                let q = v(3.0 + i as f64 * 0.5, 3.0 + j as f64 * 0.5);
                let d = dist_to_cuts(&path, -2.0 + 1e-6, q);
                assert!(d <= 3.0 + 0.01, "{name}: {q:?} is {d} from the nearest pass");
            }
        }
    }
    // Two rooms joined by a wide door, one room narrower: each is cleared to its own middle, round an island too.
    let rooms = Region2 {
        outer: polygon(&[(0.0, 0.0), (30.0, 0.0), (30.0, 8.0), (60.0, 8.0), (60.0, 26.0), (30.0, 26.0), (30.0, 40.0), (0.0, 40.0)]).outer,
        holes: vec![Loop2::circle(v(15.0, 20.0), 4.0)],
    };
    let params = PocketParams {
        stepover: 5.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(std::slice::from_ref(&rooms), &tool(6.0), &params, &cut(0.0, -2.0, 2.0)).unwrap();
    let shape = Shape::from_regions(std::slice::from_ref(&rooms), 0.01).unwrap();
    for p in cutting_points(&path, 0.0) {
        assert!(shape.clear(v(p.x, p.y), 3.0 - 1e-4), "gouge at {p:?}");
    }
    for i in 0..=120 {
        for j in 0..=80 {
            let q = v(i as f64 * 0.5, j as f64 * 0.5);
            if shape.clear(q, 3.0 + 1e-3) {
                let d = dist_to_cuts(&path, -2.0 + 1e-6, q);
                assert!(d <= 3.0 + 0.01, "{q:?} is {d} from the nearest pass");
            }
        }
    }
    let too_wide = PocketParams { stepover: 6.5, ..params };
    assert!(pocket(&regions, &tool(6.0), &too_wide, &cut(0.0, -2.0, 2.0)).is_err(), "passes more than a diameter apart");
}

#[test]
fn chamfer_cuts_the_top_edge_with_the_tool_s_cone() {
    // A 90° V-bit 10 mm across: a 1 mm chamfer is 1 mm deep, its point 0.5 mm lower and 0.5 mm off the wall.
    let vbit = |angle: f64| Tool { tip_angle: Some(angle), ..Tool::new(1, ToolKind::VBit, 10.0, 5.0) };
    let c = cut(0.0, -10.0, 2.0);
    let params = ChamferParams { side: Side::Outside, climb: true, width: 1.0, tip_offset: 0.5, tolerance: 0.01, ramp_angle: 0.0 };
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let path = chamfer(&regions, &vbit(90.0), &params, &c).unwrap();
    let pts = cutting_points(&path, -1.4);
    assert!(!pts.is_empty());
    for p in &pts {
        close(p.z, -1.5, 1e-9);
        close((p.x.clamp(0.0, 40.0) - p.x).hypot(p.y.clamp(0.0, 30.0) - p.y), 0.5, 1e-6);
    }
    // The stock: the wall cut back to 1 mm down, halfway across 0.5 mm down, untouched beyond 1 mm in.
    let mut stock = HeightField::block(Vec3::new(-5.0, -5.0, -10.0), Vec3::new(45.0, 35.0, 0.0), 0.05).unwrap();
    simulate(&mut stock, &path, &vbit(90.0)).unwrap();
    close(stock.at(v(0.025, 15.0)), -0.975, 0.03);
    close(stock.at(v(0.525, 15.0)), -0.475, 0.03);
    close(stock.at(v(1.2, 15.0)), 0.0, 1e-9);
    // A 60° bit makes the same width deeper; the inside of an opening is chamfered inside it.
    let steep = chamfer(&regions, &vbit(60.0), &params, &c).unwrap();
    assert!(steep.stats().lowest_z.unwrap() < -1.5 - 0.5);
    let inside = chamfer(&regions, &vbit(90.0), &ChamferParams { side: Side::Inside, ..params }, &c).unwrap();
    assert!(cutting_points(&inside, -1.4).iter().all(|p| p.x > 0.0 && p.x < 40.0 && p.y > 0.0 && p.y < 30.0));
    // Too wide for the tool, a flat tool, on the line, a bad width: refused.
    assert!(chamfer(&regions, &vbit(90.0), &ChamferParams { width: 5.0, ..params }, &c).is_err());
    assert!(chamfer(&regions, &tool(6.0), &params, &c).is_err());
    assert!(chamfer(&regions, &vbit(90.0), &ChamferParams { side: Side::On, ..params }, &c).is_err());
    for width in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(chamfer(&regions, &vbit(90.0), &ChamferParams { width, ..params }, &c).is_err());
    }
}

#[test]
fn rest_machining_cuts_only_what_a_larger_tool_left() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let c = cut(0.0, -4.0, 2.0);
    let big = PocketParams {
        stepover: 4.0,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let first = pocket(&regions, &tool(10.0), &big, &c).unwrap();
    let small = PocketParams { stepover: 1.5, rest_diameter: Some(10.0), ..big };
    let rest = pocket(&regions, &tool(3.0), &small, &c).unwrap();
    // The 10 mm tool leaves the corners rounded 5 mm; the 3 mm tool goes only there.
    let corners = [v(0.0, 0.0), v(40.0, 0.0), v(40.0, 30.0), v(0.0, 30.0)];
    let pts = cutting_points(&rest, -0.5);
    assert!(!pts.is_empty());
    for p in &pts {
        let near = corners.iter().map(|c| c.dist(Vec2::new(p.x, p.y))).fold(f64::INFINITY, f64::min);
        assert!(near < 8.0, "{p:?} is {near} mm from a corner");
    }
    // Across the material it cuts far less than the larger tool did.
    let across = |path: &Toolpath| -> f64 {
        let pts = path.points(0.01);
        pts.windows(2).filter(|w| w[1].1 && w[0].0.z < -0.5 && w[1].0.z < -0.5).map(|w| (w[0].0.x - w[1].0.x).hypot(w[0].0.y - w[1].0.y)).sum()
    };
    assert!(across(&rest) < across(&first) * 0.5, "{} {}", across(&rest), across(&first));
    // After both, the corners are cut down to a 1.5 mm radius; the larger tool alone left them standing.
    let block = HeightField::block(Vec3::new(-1.0, -1.0, -10.0), Vec3::new(41.0, 31.0, 0.0), 0.1).unwrap();
    let mut stock = block.clone();
    simulate(&mut stock, &first, &tool(10.0)).unwrap();
    let corner = v(1.0, 1.0);
    close(stock.at(corner), 0.0, 1e-9);
    simulate(&mut stock, &rest, &tool(3.0)).unwrap();
    close(stock.at(corner), -4.0, 1e-9);
    // The small tool never plunges into uncut material: every straight-down feed is where the
    // larger tool had cleared, a radius away from what was left.
    let mut after_first = block.clone();
    simulate(&mut after_first, &first, &tool(10.0)).unwrap();
    let mut at = Vec3::ZERO;
    for m in &rest.moves {
        let Some(to) = m.target() else { continue };
        if matches!(m, Move::Feed { .. }) && to.x == at.x && to.y == at.y && to.z < at.z {
            assert!(after_first.at(Vec2::new(to.x, to.y)) <= to.z + 1e-6, "plunge into material at {to:?}");
        }
        at = to;
    }
    // Between corners it goes over at the retract height, not up to the clearance height.
    let highest_between = rest.moves.iter().filter_map(|m| m.target()).map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    assert!(rest.stats().rapid_length < 2.0 * 140.0 + 4.0 * 2.0 * 12.0, "{}", rest.stats().rapid_length);
    assert!(highest_between <= c.heights.clearance + 1e-9);
    // A narrow slot the larger tool never entered: the small one clears it, ramping in.
    let slot = [rect(0.0, 0.0, 30.0, 6.0)];
    let ramped = PocketParams { ramp_angle: 3.0f64.to_radians(), ..small };
    let in_slot = pocket(&slot, &tool(3.0), &ramped, &c).unwrap();
    assert!(cutting_points(&in_slot, -3.9).len() > 2);
    // Not a larger tool, or one barely larger that left nothing worth a pass: refused, saying why.
    assert!(pocket(&regions, &tool(3.0), &PocketParams { rest_diameter: Some(3.0), ..small }, &c).is_err());
    let e = pocket(&regions, &tool(3.0), &PocketParams { rest_diameter: Some(3.1), ..small }, &c).unwrap_err().to_string();
    assert!(e.contains("leaves nothing for this tool"), "{e}");
}

#[test]
fn bore_mills_a_hole_on_a_helix_and_a_last_turn_at_the_bottom() {
    // A 20 mm hole with a 6 mm tool: its axis goes round 7 mm from the centre, 2 mm down a turn.
    let c = cut(0.0, -5.0, 2.0);
    let params = BoreParams { pitch: 2.0, climb: true, stock_to_leave: 0.0 };
    let path = bore(&[(v(30.0, 20.0), 20.0)], &tool(6.0), &params, &c).unwrap();
    let pts = cutting_points(&path, 0.0 + 1e-9);
    assert!(pts.iter().all(|p| (Vec2::new(p.x, p.y).dist(v(30.0, 20.0)) - 7.0).abs() < 1e-6));
    close(pts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min), -5.0, 1e-9);
    // Three turns down (5 mm), one more at the bottom, and 2 mm from the retract height to the top.
    let turn = 2.0 * PI * 7.0;
    close(path.stats().cut_length, 3.0 * turn.hypot(5.0 / 3.0) + turn + 2.0, 1e-6);
    // Climb inside a hole turns counter-clockwise.
    assert!(path.moves.iter().all(|m| !matches!(m, Move::Arc { ccw: false, .. })));
    let conventional = bore(&[(v(30.0, 20.0), 20.0)], &tool(6.0), &BoreParams { climb: false, ..params }, &c).unwrap();
    assert!(conventional.moves.iter().all(|m| !matches!(m, Move::Arc { ccw: true, .. })));
    // The stock is cut to the hole's wall, all round.
    let mut stock = HeightField::block(Vec3::new(10.0, 0.0, -10.0), Vec3::new(50.0, 40.0, 0.0), 0.25).unwrap();
    simulate(&mut stock, &path, &tool(6.0)).unwrap();
    for k in 0..16 {
        let a = k as f64 / 16.0 * 2.0 * PI;
        let inside = v(30.0, 20.0) + Vec2::from_angle(a) * 9.5;
        let outside = v(30.0, 20.0) + Vec2::from_angle(a) * 10.5;
        assert!((stock.at(inside) + 5.0).abs() < 1e-6 && stock.at(outside).abs() < 1e-6, "{a}");
    }
    // A small hole close to the tool's size: the axis goes to the middle before it goes up.
    let small = bore(&[(v(0.0, 0.0), 8.0)], &tool(6.0), &params, &c).unwrap();
    assert!(cutting_points(&small, 0.0).iter().any(|p| p.x.abs() < 1e-9 && p.y.abs() < 1e-9));
    // A hole the tool does not fit, no hole, a bad pitch: refused.
    assert!(bore(&[(v(0.0, 0.0), 6.0)], &tool(6.0), &params, &c).is_err());
    assert!(bore(&[], &tool(6.0), &params, &c).is_err());
    assert!(bore(&[(v(0.0, 0.0), 20.0)], &tool(6.0), &BoreParams { pitch: 0.0, ..params }, &c).is_err());
    assert!(bore(&[(v(0.0, 0.0), f64::NAN)], &tool(6.0), &params, &c).is_err());
}

#[test]
fn drill_pecks_and_dwells() {
    let params = DrillParams { peck: Some(4.0), dwell: 0.5 };
    let t = Tool::new(3, ToolKind::Drill, 5.0, 40.0);
    let path = drill(&[v(10.0, 10.0), v(30.0, 10.0)], &t, &params, &cut(0.0, -10.0, 1.0)).unwrap();
    // Three pecks of 10/3: from the retract height 2 to −3.333, then twice from 0.5 above the
    // last depth: 5.333 + 3.833 + 3.833 = 13 mm per hole.
    close(path.stats().cut_length, 26.0, 1e-9);
    close(path.stats().cut_minutes, 26.0 / 200.0 + 2.0 * 0.5 / 60.0, 1e-12);
    assert_eq!(path.stats().lowest_z, Some(-10.0));
    let single = drill(&[v(0.0, 0.0)], &t, &DrillParams { peck: None, dwell: 0.0 }, &cut(0.0, -10.0, 1.0)).unwrap();
    close(single.stats().cut_length, 12.0, 1e-12);
}

fn drill_program() -> Program {
    let t = Tool::new(3, ToolKind::Drill, 5.0, 40.0);
    let path = drill(&[v(10.0, 10.0), v(30.5, 10.0)], &t, &DrillParams { peck: None, dwell: 0.25 }, &cut(0.0, -10.0, 1.0)).unwrap();
    Program {
        name: "Bracket (rev 2)".into(),
        operations: vec![Operation { name: "Drill".into(), tool: t, spindle_rpm: 3000.0, coolant: true, path }],
        ..Default::default()
    }
}

#[test]
fn rs274_program_text() {
    let text = emit(&drill_program(), &Post::rs274()).unwrap();
    let expected = "\
(Bracket rev 2)
G17 G21 G40 G49 G54 G80 G90 G94
(Drill: T3 D5 drill)
M5
T3 M6
G43 H3
S3000 M3
M8
G0 X10 Y10
G0 Z10
G0 Z2
G1 Z-10 F200
G4 P0.25
G0 Z10
G0 X30.5
G0 Z2
G1 Z-10
G4 P0.25
G0 Z10
M9
M5
M30
";
    assert_eq!(text, expected);
}

#[test]
fn a_program_waits_for_the_spindle_stops_between_operations_and_cuts_air_when_asked() {
    let mut program = drill_program();
    let mut second = program.operations[0].clone();
    second.name = "Again".into();
    program.operations.push(second);
    program.notes = vec!["Stock 60 x 40 x 10 mm".into()];
    program.safety = Safety { spindle_delay: 2.5, optional_stops: true, air_cut: 0.0 };
    let text = emit(&program, &Post::grbl()).unwrap();
    let lines: Vec<&str> = text.lines().collect();
    // The notes come first; the spindle starts once and is waited for; the second operation is preceded by M1.
    assert_eq!(&lines[..2], ["(Bracket rev 2)", "(Stock 60 x 40 x 10 mm)"]);
    let on = lines.iter().position(|l| l.ends_with("M3")).unwrap();
    assert_eq!(lines[on + 1], "G4 P2.5");
    assert_eq!(lines.iter().filter(|l| l.starts_with("G4 P2.5")).count(), 1, "the speed did not change");
    let stop = lines.iter().position(|l| *l == "M1").unwrap();
    assert_eq!(lines[stop + 1], "(Again: T3 D5 drill)");
    assert_eq!(lines.iter().filter(|l| **l == "M1").count(), 1);
    // An air cut: the same moves, every Z 30 mm higher, and the program says so.
    let real = parse(&emit(&drill_program(), &Post::grbl()).unwrap()).unwrap();
    let mut air = drill_program();
    air.safety.air_cut = 30.0;
    let text = emit(&air, &Post::grbl()).unwrap();
    assert!(text.lines().nth(1).is_some_and(|l| l.contains("AIR CUT")), "{text}");
    let lifted = parse(&text).unwrap();
    assert_eq!(lifted.len(), real.len());
    for (a, b) in real.iter().zip(&lifted) {
        if let (Some(a), Some(b)) = (a.target(), b.target()) {
            assert!((b.z - a.z - 30.0).abs() < 1e-9 && (b.x - a.x).abs() < 1e-9 && (b.y - a.y).abs() < 1e-9, "{a:?} {b:?}");
        }
    }
    // Waits and lifts that are no such thing are refused.
    for safety in [
        Safety { spindle_delay: -1.0, ..Default::default() },
        Safety { spindle_delay: f64::NAN, ..Default::default() },
        Safety { air_cut: 5000.0, ..Default::default() },
    ] {
        let mut p = drill_program();
        p.safety = safety;
        assert!(emit(&p, &Post::grbl()).is_err(), "{safety:?}");
    }
}

#[test]
fn lnc_program_text() {
    // As the Advantech-LNC milling programming manual writes programs: every coordinate and
    // feed with a decimal point, `G04 X` in seconds, no `G21` (it resets the coordinate systems).
    let text = emit(&drill_program(), &Post::lnc()).unwrap();
    let expected = "\
(Bracket rev 2)
G17 G40 G80 G90 G94
(Drill: T3 D5 drill)
M03 S3000
M08
G00 X10. Y10.
G00 Z10.
G00 Z2.
G01 Z-10. F200.
G04 X0.25
G00 Z10.
G00 X30.5
G00 Z2.
G01 Z-10.
G04 X0.25
G00 Z10.
M09
M05
M30
";
    assert_eq!(text, expected);
    assert!(!text.contains("G21") && !text.contains("G43") && !text.contains("M06") && !text.contains("G54"));
    let back = Toolpath { moves: parse(&text).unwrap() };
    close(back.stats().cut_length, 24.0, 1e-9);
    close(back.stats().cut_minutes, 24.0 / 200.0 + 2.0 * 0.25 / 60.0, 1e-9);

    // With a tool changer: the tool call and its length offset, in G54.
    let atc = emit(&drill_program(), &Post::lnc_atc()).unwrap();
    assert!(atc.contains("G17 G40 G49 G80 G90 G94 G54\n") && atc.contains("M05\nT3 M06\nG43 H3\nM03 S3000\n"), "{atc}");

    // A ramped pocket: flat arcs stay arcs with decimal points, the ramp's helical arcs become lines.
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams {
        stepover: 3.0,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 3.0f64.to_radians(),
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -4.0, 2.0)).unwrap();
    assert!(
        path.moves.windows(2).any(|w| matches!((w[0].target(), w[1]), (Some(a), Move::Arc { to, .. }) if (a.z - to.z).abs() > 1e-9)),
        "the ramp has helical arcs"
    );
    let program = Program {
        name: "p".into(),
        operations: vec![Operation { name: "Pocket".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path: path.clone() }],
        ..Default::default()
    };
    let text = emit(&program, &Post::lnc()).unwrap();
    let mut z = String::new();
    for line in text.lines() {
        let arc = line.starts_with("G02 ") || line.starts_with("G03 ");
        assert!(!(arc && line.contains(" Z")), "helical arc: {line}");
        // Every coordinate, centre and feed has its decimal point.
        for word in line.split(' ').filter(|w| w.starts_with(['X', 'Y', 'Z', 'I', 'J', 'F'])) {
            assert!(word.contains('.'), "{word} in {line}");
        }
        if let Some(w) = line.split(' ').find(|w| w.starts_with('Z')) {
            z = w.to_string();
        }
    }
    assert!(text.contains("G02 ") || text.contains("G03 "));
    assert!(!z.is_empty());
    let back = Toolpath { moves: parse(&text).unwrap() };
    verify(&back, &cut(0.0, -4.0, 2.0).heights).unwrap();
    close(path.stats().cut_length, back.stats().cut_length, 0.5);
    assert_eq!(back.stats().lowest_z, Some(-4.0));
}

/// grblHAL: the first tool is the one in the spindle; a later one is called with `T… M6` (the
/// controller's `$341` decides what that does), never with `G43`, which needs a tool table.
#[test]
fn grblhal_calls_later_tools_with_m6_and_no_length_offset_word() {
    let mut program = drill_program();
    let mut second = program.operations[0].clone();
    second.tool.number = 4;
    program.operations.push(second);
    let text = emit(&program, &Post::grblhal()).unwrap();
    assert!(text.starts_with("G17 G21 G40 G49 G54 G80 G90 G94\n") || text.contains("\nG17 G21 G40 G49 G54 G80 G90 G94\n"), "{text}");
    assert_eq!(text.matches(" M6\n").count(), 1, "only the second tool is called:\n{text}");
    assert!(text.contains("M5\nT4 M6\n") && !text.contains("G43") && !text.contains("M0\n"), "{text}");
    assert_eq!(text.matches("S3000 M3").count(), 2, "the spindle restarts after the tool change");
    assert_eq!(Post::builtin("grblHAL"), Some(Post::grblhal()));
    assert!(Post::BUILTIN.iter().all(|name| Post::builtin(name).is_some_and(|p| p.validate().is_ok())));
    let back = Toolpath { moves: parse(&text).unwrap() };
    close(back.stats().cut_length, Toolpath { moves: parse(&emit(&program, &Post::grbl()).unwrap()).unwrap() }.stats().cut_length, 1e-9);
}

#[test]
fn grbl_stops_for_a_manual_tool_change_and_inches_are_converted() {
    let mut program = drill_program();
    let mut second = program.operations[0].clone();
    second.tool.number = 4;
    second.coolant = false;
    program.operations.push(second);
    let text = emit(&program, &Post::grbl()).unwrap();
    assert!(!text.contains("M6") && !text.contains("G43"), "{text}");
    assert_eq!(text.matches("M0\n").count(), 1, "one stop, before the second tool:\n{text}");
    assert_eq!(text.matches("S3000 M3").count(), 2, "the spindle restarts after the tool change");
    let mut inch = Post::rs274();
    inch.units = Units::Inch;
    inch.line_numbers = Some(LineNumbers { start: 10, step: 10 });
    let text = emit(&drill_program(), &inch).unwrap();
    assert!(text.contains("N10 G17 G20"), "{text}");
    // 30.5 mm = 1.2008 in; 200 mm/min = 7.9 in/min.
    assert!(text.contains(" X1.2008\n") && text.contains(" F7.9\n"), "{text}");
    // Read back, it is the same path in millimetres.
    let back = Toolpath { moves: parse(&text).unwrap() };
    close(back.stats().cut_length, 24.0, 0.01);
}

#[test]
fn written_program_reads_back_as_the_same_path() {
    let island = Loop2::circle(v(30.0, 20.0), 5.0);
    let regions = [Region2 { outer: rect(0.0, 0.0, 60.0, 40.0).outer, holes: vec![island] }];
    let params = PocketParams {
        stepover: 3.0,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -4.0, 2.0)).unwrap();
    let program = Program {
        name: "p".into(),
        operations: vec![Operation { name: "Pocket".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path: path.clone() }],
        ..Default::default()
    };
    for arcs in [true, false] {
        let mut post = Post::rs274();
        post.arcs = arcs;
        let text = emit(&program, &post).unwrap();
        assert_eq!(text.contains("G2 ") || text.contains("G3 "), arcs);
        let back = Toolpath { moves: parse(&text).unwrap() };
        verify(&back, &cut(0.0, -4.0, 2.0).heights).unwrap();
        let (a, b) = (path.stats(), back.stats());
        // Coordinates are written to 0.0001 mm.
        close(a.cut_length, b.cut_length, 0.2);
        close(a.rapid_length, b.rapid_length, 0.2);
        assert_eq!(a.lowest_z, b.lowest_z);
        // Every point of the path as read is on the path as planned.
        let planned: Vec<Vec3> = path.points(0.002).into_iter().map(|(p, _)| p).collect();
        for (p, _) in back.points(0.002) {
            let d = planned.windows(2).map(|w| p.dist_to_segment(w[0], w[1])).fold(f64::INFINITY, f64::min);
            assert!(d < 0.02, "{p:?} is {d} off the planned path");
        }
    }
}

#[test]
fn arc_too_short_to_write_is_never_a_full_circle() {
    // 0.00002 mm of arc: its end is written the same as its start.
    let moves = vec![
        Move::Rapid { to: Vec3::new(10.0, 0.0, 5.0) },
        Move::Feed { to: Vec3::new(10.0, 0.0, -1.0), feed: 100.0 },
        Move::Arc { to: Vec3::new(10.0, 0.00002, -1.0), center: v(0.0, 0.0), ccw: true, feed: 100.0 },
    ];
    let program = Program {
        name: "p".into(),
        operations: vec![Operation { name: "o".into(), tool: tool(6.0), spindle_rpm: 1000.0, coolant: false, path: Toolpath { moves } }],
        ..Default::default()
    };
    let text = emit(&program, &Post::rs274()).unwrap();
    assert!(!text.contains("G2 ") && !text.contains("G3 "), "{text}");
}

#[test]
fn verify_refuses_unsafe_paths() {
    let h = cut(0.0, -5.0, 1.0).heights;
    let at = |x: f64, z: f64| Vec3::new(x, 0.0, z);
    let unsafe_paths = [
        // Starts below the clearance height.
        vec![Move::Rapid { to: at(0.0, 1.0) }],
        // Starts with a cut.
        vec![Move::Feed { to: at(0.0, 10.0), feed: 100.0 }],
        // Rapid down into uncut material.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Rapid { to: at(0.0, -1.0) }],
        // Sideways rapid below the retract height.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -1.0), feed: 100.0 }, Move::Rapid { to: at(5.0, -1.0) }],
        // Below the bottom.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -5.1), feed: 100.0 }],
        // No feed.
        vec![Move::Rapid { to: at(0.0, 10.0) }, Move::Feed { to: at(0.0, -1.0), feed: 0.0 }],
        // Not a number.
        vec![Move::Rapid { to: at(f64::NAN, 10.0) }],
        // An arc whose ends are at different radii.
        vec![Move::Rapid { to: at(5.0, 10.0) }, Move::Arc { to: at(-7.0, 10.0), center: v(0.0, 0.0), ccw: true, feed: 100.0 }],
    ];
    for moves in unsafe_paths {
        assert!(matches!(verify(&Toolpath { moves: moves.clone() }, &h), Err(CamError::Unsafe(_))), "{moves:?}");
    }
    // A drill returning into its own hole is fine.
    let ok = vec![
        Move::Rapid { to: at(0.0, 10.0) },
        Move::Feed { to: at(0.0, -3.0), feed: 100.0 },
        Move::Rapid { to: at(0.0, 2.0) },
        Move::Rapid { to: at(0.0, -2.5) },
        Move::Feed { to: at(0.0, -5.0), feed: 100.0 },
    ];
    verify(&Toolpath { moves: ok }, &h).unwrap();
}

#[test]
fn hostile_values_are_errors_never_panics() {
    let bad = [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 0.0, -1.0, 1e300, -1e300, 1e-300, f64::MIN_POSITIVE];
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let good_cut = cut(0.0, -5.0, 2.0);
    for &b in &bad {
        let _ = offset_regions(&regions, b, 0.01);
        let _ = offset_regions(&regions, 1.0, b);
        let _ = offset_regions(&[rect(0.0, 0.0, b, b)], 1.0, 0.01);
        let _ = offset_regions(&[Region2 { outer: Loop2::circle(v(b, b), b), holes: vec![Loop2::circle(v(0.0, b), b)] }], b, 0.01);
        let t = Tool::new(1, ToolKind::FlatEndmill, b, b);
        let pocket_params = PocketParams {
            stepover: b,
            climb: true,
            stock_to_leave: b,
            tolerance: b,
            ramp_angle: b,
            finish: b,
            rest_diameter: None,
            ..Default::default()
        };
        let contour_params = ContourParams {
            side: Side::Outside,
            climb: true,
            stock_to_leave: b,
            tolerance: b,
            ramp_angle: b,
            tabs: Some(Tabs { count: 3, width: b, height: b }),
            finish: b,
            ..Default::default()
        };
        let bad_cut = Cut { heights: Heights { clearance: b, retract: b, top: b, bottom: -b }, feeds: Feeds { cut: b, plunge: b }, stepdown: b };
        for c in [good_cut, bad_cut, Cut { stepdown: b, ..good_cut }] {
            for tl in [&t, &tool(6.0)] {
                let _ = pocket(&regions, tl, &pocket_params, &c);
                let _ = pocket(
                    &regions,
                    tl,
                    &PocketParams {
                        stepover: 2.0,
                        climb: true,
                        stock_to_leave: 0.0,
                        tolerance: 0.01,
                        ramp_angle: 0.0,
                        finish: 0.0,
                        rest_diameter: None,
                        ..Default::default()
                    },
                    &c,
                );
                let _ = contour(&regions, tl, &contour_params, &c);
                let _ = face(v(0.0, b), v(b, 10.0), tl, &FaceParams { stepover: b, inside: false, margin: b, round: Some((v(b, b), b)) }, &c);
                let _ = face(v(0.0, 0.0), v(10.0, 10.0), tl, &FaceParams { stepover: 2.0, inside: true, margin: b, round: None }, &c);
                let _ = drill(&[v(b, 0.0)], tl, &DrillParams { peck: Some(b), dwell: b }, &c);
                let _ = drill(&[v(1.0, 0.0)], tl, &DrillParams { peck: Some(b), dwell: 0.0 }, &c);
            }
        }
        let moves = vec![
            Move::Rapid { to: Vec3::new(b, b, b) },
            Move::Arc { to: Vec3::new(b, 0.0, b), center: v(b, b), ccw: true, feed: b },
            Move::Dwell { seconds: b },
        ];
        let path = Toolpath { moves };
        let _ = path.stats();
        let _ = path.points(b);
        let _ = verify(&path, &good_cut.heights);
        let program = Program {
            name: "x".into(),
            operations: vec![Operation { name: "o".into(), tool: tool(6.0), spindle_rpm: b, coolant: false, path }],
            ..Default::default()
        };
        let _ = emit(&program, &Post::rs274());
    }
    // Things that must be refused outright.
    assert!(
        pocket(
            &regions,
            &tool(6.0),
            &PocketParams {
                stepover: 1e-9,
                climb: true,
                stock_to_leave: 0.0,
                tolerance: 0.01,
                ramp_angle: 0.0,
                finish: 0.0,
                rest_diameter: None,
                ..Default::default()
            },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &regions,
            &tool(60.0),
            &PocketParams {
                stepover: 2.0,
                climb: true,
                stock_to_leave: 0.0,
                tolerance: 0.01,
                ramp_angle: 0.0,
                finish: 0.0,
                rest_diameter: None,
                ..Default::default()
            },
            &good_cut
        )
        .is_err()
    );
    assert!(
        pocket(
            &[],
            &tool(6.0),
            &PocketParams {
                stepover: 2.0,
                climb: true,
                stock_to_leave: 0.0,
                tolerance: 0.01,
                ramp_angle: 0.0,
                finish: 0.0,
                rest_diameter: None,
                ..Default::default()
            },
            &good_cut
        )
        .is_err()
    );
    assert!(drill(&[], &tool(6.0), &DrillParams { peck: None, dwell: 0.0 }, &good_cut).is_err());
    let open = Region2 { outer: Loop2 { segs: rect(0.0, 0.0, 10.0, 10.0).outer.segs.into_iter().take(3).collect() }, holes: vec![] };
    assert!(offset_regions(&[open], 1.0, 0.01).is_err());
    assert!(emit(&Program::default(), &Post::rs274()).is_err());
}

#[test]
fn simulation_removes_what_the_tool_sweeps() {
    let regions = [rect(10.0, 10.0, 50.0, 40.0)];
    let c = cut(0.0, -5.0, 2.5);
    let block = || HeightField::block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(60.0, 50.0, 0.0), 0.25).unwrap();
    // The pocket as modelled: a 40 × 30 opening 5 deep in a 60 × 50 × 10 block.
    let mut part = block();
    for j in 0..part.ny {
        for i in 0..part.nx {
            let p = part.centre(i, j);
            if p.x > 10.0 && p.x < 50.0 && p.y > 10.0 && p.y < 40.0 {
                part.z[j * part.nx + i] = -5.0;
            }
        }
    }
    for ramp in [0.0, 3.0f64.to_radians()] {
        let params = PocketParams {
            stepover: 2.5,
            climb: true,
            stock_to_leave: 0.0,
            tolerance: 0.01,
            ramp_angle: ramp,
            finish: 0.0,
            rest_diameter: None,
            ..Default::default()
        };
        let path = pocket(&regions, &tool(6.0), &params, &c).unwrap();
        let mut stock = block();
        let report = simulate(&mut stock, &path, &tool(6.0)).unwrap();
        // Everything but the four corners the round tool cannot reach: (4 − π) r² of area.
        let swept = (1200.0 - (4.0 - PI) * 9.0) * 5.0;
        close(report.removed_volume, swept, swept * 0.01);
        close(block().volume() - stock.volume(), report.removed_volume, 1e-6);
        assert_eq!(report.rapid_collisions, 0);
        // Never more than a pass deep.
        close(report.max_engagement, 2.5, 1e-6);
        let against = compare(&stock, &part).unwrap();
        assert_eq!((against.max_gouge, against.gouge_area), (0.0, 0.0));
        // The corners keep their full 5 mm.
        close(against.max_leftover, 5.0, 1e-6);
        // At most the corners' volume; less by the one-column band the comparison forgives.
        let corners = (4.0 - PI) * 9.0 * 5.0;
        assert!(against.leftover_volume > corners * 0.3 && against.leftover_volume <= corners, "{}", against.leftover_volume);
    }
    // A pocket cut 1 mm too deep is a 1 mm gouge over its floor.
    let params = PocketParams {
        stepover: 2.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let deep = pocket(&regions, &tool(6.0), &params, &cut(0.0, -6.0, 3.0)).unwrap();
    let mut stock = block();
    simulate(&mut stock, &deep, &tool(6.0)).unwrap();
    let against = compare(&stock, &part).unwrap();
    close(against.max_gouge, 1.0, 1e-9);
    close(against.gouge_area, 1200.0 - (4.0 - PI) * 9.0, 30.0);
}

#[test]
fn a_playback_cuts_the_stock_as_far_as_the_program_has_got() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let c = cut(0.0, -5.0, 2.5);
    let params = PocketParams {
        stepover: 2.5,
        climb: true,
        stock_to_leave: 0.0,
        tolerance: 0.01,
        ramp_angle: 0.0,
        finish: 0.0,
        rest_diameter: None,
        ..Default::default()
    };
    let path = pocket(&regions, &tool(6.0), &params, &c).unwrap();
    let block = HeightField::block(Vec3::new(-5.0, -5.0, -10.0), Vec3::new(45.0, 35.0, 0.0), 0.5).unwrap();
    let mut whole = block.clone();
    simulate(&mut whole, &path, &tool(6.0)).unwrap();
    let mut play = Playback::new(&[&path], block.cell, DEFAULT_RAPID_RATE).unwrap();
    // As long as its feeds and, at a typical router's rapid rate, its rapids take.
    let stats = path.stats();
    close(play.minutes, stats.cut_minutes + stats.rapid_length / DEFAULT_RAPID_RATE, 1e-9);
    // Rapids at another rate: the times are worked out again, the tool staying where it was.
    let (t, (was, _)) = (play.minutes * 0.3, play.at(play.minutes * 0.3).unwrap());
    let now = play.retime(10_000.0, t).unwrap();
    close(play.minutes, stats.cut_minutes + stats.rapid_length / 10_000.0, 1e-9);
    assert!(play.at(now).unwrap().0.dist(was) < 1e-9);
    assert!(play.retime(0.0, now).is_err() && play.retime(f64::NAN, now).is_err());
    play.retime(DEFAULT_RAPID_RATE, now).unwrap();
    // Cut in many small pieces it leaves what the whole run leaves.
    let mut pieces = block.clone();
    let n = 37;
    for k in 0..n {
        let (t0, t1) = (play.minutes * k as f64 / n as f64, play.minutes * (k + 1) as f64 / n as f64);
        play.cut(&mut pieces, &[tool(6.0)], t0, t1).unwrap();
    }
    assert!(pieces.z.iter().zip(&whole.z).all(|(a, b)| (a - b).abs() < 1e-9));
    // Halfway the tool is on the path, and less has been cut than in the end.
    let mut half = block.clone();
    let removed = play.cut(&mut half, &[tool(6.0)], 0.0, play.minutes * 0.5).unwrap().removed_volume;
    let (at, step) = play.at(play.minutes * 0.5).unwrap();
    assert!(step.start <= play.minutes * 0.5 && play.minutes * 0.5 <= step.start + step.minutes && step.op == 0);
    close(at.dist(step.from) + at.dist(step.to), step.from.dist(step.to), 1e-9);
    assert!(removed > 0.0 && half.volume() > whole.volume() + 1.0);
    // Before the start the tool is where the path begins; past the end, where it ends.
    assert_eq!(play.at(-1.0).map(|(p, _)| p), play.steps.first().map(|s| s.from));
    assert_eq!(play.at(play.minutes + 1.0).map(|(p, _)| p), play.steps.last().map(|s| s.to));
    // Hostile times and a missing tool are no panic.
    for t in [f64::NAN, f64::INFINITY, -f64::INFINITY] {
        let _ = play.at(t);
        let _ = play.cut(&mut half.clone(), &[tool(6.0)], 0.0, t);
    }
    assert!(play.cut(&mut half, &[], 0.0, play.minutes).is_err());
    assert!(Playback::new(&[], 0.5, DEFAULT_RAPID_RATE).unwrap().at(0.0).is_none());
    assert!(Playback::new(&[&path], 0.5, -1.0).is_err());
    // A drill's dwell is time at the bottom of the hole.
    let drilled = drill(&[v(10.0, 10.0)], &tool(6.0), &DrillParams { peck: None, dwell: 3.0 }, &c).unwrap();
    let play = Playback::new(&[&drilled], 0.5, DEFAULT_RAPID_RATE).unwrap();
    let stats = drilled.stats();
    close(play.minutes, stats.cut_minutes + stats.rapid_length / DEFAULT_RAPID_RATE, 1e-9);
    let dwell = play.steps.iter().find(|s| s.dwell).expect("a dwell");
    assert!(dwell.from == dwell.to && (dwell.minutes - 0.05).abs() < 1e-12 && (dwell.to.z + 5.0).abs() < 1e-9);
}

#[test]
fn simulation_knows_tool_shapes_and_rapids_through_material() {
    let block = || HeightField::block(Vec3::new(-10.0, -10.0, -20.0), Vec3::new(10.0, 10.0, 0.0), 0.1).unwrap();
    let down =
        |z: f64| Toolpath { moves: vec![Move::Rapid { to: Vec3::new(0.0, 0.0, 5.0) }, Move::Feed { to: Vec3::new(0.0, 0.0, z), feed: 100.0 }] };
    let volume = |kind: ToolKind| {
        let mut stock = block();
        simulate(&mut stock, &down(-8.0), &Tool::new(1, kind, 6.0, 20.0)).unwrap().removed_volume
    };
    // 6 mm tools 8 mm down: a cylinder; a cylinder less the gap round a hemisphere (π r³ / 3);
    // a cylinder less the gap round a 118° point (two thirds of π r² × the point's height).
    let cylinder = PI * 9.0 * 8.0;
    close(volume(ToolKind::FlatEndmill), cylinder, cylinder * 0.01);
    close(volume(ToolKind::BallEndmill), cylinder - PI * 27.0 / 3.0, cylinder * 0.01);
    let point = 3.0 / 59.0f64.to_radians().tan();
    close(volume(ToolKind::Drill), cylinder - PI * 9.0 * point * 2.0 / 3.0, cylinder * 0.01);
    // A bull nose 1 mm round at its rim: a cylinder less the gap round its corner (its profile
    // turned round the axis, summed in rings).
    let mut stock = block();
    let bull = Tool { corner_radius: Some(1.0), ..Tool::new(1, ToolKind::BullNose, 6.0, 20.0) };
    let cut_bull = simulate(&mut stock, &down(-8.0), &bull).unwrap().removed_volume;
    let n = 10_000;
    let gap: f64 = (0..n)
        .map(|k| 2.0 + (k as f64 + 0.5) / n as f64)
        .map(|rho| 2.0 * PI * rho * (1.0 - (1.0 - (rho - 2.0).powi(2)).max(0.0).sqrt()) / n as f64)
        .sum();
    close(cut_bull, cylinder - gap, cylinder * 0.01);
    // A rapid across the top of the block is fine; one below the top is a collision and cuts nothing.
    let across = |z: f64| Toolpath { moves: vec![Move::Rapid { to: Vec3::new(-20.0, 0.0, z) }, Move::Rapid { to: Vec3::new(20.0, 0.0, z) }] };
    let mut stock = block();
    assert_eq!(simulate(&mut stock, &across(0.0), &tool(6.0)).unwrap().rapid_collisions, 0);
    let hit = simulate(&mut stock, &across(-1.0), &tool(6.0)).unwrap();
    assert_eq!((hit.rapid_collisions, hit.removed_volume), (1, 0.0));
    assert_eq!(stock, block());
    // Nothing is cut below the stock's floor, and sizes that cannot be simulated are refused.
    let mut stock = block();
    close(simulate(&mut stock, &down(-50.0), &tool(6.0)).unwrap().max_engagement, 20.0, 1e-9);
    for bad in [f64::NAN, 0.0, -1.0, 1e-9, f64::INFINITY] {
        assert!(HeightField::block(Vec3::new(0.0, 0.0, 0.0), Vec3::new(100.0, 100.0, 10.0), bad).is_err(), "{bad}");
        let _ = HeightField::block(Vec3::new(0.0, 0.0, 0.0), Vec3::new(bad, bad, bad), 1.0);
    }
    assert!(compare(&block(), &HeightField::block(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0), 0.5).unwrap()).is_err());
    // Stock cut from below up to -12: only 12 mm is left to cut, and a cut deeper goes through,
    // leaving the columns empty for a rapid to pass.
    let mut stock = block();
    stock.bottom = vec![-12.0; stock.z.len()];
    close(stock.volume(), 20.0 * 20.0 * 12.0, 1e-6);
    let through = simulate(&mut stock, &down(-15.0), &tool(6.0)).unwrap();
    close(through.max_engagement, 12.0, 1e-9);
    close(through.removed_volume, PI * 9.0 * 12.0, PI * 9.0 * 12.0 * 0.01);
    let (i, j) = stock.column(Vec2::ZERO).unwrap();
    assert_eq!((stock.get(i, j), stock.base(j * stock.nx + i)), (Some(-20.0), -20.0));
    let down_the_hole = Toolpath { moves: vec![Move::Rapid { to: Vec3::new(0.0, 0.0, 5.0) }, Move::Rapid { to: Vec3::new(0.0, 0.0, -16.0) }] };
    assert_eq!(simulate(&mut stock, &down_the_hole, &tool(2.0)).unwrap().rapid_collisions, 0);
}

#[test]
fn posts_are_data_and_bad_ones_are_refused() {
    let json = serde_json::to_string(&Post::grbl()).unwrap();
    let post: Post = serde_json::from_str(&json).unwrap();
    assert_eq!(post, Post::grbl());
    assert_eq!(Post::builtin("LinuxCNC"), Some(Post::rs274()));
    assert!(Post::BUILTIN.iter().all(|name| Post::builtin(name).is_some_and(|p| p.validate().is_ok())));
    let broken = |change: fn(&mut Post)| {
        let mut p = Post::rs274();
        change(&mut p);
        p.validate().is_err() && emit(&drill_program(), &p).is_err()
    };
    assert!(broken(|p| p.header = vec!["G21 {nonsense}".into()]));
    assert!(broken(|p| p.footer = vec!["M30 {tool".into()]));
    assert!(broken(|p| p.tool_change = vec!["M6\nG0 Z-100".into()]));
    assert!(broken(|p| p.decimals = 9));
    assert!(broken(|p| p.arc_tolerance = f64::NAN));
    assert!(broken(|p| p.line_numbers = Some(LineNumbers { start: 1, step: 0 })));
    assert!(broken(|p| p.extension = "../x".into()));
    assert!(broken(|p| p.spindle_on = vec!["M3".into(); 1000]));
    assert!(broken(|p| p.dwell = "G4 P{minutes}".into()));
    // A post saved before these switches existed reads with what it meant.
    let mut old = serde_json::to_value(Post::rs274()).unwrap();
    for k in ["helical_arcs", "trailing_decimal", "two_digit_codes", "dwell"] {
        old.as_object_mut().unwrap().remove(k);
    }
    assert_eq!(serde_json::from_value::<Post>(old).unwrap(), Post::rs274());
}

#[test]
fn reader_reports_what_it_cannot_read() {
    for text in ["G1 X1 Y1 Z1", "X1 Y1 Z1", "G0 X1 Y1 Z1\nG2 X2 Y2", "G91", "G0 X1..2", "G0 X", "G0 @", "G2 X1 Y1 Z1 I1 F10"] {
        assert!(matches!(parse(text), Err(CamError::Parse(..))), "{text}");
    }
    assert_eq!(parse("%\n(a comment) ; another\nN10 g0 x1 y2 (inline) z3\n").unwrap(), vec![Move::Rapid { to: Vec3::new(1.0, 2.0, 3.0) }]);
    assert_eq!(parse("G20 G0 X1 Y0 Z0\nG1 X2 F10").unwrap().last(), Some(&Move::Feed { to: Vec3::new(50.8, 0.0, 0.0), feed: 254.0 }));
}

#[test]
fn a_part_edge_on_the_stock_s_border_is_no_gouge() {
    // A part as large as the stock: its last row of columns has its centre on the part's edge.
    let mut part = HeightField::block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(10.0, 10.0, 0.0), 1.0).unwrap();
    part.z.iter_mut().for_each(|z| *z = 0.0);
    // A contour round it took the border row down to the floor.
    let mut stock = part.clone();
    for i in 0..stock.nx {
        if let Some(z) = stock.z.get_mut((stock.ny - 1) * stock.nx + i) {
            *z = -10.0;
        }
    }
    assert_eq!(compare(&stock, &part).unwrap().max_gouge, 0.0);
    // Two rows in, it is one.
    if let Some(z) = stock.z.get_mut((stock.ny - 3) * stock.nx + 5) {
        *z = -10.0;
    }
    assert_eq!(compare(&stock, &part).unwrap().max_gouge, 10.0);
}

#[test]
fn stock_round_a_round_part_is_waste_and_an_uncut_hole_is_left_material() {
    // A disc of radius 15 with a hole of radius 4, in a square block.
    let mut part = HeightField::block(Vec3::new(0.0, 0.0, -10.0), Vec3::new(40.0, 40.0, 0.0), 0.5).unwrap();
    let mut stock = part.clone();
    for j in 0..part.ny {
        for i in 0..part.nx {
            let r = Vec2::new(part.origin.x + (i as f64 + 0.5) * part.cell - 20.0, part.origin.y + (j as f64 + 0.5) * part.cell - 20.0).len();
            if let Some(z) = part.z.get_mut(j * part.nx + i) {
                *z = if (4.0..=15.0).contains(&r) { 0.0 } else { -10.0 };
            }
            // The program has cut round the disc, not in the hole.
            if let Some(z) = stock.z.get_mut(j * stock.nx + i) {
                *z = if (15.0..18.0).contains(&r) { -10.0 } else { 0.0 };
            }
        }
    }
    // The block's corners still stand beside the disc, but only the hole counts.
    let against = compare(&stock, &part).unwrap();
    close(against.max_leftover, 10.0, 1e-9);
    let hole = PI * 16.0 * 10.0;
    assert!(against.leftover_volume < hole * 1.2, "{} counts more than the hole", against.leftover_volume);
    // Cut out, nothing is left on the part.
    for j in 0..stock.ny {
        for i in 0..stock.nx {
            if part.get(i, j) == Some(-10.0)
                && let Some(z) = stock.z.get_mut(j * stock.nx + i)
            {
                let r = Vec2::new((i as f64 + 0.5) * 0.5 - 20.0, (j as f64 + 0.5) * 0.5 - 20.0).len();
                if r < 4.0 {
                    *z = -10.0;
                }
            }
        }
    }
    assert_eq!(compare(&stock, &part).unwrap().max_leftover, 0.0);
}

/// What an adaptive path does, checked on a fine simulation: the largest new area per mm of
/// horizontal travel at the bottom (its engagement), the pocket's material left within the
/// tool's reach, and whether it ever cut outside `inside`.
fn adaptive_check(path: &Toolpath, tool_d: f64, lo: Vec2, hi: Vec2, bottom: f64, inside: impl Fn(Vec2, f64) -> bool) -> (f64, f64, bool) {
    let block = HeightField::block(lo.extend(bottom - 1.0), hi.extend(0.0), 0.1).unwrap();
    let mut field = block.clone();
    let t = tool(tool_d);
    let mut worst: f64 = 0.0;
    let mut outside = false;
    let mut at: Option<Vec3> = None;
    for m in &path.moves {
        let Some(to) = m.target() else { continue };
        if let (Some(from), Move::Feed { .. } | Move::Arc { .. }) = (at, m) {
            let one = Toolpath { moves: vec![Move::Rapid { to: from }, *m] };
            let removed = simulate(&mut field, &one, &t).unwrap().removed_volume;
            let len = match *m {
                Move::Arc { center, ccw, .. } => crate::path::arc_sweep(from, to, center, ccw).abs() * Vec2::new(from.x, from.y).dist(center),
                _ => Vec2::new(from.x, from.y).dist(Vec2::new(to.x, to.y)),
            };
            if (from.z - bottom).abs() < 1e-9 && (to.z - bottom).abs() < 1e-9 && len > 0.2 {
                worst = worst.max(removed / -bottom / len);
            }
        } else if let Some(from) = at {
            let one = Toolpath { moves: vec![Move::Rapid { to: from }, *m] };
            simulate(&mut field, &one, &t).unwrap();
        }
        if !matches!(m, Move::Rapid { .. }) && to.z < -1e-9 && !inside(Vec2::new(to.x, to.y), tool_d * 0.5) {
            outside = true;
        }
        at = Some(to);
    }
    // Left where the tool could have reached (inside, with room for its centre within its radius).
    let mut left = 0.0;
    for j in 0..field.ny {
        for i in 0..field.nx {
            let c = field.centre(i, j);
            let z = field.get(i, j).unwrap();
            let reach = (0..16).any(|k| {
                let q = c + Vec2::from_angle(k as f64 / 16.0 * 2.0 * PI) * (tool_d * 0.5 - 0.15);
                inside(q, tool_d * 0.5)
            }) || inside(c, tool_d * 0.5);
            if inside(c, 0.15) && reach && z > bottom + 0.05 {
                left += field.cell * field.cell;
            }
        }
    }
    (worst, left, outside)
}

fn adaptive_params() -> AdaptiveParams {
    AdaptiveParams {
        stepover: 0.2,
        climb: true,
        stock_to_leave: 0.0,
        finish: 0.0,
        tolerance: 0.01,
        helix_angle: 3.0f64.to_radians(),
        helix_max: 1.0,
        helix_min: 0.1,
        rest_diameter: None,
        feed_up: 1.0,
    }
}

/// An adaptive pocket of `regions` with a 6 mm tool, 4 mm deep in one pass, checked: no cut
/// outside the pocket, engagement at most `limit` × the target width, little left within reach.
fn adaptive_case(name: &str, regions: &[Region2], params: AdaptiveParams, limit: f64) -> Toolpath {
    let started = std::time::Instant::now();
    let c = cut(0.0, -4.0, 4.0);
    let path = adaptive(regions, &tool(6.0), &params, &c).unwrap_or_else(|e| panic!("{name}: {e}"));
    let shape = Shape::from_regions(regions, 0.01).unwrap();
    let (mut lo, mut hi) = (v(f64::INFINITY, f64::INFINITY), v(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for (a, b) in shape.loops().iter().flatten().map(|p| p.bounds()) {
        (lo, hi) = (v(lo.x.min(a.x), lo.y.min(a.y)), v(hi.x.max(b.x), hi.y.max(b.y)));
    }
    let inside = |p: Vec2, m: f64| shape.dist(p) >= m - 1e-6 && shape.inside(p, m.max(1e-6));
    let (worst, left, outside) = adaptive_check(&path, 6.0, lo - v(1.0, 1.0), hi + v(1.0, 1.0), -4.0, inside);
    let ae = params.stepover * 6.0;
    eprintln!("{name}: worst {worst:.3} mm (ae {ae}), left {left:.2} mm², {} moves, {:?}", path.moves.len(), started.elapsed());
    assert!(!outside, "{name}: cut outside the pocket");
    assert!(worst <= ae * limit, "{name}: engagement {worst} over {}", ae * limit);
    assert!(left < 2.0, "{name}: left {left} mm²");
    path
}

#[test]
fn adaptive_clears_pockets_at_a_steady_load() {
    let p = adaptive_params();
    adaptive_case("rectangle", &[rect(0.0, 0.0, 40.0, 30.0)], p, 1.15);
    // A sharp inner corner (30°): its tip is out of the tool's reach, the rest is cleared.
    adaptive_case("30° corner", &[polygon(&[(0.0, 0.0), (60.0, 0.0), (0.0, 34.641)])], p, 1.15);
    // Islands.
    let mut islands = rect(0.0, 0.0, 60.0, 40.0);
    islands.holes = vec![
        polygon(&[(15.0, 15.0), (15.0, 25.0), (25.0, 25.0), (25.0, 15.0)]).outer,
        polygon(&[(35.0, 12.0), (35.0, 28.0), (45.0, 28.0), (45.0, 12.0)]).outer,
    ];
    adaptive_case("islands", &[islands], p, 1.15);
    // Two rooms joined by a narrow neck.
    let neck = polygon(&[
        (0.0, 0.0),
        (30.0, 0.0),
        (30.0, 10.0),
        (40.0, 10.0),
        (40.0, 0.0),
        (70.0, 0.0),
        (70.0, 30.0),
        (40.0, 30.0),
        (40.0, 20.0),
        (30.0, 20.0),
        (30.0, 30.0),
        (0.0, 30.0),
    ]);
    adaptive_case("neck", &[neck], p, 1.15);
    // Conventional milling, and a larger stepover.
    adaptive_case("conventional", &[rect(0.0, 0.0, 40.0, 30.0)], AdaptiveParams { climb: false, ..p }, 1.15);
    adaptive_case("stepover 35%", &[rect(0.0, 0.0, 40.0, 30.0)], AdaptiveParams { stepover: 0.35, ..p }, 1.15);
}

#[test]
fn adaptive_clears_a_slot_just_wider_than_its_tool() {
    // The steps go down the slot; the lap along the walls takes what they left in the far
    // end's corners, a little over the target there.
    adaptive_case("slot", &[rect(0.0, 0.0, 50.0, 7.0)], adaptive_params(), 1.25);
    // A cut that starts where the last one ended goes on from there, low: the tool goes up to
    // the retract height only to enter, not between the many short cuts of a slot.
    let path = adaptive(&[rect(0.0, 0.0, 30.0, 8.0)], &tool(6.0), &adaptive_params(), &cut(0.0, -4.0, 4.0)).unwrap();
    let rapids = path.moves.iter().filter(|m| matches!(m, Move::Rapid { .. })).count();
    assert!(rapids <= 6, "{rapids} rapid moves");
}

#[test]
fn adaptive_cuts_go_as_arcs_and_light_steps_feed_faster() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let c = cut(0.0, -4.0, 4.0);
    // The feeds of the moves along the floor (not those down to it).
    let feeds = |path: &Toolpath| -> Vec<f64> {
        let mut z = 0.0;
        let mut out = Vec::new();
        for m in &path.moves {
            if let Move::Feed { to, feed } | Move::Arc { to, feed, .. } = *m
                && to.z < -3.99
                && z < -3.99
            {
                out.push(feed);
            }
            z = m.target().map_or(z, |t| t.z);
        }
        out
    };
    let plain = adaptive(&regions, &tool(6.0), &adaptive_params(), &c).unwrap();
    // The steps' points on a circle are one arc move: far fewer moves than steps.
    let arcs = plain.moves.iter().filter(|m| matches!(m, Move::Arc { to, .. } if to.z < -3.99)).count();
    assert!(arcs > 50, "{arcs} arcs");
    // Without a feed allowance every cut goes at the cutting feed.
    assert!(feeds(&plain).iter().all(|f| (*f - c.feeds.cut).abs() < 1e-9));
    // With one, the lighter steps go faster, never more than allowed and never slower.
    let quick = adaptive(&regions, &tool(6.0), &AdaptiveParams { feed_up: 1.5, ..adaptive_params() }, &c).unwrap();
    let f = feeds(&quick);
    assert!(f.iter().all(|f| *f >= c.feeds.cut - 1e-9 && *f <= c.feeds.cut * 1.5 + 1e-9), "{f:?}");
    assert!(f.iter().any(|f| *f > c.feeds.cut * 1.2), "nothing faster");
    assert!(quick.stats().cut_minutes < plain.stats().cut_minutes);
    // The same cut, only faster: as long within the arcs' hundredth of a millimetre.
    close(quick.stats().cut_length, plain.stats().cut_length, plain.stats().cut_length * 1e-3);
    assert!(adaptive(&regions, &tool(6.0), &AdaptiveParams { feed_up: 0.5, ..adaptive_params() }, &c).is_err());
    assert!(adaptive(&regions, &tool(6.0), &AdaptiveParams { feed_up: f64::NAN, ..adaptive_params() }, &c).is_err());
}

#[test]
fn adaptive_passes_enter_finish_and_machine_the_rest() {
    let regions = [rect(0.0, 0.0, 40.0, 30.0)];
    let p = adaptive_params();
    // Two passes, each entering on its helix; a finishing lap 0.3 mm further out at the full depth.
    let c = cut(0.0, -8.0, 4.0);
    let path = adaptive(&regions, &tool(6.0), &AdaptiveParams { finish: 0.3, ..p }, &c).unwrap();
    assert_eq!(path.stats().lowest_z, Some(-8.0));
    let arcs_down = path.moves.iter().filter(|m| matches!(m, Move::Arc { to, .. } if to.z < 0.0)).count();
    assert!(arcs_down >= 4, "{arcs_down}");
    let wall = cutting_points(&path, -7.99).iter().map(|q| q.x).fold(f64::INFINITY, f64::min);
    close(wall, 3.0, 1e-6);
    let before_finish = cutting_points(&path, -3.99).iter().filter(|q| q.z > -4.01).map(|q| q.x).fold(f64::INFINITY, f64::min);
    close(before_finish, 3.3, 1e-6);
    // No room for the smallest helix: a ramp; no room to ramp either: refused or plunged, never a panic.
    let ramped = adaptive(&regions, &tool(6.0), &AdaptiveParams { helix_min: 4.0, helix_max: 4.0, ..p }, &c).unwrap();
    assert!(ramped.moves.iter().all(|m| !matches!(m, Move::Arc { to, .. } if to.z < -1e-9 && to.z > -3.99)));
    // Rest machining after a 12 mm tool: only the corners.
    let rest = adaptive(&regions, &tool(3.0), &AdaptiveParams { rest_diameter: Some(12.0), ..p }, &c).unwrap();
    let corners = [v(0.0, 0.0), v(40.0, 0.0), v(40.0, 30.0), v(0.0, 30.0)];
    for q in cutting_points(&rest, -0.5) {
        let near = corners.iter().map(|c| c.dist(Vec2::new(q.x, q.y))).fold(f64::INFINITY, f64::min);
        assert!(near < 8.0, "{q:?}");
    }
    // What makes no sense is refused.
    for bad in [
        AdaptiveParams { stepover: 0.0, ..p },
        AdaptiveParams { stepover: 0.9, ..p },
        AdaptiveParams { helix_angle: 0.0, ..p },
        AdaptiveParams { helix_min: 2.0, helix_max: 1.0, ..p },
        AdaptiveParams { stock_to_leave: f64::NAN, ..p },
        AdaptiveParams { rest_diameter: Some(3.0), ..p },
    ] {
        assert!(adaptive(&regions, &tool(6.0), &bad, &c).is_err(), "{bad:?}");
    }
    assert!(adaptive(&[rect(0.0, 0.0, 5.0, 5.0)], &tool(6.0), &p, &c).is_err());
}

/// The top of a box from `lo` to `hi` and its four walls, as triangles.
fn box_triangles(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let (a, b, c, d) = (p(lo.x, lo.y, hi.z), p(hi.x, lo.y, hi.z), p(hi.x, hi.y, hi.z), p(lo.x, hi.y, hi.z));
    let (e, f, g, h) = (p(lo.x, lo.y, lo.z), p(hi.x, lo.y, lo.z), p(hi.x, hi.y, lo.z), p(lo.x, hi.y, lo.z));
    vec![[a, b, c], [a, c, d], [e, f, b], [e, b, a], [f, g, c], [f, c, b], [g, h, d], [g, d, c], [h, e, a], [h, a, d]]
}

#[test]
fn a_tool_s_floor_over_a_part_follows_its_shape() {
    let mut part = HeightField::block(Vec3::new(0.0, 0.0, -1.0), Vec3::new(10.1, 10.1, 0.0), 0.1).unwrap();
    part.z.iter_mut().for_each(|z| *z = part.floor);
    let (i, j) = part.column(v(5.05, 5.05)).unwrap();
    let k = j * part.nx + i;
    part.z[k] = 10.0;
    let at = |f: &HeightField, dx: usize| f.get(i + dx, j).unwrap();
    // A 4 mm ball 0.5 off the spike: as a 5 mm ball, 0.5 lower than its surface allows.
    let ball = tool_floor(&part, &Tool::new(1, ToolKind::BallEndmill, 4.0, 10.0), 0.5).unwrap();
    close(at(&ball, 0), 10.5, 1e-9);
    close(at(&ball, 10), 10.0 - (2.5 - (2.5f64 * 2.5 - 1.0).sqrt()) + 0.5, 1e-9);
    // Beyond the grown ball's reach the part's floor (plus the allowance) is all it meets.
    close(at(&ball, 30), part.floor + 0.5, 1e-9);
    // A flat end mill stands on the spike as long as it covers it.
    let flat = tool_floor(&part, &Tool::new(1, ToolKind::FlatEndmill, 4.0, 10.0), 0.0).unwrap();
    close(at(&flat, 19), 10.0, 1e-9);
    assert!(at(&flat, 22) < 0.0);
    // A bull nose 4 mm across, its corner 1 mm round: flat out to 1 mm, then its round.
    let bull = Tool { corner_radius: Some(1.0), ..Tool::new(1, ToolKind::BullNose, 4.0, 10.0) };
    let torus = tool_floor(&part, &bull, 0.0).unwrap();
    close(at(&torus, 10), 10.0, 1e-9);
    close(at(&torus, 15), 10.0 - (1.0 - (1.0f64 - 0.25).sqrt()), 1e-9);
    assert!(tool_floor(&part, &Tool::new(1, ToolKind::Drill, 4.0, 10.0), 0.0).is_err());
    // A bull nose needs its corner radius, at most half its diameter.
    for corner in [None, Some(0.0), Some(2.5)] {
        assert!(Tool { corner_radius: corner, ..Tool::new(1, ToolKind::BullNose, 4.0, 10.0) }.validate().is_err(), "{corner:?}");
    }
}

/// The slab and block of the 3D roughing tests: a slab with its floor at -15 and on it a block
/// 20 to 40 by 12 to 28 whose top is at -5; the stock from -20 to 0 over 60 by 40.
fn slab_and_block() -> (Vec<[Vec3; 3]>, (Vec3, Vec3)) {
    let mut part = box_triangles(Vec3::new(0.0, 0.0, -20.0), Vec3::new(60.0, 40.0, -15.0));
    part.extend(box_triangles(Vec3::new(20.0, 12.0, -20.0), Vec3::new(40.0, 28.0, -5.0)));
    (part, (Vec3::new(0.0, 0.0, -20.0), Vec3::new(60.0, 40.0, 0.0)))
}

fn rough<'a>(part: &'a [[Vec3; 3]], stock: (Vec3, Vec3), boundary: &'a [Region2]) -> Rough3d<'a> {
    Rough3d { boundary, part, stock, left: None, step_up: 0.0, holder: None }
}

/// The path cut on finer columns of the stock: how deep it cuts into the part (with no
/// allowance), the share of the material over the allowance it leaves, and its deepest bite.
fn rough_check(part: &[[Vec3; 3]], stock: (Vec3, Vec3), path: &Toolpath, t: &Tool, allowance: f64) -> (f64, f64, f64) {
    let mut field = HeightField::block(stock.0, stock.1, 0.25).unwrap();
    let report = simulate(&mut field, path, t).unwrap();
    let seen = HeightField::of_triangles(v(stock.0.x, stock.0.y), v(stock.1.x, stock.1.y), 0.25, stock.0.z, part, false).unwrap();
    let (mut into, mut left, mut total) = (0.0f64, 0.0, 0.0);
    for (h, p) in field.z.iter().zip(&seen.z) {
        into = into.max(p - h);
        total += (stock.1.z - p - allowance).max(0.0);
        left += (h - p - allowance).max(0.0);
    }
    (into, left / total, report.max_engagement)
}

#[test]
fn adaptive_3d_roughs_down_to_the_part_level_by_level() {
    let (part, stock) = slab_and_block();
    let params = AdaptiveParams { stock_to_leave: 0.5, ..adaptive_params() };
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    // A ball bites a stepdown and the cusps the level above left (up to half its radius).
    for (kind, deepest_bite, most_left) in
        [(ToolKind::FlatEndmill, 4.0, 0.03), (ToolKind::BallEndmill, 4.0 + 1.5, 0.06), (ToolKind::BullNose, 4.0 + 0.5, 0.04)]
    {
        let t = Tool { corner_radius: (kind == ToolKind::BullNose).then_some(1.0), ..Tool::new(1, kind, 6.0, 20.0) };
        let started = std::time::Instant::now();
        let path = adaptive_3d(&rough(&part, stock, &area), &t, &params, &cut(0.0, -15.0, 4.0)).unwrap_or_else(|e| panic!("{kind:?}: {e}"));
        let points = cutting_points(&path, 1.0);
        let lowest = points.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        // Flat floors are levels of their own, the allowance above them.
        close(lowest, -14.5, 1e-9);
        assert!(points.iter().any(|p| (p.z + 4.5).abs() < 1e-9), "{kind:?}: a level on the block's top");
        // Never into the part; never deeper than a stepdown.
        let (into, left, bite) = rough_check(&part, stock, &path, &t, 0.5);
        // Within the rounding of the measuring columns.
        assert!(into <= 2e-3, "{kind:?}: cuts {into} into the part");
        assert!(bite <= deepest_bite + 1e-6, "{kind:?}: {bite}");
        // What stands above the allowance: the round of the tool along the block's corners (a
        // ball: and its cusps).
        assert!(left < most_left, "{kind:?}: {:.1} % left", left * 100.0);
        eprintln!("{kind:?}: {:.2} % left, bite {bite:.2}, {} moves, {:?}", left * 100.0, path.moves.len(), started.elapsed());
    }
}

/// A slope from -15 at x = 0 up to -3 at x = 60, on a stock from -20 to 0.
fn slope() -> (Vec<[Vec3; 3]>, (Vec3, Vec3)) {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let (a, b, c, d) = (p(0.0, 0.0, -15.0), p(60.0, 0.0, -3.0), p(60.0, 40.0, -3.0), p(0.0, 40.0, -15.0));
    (vec![[a, b, c], [a, c, d]], (Vec3::new(0.0, 0.0, -20.0), Vec3::new(60.0, 40.0, 0.0)))
}

#[test]
fn levels_in_between_take_the_steps_a_slope_leaves() {
    let (part, stock) = slope();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let params = AdaptiveParams { stock_to_leave: 0.3, ..adaptive_params() };
    let t = tool(6.0);
    let c = cut(0.0, -15.0, 4.0);
    let plain = adaptive_3d(&rough(&part, stock, &area), &t, &params, &c).unwrap();
    let stepped = adaptive_3d(&Rough3d { step_up: 1.0, ..rough(&part, stock, &area) }, &t, &params, &c).unwrap();
    let (into, left_plain, _) = rough_check(&part, stock, &plain, &t, 0.3);
    let (into_stepped, left_stepped, bite) = rough_check(&part, stock, &stepped, &t, 0.3);
    assert!(into <= 1e-6 && into_stepped <= 1e-6, "{into} {into_stepped}");
    // Steps of a stepdown on the slope, against steps of a millimetre.
    assert!(left_stepped < left_plain * 0.4, "{left_plain} → {left_stepped}");
    assert!(bite <= 4.0 + 1e-6, "{bite}");
    eprintln!("slope: {:.2} % left plain, {:.2} % with a 1 mm step-up", left_plain * 100.0, left_stepped * 100.0);
    assert!(adaptive_3d(&Rough3d { step_up: -1.0, ..rough(&part, stock, &area) }, &t, &params, &c).is_err());
}

#[test]
fn roughing_starts_from_what_earlier_cuts_left() {
    let (part, stock) = slab_and_block();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    // The stock faced down to -10 already: only the block above it.
    let mut faced = HeightField::block(stock.0, stock.1, 0.5).unwrap();
    faced.z.iter_mut().for_each(|z| *z = -10.0);
    let params = AdaptiveParams { stock_to_leave: 0.5, ..adaptive_params() };
    let path = adaptive_3d(&Rough3d { left: Some(&faced), ..rough(&part, stock, &area) }, &tool(6.0), &params, &cut(0.0, -15.0, 4.0)).unwrap();
    let highest = cutting_points(&path, 1.0).iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
    // Links lift half a millimetre over the level they are on.
    assert!(highest <= -10.0 + 0.5 + 1e-9, "{highest}: cuts in the air over what was faced");
}

#[test]
fn the_holder_keeps_the_tool_up_beside_a_tall_block() {
    let (part, stock) = slab_and_block();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let params = AdaptiveParams { stock_to_leave: 0.5, ..adaptive_params() };
    // A 20 mm holder, the tool 6 mm out of it: within 10 mm of the block (top -5) it goes no
    // lower than -11; further off it reaches the slab.
    let (d, out) = (20.0, 6.0);
    let path = adaptive_3d(&Rough3d { holder: Some((d, out)), ..rough(&part, stock, &area) }, &tool(6.0), &params, &cut(0.0, -15.0, 4.0)).unwrap();
    let seen = HeightField::of_triangles(v(-15.0, -15.0), v(75.0, 55.0), 0.25, -20.0, &part, true).unwrap().dilated(d * 0.5).unwrap();
    let points = cutting_points(&path, 1.0);
    for q in &points {
        assert!(seen.at(v(q.x, q.y)) <= q.z + out + 0.3, "the holder hits the part over {q:?}");
    }
    assert!(points.iter().any(|q| (q.z + 14.5).abs() < 1e-9), "the slab is reached away from the block");
    assert!(adaptive_3d(&Rough3d { holder: Some((5.0, 6.0)), ..rough(&part, stock, &area) }, &tool(6.0), &params, &cut(0.0, -15.0, 4.0)).is_err());
}

fn finish<'a>(part: &'a [[Vec3; 3]], boundary: &'a [Region2]) -> Finish3d<'a> {
    Finish3d { boundary, part, stepover: 0.5, angle: 0.0, stock_to_leave: 0.0, tolerance: 0.01, zigzag: true, left: None, rest: false }
}

/// The finished stock against the part on fine columns, over the columns `inside` keeps: the
/// deepest cut into the part and the thickest left on it (over `allowance`).
fn finish_check(part: &[[Vec3; 3]], path: &Toolpath, t: &Tool, allowance: f64, inside: impl Fn(Vec2) -> bool) -> (f64, f64) {
    let mut field = HeightField::block(Vec3::new(0.0, 0.0, -20.0), Vec3::new(60.0, 40.0, 0.0), 0.1).unwrap();
    simulate(&mut field, path, t).unwrap();
    let seen = HeightField::of_triangles(v(0.0, 0.0), v(60.0, 40.0), 0.1, -20.0, part, false).unwrap();
    let (mut into, mut left) = (f64::NEG_INFINITY, 0.0f64);
    for j in 0..field.ny {
        for i in 0..field.nx {
            // Only where there is part to compare with.
            let (Some(h), Some(p)) = (field.get(i, j), seen.get(i, j).filter(|p| *p > seen.floor + 1e-9)) else { continue };
            into = into.max(p - h);
            if inside(field.centre(i, j)) {
                left = left.max(h - p - allowance);
            }
        }
    }
    (into, left)
}

#[test]
fn parallel_finishing_follows_a_slope_to_its_cusps() {
    let (part, _) = slope();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let c = cut(0.0, -15.0, 4.0);
    // A ball's cusps between passes 0.5 mm apart: 0.5² / (8 × 3) ≈ 0.01 mm (a ball is dropped
    // exactly onto the triangles); a bull nose and a flat end also what keeping clear between
    // column centres leaves on the slope (its rim rides the slope). Nothing is cut into the part.
    for (kind, most) in [(ToolKind::BallEndmill, 0.025), (ToolKind::BullNose, 0.08), (ToolKind::FlatEndmill, 0.15)] {
        let t = Tool { corner_radius: (kind == ToolKind::BullNose).then_some(1.0), ..Tool::new(1, kind, 6.0, 20.0) };
        let path = parallel_3d(&finish(&part, &area), &t, &c).unwrap_or_else(|e| panic!("{kind:?}: {e}"));
        let (into, left) = finish_check(&part, &path, &t, 0.0, |p| p.x > 4.0 && p.x < 56.0 && p.y > 4.0 && p.y < 36.0);
        // Within the rounding of the measuring columns.
        assert!(into <= 2e-3, "{kind:?}: cuts {into} into the part");
        assert!(left <= most, "{kind:?}: leaves {left}");
        eprintln!("{kind:?}: into {into:.4}, left {left:.4}, {} moves", path.moves.len());
    }
    // An allowance stays on the part, evenly.
    let ball = Tool::new(1, ToolKind::BallEndmill, 6.0, 20.0);
    let path = parallel_3d(&Finish3d { stock_to_leave: 0.3, ..finish(&part, &area) }, &ball, &c).unwrap();
    let (into, left) = finish_check(&part, &path, &ball, 0.3, |p| p.x > 4.0 && p.x < 56.0 && p.y > 4.0 && p.y < 36.0);
    assert!(into <= -0.28, "the allowance: {into}");
    assert!(left <= 0.025, "{left}");
    for bad in [
        Finish3d { stepover: 0.0, ..finish(&part, &area) },
        Finish3d { stepover: 7.0, ..finish(&part, &area) },
        Finish3d { tolerance: 0.0, ..finish(&part, &area) },
    ] {
        assert!(parallel_3d(&bad, &ball, &c).is_err());
    }
    assert!(parallel_3d(&finish(&part, &area), &Tool::new(1, ToolKind::Drill, 6.0, 20.0), &c).is_err());
}

#[test]
fn parallel_finishing_goes_over_a_block_without_cutting_into_it() {
    let (part, _) = slab_and_block();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let ball = Tool::new(1, ToolKind::BallEndmill, 6.0, 20.0);
    let c = cut(0.0, -15.0, 4.0);
    for (angle, zigzag) in [(0.0, true), (90.0, false), (45.0, true)] {
        let path = parallel_3d(&Finish3d { angle, zigzag, ..finish(&part, &area) }, &ball, &c).unwrap();
        // Away from the walls (where a ball leaves its round), the floors and the top are finished.
        let away = |p: Vec2| {
            let off_block = p.x < 17.0 || p.x > 43.0 || p.y < 9.0 || p.y > 31.0;
            let on_top = p.x > 21.0 && p.x < 39.0 && p.y > 13.0 && p.y < 27.0;
            p.x > 3.5 && p.x < 56.5 && p.y > 3.5 && p.y < 36.5 && (off_block || on_top)
        };
        let (into, left) = finish_check(&part, &path, &ball, 0.0, away);
        // Within the rounding of the measuring columns.
        assert!(into <= 2e-3, "{angle}: cuts {into} into the part");
        assert!(left <= 0.04, "{angle}: leaves {left}");
        let lowest = path.stats().lowest_z.unwrap();
        close(lowest, -15.0, 1e-6);
    }
}

/// A slab at -15 with a frustum on it: its top 20 to 40 by 12 to 28 at -5, its foot 10 to 50 by
/// 2 to 38 at -15, so its flanks rise at 45°.
fn frustum() -> Vec<[Vec3; 3]> {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let (t0, t1, t2, t3) = (p(20.0, 12.0, -5.0), p(40.0, 12.0, -5.0), p(40.0, 28.0, -5.0), p(20.0, 28.0, -5.0));
    let (f0, f1, f2, f3) = (p(10.0, 2.0, -15.0), p(50.0, 2.0, -15.0), p(50.0, 38.0, -15.0), p(10.0, 38.0, -15.0));
    let mut part = box_triangles(Vec3::new(0.0, 0.0, -20.0), Vec3::new(60.0, 40.0, -15.0));
    part.extend([
        [t0, t1, t2],
        [t0, t2, t3],
        [f0, f1, t1],
        [f0, t1, t0],
        [f1, f2, t2],
        [f1, t2, t1],
        [f2, f3, t3],
        [f2, t3, t2],
        [f3, f0, t0],
        [f3, t0, t3],
    ]);
    part
}

fn zlevel<'a>(part: &'a [[Vec3; 3]], boundary: &'a [Region2]) -> ZLevel3d<'a> {
    ZLevel3d { boundary, part, step: 0.5, stock_to_leave: 0.0, tolerance: 0.01, steep: 30.0, climb: true, left: None, rest: false }
}

#[test]
fn z_level_finishing_goes_round_the_steep_flanks() {
    let part = frustum();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let c = cut(0.0, -15.0, 4.0);
    let ball = Tool::new(1, ToolKind::BallEndmill, 6.0, 20.0);
    let path = zlevel_3d(&zlevel(&part, &area), &ball, &c).unwrap();
    // On the flanks, away from their foot and the top's edge: the cusp between levels 0.5 mm
    // apart on a 45° flank (a ball is put exactly where it touches), as fine columns measure it;
    // nothing cut into the part.
    let flank = |p: Vec2| {
        let (dx, dy) = ((p.x - 30.0).abs(), (p.y - 20.0).abs());
        let off = (dx - 10.0).max(dy - 8.0);
        off > 1.5 && off < 8.5
    };
    let (into, left) = finish_check(&part, &path, &ball, 0.0, flank);
    assert!(into <= 2e-3, "cuts {into} into the part");
    assert!(left <= 0.04, "leaves {left} on the flanks");
    // Climbing, the part is on the right: each level's loop round the frustum runs clockwise.
    let level = |path: &Toolpath, z: f64| -> Vec<Vec2> {
        path.moves.iter().filter_map(|m| m.target()).filter(|p| (p.z - z).abs() < 1e-9).map(|p| v(p.x, p.y)).collect()
    };
    let turn = |pts: &[Vec2]| pts.windows(2).map(|w| w[0].x * w[1].y - w[1].x * w[0].y).sum::<f64>();
    assert!(turn(&level(&path, -10.0)) < 0.0, "clockwise when climbing");
    let conventional = zlevel_3d(&ZLevel3d { climb: false, ..zlevel(&part, &area) }, &ball, &c).unwrap();
    assert!(turn(&level(&conventional, -10.0)) > 0.0, "anticlockwise otherwise");
    // Steeper than its flanks only: nothing on them.
    let steep = zlevel_3d(&ZLevel3d { steep: 60.0, ..zlevel(&part, &area) }, &ball, &c).unwrap();
    assert!(level(&steep, -10.0).is_empty(), "45° flanks are left to parallel finishing");
    for bad in [ZLevel3d { step: 0.0, ..zlevel(&part, &area) }, ZLevel3d { steep: 90.0, ..zlevel(&part, &area) }] {
        assert!(zlevel_3d(&bad, &ball, &c).is_err());
    }
}

/// A flat end mill finishes a straight wall where it touches it, not a column or two away:
/// round the block's vertical walls every level runs at the tool's radius from them (it used
/// to stay 0.15 to 0.3 mm off them, which a simulation's measuring row saw as a wall-high strip).
#[test]
fn z_level_with_a_flat_end_mill_runs_against_vertical_walls() {
    let (part, _) = slab_and_block();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let c = cut(0.0, -15.0, 4.0);
    let flat = Tool::new(1, ToolKind::FlatEndmill, 6.0, 20.0);
    for (allowance, climb) in [(0.0, true), (0.0, false), (0.2, true)] {
        let path = zlevel_3d(&ZLevel3d { stock_to_leave: allowance, climb, ..zlevel(&part, &area) }, &flat, &c).unwrap();
        // The straight runs along each wall, at every level down the block (-5 to -15, the
        // last on the slab itself).
        let mut walls = 0;
        for p in path.moves.iter().filter_map(|m| m.target()).filter(|p| p.z < -5.0 && p.z > -15.0 - 1e-9) {
            let gap = if (p.x - 30.0).abs() <= 10.0 + 1e-9 && (p.y - 20.0).abs() > 8.0 {
                (p.y - 20.0).abs() - 8.0
            } else if (p.y - 20.0).abs() <= 8.0 + 1e-9 && (p.x - 30.0).abs() > 10.0 {
                (p.x - 30.0).abs() - 10.0
            } else {
                continue;
            };
            walls += 1;
            close(gap, 3.0 + allowance, 1e-6);
        }
        assert!(walls >= 4 * 20, "{allowance} {climb}: {walls} wall points");
        if allowance == 0.0 {
            // On the slab within 1 mm of the walls, up their whole height and down to the slab:
            // nothing left, nothing cut in.
            let foot = |p: Vec2| {
                let (dx, dy) = ((p.x - 30.0).abs() - 10.0, (p.y - 20.0).abs() - 8.0);
                dx.max(dy) > 0.05 && dx.max(dy) < 1.0 && dx.min(dy) < -0.5
            };
            let (into, left) = finish_check(&part, &path, &flat, 0.0, foot);
            assert!(into <= 2e-3, "cuts {into} into the part");
            assert!(left <= 0.02, "{climb}: leaves {left} at the walls' foot");
        }
    }
}

#[test]
fn rest_finishing_cleans_only_where_a_larger_ball_could_not_reach() {
    let (part, stock) = slab_and_block();
    let area = [rect(0.0, 0.0, 60.0, 40.0)];
    let c = cut(0.0, -15.0, 4.0);
    let (big, small) = (Tool::new(1, ToolKind::BallEndmill, 6.0, 20.0), Tool::new(2, ToolKind::BallEndmill, 2.0, 20.0));
    // Finished with the 6 mm ball: its round stays in the corners at the block's foot.
    let mut field = HeightField::block(stock.0, stock.1, 0.1).unwrap();
    simulate(&mut field, &parallel_3d(&finish(&part, &area), &big, &c).unwrap(), &big).unwrap();
    let seen = HeightField::of_triangles(v(0.0, 0.0), v(60.0, 40.0), 0.1, -20.0, &part, false).unwrap();
    // On the slab within 2 mm of the block's walls: the corner.
    let corner = |p: Vec2| {
        let off = (p.x - 30.0).abs().max((p.y - 20.0).abs() * 10.0 / 8.0) - 10.0;
        off > 0.2 && off < 2.0 && (p.x - 30.0).abs() < 9.0 && (p.y - 20.0).abs() < 7.0 || {
            let (dx, dy) = ((p.x - 30.0).abs() - 10.0, (p.y - 20.0).abs() - 8.0);
            (dx > 0.2 && dx < 2.0 && (p.y - 20.0).abs() < 7.0) || (dy > 0.2 && dy < 2.0 && (p.x - 30.0).abs() < 9.0)
        }
    };
    let worst = |f: &HeightField| {
        let mut most = 0.0f64;
        for j in 0..f.ny {
            for i in 0..f.nx {
                if corner(f.centre(i, j)) {
                    most = most.max(f.get(i, j).unwrap() - seen.get(i, j).unwrap());
                }
            }
        }
        most
    };
    let before = worst(&field);
    // The 2 mm ball only where that is left (its own round, a third of the 6 mm one's, stays):
    // far less cut than finishing it all.
    let rest = parallel_3d(&Finish3d { left: Some(&field), rest: true, ..finish(&part, &area) }, &small, &c).unwrap();
    let all = parallel_3d(&finish(&part, &area), &small, &c).unwrap();
    assert!(rest.stats().cut_length < all.stats().cut_length * 0.5, "{} vs {}", rest.stats().cut_length, all.stats().cut_length);
    let mut after = field.clone();
    simulate(&mut after, &rest, &small).unwrap();
    let left = worst(&after);
    assert!(before > 1.0 && left < before * 0.5, "{before} -> {left}");
    let (into, _) = finish_check(&part, &rest, &small, 0.0, |_| false);
    assert!(into <= 2e-3, "cuts {into} into the part");
    // Z-level does the same round the walls.
    let walls = zlevel_3d(&ZLevel3d { left: Some(&field), rest: true, steep: 0.0, ..zlevel(&part, &area) }, &small, &c).unwrap();
    let every = zlevel_3d(&ZLevel3d { steep: 0.0, ..zlevel(&part, &area) }, &small, &c).unwrap();
    assert!(walls.stats().cut_length < every.stats().cut_length * 0.7, "{} vs {}", walls.stats().cut_length, every.stats().cut_length);
    eprintln!(
        "rest: corner {before:.2} -> {left:.2}; parallel {:.0} of {:.0} mm, z-level {:.0} of {:.0} mm",
        rest.stats().cut_length,
        all.stats().cut_length,
        walls.stats().cut_length,
        every.stats().cut_length
    );
}
