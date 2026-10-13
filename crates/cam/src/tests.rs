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
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
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
    let params = PocketParams { stepover: 3.0, climb: false, stock_to_leave: 0.5, tolerance: 0.01, ramp_angle: 0.0 };
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
        let params = ContourParams { side: Side::Outside, climb, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
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
    let inside = ContourParams { side: Side::Inside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    assert!(contour(&[rect(0.0, 0.0, 5.0, 5.0)], &tool(6.0), &inside, &c).is_err());
    let inner = contour(&regions, &tool(6.0), &inside, &c).unwrap();
    close(inner.stats().cut_length, 2.0 * 2.0 * (34.0 + 24.0) + 8.0, 1e-6);
    // Climb inside an opening is counter-clockwise.
    assert!(turns(&inner) > 0.0);
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
    let params = ContourParams { side: Side::Outside, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp };
    let path = contour(&regions, &tool(6.0), &params, &c).unwrap();
    let lap = 140.0 + 6.0 * PI;
    close(path.stats().cut_length, 2.0 * (lap.hypot(3.0) + lap) + 2.0, 1e-6);
    let (angle, plunges) = steepest(&path, 0.0);
    assert!(!plunges && angle <= ramp + 1e-6 && angle > 0.01, "{angle} {plunges}");
    assert_eq!(path.stats().lowest_z, Some(-6.0));

    // In a pocket the first ring is short, so the ramp goes round it several times.
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp };
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

#[test]
fn face_covers_the_stock_with_the_tool_running_off_both_ends() {
    let c =
        Cut { heights: Heights { clearance: 10.0, retract: 3.0, top: 1.0, bottom: 0.0 }, feeds: Feeds { cut: 800.0, plunge: 300.0 }, stepdown: 1.0 };
    let path = face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), 15.0, &c).unwrap();
    // ceil(50/15) + 1 = 5 passes 12.5 apart, each 100 + 2 × (10 + 1) long, four 12.5 mm
    // moves between them and one 3 mm plunge.
    close(path.stats().cut_length, 5.0 * 122.0 + 4.0 * 12.5 + 3.0, 1e-9);
    assert_eq!(path.stats().lowest_z, Some(0.0));
    assert!(face(v(0.0, 0.0), v(100.0, 50.0), &tool(20.0), 25.0, &c).is_err(), "stepover above the tool diameter");
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
    let params = PocketParams { stepover: 3.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let path = pocket(&regions, &tool(6.0), &params, &cut(0.0, -4.0, 2.0)).unwrap();
    let program = Program {
        name: "p".into(),
        operations: vec![Operation { name: "Pocket".into(), tool: tool(6.0), spindle_rpm: 12000.0, coolant: false, path: path.clone() }],
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
        let pocket_params = PocketParams { stepover: b, climb: true, stock_to_leave: b, tolerance: b, ramp_angle: b };
        let contour_params = ContourParams { side: Side::Outside, climb: true, stock_to_leave: b, tolerance: b, ramp_angle: b };
        let bad_cut = Cut { heights: Heights { clearance: b, retract: b, top: b, bottom: -b }, feeds: Feeds { cut: b, plunge: b }, stepdown: b };
        for c in [good_cut, bad_cut, Cut { stepdown: b, ..good_cut }] {
            for tl in [&t, &tool(6.0)] {
                let _ = pocket(&regions, tl, &pocket_params, &c);
                let _ = pocket(&regions, tl, &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 }, &c);
                let _ = contour(&regions, tl, &contour_params, &c);
                let _ = face(v(0.0, b), v(b, 10.0), tl, b, &c);
                let _ = face(v(0.0, 0.0), v(10.0, 10.0), tl, 2.0, &c);
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
        let program =
            Program { name: "x".into(), operations: vec![Operation { name: "o".into(), tool: tool(6.0), spindle_rpm: b, coolant: false, path }] };
        let _ = emit(&program, &Post::rs274());
    }
    // Things that must be refused outright.
    assert!(
        pocket(&regions, &tool(6.0), &PocketParams { stepover: 1e-9, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 }, &good_cut)
            .is_err()
    );
    assert!(
        pocket(&regions, &tool(60.0), &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 }, &good_cut)
            .is_err()
    );
    assert!(
        pocket(&[], &tool(6.0), &PocketParams { stepover: 2.0, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 }, &good_cut)
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
        let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: ramp };
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
    let params = PocketParams { stepover: 2.5, climb: true, stock_to_leave: 0.0, tolerance: 0.01, ramp_angle: 0.0 };
    let deep = pocket(&regions, &tool(6.0), &params, &cut(0.0, -6.0, 3.0)).unwrap();
    let mut stock = block();
    simulate(&mut stock, &deep, &tool(6.0)).unwrap();
    let against = compare(&stock, &part).unwrap();
    close(against.max_gouge, 1.0, 1e-9);
    close(against.gouge_area, 1200.0 - (4.0 - PI) * 9.0, 30.0);
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
}

#[test]
fn reader_reports_what_it_cannot_read() {
    for text in ["G1 X1 Y1 Z1", "X1 Y1 Z1", "G0 X1 Y1 Z1\nG2 X2 Y2", "G91", "G0 X1..2", "G0 X", "G0 @", "G2 X1 Y1 Z1 I1 F10"] {
        assert!(matches!(parse(text), Err(CamError::Parse(..))), "{text}");
    }
    assert_eq!(parse("%\n(a comment) ; another\nN10 g0 x1 y2 (inline) z3\n").unwrap(), vec![Move::Rapid { to: Vec3::new(1.0, 2.0, 3.0) }]);
    assert_eq!(parse("G20 G0 X1 Y0 Z0\nG1 X2 F10").unwrap().last(), Some(&Move::Feed { to: Vec3::new(50.8, 0.0, 0.0), feed: 254.0 }));
}
