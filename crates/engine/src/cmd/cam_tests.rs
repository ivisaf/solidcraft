use serde_json::{Value, json};
use solidcraft_cam::{Move, Toolpath};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} is not within {tol} of {b}");
}

/// A 40 × 30 rectangle sketched on XY at `offset`, a 6 mm end mill and a setup.
fn pocket_job(offset: f64, setup: Value) -> Session {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": offset, "name": "Pocket"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6}));
    run(&mut s, "cam.setup.create", setup);
    s
}

fn pocket_op() -> Value {
    json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -5, "stepdown": 2.5, "stepover": 2.5})
}

/// The cutting moves of a program, read back from its text.
fn cuts(gcode: &str) -> Vec<solidcraft_geom::Vec3> {
    let moves = solidcraft_cam::parse(gcode).unwrap();
    Toolpath { moves }.points(0.01).into_iter().filter(|(_, cutting)| *cutting).map(|(p, _)| p).collect()
}

#[test]
fn pocket_from_a_sketch_to_gcode() {
    let mut s = pocket_job(0.0, json!({}));
    let op = run(&mut s, "cam.op.pocket", pocket_op());
    assert_eq!(op["name"], "Pocket1");
    assert_eq!(op["lowest_z"], -5.0);
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!(first["kind"], "pocket");
    // Heights left out are not stored: they follow the sketch plane (top 0, retract 2, clearance 10).
    assert!(first.get("top").is_none() && first.get("retract").is_none() && first["bottom"] == "-5");
    assert_eq!(first["toolpath"]["cut_length"], op["cut_length"]);

    let posted = run(&mut s, "cam.post", json!({}));
    let gcode = posted["gcode"].as_str().unwrap();
    assert!(
        gcode.starts_with("(Untitled Setup1)\n(Work zero G54: the world point 0, 0, 0)\n(T1: D6 flat end mill)\n")
            && gcode.contains("G17 G21 G40 G49 G54 G80 G90 G94\n(Pocket1: T1 D6 flat end mill)\nM5\nT1 M6\nG43 H1\nS12000 M3\n"),
        "{gcode}"
    );
    assert!(gcode.ends_with("M5\nM30\n"));
    let pts = cuts(gcode);
    // The 6 mm tool stays 3 mm inside the 40 × 30 walls and reaches −5 in two passes.
    for p in pts.iter().filter(|p| p.z < 0.0) {
        assert!(p.x >= 3.0 - 1e-4 && p.x <= 37.0 + 1e-4 && p.y >= 3.0 - 1e-4 && p.y <= 27.0 + 1e-4, "gouge at {p:?}");
    }
    let lowest = pts.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
    close(lowest, -5.0, 1e-9);
    assert!(pts.iter().any(|p| (p.z + 2.5).abs() < 1e-9));
    // The tool ramps in: straight-down feeds only go through air, to the top of the material
    // and then to the depth the pass before cleared, never to a new depth.
    let moves = solidcraft_cam::parse(gcode).unwrap();
    let mut at = solidcraft_geom::Vec3::ZERO;
    for m in &moves {
        let Some(to) = m.target() else { continue };
        if matches!(m, Move::Feed { .. }) && to.x == at.x && to.y == at.y {
            assert!(to.z >= -2.5 - 1e-9, "plunge to {to:?}");
        }
        at = to;
    }
    // Written to a file with Grbl's post, the program has no tool change.
    let path = std::env::temp_dir().join(format!("solidcraft-cam-{}.nc", std::process::id()));
    let written = run(&mut s, "cam.post", json!({"post": "grbl", "path": path.to_string_lossy()}));
    assert_eq!(written["extension"], "nc");
    let text = std::fs::read_to_string(&path).unwrap();
    let _ = std::fs::remove_file(&path);
    assert_eq!(text.lines().count() as u64, written["lines"].as_u64().unwrap());
    assert!(!text.contains("M6") && text.contains("S12000 M3"));
}

#[test]
fn toolpaths_follow_parameters_the_work_origin_and_the_sketch_height() {
    let mut s = pocket_job(10.0, json!({"name": "Top", "origin": [5, -5, 4]}));
    run(&mut s, "parameters.add", json!({"name": "depth", "expression": "3 mm"}));
    let mut op = pocket_op();
    op["bottom"] = json!("6 mm - depth");
    op["stepdown"] = json!(10);
    let made = run(&mut s, "cam.op.pocket", op);
    // The sketch is at world Z 10, the work origin at Z 4: the top is 6 in work coordinates.
    assert_eq!(made["lowest_z"], 3.0);
    // World points are back at the sketch; work coordinates are shifted by the origin.
    let world = run(&mut s, "cam.toolpath", json!({"setup": "Top"}));
    let pts = world["operations"][0]["points"].as_array().unwrap();
    let low = pts
        .iter()
        .filter(|p| p[3] == true)
        .map(|p| (p[0].as_f64().unwrap(), p[2].as_f64().unwrap()))
        .fold((f64::INFINITY, f64::INFINITY), |a, b| (a.0.min(b.0), a.1.min(b.1)));
    close(low.0, 3.0, 1e-6);
    close(low.1, 7.0, 1e-9);
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    let min_x = cuts(&gcode).iter().filter(|p| p.z < 6.0).map(|p| p.x).fold(f64::INFINITY, f64::min);
    close(min_x, 3.0 - 5.0, 1e-4);

    run(&mut s, "parameters.change", json!({"name": "depth", "expression": "5 mm"}));
    assert_eq!(run(&mut s, "cam.toolpath", json!({"points": false}))["operations"][0]["lowest_z"], 1.0);
}

#[test]
fn contour_face_and_drill_from_sketches() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
    run(&mut s, "sketch.circle.center", json!({"center": [10, 10], "radius": 2.5}));
    run(&mut s, "sketch.circle.center", json!({"center": [30, 20], "radius": 2.5}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "drill", "diameter": 5, "description": "5 mm drill"}));
    let common = json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 10000, "feed": 600, "plunge_feed": 200});
    let with = |extra: Value| {
        let mut v = common.clone();
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    };
    // One level 1 mm down, over the sketch's box by the tool's radius: 30/5 + 1 = 7 passes 5 apart, each 40 + 2 × 3 long, six 5 mm steps and the 3 mm plunge.
    let face = run(&mut s, "cam.op.face", with(json!({"top": 1, "bottom": 0, "stepover": 5, "stepdown": 1, "retract": 3})));
    close(face["cut_length"].as_f64().unwrap(), 7.0 * 46.0 + 6.0 * 5.0 + 3.0, 1e-9);
    // Two passes round the part 3 mm outside it, and 8 mm of plunging (see the cam crate's test).
    let contour = run(&mut s, "cam.op.contour", with(json!({"bottom": -6, "stepdown": 3, "ramp_angle": 0})));
    close(contour["cut_length"].as_f64().unwrap(), 2.0 * (140.0 + 6.0 * std::f64::consts::PI) + 8.0, 1e-6);
    close(contour["cut_minutes"].as_f64().unwrap(), 2.0 * (140.0 + 6.0 * std::f64::consts::PI) / 600.0 + 8.0 / 200.0, 1e-9);
    // Two holes, 2 → −10 each.
    let drill =
        run(&mut s, "cam.op.drill", json!({"sketch": "Holes", "tool": 2, "spindle_rpm": 3000, "feed": 150, "plunge_feed": 150, "bottom": -10}));
    close(drill["cut_length"].as_f64().unwrap(), 24.0, 1e-9);
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert_eq!(gcode.matches(" M6\n").count(), 2, "T1 for face and contour, then T2:\n{gcode}");
    assert!(gcode.contains("G0 X10 Y10\n") && gcode.contains("G0 X30 Y20\n"), "{gcode}");
    let moves = solidcraft_cam::parse(&gcode).unwrap();
    assert!(moves.iter().any(|m| matches!(m, Move::Arc { .. })), "the contour's corners are arcs");

    // Operations are cut in the setup's order, which can be changed.
    assert_eq!(run(&mut s, "cam.op.move", json!({"op": "Drill1", "to": 0}))["order"], json!(["Drill1", "Face1", "Contour1"]));
    let reordered = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(reordered.find("(Drill1").unwrap() < reordered.find("(Face1").unwrap());
    assert_eq!(run(&mut s, "cam.op.move", json!({"op": "Drill1", "by": 1}))["order"], json!(["Face1", "Drill1", "Contour1"]));
    assert_eq!(run(&mut s, "cam.op.move", json!({"op": "Drill1", "by": 99}))["order"], json!(["Face1", "Contour1", "Drill1"]));
    assert_eq!(run(&mut s, "cam.op.move", json!({"op": "Face1", "to": -5}))["order"], json!(["Face1", "Contour1", "Drill1"]));
    assert!(
        s.execute("cam.op.move", &json!({"op": "Face1"})).is_err() && s.execute("cam.op.move", &json!({"op": "Face1", "to": 0, "by": 1})).is_err()
    );
    // A tool in use stays; an unused one goes.
    assert!(s.execute("cam.tool.remove", &json!({"number": 2})).is_err());
    run(&mut s, "cam.op.remove", json!({"op": "Drill1"}));
    run(&mut s, "cam.tool.remove", json!({"number": 2}));
    assert_eq!(run(&mut s, "cam.list", json!({}))["setups"][0]["operations"].as_array().unwrap().len(), 2);
}

/// A 60 × 40 × 10 plate (top at Z 0) with a 40 × 20 pocket `depth` deep and a 4 mm hole through it.
fn machined_plate() -> Session {
    let mut s = Session::default();
    run(&mut s, "parameters.add", json!({"name": "depth", "expression": "4 mm"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "PocketSketch"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [50, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "PocketSketch", "distance": "depth", "direction": "negative", "operation": "cut"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "HoleSketch"}));
    run(&mut s, "sketch.circle.center", json!({"center": [5, 35], "radius": 2}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "HoleSketch", "distance": 10, "direction": "negative", "operation": "cut"}));
    assert_eq!(run(&mut s, "inspect.measure", json!({}))["body_count"], 1);
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "drill", "diameter": 4}));
    run(&mut s, "cam.setup.create", json!({}));
    s
}

#[test]
fn operations_from_picked_faces_take_their_heights_from_the_model() {
    let mut s = machined_plate();
    let common = json!({"tool": 1, "spindle_rpm": 10000, "feed": 600, "plunge_feed": 200, "ramp_angle": 0});
    let with = |extra: Value| {
        let mut v = common.clone();
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    };
    // The pocket's floor: cut from the top of the plate (0) down to the floor (−4), 3 mm inside its walls.
    let pocket = run(&mut s, "cam.op.pocket", with(json!({"faces": [{"point": [30, 20, -4]}]})));
    assert_eq!(pocket["lowest_z"], -4.0);
    let world = run(&mut s, "cam.toolpath", json!({"op": "Pocket1"}));
    let pts: Vec<[f64; 3]> = world["operations"][0]["points"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p[3] == true && p[2].as_f64().unwrap() < 0.0)
        .map(|p| [p[0].as_f64().unwrap(), p[1].as_f64().unwrap(), p[2].as_f64().unwrap()])
        .collect();
    assert!(!pts.is_empty());
    for p in &pts {
        assert!(p[0] >= 13.0 - 1e-4 && p[0] <= 47.0 + 1e-4 && p[1] >= 13.0 - 1e-4 && p[1] <= 27.0 + 1e-4, "gouge at {p:?}");
    }
    // The plate's top: round the 60 × 40 outline (not its pocket or hole) down to the bottom
    // of the plate in 10/3 mm passes: four laps of 200 + 6π, and 2 + 10 mm of plunging.
    let contour = run(&mut s, "cam.op.contour", with(json!({"faces": [{"body": "Body1", "point": [5, 5, 0]}], "stepdown": 3})));
    close(contour["lowest_z"].as_f64().unwrap(), -10.0, 1e-9);
    close(contour["cut_length"].as_f64().unwrap(), 4.0 * (200.0 + 6.0 * std::f64::consts::PI) + 12.0, 1e-3);
    // A round opening too big for the drill is a pocket's, not a hole: a 16 mm counterbore in the top face.
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Bore"}));
    run(&mut s, "sketch.circle.center", json!({"center": [52, 30], "radius": 8}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Bore", "distance": 2, "direction": "negative", "operation": "cut"}));
    // Its one round hole is drilled from the top through the plate: 2 → −10.
    let drill = run(&mut s, "cam.op.drill", json!({"faces": [{"point": [5, 5, 0]}], "tool": 2, "spindle_rpm": 3000, "feed": 150}));
    close(drill["cut_length"].as_f64().unwrap(), 12.0, 1e-9);
    let big = json!({"number": 3, "kind": "drill", "diameter": 3});
    run(&mut s, "cam.tool.define", big);
    let small = s.execute("cam.op.drill", &json!({"faces": [{"point": [5, 5, 0]}], "tool": 3, "spindle_rpm": 3000, "feed": 150}));
    assert!(small.unwrap_err().to_string().contains("no round holes a 3 mm drill can make"));
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(gcode.contains("G0 X5 Y35\n"), "{gcode}");

    // A deeper pocket in the design is a deeper pocket in the toolpath: the face is found again by name.
    run(&mut s, "parameters.change", json!({"name": "depth", "expression": "7 mm"}));
    assert_eq!(run(&mut s, "cam.toolpath", json!({"op": "Pocket1", "points": false}))["operations"][0]["lowest_z"], -7.0);
    let saved = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(saved.manufacture, s.doc.manufacture);

    // Faces the tool cannot reach from above, and points that are on no face.
    let refused =
        |s: &mut Session, p: Value| s.execute("cam.op.pocket", &p).err().map(|e| e.to_string()).unwrap_or_else(|| panic!("{p} was accepted"));
    assert!(refused(&mut s, with(json!({"faces": [{"point": [0, 20, -5]}]}))).contains("not supported yet"));
    assert!(refused(&mut s, with(json!({"faces": [{"point": [500, 500, 500]}]}))).contains("no face at"));
    assert!(refused(&mut s, with(json!({"faces": [{"point": [30, 20, -7]}], "sketch": "Plate"}))).contains("not both"));
    assert!(refused(&mut s, with(json!({"faces": [{"point": [30, 20, -7]}, {"point": [5, 5, 0]}]}))).contains("different heights"));
    // A sketch gives no bottom.
    assert!(refused(&mut s, with(json!({"sketch": "PocketSketch"}))).contains("`bottom` is required"));
}

#[test]
fn simulation_checks_the_program_against_the_model() {
    let mut s = machined_plate();
    let pocket = json!({"faces": [{"point": [30, 20, -4]}], "tool": 1, "spindle_rpm": 10000, "feed": 600});
    run(&mut s, "cam.op.pocket", pocket.clone());
    let sim = run(&mut s, "cam.simulate", json!({}));
    // The 40 × 20 × 4 pocket less the four corners a 6 mm tool leaves: (4 − π) × 9 mm² each way round.
    let swept = (800.0 - (4.0 - std::f64::consts::PI) * 9.0) * 4.0;
    close(sim["removed_volume"].as_f64().unwrap(), swept, swept * 0.02);
    assert_eq!((sim["max_gouge"].as_f64(), sim["operations"][0]["rapid_collisions"].as_u64()), (Some(0.0), Some(0)));
    assert_eq!(sim["warnings"], json!([]));
    // 4 mm in two passes of 2.
    close(sim["operations"][0]["max_engagement"].as_f64().unwrap(), 2.0, 1e-6);
    // The hole through the plate has not been drilled: 10 mm of material stands where the model has none.
    close(sim["max_leftover"].as_f64().unwrap(), 10.0, 1e-6);
    run(&mut s, "cam.op.drill", json!({"faces": [{"point": [5, 5, 0]}], "tool": 2, "spindle_rpm": 3000, "feed": 150}));
    let sim = run(&mut s, "cam.simulate", json!({}));
    // Now the most left anywhere is the pocket's corners, at its full depth.
    close(sim["max_leftover"].as_f64().unwrap(), 4.0, 1e-6);
    assert_eq!((sim["max_gouge"].as_f64(), sim["warnings"].clone()), (Some(0.0), json!([])));

    // Cut 1 mm deeper than the model's pocket, the program gouges its floor by 1 mm.
    run(&mut s, "cam.op.edit", json!({"op": "Pocket1", "bottom": -5}));
    let sim = run(&mut s, "cam.simulate", json!({}));
    close(sim["max_gouge"].as_f64().unwrap(), 1.0, 1e-6);
    assert!(sim["warnings"].as_array().unwrap().iter().any(|w| w.as_str().unwrap().contains("cuts up to 1.000 mm into the part")), "{sim}");
    run(&mut s, "cam.op.edit", json!({"op": "Pocket1", "bottom": null}));

    // A tool with flutes shorter than a pass is deep, and a clearance height inside the stock, are warned about.
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "flute_length": 1}));
    let sim = run(&mut s, "cam.simulate", json!({}));
    assert!(sim["warnings"].as_array().unwrap().iter().any(|w| w.as_str().unwrap().contains("flutes are 1.00 mm long")), "{sim}");
    let tall = json!({"min": [0, 0, -10], "max": [60, 40, 30]});
    let sim = run(&mut s, "cam.simulate", json!({"stock": tall, "cell": 0.5}));
    assert!(sim["operations"][0]["rapid_collisions"].as_u64().unwrap() > 0, "{sim}");
    assert!(sim["warnings"].as_array().unwrap().iter().any(|w| w.as_str().unwrap().contains("rapid move(s) run into material")));
    assert_eq!(sim["columns"], json!([120, 80]));

    // The cut stock itself, for drawing: one height and one deviation per column, in world coordinates.
    let field = &run(&mut s, "cam.simulate", json!({"field": true, "cell": 1}))["field"];
    let sizes = (field["nx"].as_u64(), field["ny"].as_u64(), field["z"].as_array().map(Vec::len), field["deviation"].as_array().map(Vec::len));
    assert_eq!(sizes, (Some(60), Some(40), Some(2400), Some(2400)));
    let column = |x: usize, y: usize, k: &str| field[k][y * 60 + x].as_f64().unwrap();
    // The middle of the pocket is at its floor; the plate beside it is untouched.
    assert_eq!((column(30, 20, "z"), column(30, 20, "deviation"), column(2, 2, "z"), column(2, 2, "deviation")), (-4.0, 0.0, 0.0, 0.0));
    assert!(run(&mut s, "cam.simulate", json!({})).get("field").is_none());

    for bad in
        [json!({"cell": 0}), json!({"cell": 1e-9}), json!({"stock": {"min": [0, 0, 0]}}), json!({"stock": {"min": [0, 0, 0], "max": [0, 0, 0]}})]
    {
        assert!(s.execute("cam.simulate", &bad).is_err(), "{bad}");
    }
}

fn work_frame(s: &mut Session) -> Value {
    run(s, "cam.list", json!({}))["setups"][0].clone()
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-6)
}

fn xyz(v: &Value) -> [f64; 3] {
    let p = crate::params::vec3(v).unwrap_or_else(|| panic!("not a point: {v}"));
    [p.x, p.y, p.z]
}

/// The world box round the cutting moves of `op` in `setup`.
fn world_box(s: &mut Session, setup: &str, op: &str) -> ([f64; 3], [f64; 3]) {
    let v = run(s, "cam.toolpath", json!({"setup": setup, "op": op}));
    let mut b = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in v["operations"][0]["points"].as_array().unwrap().iter().filter(|p| p[3] == true) {
        for k in 0..3 {
            let c = p[k].as_f64().unwrap();
            (b.0[k], b.1[k]) = (b.0[k].min(c), b.1[k].max(c));
        }
    }
    b
}

fn lowest(s: &mut Session, op: &str) -> f64 {
    run(s, "cam.toolpath", json!({"op": op, "points": false}))["operations"][0]["lowest_z"].as_f64().unwrap()
}

#[test]
fn stock_offsets_and_an_origin_on_the_stock_or_part_box() {
    let mut s = machined_plate();
    // The plate is 60 × 40 × 10 with its top at world Z 0. Stock: 5 mm round it and 2 mm on top;
    // origin: the stock's top at its lowest X and Y, which is world (−5, −5, 2).
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"side": 5, "top": 2}, "origin": {}}));
    let setup = work_frame(&mut s);
    assert!(near(xyz(&setup["frame"]["origin"]), [-5.0, -5.0, 2.0]), "{setup}");
    assert!(near(xyz(&setup["stock_box"]["min"]), [0.0, 0.0, -12.0]) && near(xyz(&setup["stock_box"]["max"]), [70.0, 50.0, 0.0]), "{setup}");
    assert!(near(xyz(&setup["part_box"]["min"]), [5.0, 5.0, -12.0]) && near(xyz(&setup["part_box"]["max"]), [65.0, 45.0, -2.0]), "{setup}");
    // Facing with nothing picked covers the stock, from its top (0) down to the part's top (−2).
    let face = run(&mut s, "cam.op.face", json!({"tool": 1, "spindle_rpm": 10000, "feed": 800, "stepover": 5, "stepdown": 1}));
    close(face["lowest_z"].as_f64().unwrap(), -2.0, 1e-9);
    // The pocket's floor, world Z −4, is at −6 in work coordinates; its walls start 13 mm in from the part: 18 from the stock.
    let pocket = json!({"faces": [{"point": [30, 20, -4]}], "tool": 1, "spindle_rpm": 10000, "feed": 600, "ramp_angle": 0});
    close(run(&mut s, "cam.op.pocket", pocket)["lowest_z"].as_f64().unwrap(), -6.0, 1e-9);
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    let pts = cuts(&gcode);
    close(pts.iter().filter(|p| p.z < -2.5).map(|p| p.x).fold(f64::INFINITY, f64::min), 18.0, 1e-4);
    // The faced area is the stock's, over it by the tool's radius: the centre runs from X −3 to 70 + 3.
    let faced = pts.iter().filter(|p| p.z > -2.5 && p.z < 0.5);
    let (lo, hi) = faced.fold((f64::INFINITY, f64::NEG_INFINITY), |m, p| (m.0.min(p.x), m.1.max(p.x)));
    close(lo, -3.0, 1e-6);
    close(hi, 73.0, 1e-6);
    // Facing the part instead, 1 mm inside its outline: the 6 mm tool's centre stays 4 mm in from the 60 × 40 part (5 mm from the stock's edge).
    run(&mut s, "cam.op.edit", json!({"op": "Face1", "boundary": "part", "limit": "inside", "margin": 1}));
    let b = world_box(&mut s, "Setup1", "Face1");
    assert!(near([b.0[0], b.0[1], 0.0], [4.0, 4.0, 0.0]) && near([b.1[0], b.1[1], 0.0], [56.0, 36.0, 0.0]), "{b:?}");
    // And over it by 2 mm, as a new operation would: passes from 2 mm before the part to 2 mm past it.
    run(&mut s, "cam.op.edit", json!({"op": "Face1", "limit": "over", "margin": 2}));
    let b = world_box(&mut s, "Setup1", "Face1");
    assert!(near([b.0[0], b.0[1], 0.0], [-2.0, 1.0, 0.0]) && near([b.1[0], b.1[1], 0.0], [62.0, 39.0, 0.0]), "{b:?}");
    for bad in [json!({"limit": "beside"}), json!({"margin": -1}), json!({"boundary": "table"})] {
        let mut p = bad.clone();
        p["op"] = json!("Face1");
        assert!(s.execute("cam.op.edit", &p).is_err(), "{bad}");
    }
    assert!(s.execute("cam.op.edit", &json!({"op": "Pocket1", "boundary": "part"})).unwrap_err().to_string().contains("no `boundary`"));
    run(&mut s, "cam.op.edit", json!({"op": "Face1", "boundary": "stock", "margin": null}));
    // One operation posts on its own; the setup posts them all.
    let alone = run(&mut s, "cam.post", json!({"op": "Pocket1"}))["gcode"].as_str().unwrap().to_string();
    assert!(
        alone.starts_with("(Untitled Setup1 Pocket1)") && alone.contains("(Pocket1:") && !alone.contains("(Face1:") && alone.ends_with("M30\n"),
        "{alone}"
    );
    assert!(gcode.contains("(Face1:") && gcode.contains("(Pocket1:"));
    assert!(s.execute("cam.post", &json!({"op": "Nope"})).is_err());
    // In the world the toolpath is where the part is.
    close(world_box(&mut s, "Setup1", "Pocket1").0[2], -4.0, 1e-9);
    // The simulation uses the setup's stock: the facing takes its top 2 mm off.
    let sim = run(&mut s, "cam.simulate", json!({}));
    close(sim["operations"][0]["removed_volume"].as_f64().unwrap(), 7000.0, 140.0);
    // (the faced area is back to the stock's)
    assert_eq!(sim["max_gouge"], 0.0);

    // More stock on top lifts the origin with it: the same floor is deeper in work coordinates.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"top": "2 mm + depth"}}));
    assert!(near(xyz(&work_frame(&mut s)["frame"]["origin"]), [-5.0, -5.0, 6.0]));
    close(lowest(&mut s, "Pocket1"), -10.0, 1e-9);
    // The middle of the part's top face, and the middle of the bottom of the stock.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "origin": {"of": "part", "x": "center", "y": "center", "z": "max"}}));
    assert!(near(xyz(&work_frame(&mut s)["frame"]["origin"]), [30.0, 20.0, 0.0]));
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "origin": {"x": "center", "y": "center", "z": "min"}}));
    assert!(near(xyz(&work_frame(&mut s)["frame"]["origin"]), [30.0, 20.0, -10.0]));
    // A fixed world point still works, and what was not named was kept.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "origin": [1, 2, 3], "name": "Top", "post": "lnc_atc"}));
    let setup = work_frame(&mut s);
    assert!(near(xyz(&setup["frame"]["origin"]), [1.0, 2.0, 3.0]));
    assert_eq!((setup["name"].as_str(), setup["post"].as_str(), setup["stock"]["side"].as_str()), (Some("Top"), Some("lnc_atc"), Some("5")));

    let before = s.doc.manufacture.clone();
    for bad in [
        json!({"axis": "q"}),
        json!({"axis": [0, 0, 0]}),
        json!({"origin": {"of": "table"}}),
        json!({"origin": {"x": "left"}}),
        json!({"origin": {"corner": 1}}),
        json!({"origin": "top"}),
        json!({"stock": {"side": -1}}),
        json!({"stock": 5}),
        json!({"post": "fanuc"}),
        json!({"name": "Top", "setup": "Nope"}),
    ] {
        let mut p = bad.clone();
        if p.get("setup").is_none() {
            p["setup"] = json!("Top");
        }
        assert!(s.execute("cam.setup.edit", &p).is_err(), "{bad}");
    }
    assert_eq!(s.doc.manufacture, before);
    // An origin on the box needs a body to put a box round.
    let mut empty = Session::default();
    assert!(empty.execute("cam.setup.create", &json!({"origin": {"of": "stock"}})).unwrap_err().to_string().contains("there is no body"));
}

#[test]
fn an_origin_on_a_corner_an_edge_or_a_circle_of_the_model() {
    let mut s = machined_plate();
    let origin = |s: &mut Session, o: Value| {
        run(s, "cam.setup.edit", json!({"setup": "Setup1", "origin": o}));
        xyz(&work_frame(s)["frame"]["origin"])
    };
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"side": 5, "top": 2}}));
    // A vertex picked: that corner across, the stock's top by default, or the corner's own height.
    assert!(near(origin(&mut s, json!({"edge": [0, 0, 0]})), [0.0, 0.0, 2.0]));
    let o = &work_frame(&mut s)["origin"];
    assert_eq!((o["type"].as_str(), o["on"].as_str()), (Some("edge"), Some("vertex")), "{o}");
    assert!(near(origin(&mut s, json!({"edge": [0, 0, 0], "z": "point"})), [0.0, 0.0, 0.0]));
    assert!(near(origin(&mut s, json!({"edge": [60, 40, -10], "z": {"of": "part", "z": "min"}})), [60.0, 40.0, -10.0]));
    // A point on the hole's rim: its centre; on a straight edge: its middle.
    let rim = 2.0 * std::f64::consts::FRAC_1_SQRT_2;
    assert!(near(origin(&mut s, json!({"edge": [5.0 + rim, 35.0 + rim, 0]})), [5.0, 35.0, 2.0]));
    // Where its two halves meet (the seam down the hole's side ends there too) is a corner.
    assert!(near(origin(&mut s, json!({"edge": [7, 35, 0], "z": "point"})), [7.0, 35.0, 0.0]));
    assert!(near(origin(&mut s, json!({"edge": [20, 0, 0], "on": "middle", "z": "point"})), [30.0, 0.0, 0.0]));
    run(&mut s, "cam.op.face", json!({"tool": 1, "spindle_rpm": 10000, "feed": 800, "stepover": 5, "stepdown": 1}));
    let notes = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(notes.contains("(Work zero G54: the middle of an edge of Body1, Z there)"), "{notes}");
    let before = s.doc.manufacture.clone();
    for bad in [
        json!({"edge": [30, 20, 5]}),
        json!({"edge": [20, 0, 0], "on": "center"}),
        json!({"edge": [0, 0, 0], "on": "side"}),
        json!({"edge": [0, 0, 0], "z": "top"}),
        json!({"edge": [0, 0, 0], "body": "Nope"}),
        json!({"edge": [0, 0, 0], "x": "min"}),
    ] {
        assert!(s.execute("cam.setup.edit", &json!({"setup": "Setup1", "origin": bad})).is_err(), "{bad}");
    }
    assert_eq!(s.doc.manufacture, before);

    // It follows the design: a thicker plate moves the bottom corner with it.
    let mut s = Session::default();
    run(&mut s, "parameters.add", json!({"name": "t", "expression": "10 mm"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": "t", "direction": "negative"}));
    run(&mut s, "cam.setup.create", json!({"origin": {"edge": [60, 0, -10], "z": "point"}}));
    assert!(near(xyz(&work_frame(&mut s)["frame"]["origin"]), [60.0, 0.0, -10.0]));
    run(&mut s, "parameters.change", json!({"name": "t", "expression": "15 mm"}));
    assert!(near(xyz(&work_frame(&mut s)["frame"]["origin"]), [60.0, 0.0, -15.0]), "{}", work_frame(&mut s));
    // Saved and read back as it was.
    let text = s.doc.to_json();
    assert_eq!(solidcraft_doc::Document::from_json(&text).unwrap().manufacture, s.doc.manufacture);
}

#[test]
fn tool_axis_turns_the_setup() {
    let mut s = machined_plate();
    let contour = |setup: &str, point: [f64; 3]| json!({"setup": setup, "faces": [{"point": point}], "tool": 1, "spindle_rpm": 10000, "feed": 600, "ramp_angle": 0});
    // From below: the setup's +Z is world −Z, its X world X and its Y world −Y.
    run(&mut s, "cam.setup.create", json!({"name": "Bottom", "axis": "-z"}));
    let f = run(&mut s, "cam.list", json!({}))["setups"][1]["frame"].clone();
    assert!(near(xyz(&f["x"]), [1.0, 0.0, 0.0]) && near(xyz(&f["y"]), [0.0, -1.0, 0.0]) && near(xyz(&f["z"]), [0.0, 0.0, -1.0]), "{f}");
    // The plate's underside (world Z −10) is the top of this setup: 10 in work coordinates, cut down to 0.
    let made = run(&mut s, "cam.op.contour", contour("Bottom", [5.0, 5.0, -10.0]));
    close(made["lowest_z"].as_f64().unwrap(), 0.0, 1e-9);
    // In the world it runs round the plate 3 mm out, between its underside and its top.
    let b = world_box(&mut s, "Bottom", "Contour1");
    assert!(near(b.0, [-3.0, -3.0, -10.0]) && near(b.1, [63.0, 43.0, 0.0]), "{b:?}");
    // The pocket opens upwards: from below its floor points away, and only a contour may use such a face.
    assert!(s.execute("cam.op.pocket", &contour("Bottom", [30.0, 20.0, -4.0])).unwrap_err().to_string().contains("points away from the tool"));
    // A face on the side cannot be cut from above or below.
    assert!(s.execute("cam.op.contour", &contour("Bottom", [60.0, 20.0, -5.0])).unwrap_err().to_string().contains("not supported yet"));

    // From the side: +X is the tool axis, work X is world Y and work Y world Z.
    run(&mut s, "cam.setup.edit", json!({"setup": "Bottom", "axis": "+x", "name": "Side"}));
    let listed = run(&mut s, "cam.list", json!({}));
    assert!(
        listed["setups"][1]["operations"][0]["error"].as_str().unwrap().contains("not supported yet"),
        "the old operation no longer faces the tool"
    );
    run(&mut s, "cam.op.remove", json!({"setup": "Side", "op": "Contour1"}));
    run(&mut s, "cam.op.contour", contour("Side", [60.0, 20.0, -5.0]));
    // Round the 40 × 10 end face, 3 mm out, from X 60 back through the plate's length to X 0.
    let b = world_box(&mut s, "Side", "Contour1");
    assert!(near(b.0, [0.0, -3.0, -13.0]) && near(b.1, [60.0, 43.0, 3.0]), "{b:?}");
    // Any direction will do; a tilted one keeps work X level.
    run(&mut s, "cam.setup.edit", json!({"setup": "Side", "axis": [1, 0, 1]}));
    let f = run(&mut s, "cam.list", json!({}))["setups"][1]["frame"].clone();
    let (x, z) = (xyz(&f["x"]), xyz(&f["z"]));
    assert!(x[2].abs() < 1e-12 && near(z, [0.5f64.sqrt(), 0.0, 0.5f64.sqrt()]), "{f}");
}

#[test]
fn manufacture_is_saved_undone_and_reported_when_it_breaks() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "cam.op.pocket", pocket_op());
    let length = run(&mut s, "cam.toolpath", json!({"points": false}))["operations"][0]["cut_length"].clone();

    let text = s.doc.to_json();
    assert!(text.contains(&format!("\"solidcraft/{}\"", solidcraft_doc::format::FORMAT)) && text.contains("\"manufacture\""));
    let reopened = solidcraft_doc::Document::from_json(&text).unwrap();
    assert_eq!(reopened.manufacture, s.doc.manufacture);
    let mut again = Session::new(reopened);
    assert_eq!(run(&mut again, "cam.toolpath", json!({"points": false}))["operations"][0]["cut_length"], length);
    // A design without manufacturing data does not mention it.
    assert!(!Session::default().doc.to_json().contains("\"manufacture\""));

    run(&mut s, "edit.undo", json!({}));
    assert!(s.doc.manufacture.setups[0].operations.is_empty());
    run(&mut s, "edit.redo", json!({}));
    assert_eq!(s.doc.manufacture.setups[0].operations.len(), 1);

    // An operation whose sketch is gone says so; it is not dropped and nothing panics.
    run(&mut s, "timeline.delete", json!({"feature": "Pocket"}));
    let listed = run(&mut s, "cam.list", json!({}));
    assert!(listed["setups"][0]["operations"][0]["error"].as_str().unwrap().contains("sketch"), "{listed}");
    assert!(s.execute("cam.post", &json!({})).is_err());
}

#[test]
fn operations_are_edited_in_place() {
    let mut s = pocket_job(0.0, json!({}));
    let made = run(&mut s, "cam.op.pocket", pocket_op());
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 4}));
    // Deeper, with the smaller tool, renamed; what is not named stays.
    let edited = run(&mut s, "cam.op.edit", json!({"op": "Pocket1", "bottom": -8, "tool": 2, "stepover": 1.5, "name": "Rough", "coolant": true}));
    assert_eq!((edited["operation"].clone(), edited["lowest_z"].clone()), (made["operation"].clone(), json!(-8.0)));
    let listed = run(&mut s, "cam.list", json!({}));
    let op = &listed["setups"][0]["operations"][0];
    assert_eq!(
        (op["name"].as_str(), op["tool"].as_u64(), op["spindle_rpm"].as_f64(), op["coolant"].as_bool()),
        (Some("Rough"), Some(2), Some(12000.0), Some(true))
    );
    assert_eq!((op["kind"].as_str(), op["inputs"]["stepover"].as_str(), op["inputs"]["climb"].as_bool()), (Some("pocket"), Some("1.5"), Some(true)));
    // The 4 mm tool runs 2 mm inside the walls.
    let world = run(&mut s, "cam.toolpath", json!({}));
    let min_x = world["operations"][0]["points"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p[3] == true)
        .map(|p| p[0].as_f64().unwrap())
        .fold(f64::INFINITY, f64::min);
    close(min_x, 2.0, 1e-6);
    // A height given as null follows the geometry again; here the sketch gives no bottom, so the edit is refused and nothing changes.
    let before = s.doc.manufacture.clone();
    let refused = |s: &mut Session, p: Value| s.execute("cam.op.edit", &p).err().map(|e| e.to_string()).unwrap_or_else(|| panic!("{p} was accepted"));
    assert!(refused(&mut s, json!({"op": "Rough", "bottom": null})).contains("`bottom` is required"));
    assert!(refused(&mut s, json!({"op": "Rough", "tool": 9})).contains("no tool 9"));
    assert!(refused(&mut s, json!({"op": "Rough", "peck": 2})).contains("a pocket operation has no `peck`"));
    assert!(refused(&mut s, json!({"op": "Rough", "stepover": 5})).contains("stepover"));
    assert!(refused(&mut s, json!({"op": "Nope", "bottom": -1})).contains("no operation"));
    assert_eq!(s.doc.manufacture, before);
    run(&mut s, "cam.op.edit", json!({"op": "Rough", "top": 1, "retract": null}));
    assert_eq!(s.doc.manufacture.setups[0].operations[0].top.as_deref(), Some("1"));
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.doc.manufacture, before);
}

#[test]
fn tools_carry_cutting_data_and_libraries_move_between_designs() {
    let mut s = pocket_job(0.0, json!({}));
    // A tool with its own speed and feeds: operations need not repeat them.
    run(
        &mut s,
        "cam.tool.define",
        json!({"number": 5, "kind": "flat", "diameter": 6, "flutes": 3, "spindle_rpm": 18000, "feed": 1500, "plunge_feed": 400, "description": "3-flute"}),
    );
    run(&mut s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": 5, "bottom": -2}));
    let op = &s.doc.manufacture.setups[0].operations[0];
    assert_eq!((op.spindle_rpm, op.cut_feed, op.plunge_feed), (18000.0, 1500.0, 400.0));
    // Given values win; a tool without data needs them.
    run(&mut s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": 5, "bottom": -2, "feed": 900}));
    let op = &s.doc.manufacture.setups[0].operations[1];
    assert_eq!((op.spindle_rpm, op.cut_feed, op.plunge_feed), (18000.0, 900.0, 400.0));
    let refused = s.execute("cam.op.pocket", &json!({"sketch": "Pocket", "tool": 1, "bottom": -2, "feed": 600})).unwrap_err().to_string();
    assert!(refused.contains("`spindle_rpm` is required: tool 1 has none of its own"), "{refused}");
    // Redefining a tool keeps the data that is not named; null clears it.
    run(&mut s, "cam.tool.define", json!({"number": 5, "kind": "flat", "diameter": 6, "feed": 1200, "plunge_feed": null}));
    let t = s.doc.manufacture.tool(5).unwrap().clone();
    assert_eq!((t.flutes, t.spindle_rpm, t.feed, t.plunge_feed), (3, Some(18000.0), Some(1200.0), None));
    // A chip load is kept as given: operations feed at speed × flutes × chip load, 18000 × 3 × 0.02 = 1080 mm/min.
    let made =
        run(&mut s, "cam.tool.define", json!({"number": 8, "kind": "flat", "diameter": 6, "flutes": 3, "spindle_rpm": 18000, "chip_load": 0.02}));
    assert_eq!((made["tool"]["chip_load"].as_f64(), made["tool"].get("feed")), (Some(0.02), None));
    run(&mut s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": 8, "bottom": -2}));
    close(s.doc.manufacture.setups[0].operations[2].cut_feed, 1080.0, 1e-9);
    run(&mut s, "cam.op.remove", json!({"op": "Pocket3"}));
    // A listing works it out from what the tool has: tool 5 runs 1200 mm/min at 18000 with 3 flutes.
    let listed = run(&mut s, "cam.list", json!({}));
    let five = listed["tools"].as_array().unwrap().iter().find(|t| t["number"] == 5).unwrap();
    close(five["chip_load"].as_f64().unwrap(), 1200.0 / 54000.0, 1e-12);
    assert!(listed["tools"][0].get("chip_load").is_none(), "tool 1 has no speed or feed");
    // Naming one of a pair clears the other; naming both is refused.
    let made = run(&mut s, "cam.tool.define", json!({"number": 8, "kind": "flat", "diameter": 6, "feed": 1440}));
    assert_eq!((made["tool"]["feed"].as_f64(), made["tool"]["chip_load"].as_f64()), (Some(1440.0), Some(1440.0 / 54000.0)));
    assert!(s.doc.manufacture.tool(8).unwrap().chip_load.is_none());
    let p = json!({"number": 8, "kind": "flat", "diameter": 6, "chip_load": 0.02, "feed": 900});
    assert!(s.execute("cam.tool.define", &p).unwrap_err().to_string().contains("give `feed` or `chip_load`, not both"));
    let p = json!({"number": 8, "kind": "flat", "diameter": 6, "surface_speed": 300, "spindle_rpm": 900});
    assert!(s.execute("cam.tool.define", &p).unwrap_err().to_string().contains("give `spindle_rpm` or `surface_speed`, not both"));
    run(&mut s, "cam.tool.remove", json!({"number": 8}));
    for bad in [
        json!({"flutes": 0}),
        json!({"flutes": 2.5}),
        json!({"feed": -1}),
        json!({"spindle_rpm": "fast"}),
        json!({"chip_load": 0, "spindle_rpm": 1000}),
        json!({"chip_load": 101}),
        json!({"surface_speed": -5}),
        json!({"surface_speed": 20000}),
        json!({"cutting": [{"material": "MDF", "surface_speed": 0}]}),
        json!({"cutting": [{"material": "MDF", "feed": 900, "chip_load": 0.02}]}),
    ] {
        let mut p = json!({"number": 6, "kind": "flat", "diameter": 6});
        for (k, v) in bad.as_object().unwrap() {
            p[k] = v.clone();
        }
        assert!(s.execute("cam.tool.define", &p).is_err(), "{bad}");
    }

    // The library goes to a file and into another design.
    let path = std::env::temp_dir().join(format!("solidcraft-tools-{}.json", std::process::id()));
    let path = path.to_string_lossy().to_string();
    assert_eq!(run(&mut s, "cam.tool.export", json!({"path": path}))["tools"], 2);
    let mut other = Session::default();
    run(&mut other, "cam.tool.define", json!({"number": 5, "kind": "drill", "diameter": 3}));
    let merged = run(&mut other, "cam.tool.import", json!({"path": path}));
    assert_eq!((merged["added"].clone(), merged["skipped"].clone()), (json!([1]), json!([5])));
    assert_eq!(other.doc.manufacture.tool(5).unwrap().diameter, 3.0);
    let merged = run(&mut other, "cam.tool.import", json!({"path": path, "replace": true}));
    assert_eq!(merged["replaced"], json!([1, 5]));
    assert_eq!(other.doc.manufacture.tools, s.doc.manufacture.tools);
    run(&mut other, "edit.undo", json!({}));
    assert_eq!(other.doc.manufacture.tool(5).unwrap().diameter, 3.0);
    // Files that are not tool libraries, or hold tools that cannot exist, are refused whole.
    for text in [
        "{}",
        "not json",
        r#"{"format": "solidcraft-tools/1", "tools": [{"number": 1, "kind": "flat_endmill", "diameter": -6, "flute_length": 10}]}"#,
    ] {
        std::fs::write(&path, text).unwrap();
        let before = other.doc.manufacture.clone();
        assert!(other.execute("cam.tool.import", &json!({"path": path})).is_err(), "{text}");
        assert_eq!(other.doc.manufacture, before);
    }
    let _ = std::fs::remove_file(&path);
}

/// A library file of this test's own.
fn library_file(name: &str) -> String {
    let path = std::env::temp_dir().join(format!("solidcraft-lib-{name}-{}.json", std::process::id()));
    let _ = std::fs::remove_file(&path);
    path.to_string_lossy().to_string()
}

#[test]
fn machine_library_tools_are_used_by_any_design_which_keeps_its_own_copy() {
    let lib = library_file("machine");
    let mut s = pocket_job(0.0, json!({}));
    // A library that does not exist yet is empty; the design's own tool is the only one about.
    let listed = run(&mut s, "cam.library.list", json!({"library": lib}));
    assert_eq!((listed["tools"].clone(), listed["only_in_design"][0]["number"].clone()), (json!([]), json!(1)));
    // A tool with everything an operation starts from.
    let rougher = json!({"library": lib, "number": 7, "kind": "flat", "diameter": 6, "description": "Rougher", "flutes": 3,
        "spindle_rpm": 16000, "feed": 1400, "plunge_feed": 350, "stepdown": 1.5, "stepover": 2});
    run(&mut s, "cam.library.define", rougher);
    assert!(s.doc.manufacture.tool(7).is_none(), "defining a library tool does not touch the design");
    assert_eq!(run(&mut s, "cam.library.list", json!({"library": lib}))["tools"][0]["design"], "absent");

    // An operation names only the tool: speed, feeds, stepdown and stepover are the tool's.
    run(&mut s, "cam.op.pocket", json!({"library": lib, "sketch": "Pocket", "tool": 7, "bottom": -3}));
    let op = s.doc.manufacture.setups[0].operations[0].clone();
    assert_eq!((op.spindle_rpm, op.cut_feed, op.plunge_feed, op.stepdown.as_str()), (16000.0, 1400.0, 350.0, "1.5 mm"));
    assert!(matches!(&op.kind, solidcraft_doc::manufacture::CamOperationKind::Pocket { stepover, .. } if stepover == "2 mm"));
    // The design now has its own copy of the tool, the same as the library's.
    assert_eq!(s.doc.manufacture.tool(7).map(|t| (t.diameter, t.flutes)), Some((6.0, 3)));
    assert_eq!(run(&mut s, "cam.library.list", json!({"library": lib}))["tools"][0]["design"], "same");
    // Each operation's values are its own: changed on one, they change nothing else.
    run(&mut s, "cam.op.edit", json!({"op": "Pocket1", "feed": 900, "spindle_rpm": 12000, "stepdown": 3}));
    run(&mut s, "cam.op.pocket", json!({"library": lib, "sketch": "Pocket", "tool": 7, "bottom": -3, "plunge_feed": 100}));
    let ops = &s.doc.manufacture.setups[0].operations;
    assert_eq!((ops[0].cut_feed, ops[0].spindle_rpm, ops[0].stepdown.as_str()), (900.0, 12000.0, "3"));
    assert_eq!((ops[1].cut_feed, ops[1].spindle_rpm, ops[1].plunge_feed), (1400.0, 16000.0, 100.0));
    assert_eq!(s.doc.manufacture.tool(7).and_then(|t| t.feed), Some(1400.0), "nor the tool");

    // The library's tool becomes 8 mm: this design cuts as before until its copy is updated.
    let before = lowest_x(&mut s);
    close(before, 3.0, 1e-6);
    run(&mut s, "cam.library.define", json!({"library": lib, "number": 7, "kind": "flat", "diameter": 8}));
    assert_eq!(run(&mut s, "cam.library.list", json!({"library": lib}))["tools"][0]["design"], "differs");
    close(lowest_x(&mut s), 3.0, 1e-6);
    assert_eq!(run(&mut s, "cam.tool.update", json!({"library": lib}))["updated"], json!([7]));
    close(lowest_x(&mut s), 4.0, 1e-6);
    assert_eq!(run(&mut s, "cam.tool.update", json!({"library": lib}))["updated"], json!([]));
    // The redefinition kept what it did not name: cutting data, description, flute length.
    let t = s.doc.manufacture.tool(7).cloned().unwrap();
    assert_eq!((t.spindle_rpm, t.stepover, t.description.as_str(), t.flute_length), (Some(16000.0), Some(2.0), "Rougher", 18.0));
    run(&mut s, "edit.undo", json!({}));
    close(lowest_x(&mut s), 3.0, 1e-6);

    // Another design on this machine sees the same library.
    let mut other = pocket_job(0.0, json!({}));
    run(&mut other, "cam.op.pocket", json!({"library": lib, "sketch": "Pocket", "tool": 7, "bottom": -3}));
    assert_eq!(other.doc.manufacture.tool(7).map(|t| t.diameter), Some(8.0));
    // A tool that cannot cut the pocket is not copied in on the way to finding that out.
    run(&mut other, "cam.library.define", json!({"library": lib, "number": 9, "kind": "flat", "diameter": 50, "spindle_rpm": 1000, "feed": 100}));
    assert!(
        other
            .execute("cam.op.pocket", &json!({"library": lib, "sketch": "Pocket", "tool": 9, "bottom": -3}))
            .unwrap_err()
            .to_string()
            .contains("does not fit")
    );
    assert!(other.doc.manufacture.tool(9).is_none());

    // A design's own tool goes into the library; library files move between machines.
    assert_eq!(run(&mut s, "cam.library.add", json!({"library": lib, "number": 1}))["added"], json!([1]));
    let file = library_file("exported");
    assert_eq!(run(&mut s, "cam.library.export", json!({"library": lib, "path": file}))["tools"], 3);
    let elsewhere = library_file("elsewhere");
    run(&mut s, "cam.library.define", json!({"library": elsewhere, "number": 7, "kind": "drill", "diameter": 3}));
    let merged = run(&mut s, "cam.library.import", json!({"library": elsewhere, "path": file}));
    assert_eq!((merged["added"].clone(), merged["skipped"].clone()), (json!([1, 9]), json!([7])));
    run(&mut s, "cam.library.remove", json!({"library": elsewhere, "number": 9}));
    let numbers: Vec<u64> = run(&mut s, "cam.library.list", json!({"library": elsewhere}))["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["number"].as_u64().unwrap())
        .collect();
    assert_eq!(numbers, [1, 7]);
    assert!(s.execute("cam.library.remove", &json!({"library": elsewhere, "number": 9})).is_err());
    // A damaged library is reported, never read as empty and written over.
    std::fs::write(&elsewhere, "{ not a library").unwrap();
    for (cmd, p) in [
        ("cam.library.list", json!({})),
        ("cam.library.define", json!({"number": 2, "kind": "flat", "diameter": 3})),
        ("cam.library.add", json!({"all": true})),
    ] {
        let mut p = p;
        p["library"] = json!(elsewhere);
        assert!(s.execute(cmd, &p).unwrap_err().to_string().contains("not a tool library"), "{cmd}");
    }
    assert_eq!(std::fs::read_to_string(&elsewhere).unwrap(), "{ not a library");
    for f in [lib, file, elsewhere] {
        let _ = std::fs::remove_file(f);
    }
}

/// Lowest X the first operation cuts at, in the world.
fn lowest_x(s: &mut Session) -> f64 {
    let v = run(s, "cam.toolpath", json!({}));
    v["operations"][0]["points"].as_array().unwrap().iter().filter(|p| p[3] == true).map(|p| p[0].as_f64().unwrap()).fold(f64::INFINITY, f64::min)
}

#[test]
fn what_cannot_be_cut_is_refused() {
    let mut s = pocket_job(0.0, json!({}));
    let refused =
        |s: &mut Session, id: &str, p: Value| s.execute(id, &p).err().map(|e| e.to_string()).unwrap_or_else(|| panic!("{id} {p} was accepted"));
    let with = |k: &str, v: Value| {
        let mut p = pocket_op();
        p[k] = v;
        p
    };
    assert!(refused(&mut s, "cam.op.pocket", with("tool", json!(7))).contains("no tool 7"));
    assert!(refused(&mut s, "cam.op.pocket", with("stepover", json!(7))).contains("stepover"));
    assert!(refused(&mut s, "cam.op.pocket", with("bottom", json!(5))).contains("heights"));
    assert!(refused(&mut s, "cam.op.pocket", with("sketch", json!("Nope"))).contains("no sketch"));
    assert!(refused(&mut s, "cam.op.pocket", with("setup", json!("Nope"))).contains("no setup"));
    run(&mut s, "cam.tool.define", json!({"number": 9, "kind": "flat", "diameter": 50}));
    assert!(refused(&mut s, "cam.op.pocket", with("tool", json!(9))).contains("does not fit"));
    assert!(refused(&mut s, "cam.setup.create", json!({"post": "fanuc"})).contains("unknown post"));
    assert!(refused(&mut s, "cam.post", json!({})).contains("no operation"));
    // A sketch that does not face the tool.
    run(&mut s, "sketch.create", json!({"plane": "XZ", "name": "Side"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    assert!(refused(&mut s, "cam.op.pocket", with("sketch", json!("Side"))).contains("not supported yet"));
    assert!(s.doc.manufacture.setups[0].operations.is_empty(), "nothing refused was added");
    // A post definition with a template the writer does not know.
    run(&mut s, "cam.op.pocket", pocket_op());
    let mut custom = serde_json::to_value(solidcraft_cam::Post::grbl()).unwrap();
    custom["header"] = json!(["G21", "{password}"]);
    assert!(refused(&mut s, "cam.post", json!({"post": custom})).contains("placeholder"));
}

#[test]
fn hostile_cam_params_never_panic() {
    let lib = library_file("hostile");
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "cam.op.pocket", pocket_op());
    let hostile = [
        json!(null),
        json!(-1),
        json!(0),
        json!(1e308),
        json!(-1e308),
        json!(1e-308),
        json!("NaN"),
        json!("1/0"),
        json!(""),
        json!("x".repeat(5000)),
        json!([]),
        json!({}),
        json!([1e308, -1e308, 1e308]),
        json!(true),
        json!(18446744073709551615u64),
    ];
    let commands = [
        "cam.op.edit",
        "cam.op.move",
        "cam.simulate",
        "cam.setup.edit",
        "cam.tool.import",
        "cam.tool.export",
        "cam.tool.update",
        "cam.library.list",
        "cam.library.define",
        "cam.library.remove",
        "cam.library.add",
        "cam.library.import",
        "cam.library.export",
        "cam.tool.define",
        "cam.tool.remove",
        "cam.setup.create",
        "cam.op.face",
        "cam.op.contour",
        "cam.op.pocket",
        "cam.op.drill",
        "cam.op.remove",
        "cam.toolpath",
        "cam.post",
        "cam.list",
    ];
    let keys = [
        "number",
        "kind",
        "diameter",
        "flute_length",
        "origin",
        "post",
        "setup",
        "sketch",
        "profiles",
        "tool",
        "spindle_rpm",
        "feed",
        "plunge_feed",
        "bottom",
        "top",
        "retract",
        "clearance",
        "stepdown",
        "stepover",
        "stock_to_leave",
        "ramp_angle",
        "faces",
        "to",
        "by",
        "limit",
        "margin",
        "boundary",
        "cell",
        "stock",
        "axis",
        "side",
        "peck",
        "dwell",
        "op",
        "name",
        "points",
    ];
    for cmd in commands {
        for key in keys {
            for value in &hostile {
                let mut p = pocket_op();
                p["number"] = json!(3);
                p["kind"] = json!("flat");
                p["op"] = json!("Pocket1");
                p["library"] = json!(lib);
                p["diameter"] = json!(4);
                p[key] = value.clone();
                if let Err(e) = s.execute(cmd, &p) {
                    assert!(!matches!(e, crate::EngineError::Internal(..)), "{cmd} {key}={value}: {e}");
                }
            }
        }
    }
}

/// Heights from a sketch follow the part and the work origin: an outline on the part's floor is
/// cut from the material's top down to it, holes sketched on its top go through it, and a pocket
/// asks for its depth; heights out of order say what they are.
#[test]
fn sketch_heights_follow_the_part_and_the_work_origin() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Outline"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Holes"}));
    run(&mut s, "sketch.circle.center", json!({"center": [15, 20], "radius": 6}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "drill", "diameter": 5, "spindle_rpm": 3000, "feed": 150}));
    // Work origin on the top of the stock, 1 mm above the part: the part spans -11 … -1.
    run(&mut s, "cam.setup.create", json!({"stock": {"top": 1}, "origin": {"of": "stock"}}));

    let contour = run(&mut s, "cam.op.contour", json!({"sketch": "Outline", "tool": 1}));
    close(contour["lowest_z"].as_f64().unwrap(), -11.0, 1e-9);
    let drill = run(&mut s, "cam.op.drill", json!({"sketch": "Holes", "tool": 2}));
    close(drill["lowest_z"].as_f64().unwrap(), -11.0, 1e-9);

    let err = s.execute("cam.op.pocket", &json!({"sketch": "Holes", "tool": 1})).unwrap_err().to_string();
    assert!(err.contains("`bottom` is required") && err.contains("-1 mm"), "{err}");
    let err = s.execute("cam.op.pocket", &json!({"sketch": "Holes", "tool": 1, "bottom": -0.5})).unwrap_err().to_string();
    assert!(err.contains("the bottom (-0.5 mm) is above the top (-1 mm)"), "{err}");
    close(run(&mut s, "cam.op.pocket", json!({"sketch": "Holes", "tool": 1, "bottom": -4}))["lowest_z"].as_f64().unwrap(), -4.0, 1e-9);
}

#[test]
fn forms_learn_which_heights_an_operation_finds_by_itself() {
    let mut s = machined_plate();
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": -4, "name": "Floor"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [12, 12], "p1": [48, 28]}));
    run(&mut s, "sketch.finish", json!({}));
    // Rounded to a micrometre: the part's box is measured, not exact.
    let um = |v: Option<f64>| v.map(|v| (v * 1e3).round() / 1e3);
    let heights = |s: &Session, cmd: &str, p: Value| {
        let (top, bottom) = crate::cam_heights(s, cmd, &p).unwrap();
        (um(top), um(bottom))
    };
    // A pocket sketched on the part's top: its depth is the user's to give.
    assert_eq!(heights(&s, "cam.op.pocket", json!({"sketch": "PocketSketch"})), (Some(0.0), None));
    // A contour on the top goes through the part; an outline on a floor cuts down to it.
    assert_eq!(heights(&s, "cam.op.contour", json!({"sketch": "Plate"})), (Some(0.0), Some(-10.0)));
    assert_eq!(heights(&s, "cam.op.pocket", json!({"sketch": "Floor"})), (Some(0.0), Some(-4.0)));
    // What stops the geometry is said, and nothing is added.
    let none = crate::cam_heights(&s, "cam.op.pocket", &json!({"sketch": "Nowhere"}));
    assert!(none.is_err());
    assert!(crate::cam_heights(&s, "cam.setup.create", &json!({})).is_err());
    assert!(s.doc.manufacture.setups[0].operations.is_empty());
}

#[test]
fn a_hole_s_wall_gives_its_circle_and_its_depth() {
    let mut s = machined_plate();
    // The through hole of 4 mm at (5, 35): its wall runs from the top (0) to the bottom (-10).
    let wall = json!([{"point": [7, 35, -5]}]);
    let um = |v: Option<f64>| v.map(|v| (v * 1e3).round() / 1e3);
    let (top, bottom) = crate::cam_heights(&s, "cam.op.drill", &json!({"faces": wall})).unwrap();
    assert_eq!((um(top), um(bottom)), (Some(0.0), Some(-10.0)));
    let (top, bottom) = crate::cam_heights(&s, "cam.op.pocket", &json!({"faces": wall})).unwrap();
    assert_eq!((um(top), um(bottom)), (Some(0.0), Some(-10.0)));
    // Nothing typed: the drill goes to the wall's foot at the hole's centre.
    run(&mut s, "cam.op.drill", json!({"faces": wall, "tool": 2, "spindle_rpm": 3000, "feed": 200}));
    let path = run(&mut s, "cam.toolpath", json!({}));
    let cuts: Vec<[f64; 3]> = path["operations"][0]["points"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p[3].as_bool() == Some(true))
        .map(|p| [p[0].as_f64().unwrap(), p[1].as_f64().unwrap(), p[2].as_f64().unwrap()])
        .collect();
    let low = cuts.iter().fold([0.0, 0.0, f64::INFINITY], |m, p| if p[2] < m[2] { *p } else { m });
    assert!((low[0] - 5.0).abs() < 1e-6 && (low[1] - 35.0).abs() < 1e-6 && (low[2] + 10.0).abs() < 1e-3, "{low:?}");
}

#[test]
fn a_selection_s_heights_are_read_in_the_setup() {
    let mut s = machined_plate();
    let heights = |s: &Session, sel: Vec<crate::Sel>| crate::cam_selection_heights(s, &json!({}), &sel).unwrap();
    let v = |x: f64, y: f64, z: f64| crate::Sel::Vertex { body: "Body1".into(), point: solidcraft_geom::Vec3::new(x, y, z) };
    assert_eq!(heights(&s, vec![v(0.0, 0.0, -10.0)]), Some((-10.0, -10.0)));
    assert_eq!(heights(&s, vec![v(0.0, 0.0, -10.0), v(10.0, 10.0, -4.0)]), Some((-10.0, -4.0)));
    // A whole body reaches from its foot to its top; things that are not geometry give nothing.
    let body = heights(&s, vec![crate::Sel::Body { name: "Body1".into() }]).unwrap();
    assert!((body.0 + 10.0).abs() < 1e-6 && body.1.abs() < 1e-6, "{body:?}");
    assert_eq!(heights(&s, vec![crate::Sel::Feature { id: 1 }]), None);
    // In a setup whose zero is at the part's bottom, the same point is 10 mm higher.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "origin": {"of": "part", "x": "min", "y": "min", "z": "min"}}));
    assert_eq!(heights(&s, vec![v(0.0, 0.0, -10.0)]).map(|(lo, _)| (lo * 1e3).round() / 1e3), Some(0.0));
}

/// A 60 × 40 plate with its top at 0, a boss of radius 12 at (30, 20) up to 12, a 6 mm end
/// mill, and a setup with a millimetre of stock on top.
fn plate_with_boss() -> Session {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Boss"}));
    run(&mut s, "sketch.circle.center", json!({"center": [30, 20], "radius": 12}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Boss", "distance": 12, "operation": "join"}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.setup.create", json!({"stock": {"top": "1 mm"}}));
    s
}

#[test]
fn a_stock_is_a_set_box_or_a_round_bar_and_the_part_is_the_bodies_named() {
    let mut s = plate_with_boss();
    // The part reaches 0..60 across X, 0..40 across Y and -10..12 up; its stock is 1 mm over it.
    let boxes = |s: &mut Session| {
        let listed = run(s, "cam.list", json!({}));
        let setup = listed["setups"][0].clone();
        // To a micron: the bodies' meshes are not exact.
        let um = |v: f64| (v * 1e6).round() / 1e6;
        let corner = |b: &str, k: &str| -> Vec<f64> { (0..3).map(|i| um(setup[b][k][i].as_f64().unwrap())).collect() };
        let round = setup["stock_round"]
            .as_object()
            .map(|r| (um(r["centre"][0].as_f64().unwrap()), um(r["centre"][1].as_f64().unwrap()), um(r["radius"].as_f64().unwrap())));
        ((corner("stock_box", "min"), corner("stock_box", "max")), corner("part_box", "max"), round)
    };
    // A box of a set size is centred on the part across, its top 1 mm over the part's.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"shape": "box", "size": [80, "50 mm", 30]}}));
    let ((lo, hi), _, round) = boxes(&mut s);
    assert_eq!((lo, hi, round), (vec![-10.0, -5.0, -17.0], vec![70.0, 45.0, 13.0], None));
    // A round bar along the axis: 80 across, from the part's foot to 1 mm over its top.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"shape": "round", "diameter": 80}}));
    let ((lo, hi), _, round) = boxes(&mut s);
    assert_eq!((lo, hi), (vec![-10.0, -20.0, -10.0], vec![70.0, 60.0, 13.0]));
    assert_eq!(round, Some((30.0, 20.0, 40.0)));
    // Facing off the 1 mm over the part takes the bar's circle, not the box round it: no cut runs
    // further out than the bar and the tool's radius, so none goes through the air at its corners.
    let square = run(&mut s, "cam.op.face", json!({"tool": 1, "bottom": 12, "stepover": 4}))["cut_minutes"].as_f64().unwrap();
    let points = run(&mut s, "cam.toolpath", json!({}))["operations"][0]["points"].clone();
    let furthest = points
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p[3].as_bool() == Some(true))
        .map(|p| (p[0].as_f64().unwrap() - 30.0).hypot(p[1].as_f64().unwrap() - 20.0))
        .fold(0.0, f64::max);
    assert!(furthest <= 40.0 + 3.0 + 1e-6, "{furthest}");
    // A box of the bar's size round it takes longer to face.
    run(&mut s, "cam.op.remove", json!({"op": "Face1"}));
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"shape": "box", "size": [80, 80, 23]}}));
    let boxed = run(&mut s, "cam.op.face", json!({"tool": 1, "bottom": 12, "stepover": 4}))["cut_minutes"].as_f64().unwrap();
    assert!(square < boxed * 0.9, "{square} {boxed}");
    run(&mut s, "cam.op.remove", json!({"op": "Face1"}));
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"shape": "round", "diameter": 80}}));
    run(&mut s, "cam.op.face", json!({"tool": 1, "bottom": 12, "stepover": 4}));
    let sim = run(&mut s, "cam.simulate", json!({}));
    let disc = std::f64::consts::PI * 40.0 * 40.0;
    close(sim["removed_volume"].as_f64().unwrap(), disc, disc * 0.02);
    let sheet = run(&mut s, "cam.sheet", json!({}))["text"].as_str().unwrap().to_string();
    assert!(sheet.contains("Stock round bar D80 x 23 mm, centre X30 Y20"), "{sheet}");
    // A round bar holds one part; a box can hold copies again.
    assert!(
        s.execute("cam.setup.edit", &json!({"setup": "Setup1", "copies": {"nx": 2, "dx": 70}}))
            .unwrap_err()
            .to_string()
            .contains("copies need a box")
    );
    for bad in [json!({"shape": "box"}), json!({"shape": "round", "diameter": 0}), json!({"shape": "cone"}), json!({"shape": "box", "size": [1, 2]})]
    {
        assert!(s.execute("cam.setup.edit", &json!({"setup": "Setup1", "stock": bad})).is_err(), "{bad}");
    }
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "stock": {"shape": "offset"}}));

    // A fixture modelled beside the part makes the stock reach over it, until the setup names its part.
    run(&mut s, "solid.box", json!({"length": 20, "width": 20, "height": 40, "corner": [100, 0, -10], "body_name": "Fixture"}));
    let plate = s.model.state().bodies.iter().map(|b| b.name.clone()).find(|n| n != "Fixture").unwrap();
    assert_eq!(boxes(&mut s).1, vec![120.0, 40.0, 30.0]);
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "bodies": [plate.to_lowercase()]}));
    assert_eq!(s.doc.manufacture.setups[0].bodies, vec![plate.clone()], "kept as the design names it");
    assert_eq!(boxes(&mut s).1, vec![60.0, 40.0, 12.0]);
    // The fixture is no part to check the program against: the facing above leaves it be.
    let sim = run(&mut s, "cam.simulate", json!({}));
    close(sim["max_gouge"].as_f64().unwrap(), 0.0, 1e-9);
    assert!(s.execute("cam.setup.edit", &json!({"setup": "Setup1", "bodies": ["Nope"]})).unwrap_err().to_string().contains("no body `Nope`"));
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "bodies": null}));
    assert!(s.doc.manufacture.setups[0].bodies.is_empty());
}

#[test]
fn facing_below_the_part_s_top_goes_round_it_and_cuts_into_it_are_reported() {
    let mut s = plate_with_boss();
    // Facing the stock down to the plate: the boss stays, the plate round it is cut to its top.
    run(&mut s, "cam.op.face", json!({"tool": 1, "bottom": 0, "stepover": 4}));
    let points = run(&mut s, "cam.toolpath", json!({}))["operations"][0]["points"].clone();
    let cuts: Vec<[f64; 3]> = points
        .as_array()
        .unwrap()
        .iter()
        .filter(|p| p[3].as_bool() == Some(true))
        .map(|p| [p[0].as_f64().unwrap(), p[1].as_f64().unwrap(), p[2].as_f64().unwrap()])
        .collect();
    let near = cuts.iter().filter(|p| p[2] < 12.0).map(|p| (p[0] - 30.0).hypot(p[1] - 20.0)).fold(f64::INFINITY, f64::min);
    assert!(near >= 15.0, "{near}");
    assert!(cuts.iter().any(|p| p[2].abs() < 1e-9), "down to the plate's top");
    // A pocket over the boss's sketch box goes through it: the listing says so; facing does not.
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Over"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 5], "p1": [50, 35]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.op.pocket", json!({"sketch": "Over", "tool": 1, "top": 12, "bottom": 2}));
    let listed = run(&mut s, "cam.list", json!({}));
    let ops = listed["setups"][0]["operations"].as_array().unwrap();
    assert!(ops[0].get("warning").is_none(), "{}", ops[0]);
    let warning = ops[1]["warning"].as_str().unwrap();
    assert!(warning.starts_with("cuts up to 10.") && warning.ends_with("mm into the part"), "{warning}");
}

#[test]
fn a_picked_face_under_standing_stock_is_cut_from_the_stock_s_top() {
    // Facing the plate's picked top round the boss: the stock stands 13 mm above it.
    let mut s = plate_with_boss();
    run(&mut s, "cam.op.face", json!({"tool": 1, "faces": [{"point": [5, 5, 0]}], "stepover": 3, "stepdown": 3}));
    let (top, bottom) = crate::cam_heights(&s, "cam.op.face", &json!({"faces": [{"point": [5, 5, 0]}]})).unwrap();
    assert_eq!((top.map(|t| (t * 1e3).round() / 1e3), bottom.map(|b| (b * 1e3).round() / 1e3)), (Some(13.0), Some(0.0)));
    // In passes from the stock's top, never at a rapid into it, and not into the boss.
    let sim = run(&mut s, "cam.simulate", json!({}));
    let op = &sim["operations"][0];
    assert_eq!(op["rapid_collisions"], 0, "{sim}");
    assert!(op["max_engagement"].as_f64().unwrap() <= 3.0 + 1e-6, "{sim}");
    assert_eq!(sim["max_gouge"], 0.0);
}

#[test]
fn a_preview_shows_the_toolpath_and_changes_nothing() {
    let mut s = plate_with_boss();
    run(&mut s, "cam.op.face", json!({"tool": 1, "bottom": 12, "stepover": 4}));
    let (doc, revision) = (s.doc.clone(), s.revision);
    let started = std::time::Instant::now();
    let new = crate::cam_preview(&s, "cam.op.contour", &json!({"faces": [{"point": [42, 20, 6]}], "tool": 1})).unwrap().points;
    let edited = crate::cam_preview(&s, "cam.op.edit", &json!({"op": "Face1", "bottom": 11})).unwrap().points;
    eprintln!("two previews in {:?}", started.elapsed());
    // Round the boss's wall, from the stock's top down to its foot.
    let low = new.iter().filter(|p| p.1).map(|p| p.0.z).fold(f64::INFINITY, f64::min);
    let near = new.iter().filter(|p| p.1).map(|p| (p.0.x - 30.0).hypot(p.0.y - 20.0)).fold(f64::INFINITY, f64::min);
    assert!(low.abs() < 1e-6 && (near - 15.0).abs() < 1e-3, "{low} {near}");
    assert!(edited.iter().any(|p| p.1 && (p.0.z - 11.0).abs() < 1e-6));
    // Nothing of it is in the design.
    assert!(std::sync::Arc::ptr_eq(&doc, &s.doc) && s.revision == revision);
    assert_eq!(s.doc.manufacture.setups[0].operations.len(), 1);
    assert!(crate::cam_preview(&s, "cam.op.remove", &json!({"op": "Face1"})).is_err());
    assert_eq!(s.doc.manufacture.setups[0].operations.len(), 1);
}

#[test]
fn a_contour_round_a_picked_boss_top_stops_at_the_floor_it_stands_on() {
    let mut s = plate_with_boss();
    // Round the boss's top face: down to the plate's top (0), not through the plate (-10).
    let top = json!({"faces": [{"point": [30, 26, 12]}], "tool": 1});
    let (t, b) = crate::cam_heights(&s, "cam.op.contour", &top).unwrap();
    assert_eq!((t.map(|v| (v * 1e3).round() / 1e3), b.map(|v| (v * 1e3).round() / 1e3)), (Some(13.0), Some(0.0)));
    let preview = crate::cam_preview(&s, "cam.op.contour", &top).unwrap();
    assert_eq!(preview.warning, None);
    // Round the plate's own top: through it, as before; a pocket over the boss is warned of before it is added.
    let (_, b) = crate::cam_heights(&s, "cam.op.contour", &json!({"faces": [{"point": [5, 5, 0]}]})).unwrap();
    assert_eq!(b.map(|v| (v * 1e3).round() / 1e3), Some(-10.0));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Over"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 5], "p1": [50, 35]}));
    run(&mut s, "sketch.finish", json!({}));
    let over = crate::cam_preview(&s, "cam.op.pocket", &json!({"sketch": "Over", "tool": 1, "top": 12, "bottom": 2})).unwrap();
    assert!(over.warning.is_some_and(|w| w.starts_with("cuts up to 10.")));
}

#[test]
fn an_rp23cnc_tool_library_is_imported_with_its_materials_and_cutting_data() {
    let mut s = Session::default();
    let library = library_file("rp23cnc");
    let file = std::env::temp_dir().join(format!("solidcraft-rp23cnc-{}.json", std::process::id()));
    let rp = json!({
        "Tools": [
            {"Number": 1, "Name": "6 mm flat end mill", "Type": "EndMill", "DiameterMm": 6, "FluteCount": 2, "VeeAngleDeg": 0, "TipDiameterMm": 0, "CornerRadiusMm": 0, "Notes": null},
            {"Number": 2, "Name": "3 mm flat end mill", "Type": "EndMill", "DiameterMm": 3, "FluteCount": 2, "VeeAngleDeg": 0, "TipDiameterMm": 0, "CornerRadiusMm": 0, "Notes": null},
            {"Number": 3, "Name": "V-bit 60°", "Type": "VBit", "DiameterMm": 6, "FluteCount": 1, "VeeAngleDeg": 60, "TipDiameterMm": 0, "CornerRadiusMm": 0, "Notes": null},
            {"Number": 4, "Name": "8 mm R1", "Type": "EndMill", "DiameterMm": 8, "FluteCount": 3, "VeeAngleDeg": 0, "TipDiameterMm": 0, "CornerRadiusMm": 1, "Notes": null},
            {"Number": 5, "Name": "4 mm R2", "Type": "EndMill", "DiameterMm": 4, "FluteCount": 2, "VeeAngleDeg": 0, "TipDiameterMm": 0, "CornerRadiusMm": 2, "Notes": null}
        ],
        "Materials": [{"Name": "Plywood", "Notes": null}, {"Name": "Aluminium", "Notes": null}],
        "Presets": [
            {"Name": "6 mm in plywood", "ToolNumber": 1, "Material": null, "FeedMmMin": 1200, "PlungeMmMin": 400, "SpindleRpm": 18000, "DepthOfCutMm": 3, "StepoverPercent": 40, "Notes": null},
            {"Name": "6 mm in aluminium", "ToolNumber": 1, "Material": "Aluminium", "FeedMmMin": 500, "PlungeMmMin": 100, "SpindleRpm": 20000, "DepthOfCutMm": 0.5, "StepoverPercent": 25, "Notes": null},
            {"Name": "6 mm again", "ToolNumber": 1, "Material": null, "FeedMmMin": 900, "Notes": null},
            {"Name": "3 mm in MDF", "ToolNumber": 2, "Material": "MDF", "FeedMmMin": 800, "Notes": null}
        ]
    });
    std::fs::write(&file, rp.to_string()).unwrap();
    let out = run(&mut s, "cam.library.import", json!({"library": library, "path": file}));
    let _ = std::fs::remove_file(&file);
    assert_eq!(out["added"], json!([1, 2, 3, 4, 5]));
    assert_eq!(out["notes"], json!(["preset `6 mm again` of tool 1 (6 mm flat end mill): its own data is `6 mm in plywood`'s"]));
    // The preset without a material is tool 1's own cutting data; its stepover, 40% of 6 mm.
    let read = crate::toollib::load(std::path::Path::new(&library)).unwrap();
    let tools = &read.tools;
    let t1 = &tools[0];
    assert_eq!((t1.description.as_str(), t1.diameter, t1.flutes, t1.flute_length), ("6 mm flat end mill", 6.0, 2, 18.0));
    assert_eq!((t1.spindle_rpm, t1.feed, t1.plunge_feed, t1.stepdown), (Some(18000.0), Some(1200.0), Some(400.0), Some(3.0)));
    assert!((t1.stepover.unwrap() - 2.4).abs() < 1e-9);
    // A preset for a material is the tool's data in it; a material only a preset names joins the list.
    let alu = t1.cutting_in("aluminium").unwrap();
    assert_eq!((alu.spindle_rpm, alu.feed, alu.plunge_feed, alu.stepdown), (Some(20000.0), Some(500.0), Some(100.0), Some(0.5)));
    assert!((alu.stepover.unwrap() - 1.5).abs() < 1e-9);
    assert_eq!(read.materials, ["Plywood", "Aluminium", "MDF"]);
    assert_eq!((tools[1].diameter, tools[1].feed, tools[1].cutting_in("MDF").and_then(|c| c.feed)), (3.0, None, Some(800.0)));
    // The V-bit with its point.
    assert_eq!((tools[2].kind, tools[2].tip_angle, tools[2].tip_diameter), (solidcraft_cam::ToolKind::VBit, Some(60.0), None));
    // An end mill rounded at its rim is a bull nose; rounded all the way, a ball.
    assert_eq!((tools[3].kind, tools[3].corner_radius), (solidcraft_cam::ToolKind::BullNose, Some(1.0)));
    assert_eq!((tools[4].kind, tools[4].corner_radius), (solidcraft_cam::ToolKind::BallEndmill, None));
    let _ = std::fs::remove_file(&library);
}

#[test]
fn contours_hold_their_part_on_tabs_and_operations_finish_and_count_their_passes() {
    let mut s = pocket_job(0.0, json!({}));
    let base = json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -6, "stepdown": 1, "ramp_angle": 0});
    let with = |extra: Value| {
        let mut p = base.clone();
        p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
        p
    };
    // Three passes instead of a 1 mm stepdown; four tabs 2 mm high with the defaults' width.
    let op = run(&mut s, "cam.op.contour", with(json!({"passes": 3, "tabs": {"height": "2 mm"}})));
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!((first["passes"].clone(), first["inputs"]["tabs"].clone()), (json!(3), json!({"count": 4, "width": "6 mm", "height": "2 mm"})));
    let path = run(&mut s, "cam.toolpath", json!({"op": op["operation"], "heights": true}));
    assert_eq!(path["operations"][0]["heights"]["levels"], json!([-2.0, -4.0, -6.0]));
    // Below the tabs' top (−4) the last pass lifts over each of them: the cut is longer by 4 × 2 × 2 mm.
    let plain = run(&mut s, "cam.op.contour", with(json!({"passes": 3})));
    close(op["cut_length"].as_f64().unwrap() - plain["cut_length"].as_f64().unwrap(), 16.0, 1e-6);
    // A finishing allowance adds one lap at full depth; edited away, and passes back to the stepdown.
    let finished = run(&mut s, "cam.op.edit", json!({"op": plain["operation"], "finish": "0.5 mm"}));
    assert!(finished["cut_length"].as_f64().unwrap() > plain["cut_length"].as_f64().unwrap() + 140.0);
    run(&mut s, "cam.op.edit", json!({"op": plain["operation"], "finish": null, "passes": null, "tabs": {"count": 2}}));
    let listed = run(&mut s, "cam.list", json!({}));
    let second = &listed["setups"][0]["operations"][1];
    assert!(second.get("passes").is_none() && second["inputs"].get("finish").is_none());
    assert_eq!(second["inputs"]["tabs"]["count"], 2);
    let pocket = run(&mut s, "cam.op.pocket", with(json!({"finish": "0.3 mm", "stepover": 2.5})));
    assert!(pocket["cut_length"].as_f64().unwrap() > 0.0);
    // Saved and opened again, all of it is kept.
    let reopened = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(reopened.manufacture, s.doc.manufacture);
    // What makes no sense is refused, naming its input.
    for (cmd, bad) in [
        ("cam.op.contour", with(json!({"passes": 0}))),
        ("cam.op.contour", with(json!({"tabs": {"count": 0}}))),
        ("cam.op.contour", with(json!({"tabs": {"height": "6 mm"}}))),
        ("cam.op.contour", with(json!({"tabs": 3}))),
        ("cam.op.contour", with(json!({"side": "on", "finish": "1 mm"}))),
        ("cam.op.pocket", with(json!({"finish": "-1 mm"}))),
    ] {
        assert!(s.execute(cmd, &bad).is_err(), "{cmd} {bad}");
    }
    assert!(s.execute("cam.op.edit", &json!({"op": op["operation"], "finish": "x"})).is_err());
}

#[test]
fn adaptive_clears_a_pocket_and_is_edited_and_saved() {
    let mut s = pocket_job(0.0, json!({}));
    let op = run(&mut s, "cam.op.adaptive", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 15000, "feed": 900, "bottom": -6, "stepdown": 3}));
    assert_eq!(op["lowest_z"], -6.0);
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!(first["kind"], "adaptive");
    assert_eq!(
        (first["inputs"]["stepover_percent"].clone(), first["inputs"]["helix_angle"].clone(), first["inputs"]["helix_max"].clone()),
        (json!(20.0), json!("5 deg"), json!(100.0))
    );
    // A lighter cut is longer; a finishing allowance and rest machining are kept with it.
    let light = run(&mut s, "cam.op.edit", json!({"op": op["operation"], "stepover_percent": 10}));
    assert!(light["cut_length"].as_f64().unwrap() > op["cut_length"].as_f64().unwrap() * 1.3);
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 3, "spindle_rpm": 20000, "feed": 600}));
    let rest = run(&mut s, "cam.op.adaptive", json!({"sketch": "Pocket", "tool": 2, "bottom": -6, "rest_from": 1, "finish": "0.2 mm"}));
    assert!(rest["cut_length"].as_f64().unwrap() < light["cut_length"].as_f64().unwrap() * 0.5);
    // Light steps may go faster: kept, listed, cleared again; the cut takes less time.
    let quick = run(&mut s, "cam.op.edit", json!({"op": op["operation"], "feed_up": 150}));
    assert!(quick["cut_minutes"].as_f64().unwrap() < light["cut_minutes"].as_f64().unwrap());
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["setups"][0]["operations"][0]["inputs"]["feed_up"], json!(150.0));
    let reopened = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(reopened.manufacture, s.doc.manufacture);
    run(&mut s, "cam.op.edit", json!({"op": op["operation"], "feed_up": null}));
    assert!(run(&mut s, "cam.list", json!({}))["setups"][0]["operations"][0]["inputs"].get("feed_up").is_none());
    for bad in [json!(50), json!(600), json!("x")] {
        assert!(s.execute("cam.op.edit", &json!({"op": op["operation"], "feed_up": bad})).is_err(), "{bad}");
    }
    assert!(s.execute("cam.tool.remove", &json!({"number": 1})).is_err(), "the rest pass needs tool 1");
    for bad in
        [json!({"stepover_percent": 80}), json!({"stepover_percent": "x"}), json!({"helix_min": 50, "helix_max": 20}), json!({"helix_angle": "x"})]
    {
        let mut q = json!({"sketch": "Pocket", "tool": 1, "bottom": -6, "spindle_rpm": 15000, "feed": 900});
        q.as_object_mut().unwrap().extend(bad.as_object().unwrap().clone());
        assert!(s.execute("cam.op.adaptive", &q).is_err(), "{bad}");
    }
}

#[test]
fn a_circle_drawn_inside_a_pocket_is_an_island_not_a_second_area() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "sketch.edit", json!({"sketch": "Pocket"}));
    run(&mut s, "sketch.circle.center", json!({"center": [20, 15], "radius": 5}));
    run(&mut s, "sketch.finish", json!({}));
    let base = json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 15000, "feed": 900, "bottom": -4, "stepdown": 2});
    let with = |extra: Value| {
        let mut q = base.clone();
        q.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
        q
    };
    for (cmd, p) in [
        ("cam.op.pocket", base.clone()),
        ("cam.op.adaptive", base.clone()),
        ("cam.op.contour", with(json!({"side": "inside"}))),
        ("cam.op.contour", with(json!({"side": "on"}))),
    ] {
        let op = run(&mut s, cmd, p.clone());
        let path = run(&mut s, "cam.toolpath", json!({"op": op["operation"]}));
        let points = path["operations"][0]["points"].as_array().unwrap().clone();
        // The tool's edge stays off the island (on the line, its centre rides the circle).
        let keep = if p["side"] == "on" { 5.0 } else { 8.0 };
        let closest = points
            .iter()
            .filter(|q| q[3].as_bool() == Some(true) && q[2].as_f64().unwrap() < -1e-6)
            .map(|q| (q[0].as_f64().unwrap() - 20.0).hypot(q[1].as_f64().unwrap() - 15.0))
            .fold(f64::INFINITY, f64::min);
        assert!(closest > keep - 0.05, "{cmd} {p}: the tool comes within {closest} mm of the island's centre");
        assert!(closest < keep + 1.0, "{cmd} {p}: it should go round the island, closest {closest}");
        run(&mut s, "cam.op.remove", json!({"op": op["operation"]}));
    }
    // A drill still drills the circle's centre.
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "drill", "diameter": 5, "spindle_rpm": 3000, "feed": 150}));
    let drill = run(&mut s, "cam.op.drill", json!({"sketch": "Pocket", "tool": 2, "bottom": -4}));
    let path = run(&mut s, "cam.toolpath", json!({"op": drill["operation"]}));
    let points = path["operations"][0]["points"].as_array().unwrap();
    assert!(points.iter().any(|q| (q[0].as_f64().unwrap() - 20.0).abs() < 1e-6 && (q[1].as_f64().unwrap() - 15.0).abs() < 1e-6), "{path}");
}

#[test]
fn a_pocket_machines_the_rest_a_larger_tool_left() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 12, "spindle_rpm": 10000, "feed": 800}));
    let big = run(&mut s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": 2, "bottom": -4, "stepdown": 2, "stepover": 5}));
    let rest = run(
        &mut s,
        "cam.op.pocket",
        json!({"sketch": "Pocket", "tool": 1, "bottom": -4, "stepdown": 2, "spindle_rpm": 15000, "feed": 600, "rest_from": 2}),
    );
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["setups"][0]["operations"][1]["inputs"]["rest_from"], 2);
    // Only the corners: far less than the whole pocket with the same tool.
    let whole = run(&mut s, "cam.op.edit", json!({"op": rest["operation"], "rest_from": null}));
    assert!(rest["cut_length"].as_f64().unwrap() < whole["cut_length"].as_f64().unwrap() * 0.5);
    run(&mut s, "cam.op.edit", json!({"op": rest["operation"], "rest_from": 2}));
    // The larger tool stays while the rest is machined after it; no such tool, or a smaller one: refused.
    run(&mut s, "cam.op.remove", json!({"op": big["operation"]}));
    let e = s.execute("cam.tool.remove", &json!({"number": 2})).unwrap_err().to_string();
    assert!(e.contains("machines the rest of tool 2"), "{e}");
    assert!(
        s.execute("cam.op.pocket", &json!({"sketch": "Pocket", "tool": 1, "bottom": -4, "spindle_rpm": 15000, "feed": 600, "rest_from": 9})).is_err()
    );
    assert!(s.execute("cam.op.pocket", &json!({"sketch": "Pocket", "tool": 2, "bottom": -4, "rest_from": 1})).is_err());
    let reopened = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(reopened.manufacture, s.doc.manufacture);
}

#[test]
fn chamfer_cuts_the_top_edge_at_the_v_bit_s_angle() {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "vbit", "diameter": 10, "tip_angle": 90, "spindle_rpm": 18000, "feed": 900}));
    run(&mut s, "cam.setup.create", json!({}));
    // 1 mm wide at 90°: 1 mm deep, the point 0.5 mm lower; no bottom asked for.
    let op = run(&mut s, "cam.op.chamfer", json!({"sketch": "Plate", "tool": 1}));
    close(op["lowest_z"].as_f64().unwrap(), -1.5, 1e-9);
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!(
        (first["kind"].as_str(), first["inputs"]["width"].as_str(), first["inputs"]["tip_offset"].as_str()),
        (Some("chamfer"), Some("1 mm"), Some("0.5 mm"))
    );
    // What it takes off: a 1 × 1 mm triangle round the 200 mm edge, within the simulation's columns.
    let sim = run(&mut s, "cam.simulate", json!({"cell": 0.1, "field": true}));
    let removed = sim["removed_volume"].as_f64().unwrap();
    assert!((removed / 100.0 - 1.0).abs() < 0.05, "{removed}");
    // The model has no chamfer: it is no fault, neither in the listing nor in the simulation.
    assert!(sim["max_gouge"].as_f64().unwrap() < 1e-3 && sim["warnings"] == json!([]), "{}", sim["warnings"]);
    assert!(sim["field"]["deviation"].as_array().unwrap().iter().all(|d| d.as_f64().unwrap() >= 0.0));
    assert_eq!(sim["notes"][0], "Chamfers are left out of the check against the part: the model has none");
    assert!(run(&mut s, "cam.list", json!({}))["setups"][0]["operations"][0].get("warning").is_none());
    // A pocket that does cut into the part is still found, chamfer or not.
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Slot"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [20, 10], "p1": [40, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 4, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    let slot = run(&mut s, "cam.op.pocket", json!({"sketch": "Slot", "tool": 4, "bottom": -2}));
    let sim = run(&mut s, "cam.simulate", json!({"cell": 0.2}));
    assert!(sim["max_gouge"].as_f64().unwrap() > 1.9, "{sim}");
    run(&mut s, "cam.op.remove", json!({"op": slot["operation"]}));
    // Wider is deeper; on the line, too wide, or with a flat tool, refused.
    let wider = run(&mut s, "cam.op.edit", json!({"op": op["operation"], "width": "2 mm"}));
    close(wider["lowest_z"].as_f64().unwrap(), -2.5, 1e-9);
    assert!(s.execute("cam.op.chamfer", &json!({"sketch": "Plate", "tool": 1, "side": "on"})).is_err());
    assert!(s.execute("cam.op.chamfer", &json!({"sketch": "Plate", "tool": 1, "width": "6 mm"})).is_err());
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    assert!(s.execute("cam.op.chamfer", &json!({"sketch": "Plate", "tool": 2})).is_err());
    // On a picked face below the stock's top, the chamfer is on that face's edge, not at the stock's top.
    run(&mut s, "cam.setup.create", json!({"name": "Raised", "stock": {"top": 5}, "origin": {"of": "stock", "z": "max"}}));
    let picked = run(&mut s, "cam.op.chamfer", json!({"setup": "Raised", "faces": [{"point": [30, 20, 0]}], "tool": 1}));
    close(picked["lowest_z"].as_f64().unwrap(), -6.5, 1e-9);
    // On a plate with a boss, the plate's picked top is chamfered at its own height, not the boss's top.
    let mut s = plate_with_boss();
    run(&mut s, "cam.tool.define", json!({"number": 5, "kind": "vbit", "diameter": 10, "tip_angle": 90, "spindle_rpm": 18000, "feed": 900}));
    let plate = run(&mut s, "cam.op.chamfer", json!({"faces": [{"point": [5, 5, 0]}], "tool": 5}));
    close(plate["lowest_z"].as_f64().unwrap(), -1.5, 1e-9);
    let boss = run(&mut s, "cam.op.chamfer", json!({"faces": [{"point": [30, 20, 12]}], "tool": 5}));
    close(boss["lowest_z"].as_f64().unwrap(), 10.5, 1e-9);
}

#[test]
fn bore_mills_round_holes_wider_than_its_tool_through_the_part() {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
    run(&mut s, "sketch.circle.center", json!({"center": [20, 20], "radius": 8}));
    run(&mut s, "sketch.circle.center", json!({"center": [45, 20], "radius": 5}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600, "stepdown": 2}));
    run(&mut s, "cam.setup.create", json!({}));
    // Through the plate (the sketch is on its top), 2 mm down a turn: the tool's stepdown.
    let op = run(&mut s, "cam.op.bore", json!({"sketch": "Holes", "tool": 1}));
    assert_eq!(op["lowest_z"], -10.0);
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!(
        (first["kind"].as_str(), first["inputs"]["pitch"].as_str(), first["inputs"]["climb"].as_bool()),
        (Some("bore"), Some("2 mm"), Some(true))
    );
    // Both holes are cut: π × (8² + 5²) × 10 mm³, less the 16 mm hole's core (radius 8 − 2 × 3),
    // which a through hole drops; within the simulation's columns.
    let sim = run(&mut s, "cam.simulate", json!({"cell": 0.1}));
    let removed = sim["removed_volume"].as_f64().unwrap();
    assert!((removed / (std::f64::consts::PI * (89.0 - 4.0) * 10.0) - 1.0).abs() < 0.01, "{removed}");
    // A finer helix is longer; climb off turns the other way.
    let fine = run(&mut s, "cam.op.edit", json!({"op": op["operation"], "pitch": "1 mm", "climb": false}));
    assert!(fine["cut_length"].as_f64().unwrap() > op["cut_length"].as_f64().unwrap());
    // A profile that is not round, a hole the tool does not fit, a V-bit: refused.
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Square"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [5, 5], "p1": [15, 15]}));
    run(&mut s, "sketch.finish", json!({}));
    assert!(s.execute("cam.op.bore", &json!({"sketch": "Square", "tool": 1})).is_err());
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 12, "spindle_rpm": 12000, "feed": 600}));
    assert!(s.execute("cam.op.bore", &json!({"sketch": "Holes", "tool": 2})).is_err());
    run(&mut s, "cam.tool.define", json!({"number": 3, "kind": "vbit", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    assert!(s.execute("cam.op.bore", &json!({"sketch": "Holes", "tool": 3})).is_err());
    assert!(s.execute("cam.op.bore", &json!({"sketch": "Holes", "tool": 1, "pitch": "0 mm"})).is_err());
}

#[test]
fn a_setup_s_program_plays_back_to_what_its_simulation_leaves() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    run(&mut s, "cam.op.pocket", pocket_op());
    run(&mut s, "cam.op.contour", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -8, "side": "inside"}));
    let play = crate::cam_playback(&s, &json!({"columns": 80})).unwrap();
    assert_eq!((play.ops.as_slice(), play.tools.len()), (["Pocket1".to_string(), "Contour1".to_string()].as_slice(), 2));
    assert!(play.playback.steps.iter().any(|st| st.op == 1));
    let mut stock = play.stock.clone();
    play.playback.cut(&mut stock, &play.tools, 0.0, play.playback.minutes).unwrap();
    let simulated = run(&mut s, "cam.simulate", json!({"columns": 80, "field": true}));
    let z: Vec<f64> = simulated["field"]["z"].as_array().unwrap().iter().map(|v| v.as_f64().unwrap()).collect();
    assert_eq!(z.len(), stock.z.len());
    assert!(z.iter().zip(&stock.z).all(|(a, b)| (a - b).abs() < 1e-9));
    // Faster rapids, a shorter program; a rate that is no rate is refused.
    let fast = crate::cam_playback(&s, &json!({"columns": 80, "rapid_rate": 20000})).unwrap();
    assert!(fast.playback.minutes < play.playback.minutes && fast.playback.rapid_rate == 20000.0);
    assert!(crate::cam_playback(&s, &json!({"rapid_rate": 0})).is_err() && crate::cam_playback(&s, &json!({"rapid_rate": "x"})).is_err());
    // Nothing to play without operations, or without a setup.
    run(&mut s, "cam.setup.create", json!({"name": "Empty"}));
    assert!(crate::cam_playback(&s, &json!({"setup": "Empty"})).is_err());
    assert!(crate::cam_playback(&Session::default(), &json!({})).is_err());
}

#[test]
fn one_operation_simulates_and_plays_on_the_stock_the_ones_before_it_left() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    run(&mut s, "cam.op.pocket", pocket_op());
    run(&mut s, "cam.op.contour", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -8, "side": "inside"}));
    let field = |v: &Value| v["field"]["z"].as_array().unwrap().iter().map(|z| z.as_f64().unwrap()).collect::<Vec<f64>>();
    let all = run(&mut s, "cam.simulate", json!({"columns": 80, "field": true}));
    let pocket = run(&mut s, "cam.simulate", json!({"columns": 80, "field": true, "ops": ["Pocket1"]}));
    let after = run(&mut s, "cam.simulate", json!({"columns": 80, "field": true, "ops": ["Contour1"], "first": ["Pocket1"]}));
    // Only the contour reports, cutting what the pocket left; together they leave what the whole setup does.
    assert_eq!(after["operations"].as_array().unwrap().iter().map(|o| o["name"].clone()).collect::<Vec<_>>(), [json!("Contour1")]);
    let volume = |v: &Value| v["removed_volume"].as_f64().unwrap();
    assert!((volume(&pocket) + volume(&after) - volume(&all)).abs() < 1e-6, "{} + {} vs {}", volume(&pocket), volume(&after), volume(&all));
    assert!(field(&after).iter().zip(field(&all)).all(|(a, b)| (a - b).abs() < 1e-9));
    // From the raw stock the contour alone would cut more.
    let contour = run(&mut s, "cam.list", json!({}))["setups"][0]["operations"][1]["id"].clone();
    let alone = run(&mut s, "cam.simulate", json!({"columns": 80, "ops": [contour]}));
    assert!(volume(&alone) > volume(&after) + 1.0, "{} vs {}", volume(&alone), volume(&after));
    // Played, it starts from the stock the pocket left and plays only itself.
    let play = crate::cam_playback(&s, &json!({"columns": 80, "ops": ["Contour1"], "first": ["Pocket1"]})).unwrap();
    assert_eq!(play.ops, ["Contour1"]);
    assert!(play.playback.steps.iter().all(|st| st.op == 0));
    assert!(play.stock.z.iter().zip(field(&pocket)).all(|(a, b)| (a - b).abs() < 1e-9));
    // Lists that are no lists, name no operation of the setup, or nothing at all are refused.
    for bad in [json!({"ops": "Pocket1"}), json!({"ops": ["Drill9"]}), json!({"ops": []}), json!({"first": [true]}), json!({"ops": [1, 2, 3]})] {
        assert!(s.execute("cam.simulate", &bad).is_err(), "{bad}");
        assert!(crate::cam_playback(&s, &bad).is_err(), "{bad}");
    }
}

#[test]
fn a_program_says_at_its_top_what_to_check_and_waits_stops_or_cuts_air_when_asked() {
    let mut s = pocket_job(0.0, json!({"post": "grbl"}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    run(&mut s, "cam.op.pocket", pocket_op());
    run(&mut s, "cam.op.contour", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -8, "side": "outside"}));
    let plain = run(&mut s, "cam.post", json!({}));
    let text = plain["gcode"].as_str().unwrap();
    // The stock and where it is against the work zero, the tool and the time, as comments at the top.
    let head: Vec<&str> = text.lines().take_while(|l| l.starts_with('(')).collect();
    assert!(head.iter().any(|l| l.starts_with("(Stock 40 x 30 x 8 mm, top at Z0, bottom at Z-8")), "{head:?}");
    assert!(head.contains(&"(Work zero G54: the world point 0, 0, 0)"), "{head:?}");
    assert!(head.iter().any(|l| l.starts_with("(T1: D6 flat")) && head.iter().any(|l| l.starts_with("(Cutting time about")), "{head:?}");
    assert!(!text.contains("G4 P") && !text.lines().any(|l| l == "M1"), "nothing asked for, nothing added");
    let safe = run(&mut s, "cam.post", json!({"spindle_delay": 3, "optional_stops": true, "air_cut": 25}));
    let text = safe["gcode"].as_str().unwrap();
    assert!(text.contains("G4 P3") && text.lines().any(|l| l == "M1") && text.contains("AIR CUT"), "{text}");
    let lowest = |t: &str| solidcraft_cam::parse(t).unwrap().iter().filter_map(|m| m.target()).map(|p| p.z).fold(f64::INFINITY, f64::min);
    assert!((lowest(text) - lowest(plain["gcode"].as_str().unwrap()) - 25.0).abs() < 1e-6);
    for bad in [json!({"spindle_delay": -1}), json!({"spindle_delay": "x"}), json!({"air_cut": 1e6})] {
        assert!(s.execute("cam.post", &bad).is_err(), "{bad}");
    }
}

#[test]
fn a_contour_through_the_stock_without_tabs_is_warned_about() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    let contour = |bottom: f64| json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": bottom, "side": "outside"});
    run(&mut s, "cam.op.contour", contour(-8.0));
    run(&mut s, "cam.op.contour", contour(-6.0));
    let mut held = contour(-8.0);
    held["tabs"] = json!({"count": 4});
    run(&mut s, "cam.op.contour", held);
    let listed = run(&mut s, "cam.list", json!({}));
    let warning = |k: usize| listed["setups"][0]["operations"][k]["warning"].as_str().unwrap_or("").to_string();
    // Through the bottom with nothing holding it: warned; not as deep, or with tabs: not.
    assert!(warning(0).contains("without tabs"), "{}", warning(0));
    assert!(!warning(1).contains("without tabs") && !warning(2).contains("without tabs"), "{} / {}", warning(1), warning(2));
}

#[test]
fn a_setup_runs_in_its_work_offset_and_has_a_sheet_for_the_machine() {
    let mut s = pocket_job(0.0, json!({"post": "grbl", "work_offset": "G55"}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    run(&mut s, "cam.op.pocket", pocket_op());
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["setups"][0]["work_offset"], 55);
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(gcode.contains("G17 G21 G55 G90 G94") && !gcode.contains("G54") && gcode.contains("(Work zero G55:"), "{gcode}");
    // A post without the offset in its header gets it on a line of its own; G54 is the controller's usual and is not added.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "post": "lnc", "work_offset": 57}));
    let lnc = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(lnc.lines().any(|l| l == "G57"), "{lnc}");
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "work_offset": 54}));
    assert!(!run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().lines().any(|l| l == "G54"));
    // Saved only when it is not G54, and read back.
    let setup = s.doc.manufacture.setups[0].clone();
    assert!(json!(setup).get("work_offset").is_none());
    let mut moved = setup.clone();
    moved.work_offset = 59;
    let back: solidcraft_doc::manufacture::Setup = serde_json::from_value(json!(moved)).unwrap();
    assert_eq!(back.work_offset, 59);
    for bad in [json!({"work_offset": 53}), json!({"work_offset": "G60"}), json!({"work_offset": "x"}), json!({"work_offset": true})] {
        let mut p = bad.clone();
        p["setup"] = json!("Setup1");
        assert!(s.execute("cam.setup.edit", &p).is_err(), "{bad}");
    }
    // The sheet: the stock and the zero, the tools, the operations in order.
    let sheet = run(&mut s, "cam.sheet", json!({}))["text"].as_str().unwrap().to_string();
    for part in [
        "# Untitled – Setup1",
        "- Stock 40 x 30 x 8 mm",
        "Work zero G54",
        "## Tools",
        "| T1 | flat end mill | 6 mm |",
        "## Operations",
        "| 1 | Pocket1 | pocket | T1 | 12000 rev/min | 600 mm/min |",
    ] {
        assert!(sheet.contains(part), "{part} in\n{sheet}");
    }
}

#[test]
fn a_setup_starts_from_what_an_earlier_one_leaves_also_turned_over() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [15, 10], "p1": [45, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    run(&mut s, "cam.setup.create", json!({"name": "Front", "origin": top}));
    // The window cut right through: a 30 × 20 hole in the 60 × 40 plate.
    run(&mut s, "cam.op.pocket", json!({"setup": "Front", "sketch": "Window", "tool": 1, "bottom": -10, "stepdown": 5}));
    let left = run(&mut s, "cam.simulate", json!({"setup": "Front", "columns": 120, "field": true}));
    let removed = left["removed_volume"].as_f64().unwrap();
    assert!((removed - 6000.0).abs() < 300.0, "{removed}");
    run(&mut s, "cam.setup.create", json!({"name": "Again", "origin": top, "stock_from": "Front"}));
    run(&mut s, "cam.setup.create", json!({"name": "Back", "axis": "-z", "origin": top, "stock_from": "Front"}));
    // Turned over about Y: the tool from below, X the other way, Y as it was.
    run(&mut s, "cam.setup.create", json!({"name": "Side", "axis": "-z", "x_axis": "-x", "origin": top, "stock_from": "Front"}));
    let frame = run(&mut s, "cam.list", json!({}))["setups"][3]["frame"].clone();
    assert_eq!((frame["x"].clone(), frame["y"].clone()), (json!([-1.0, 0.0, 0.0]), json!([0.0, 1.0, -0.0])), "{frame}");
    for name in ["Again", "Back", "Side"] {
        run(&mut s, "cam.op.face", json!({"setup": name, "tool": 1, "bottom": -0.5}));
        let play = crate::cam_playback(&s, &json!({"setup": name, "columns": 120})).unwrap();
        // It starts with the hole already there, seen from its own side.
        let volume = play.stock.volume();
        assert!((volume - (24000.0 - removed)).abs() < 400.0, "{name}: {volume} vs {}", 24000.0 - removed);
    }
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["setups"][1]["stock_from"], listed["setups"][0]["id"]);
    // Turned over, its program says how to turn the part.
    assert!(run(&mut s, "cam.post", json!({"setup": "Back"}))["gcode"].as_str().unwrap().contains("part flipped about the X axis"));
    assert!(run(&mut s, "cam.post", json!({"setup": "Side"}))["gcode"].as_str().unwrap().contains("part flipped about the Y axis"));
    // An X direction along the tool axis is no direction across it.
    assert!(s.execute("cam.setup.edit", &json!({"setup": "Side", "x_axis": "+z"})).is_err());
    // Only an earlier setup, facing the same way or turned over; removing it leaves the later ones on their own stock.
    for bad in
        [json!({"setup": "Front", "stock_from": "Back"}), json!({"setup": "Again", "stock_from": "Nowhere"}), json!({"setup": "Again", "axis": "+x"})]
    {
        assert!(s.execute("cam.setup.edit", &bad).is_err(), "{bad}");
    }
    run(&mut s, "cam.setup.remove", json!({"setup": "Front"}));
    assert!(s.doc.manufacture.setups.iter().all(|x| x.stock_from.is_none()));
}

#[test]
fn a_turned_over_setup_sees_what_was_cut_from_the_other_side() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [15, 10], "p1": [45, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    // A pocket 4 deep from the top, then the same window 7 deep from below: only the 6 mm left
    // between them is cut, and the window goes through.
    run(&mut s, "cam.setup.create", json!({"name": "Front", "origin": top}));
    run(&mut s, "cam.op.pocket", json!({"setup": "Front", "sketch": "Window", "tool": 1, "bottom": -4, "stepdown": 4}));
    let front = run(&mut s, "cam.simulate", json!({"setup": "Front", "columns": 120}))["removed_volume"].as_f64().unwrap();
    run(&mut s, "cam.setup.create", json!({"name": "Back", "axis": "-z", "origin": top, "stock_from": "Front"}));
    run(&mut s, "cam.op.pocket", json!({"setup": "Back", "sketch": "Window", "tool": 1, "bottom": -7, "stepdown": 7}));
    let play = crate::cam_playback(&s, &json!({"setup": "Back", "columns": 120})).unwrap();
    assert!(!play.stock.bottom.is_empty(), "the pocket is the bottom of the columns seen from below");
    close(play.stock.volume(), 24000.0 - front, 1.0);
    let back = run(&mut s, "cam.simulate", json!({"setup": "Back", "columns": 120, "field": true}));
    let removed = back["removed_volume"].as_f64().unwrap();
    close(removed, front / 4.0 * 6.0, front / 4.0 * 0.3);
    // Along the middle row, the window's columns (half the width) are empty.
    let field = &back["field"];
    let (nx, j) = (field["nx"].as_u64().unwrap() as usize, field["ny"].as_u64().unwrap() as usize / 2);
    let floor = field["floor"].as_f64().unwrap();
    let middle = field["z"].as_array().unwrap().iter().zip(field["bottom"].as_array().unwrap()).skip(j * nx).take(nx);
    assert!(middle.filter(|(z, b)| z.as_f64() == Some(floor) && b.as_f64() == Some(floor)).count() > nx / 3);
}

#[test]
fn a_template_carries_a_setup_s_operations_to_another_design() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "cam.op.pocket", pocket_op());
    run(&mut s, "cam.op.contour", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -5, "side": "outside"}));
    let template = run(&mut s, "cam.template.save", json!({}))["template"].clone();
    assert_eq!(template["operations"][0]["sketch_name"], "Pocket");
    // Another design: its sketch of that name has another id, and it has no tool yet.
    let mut other = Session::default();
    run(&mut other, "sketch.create", json!({"plane": "XY", "name": "Other"}));
    run(&mut other, "sketch.circle.center", json!({"center": [5, 5], "radius": 2}));
    run(&mut other, "sketch.finish", json!({}));
    run(&mut other, "sketch.create", json!({"plane": "XY", "name": "Pocket"}));
    run(&mut other, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [50, 20]}));
    run(&mut other, "sketch.finish", json!({}));
    run(&mut other, "cam.setup.create", json!({}));
    let added = run(&mut other, "cam.template.apply", json!({"template": template}));
    assert_eq!(added["added"], json!(["Pocket1", "Contour1"]));
    assert!(other.doc.manufacture.tool(1).is_some(), "the tool came with it");
    let listed = run(&mut other, "cam.list", json!({}));
    let ops = listed["setups"][0]["operations"].as_array().unwrap().clone();
    assert!(ops.iter().all(|o| o.get("error").is_none() && o["toolpath"]["cut_length"].as_f64().unwrap() > 0.0), "{ops:?}");
    // Again: its names are made unique.
    let again = run(&mut other, "cam.template.apply", json!({"template": template}));
    assert_eq!(again["added"], json!(["Pocket2", "Contour2"]));
    // A sketch the design does not have: added all the same, saying so.
    let mut lone = Session::default();
    run(&mut lone, "cam.setup.create", json!({}));
    run(&mut lone, "cam.template.apply", json!({"template": template}));
    let listed = run(&mut lone, "cam.list", json!({}));
    assert!(listed["setups"][0]["operations"][0]["error"].as_str().is_some());
    for bad in [
        json!({"template": {"format": "x", "operations": []}}),
        json!({"template": {"format": "solidcraft-cam-template", "operations": "no"}}),
        json!({}),
    ] {
        assert!(other.execute("cam.template.apply", &bad).is_err(), "{bad}");
    }
}

#[test]
fn a_contour_leads_in_on_arcs_cuts_dogbones_and_leaves_an_onion_skin() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Pocket", "distance": 8, "direction": "negative"}));
    let lowest = |s: &mut Session, k: usize| run(s, "cam.list", json!({}))["setups"][0]["operations"][k]["toolpath"]["lowest_z"].as_f64().unwrap();
    run(&mut s, "cam.op.contour", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -8, "lead": 3, "skin": 0.4}));
    // The skin holds it: not through, so no warning of a part coming loose.
    assert!((lowest(&mut s, 0) + 7.6).abs() < 1e-9);
    let listed = run(&mut s, "cam.list", json!({}));
    assert!(listed["setups"][0]["operations"][0].get("warning").is_none_or(|w| !w.as_str().unwrap_or("").contains("without tabs")));
    run(&mut s, "cam.op.edit", json!({"op": "Contour1", "skin_last": true}));
    assert!((lowest(&mut s, 0) + 8.0).abs() < 1e-9);
    // Edited off again, and dogbones in a pocket; what makes no sense is refused.
    run(&mut s, "cam.op.edit", json!({"op": "Contour1", "skin": null, "lead": null, "dogbones": true}));
    let op = &s.doc.manufacture.setups[0].operations[0];
    assert!(matches!(&op.kind, solidcraft_doc::manufacture::CamOperationKind::Contour { skin: None, lead: None, dogbones: true, .. }));
    run(&mut s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -3, "dogbones": true}));
    assert!(matches!(&s.doc.manufacture.setups[0].operations[1].kind, solidcraft_doc::manufacture::CamOperationKind::Pocket { dogbones: true, .. }));
    for bad in [json!({"op": "Contour1", "skin": 20}), json!({"op": "Contour1", "dogbones": "x"}), json!({"op": "Pocket1", "lead": 3})] {
        assert!(s.execute("cam.op.edit", &bad).is_err(), "{bad}");
    }
    // Lead-in arcs and tabs together.
    let together = s.execute(
        "cam.op.contour",
        &json!({"sketch": "Pocket", "tool": 1, "spindle_rpm": 12000, "feed": 600, "bottom": -8, "lead": 3, "tabs": {"count": 2}}),
    );
    assert!(together.is_ok(), "{together:?}");
}

#[test]
fn clamps_in_the_way_and_a_tool_too_short_are_warned_about() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [15, 10], "p1": [45, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600, "length": 20}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    // A clamp along the far edge, out of the way.
    run(&mut s, "cam.setup.create", json!({"origin": top, "clamps": [{"name": "Edge", "min": [52, 0], "max": [60, 40], "top": 5}]}));
    run(&mut s, "cam.op.pocket", json!({"sketch": "Window", "tool": 1, "bottom": -3}));
    let warning = |s: &mut Session| run(s, "cam.list", json!({}))["setups"][0]["operations"][0]["warning"].as_str().unwrap_or("").to_string();
    assert!(!warning(&mut s).contains("clamp"), "{}", warning(&mut s));
    // One over the window: the pocket runs into it.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "clamps": [{"name": "Vice", "min": [20, 0], "max": [25, 40], "top": 5}]}));
    assert!(warning(&mut s).contains("runs into clamp Vice"), "{}", warning(&mut s));
    assert_eq!(run(&mut s, "cam.list", json!({}))["setups"][0]["clamps"][0]["name"], "Vice");
    // A tall clamp beside the window: the tool keeps clear of it, its wide holder does not.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "clamps": [{"name": "Tall", "min": [52, 0], "max": [60, 40], "top": 30}]}));
    assert!(!warning(&mut s).contains("clamp"), "{}", warning(&mut s));
    run(
        &mut s,
        "cam.tool.define",
        json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600, "length": 20, "holder_diameter": 30, "holder_length": 40}),
    );
    assert!(warning(&mut s).contains("the holder runs into clamp Tall"), "{}", warning(&mut s));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600, "length": 20}));
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "clamps": null}));
    assert!(s.doc.manufacture.setups[0].clamps.is_empty());
    // A tool that reaches 2 mm out of its holder cannot go 3 mm deep.
    run(
        &mut s,
        "cam.tool.define",
        json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600, "flute_length": 2, "length": 2}),
    );
    let w = warning(&mut s);
    assert!(w.contains("goes 3.0 mm deep but the tool reaches 2.0 mm out of its holder"), "{w}");
    for bad in
        [json!({"clamps": "x"}), json!({"clamps": [{"min": [5, 5], "max": [1, 9], "top": 1}]}), json!({"clamps": [{"min": [0, 0], "max": [1, 1]}]})]
    {
        let mut p = bad.clone();
        p["setup"] = json!("Setup1");
        assert!(s.execute("cam.setup.edit", &p).is_err(), "{bad}");
    }
}

#[test]
fn ordering_by_tool_loads_each_tool_once() {
    let mut s = pocket_job(0.0, json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 3}));
    let op = |s: &mut Session, tool: u32, bottom: f64| {
        run(s, "cam.op.pocket", json!({"sketch": "Pocket", "tool": tool, "spindle_rpm": 12000, "feed": 600, "bottom": bottom}));
    };
    op(&mut s, 1, -2.0);
    op(&mut s, 2, -2.0);
    op(&mut s, 1, -4.0);
    op(&mut s, 2, -4.0);
    assert_eq!(run(&mut s, "cam.list", json!({}))["setups"][0]["tool_changes"], 4);
    let ordered = run(&mut s, "cam.setup.order", json!({}));
    // Tool 1's two, in their order, then tool 2's.
    assert_eq!(ordered["order"], json!(["Pocket1", "Pocket3", "Pocket2", "Pocket4"]));
    assert_eq!(ordered["tool_changes"], json!([4, 2]));
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(gcode.contains("(Tools loaded: 2)"), "{gcode}");
}

#[test]
fn a_v_carve_carves_letters_with_a_v_bit_to_the_depth_the_shape_asks() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Logo"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [50, 18]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 3, "kind": "vbit", "diameter": 12, "tip_angle": 90, "spindle_rpm": 18000, "feed": 800}));
    run(&mut s, "cam.setup.create", json!({"origin": {"of": "stock", "x": "min", "y": "min", "z": "max"}}));
    run(&mut s, "cam.op.vcarve", json!({"sketch": "Logo", "tool": 3, "stepover": 0.2}));
    // An 8 mm wide strip at 90°: 4 mm deep in the middle; no warning for cutting into the model.
    let listed = run(&mut s, "cam.list", json!({}));
    let op = &listed["setups"][0]["operations"][0];
    assert_eq!(op["kind"], "vcarve");
    let lowest = op["toolpath"]["lowest_z"].as_f64().unwrap();
    assert!((lowest + 4.0).abs() < 0.25 && op.get("warning").is_none(), "{op}");
    run(&mut s, "cam.op.edit", json!({"op": "VCarve1", "max_depth": 1.5}));
    let lowest = run(&mut s, "cam.list", json!({}))["setups"][0]["operations"][0]["toolpath"]["lowest_z"].as_f64().unwrap();
    assert!((lowest + 1.5).abs() < 1e-6, "{lowest}");
    // Only with a V-bit.
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    assert!(s.execute("cam.op.vcarve", &json!({"sketch": "Logo", "tool": 1})).is_err());
}

#[test]
fn a_thread_is_milled_in_a_hole_drawn_at_its_core_diameter() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Holes"}));
    // M8 × 1.25: drawn at its core, 8 − 1.0825 × 1.25 = 6.647 mm.
    run(&mut s, "sketch.circle.center", json!({"center": [20, 20], "radius": 3.3234375}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 4, "kind": "flat", "diameter": 5, "spindle_rpm": 10000, "feed": 300}));
    run(&mut s, "cam.setup.create", json!({"origin": {"of": "stock", "x": "min", "y": "min", "z": "max"}}));
    run(&mut s, "cam.op.thread", json!({"sketch": "Holes", "tool": 4, "pitch": 1.25, "bottom": -8, "thread_passes": 2}));
    let listed = run(&mut s, "cam.list", json!({}));
    let op = &listed["setups"][0]["operations"][0];
    assert!(op["kind"] == "thread" && op.get("error").is_none(), "{op}");
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    // The last pass runs 1.5 mm from the centre: the 8 mm major diameter less the tool's radius.
    let moves = solidcraft_cam::parse(&gcode).unwrap();
    let farthest = moves.iter().filter_map(|m| m.target()).map(|p| (p.x - 20.0).hypot(p.y - 20.0)).filter(|r| *r < 10.0).fold(0.0, f64::max);
    assert!((farthest - 1.5).abs() < 0.01, "{farthest}");
    run(&mut s, "cam.op.edit", json!({"op": "Thread1", "diameter": 7.5, "thread_passes": 1}));
    assert!(matches!(
        &s.doc.manufacture.setups[0].operations[0].kind,
        solidcraft_doc::manufacture::CamOperationKind::Thread { passes: 1, diameter: Some(_), .. }
    ));
    assert!(s.execute("cam.op.thread", &json!({"sketch": "Holes", "tool": 4, "bottom": -8})).is_err(), "a pitch is needed");
    // Left-hand: from the top down.
    run(&mut s, "cam.op.edit", json!({"op": "Thread1", "hand": "left"}));
    assert!(matches!(&s.doc.manufacture.setups[0].operations[0].kind, solidcraft_doc::manufacture::CamOperationKind::Thread { left_hand: true, .. }));
    assert!(s.execute("cam.op.edit", &json!({"op": "Thread1", "hand": "up"})).is_err());
}

#[test]
fn copies_of_the_part_are_each_cut_by_every_operation() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [15, 10], "p1": [45, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.setup.create", json!({"origin": {"of": "stock", "x": "min", "y": "min", "z": "max"}}));
    run(&mut s, "cam.op.pocket", json!({"sketch": "Window", "tool": 1, "bottom": -3}));
    run(&mut s, "cam.op.face", json!({"tool": 1, "top": 0, "bottom": -0.5}));
    let face_x = |s: &mut Session| {
        let gcode = run(s, "cam.post", json!({"op": "Face1"}))["gcode"].as_str().unwrap().to_string();
        solidcraft_cam::parse(&gcode).unwrap().iter().filter_map(|m| m.target()).map(|p| p.x).fold(0.0, f64::max)
    };
    let alone = face_x(&mut s);
    run(&mut s, "cam.op.remove", json!({"op": "Face1"}));
    let one = run(&mut s, "cam.simulate", json!({"columns": 150}))["removed_volume"].as_f64().unwrap();
    let length = |s: &mut Session| run(s, "cam.list", json!({}))["setups"][0]["operations"][0]["toolpath"]["cut_length"].as_f64().unwrap();
    let single = length(&mut s);
    // Three side by side, 70 mm apart: three times the cut, the stock reaching over all of them.
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "copies": {"nx": 3, "ny": 1, "dx": 70}}));
    let listed = run(&mut s, "cam.list", json!({}));
    assert!((listed["setups"][0]["stock_box"]["max"][0].as_f64().unwrap() - 200.0).abs() < 1e-6, "{}", listed["setups"][0]["stock_box"]);
    assert!((length(&mut s) - 3.0 * single).abs() < 1e-6);
    let three = run(&mut s, "cam.simulate", json!({"columns": 300}))["removed_volume"].as_f64().unwrap();
    assert!((three - 3.0 * one).abs() < 0.05 * one, "{three} vs 3 × {one}");
    let gcode = run(&mut s, "cam.post", json!({}))["gcode"].as_str().unwrap().to_string();
    assert!(gcode.contains("(Copies: 3 x 1, 70 x 0 mm apart)"));
    let xs = solidcraft_cam::parse(&gcode).unwrap().iter().filter_map(|m| m.target()).map(|p| p.x).fold(0.0, f64::max);
    assert!(xs > 140.0 + 15.0, "the last copy's window: {xs}");
    // Facing the stock is not repeated: the stock already reaches over every copy.
    run(&mut s, "cam.op.face", json!({"tool": 1, "top": 0, "bottom": -0.5}));
    let wide = face_x(&mut s);
    assert!(wide > alone + 130.0 && wide < 200.0 + 10.0, "{alone} → {wide}");
    let lengths = run(&mut s, "cam.list", json!({}))["setups"][0]["operations"].clone();
    let face_moves = lengths[1]["toolpath"]["moves"].as_u64().unwrap();
    assert!(face_moves < 400, "{face_moves}");
    run(&mut s, "cam.setup.edit", json!({"setup": "Setup1", "copies": null}));
    assert!(s.doc.manufacture.setups[0].copies.is_none());
    for bad in [json!({"nx": 2}), json!({"nx": 0}), json!({"nx": 2000, "dx": 1}), json!({"nx": 2, "dx": -5})] {
        assert!(s.execute("cam.setup.edit", &json!({"setup": "Setup1", "copies": bad})).is_err(), "{bad}");
    }
}

#[test]
fn operations_take_the_cutting_data_for_their_setups_material() {
    let lib = library_file("materials");
    let mut s = pocket_job(0.0, json!({"material": "MDF"}));
    // Tool 1 cuts MDF faster than anything else; in aluminium it has only a feed of its own.
    let cutting = json!([
        {"material": "mdf", "spindle_rpm": 18000, "chip_load": 0.05, "stepdown": 4},
        {"material": "Aluminium", "feed": 300},
        {"material": "Brass"}
    ]);
    let tool =
        run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 900, "cutting": cutting}));
    // The chip load in MDF is kept (its feed is worked out at its own speed); a material given nothing is left out.
    assert_eq!(
        tool["tool"]["cutting"],
        json!([{"material": "mdf", "spindle_rpm": 18000.0, "chip_load": 0.05, "stepdown": 4.0}, {"material": "Aluminium", "feed": 300.0}])
    );
    let op = |s: &mut Session, setup: &str| {
        run(s, "cam.op.pocket", json!({"setup": setup, "sketch": "Pocket", "tool": 1, "bottom": -5}));
        let listed = run(s, "cam.list", json!({}));
        let setup = listed["setups"].as_array().unwrap().iter().find(|x| x["name"] == setup).unwrap().clone();
        let op = setup["operations"][0].clone();
        (op["spindle_rpm"].as_f64(), op["cut_feed"].as_f64(), op["stepdown"].clone())
    };
    assert_eq!(op(&mut s, "Setup1"), (Some(18000.0), Some(1800.0), json!("4 mm")));
    // In aluminium the speed is the tool's own; with no material, all of it is.
    run(&mut s, "cam.setup.create", json!({"name": "Alu", "material": "Aluminium"}));
    assert_eq!(op(&mut s, "Alu"), (Some(12000.0), Some(300.0), json!("3 mm")));
    run(&mut s, "cam.setup.create", json!({"name": "Plain"}));
    assert_eq!(op(&mut s, "Plain"), (Some(12000.0), Some(900.0), json!("3 mm")));
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["materials"], json!(["MDF", "Aluminium"]));
    assert_eq!(listed["setups"][1]["material"], "Aluminium");

    // A tool with no feed of its own and none in the setup's material says which is missing.
    run(
        &mut s,
        "cam.tool.define",
        json!({"number": 2, "kind": "flat", "diameter": 3, "spindle_rpm": 15000, "cutting": [{"material": "MDF", "feed": 700}]}),
    );
    let e = s.execute("cam.op.pocket", &json!({"setup": "Alu", "sketch": "Pocket", "tool": 2, "bottom": -5})).unwrap_err().to_string();
    assert!(e.contains("`feed` is required: tool 2 has none for Aluminium and none of its own"), "{e}");
    // A setup's material is changed and cleared; a design saved and opened again keeps it.
    run(&mut s, "cam.setup.edit", json!({"setup": "Plain", "material": "Pine"}));
    run(&mut s, "cam.setup.edit", json!({"setup": "Alu", "material": null}));
    let reopened = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(reopened.manufacture, s.doc.manufacture);
    let materials: Vec<_> = reopened.manufacture.setups.iter().map(|x| x.material.clone()).collect();
    assert_eq!(materials, [Some("MDF".to_string()), None, Some("Pine".to_string())]);
    for bad in [json!({"setup": "Plain", "material": 3}), json!({"setup": "Plain", "material": "x".repeat(65)})] {
        assert!(s.execute("cam.setup.edit", &bad).is_err(), "{bad}");
    }
    assert!(
        s.execute(
            "cam.tool.define",
            &json!({"number": 3, "kind": "flat", "diameter": 3, "cutting": [{"material": "MDF"}, {"material": "mdf", "feed": 1}]})
        )
        .is_err()
    );
    assert!(
        s.execute("cam.tool.define", &json!({"number": 3, "kind": "flat", "diameter": 3, "cutting": [{"material": "MDF", "feed": -1}]})).is_err()
    );

    // The machine's library names materials: those defined, and those its tools have data for.
    run(&mut s, "cam.material.define", json!({"library": lib, "name": "Plywood"}));
    run(&mut s, "cam.material.define", json!({"library": lib, "name": " plywood "}));
    run(&mut s, "cam.library.add", json!({"library": lib, "number": 1}));
    let listed = run(&mut s, "cam.library.list", json!({"library": lib}));
    assert_eq!(listed["materials"], json!(["Plywood", "mdf", "Aluminium"]));
    // Removing one takes it from the library's tools too; the design keeps its copy.
    let removed = run(&mut s, "cam.material.remove", json!({"library": lib, "name": "MDF"}));
    assert_eq!(removed["tools"], json!([1]));
    let listed = run(&mut s, "cam.library.list", json!({"library": lib}));
    assert_eq!(listed["materials"], json!(["Plywood", "Aluminium"]));
    assert_eq!(listed["tools"][0]["design"], "differs");
    assert!(s.doc.manufacture.tool(1).and_then(|t| t.cutting_in("MDF")).is_some());
    assert!(s.execute("cam.material.remove", &json!({"library": lib, "name": "MDF"})).is_err());
    assert!(s.execute("cam.material.define", &json!({"library": lib, "name": "  "})).is_err());
    let _ = std::fs::remove_file(&lib);
}

#[test]
fn a_surface_speed_and_chip_load_give_speed_and_feed_for_the_tools_size() {
    let mut s = pocket_job(0.0, json!({"material": "Aluminium"}));
    // 6 mm, 2 flutes, 150 m/min and 0.03 mm per tooth: 150000 / (π × 6) = 7958 rev/min, × 2 × 0.03 = 477.5 mm/min.
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "surface_speed": 150, "chip_load": 0.03}));
    let pocket = |s: &mut Session, extra: Value| {
        let mut p = json!({"sketch": "Pocket", "tool": 1, "bottom": -2});
        for (k, v) in extra.as_object().unwrap() {
            p[k] = v.clone();
        }
        run(s, "cam.op.pocket", p);
        let op = s.doc.manufacture.setups[0].operations.last().unwrap();
        (op.spindle_rpm, op.cut_feed)
    };
    assert_eq!(pocket(&mut s, json!({})), (7958.0, 477.5));
    // A smaller tool at the same surface speed turns faster and, at the same chip load, feeds faster.
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 3}));
    assert_eq!(pocket(&mut s, json!({})), (15915.0, 954.9));
    // The machine's spindle caps the speed, and the chip load then feeds at the capped speed.
    assert_eq!(pocket(&mut s, json!({"max_spindle_rpm": 12000})), (12000.0, 720.0));
    // A speed given for the operation is what the chip load feeds at; a given feed wins.
    assert_eq!(pocket(&mut s, json!({"spindle_rpm": 10000})), (10000.0, 600.0));
    assert_eq!(pocket(&mut s, json!({"feed": 500})), (15915.0, 500.0));
    assert!(s.execute("cam.op.pocket", &json!({"sketch": "Pocket", "tool": 1, "bottom": -2, "max_spindle_rpm": 0})).is_err());
    // A material's speed or feed replaces the tool's however each gives it: in aluminium this tool
    // turns at 9000 and keeps its own chip load.
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 3, "cutting": [{"material": "aluminium", "spindle_rpm": 9000}]}));
    assert_eq!(pocket(&mut s, json!({})), (9000.0, 540.0));
    // The design keeps the inputs, not what they work out to.
    let saved = serde_json::to_value(s.doc.manufacture.tool(1).unwrap()).unwrap();
    assert_eq!((saved["surface_speed"].as_f64(), saved["chip_load"].as_f64(), saved.get("spindle_rpm")), (Some(150.0), Some(0.03), None));
}

#[test]
fn a_v_bit_engraves_on_a_line_and_its_groove_is_a_v() {
    // A flat plate, its top at 0 and the stock's too.
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [30, 20]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Plate", "distance": 5, "direction": "negative"}));
    run(&mut s, "cam.setup.create", json!({}));
    // A 90° V-bit, 6 mm across, with a 0.2 mm flat at its point.
    run(
        &mut s,
        "cam.tool.define",
        json!({"number": 5, "kind": "vbit", "diameter": 6, "tip_angle": 90, "tip_diameter": 0.2, "flutes": 2, "spindle_rpm": 18000, "feed": 800}),
    );
    let tool = s.doc.manufacture.tools.iter().find(|t| t.number == 5).unwrap().clone();
    assert_eq!((tool.kind, tool.point_angle(), tool.tip()), (solidcraft_cam::ToolKind::VBit, 90.0, 0.2));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Line"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [2, 2], "p1": [12, 8]}));
    run(&mut s, "sketch.finish", json!({}));
    // Engraving on the outline, 1 mm deep: allowed; a pocket or a contour beside the line is not.
    run(&mut s, "cam.op.contour", json!({"sketch": "Line", "tool": 5, "side": "on", "top": 0, "bottom": -1, "name": "Engrave"}));
    for (cmd, extra) in [("cam.op.pocket", json!({})), ("cam.op.contour", json!({"side": "outside"}))] {
        let mut p = json!({"sketch": "Line", "tool": 5, "top": 0, "bottom": -1});
        p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
        let e = s.execute(cmd, &p).unwrap_err().to_string();
        assert!(e.contains("V-bit"), "{cmd}: {e}");
    }
    // The groove the simulation cuts on the line is 1 mm deep in the middle and 1 mm up 1 mm aside.
    let sim = run(&mut s, "cam.simulate", json!({"field": true, "columns": 600}));
    let f = &sim["field"];
    let (nx, cell) = (f["nx"].as_u64().unwrap() as usize, f["cell"].as_f64().unwrap());
    let (ox, oy) = (f["origin"][0].as_f64().unwrap(), f["origin"][1].as_f64().unwrap());
    let z = |x: f64, y: f64| {
        let (i, j) = (((x - ox) / cell) as usize, ((y - oy) / cell) as usize);
        f["z"][j * nx + i].as_f64().unwrap()
    };
    // On the line, 1 mm deep; half a millimetre beside it, 0.6 mm (a 90° V with a 0.2 mm flat);
    // 1.2 mm beside it, untouched.
    close(z(7.0, 5.0), 0.0, 1e-9);
    close(z(7.0, 2.0), -1.0, 0.05);
    close(z(7.0, 2.5), -0.6, 0.05);
    close(z(7.0, 3.2), 0.0, 1e-9);
}

#[test]
fn adaptive_3d_roughs_the_stock_down_to_the_model() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 60, "width": 40, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [15, 10], "p1": [45, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Window", "distance": -5, "operation": "cut"}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "ball", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.tool.define", json!({"number": 3, "kind": "drill", "diameter": 6, "spindle_rpm": 3000, "feed": 200}));
    // A bull nose needs its corner radius; with it, it roughs too.
    assert!(s.execute("cam.tool.define", &json!({"number": 4, "kind": "bull", "diameter": 6})).is_err());
    let bull =
        run(&mut s, "cam.tool.define", json!({"number": 4, "kind": "bull", "diameter": 6, "corner_radius": 1, "spindle_rpm": 12000, "feed": 600}));
    assert_eq!((bull["tool"]["kind"].as_str(), bull["tool"]["corner_radius"].as_f64()), (Some("bull_nose"), Some(1.0)));
    // 2 mm of stock over the plate; the work zero on its top.
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    run(&mut s, "cam.setup.create", json!({"name": "Rough", "origin": top, "stock": {"top": 2}}));
    let made = run(&mut s, "cam.op.adaptive3d", json!({"tool": 1, "stepdown": 3, "stock_to_leave": 0.2}));
    // No sketch: the whole stock, down to the pocket's floor (7 below the zero) less the allowance.
    close(made["lowest_z"].as_f64().unwrap(), -6.8, 1e-9);
    let listed = run(&mut s, "cam.list", json!({}));
    let op = &listed["setups"][0]["operations"][0];
    assert_eq!(op["kind"], "adaptive_3d", "{op}");
    assert!(op.get("warning").is_none_or(Value::is_null), "{op}");
    let sim = run(&mut s, "cam.simulate", json!({"setup": "Rough", "columns": 240}));
    // The skin over the plate and the pocket, less the allowance and the tool's round in the
    // pocket's corners; nothing cut into the plate.
    let removed = sim["removed_volume"].as_f64().unwrap();
    let skin = 60.0 * 40.0 * 1.8;
    let pocket = 30.0 * 20.0 * 4.8;
    assert!(removed > (skin + pocket) * 0.95 && removed < skin + pocket + 100.0, "{removed}");
    assert!(sim["max_gouge"].as_f64().unwrap() < 0.05, "{sim}");
    // A flat or ball end mill; no finishing lap or rest machining in 3D yet, no negative step-up.
    for bad in [
        json!({"setup": "Rough", "tool": 3}),
        json!({"setup": "Rough", "tool": 1, "finish": 0.5}),
        json!({"setup": "Rough", "tool": 1, "rest_from": 2}),
        json!({"setup": "Rough", "tool": 1, "step_up": -1}),
    ] {
        assert!(s.execute("cam.op.adaptive3d", &bad).is_err(), "{bad}");
    }
    use solidcraft_doc::manufacture::CamOperationKind;
    let ball = run(&mut s, "cam.op.adaptive3d", json!({"setup": "Rough", "tool": 2, "stepdown": 3, "stock_to_leave": 0.2, "step_up": 0}));
    assert!(ball["lowest_z"].as_f64().unwrap() <= -6.8 + 1e-9, "{ball}");
    let rough = |s: &Session| s.doc.manufacture.setups[0].operations.clone();
    assert!(matches!(&rough(&s)[1].kind, CamOperationKind::Adaptive { model: true, step_up: Some(e), .. } if e == "0"));
    run(&mut s, "cam.op.edit", json!({"setup": "Rough", "op": ball["operation"], "step_up": null}));
    assert!(matches!(&rough(&s)[1].kind, CamOperationKind::Adaptive { step_up: None, .. }));
    run(&mut s, "cam.op.remove", json!({"setup": "Rough", "op": ball["operation"]}));
    // After a facing pass took the 2 mm over the plate, roughing starts where it stopped.
    let minutes =
        |s: &mut Session, k: usize| run(s, "cam.list", json!({}))["setups"][0]["operations"][k]["toolpath"]["cut_minutes"].as_f64().unwrap();
    let alone = minutes(&mut s, 0);
    let faced = run(&mut s, "cam.op.face", json!({"setup": "Rough", "tool": 1}));
    run(&mut s, "cam.op.move", json!({"setup": "Rough", "op": faced["operation"], "to": 0}));
    let after = minutes(&mut s, 1);
    assert!(after < alone * 0.8, "{alone} min alone, {after} min after facing");
    run(&mut s, "cam.op.remove", json!({"setup": "Rough", "op": faced["operation"]}));
    // With room for the tool beside the part, it still goes no deeper than the deepest floor seen
    // from above by itself: down beside the part to the stock's bottom would cut the part loose.
    run(&mut s, "cam.setup.create", json!({"name": "Wide", "origin": top, "stock": {"top": 2, "side": 8}}));
    let made = run(&mut s, "cam.op.adaptive3d", json!({"setup": "Wide", "tool": 1, "stepdown": 3, "stock_to_leave": 0.2}));
    // Beside the part (no allowance to keep there) the stock goes down to that floor's height.
    close(made["lowest_z"].as_f64().unwrap(), -7.0, 1e-9);
    let warning = |s: &mut Session| run(s, "cam.list", json!({}))["setups"][1]["operations"][0]["warning"].clone();
    assert!(warning(&mut s).is_null(), "{}", warning(&mut s));
    // Told to go to the stock's bottom, it does, and says the part comes loose.
    run(&mut s, "cam.op.edit", json!({"setup": "Wide", "op": made["operation"], "bottom": -12}));
    assert!(warning(&mut s).as_str().is_some_and(|w| w.contains("without tabs")), "{}", warning(&mut s));
    // Two copies on one stock: roughing the stock goes round both, cutting into neither.
    run(&mut s, "cam.setup.create", json!({"name": "Two", "origin": top, "stock": {"top": 2}, "copies": {"nx": 2, "ny": 1, "dx": 70, "dy": 0}}));
    run(&mut s, "cam.op.adaptive3d", json!({"setup": "Two", "tool": 1, "stepdown": 3, "stock_to_leave": 0.2}));
    let sim = run(&mut s, "cam.simulate", json!({"setup": "Two", "columns": 300}));
    assert!(sim["max_gouge"].as_f64().unwrap() < 0.05, "{sim}");
}

#[test]
fn parallel_finishing_follows_the_model_after_roughing() {
    use solidcraft_doc::manufacture::{CamGeometry, CamOperationKind};
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 8], "p1": [30, 22]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Window", "distance": -4, "operation": "cut"}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "ball", "diameter": 4, "spindle_rpm": 15000, "feed": 800}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    run(&mut s, "cam.setup.create", json!({"name": "Both", "origin": top, "stock": {"top": 1}}));
    run(&mut s, "cam.op.adaptive3d", json!({"tool": 1, "stepdown": 3, "stock_to_leave": 0.3}));
    let made = run(&mut s, "cam.op.parallel3d", json!({"tool": 2}));
    // The pocket's floor, 5 below the zero, is as deep as it goes.
    close(made["lowest_z"].as_f64().unwrap(), -5.0, 1e-6);
    let op = s.doc.manufacture.setups[0].operations[1].clone();
    assert_eq!(op.geometry, CamGeometry::Part, "the model's box by itself");
    assert!(
        matches!(&op.kind, CamOperationKind::Parallel3d { stepover, angle, zigzag: true, .. } if stepover == "0.4 mm" && *angle == 0.0),
        "{:?}",
        op.kind
    );
    let listed = run(&mut s, "cam.list", json!({}));
    assert_eq!(listed["setups"][0]["operations"][1]["kind"], "parallel_3d");
    assert!(listed["setups"][0]["operations"][1].get("warning").is_none_or(Value::is_null), "{listed}");
    let sim = run(&mut s, "cam.simulate", json!({"setup": "Both", "columns": 300}));
    assert!(sim["max_gouge"].as_f64().unwrap() < 0.05, "{sim}");
    // Edited: across the other way, one way only.
    run(&mut s, "cam.op.edit", json!({"op": made["operation"], "angle": 90, "zigzag": false}));
    assert!(matches!(&s.doc.manufacture.setups[0].operations[1].kind, CamOperationKind::Parallel3d { angle, zigzag: false, .. } if *angle == 90.0));
    for bad in [json!({"tool": 2, "stepover": 0}), json!({"tool": 2, "angle": 1000})] {
        assert!(s.execute("cam.op.parallel3d", &bad).is_err(), "{bad}");
    }
}

#[test]
fn z_level_finishing_goes_round_the_walls() {
    use solidcraft_doc::manufacture::{CamGeometry, CamOperationKind};
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 8], "p1": [30, 22]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Window", "distance": -4, "operation": "cut"}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "ball", "diameter": 4, "spindle_rpm": 15000, "feed": 800}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    run(&mut s, "cam.setup.create", json!({"name": "Walls", "origin": top, "stock": {"top": 1}}));
    let made = run(&mut s, "cam.op.zlevel3d", json!({"tool": 2, "stock_to_leave": 0.1}));
    let op = s.doc.manufacture.setups[0].operations[0].clone();
    assert_eq!(op.geometry, CamGeometry::Part, "the model's box by itself");
    assert!(
        matches!(&op.kind, CamOperationKind::ZLevel3d { step, steep, climb: true, .. } if step == "0.333333 mm" && *steep == 30.0),
        "{:?}",
        op.kind
    );
    // Round the pocket's walls, down to its floor (5 below the zero).
    assert!(made["lowest_z"].as_f64().unwrap() <= -4.0, "{made}");
    let listed = run(&mut s, "cam.list", json!({}));
    let first = &listed["setups"][0]["operations"][0];
    assert_eq!(first["kind"], "zlevel_3d");
    assert!(first.get("warning").is_none_or(Value::is_null), "{first}");
    let sim = run(&mut s, "cam.simulate", json!({"setup": "Walls", "columns": 300}));
    assert!(sim["max_gouge"].as_f64().unwrap() < 0.05, "{sim}");
    // Edited: coarser levels, steeper walls only, the other way round.
    run(&mut s, "cam.op.edit", json!({"op": made["operation"], "step": 1, "steep": 60, "climb": false}));
    assert!(
        matches!(&s.doc.manufacture.setups[0].operations[0].kind, CamOperationKind::ZLevel3d { step, steep, climb: false, .. } if step == "1" && *steep == 60.0)
    );
    for bad in [json!({"tool": 2, "step": 0}), json!({"tool": 2, "steep": 90})] {
        assert!(s.execute("cam.op.zlevel3d", &bad).is_err(), "{bad}");
    }
}

#[test]
fn rest_finishing_goes_only_where_the_larger_ball_left_material() {
    let mut s = Session::default();
    run(&mut s, "solid.box", json!({"length": 40, "width": 30, "height": 10}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "offset": 10, "name": "Window"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 8], "p1": [30, 22]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.extrude", json!({"sketch": "Window", "distance": -4, "operation": "cut"}));
    run(&mut s, "cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": 6, "spindle_rpm": 12000, "feed": 600}));
    run(&mut s, "cam.tool.define", json!({"number": 2, "kind": "ball", "diameter": 4, "spindle_rpm": 15000, "feed": 800}));
    run(&mut s, "cam.tool.define", json!({"number": 3, "kind": "ball", "diameter": 1.5, "spindle_rpm": 20000, "feed": 500}));
    let top = json!({"of": "stock", "x": "min", "y": "min", "z": "max"});
    run(&mut s, "cam.setup.create", json!({"name": "Job", "origin": top, "stock": {"top": 1}}));
    // Nothing before it: nothing to go by.
    assert!(s.execute("cam.op.parallel3d", &json!({"tool": 3, "rest": true})).is_err());
    run(&mut s, "cam.op.adaptive3d", json!({"tool": 1, "stepdown": 3, "stock_to_leave": 0.3}));
    run(&mut s, "cam.op.parallel3d", json!({"tool": 2}));
    let left = |s: &mut Session| run(s, "cam.simulate", json!({"setup": "Job", "columns": 400}))["leftover_volume"].as_f64().unwrap();
    let before = left(&mut s);
    let rest = run(&mut s, "cam.op.parallel3d", json!({"tool": 3, "rest": true, "stepover": 0.2}));
    let all = run(&mut s, "cam.op.parallel3d", json!({"tool": 3, "stepover": 0.2}));
    // Round the pocket's walls only, where the 4 mm ball left its round: a fraction of it all.
    assert!(rest["cut_length"].as_f64().unwrap() < all["cut_length"].as_f64().unwrap() * 0.5, "{rest} {all}");
    run(&mut s, "cam.op.remove", json!({"setup": "Job", "op": all["operation"]}));
    let after = left(&mut s);
    assert!(after < before * 0.8, "{before} -> {after}");
    let sim = run(&mut s, "cam.simulate", json!({"setup": "Job", "columns": 400}));
    assert!(sim["max_gouge"].as_f64().unwrap() < 0.05, "{sim}");
    // Switched off again it finishes it all.
    run(&mut s, "cam.op.edit", json!({"op": rest["operation"], "rest": false}));
    assert!(matches!(&s.doc.manufacture.setups[0].operations[2].kind, solidcraft_doc::manufacture::CamOperationKind::Parallel3d { rest: false, .. }));
}
