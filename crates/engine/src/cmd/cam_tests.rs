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
        gcode.starts_with("(Untitled Setup1)\nG17 G21 G40 G49 G54 G80 G90 G94\n(Pocket1: T1 D6 flat end mill)\nM5\nT1 M6\nG43 H1\nS12000 M3\n"),
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
    // One level 1 mm down: 30/5 + 1 = 7 passes 5 apart, each 40 + 2 × (3 + 1) long, six 5 mm steps and the 3 mm plunge.
    let face = run(&mut s, "cam.op.face", with(json!({"top": 1, "bottom": 0, "stepover": 5, "stepdown": 1, "retract": 3})));
    close(face["cut_length"].as_f64().unwrap(), 7.0 * 48.0 + 6.0 * 5.0 + 3.0, 1e-9);
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
    // Its one round hole is drilled from the top through the plate: 2 → −10.
    let drill = run(&mut s, "cam.op.drill", json!({"faces": [{"point": [5, 5, 0]}], "tool": 2, "spindle_rpm": 3000, "feed": 150}));
    close(drill["cut_length"].as_f64().unwrap(), 12.0, 1e-9);
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

    for bad in
        [json!({"cell": 0}), json!({"cell": 1e-9}), json!({"stock": {"min": [0, 0, 0]}}), json!({"stock": {"min": [0, 0, 0], "max": [0, 0, 0]}})]
    {
        assert!(s.execute("cam.simulate", &bad).is_err(), "{bad}");
    }
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
    assert!(!Session::default().doc.to_json().contains("manufacture"));

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
    assert!(refused(&mut s, json!({"op": "Rough", "stepover": 3})).contains("stepover"));
    assert!(refused(&mut s, json!({"op": "Nope", "bottom": -1})).contains("no operation"));
    assert_eq!(s.doc.manufacture, before);
    run(&mut s, "cam.op.edit", json!({"op": "Rough", "top": 1, "retract": null}));
    assert_eq!(s.doc.manufacture.setups[0].operations[0].top.as_deref(), Some("1"));
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.doc.manufacture, before);
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
    assert!(refused(&mut s, "cam.op.pocket", with("stepover", json!(5))).contains("stepover"));
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
        "cell",
        "stock",
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
                p["diameter"] = json!(4);
                p[key] = value.clone();
                if let Err(e) = s.execute(cmd, &p) {
                    assert!(!matches!(e, crate::EngineError::Internal(..)), "{cmd} {key}={value}: {e}");
                }
            }
        }
    }
}
