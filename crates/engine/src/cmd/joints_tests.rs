use std::f64::consts::FRAC_PI_2;

use serde_json::{Value, json};
use solidcraft_geom::Vec3;

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn occ(s: &Session, component: &str) -> u64 {
    let c = s.doc.find_component(component).unwrap_or(0);
    s.doc.occurrence_of(c).map(|o| o.id).unwrap_or(0)
}

fn place(s: &Session, component: &str, p: Vec3) -> Vec3 {
    solidcraft_doc::apply_point(&solidcraft_doc::joints::occurrence_world(&s.doc, occ(s, component)), p)
}

fn near(a: Vec3, b: Vec3) -> bool {
    a.dist(b) < 1e-6
}

/// A hinge leaf (oracle 63's knuckle geometry): a 40 x 30 x 3 plate and an r4 knuckle along X
/// at y = 33, z = 4.
fn leaf(s: &mut Session, name: &str) {
    run(s, "component.activate", json!({"component": "root"}));
    run(s, "component.create", json!({"name": name}));
    run(s, "solid.box", json!({"length": 40, "width": 30, "height": 3, "corner": [0, 0, 0]}));
    run(s, "solid.cylinder", json!({"base": [0, 33, 4], "axis": [1, 0, 0], "radius": 4, "height": 40, "operation": "new"}));
}

#[test]
fn revolute_hinge_turns_about_the_knuckle() {
    let mut s = Session::default();
    leaf(&mut s, "LeafA");
    leaf(&mut s, "LeafB");
    let o = occ(&s, "LeafA");
    run(&mut s, "occurrence.ground", json!({"occurrence": o, "grounded": true}));
    let r = run(
        &mut s,
        "joint.create",
        json!({
        "type": "revolute", "a": {"occurrence": "LeafA", "circle": [1, 37, 4]}, "b": {"occurrence": "LeafB", "circle": [1, 37, 4]}, "flip": true, "name": "Hinge"}),
    );
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    // Same frames: B stays put at angle 0.
    assert!(near(place(&s, "LeafB", Vec3::new(20.0, 0.0, 0.0)), Vec3::new(20.0, 0.0, 0.0)));
    let l = run(&mut s, "joint.list", json!({}));
    let dof = l["dof"].as_array().unwrap().iter().find(|d| d["occurrence"] == occ(&s, "LeafB")).unwrap()["dof"].clone();
    assert_eq!(dof, 1, "{l}");
    // Drive 90°: B turns about the knuckle axis (through (0, 33, 4), along the frame's z = −X).
    run(&mut s, "joint.drive", json!({"joint": "Hinge", "value": 90}));
    let want = solidcraft_doc::rigid(Vec3::ZERO, Vec3::new(0.0, 33.0, 4.0), Vec3::new(-1.0, 0.0, 0.0), FRAC_PI_2);
    let p = Vec3::new(20.0, 0.0, 0.0);
    assert!(near(place(&s, "LeafB", p), solidcraft_doc::apply_point(&want, p)), "{:?}", place(&s, "LeafB", p));
    assert!(near(place(&s, "LeafB", p), Vec3::new(20.0, 29.0, 37.0)));
    // LeafA did not move; the knuckle axis is fixed.
    assert!(near(place(&s, "LeafA", p), p));
    assert!(near(place(&s, "LeafB", Vec3::new(10.0, 33.0, 4.0)), Vec3::new(10.0, 33.0, 4.0)));
    // Expressions drive too, and undo puts it back.
    run(&mut s, "joint.drive", json!({"joint": "Hinge", "value": "pi rad"}));
    assert!(near(place(&s, "LeafB", p), Vec3::new(20.0, 66.0, 8.0)));
    run(&mut s, "edit.undo", json!({}));
    assert!(near(place(&s, "LeafB", p), Vec3::new(20.0, 29.0, 37.0)));
}

/// Two 10 mm cubes in components A (grounded) and B.
fn two_cubes() -> Session {
    let mut s = Session::default();
    for (name, x) in [("A", 0), ("B", 20)] {
        run(&mut s, "component.activate", json!({"component": "root"}));
        run(&mut s, "component.create", json!({"name": name}));
        run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 10, "corner": [x, 0, 0]}));
    }
    let o = occ(&s, "A");
    run(&mut s, "occurrence.ground", json!({"occurrence": o, "grounded": true}));
    s
}

#[test]
fn slider_limits_clamp() {
    let mut s = two_cubes();
    // B's bottom face onto A's top face (faces meet), sliding along their common normal.
    let r = run(
        &mut s,
        "joint.create",
        json!({"type": "slider", "a": {"face": [5, 5, 10]}, "b": {"face": [25, 5, 0]}, "limits": [[0, 10]], "name": "Rail"}),
    );
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    // B now sits on A.
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 10.0)), "{:?}", place(&s, "B", Vec3::new(25.0, 5.0, 0.0)));
    let r = run(&mut s, "joint.drive", json!({"joint": "Rail", "value": 25}));
    assert_eq!(r["values"], json!([10.0]), "{r}");
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 20.0)));
    run(&mut s, "joint.drive", json!({"joint": "Rail", "value": -5}));
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 10.0)));
    // Changing the limits clamps the current value.
    run(&mut s, "joint.drive", json!({"joint": "Rail", "value": 8}));
    run(&mut s, "joint.limits", json!({"joint": "Rail", "min": 0, "max": 4}));
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 14.0)));
    // Joints follow edits: A gets taller, B rides on its new top.
    let h = s
        .doc
        .features
        .iter()
        .find(|f| matches!(f.kind, solidcraft_doc::FeatureKind::Box { .. }))
        .and_then(|f| f.param_names.get(2).cloned())
        .unwrap_or_default();
    run(&mut s, "parameters.change", json!({"name": h, "expression": "15"}));
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 19.0)), "{:?}", place(&s, "B", Vec3::new(25.0, 5.0, 0.0)));
}

#[test]
fn rigid_cycles_solve_and_conflicts_report() {
    let mut s = two_cubes();
    run(&mut s, "component.activate", json!({"component": "root"}));
    run(&mut s, "component.create", json!({"name": "C"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 10, "corner": [40, 0, 0]}));
    run(&mut s, "component.activate", json!({"component": "root"}));
    // A rigid group and a closing as-built joint: a consistent loop.
    run(&mut s, "joint.rigid_group", json!({"occurrences": ["A", "B", "C"]}));
    let r = run(&mut s, "joint.as_built", json!({"type": "rigid", "a": "B", "b": "C"}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    assert!(near(place(&s, "C", Vec3::new(40.0, 0.0, 0.0)), Vec3::new(40.0, 0.0, 0.0)));
    let l = run(&mut s, "joint.list", json!({}));
    assert!(l["dof"].as_array().unwrap().iter().all(|d| d["dof"] == 0), "{l}");
    // A joint that wants C elsewhere conflicts and says so.
    let r = run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": {"occurrence": "A", "point": [0, 0, 0]}, "b": {"occurrence": "C", "point": [40, 0, 0]}, "name": "Wrong"}),
    );
    let c = r["conflicts"].as_array().cloned().unwrap_or_default();
    assert_eq!(c.len(), 1, "{r}");
    assert!(c[0]["why"].as_str().unwrap_or("").contains("off by"), "{r}");
    // Suppressing it clears the conflict.
    let r = run(&mut s, "joint.edit", json!({"joint": "Wrong", "suppressed": true}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0));
    assert!(
        s.execute(
            "joint.create",
            &json!({"type": "rigid", "a": {"occurrence": "A", "point": [0, 0, 0]}, "b": {"occurrence": "A", "point": [1, 0, 0]}})
        )
        .is_err()
    );
    assert!(s.execute("joint.create", &json!({"type": "hinge"})).is_err());
}

/// A 30 x 30 x 3 plate with a through slot whose x = 20 wall carries one rib (0.3 tall, 0.8
/// wide), and a peg in the slot with 0.15 clearance all round.
fn ribbed_slot_and_peg(s: &mut Session, peg_x1: f64) {
    run(s, "sketch.create", json!({"plane": "XY"}));
    run(s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [30, 30]}));
    run(s, "sketch.finish", json!({}));
    run(s, "solid.extrude", json!({"distance": 3, "body_name": "Plate"}));
    run(s, "sketch.create", json!({"plane": "XY", "offset": -1}));
    let slot = json!([[10, 5], [20, 5], [20, 14.6], [19.7, 14.6], [19.7, 15.4], [20, 15.4], [20, 25], [10, 25]]);
    run(s, "sketch.line", json!({"points": slot, "closed": true}));
    run(s, "sketch.finish", json!({}));
    run(s, "solid.extrude", json!({"distance": 5, "operation": "cut", "targets": ["Plate"]}));
    run(s, "sketch.create", json!({"plane": "XY"}));
    run(s, "sketch.rectangle.two_point", json!({"p0": [10.15, 5.15], "p1": [peg_x1, 24.85]}));
    run(s, "sketch.finish", json!({}));
    run(s, "solid.extrude", json!({"distance": 3, "operation": "new", "body_name": "Peg"}));
}

/// The peg overlaps only the rib: 0.15 x 0.8 x 3 = 0.36 mm³ (#442).
#[test]
fn interference_finds_a_peg_overlapping_a_small_rib() {
    let mut s = Session::default();
    ribbed_slot_and_peg(&mut s, 19.85);
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["count"], 1, "{i}");
    assert!((i["interferences"][0]["volume_mm3"].as_f64().unwrap_or(0.0) - 0.36).abs() < 1e-6, "{i}");
    let (lo, hi) = (&i["interferences"][0]["bbox"]["min"], &i["interferences"][0]["bbox"]["max"]);
    assert!((lo[0].as_f64().unwrap_or(0.0) - 19.7).abs() < 1e-6 && (hi[0].as_f64().unwrap_or(0.0) - 19.85).abs() < 1e-6, "{i}");
}

/// An arc revolved up to a post ends on the post's wall, where the two bodies can't be
/// intersected: the pair is listed as unchecked instead of passing as clear.
#[test]
fn interference_lists_a_pair_it_could_not_check() {
    let mut s = Session::default();
    run(&mut s, "solid.cylinder", json!({"base": [0, 14, -5], "radius": 4, "height": 15, "body_name": "Post"}));
    run(&mut s, "sketch.create", json!({"name": "Profile", "plane": "XZ"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [10, 0], "p1": [15, 5]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "solid.revolve", json!({"sketch": "Profile", "axis": {"origin": [0, 0, 0], "dir": [0, 0, 1]}, "to": [4, 14, 2], "body_name": "Arc"}));
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["count"], 0, "{i}");
    let unchecked = i["unchecked"].as_array().cloned().unwrap_or_default();
    assert_eq!(unchecked.len(), 1, "{i}");
    assert!(unchecked[0]["reason"].as_str().is_some_and(|r| r.starts_with("not verified") && !r.contains("displacement")), "{i}");
    // Bodies that are clear of each other, or overlap plainly, leave the list empty.
    let mut s = two_cubes();
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["unchecked"].as_array().map(Vec::len), Some(0), "{i}");
}

#[test]
fn motion_links_and_interference() {
    let mut s = two_cubes();
    run(&mut s, "joint.create", json!({"type": "cylindrical", "a": {"face": [5, 5, 10]}, "b": {"face": [25, 5, 0]}, "name": "Screw"}));
    run(&mut s, "joint.drive", json!({"joint": "Screw", "values": [0, 0]}));
    // No overlap while B sits on A; then push B 4 mm into A.
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["count"], 0, "{i}");
    run(&mut s, "joint.drive", json!({"joint": "Screw", "values": [0, -4]}));
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["count"], 1, "{i}");
    assert!((i["interferences"][0]["volume_mm3"].as_f64().unwrap_or(0.0) - 400.0).abs() < 1e-3, "{i}");
    // A motion link: a second joint's value follows.
    run(&mut s, "component.activate", json!({"component": "root"}));
    run(&mut s, "component.create", json!({"name": "D"}));
    run(&mut s, "solid.box", json!({"length": 10, "width": 10, "height": 10, "corner": [60, 0, 0]}));
    run(
        &mut s,
        "joint.create",
        json!({"type": "slider", "a": {"occurrence": "A", "face": [5, 5, 10]}, "b": {"occurrence": "D", "face": [65, 5, 0]}, "name": "Follower"}),
    );
    run(&mut s, "joint.motion_link", json!({"a": "Screw", "ia": 1, "b": "Follower", "ratio": 2}));
    run(&mut s, "joint.drive", json!({"joint": "Screw", "values": [0, 3]}));
    assert!(near(place(&s, "D", Vec3::new(65.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 16.0)), "{:?}", place(&s, "D", Vec3::new(65.0, 5.0, 0.0)));
    // Saved and reopened, joints come back.
    let back = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    assert_eq!(back.assembly, s.doc.assembly);
    let mut t = Session::new(back);
    let l = run(&mut t, "joint.list", json!({}));
    assert_eq!(l["joints"].as_array().map(Vec::len), Some(2));
}

/// Two faces joined mate: B's bottom face lands on A's top face, B on top of A (not inside
/// it); Flip turns B over instead.
#[test]
fn faces_mate_by_default() {
    let mut s = two_cubes();
    run(&mut s, "joint.create", json!({"type": "rigid", "a": {"face": [5, 5, 10]}, "b": {"face": [25, 5, 0]}, "name": "Stack"}));
    // B's bottom centre (25, 5, 0) at A's top centre, B above it.
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 0.0)), Vec3::new(5.0, 5.0, 10.0)));
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 10.0)), Vec3::new(5.0, 5.0, 20.0)), "{:?}", place(&s, "B", Vec3::new(25.0, 5.0, 10.0)));
    run(&mut s, "joint.edit", json!({"joint": "Stack", "flip": true}));
    assert!(near(place(&s, "B", Vec3::new(25.0, 5.0, 10.0)), Vec3::new(5.0, 5.0, 0.0)), "{:?}", place(&s, "B", Vec3::new(25.0, 5.0, 10.0)));
}

/// #527: an assembly with its fasteners passes 200 bodies quickly; the clash check must still
/// cover the whole design. 201 separate 1 mm cubes 2 mm apart, plus one cube pushed half-way
/// into the first: exactly that pair overlaps, by 0.5 mm³.
#[test]
fn interference_checks_designs_with_more_than_200_bodies() {
    let mut s = Session::default();
    for i in 0..201 {
        run(&mut s, "solid.box", json!({"length": 1, "width": 1, "height": 1, "corner": [2 * i, 0, 0], "operation": "new"}));
    }
    run(&mut s, "solid.box", json!({"length": 1, "width": 1, "height": 1, "corner": [0.5, 0, 0], "operation": "new", "body_name": "Clash"}));
    assert_eq!(s.model.state().bodies.len(), 202);
    let i = run(&mut s, "inspect.interference", json!({}));
    assert_eq!(i["checked"], 202, "{i}");
    assert_eq!(i["count"], 1, "{i}");
    let hit = &i["interferences"][0];
    let mut pair = [hit["a"].as_str().unwrap_or(""), hit["b"].as_str().unwrap_or("")];
    pair.sort_unstable();
    assert_eq!(pair, ["Body1", "Clash"], "{i}");
    assert!((hit["volume_mm3"].as_f64().unwrap_or(0.0) - 0.5).abs() < 1e-6, "{i}");
}

/// The bounding-box sweep behind `inspect.interference` finds exactly the pairs a check of every
/// pair finds (touching boxes count), and reports how many it left out past its limit.
#[test]
fn interference_box_sweep_matches_every_pair_check() {
    use solidcraft_geom::Aabb3;
    let cube = |x: f64, y: f64, z: f64, e: f64| Aabb3 { min: Vec3::new(x, y, z), max: Vec3::new(x + e, y + e, z + e) };
    // A deterministic scatter: overlapping, touching, and apart in only one axis.
    let boxes: Vec<Aabb3> = (0..60)
        .map(f64::from)
        .map(|i| cube((i * 7.3) % 20.0, (i * 3.1) % 9.0, (i * 5.7) % 4.0, 1.0 + (i % 3.0)))
        .chain([cube(30.0, 0.0, 0.0, 1.0), cube(31.0, 0.0, 0.0, 1.0)])
        .collect();
    let mut every = Vec::new();
    for (i, a) in boxes.iter().enumerate() {
        for (j, b) in boxes.iter().enumerate().skip(i + 1) {
            let apart = a.max.x < b.min.x || b.max.x < a.min.x || a.max.y < b.min.y || b.max.y < a.min.y || a.max.z < b.min.z || b.max.z < a.min.z;
            if !apart {
                every.push((i, j));
            }
        }
    }
    assert!(every.contains(&(60, 61)), "touching boxes are a candidate pair");
    let (pairs, skipped) = super::overlapping_pairs(&boxes, usize::MAX);
    assert_eq!((pairs, skipped), (every.clone(), 0));
    let (pairs, skipped) = super::overlapping_pairs(&boxes, 10);
    assert_eq!((pairs.len(), skipped), (10, every.len() - 10));
}

/// A small block in a new component: a part of a linkage, where only its joints matter.
fn part(s: &mut Session, name: &str, corner: [f64; 3]) {
    run(s, "component.activate", json!({"component": "root"}));
    run(s, "component.create", json!({"name": name}));
    run(s, "solid.box", json!({"length": 2, "width": 2, "height": 2, "corner": corner}));
}

/// A joint between two parts as they are, at a world point with its z along `z`.
fn pin(s: &mut Session, kind: &str, name: &str, a: &str, b: &str, p: Vec3, z: Vec3) {
    let at = |o: &str| json!({"occurrence": o, "point": [p.x, p.y, p.z], "z": [z.x, z.y, z.z]});
    let r = run(s, "joint.create", json!({"type": kind, "a": at(a), "b": at(b), "flip": true, "name": name}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
}

/// Where circles about `p` (radius `r`) and `q` (radius `s`) in the XY plane meet.
fn meet(p: Vec3, r: f64, q: Vec3, s: f64) -> [Vec3; 2] {
    let d = q - p;
    let l = d.len();
    let a = (r * r - s * s + l * l) / (2.0 * l);
    let m = p + d * (a / l);
    let n = Vec3::new(-d.y, d.x, 0.0) * ((r * r - a * a).max(0.0).sqrt() / l);
    [m + n, m - n]
}

/// A crank-rocker four-bar about +Z: ground pivots O2 = (0, 0) and O4 = (45, 0), crank 20,
/// coupler 50, rocker 40, assembled with the crank at 90°. Returns the crank pin P0 and the
/// rocker pin Q0 (the upper, open assembly).
fn four_bar() -> (Session, Vec3, Vec3) {
    let mut s = Session::default();
    let (o2, o4, p0) = (Vec3::ZERO, Vec3::new(45.0, 0.0, 0.0), Vec3::new(0.0, 20.0, 0.0));
    let [q0, _] = meet(p0, 50.0, o4, 40.0);
    for (name, c) in [("Ground", o2), ("Crank", p0), ("Coupler", (p0 + q0) * 0.5), ("Rocker", q0)] {
        part(&mut s, name, [c.x, c.y, -5.0]);
    }
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    pin(&mut s, "revolute", "Crank", "Ground", "Crank", o2, Vec3::Z);
    pin(&mut s, "revolute", "Crank pin", "Crank", "Coupler", p0, Vec3::Z);
    pin(&mut s, "revolute", "Rocker pin", "Coupler", "Rocker", q0, Vec3::Z);
    pin(&mut s, "revolute", "Rocker", "Ground", "Rocker", o4, Vec3::Z);
    (s, p0, q0)
}

/// The four-bar's rocker pin with the crank driven to `to`°, followed a degree at a time from
/// `from`° (where it is at `q`), so it stays on its branch.
fn rocker_pin(from: f64, to: f64, mut q: Vec3) -> Vec3 {
    let n = ((to - from).abs().ceil() as usize).max(1);
    for k in 1..=n {
        let th = (90.0 + from + (to - from) * k as f64 / n as f64).to_radians();
        let [m1, m2] = meet(Vec3::new(20.0 * th.cos(), 20.0 * th.sin(), 0.0), 50.0, Vec3::new(45.0, 0.0, 0.0), 40.0);
        q = if m1.dist(q) <= m2.dist(q) { m1 } else { m2 };
    }
    q
}

#[test]
fn four_bar_follows_its_crank() {
    let (mut s, p0, q0) = four_bar();
    let mut q = q0;
    let mut at_330 = q0;
    for k in 1..=12 {
        let v = 30.0 * f64::from(k);
        let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": v}));
        assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{v}°: {r}");
        assert!(r["stopped_by_loop"].is_null(), "{v}°: {r}");
        // The driven joint keeps exactly the value asked for.
        assert!((r["values"][0].as_f64().unwrap_or(f64::NAN) - v.to_radians()).abs() < 1e-12, "{r}");
        q = rocker_pin(v - 30.0, v, q);
        let th = (90.0 + v).to_radians();
        let p = Vec3::new(20.0 * th.cos(), 20.0 * th.sin(), 0.0);
        for (part, local, want) in [("Crank", p0, p), ("Coupler", p0, p), ("Coupler", q0, q), ("Rocker", q0, q)] {
            assert!(near(place(&s, part, local), want), "{v}°: {part} {:?} wants {want:?}", place(&s, part, local));
        }
        if k == 11 {
            at_330 = q;
        }
    }
    // Back where it started after a full turn, and undo steps back.
    assert!(near(q, q0));
    run(&mut s, "edit.undo", json!({}));
    assert!(near(place(&s, "Rocker", q0), at_330), "{:?}", place(&s, "Rocker", q0));
    // One 180° drive lands on the same (open) branch as the stepped path.
    let (mut s, _, q0) = four_bar();
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 180}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    assert!(near(place(&s, "Rocker", q0), rocker_pin(0.0, 180.0, q0)), "{:?}", place(&s, "Rocker", q0));
    // A motion study playing the crank places the rocker the same way.
    run(&mut s, "motion.study", json!({"name": "Turn", "steps": 10, "keys": [{"joint": "Crank", "points": [[0, 0], [10, 120]]}]}));
    let r = run(&mut s, "motion.play", json!({"study": "Turn", "step": 5}));
    assert!(r["stopped_by_loop"].is_null(), "{r}");
    assert!(near(place(&s, "Rocker", q0), rocker_pin(180.0, 60.0, rocker_pin(0.0, 180.0, q0))), "{:?}", place(&s, "Rocker", q0));
}

/// The rocker swings between its toggle positions, where crank and coupler line up (|Q| = 70
/// and |Q| = 30, about 69.3° and 139.2° from the ground line); driving it further stops where
/// the loop still closes.
#[test]
fn four_bar_rocker_stops_where_the_loop_cannot_close() {
    let (mut s, _, q0) = four_bar();
    let start = q0.y.atan2(q0.x - 45.0).to_degrees();
    let r = run(&mut s, "joint.drive", json!({"joint": "Rocker", "value": 100}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    let reached = r["values"][0].as_f64().unwrap_or(f64::NAN).to_degrees();
    // |Q|² = 45² + 40² + 2·45·40·cos φ = 30².
    let limit = ((900.0f64 - 3625.0) / 3600.0).acos().to_degrees() - start;
    assert!(reached > limit - 5.0 && reached <= limit, "stopped at {reached}°, the limit is {limit}°");
    // Inside its swing it moves freely, and the crank follows.
    let r = run(&mut s, "joint.drive", json!({"joint": "Rocker", "value": -10}));
    assert!(r["stopped_by_loop"].is_null() && r["conflicts"].as_array().map(Vec::len) == Some(0), "{r}");
    let q = place(&s, "Rocker", q0);
    assert!((q.y.atan2(q.x - 45.0).to_degrees() - (start - 10.0)).abs() < 1e-6, "{q:?}");
    assert!(place(&s, "Coupler", q0).dist(q) < 1e-6);
    // A full turn would end where it started, but the rocker can't get there.
    let (mut s, _, _) = four_bar();
    let r = run(&mut s, "joint.drive", json!({"joint": "Rocker", "value": 360}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert!(r["values"][0].as_f64().unwrap_or(f64::NAN).to_degrees() <= limit, "{r}");
}

/// A loop value a motion link sets follows through the start of its link chain: a dial outside
/// the loop turns a gear, which sets the rocker's angle, so turning the crank turns the dial.
#[test]
fn four_bar_turns_the_source_of_a_linked_loop_value() {
    let linked = || {
        let (mut s, _, q0) = four_bar();
        for (name, x) in [("Dial", 80.0), ("Gear", 100.0)] {
            part(&mut s, name, [x, 0.0, -5.0]);
            pin(&mut s, "revolute", name, "Ground", name, Vec3::new(x, 0.0, 0.0), Vec3::Z);
        }
        run(&mut s, "joint.motion_link", json!({"a": "Dial", "b": "Gear", "ratio": 1}));
        run(&mut s, "joint.motion_link", json!({"a": "Gear", "b": "Rocker", "ratio": 0.5}));
        (s, q0)
    };
    let (mut s, q0) = linked();
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 60}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    let q = rocker_pin(0.0, 60.0, q0);
    assert!(near(place(&s, "Rocker", q0), q), "{:?} wants {q:?}", place(&s, "Rocker", q0));
    let turn = q.y.atan2(q.x - 45.0) - q0.y.atan2(q0.x - 45.0);
    let dial = s.doc.assembly.joints.iter().find(|j| j.name == "Dial").and_then(|j| j.values.first().copied()).unwrap_or(f64::NAN);
    assert!((dial - 2.0 * turn).abs() < 1e-9, "dial {dial}, rocker turned {turn}");
    // Driving the dial drives the rocker through the chain: two turns of the dial would turn
    // the rocker a full turn, which it can't make.
    let (mut s, _) = linked();
    let r = run(&mut s, "joint.drive", json!({"joint": "Dial", "value": 720}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
}

/// Values left out of a drive are free to follow in a loop: a plate on a planar joint whose
/// travel a diagonal slider limits. Driving only the planar joint's x moves its y along.
#[test]
fn values_left_out_of_a_drive_follow_the_loop() {
    let mut s = Session::default();
    part(&mut s, "Ground", [-1.0, -1.0, -5.0]);
    part(&mut s, "Plate", [-1.0, -1.0, 0.0]);
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    let frame = |o: &str, z: [f64; 3], x: [f64; 3]| json!({"occurrence": o, "frame": {"origin": [0, 0, 0], "z": z, "x": x}});
    let flat = |o: &str| frame(o, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let r = run(&mut s, "joint.create", json!({"type": "planar", "a": flat("Ground"), "b": flat("Plate"), "flip": true, "name": "Table"}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    let diagonal = |o: &str| frame(o, [1.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
    let r = run(&mut s, "joint.create", json!({"type": "slider", "a": diagonal("Ground"), "b": diagonal("Plate"), "flip": true, "name": "Rail"}));
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    let r = run(&mut s, "joint.drive", json!({"joint": "Table", "values": [10]}));
    assert!(r["stopped_by_loop"].is_null() && r["conflicts"].as_array().map(Vec::len) == Some(0), "{r}");
    assert!(near(place(&s, "Plate", Vec3::ZERO), Vec3::new(10.0, 10.0, 0.0)), "{:?}", place(&s, "Plate", Vec3::ZERO));
}

/// A slider-crank: crank 15 about the origin, rod 40, a piston sliding along X. The piston
/// sits at a·cos θ + √(b² − (a·sin θ)²).
#[test]
fn slider_crank_moves_the_piston() {
    let mut s = Session::default();
    let (o, p0) = (Vec3::ZERO, Vec3::new(0.0, 15.0, 0.0));
    let s0 = Vec3::new((40.0f64 * 40.0 - 15.0 * 15.0).sqrt(), 0.0, 0.0);
    for (name, c) in [("Ground", o), ("Crank", p0), ("Rod", s0 * 0.5), ("Piston", s0)] {
        part(&mut s, name, [c.x, c.y, -5.0]);
    }
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    pin(&mut s, "revolute", "Crank", "Ground", "Crank", o, Vec3::Z);
    pin(&mut s, "revolute", "Crank pin", "Crank", "Rod", p0, Vec3::Z);
    pin(&mut s, "revolute", "Wrist pin", "Rod", "Piston", s0, Vec3::Z);
    pin(&mut s, "slider", "Bore", "Ground", "Piston", s0, Vec3::X);
    for v in [40.0, 135.0, 270.0] {
        let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": v}));
        assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{v}°: {r}");
        let th = (90.0f64 + v).to_radians();
        let x = 15.0 * th.cos() + (40.0f64.powi(2) - (15.0 * th.sin()).powi(2)).sqrt();
        assert!(near(place(&s, "Piston", s0), Vec3::new(x, 0.0, 0.0)), "{v}°: {:?} wants x = {x}", place(&s, "Piston", s0));
        assert!(near(place(&s, "Rod", p0), Vec3::new(15.0 * th.cos(), 15.0 * th.sin(), 0.0)));
    }
}

/// A joint frame on an occurrence, by origin and axes.
fn framed(o: &str, origin: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Value {
    json!({"occurrence": o, "frame": {"origin": origin, "z": z, "x": x}})
}

/// A loop open from the start closes from where it is: an arm on a revolute joint, and a rigid
/// joint that wants it turned 90°.
#[test]
fn an_open_loop_closes_from_where_it_is() {
    let mut s = Session::default();
    part(&mut s, "Ground", [-1.0, -1.0, -5.0]);
    part(&mut s, "Arm", [0.0, -1.0, 0.0]);
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    pin(&mut s, "revolute", "Pivot", "Ground", "Arm", Vec3::ZERO, Vec3::Z);
    let r = run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": framed("Ground", [0.0; 3], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
            "b": framed("Arm", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), "flip": true, "name": "Lock"}),
    );
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
    assert!(near(place(&s, "Arm", Vec3::new(10.0, 0.0, 0.0)), Vec3::new(0.0, 10.0, 0.0)), "{:?}", place(&s, "Arm", Vec3::new(10.0, 0.0, 0.0)));
}

/// A loop that can't close does not hold up another one: the four-bar still drives beside a
/// block whose rigid joint wants it where its pivot can't take it.
#[test]
fn a_broken_loop_does_not_stop_another() {
    let (mut s, _, q0) = four_bar();
    part(&mut s, "Block", [200.0, -1.0, -5.0]);
    pin(&mut s, "revolute", "Block pivot", "Ground", "Block", Vec3::new(200.0, 0.0, 0.0), Vec3::Z);
    run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": {"occurrence": "Ground", "point": [200, 0, 0]}, "b": {"occurrence": "Block", "point": [210, 0, 0]}, "flip": true, "name": "Wrong"}),
    );
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 60}));
    let names: Vec<&str> = r["conflicts"].as_array().into_iter().flatten().filter_map(|c| c["name"].as_str()).collect();
    assert_eq!(names, ["Wrong"], "{r}");
    assert!(r["stopped_by_loop"].is_null(), "{r}");
    assert!(near(place(&s, "Rocker", q0), rocker_pin(0.0, 60.0, q0)), "{:?}", place(&s, "Rocker", q0));
}

/// A loop locked rigid can't move at all: driving its crank stops at once.
#[test]
fn a_locked_loop_does_not_move() {
    let (mut s, _, _) = four_bar();
    run(&mut s, "joint.as_built", json!({"type": "rigid", "a": "Ground", "b": "Rocker", "name": "Lock"}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 360}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert!(r["values"][0].as_f64().unwrap_or(f64::NAN).abs() < 1e-3, "{r}");
}

/// A link that turns the rocker 100 times as far as its dial: a small turn of the dial is a full
/// turn of the rocker, which it can't make.
#[test]
fn a_geared_down_loop_value_is_followed_all_the_way() {
    let (mut s, _, _) = four_bar();
    part(&mut s, "Dial", [80.0, 0.0, -5.0]);
    pin(&mut s, "revolute", "Dial", "Ground", "Dial", Vec3::new(80.0, 0.0, 0.0), Vec3::Z);
    run(&mut s, "joint.motion_link", json!({"a": "Dial", "b": "Rocker", "ratio": 100}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Dial", "value": 3.6}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
}

/// A suppressed dial still turns the rocker through its motion link, and a rocker whose value
/// the file leaves out starts from 0: both are followed all the way, so a full turn stops.
#[test]
fn a_full_turn_stops_from_a_suppressed_dial_or_a_missing_value() {
    let (mut s, _, _) = four_bar();
    part(&mut s, "Dial", [80.0, 0.0, -5.0]);
    pin(&mut s, "revolute", "Dial", "Ground", "Dial", Vec3::new(80.0, 0.0, 0.0), Vec3::Z);
    run(&mut s, "joint.motion_link", json!({"a": "Dial", "b": "Rocker", "ratio": 0.5}));
    run(&mut s, "joint.edit", json!({"joint": "Dial", "suppressed": true}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Dial", "value": 720}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    // Undriven, the suppressed dial still follows the rocker when the crank turns.
    run(&mut s, "joint.drive", json!({"joint": "Dial", "value": 0}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 60}));
    assert!(r["stopped_by_loop"].is_null() && r["conflicts"].as_array().map(Vec::len) == Some(0), "{r}");
    let (mut s, _, _) = four_bar();
    if let Some(j) = s.doc_mut().assembly.joints.iter_mut().find(|j| j.name == "Rocker") {
        j.values.clear();
    }
    let r = run(&mut s, "joint.drive", json!({"joint": "Rocker", "value": 360}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
}

/// Loops that only share a rigid joint are independent: an arm whose lock can't close does not
/// stop another arm, beside it on the same rigidly mounted base, from closing its own loop.
#[test]
fn loops_sharing_only_a_rigid_joint_are_solved_apart() {
    let mut s = Session::default();
    for (name, x) in [("Ground", -1.0), ("Base", -1.0), ("Arm", 0.0), ("Other", 100.0)] {
        part(&mut s, name, [x, -1.0, 0.0]);
    }
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    run(&mut s, "joint.as_built", json!({"type": "rigid", "a": "Ground", "b": "Base", "name": "Mount"}));
    pin(&mut s, "revolute", "Pivot", "Base", "Arm", Vec3::ZERO, Vec3::Z);
    pin(&mut s, "revolute", "Other pivot", "Base", "Other", Vec3::new(100.0, 0.0, 0.0), Vec3::Z);
    run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": {"occurrence": "Ground", "point": [100, 0, 0]}, "b": {"occurrence": "Other", "point": [110, 0, 0]}, "flip": true, "name": "Wrong"}),
    );
    let r = run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": framed("Ground", [0.0; 3], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
            "b": framed("Arm", [0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]), "flip": true, "name": "Lock"}),
    );
    // The broken loop is reported (on whichever of its joints closes it), the arm's is not.
    let names: Vec<&str> = r["conflicts"].as_array().into_iter().flatten().filter_map(|c| c["name"].as_str()).collect();
    assert!(names.len() == 1 && ["Wrong", "Other pivot"].contains(&names[0]), "{r}");
    assert!(near(place(&s, "Arm", Vec3::new(10.0, 0.0, 0.0)), Vec3::new(0.0, 10.0, 0.0)), "{:?}", place(&s, "Arm", Vec3::new(10.0, 0.0, 0.0)));
}

/// Two links writing one value: whichever carries a driven dial's turn into the loop, the turn
/// is followed (here dial A turns gear B, which turns the rocker, before dial D rewrites B).
#[test]
fn a_turn_carried_in_by_any_link_is_followed() {
    let (mut s, _, _) = four_bar();
    for (name, x) in [("A", 80.0), ("B", 100.0), ("D", 120.0)] {
        part(&mut s, name, [x, 0.0, -5.0]);
        pin(&mut s, "revolute", name, "Ground", name, Vec3::new(x, 0.0, 0.0), Vec3::Z);
    }
    run(&mut s, "joint.motion_link", json!({"a": "A", "b": "B", "ratio": 1}));
    run(&mut s, "joint.motion_link", json!({"a": "B", "b": "Rocker", "ratio": 0.5}));
    run(&mut s, "joint.motion_link", json!({"a": "D", "b": "B", "ratio": 1}));
    let r = run(&mut s, "joint.drive", json!({"joint": "A", "value": 720}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
}

/// Several links writing one value: every value a link chain starts from can move to close the
/// loop, whichever of them the link order lets through (here dial A's, before dial B rewrites
/// the gear).
#[test]
fn a_loop_value_moves_through_whichever_link_carries_it() {
    let (mut s, _, q0) = four_bar();
    for (name, x) in [("A", 80.0), ("G", 100.0), ("B", 120.0)] {
        part(&mut s, name, [x, 0.0, -5.0]);
        pin(&mut s, "revolute", name, "Ground", name, Vec3::new(x, 0.0, 0.0), Vec3::Z);
    }
    run(&mut s, "joint.motion_link", json!({"a": "A", "b": "G", "ratio": 1}));
    run(&mut s, "joint.motion_link", json!({"a": "G", "b": "Rocker", "ratio": 1}));
    run(&mut s, "joint.motion_link", json!({"a": "B", "b": "G", "ratio": 1}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Crank", "value": 60}));
    assert!(r["stopped_by_loop"].is_null() && r["conflicts"].as_array().map(Vec::len) == Some(0), "{r}");
    assert!(near(place(&s, "Rocker", q0), rocker_pin(0.0, 60.0, q0)), "{:?}", place(&s, "Rocker", q0));
}

/// Two values of one joint can feed two independent loops: a puck's x drives a pair of parallel
/// slides, its y a slide whose rigid lock can't close. Driving x moves the healthy pair.
#[test]
fn values_of_one_joint_feed_independent_loops() {
    let mut s = Session::default();
    for (name, y) in [("Ground", -10.0), ("Puck", -20.0), ("Car", 50.0), ("Other", 100.0)] {
        part(&mut s, name, [-1.0, y, 0.0]);
    }
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    let flat = |o: &str| framed(o, [0.0, -20.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    run(&mut s, "joint.create", json!({"type": "planar", "a": flat("Ground"), "b": flat("Puck"), "flip": true, "name": "Table"}));
    pin(&mut s, "slider", "Rail", "Ground", "Car", Vec3::new(0.0, 50.0, 0.0), Vec3::X);
    pin(&mut s, "slider", "Guide", "Ground", "Car", Vec3::new(10.0, 50.0, 0.0), Vec3::X);
    pin(&mut s, "slider", "Lift", "Ground", "Other", Vec3::new(0.0, 100.0, 0.0), Vec3::Y);
    run(
        &mut s,
        "joint.create",
        json!({"type": "rigid", "a": {"occurrence": "Ground", "point": [0, 100, 0]}, "b": {"occurrence": "Other", "point": [10, 100, 0]}, "flip": true, "name": "Wrong"}),
    );
    run(&mut s, "joint.motion_link", json!({"a": "Table", "ia": 0, "b": "Rail", "ratio": 1}));
    run(&mut s, "joint.motion_link", json!({"a": "Table", "ia": 1, "b": "Lift", "ratio": 1}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Table", "values": [10]}));
    assert!(r["stopped_by_loop"].is_null(), "{r}");
    let names: Vec<&str> = r["conflicts"].as_array().into_iter().flatten().filter_map(|c| c["name"].as_str()).collect();
    assert_eq!(names, ["Wrong"], "{r}");
    assert!(near(place(&s, "Car", Vec3::new(0.0, 50.0, 0.0)), Vec3::new(10.0, 50.0, 0.0)), "{:?}", place(&s, "Car", Vec3::new(0.0, 50.0, 0.0)));
}

/// Two loops can move different values of one joint: each keeps what it moved. A table's x
/// feeds one pair of parallel slides and its y another; a study drives both pairs.
#[test]
fn loops_moving_different_values_of_one_joint_keep_both() {
    let mut s = Session::default();
    for (name, y) in [("Ground", -10.0), ("Puck", -20.0), ("CarA", 50.0), ("CarB", 100.0)] {
        part(&mut s, name, [-1.0, y, 0.0]);
    }
    let g = occ(&s, "Ground");
    run(&mut s, "occurrence.ground", json!({"occurrence": g, "grounded": true}));
    let flat = |o: &str| framed(o, [0.0, -20.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    run(&mut s, "joint.create", json!({"type": "planar", "a": flat("Ground"), "b": flat("Puck"), "flip": true, "name": "Table"}));
    for (car, y) in [("CarA", 50.0), ("CarB", 100.0)] {
        pin(&mut s, "slider", &format!("{car} rail"), "Ground", car, Vec3::new(0.0, y, 0.0), Vec3::X);
        pin(&mut s, "slider", &format!("{car} guide"), "Ground", car, Vec3::new(10.0, y, 0.0), Vec3::X);
    }
    run(&mut s, "joint.motion_link", json!({"a": "Table", "ia": 0, "b": "CarA rail", "ratio": 1}));
    run(&mut s, "joint.motion_link", json!({"a": "Table", "ia": 1, "b": "CarB rail", "ratio": 1}));
    let keys = json!([{"joint": "CarA guide", "points": [[0, 0], [10, 10]]}, {"joint": "CarB guide", "points": [[0, 0], [10, 20]]}]);
    run(&mut s, "motion.study", json!({"name": "Both", "steps": 10, "keys": keys}));
    let r = run(&mut s, "motion.play", json!({"study": "Both", "step": 10}));
    assert!(r["stopped_by_loop"].is_null(), "{r}");
    let table = s.doc.assembly.joints.iter().find(|j| j.name == "Table").map(|j| j.values.clone()).unwrap_or_default();
    assert!(table.len() == 3 && (table[0] - 10.0).abs() < 1e-6 && (table[1] - 20.0).abs() < 1e-6, "{table:?}");
    // A solve with nothing driven leaves them where they are.
    run(&mut s, "joint.solve", json!({}));
    assert!(near(place(&s, "CarA", Vec3::new(0.0, 50.0, 0.0)), Vec3::new(10.0, 50.0, 0.0)), "{:?}", place(&s, "CarA", Vec3::new(0.0, 50.0, 0.0)));
    assert!(near(place(&s, "CarB", Vec3::new(0.0, 100.0, 0.0)), Vec3::new(20.0, 100.0, 0.0)), "{:?}", place(&s, "CarB", Vec3::new(0.0, 100.0, 0.0)));
}

/// A clamped link can jump: a 100:1 link to a rocker limited to a full turn reaches its limit
/// after the dial's first few degrees, back where it started. Each step is checked, so the drive
/// still stops where the rocker can't go.
#[test]
fn a_clamped_link_is_followed_step_by_step() {
    let (mut s, _, _) = four_bar();
    part(&mut s, "Dial", [80.0, 0.0, -5.0]);
    pin(&mut s, "revolute", "Dial", "Ground", "Dial", Vec3::new(80.0, 0.0, 0.0), Vec3::Z);
    run(&mut s, "joint.limits", json!({"joint": "Rocker", "min": 0, "max": 360}));
    run(&mut s, "joint.motion_link", json!({"a": "Dial", "b": "Rocker", "ratio": 100}));
    let r = run(&mut s, "joint.drive", json!({"joint": "Dial", "value": 360}));
    assert!(r["stopped_by_loop"].is_object(), "{r}");
    assert_eq!(r["conflicts"].as_array().map(Vec::len), Some(0), "{r}");
}
