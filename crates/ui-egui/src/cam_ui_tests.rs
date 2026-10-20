use serde_json::json;

use super::*;
use crate::scenario::Harness;

/// A 40 × 30 rectangle sketched on XY.
/// An app with a tool library file of its own, so tests do not share one.
fn lab() -> Harness {
    static NEXT: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!("solidcraft-ui-tools-{}-{n}.json", std::process::id()));
    let _ = std::fs::remove_file(&path);
    let mut h = Harness::new();
    h.app.manufacture.library = Some(path.to_string_lossy().to_string());
    h
}

/// Defines the library's next tool from the toolbar: a 6 mm end mill at 10000 rev/min and 600 mm/min.
fn define_tool(h: &mut Harness) {
    fill(h, "cam.tool.define", |f| {
        let Form::Tool(tool) = f else { panic!("not a tool form") };
        assert!(tool.library && !tool.editing);
        tool.spindle_rpm = "10000".into();
        tool.feed = "600".into();
    });
}

fn sketched() -> Harness {
    let mut h = lab();
    h.app.run("sketch.create", json!({"plane": "XY", "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    h
}

/// Starts `id` from the toolbar, lets `edit` fill the form in, and presses OK.
fn fill(h: &mut Harness, id: &str, edit: impl FnOnce(&mut Form)) {
    h.app.start(id);
    assert!(h.app.manufacture.open || h.app.manufacture.tools_open, "{id} opens its window");
    edit(h.app.manufacture.form.as_mut().unwrap_or_else(|| panic!("{id} has no form")));
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None, "{id}");
    assert!(h.app.manufacture.form.is_none(), "the form closes when its command succeeds");
}

#[test]
fn toolbar_forms_make_a_job_and_its_toolpath_is_drawn() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    // The form starts on the newest sketch and the first tool; only the depth is typed.
    fill(&mut h, "cam.op.pocket", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        assert_eq!((op.sketch.as_str(), op.tool.as_str()), ("Pocket", "1"));
        op.bottom = "-3 mm".into();
    });
    let m = &h.app.session.doc.manufacture;
    assert_eq!((m.tools.len(), m.setups.len(), m.setups[0].operations.len()), (1, 1, 1));

    h.frames(3);
    assert!(h.widget("Toolpaths").is_some() && h.widget("Pocket1  (pocket, T1)").is_some(), "the window lists the operation");
    let proj = crate::viewport::projection(&h.app, h.app.viewport.rect.unwrap());
    let drawn = segments(&h.app, &proj);
    assert!(drawn.iter().any(|s| s.2) && drawn.iter().any(|s| !s.2), "cuts and rapids are drawn: {}", drawn.len());
    // The 6 mm tool's outer pass is 3 mm inside the 40 × 30 pocket, 3 mm down.
    let low = paths(&mut h.app).iter().flatten().filter(|p| p.1).fold([f64::INFINITY; 3], |m, p| [m[0].min(p.0.x), m[1].min(p.0.y), m[2].min(p.0.z)]);
    assert!((low[0] - 3.0).abs() < 1e-6 && (low[1] - 3.0).abs() < 1e-6 && (low[2] + 3.0).abs() < 1e-9, "{low:?}");

    // Hidden toolpaths are not drawn; a closed window draws none either.
    h.app.manufacture.show_paths = false;
    assert!(segments(&h.app, &proj).is_empty());
    h.app.manufacture.show_paths = true;
    h.app.manufacture.open = false;
    assert!(segments(&h.app, &proj).is_empty());
}

#[test]
fn toolpath_follows_the_design_and_errors_stay_in_the_window() {
    let mut h = sketched();
    h.app.run("parameters.add", json!({"name": "depth", "expression": "2 mm"})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    fill(&mut h, "cam.op.pocket", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        op.bottom = "-depth".into();
    });
    let lowest = |h: &mut Harness| paths(&mut h.app).iter().flatten().map(|p| p.0.z).fold(f64::INFINITY, f64::min);
    assert_eq!(lowest(&mut h), -2.0);
    h.app.run("parameters.change", json!({"name": "depth", "expression": "4 mm"})).unwrap();
    assert_eq!(lowest(&mut h), -4.0);

    // A form the engine refuses stays open with the reason.
    h.app.start("cam.op.pocket");
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.tool = "7".into();
    }
    apply(&mut h.app);
    assert!(h.app.manufacture.error.as_deref().is_some_and(|e| e.contains("no tool 7")), "{:?}", h.app.manufacture.error);
    assert!(h.app.manufacture.form.is_some());
    // So does one that is not a number.
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.feed = "fast".into();
    }
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error.as_deref(), Some("The feed must be a number"));
    assert_eq!(h.app.session.doc.manufacture.setups[0].operations.len(), 1);
    h.frames(2);
}

#[test]
fn selected_faces_become_the_operation_s_geometry() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    let top = solidcraft_engine::geom::Vec3::new(30.0, 20.0, 10.0);
    h.app.session.selection = crate::dialogs::face_sel(&h.app.session, top).into_iter().collect();
    assert_eq!(h.app.session.selection.len(), 1);
    // Nothing is typed: the outline and both heights come from the face and its body.
    fill(&mut h, "cam.op.contour", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        assert_eq!((op.faces.len(), op.bottom.as_str()), (1, ""));
    });
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in paths(&mut h.app).iter().flatten().filter(|p| p.1) {
        (low, high) = (low.min(p.0.z), high.max(p.0.z));
    }
    assert!(low.abs() < 1e-9 && high <= 10.0 + 1e-9, "{low} {high}");
    h.frames(2);
    // With a face selected an open form offers to cut it; a sketch can still be used instead.
    h.app.start("cam.op.pocket");
    h.frames(2);
    assert!(h.widget("1 picked").is_some() && h.widget("Use a sketch").is_some());
}

#[test]
fn edit_fills_the_form_from_the_operation_and_changes_it_in_place() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    fill(&mut h, "cam.op.pocket", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        op.bottom = "-3 mm".into();
    });
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let (setup, op) = (&listed["setups"][0], &listed["setups"][0]["operations"][0]);
    let Some(Form::Op(mut form)) = edit_form(&h.app, setup, op) else { panic!("no form") };
    // The form shows what the operation has, defaults included.
    assert_eq!(
        (form.name.as_str(), form.sketch.as_str(), form.tool.as_str(), form.bottom.as_str(), form.top.as_str()),
        ("Pocket1", "Pocket", "1", "-3 mm", "")
    );
    assert_eq!((form.stepover.as_str(), form.ramp_angle.as_str(), form.feed.as_str(), form.climb), ("2.4 mm", "3 deg", "600", true));
    // Unchanged, it changes nothing (and does not reset which profiles are cut).
    let before = h.app.session.doc.manufacture.clone();
    let (id, p) = command(&Form::Op(form.clone())).unwrap();
    assert_eq!(id, "cam.op.edit");
    assert!(p.get("sketch").is_none() && p.get("faces").is_none() && p["top"].is_null(), "{p}");
    h.app.manufacture.form = Some(Form::Op(form.clone()));
    apply(&mut h.app);
    assert_eq!((h.app.manufacture.error.clone(), &h.app.session.doc.manufacture), (None, &before));
    // Deeper and renamed: the same operation, changed.
    form.bottom = "-6 mm".into();
    form.name = "Deep".into();
    h.app.manufacture.form = Some(Form::Op(form));
    apply(&mut h.app);
    let ops = &h.app.session.doc.manufacture.setups[0].operations;
    assert_eq!((ops.len(), ops[0].id, ops[0].name.as_str(), ops[0].bottom.as_deref()), (1, before.setups[0].operations[0].id, "Deep", Some("-6 mm")));
    assert_eq!(paths(&mut h.app).iter().flatten().map(|p| p.0.z).fold(f64::INFINITY, f64::min), -6.0);
    h.frames(2);
    assert!(h.widget("Edit").is_some() && h.widget("Up").is_some() && h.widget("Down").is_some());
    // An open form offers the design's sketches and the library's tools.
    h.app.start("cam.op.contour");
    h.frames(2);
    assert!(h.widget("Pocket").is_some() && h.widget("T1  D6 flat endmill").is_some());
}

#[test]
fn simulate_reports_in_the_window_until_the_design_changes() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    let top = solidcraft_engine::geom::Vec3::new(30.0, 20.0, 10.0);
    h.app.session.selection = crate::dialogs::face_sel(&h.app.session, top).into_iter().collect();
    fill(&mut h, "cam.op.contour", |_| {});
    h.frames(2);
    h.app.start("cam.simulate");
    let (found, warnings) = h.app.manufacture.simulation.clone().expect("a simulation");
    // The tool runs round the outside of the box: it removes nothing and cuts into nothing.
    assert_eq!(found, ["Removed 0.00 cm³", "Does not cut into the part", "Leaves up to 0.00 mm on the part (0.00 cm³)"]);
    assert!(warnings.is_empty(), "{warnings:?}");
    // Nothing was cut: the whole stock is drawn as the part, its top at the box's top.
    let field = h.app.manufacture.stock.clone().expect("the cut stock");
    let tris = stock_triangles(&field);
    assert!(!tris.is_empty() && tris.iter().all(|t| t.1 == StockMark::Fine));
    let (low, high) = tris.iter().flat_map(|t| t.0).fold((f64::INFINITY, f64::NEG_INFINITY), |m, p| (m.0.min(p.z), m.1.max(p.z)));
    assert!((low - 0.0).abs() < 1e-9 && (high - 10.0).abs() < 1e-9, "{low} {high}");
    h.frames(2);
    assert!(h.widget("Does not cut into the part").is_some() && h.widget("Simulate").is_some() && h.widget("Simulated stock").is_some());
    // Cutting inside the outline instead goes into the part, and says so.
    h.app.run("cam.op.edit", json!({"op": "Contour1", "side": "inside"})).unwrap();
    h.frames(2);
    assert!(h.app.manufacture.simulation.is_none(), "the old result went with the change");
    assert!(h.app.manufacture.stock.is_none() && !stock_shown(&h.app), "and so did the drawn stock");
    simulate(&mut h.app);
    // The stock is drawn with the cut-into columns marked, and goes when it is hidden.
    let field = h.app.manufacture.stock.clone().expect("the cut stock");
    let marks: Vec<StockMark> = stock_triangles(&field).into_iter().map(|t| t.1).collect();
    assert!(marks.contains(&StockMark::Gouge) && marks.contains(&StockMark::Fine), "{}", marks.len());
    let mut scene = crate::gpu::GpuScene::default();
    stock_scene(&h.app, &mut scene);
    // 28 bytes a vertex, three a triangle.
    assert_eq!(scene.tris.len(), marks.len() * 3 * 28);
    let (shown, stamp) = stock_key(&h.app);
    h.app.manufacture.show_stock = false;
    assert_eq!((shown, stock_key(&h.app)), (true, (false, stamp)));
    let mut scene = crate::gpu::GpuScene::default();
    stock_scene(&h.app, &mut scene);
    assert!(scene.tris.is_empty());
    h.app.manufacture.show_stock = true;
    h.frames(2);
    let (found, warnings) = h.app.manufacture.simulation.clone().unwrap();
    assert!(
        found[1].starts_with("Cuts up to 10.000 mm into the part") && warnings.iter().any(|w| w.contains("into the part")),
        "{found:?} {warnings:?}"
    );
}

#[test]
fn setup_form_sets_stock_origin_and_axis_and_they_are_drawn() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    // With a body in the design a new setup starts on the stock's top corner; 5 mm of stock is typed.
    fill(&mut h, "cam.setup.create", |f| {
        let Form::Setup(setup) = f else { panic!("not a setup form") };
        assert_eq!((setup.origin, setup.anchors), (0, [0, 0, 2]));
        setup.side = "5 mm".into();
        setup.top = "2 mm".into();
    });
    let view = setup_views(&mut h.app).first().cloned().expect("a setup to draw");
    let close = |a: Vec3, b: [f64; 3]| a.dist(Vec3::new(b[0], b[1], b[2])) < 1e-6;
    assert!(close(view.frame[0], [-5.0, -5.0, 12.0]), "{view:?}");
    let (lo, hi) = view.stock.unwrap();
    assert!(close(lo, [0.0, 0.0, -12.0]) && close(hi, [70.0, 50.0, 0.0]), "{view:?}");
    // Twelve edges of the stock box, in the world, and the origin's three arms along the work axes.
    let (edges, arms) = setup_lines(&view);
    assert_eq!(edges.len(), 12);
    let (min, max) =
        edges.iter().flat_map(|e| [e.0, e.1]).fold((Vec3::new(1e9, 1e9, 1e9), Vec3::new(-1e9, -1e9, -1e9)), |m, p| (m.0.min(p), m.1.max(p)));
    assert!(close(min, [-5.0, -5.0, 0.0]) && close(max, [65.0, 45.0, 12.0]));
    assert!(arms.iter().all(|a| close(a.0, [-5.0, -5.0, 12.0])) && (arms[2].1 - arms[2].0).normalized().is_some_and(|d| close(d, [0.0, 0.0, 1.0])));
    h.frames(2);
    assert!(h.widget("Stock and origin").is_some());

    // Edit shows what the setup has; turned over with the origin in the middle of the part's far face.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Setup(mut form)) = setup_form(&listed["setups"][0]) else { panic!("no form") };
    assert_eq!(
        (form.name.as_str(), form.axis, form.origin, form.side.as_str(), form.top.as_str(), form.bottom.as_str()),
        ("Setup1", 0, 0, "5 mm", "2 mm", "")
    );
    form.axis = 1;
    form.origin = 1;
    form.anchors = [1, 1, 2];
    form.top = String::new();
    h.app.manufacture.form = Some(Form::Setup(form));
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let view = setup_views(&mut h.app).first().cloned().unwrap();
    // The underside is the top of this setup: the middle of the box's bottom face, Z pointing down in the world.
    assert!(close(view.frame[0], [30.0, 20.0, 0.0]) && close(view.frame[3], [0.0, 0.0, -1.0]), "{view:?}");
    let setup = &h.app.session.doc.manufacture.setups[0];
    assert_eq!((setup.stock.side.as_str(), setup.stock.top.as_str()), ("5 mm", "0"));
    h.frames(2);
}

#[test]
fn facing_boundary_and_posting_one_operation_or_the_setup() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    h.app.run("sketch.create", json!({"plane": "XY", "offset": 10, "name": "Patch"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [30, 20]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |f| {
        let Form::Setup(setup) = f else { panic!("not a setup form") };
        setup.side = "5 mm".into();
        setup.top = "1 mm".into();
        setup.origin = 2;
    });
    let span = |h: &mut Harness, op: usize| {
        let cuts = paths(&mut h.app)[op].iter().filter(|p| p.1).fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |m, p| {
            ([m.0[0].min(p.0.x), m.0[1].min(p.0.y)], [m.1[0].max(p.0.x), m.1[1].max(p.0.y)])
        });
        [cuts.0[0], cuts.0[1], cuts.1[0], cuts.1[1]].map(|c| (c * 1e6).round() / 1e6)
    };
    // With a body in the design, facing starts on the stock, over it by the tool's radius.
    fill(&mut h, "cam.op.face", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        assert_eq!((op.boundary, op.limit, op.margin.as_str()), (0, 0, ""));
    });
    assert_eq!(span(&mut h, 0), [-8.0, -5.0, 68.0, 45.0]);
    h.frames(2);
    assert!(h.widget("Post Process  (rs274)").is_some() && h.widget("Post").is_some());

    // Edit: the part instead, staying 1 mm inside its outline.
    let edit = |h: &mut Harness, change: &dyn Fn(&mut OpForm)| {
        let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
        let Some(Form::Op(mut form)) = edit_form(&h.app, &listed["setups"][0], &listed["setups"][0]["operations"][0]) else { panic!("no form") };
        change(&mut form);
        h.app.manufacture.form = Some(Form::Op(form));
        apply(&mut h.app);
        assert_eq!(h.app.manufacture.error, None);
    };
    edit(&mut h, &|f| {
        assert_eq!((f.boundary, f.limit), (0, 0));
        f.boundary = 1;
        f.limit = 1;
        f.margin = "1 mm".into();
    });
    assert_eq!(span(&mut h, 0), [4.0, 4.0, 56.0, 36.0]);
    // A sketch's box, over it by 2 mm; the form shows what the operation has.
    edit(&mut h, &|f| {
        assert_eq!((f.boundary, f.limit, f.margin.as_str()), (1, 1, "1 mm"));
        f.boundary = 2;
        f.sketch = "Patch".into();
        f.limit = 0;
        f.margin = "2 mm".into();
    });
    assert_eq!(span(&mut h, 0), [8.0, 11.0, 32.0, 19.0]);
    // Unchanged, an edit changes nothing; emptied, the margin goes back to the tool's radius.
    let before = h.app.session.doc.manufacture.clone();
    edit(&mut h, &|_| {});
    assert_eq!(h.app.session.doc.manufacture, before);
    edit(&mut h, &|f| f.margin = String::new());
    assert_eq!(span(&mut h, 0), [7.0, 10.0, 33.0, 20.0]);

    // One operation's program, and the setup's.
    fill(&mut h, "cam.op.contour", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        op.sketch = "Patch".into();
        op.bottom = "8 mm".into();
    });
    let dir = std::env::temp_dir().join(format!("solidcraft-post-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let (one, all) = (dir.join("one.ngc"), dir.join("all.ngc"));
    let (setup, op) = (h.app.session.doc.manufacture.setups[0].id, h.app.session.doc.manufacture.setups[0].operations[1].id);
    post_to(&mut h.app, setup, Some(op), &one.to_string_lossy());
    post_to(&mut h.app, setup, None, &all.to_string_lossy());
    assert_eq!(h.app.manufacture.error, None);
    let (one, all) = (std::fs::read_to_string(one).unwrap(), std::fs::read_to_string(all).unwrap());
    assert!(one.contains("(Contour1:") && !one.contains("(Face1:") && all.contains("(Face1:") && all.contains("(Contour1:"));
    let _ = std::fs::remove_dir_all(dir);
}

#[test]
fn tool_library_is_the_machine_s_and_fills_operation_forms() {
    let mut h = lab();
    fill(&mut h, "cam.tool.define", |f| {
        let Form::Tool(tool) = f else { panic!("not a tool form") };
        tool.description = "Rougher".into();
        tool.flutes = "3".into();
        tool.spindle_rpm = "18000".into();
        tool.feed = "1500".into();
        tool.stepdown = "1.5".into();
        tool.stepover = "2".into();
    });
    assert!(h.app.manufacture.tools_open && !h.app.manufacture.open);
    // The tool is the machine's, not this design's.
    assert!(h.app.session.doc.manufacture.tools.is_empty());
    h.frames(2);
    assert!(h.widget("TOOL LIBRARY").is_some() && h.widget("T1  Rougher").is_some() && h.widget("18000").is_some() && h.widget("New Tool").is_some());
    // 1500 mm/min at 18000 rev/min with 3 flutes is 0.0278 mm a tooth; no form is open until one is asked for.
    assert!(h.widget("0.0278").is_some() && h.widget("Pick a tool to change or remove it").is_some());
    assert!(
        h.app.manufacture.form.is_none()
            && h.widget("NEW TOOL").is_none()
            && h.widget("Diameter").is_some_and(|_| h.widget("Flute length").is_none())
    );
    // Picking the tool offers what can be done with it.
    h.app.manufacture.selected_tool = Some((true, 1));
    h.frames(2);
    assert!(h.widget("Edit").is_some() && h.widget("Remove").is_some() && h.widget("Pick a tool to change or remove it").is_none());
    // Edit fills the form from the tool; an emptied feed is cleared, the number stays.
    let library = library_listing(&mut h.app, true);
    let Some(Form::Tool(mut form)) = tool_form(&library["tools"][0], true) else { panic!("no form") };
    assert_eq!(
        (form.number.as_str(), form.diameter.as_str(), form.flutes.as_str(), form.feed.as_str(), form.plunge_feed.as_str(), form.stepover.as_str()),
        ("1", "6 mm", "3", "1500", "", "2")
    );
    assert_eq!(form.chip_load, "0.0278");
    form.plunge_feed = "400".into();
    h.app.manufacture.form = Some(Form::Tool(form.clone()));
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    // Speed, flutes, chip load and feed move together.
    let (mut feed, mut chip) = ("1500".to_string(), "0.0278".to_string());
    link_chip_load("18000", Some(3.0), &mut feed, &mut chip, false, false, true);
    assert_eq!(feed, "1501.2", "a typed chip load gives the feed");
    chip = "0.02".into();
    link_chip_load("18000", Some(4.0), &mut feed, &mut chip, true, false, false);
    assert_eq!((feed.as_str(), chip.as_str()), ("1440", "0.02"), "another flute count at the same chip load");
    feed = "720".into();
    link_chip_load("18000", Some(4.0), &mut feed, &mut chip, false, true, false);
    assert_eq!(chip, "0.01", "a typed feed gives the chip load");
    link_chip_load("", Some(4.0), &mut feed, &mut chip, false, true, false);
    assert_eq!((feed.as_str(), chip.as_str()), ("720", "0.01"), "nothing to work out without a speed");
    // A new tool given a chip load and no feed gets its feed from the engine: 20000 × 2 × 0.015.
    fill(&mut h, "cam.tool.define", |f| {
        let Form::Tool(tool) = f else { panic!("not a tool form") };
        tool.number = "40".into();
        tool.spindle_rpm = "20000".into();
        tool.chip_load = "0.015".into();
    });
    let library = library_listing(&mut h.app, true);
    assert_eq!(library["tools"][1]["feed"], 600.0);
    run(&mut h.app, "cam.library.remove", json!({"number": 40})).unwrap();

    // An operation form shows the tool's speed, feeds, stepdown and stepover, to keep or change.
    h.app.run("sketch.create", json!({"plane": "XY", "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    fill(&mut h, "cam.setup.create", |_| {});
    fill(&mut h, "cam.op.pocket", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        assert_eq!((op.tool.as_str(), op.spindle_rpm.as_str(), op.feed.as_str(), op.plunge_feed.as_str()), ("1", "18000", "1500", "400"));
        assert_eq!((op.flutes, op.chip_load.as_str()), (3, "0.0278"));
        assert_eq!((op.stepdown.as_str(), op.stepover.as_str()), ("1.5 mm", "2 mm"));
        // This operation runs slower; the tool keeps its own feed.
        op.feed = "900".into();
        op.bottom = "-3 mm".into();
    });
    let op = &h.app.session.doc.manufacture.setups[0].operations[0];
    assert_eq!((op.spindle_rpm, op.cut_feed, op.plunge_feed, op.stepdown.as_str()), (18000.0, 900.0, 400.0, "1.5 mm"));
    // The design took a copy of the tool, which the library window says.
    assert_eq!(h.app.session.doc.manufacture.tool(1).and_then(|t| t.feed), Some(1500.0));
    h.frames(2);
    assert!(h.widget("in use").is_some() && h.widget("Tool Library…").is_some());

    // A second tool with other data: choosing it in an open form brings its data in.
    fill(&mut h, "cam.tool.define", |f| {
        let Form::Tool(tool) = f else { panic!("not a tool form") };
        assert_eq!(tool.number, "2");
        tool.diameter = "4 mm".into();
        tool.spindle_rpm = "20000".into();
        tool.feed = "700".into();
    });
    h.app.start("cam.op.contour");
    h.frames(1);
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        assert_eq!(op.feed, "1500");
        op.tool = "2".into();
    }
    h.frames(2);
    let Some(Form::Op(op)) = &h.app.manufacture.form else { panic!("the form closed") };
    assert_eq!((op.spindle_rpm.as_str(), op.feed.as_str(), op.stepdown.as_str()), ("20000", "700", ""));

    // The library's tool changes: this design is offered the update and takes it.
    let library = library_listing(&mut h.app, true);
    let Some(Form::Tool(mut form)) = tool_form(&library["tools"][0], true) else { panic!("no form") };
    form.feed = "1600".into();
    h.app.manufacture.form = Some(Form::Tool(form));
    apply(&mut h.app);
    h.frames(2);
    assert!(h.widget("changed").is_some());
    h.app.manufacture.selected_tool = Some((true, 1));
    h.frames(2);
    assert!(h.widget("Update this design").is_some());
    assert_eq!(h.app.session.doc.manufacture.tool(1).and_then(|t| t.feed), Some(1500.0));
    run(&mut h.app, "cam.tool.update", json!({"number": 1})).unwrap();
    assert_eq!(h.app.session.doc.manufacture.tool(1).and_then(|t| t.feed), Some(1600.0));
    // A tool made only in this design is listed apart and can join the library.
    h.app.run("cam.tool.define", json!({"number": 30, "kind": "drill", "diameter": 3})).unwrap();
    h.app.manufacture.form = None;
    h.frames(2);
    assert!(h.widget("Only in this design").is_some());
    h.app.manufacture.selected_tool = Some((false, 30));
    h.frames(2);
    assert!(h.widget("Add to library").is_some());
    run(&mut h.app, "cam.library.add", json!({"number": 30})).unwrap();
    assert_eq!(library_listing(&mut h.app, true)["tools"].as_array().map(Vec::len), Some(3));
}

#[test]
fn manufacture_tab_shows_every_cam_command_with_a_drawn_icon() {
    assert!(crate::toolbar::TABS.contains(&"MANUFACTURE"));
    let specs = solidcraft_engine::command_specs();
    let on_tab: Vec<_> = specs.iter().filter(|c| c.tab == "MANUFACTURE").collect();
    assert_eq!(on_tab.len(), 10, "cam.op.edit and the removals are reached from the window");
    for c in on_tab {
        assert!(crate::toolbar::panels("MANUFACTURE").contains(&c.panel), "{}: panel {}", c.id, c.panel);
        assert!(c.icon.starts_with("cam_"), "{}", c.id);
        let mut h = lab();
        h.app.start(c.id);
        assert_eq!(
            (h.app.manufacture.open, h.app.manufacture.tools_open),
            (!c.id.starts_with("cam.library."), c.id.starts_with("cam.library.")),
            "{}",
            c.id
        );
        h.frames(2);
    }
}

/// The Manufacture and Tool Library panels move by their headers, each on its own and kept
/// on screen; a double-click on a header docks the panel again.
#[test]
fn panels_move_by_their_headers_and_dock_again() {
    let mut h = lab();
    h.call("ui.resize", json!({"width": 1600.0, "height": 960.0}));
    h.app.manufacture.open = true;
    h.app.manufacture.tools_open = true;
    h.frames(6);
    let rect = |h: &Harness, id: &str| h.ctx.memory(|m| m.area_rect(egui::Id::new(id))).expect("drawn");
    let (panel, tools) = (rect(&h, "sc_cam_window"), rect(&h, "sc_cam_tools"));
    assert!(tools.right() <= panel.left(), "the Tool Library docks left of the Manufacture panel: {tools:?} {panel:?}");

    let title = crate::scenario::handle_at("sc_cam_window:title").expect("header");
    h.call("ui.drag", json!({"x0": title.x, "y0": title.y, "x1": title.x - 200.0, "y1": title.y + 250.0, "steps": 8}));
    h.frames(4);
    let moved = rect(&h, "sc_cam_window");
    assert!((moved.min - panel.min - egui::vec2(-200.0, 250.0)).length() < 2.0, "{panel:?} -> {moved:?}");
    assert!((rect(&h, "sc_cam_tools").min - tools.min).length() < 0.5, "the Tool Library stays where it docks");
    assert!(h.app.manufacture.open, "dragging the header keeps the panel open");

    // Dragged far off, it stays on screen.
    let title = crate::scenario::handle_at("sc_cam_tools:title").expect("header");
    h.call("ui.drag", json!({"x0": title.x, "y0": title.y, "x1": title.x - 5000.0, "y1": title.y + 5000.0, "steps": 8}));
    h.frames(4);
    let viewport = h.app.viewport.rect.expect("viewport");
    assert!(viewport.contains(rect(&h, "sc_cam_tools").min));

    let title = crate::scenario::handle_at("sc_cam_window:title").expect("header");
    h.call("ui.click", json!({"x": title.x, "y": title.y, "double": true}));
    h.frames(4);
    assert!((rect(&h, "sc_cam_window").min - panel.min).length() < 0.5, "double-click docks it again");
}

/// The browser lists the setups and their operations under a Manufacture folder; its eye
/// shows and hides the toolpaths, a double-click edits, and the right button's menu edits,
/// reorders and removes.
#[test]
fn browser_lists_setups_and_operations_with_their_menus() {
    use crate::context_menu::{Target, items, run_item};
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    h.frames(2);
    let rows = |_: &Harness| crate::browser::drawn_rows().into_iter().map(|r| r.0).collect::<Vec<_>>();
    assert!(!rows(&h).iter().any(|r| r == "Manufacture"), "no folder without a setup");

    h.app.run("cam.tool.define", json!({"number": 1, "kind": "flat", "diameter": "6 mm", "spindle_rpm": 10000, "feed": 600})).unwrap();
    h.app.run("cam.setup.create", json!({"stock": {"top": 1}})).unwrap();
    h.app.run("cam.op.face", json!({"tool": 1, "name": "Top"})).unwrap();
    h.app.run("cam.op.face", json!({"tool": 1, "name": "Again"})).unwrap();
    h.frames(2);
    let drawn = rows(&h);
    let at = |name: &str| drawn.iter().position(|r| r == name).unwrap_or_else(|| panic!("no row {name}: {drawn:?}"));
    assert!(at("Manufacture") < at("Setup1") && at("Setup1") < at("Top") && at("Top") < at("Again"));

    let setup = h.app.session.doc.manufacture.setups[0].id;
    let ops: Vec<u64> = h.app.session.doc.manufacture.setups[0].operations.iter().map(|o| o.id).collect();
    let labels =
        |h: &Harness, op: Option<u64>| items(&h.app, &Target::Cam { setup, op }).into_iter().map(|i| (i.label, i.enabled)).collect::<Vec<_>>();
    assert!(labels(&h, None).contains(&("Simulate".to_string(), true)));
    let first = labels(&h, Some(ops[0]));
    assert!(first.contains(&("Move Up".to_string(), false)) && first.contains(&("Move Down".to_string(), true)), "{first:?}");

    // Move Down puts the first operation last.
    let down = items(&h.app, &Target::Cam { setup, op: Some(ops[0]) }).into_iter().find(|i| i.label == "Move Down").unwrap();
    run_item(&mut h.app, &down, egui::Pos2::ZERO);
    let order: Vec<u64> = h.app.session.doc.manufacture.setups[0].operations.iter().map(|o| o.id).collect();
    assert_eq!(order, vec![ops[1], ops[0]]);

    // Edit opens the Manufacture panel on the operation's form.
    let edit = items(&h.app, &Target::Cam { setup, op: Some(ops[0]) }).into_iter().find(|i| i.label == "Edit Operation").unwrap();
    run_item(&mut h.app, &edit, egui::Pos2::ZERO);
    assert!(h.app.manufacture.open && matches!(&h.app.manufacture.form, Some(Form::Op(f)) if f.editing.is_some()));
    h.app.manufacture.form = None;

    // The folder's eye hides the toolpaths, and keeps them drawn once the panel is closed.
    h.frames(1);
    assert!(paths_drawn(&h.app));
    let eye = crate::scenario::handle_at("eye:Manufacture").expect("the folder has an eye");
    h.call("ui.click", json!({"x": eye.x, "y": eye.y}));
    h.frames(2);
    assert!(!paths_drawn(&h.app) && !h.app.manufacture.show_paths);
    h.app.manufacture.open = false;
    h.frames(1);
    h.call("ui.click", json!({"x": eye.x, "y": eye.y}));
    h.frames(2);
    assert!(paths_drawn(&h.app), "the eye draws the toolpaths without the panel");
    assert!(!paths(&mut h.app).is_empty());

    let remove = items(&h.app, &Target::Cam { setup, op: Some(ops[1]) }).into_iter().find(|i| i.label == "Remove Operation").unwrap();
    run_item(&mut h.app, &remove, egui::Pos2::ZERO);
    assert_eq!(h.app.session.doc.manufacture.setups[0].operations.len(), 1);
    let remove = items(&h.app, &Target::Cam { setup, op: None }).into_iter().find(|i| i.label == "Remove Setup").unwrap();
    run_item(&mut h.app, &remove, egui::Pos2::ZERO);
    h.frames(2);
    assert!(!rows(&h).iter().any(|r| r == "Manufacture"), "the folder goes with the last setup");
}

#[test]
fn a_form_marks_what_it_still_needs_and_ok_waits_for_it() {
    let mut h = lab();
    // A plate with a pocket sketched on its top: how deep it goes is the user's to say.
    for (name, [p0, p1]) in [("Plate", [[0, 0], [60, 40]]), ("Top", [[10, 10], [50, 30]])] {
        h.app.run("sketch.create", json!({"plane": "XY", "name": name})).unwrap();
        h.app.run("sketch.rectangle.two_point", json!({"p0": p0, "p1": p1})).unwrap();
        h.app.run("sketch.finish", json!({})).unwrap();
        if name == "Plate" {
            h.app.run("solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"})).unwrap();
        }
    }
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.pocket");
    h.frames(3);
    assert!(h.widget("Still needed: Bottom (marked red; point at a row for more)").is_some());
    // OK is off: pressing it adds nothing.
    h.step(&json!({"widget": "OK"})).unwrap();
    h.frames(2);
    assert!(h.app.session.doc.manufacture.setups[0].operations.is_empty());
    assert!(h.app.manufacture.form.is_some());

    // A contour on the same sketch goes through the part: nothing is missing.
    h.app.start("cam.op.contour");
    h.frames(3);
    assert!(h.widget("Still needed: Bottom (marked red; point at a row for more)").is_none());

    // Back to the pocket: with the depth typed, OK runs.
    h.app.start("cam.op.pocket");
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.bottom = "-3 mm".into();
    }
    h.frames(3);
    assert!(h.widget("Still needed: Bottom (marked red; point at a row for more)").is_none());
    h.step(&json!({"widget": "OK"})).unwrap();
    h.frames(2);
    assert_eq!(h.app.session.doc.manufacture.setups[0].operations.len(), 1);
    assert!(h.app.manufacture.form.is_none());
}

#[test]
fn missing_rows_follow_the_form() {
    let tools = vec![("1".to_string(), "T1  D6 flat".to_string())];
    let data = vec![("1".to_string(), ToolData { diameter: 6.0, flutes: 2, ..ToolData::default() })];
    let lists = Choices { sketches: Vec::new(), tools: tools.clone(), data: data.clone(), heights: None, picked: None };
    let fields = |form: &Form, lists: &Choices| missing(form, lists).into_iter().map(|n| n.field).collect::<Vec<_>>();
    // No sketch to cut, and a tool without speed or feed of its own.
    let mut h = sketched();
    h.app.start("cam.op.contour");
    let Some(Form::Op(mut op)) = h.app.manufacture.form.clone() else { panic!("not an operation form") };
    op.sketch.clear();
    op.tool = "1".into();
    op.spindle_rpm.clear();
    op.feed.clear();
    let form = Form::Op(op.clone());
    assert_eq!(fields(&form, &lists), ["Sketch", "Spindle", "Feed"]);
    // A chip load with a speed gives the feed; what the geometry cannot give is said as it is.
    op.spindle_rpm = "10000".into();
    op.chip_load = "0.03".into();
    op.flutes = 2;
    op.sketch = "Pocket".into();
    let failing = Choices { sketches: Vec::new(), tools, data, heights: Some(Err("no closed profile".into())), picked: None };
    let need = missing(&Form::Op(op), &failing);
    assert_eq!(need, vec![Need { field: "", why: "no closed profile".into() }]);
    assert_eq!(need_text(&need), "no closed profile");
    // A tool needs its number and diameter; a setup on a world point needs all three of its coordinates.
    if let Some(Form::Tool(mut tool)) = tool_form(&json!({"number": 3, "kind": "flat", "diameter": 6}), true) {
        tool.editing = false;
        tool.number.clear();
        tool.diameter.clear();
        assert_eq!(fields(&Form::Tool(tool), &lists), ["Number", "Diameter"]);
    }
    h.app.start("cam.setup.create");
    let Some(Form::Setup(mut setup)) = h.app.manufacture.form.clone() else { panic!("not a setup form") };
    setup.origin = 2;
    setup.point[1].clear();
    assert_eq!(fields(&Form::Setup(setup), &lists), ["Origin Y"]);
}

#[test]
fn heights_are_taken_from_the_selection() {
    let mut h = lab();
    h.app.run("sketch.create", json!({"plane": "XY", "name": "Plate"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [60, 40]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    h.app.run("solid.extrude", json!({"sketch": "Plate", "distance": 10, "direction": "negative"})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.pocket");
    h.frames(2);
    assert!(h.widget("Bottom from the selection").is_some(), "the button is there, off while nothing is selected");
    // A corner at the plate's foot: Bottom takes its height.
    let corner = solidcraft_engine::geom::Vec3::new(0.0, 0.0, -10.0);
    h.app.session.selection = vec![solidcraft_engine::Sel::Vertex { body: "Body1".into(), point: corner }];
    h.frames(2);
    h.step(&json!({"widget": "Bottom from the selection"})).unwrap();
    h.frames(1);
    let Some(Form::Op(op)) = &h.app.manufacture.form else { panic!("the form stays open") };
    assert_eq!(op.bottom, "-10 mm");
}

#[test]
fn the_steps_lead_from_a_setup_to_a_program() {
    // A 40 × 30 block 10 high, its top the pocket's sketch.
    let mut h = sketched();
    h.app.run("solid.extrude", json!({"sketch": "Pocket", "distance": 10, "direction": "negative"})).unwrap();
    define_tool(&mut h);
    h.app.start("cam.list");
    h.frames(2);
    assert!(h.widget("Next: make a setup: where the raw block is, and the zero the machine is set to.").is_some());
    // Step 1 makes a setup.
    h.step(&json!({"widget": "1  Setup"})).unwrap();
    h.frames(1);
    assert!(matches!(h.app.manufacture.form, Some(Form::Setup(_))));
    apply(&mut h.app);
    h.frames(2);
    assert!(h.widget("Next: add an operation: what to cut, and with which tool.").is_some());
    // Step 2 offers the operations; a 2D Pocket's form opens.
    h.step(&json!({"widget": "2  Operation"})).unwrap();
    h.frames(1);
    h.step(&json!({"widget": "2D Pocket"})).unwrap();
    h.frames(1);
    let Some(Form::Op(op)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    assert_eq!(op.kind, OpKind::Pocket);
    op.bottom = "-3 mm".into();
    apply(&mut h.app);
    h.frames(2);
    assert!(h.widget("Next: simulate, to check the toolpaths against the part.").is_some());
    // Step 3 simulates.
    h.step(&json!({"widget": "3  Simulate"})).unwrap();
    h.frames(2);
    assert!(h.app.manufacture.simulation.is_some());
    assert!(h.widget("Next: write the G-code for the machine.").is_some());
    // A setup's program written finishes the steps, until the design changes.
    let setup = h.app.session.doc.manufacture.setups[0].id;
    let path = std::env::temp_dir().join(format!("solidcraft-steps-{}.nc", std::process::id()));
    post_to(&mut h.app, setup, None, &path.to_string_lossy());
    let _ = std::fs::remove_file(&path);
    h.frames(2);
    assert!(h.widget("Done: the G-code is written. A change to the design or the operations starts the checks again.").is_some());
    h.app.run("parameters.add", json!({"name": "w", "expression": "1 mm"})).unwrap();
    h.frames(2);
    assert!(h.widget("Next: simulate, to check the toolpaths against the part.").is_some());
}

#[test]
fn eyes_hide_a_setup_s_or_an_operation_s_toolpaths() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    for bottom in ["-2 mm", "-4 mm"] {
        fill(&mut h, "cam.op.pocket", |f| {
            let Form::Op(op) = f else { panic!("not an operation form") };
            op.bottom = bottom.into();
        });
    }
    h.frames(2);
    let proj = crate::viewport::projection(&h.app, h.app.viewport.rect.unwrap());
    let drawn = |h: &mut Harness| {
        paths(&mut h.app);
        segments(&h.app, &proj).len()
    };
    let all = drawn(&mut h);
    let m = &h.app.session.doc.manufacture;
    let (setup, first) = (m.setups[0].id, m.setups[0].operations[0].id);
    // One operation hidden: only the other's toolpath is drawn; the setup hidden: none.
    toggle_hidden(&mut h.app, first);
    let one = drawn(&mut h);
    assert!(one > 0 && one < all, "{one} of {all}");
    toggle_hidden(&mut h.app, setup);
    assert_eq!(drawn(&mut h), 0);
    toggle_hidden(&mut h.app, setup);
    toggle_hidden(&mut h.app, first);
    assert_eq!(drawn(&mut h), all);
    h.frames(2);
}

#[test]
fn shown_numbers_have_no_negative_zero() {
    assert_eq!(shown(-1e-12, 3), "0");
    assert_eq!(shown(-0.0004, 3), "0");
    assert_eq!(shown(-14.0, 3), "-14");
    assert_eq!(shown(2.5, 3), "2.5");
}

#[test]
fn an_operation_form_shows_its_toolpath_before_ok() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.pocket");
    // Nothing while the depth is missing.
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.bottom.clear();
    }
    h.frames(2);
    assert!(h.app.manufacture.preview.is_none());
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.bottom = "-3 mm".into();
    }
    h.frames(2);
    let low = |h: &Harness| match &h.app.manufacture.preview {
        Some((_, _, Ok(preview))) => preview.points.iter().filter(|p| p.1).map(|p| p.0.z).fold(f64::INFINITY, f64::min),
        other => panic!("no preview: {other:?}"),
    };
    assert_eq!(low(&h), -3.0);
    assert!(h.app.session.doc.manufacture.setups[0].operations.is_empty(), "a preview adds nothing");
    // A change shows at once; what stops it is said before OK.
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.bottom = "-5 mm".into();
    }
    h.frames(2);
    assert_eq!(low(&h), -5.0);
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        op.top = "-6 mm".into();
    }
    h.frames(2);
    assert!(
        matches!(&h.app.manufacture.preview, Some((_, _, Err(e))) if e.contains("above the top")),
        "{:?}",
        h.app.manufacture.preview.as_ref().map(|p| &p.2)
    );
    // Closed, the form leaves no preview behind.
    h.app.manufacture.form = None;
    h.frames(2);
    assert!(h.app.manufacture.preview.is_none());
}

#[test]
fn an_operation_on_picked_faces_opens_again_with_them() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    let top = solidcraft_engine::geom::Vec3::new(30.0, 20.0, 10.0);
    h.app.session.selection = crate::dialogs::face_sel(&h.app.session, top).into_iter().collect();
    fill(&mut h, "cam.op.contour", |_| {});
    h.app.session.selection.clear();
    // Edited, the form has the face it was made on, and nothing is missing.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let (setup, op) = (&listed["setups"][0], &listed["setups"][0]["operations"][0]);
    let form = edit_form(&h.app, setup, op).unwrap();
    let Form::Op(f) = &form else { panic!("not an operation form") };
    assert_eq!(f.faces.len(), 1);
    assert!(missing(&form, &Choices::default()).iter().all(|n| n.field != "Sketch"), "{:?}", missing(&form, &Choices::default()));
}
