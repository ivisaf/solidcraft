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
    let rows = crate::browser::drawn_rows();
    assert!(rows.iter().any(|r| r.0 == "Pocket1") && h.widget("Pocket1  (pocket, T1)").is_none(), "the browser lists the operation, the panel not");
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
    assert!(crate::browser::drawn_rows().iter().any(|r| r.0 == "Deep") && h.widget("Up").is_none(), "renamed in the browser; no list in the panel");
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
    assert_eq!(found, ["Simulated: Setup1", "Removed 0.00 cm³", "Does not cut into the part", "Leaves up to 0.00 mm on the part (0.00 cm³)"]);
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
        found[2].starts_with("Cuts up to 10.000 mm into the part") && warnings.iter().any(|w| w.contains("into the part")),
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
    let shown = folder_menu_items(&h.app).into_iter().find(|i| i.label == "Stock and origin").expect("the folder menu's entry");
    assert_eq!(shown.shortcut, "current", "checked while the stock is shown");

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
    // The spindle's spin-up from the preferences; the panel's stops and air cut when chosen.
    assert!(all.contains("G4 P3") && !all.lines().any(|l| l == "M1") && !all.contains("AIR CUT"), "{all}");
    h.app.preferences.cam_spindle_delay = 0.0;
    (h.app.manufacture.optional_stops, h.app.manufacture.air_cut, h.app.manufacture.air_height) = (true, true, 15.0);
    let safe = dir.join("safe.ngc");
    post_to(&mut h.app, setup, None, &safe.to_string_lossy());
    let safe = std::fs::read_to_string(safe).unwrap();
    assert!(!safe.contains("G4 P3") && safe.lines().any(|l| l == "M1") && safe.contains("raised by 15 mm"), "{safe}");
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
    assert!(h.widget("in use").is_some());

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
    assert_eq!(on_tab.len(), 13, "cam.op.edit and the removals are reached from the window");
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

/// Whether the form's OK button is on (`None`: there is none).
fn ok_enabled(h: &Harness) -> Option<bool> {
    let tree = h.output.accesskit_update.as_ref()?;
    tree.nodes.iter().find(|(_, n)| n.label() == Some("OK")).map(|(_, n)| !n.is_disabled())
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
    // OK is off.
    assert_eq!(ok_enabled(&h), Some(false));

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
    assert_eq!(ok_enabled(&h), Some(true));
    apply(&mut h.app);
    h.frames(2);
    assert_eq!(h.app.session.doc.manufacture.setups[0].operations.len(), 1);
    assert!(h.app.manufacture.form.is_none());
}

#[test]
fn missing_rows_follow_the_form() {
    let tools = vec![("1".to_string(), "T1  D6 flat".to_string())];
    let data = vec![("1".to_string(), ToolData { diameter: 6.0, flutes: 2, ..ToolData::default() })];
    let lists = Choices { tools: tools.clone(), data: data.clone(), ..Choices::default() };
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
    let failing = Choices { tools, data, heights: Some(Err("no closed profile".into())), ..Choices::default() };
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

#[test]
fn display_choices_are_kept_with_the_preferences() {
    let mut h = sketched();
    h.app.start("cam.list");
    h.frames(1);
    assert_eq!((h.app.manufacture.sim_detail, h.app.manufacture.path_width, h.app.manufacture.path_colour), (1, 1, 0));
    // They are preferences: saved and read again, the panel has them.
    (h.app.preferences.cam_sim_detail, h.app.preferences.cam_path_width, h.app.preferences.cam_path_colour) = (2, 2, 1);
    h.frames(1);
    assert_eq!((h.app.manufacture.sim_detail, h.app.manufacture.path_width, h.app.manufacture.path_colour), (2, 2, 1));
    let saved = serde_json::to_value(&h.app.preferences).unwrap();
    let mut again = Harness::new();
    again.app.load_prefs(&json!({ "preferences": saved }).to_string());
    again.app.start("cam.list");
    again.frames(1);
    assert_eq!((again.app.manufacture.sim_detail, again.app.manufacture.path_width, again.app.manufacture.path_colour), (2, 2, 1));
}

#[test]
fn an_import_says_what_it_took_and_left_out() {
    let mut h = lab();
    let file = std::env::temp_dir().join(format!("solidcraft-ui-rp23cnc-{}.json", std::process::id()));
    let rp = json!({"Tools": [
        {"Number": 1, "Name": "6 mm flat end mill", "Type": "EndMill", "DiameterMm": 6, "FluteCount": 2},
        {"Number": 3, "Name": "V-bit 60°", "Type": "VBit", "DiameterMm": 6, "FluteCount": 1},
        {"Number": 4, "Name": "Dovetail", "Type": "Dovetail", "DiameterMm": 10, "FluteCount": 4}
    ]});
    std::fs::write(&file, rp.to_string()).unwrap();
    h.app.start("cam.tool.define");
    h.app.manufacture.form = None;
    let result = run(&mut h.app, "cam.library.import", json!({ "path": file }));
    let _ = std::fs::remove_file(&file);
    imported(&mut h.app, result);
    h.frames(2);
    assert!(h.widget("Imported 2 tool(s)").is_some());
    assert!(h.widget("Left out tool 4 (Dovetail): a dovetail is not a kind of tool SolidCraft has yet").is_some());
    assert!(h.widget("T1  6 mm flat end mill").is_some(), "the tool is in the table");
}

#[test]
fn panels_are_dragged_wider_and_keep_their_width() {
    let mut h = sketched();
    h.app.start("cam.list");
    h.frames(3);
    let usual = h.app.manufacture.panel.unwrap();
    // The grip on the left edge drags the panel 150 wider; its right edge stays docked.
    let y = usual.center().y as f64;
    h.step(&json!({"drag": [[usual.left() as f64, y], [usual.left() as f64 - 150.0, y]], "steps": 6})).unwrap();
    h.frames(3);
    let wider = h.app.manufacture.panel.unwrap();
    assert!((wider.width() - usual.width() - 150.0).abs() < 8.0, "{} → {}", usual.width(), wider.width());
    assert!((wider.right() - usual.right()).abs() < 1.0);
    assert!((h.app.preferences.cam_panel_width - wider.width()).abs() < 1.0, "kept with the preferences");
    // A double-click on the grip gives it its usual width again.
    h.step(&json!({"click": [wider.left() as f64, y], "double": true})).unwrap();
    h.frames(3);
    assert_eq!(h.app.preferences.cam_panel_width, 0.0);
    assert!(h.app.manufacture.panel.unwrap().width() <= DIALOG_WIDE_MAX_W + 1.0);
}

#[test]
fn the_form_knows_every_height_it_cuts_at() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.pocket");
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        (op.bottom, op.stepdown) = ("-3 mm".into(), "1 mm".into());
    }
    h.frames(2);
    let seen = |h: &Harness| match &h.app.manufacture.preview {
        Some((_, _, Ok(p))) => p.heights.clone().unwrap(),
        other => panic!("no preview: {other:?}"),
    };
    // Automatic: retract 2 over the top (the sketch's plane), clearance 10 over it; three passes.
    let auto = seen(&h);
    assert_eq!((auto.clearance, auto.retract, auto.top, auto.bottom), (10.0, 2.0, 0.0, -3.0));
    assert_eq!(auto.levels, vec![-1.0, -2.0, -3.0]);
    assert!(h.widget("A picture of the heights shows once nothing is missing").is_none(), "the picture is drawn");
    // Typed in, they are what it cuts at, and what an edit opens with.
    if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
        (op.clearance, op.retract) = ("15 mm".into(), "5 mm".into());
    }
    h.frames(2);
    assert_eq!((seen(&h).clearance, seen(&h).retract), (15.0, 5.0));
    apply(&mut h.app);
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Op(f)) = edit_form(&h.app, &listed["setups"][0], &listed["setups"][0]["operations"][0]) else { panic!("no form") };
    assert_eq!((f.clearance.as_str(), f.retract.as_str()), ("15 mm", "5 mm"));
}

#[test]
fn the_tool_assistant_makes_a_v_bit() {
    let mut h = lab();
    h.app.start("cam.tool.define");
    h.frames(2);
    // Step one: the kinds as tiles; the V-bit's opens its shape.
    for name in ["Flat end mill", "Ball end mill", "Drill", "V-bit"] {
        assert!(h.widget(name).is_some(), "{name}");
    }
    h.step(&json!({"widget": "V-bit"})).unwrap();
    h.frames(2);
    let Some(Form::Tool(f)) = &mut h.app.manufacture.form else { panic!("no tool form") };
    assert_eq!((f.kind, f.step), (3, 1));
    assert!(h.widget("Point angle (α)").is_some() && h.widget("Tip diameter (d)").is_some());
    if let Some(Form::Tool(f)) = &mut h.app.manufacture.form {
        (f.tip_angle, f.tip_diameter, f.length) = ("60°".into(), "0.2 mm".into(), "40".into());
    }
    h.step(&json!({"widget": "Next >"})).unwrap();
    h.frames(1);
    assert!(h.widget("Spindle (rev/min)").is_some(), "step three: the cutting data");
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    // It is in the machine's library as a V-bit, and opens again as one.
    let library = library_listing(&mut h.app, true);
    let tool = &library["tools"][0];
    assert_eq!(
        (tool["kind"].as_str(), tool["tip_angle"].as_f64(), tool["tip_diameter"].as_f64(), tool["length"].as_f64()),
        (Some("v_bit"), Some(60.0), Some(0.2), Some(40.0))
    );
    let Some(Form::Tool(again)) = tool_form(tool, true) else { panic!("no form") };
    assert_eq!((again.kind, again.tip_angle.as_str(), again.step), (3, "60", 1));
}

#[test]
fn a_tool_s_data_in_a_material_fills_the_operations_of_a_setup_of_it() {
    let mut h = sketched();
    // A material is added in the Tool Library window.
    h.app.manufacture.tools_open = true;
    h.app.manufacture.new_material = "MDF".into();
    h.frames(2);
    h.step(&json!({"widget": "Add material"})).unwrap();
    h.frames(2);
    assert_eq!(library_listing(&mut h.app, true)["materials"], json!(["MDF"]));
    // A tool: its own data, and a faster feed in MDF; switching material keeps what each has.
    fill(&mut h, "cam.tool.define", |f| {
        let Form::Tool(tool) = f else { panic!("not a tool form") };
        (tool.spindle_rpm, tool.feed) = ("12000".into(), "900".into());
        tool.show_material("MDF");
        assert!(tool.spindle_rpm.is_empty() && tool.feed.is_empty(), "MDF has no data of its own yet");
        tool.feed = "1800".into();
        tool.show_material("");
        assert_eq!((tool.spindle_rpm.as_str(), tool.feed.as_str()), ("12000", "900"));
    });
    let library = library_listing(&mut h.app, true);
    assert_eq!(library["tools"][0]["cutting"], json!([{"material": "MDF", "feed": 1800.0}]));
    // Opened again, the tool has its MDF data; its cutting data step offers the material.
    let Some(Form::Tool(mut again)) = tool_form(&library["tools"][0], true) else { panic!("no form") };
    again.show_material("mdf");
    assert_eq!((again.feed.as_str(), again.chip_load.as_str()), ("1800", "0.075"));
    again.step = 2;
    h.app.manufacture.form = Some(Form::Tool(again));
    h.frames(2);
    assert!(h.widget("Material").is_some() && h.widget("MDF  (has data)").is_some());
    let Some(Form::Tool(shown)) = &h.app.manufacture.form else { panic!("no tool form") };
    assert_eq!(shown.material, "MDF", "kept under the spelling it has");
    h.app.manufacture.form = None;
    // A setup of MDF: a new operation's form is filled with the tool's data in it.
    fill(&mut h, "cam.setup.create", |f| {
        let Form::Setup(setup) = f else { panic!("not a setup form") };
        setup.material = "MDF".into();
    });
    assert_eq!(h.app.session.doc.manufacture.setups[0].material.as_deref(), Some("MDF"));
    h.app.start("cam.op.pocket");
    h.frames(2);
    let Some(Form::Op(op)) = &h.app.manufacture.form else { panic!("no operation form") };
    assert_eq!((op.spindle_rpm.as_str(), op.feed.as_str()), ("12000", "1800"));
    // The setup opens again with its material, and can be set back to none.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Setup(mut setup)) = setup_form(&listed["setups"][0]) else { panic!("no setup form") };
    assert_eq!(setup.material, "MDF");
    setup.material.clear();
    h.app.manufacture.form = Some(Form::Setup(setup));
    apply(&mut h.app);
    assert_eq!(h.app.session.doc.manufacture.setups[0].material, None);
}

#[test]
fn a_contour_form_sets_tabs_a_finishing_pass_and_its_passes() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.contour");
    h.frames(2);
    // Tabs are asked for with a tick; their rows follow it.
    assert!(h.widget("Hold the part").is_some() && h.widget("Finish allowance").is_some() && h.widget("Passes").is_some());
    assert!(h.widget("Tab count").is_none());
    let Some(Form::Op(f)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    (f.bottom, f.passes, f.finish, f.tabs, f.tab_height) = ("-6".into(), "3".into(), "0.5 mm".into(), true, "2 mm".into());
    h.frames(2);
    assert!(h.widget("Tab count").is_some() && h.widget("Tab width").is_some());
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = h.app.session.doc.manufacture.setups[0].operations[0].clone();
    assert_eq!(op.passes, Some(3));
    let solidcraft_engine::doc::manufacture::CamOperationKind::Contour { tabs: Some(tabs), finish, .. } = &op.kind else { panic!("no tabs") };
    assert_eq!((tabs.count, tabs.width.as_str(), tabs.height.as_str(), finish.as_deref()), (4, "6 mm", "2 mm", Some("0.5 mm")));
    // Edited, the form shows them; emptied and unticked, they are gone again.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let setup = listed["setups"][0].clone();
    let Some(Form::Op(mut f)) = edit_form(&h.app, &setup, &setup["operations"][0]) else { panic!("no form") };
    assert_eq!((f.passes.as_str(), f.tabs, f.tab_count.as_str(), f.tab_height.as_str(), f.finish.as_str()), ("3", true, "4", "2 mm", "0.5 mm"));
    (f.passes, f.finish, f.tabs) = (String::new(), String::new(), false);
    h.app.manufacture.form = Some(Form::Op(f));
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = &h.app.session.doc.manufacture.setups[0].operations[0];
    assert!(op.passes.is_none());
    assert!(matches!(&op.kind, solidcraft_engine::doc::manufacture::CamOperationKind::Contour { tabs: None, finish: None, .. }));
    // A number of passes that is not one says so.
    let Some(Form::Op(mut f)) = edit_form(&h.app, &setup, &setup["operations"][0]) else { panic!("no form") };
    f.passes = "two".into();
    assert!(command(&Form::Op(f)).is_err());
}

#[test]
fn a_simulated_program_plays_with_the_tool_cutting_the_stock() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    let top = solidcraft_engine::geom::Vec3::new(30.0, 20.0, 10.0);
    h.app.session.selection = crate::dialogs::face_sel(&h.app.session, top).into_iter().collect();
    fill(&mut h, "cam.op.contour", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        op.side = 1;
    });
    h.frames(2);
    h.app.start("cam.simulate");
    h.frames(2);
    let simulated = h.app.manufacture.stock.clone().expect("the simulated stock");
    // Under the simulation: play it; it starts from the uncut block, the tool drawn where it is,
    // rapids at the machine's rate from the preferences.
    h.app.preferences.cam_rapid_rate = 8000.0;
    h.frames(1);
    h.step(&json!({"widget": "Play the program"})).unwrap();
    h.frames(2);
    let play = h.app.manufacture.play.as_ref().expect("playing");
    assert!(play.playing && play.t < play.data.playback.minutes);
    assert_eq!(play.data.playback.rapid_rate, 8000.0);
    assert!(h.widget("Rapids").is_some());
    assert!(h.widget("Pause").is_some() && h.widget("Contour1  T1   X 0  Y 0  Z 0   rapid").is_none(), "where the tool is, as it goes");
    // Halfway, less is cut than in the end; at the end, the simulation's stock is back.
    let total = play.data.playback.minutes;
    // On to where each cut ends: the next rapid up, one after the other.
    let mut last = -1.0;
    for _ in 0..2 {
        h.step(&json!({"widget": ">R"})).unwrap();
        h.frames(2);
        let play = h.app.manufacture.play.as_ref().unwrap();
        let k = play.data.playback.step_at(play.t).unwrap();
        let at = &play.data.playback.steps[k];
        assert!(!play.playing && at.feed.is_none() && at.to.z > at.from.z && play.t > last, "{at:?} at {}", play.t);
        last = play.t;
    }
    let volume = |f: &StockField| f.z.iter().sum::<f64>();
    if let Some(play) = h.app.manufacture.play.as_mut() {
        play.seek = Some(total * 0.5);
        play.playing = false;
    }
    h.frames(2);
    let half = h.app.manufacture.stock.clone().expect("the stock so far");
    assert!(half.deviation.is_empty() && volume(&half) > volume(&simulated) + 1.0);
    if let Some(play) = h.app.manufacture.play.as_mut() {
        play.seek = Some(total);
    }
    h.frames(2);
    assert_eq!(h.app.manufacture.stock.as_ref().map(|f| &f.z), Some(&simulated.z));
    assert!(h.widget("Again").is_some(), "played to the end");
    // Back to the start: the block is whole again; Stop shows the simulation's result.
    if let Some(play) = h.app.manufacture.play.as_mut() {
        play.seek = Some(0.0);
    }
    h.frames(2);
    assert!(h.app.manufacture.stock.as_ref().is_some_and(|f| f.z.iter().all(|z| (z - f.z[0]).abs() < 1e-9) && volume(f) > volume(&half)));
    h.step(&json!({"widget": "Stop"})).unwrap();
    h.frames(2);
    assert!(h.app.manufacture.play.is_none() && h.app.manufacture.stock.as_ref() == Some(&simulated));
    // A change to the design stops it with the simulation.
    h.app.manufacture.play = None;
    h.step(&json!({"widget": "Play the program"})).unwrap();
    h.app.run("cam.op.edit", json!({"op": "Contour1", "side": "outside"})).unwrap();
    h.frames(2);
    assert!(h.app.manufacture.play.is_none());
}

#[test]
fn a_hull_goes_round_its_points() {
    let p = egui::pos2;
    let hull = convex_hull(vec![p(0.0, 0.0), p(2.0, 0.0), p(1.0, 1.0), p(2.0, 2.0), p(0.0, 2.0), p(1.0, 1.0), p(f32::NAN, 1.0)]);
    assert_eq!(hull.len(), 4);
    assert!(hull.contains(&p(2.0, 2.0)) && !hull.contains(&p(1.0, 1.0)));
    assert!(convex_hull(vec![p(0.0, 0.0), p(1.0, 1.0)]).len() == 2 && convex_hull(Vec::new()).is_empty());
}

#[test]
fn a_bore_form_mills_round_holes_on_a_helix() {
    let mut h = lab();
    h.app.run("sketch.create", json!({"plane": "XY", "name": "Holes"})).unwrap();
    h.app.run("sketch.circle.center", json!({"center": [20, 20], "radius": 8})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.bore");
    h.frames(2);
    // Its own rows: no stepdown or passes, a depth per turn and the direction.
    assert!(h.widget("BORE").is_some() && h.widget("Depth per turn").is_some() && h.widget("Climb").is_some());
    assert!(h.widget("Stepdown").is_none() && h.widget("Passes").is_none() && h.widget("Ramp angle").is_none());
    let Some(Form::Op(f)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    (f.bottom, f.pitch, f.climb) = ("-5".into(), "1.5 mm".into(), false);
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = h.app.session.doc.manufacture.setups[0].operations[0].clone();
    assert!(matches!(&op.kind, solidcraft_engine::doc::manufacture::CamOperationKind::Bore { pitch, climb: false, .. } if pitch == "1.5 mm"));
    // It opens again as it was, and the browser lists it with the drill's icon.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let setup = listed["setups"][0].clone();
    let Some(Form::Op(again)) = edit_form(&h.app, &setup, &setup["operations"][0]) else { panic!("no form") };
    assert_eq!((again.kind, again.pitch.as_str(), again.climb), (OpKind::Bore, "1.5 mm", false));
    h.frames(2);
    assert!(crate::browser::drawn_rows().into_iter().any(|r| r.0 == "Bore1"));
}

#[test]
fn a_chamfer_form_takes_its_depth_from_its_width() {
    let mut h = sketched();
    h.app.run("cam.tool.define", json!({"number": 1, "kind": "vbit", "diameter": 10, "tip_angle": 90, "spindle_rpm": 18000, "feed": 900})).unwrap();
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.chamfer");
    h.frames(2);
    // No bottom, stepdown or passes: the width, the tip offset, outside or inside.
    assert!(h.widget("CHAMFER").is_some() && h.widget("Chamfer width").is_some() && h.widget("Tip offset").is_some());
    assert!(h.widget("Bottom").is_none() && h.widget("Stepdown").is_none() && h.widget("Passes").is_none());
    let Some(Form::Op(f)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    (f.tool, f.width, f.side) = ("1".into(), "0.8 mm".into(), 1);
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = h.app.session.doc.manufacture.setups[0].operations[0].clone();
    assert!(matches!(
        &op.kind,
        solidcraft_engine::doc::manufacture::CamOperationKind::Chamfer { width, side: solidcraft_engine::cam::Side::Inside, .. } if width == "0.8 mm"
    ));
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let setup = listed["setups"][0].clone();
    let Some(Form::Op(again)) = edit_form(&h.app, &setup, &setup["operations"][0]) else { panic!("no form") };
    assert_eq!((again.kind, again.width.as_str(), again.side), (OpKind::Chamfer, "0.8 mm", 1));
}

#[test]
fn a_pocket_form_machines_the_rest_of_a_larger_tool() {
    let mut h = sketched();
    define_tool(&mut h);
    h.app.run("cam.tool.define", json!({"number": 2, "kind": "flat", "diameter": 12, "spindle_rpm": 10000, "feed": 800})).unwrap();
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 2, "bottom": -4, "stepover": 5})).unwrap();
    h.app.start("cam.op.pocket");
    h.frames(2);
    assert!(h.widget("Rest of tool").is_some() && h.widget("none: the whole pocket").is_some());
    let Some(Form::Op(f)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    (f.tool, f.bottom, f.rest_from) = ("1".into(), "-4".into(), "2".into());
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = h.app.session.doc.manufacture.setups[0].operations[1].clone();
    assert!(matches!(&op.kind, solidcraft_engine::doc::manufacture::CamOperationKind::Pocket { rest_from: Some(2), .. }));
    // Opened again it keeps the larger tool; emptied, the whole pocket is cut again.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let setup = listed["setups"][0].clone();
    let Some(Form::Op(mut again)) = edit_form(&h.app, &setup, &setup["operations"][1]) else { panic!("no form") };
    assert_eq!(again.rest_from, "2");
    again.rest_from.clear();
    h.app.manufacture.form = Some(Form::Op(again));
    apply(&mut h.app);
    assert!(matches!(
        &h.app.session.doc.manufacture.setups[0].operations[1].kind,
        solidcraft_engine::doc::manufacture::CamOperationKind::Pocket { rest_from: None, .. }
    ));
}

#[test]
fn an_adaptive_form_clears_a_pocket_at_a_steady_load() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.adaptive");
    h.frames(2);
    for row in [
        "ADAPTIVE",
        "Stepover (%)",
        "Helix angle",
        "Helix largest (%)",
        "Helix smallest (%)",
        "Rest of tool",
        "Finish allowance",
        "Stepdown",
        "Climb",
    ] {
        assert!(h.widget(row).is_some(), "{row}");
    }
    let Some(Form::Op(f)) = &mut h.app.manufacture.form else { panic!("no operation form") };
    (f.bottom, f.stepover_percent, f.helix_angle, f.climb) = ("-3".into(), "15 %".into(), "4 deg".into(), false);
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None);
    let op = h.app.session.doc.manufacture.setups[0].operations[0].clone();
    assert!(matches!(
        &op.kind,
        solidcraft_engine::doc::manufacture::CamOperationKind::Adaptive { stepover_percent, helix_angle, climb: false, .. } if *stepover_percent == 15.0 && helix_angle == "4 deg"
    ));
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let setup = listed["setups"][0].clone();
    let Some(Form::Op(again)) = edit_form(&h.app, &setup, &setup["operations"][0]) else { panic!("no form") };
    assert_eq!(
        (again.kind, again.stepover_percent.as_str(), again.helix_angle.as_str(), again.helix_max.as_str()),
        (OpKind::Adaptive, "15", "4 deg", "100")
    );
    h.frames(2);
    assert!(crate::browser::drawn_rows().into_iter().any(|r| r.0 == "Adaptive1"));
}

/// In the browser an operation shows its time, and is dragged before another of its setup
/// (onto its upper half) or after it (onto its lower half).
#[test]
fn browser_operations_show_their_time_and_are_dragged_into_order() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    for _ in 0..3 {
        h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": -3})).unwrap();
    }
    let order = |h: &Harness| h.app.session.doc.manufacture.setups[0].operations.iter().map(|o| o.name.clone()).collect::<Vec<_>>();
    assert_eq!(order(&h), ["Pocket1", "Pocket2", "Pocket3"]);
    h.frames(2);
    let minutes = h.app.session.execute("cam.list", &json!({})).unwrap()["setups"][0]["operations"][0]["toolpath"]["cut_minutes"].as_f64().unwrap();
    assert!(painted_texts(&mut h).contains(&minutes_text(minutes)), "the time beside the operation");
    let row = |name: &str| crate::browser::drawn_rows().into_iter().find(|r| r.0 == name).map(|r| r.1).unwrap();
    let (from, onto) = (row("Pocket3").center(), row("Pocket1"));
    h.step(&json!({"drag": [[from.x, from.y], [onto.center().x, onto.top() + 3.0]], "steps": 6})).unwrap();
    h.frames(2);
    assert_eq!(order(&h), ["Pocket3", "Pocket1", "Pocket2"], "before the first");
    let (from, onto) = (row("Pocket3").center(), row("Pocket2"));
    h.step(&json!({"drag": [[from.x, from.y], [onto.center().x, onto.bottom() - 3.0]], "steps": 6})).unwrap();
    h.frames(2);
    assert_eq!(order(&h), ["Pocket1", "Pocket2", "Pocket3"], "after the last");
}

/// The Manufacture folder's menu shows and hides toolpaths and stock and sets how toolpaths
/// are drawn; the same choices are in the Preferences' Graphics page.
#[test]
fn the_manufacture_folder_menu_and_preferences_set_the_display() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": -3})).unwrap();
    let showing = h.app.manufacture.show_setups;
    menu_action(&mut h.app, &json!({"do": "setups"}));
    assert_eq!(h.app.manufacture.show_setups, !showing);
    menu_action(&mut h.app, &json!({"do": "colour", "index": 2}));
    menu_action(&mut h.app, &json!({"do": "width", "index": 99}));
    assert_eq!(
        (h.app.preferences.cam_path_colour, h.app.preferences.cam_path_width),
        (2, PATH_WIDTHS.len() - 1),
        "an index past the end is the last"
    );
    let colours = folder_menu_items(&h.app).into_iter().find(|i| i.label == "Toolpath colour").unwrap();
    let checked: Vec<&str> = colours.children.iter().filter(|c| c.shortcut == "current").map(|c| c.label.as_str()).collect();
    assert_eq!(checked, ["white"]);
    h.app.prefs_window.open = true;
    h.app.prefs_window.section = crate::prefs::PAGE_GRAPHICS;
    h.frames(3);
    for row in ["Toolpath colour", "Toolpath width", "Panel fade", "Simulation detail", "Rapid rate"] {
        assert!(h.widget(row).is_some(), "{row} in the Preferences");
    }
}

/// One click on an operation (or a setup) in the browser shows its form in the panel; a form
/// with changes stays until OK or Cancel, and the status line says so.
#[test]
fn a_click_in_the_browser_shows_that_operation_s_form() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    for bottom in [-3, -4] {
        h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": bottom})).unwrap();
    }
    h.frames(2);
    let click = |h: &mut Harness, name: &str| {
        let at = crate::browser::drawn_rows().into_iter().find(|r| r.0 == name).map(|r| r.1.center()).unwrap();
        h.step(&json!({"click": [at.x, at.y]})).unwrap();
        h.frames(2);
    };
    let editing = |h: &Harness| match &h.app.manufacture.form {
        Some(Form::Op(f)) => f.name.clone(),
        Some(Form::Setup(f)) => f.name.clone(),
        _ => String::new(),
    };
    click(&mut h, "Pocket1");
    assert_eq!(editing(&h), "Pocket1");
    click(&mut h, "Pocket2");
    assert_eq!(editing(&h), "Pocket2", "the panel follows the click");
    if let Some(Form::Op(f)) = &mut h.app.manufacture.form {
        f.bottom = "-5 mm".into();
    }
    h.frames(1);
    click(&mut h, "Pocket1");
    assert_eq!(editing(&h), "Pocket2", "a form with changes stays");
    assert!(h.app.status.as_ref().is_some_and(|s| s.0.contains("OK keeps them")), "{:?}", h.app.status);
    h.app.manufacture.form = None;
    h.frames(1);
    click(&mut h, "Setup1");
    assert_eq!(editing(&h), "Setup1", "a setup shows its own form");
}

/// One operation simulates on the stock the operations before it leave: from its form as it
/// stands (not applied; the result goes when the form closes), from its browser menu, and a
/// setup without its hidden operations; what plays is what was simulated.
#[test]
fn an_operation_simulates_on_what_the_ones_before_it_leave() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    h.app.run("sketch.create", json!({"plane": "XY", "offset": 10, "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    for bottom in [-2, -4] {
        h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": bottom})).unwrap();
    }
    let ids: Vec<u64> = h.app.session.doc.manufacture.setups[0].operations.iter().map(|o| o.id).collect();
    let setup = h.app.session.doc.manufacture.setups[0].id;
    let title = |h: &Harness| h.app.manufacture.simulation.as_ref().map(|s| s.0[0].clone()).unwrap_or_default();
    // From the form, deeper than the operation is: simulated so, the design untouched.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Op(mut form)) = edit_form(&h.app, &listed["setups"][0], &listed["setups"][0]["operations"][1]) else { panic!("no form") };
    form.bottom = "-6 mm".into();
    h.app.manufacture.form = Some(Form::Op(form));
    h.app.manufacture.open = true;
    h.frames(2);
    // What the form's Simulate, beside OK and Cancel, asks for.
    answer(&mut h.app, Asked { simulate_form: true, ..Default::default() });
    h.frames(1);
    assert_eq!(title(&h), "Simulated: Pocket2 with the form's values (not applied yet)", "{:?} {:?}", h.app.manufacture.error, h.app.status);
    assert_eq!(h.app.session.doc.manufacture.setups[0].operations[1].bottom.as_deref(), Some("-4"));
    assert!(h.app.manufacture.form.is_some(), "the form stays open");
    start_play(&mut h.app, setup);
    assert_eq!(h.app.manufacture.play.as_ref().map(|p| p.data.ops.clone()), Some(vec!["Pocket2".to_string()]));
    h.app.manufacture.form = None;
    h.frames(1);
    assert!(h.app.manufacture.simulation.is_none(), "the form's simulation goes with it");
    // From the browser menu: the operation as it is, on what the first left.
    menu_action(&mut h.app, &json!({"do": "simulate", "setup": setup, "op": ids[1]}));
    assert_eq!(title(&h), "Simulated: Pocket2, on what the operations before it leave");
    // The setup, with the first operation hidden in the browser.
    h.app.manufacture.hidden.insert(ids[0]);
    simulate_setup(&mut h.app, setup);
    assert_eq!(title(&h), "Simulated: Setup1, 1 of 2 operations (hidden ones are left out)");
}

/// A program whose rapids run into material is written only once that is confirmed; a clean
/// one goes straight to its file.
#[test]
fn rapids_into_material_are_confirmed_before_a_program_is_written() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    h.app.run("sketch.create", json!({"plane": "XY", "offset": 10, "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": -3})).unwrap();
    let setup = h.app.session.doc.manufacture.setups[0].id;
    menu_action(&mut h.app, &json!({"do": "post", "setup": setup}));
    assert!(h.app.manufacture.confirm_post.is_none() && h.app.file_picker.is_some(), "clean: no question, straight to where to write it");
    h.app.file_picker = None;
    // Started 6 mm down without the cut above it: its rapids at the retract height are in the material.
    h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "top": -6, "bottom": -8})).unwrap();
    let late = h.app.session.doc.manufacture.setups[0].operations[1].id;
    menu_action(&mut h.app, &json!({"do": "post", "setup": setup, "op": late}));
    let asked = h.app.manufacture.confirm_post.clone().expect("the question");
    assert!(asked.collisions > 0 && asked.op.as_ref().map(|o| o.0) == Some(late));
    h.frames(2);
    assert!(h.widget("Write anyway").is_some());
    h.step(&json!({"widget": "Cancel"})).unwrap();
    h.frames(1);
    assert!(h.app.manufacture.confirm_post.is_none());
}

/// A setup's form sets the work offset its programs run in; its menu writes its setup sheet.
#[test]
fn a_setup_form_sets_the_work_offset_and_its_menu_offers_the_sheet() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |f| {
        let Form::Setup(setup) = f else { panic!("not a setup form") };
        setup.work_offset = 2;
    });
    assert_eq!(h.app.session.doc.manufacture.setups[0].work_offset, 56);
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Setup(again)) = setup_form(&listed["setups"][0]) else { panic!("no form") };
    assert_eq!(again.work_offset, 2, "opened again it shows G56");
    h.app.run("cam.op.pocket", json!({"sketch": "Pocket", "tool": 1, "bottom": -3})).unwrap();
    let setup = h.app.session.doc.manufacture.setups[0].id;
    let sheet = menu_items(&h.app, setup, None).into_iter().find(|i| i.label == "Setup Sheet…").expect("the sheet in the menu");
    assert!(sheet.enabled);
}

/// A setup's menu makes the setup for the part turned over, its stock what the first leaves;
/// a setup's form chooses where its stock comes from.
#[test]
fn a_setup_turned_over_takes_its_stock_from_the_first() {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |f| {
        let Form::Setup(setup) = f else { panic!("not a setup form") };
        setup.work_offset = 1;
    });
    let first = h.app.session.doc.manufacture.setups[0].id;
    menu_action(&mut h.app, &json!({"do": "turn", "setup": first}));
    let setups = &h.app.session.doc.manufacture.setups;
    assert_eq!(setups.len(), 2, "{:?}", h.app.manufacture.error);
    let turned = &setups[1];
    assert_eq!((turned.axis.z, turned.stock_from, turned.work_offset), (-1.0, Some(first), 55));
    // Its form shows where the stock comes from; set back to its own box, it is.
    let listed = h.app.session.execute("cam.list", &json!({})).unwrap();
    let Some(Form::Setup(mut form)) = setup_form(&listed["setups"][1]) else { panic!("no form") };
    assert_eq!(form.stock_from, first.to_string());
    form.stock_from.clear();
    h.app.manufacture.form = Some(Form::Setup(form));
    apply(&mut h.app);
    assert_eq!(h.app.session.doc.manufacture.setups[1].stock_from, None);
}

/// An operation form's rows kept as the default of its kind start the next form of that kind,
/// its geometry and heights not; forgotten, the built-in ones start it again.
#[test]
fn an_operation_form_kept_as_default_starts_the_next_one() {
    let mut h = sketched();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    h.app.start("cam.op.pocket");
    let built_in = match &h.app.manufacture.form {
        Some(Form::Op(f)) => (f.stepdown.clone(), f.climb, f.spindle_rpm.clone()),
        _ => panic!("no operation form"),
    };
    if let Some(Form::Op(f)) = &mut h.app.manufacture.form {
        (f.stepdown, f.climb, f.spindle_rpm, f.bottom, f.ramp_angle) = ("1.5 mm".into(), false, "9000".into(), "-7".into(), "2 deg".into());
    }
    answer(&mut h.app, Asked { save_default: true, ..Default::default() });
    assert!(h.app.status.as_ref().is_some_and(|s| s.0.contains("2D Pocket")), "{:?}", h.app.status);
    h.app.manufacture.form = None;
    h.app.start("cam.op.pocket");
    let Some(Form::Op(f)) = h.app.manufacture.form.clone() else { panic!("no operation form") };
    assert_eq!((f.stepdown.as_str(), f.climb, f.spindle_rpm.as_str(), f.ramp_angle.as_str()), ("1.5 mm", false, "9000", "2 deg"));
    assert_ne!(f.bottom, "-7", "the depth is not kept");
    // Another kind keeps its own; forgotten, the built-in defaults are back.
    h.app.start("cam.op.contour");
    assert!(matches!(&h.app.manufacture.form, Some(Form::Op(c)) if c.climb));
    h.app.start("cam.op.pocket");
    answer(&mut h.app, Asked { forget_default: true, ..Default::default() });
    h.app.start("cam.op.pocket");
    let Some(Form::Op(f)) = h.app.manufacture.form.clone() else { panic!("no operation form") };
    assert_eq!((f.stepdown, f.climb, f.spindle_rpm), built_in);
}

/// Every text one frame paints, whole and in the parts a composed one is made of.
fn painted_texts(h: &mut Harness) -> std::collections::BTreeSet<String> {
    fn walk(shape: &egui::Shape, out: &mut std::collections::BTreeSet<String>) {
        match shape {
            egui::Shape::Text(t) => {
                let text = t.galley.job.text.trim();
                out.insert(text.to_string());
                for part in text.split(['›', '·', '•', '→']).flat_map(|p| p.split("  ")) {
                    out.insert(part.trim().to_string());
                }
            }
            egui::Shape::Vec(shapes) => shapes.iter().for_each(|s| walk(s, out)),
            _ => {}
        }
    }
    let mut texts = std::collections::BTreeSet::new();
    h.painted_frame().iter().for_each(|s| walk(&s.shape, &mut texts));
    texts
}

/// The CAM surfaces in `language`: the Manufacture panel with a setup, operations and a
/// simulation, the program playing, every operation's form, the setup form, the tool library
/// and the tool assistant's three steps; each surface's painted texts.
fn cam_surfaces(language: &str) -> Vec<(String, std::collections::BTreeSet<String>)> {
    let mut h = lab();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    h.app.run("sketch.create", json!({"plane": "XY", "offset": 10, "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    define_tool(&mut h);
    fill(&mut h, "cam.setup.create", |_| {});
    fill(&mut h, "cam.op.pocket", |f| {
        let Form::Op(op) = f else { panic!("not an operation form") };
        op.bottom = "-3 mm".into();
    });
    h.app.preferences.language = language.into();
    h.app.start("cam.list");
    h.frames(2);
    let mut seen = vec![("panel".to_string(), painted_texts(&mut h))];
    h.app.start("cam.simulate");
    h.frames(3);
    seen.push(("simulation".into(), painted_texts(&mut h)));
    if let Some(at) = h.widget(crate::i18n::Locale::for_language(language).label("Play the program")) {
        h.step(&json!({"click": at})).unwrap();
        h.frames(2);
        seen.push(("playback".into(), painted_texts(&mut h)));
        h.app.manufacture.play = None;
    }
    for id in
        ["cam.op.face", "cam.op.contour", "cam.op.pocket", "cam.op.adaptive", "cam.op.chamfer", "cam.op.bore", "cam.op.drill", "cam.setup.create"]
    {
        h.app.start(id);
        h.frames(2);
        seen.push((id.to_string(), painted_texts(&mut h)));
        h.app.manufacture.form = None;
    }
    h.app.start("cam.library.list");
    h.frames(2);
    seen.push(("tool library".into(), painted_texts(&mut h)));
    h.app.start("cam.tool.define");
    for step in 0..3 {
        if let Some(Form::Tool(f)) = &mut h.app.manufacture.form {
            f.step = step;
        }
        h.frames(2);
        seen.push((format!("tool step {step}"), painted_texts(&mut h)));
    }
    seen
}

/// The CAM surfaces paint no English text that their language has no translation of.
#[test]
fn cam_surfaces_speak_the_chosen_language() {
    type Messages = std::collections::BTreeMap<String, String>;
    let english: Messages = serde_json::from_str(include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/locales/en.json"))).unwrap();
    for (language, catalog) in [
        ("de", include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/locales/de.json"))),
        ("ja", include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/locales/ja.json"))),
        ("zh", include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/locales/zh.json"))),
    ] {
        let messages: Messages = serde_json::from_str(catalog).unwrap();
        // English where the catalog has nothing, or something else, for one of the text's ids.
        let english_left = |text: &str| english.iter().any(|(id, en)| en == text && !en.contains('{') && messages.get(id) != Some(en));
        let leaks: Vec<String> = cam_surfaces(language)
            .into_iter()
            .flat_map(|(surface, texts)| texts.into_iter().filter(|t| english_left(t)).map(move |t| format!("{surface}: {t:?}")))
            .collect();
        assert!(leaks.is_empty(), "English left on the CAM surfaces in {language}:\n{}", leaks.join("\n"));
    }
}

/// The heights picture stands left of the height rows' labels, each height's line level with
/// its row and ending before its label, in German's longer labels too, and already while the
/// form still misses something.
#[test]
fn the_heights_picture_lines_up_with_the_height_rows() {
    for language in ["en", "de"] {
        let mut h = lab();
        h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
        h.app.run("sketch.create", json!({"plane": "XY", "offset": 10, "name": "Pocket"})).unwrap();
        h.app.run("sketch.rectangle.two_point", json!({"p0": [10, 10], "p1": [40, 30]})).unwrap();
        h.app.run("sketch.finish", json!({})).unwrap();
        define_tool(&mut h);
        fill(&mut h, "cam.setup.create", |_| {});
        h.app.preferences.language = language.into();
        let locale = crate::i18n::Locale::for_language(language);
        for (id, bottom) in [("cam.op.pocket", ""), ("cam.op.adaptive", "-3 mm")] {
            h.app.start(id);
            if let Some(Form::Op(op)) = &mut h.app.manufacture.form {
                op.bottom = bottom.into();
            }
            h.frames(3);
            let mut last = f32::NEG_INFINITY;
            for (line, label) in [("Clearance", "Clearance"), ("Retract", "Retract"), ("Top", "Top"), ("Bottom", "Bottom")] {
                let end = crate::scenario::handle_at(&format!("cam_height:{line}")).unwrap_or_else(|| panic!("{language} {id}: no {line} line"));
                let [x, y] = h.widget(locale.manufacture_label(label)).unwrap();
                let row = egui::pos2(x as f32, y as f32);
                assert!(
                    (end.y - row.y).abs() < 3.0 && end.x < row.x && end.y > last,
                    "{language} {id} {line}: line ends at {end:?}, label at {row:?}"
                );
                last = end.y;
            }
            h.app.manufacture.form = None;
        }
    }
}
