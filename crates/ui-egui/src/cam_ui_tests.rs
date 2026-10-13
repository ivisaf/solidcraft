use serde_json::json;

use super::*;
use crate::scenario::Harness;

/// A 40 × 30 rectangle sketched on XY.
fn sketched() -> Harness {
    let mut h = Harness::new();
    h.app.run("sketch.create", json!({"plane": "XY", "name": "Pocket"})).unwrap();
    h.app.run("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]})).unwrap();
    h.app.run("sketch.finish", json!({})).unwrap();
    h
}

/// Starts `id` from the toolbar, lets `edit` fill the form in, and presses OK.
fn fill(h: &mut Harness, id: &str, edit: impl FnOnce(&mut Form)) {
    h.app.start(id);
    assert!(h.app.manufacture.open, "{id} opens the Manufacture window");
    edit(h.app.manufacture.form.as_mut().unwrap_or_else(|| panic!("{id} has no form")));
    apply(&mut h.app);
    assert_eq!(h.app.manufacture.error, None, "{id}");
    assert!(h.app.manufacture.form.is_none(), "the form closes when its command succeeds");
}

#[test]
fn toolbar_forms_make_a_job_and_its_toolpath_is_drawn() {
    let mut h = sketched();
    fill(&mut h, "cam.tool.define", |_| {});
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
    assert!(h.widget("Show toolpaths").is_some() && h.widget("Pocket1  (pocket, T1)").is_some(), "the window lists the operation");
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
    fill(&mut h, "cam.tool.define", |_| {});
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
    let mut h = Harness::new();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    fill(&mut h, "cam.tool.define", |_| {});
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
    fill(&mut h, "cam.tool.define", |_| {});
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
    let mut h = Harness::new();
    h.app.run("solid.box", json!({"length": 60, "width": 40, "height": 10})).unwrap();
    fill(&mut h, "cam.tool.define", |_| {});
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
    assert!(h.widget("Does not cut into the part").is_some() && h.widget("Simulate").is_some() && h.widget("Show simulated stock").is_some());
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
fn manufacture_tab_shows_every_cam_command_with_a_drawn_icon() {
    assert!(crate::toolbar::TABS.contains(&"MANUFACTURE"));
    let specs = solidcraft_engine::command_specs();
    let on_tab: Vec<_> = specs.iter().filter(|c| c.tab == "MANUFACTURE").collect();
    assert_eq!(on_tab.len(), 10, "cam.op.edit and the removals are reached from the window");
    for c in on_tab {
        assert!(crate::toolbar::panels("MANUFACTURE").contains(&c.panel), "{}: panel {}", c.id, c.panel);
        assert!(c.icon.starts_with("cam_"), "{}", c.id);
        let mut h = Harness::new();
        h.app.start(c.id);
        assert!(h.app.manufacture.open, "{} opens the window", c.id);
        h.frames(2);
    }
}
