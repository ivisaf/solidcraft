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
fn manufacture_tab_shows_every_cam_command_with_a_drawn_icon() {
    assert!(crate::toolbar::TABS.contains(&"MANUFACTURE"));
    let specs = solidcraft_engine::command_specs();
    let on_tab: Vec<_> = specs.iter().filter(|c| c.tab == "MANUFACTURE").collect();
    assert_eq!(on_tab.len(), 9);
    for c in on_tab {
        assert!(crate::toolbar::panels("MANUFACTURE").contains(&c.panel), "{}: panel {}", c.id, c.panel);
        assert!(c.icon.starts_with("cam_"), "{}", c.id);
        let mut h = Harness::new();
        h.app.start(c.id);
        assert!(h.app.manufacture.open, "{} opens the window", c.id);
        h.frames(2);
    }
}
