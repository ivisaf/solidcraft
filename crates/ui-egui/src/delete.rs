//! Delete (the Delete and Backspace keys, and every menu's Delete item): whatever is selected in
//! the viewport or the browser goes in one undoable step through `selection.delete`. When it
//! takes other features with it or makes some fail, a confirmation lists them first.

use serde_json::{Value, json};

use crate::SolveApp;
use crate::ui_kit;

/// Longest list of names the confirmation shows before "and n more".
const MAX_LISTED: usize = 20;

/// A delete waiting for confirmation: its parameters and the dry run's report.
#[derive(Clone, Debug)]
pub struct Pending {
    pub params: Value,
    pub report: Value,
}

/// What Delete acts on now: the selection (bodies not locked), plus the components picked in
/// the browser (their occurrences).
pub fn selection_params(app: &SolveApp) -> Value {
    let items: Vec<&solidcraft_engine::Sel> =
        app.session.selection.iter().filter(|s| !matches!(s, solidcraft_engine::Sel::Body { name } if app.ui.locked_bodies.contains(name))).collect();
    let occurrences: Vec<u64> = app.tree.picked_components.iter().filter_map(|c| app.session.doc.occurrence_of(*c).map(|o| o.id)).collect();
    json!({ "items": items, "occurrences": occurrences, "canvases": app.tree.picked_canvases, "joints": app.tree.picked_joints })
}

fn is_empty(p: &Value) -> bool {
    ["items", "occurrences", "components", "canvases", "joints"].iter().all(|k| p.get(*k).and_then(Value::as_array).is_none_or(|a| a.is_empty()))
}

fn shared_occurrences(app: &SolveApp, params: &Value) -> bool {
    let explicit = params.get("occurrences").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_u64);
    let selected = params
        .get("items")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter(|item| item["type"] == "occurrence")
        .filter_map(|item| item["path"].as_array()?.last()?.as_u64());
    explicit.chain(selected).any(|id| app.session.occurrence_id_scope_count(id).is_ok_and(|n| n > 1))
}

/// Delete what is selected (the Delete key). A selected sketch dimension or constraint glyph
/// goes first, as before.
pub fn delete_selection(app: &mut SolveApp) {
    if crate::sketch3d::delete_selected(app) || crate::sketch_tools::delete_glyph(app) {
        return;
    }
    request(app, selection_params(app));
}

/// Delete `params` (as `selection.delete` takes them): at once, or after a confirmation when
/// other features go with it or would fail. The dry run that finds out evaluates the history
/// after what is deleted, so it may compute off the window thread (#2678).
pub fn request(app: &mut SolveApp, params: Value) {
    if params.get("joints").and_then(Value::as_array).is_some_and(|a| !a.is_empty())
        && (app.dialog.is_some() || app.tool.is_some() || app.computing_since().is_some() || app.file_load.busy())
    {
        return;
    }
    if is_empty(&params) {
        return;
    }
    let mut dry = params.clone();
    dry["dry_run"] = json!(true);
    app.check_delete(vec![("selection.delete".to_string(), dry)], crate::deferred::AfterCheck::Selection(params));
}

/// [`request`] once its dry run's report is in.
pub(crate) fn checked(app: &mut SolveApp, params: Value, report: Result<Value, String>) {
    let report = match report {
        Ok(r) => r,
        Err(e) => {
            app.set_status(e.replace("selection.delete: ", ""), true);
            return;
        }
    };
    let asked =
        params.get("items").and_then(Value::as_array).map_or(0, |a| a.iter().filter(|i| i["type"] == "feature" || i["type"] == "plane").count());
    let extra = report["deleted"].as_array().map_or(0, Vec::len) > asked;
    let fails = report["would_fail"].as_array().is_some_and(|a| !a.is_empty());
    let shared = shared_occurrences(app, &params);
    if extra || fails || shared {
        app.menu.confirm = Some(Pending { params, report });
    } else {
        run(app, &params);
    }
}

fn run(app: &mut SolveApp, params: &Value) {
    match app.run("selection.delete", params.clone()) {
        Ok(r) => {
            app.tree.picked_components.clear();
            app.tree.picked_canvases.clear();
            app.tree.picked_joints.clear();
            if let Some(s) = r["skipped"].as_array().and_then(|a| a.first()).and_then(Value::as_str) {
                app.set_status(format!("deleted; {s}"), false);
            }
        }
        Err(e) => app.set_status(e.replace("selection.delete: ", ""), true),
    }
}

/// Answer the confirmation (also `ui.confirm`): delete, or keep everything.
pub fn confirm(app: &mut SolveApp, accept: bool) -> bool {
    let Some(p) = app.menu.confirm.take() else { return false };
    if accept {
        run(app, &p.params);
    }
    true
}

/// The confirmation card.
pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    let Some(p) = app.menu.confirm.clone() else { return };
    let list = |k: &str| -> Vec<String> {
        p.report[k].as_array().map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect()).unwrap_or_default()
    };
    let (deleted, fails) = (list("deleted"), list("would_fail"));
    let shared = shared_occurrences(app, &p.params);
    let (closed, action) = ui_kit::Confirm::new(egui::Id::new("sc_delete_confirm"), "delete", crate::i18n::tr_label("Delete"))
        .tone(ui_kit::Tone::Danger)
        .actions("Delete", "Cancel", None)
        .show(ctx, true, |ui| {
            if shared {
                ui_kit::text(
                    ui,
                    crate::i18n::tr_label("This child is shared. Deleting it removes it from every copy of its parent."),
                    ui_kit::Text::Report("shared", false),
                );
            }
            for (key, heading, names, error) in
                [("deleted", "These features will be deleted:", &deleted, false), ("fails", "These features will fail:", &fails, true)]
            {
                if names.is_empty() {
                    continue;
                }
                ui_kit::text(ui, crate::i18n::tr_label(heading), ui_kit::Text::Report(key, error));
                for name in names.iter().take(MAX_LISTED) {
                    ui_kit::text(ui, &format!("  • {name}"), ui_kit::Text::Report(&format!("{key}:{name}"), error));
                }
                if names.len() > MAX_LISTED {
                    let more = (names.len() - MAX_LISTED).to_string();
                    ui_kit::text(
                        ui,
                        &crate::i18n::format("delete.and_more", &[("count", &more)]),
                        ui_kit::Text::Report(&format!("{key}:more"), error),
                    );
                }
            }
        });
    match action {
        ui_kit::Action::Ok => drop(confirm(app, true)),
        ui_kit::Action::Cancel => drop(confirm(app, false)),
        _ if closed => drop(confirm(app, false)),
        _ => {}
    }
}
