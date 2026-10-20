//! Autosave ticks and the "Recover unsaved design?" window shown at launch.

use serde_json::json;
use solidcraft_engine::recovery::{self, Autosaver, Entry};

use crate::SolveApp;
use crate::ui_kit;

/// A command that ran at least this long counts as a big operation: autosave right after it.
pub const BIG_OPERATION_MS: f64 = 1500.0;

/// Autosave (when due, or after a big operation).
pub fn tick(app: &mut SolveApp) {
    let big = std::mem::take(&mut app.big_operation);
    let Some(a) = app.autosave.as_mut() else { return };
    if let Err(e) = a.tick(&app.session, big) {
        let msg = format!("autosave failed: {e}");
        if app.status.as_ref().is_none_or(|(s, _, _)| *s != msg) {
            app.set_status(msg, true);
        }
    }
}

/// Start autosaving into `dir` and queue the designs that crashed apps left there.
pub fn start(app: &mut SolveApp, dir: &std::path::Path, interval: std::time::Duration) {
    match Autosaver::new(dir, interval) {
        Ok(a) => {
            app.recoverable = recovery::orphans(dir);
            app.autosave = Some(a);
        }
        Err(e) => app.set_status(format!("autosave is off: {e}"), true),
    }
}

/// The app is closing: keep unsaved work for the next launch, drop everything else.
pub fn close(app: &mut SolveApp) {
    if let Some(a) = app.autosave.take() {
        a.close(&app.session);
    }
}

fn ago(secs: u64) -> String {
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(secs);
    let d = now.saturating_sub(secs);
    match d {
        0..=59 => crate::i18n::tr("recovery.just_now").to_string(),
        60..=3599 => crate::i18n::format("recovery.min_ago", &[("count", &(d / 60).to_string())]),
        3600..=86_399 => crate::i18n::format("recovery.hours_ago", &[("count", &(d / 3600).to_string())]),
        _ => crate::i18n::format("recovery.days_ago", &[("count", &(d / 86_400).to_string())]),
    }
}

pub fn show(app: &mut SolveApp, ctx: &egui::Context) {
    if app.recoverable.is_empty() {
        return;
    }
    let dir = app.autosave.as_ref().map(|a| a.dir.clone()).or_else(recovery::default_dir);
    let mut act: Option<(Entry, bool)> = None;
    let (closed, action) = ui_kit::Window::new(egui::Id::new("sc_recover_prompt"), "restore", crate::i18n::tr_label("Recover unsaved design?"))
        .actions("", crate::i18n::tr_label("Later"), None)
        .show(ctx, true, |ui| {
            ui_kit::text(
                ui,
                crate::i18n::tr_label("SolidCraft closed before these designs were saved. Their last autosave can be opened."),
                ui_kit::Text::Report("intro", false),
            );
            for (index, e) in app.recoverable.iter().take(20).enumerate() {
                ui_kit::divider(ui, &format!("entry:{index}"));
                ui_kit::label_actions(ui, &e.name, |ui| {
                    if ui_kit::button(ui, crate::i18n::tr_label("Recover"), ui_kit::Button::Normal).clicked() {
                        act = Some((e.clone(), true));
                    }
                    if ui_kit::button(ui, crate::i18n::tr_label("Discard"), ui_kit::Button::Normal).clicked() {
                        act = Some((e.clone(), false));
                    }
                });
                let file = e.path.clone().unwrap_or_else(|| crate::i18n::tr("recovery.never_saved").to_string());
                let detail =
                    crate::i18n::format("recovery.detail", &[("file", &file), ("features", &e.features.to_string()), ("when", &ago(e.saved_at))]);
                ui_kit::text(ui, &detail, ui_kit::Text::Note(&format!("detail:{index}")));
            }
        });
    let later = closed || action != ui_kit::Action::None;
    let dir_s = dir.map(|d| d.to_string_lossy().to_string());
    if let Some((e, recover)) = act {
        app.recoverable.retain(|x| x.id != e.id);
        if recover {
            if app.session.is_dirty() {
                app.set_status("save or discard the open design's changes before recovering another", true);
                app.recoverable.insert(0, e);
                return;
            }
            // Rebuilt off the window thread like any other file; once it is in, it is
            // autosaved under this app's entry before the old entry goes.
            let kind = crate::file_load::Kind::Recover { id: e.id.clone(), dir: dir_s };
            let _ = crate::file_load::begin_request(app, crate::file_load::Request::new(&e.name, kind));
        } else {
            let _ = app.run("doc.recovery_discard", json!({"id": e.id, "dir": dir_s}));
        }
    }
    if later {
        app.recoverable.clear();
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;
    use solidcraft_engine::recovery::Entry;

    fn entry(id: &str, name: &str) -> Entry {
        Entry { id: id.into(), path: None, name: name.into(), saved_at: 0, features: 3 }
    }

    #[test]
    fn recovered_sessions_are_a_card_with_one_row_each_and_later_keeps_them_for_next_launch() {
        let mut h = crate::scenario::Harness::new();
        h.app.home.open = false;
        h.app.recoverable = vec![entry("a", "Bracket"), entry("b", "Housing")];
        h.frames(4);
        assert!(crate::scenario::handle_at("dialog:close").is_some(), "a kit card");
        assert!(h.widget("Recover").is_some() && h.widget("Discard").is_some() && h.widget("Later").is_some());
        assert!(crate::scenario::handle_at("dialog:cell:Bracket").is_some(), "one row per design");
        assert!(crate::scenario::handle_at("dialog:cell:Housing").is_some());
        h.step(&json!({"widget": "Later"})).unwrap();
        h.frames(2);
        assert!(h.app.recoverable.is_empty(), "Later hides the card for this launch");
    }
}
