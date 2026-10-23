//! The Manufacture windows' questions, in the kit's modal (a file of their own: the windows
//! themselves draw frames, which a modal's file may not).

use crate::SolveApp;
use crate::i18n::{format as tf, tr_manufacture as tm};
use crate::ui_kit;

use super::pick_post;

/// The question before a program whose rapids run into material is written.
pub(super) fn post_confirmation(app: &mut SolveApp, ctx: &egui::Context) {
    let Some(pending) = app.manufacture.confirm_post.clone() else { return };
    let mut lines = Vec::new();
    if pending.collisions > 0 {
        lines.push(tf("manufacture.rapids_before_post", &[("count", &pending.collisions.to_string())]));
    }
    if !pending.clamped.is_empty() {
        lines.push(tf("manufacture.clamps_before_post", &[("ops", &pending.clamped.join(", "))]));
    }
    let message = lines.join("\n\n");
    let (closed, action) = ui_kit::Confirm::new(
        egui::Id::new("sc_cam_post_confirm"),
        "status_warning",
        tm(if pending.collisions > 0 { "Rapid moves run into material" } else { "The tool runs into a clamp" }),
    )
    .tone(ui_kit::Tone::Warning)
    .actions(tm("Write anyway"), tm("Cancel"), None)
    .show(ctx, true, |ui| {
        ui_kit::text(ui, &message, ui_kit::Text::Report("rapids", true));
    });
    match action {
        ui_kit::Action::Ok => {
            app.manufacture.confirm_post = None;
            pick_post(app, pending.setup, pending.op, pending.post);
        }
        ui_kit::Action::Cancel => app.manufacture.confirm_post = None,
        _ if closed => app.manufacture.confirm_post = None,
        _ => {}
    }
}
