//! Read-only validation of candidate engineering data; no assignment or simulation.
use super::CommandSpec;
use crate::params::bad;
use crate::{Result, Session};
use serde_json::{Value, json};

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("material.properties.validate", "Validate Physical Material Data", validate)
    .icon("material")
    .noundo()
    .doc_only()
    .params("card: schema_version 1 engineering material card in SI units. Returns numerical consistency, derived isotropic moduli, missing conditions and sources needing review. Read-only; does not assign/save properties or provide an analysis solver. See docs/physical-material-cards.md")];

fn validate(_: &mut Session, params: &Value) -> Result<Value> {
    let object = params
        .as_object()
        .filter(|o| o.len() == 1 && o.contains_key("card"))
        .ok_or_else(|| bad("material.properties.validate", "expected {card: ...}"))?;
    let card = object.get("card").ok_or_else(|| bad("material.properties.validate", "missing card"))?;
    let result = solidcraft_doc::physical_material::validate_card(card).map_err(|e| bad("material.properties.validate", e.to_string()))?;
    Ok(json!({"valid":true,"validation":result}))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn command_reports_analytic_moduli_and_refuses_unstable_data() {
        let mut s = Session::default();
        let mut card = json!({"schema_version":1,"designation":"Synthetic elastic fixture","family":"other",
            "sources":[{"id":"fixture","title":"Original analytic fixture","kind":"illustrative"}],
            "properties":[{"source_id":"fixture","data":{"kind":"elasticity","model":{
                "symmetry":"isotropic","young_modulus_pa":12e9,"poisson_ratio":0.25}}}]});
        let result = s.execute("material.properties.validate", &json!({"card":card})).unwrap();
        assert_eq!(result["validation"]["isotropic_moduli"], json!({"shear_pa":4.8e9,"bulk_pa":8e9}));
        assert_eq!(result["validation"]["source_review_required"], json!(["fixture"]));
        assert_eq!(result["validation"]["rendering_properties_available"], false);
        card["properties"][0]["data"]["model"]["poisson_ratio"] = json!(0.6);
        assert!(s.execute("material.properties.validate", &json!({"card":card})).is_err());
    }
    #[test]
    fn validation_preserves_document_history_selection_and_evaluation() {
        let mut s = Session::default();
        s.execute("solid.box", &json!({"length":10,"width":20,"height":5})).unwrap();
        s.execute("edit.undo", &json!({})).unwrap();
        let doc = s.doc.clone();
        let undo = s.undo.clone();
        let redo = s.redo.clone();
        let before = (s.is_dirty(), s.revision, s.evaluations, s.model.is_stale(), s.camera, s.selection.clone());
        let card = json!({"schema_version":1,"designation":"Synthetic empty card","family":"other","sources":[],"properties":[]});
        let result = s.execute("material.properties.validate", &json!({"card":card})).unwrap();
        assert_eq!(result["validation"]["analysis_solver_available"], false);
        for p in [json!({}), json!({"card":card,"assign":true}), json!({"card":{"schema_version":9}})] {
            assert!(s.execute("material.properties.validate", &p).is_err());
        }
        assert!(Arc::ptr_eq(&doc, &s.doc));
        assert_eq!(before, (s.is_dirty(), s.revision, s.evaluations, s.model.is_stale(), s.camera, s.selection.clone()));
        for (a, b) in [(&undo, &s.undo), (&redo, &s.redo)] {
            assert_eq!(a.len(), b.len());
            for (a, b) in a.iter().zip(b) {
                assert!(Arc::ptr_eq(&a.doc, &b.doc));
            }
        }
    }

    #[test]
    fn validation_does_not_settle_live_geometry() {
        let mut s = Session::default();
        for (id, p) in [
            ("sketch.create", json!({"plane":"XY"})),
            ("sketch.rectangle.two_point", json!({"p0":[5,5],"p1":[25,20]})),
            ("sketch.dimension", json!({"entities":["l1"],"value":20})),
            ("sketch.dimension", json!({"entities":["l2"],"value":15})),
            ("sketch.finish", json!({})),
            ("solid.extrude", json!({"distance":5})),
            ("sketch.edit", json!({"sketch":"Sketch1"})),
            ("sketch.drag", json!({"entity":"l1.start","from":[5,5],"to":[4,5],"live":true})),
        ] {
            s.execute(id, &p).unwrap();
        }
        assert!(s.model.is_stale());
        let doc = s.doc.clone();
        let before = (s.revision, s.evaluations, s.active_sketch, s.undo.len(), s.redo.len());
        let card = json!({"schema_version":1,"designation":"Synthetic","family":"other","sources":[],"properties":[]});
        s.execute("material.properties.validate", &json!({"card":card})).unwrap();
        assert!(s.execute("material.properties.validate", &json!({"card":false})).is_err());
        assert!(Arc::ptr_eq(&doc, &s.doc));
        assert!(s.model.is_stale());
        assert_eq!(before, (s.revision, s.evaluations, s.active_sketch, s.undo.len(), s.redo.len()));
    }
}
