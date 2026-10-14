use super::*;
use serde_json::json;

fn card(data: Value) -> Value {
    json!({"schema_version":1,"designation":"Original synthetic fixture","family":"wood",
        "sources":[{"id":"test","title":"Analytic synthetic data, not a material specification","kind":"illustrative","license":"MIT OR Apache-2.0"}],
        "properties":[{"data":data,"source_id":"test"}]})
}
fn isotropic(e: f64, nu: f64) -> Value {
    json!({"kind":"elasticity","model":{"symmetry":"isotropic","young_modulus_pa":e,"poisson_ratio":nu}})
}
fn orthotropic(e: [f64; 3], nu: [f64; 3]) -> Value {
    json!({"kind":"elasticity","model":{"symmetry":"orthotropic","axes":"wood_lrt",
        "young_moduli_pa":e,"shear_moduli_pa":[4e9,4e9,4e9],"poisson_ratios":nu}})
}

#[test]
fn isotropic_moduli_match_closed_form_and_auxetic_values_are_valid() {
    let r = validate_card(&card(isotropic(12e9, 0.25))).unwrap();
    assert_eq!(r.isotropic_moduli, Some(IsotropicModuli { shear_pa: 4.8e9, bulk_pa: 8e9 }));
    let r = validate_card(&card(isotropic(12e9, -0.5))).unwrap();
    assert_eq!(r.isotropic_moduli, Some(IsotropicModuli { shear_pa: 12e9, bulk_pa: 2e9 }));
    for nu in [-2., -1., 0.5, 0.6] {
        assert!(validate_card(&card(isotropic(12e9, nu))).is_err());
    }
    for e in [0., -1., 1e16] {
        assert!(validate_card(&card(isotropic(e, 0.25))).is_err());
    }
    assert!(Elasticity::Isotropic { young_modulus_pa: f64::NAN, poisson_ratio: 0.25 }.validate().is_err());
    assert!(Elasticity::Isotropic { young_modulus_pa: 1e-309, poisson_ratio: 0.25 }.validate().is_err());
}

#[test]
fn orthotropy_uses_reciprocity_and_full_positive_definiteness() {
    // With equal E and nu=.25 this is the same normal compliance as isotropy.
    assert!(validate_card(&card(orthotropic([12e9; 3], [0.25; 3]))).is_ok());
    // A general orthotropic ratio may exceed .5: reciprocal nu21 is .08 here.
    assert!(validate_card(&card(orthotropic([10e9, 1e9, 1e9], [0.8, 0., 0.]))).is_ok());
    // Every 2x2 principal minor is positive; the full determinant is negative.
    assert!(validate_card(&card(orthotropic([12e9; 3], [0.6; 3]))).is_err());
    assert!(validate_card(&card(orthotropic([10e9, 1e9, 1e9], [4., 0., 0.]))).is_err());
    assert!(validate_card(&card(orthotropic([0., 1e9, 1e9], [0.; 3]))).is_err());
    let mut bad = card(orthotropic([12e9; 3], [0.25; 3]));
    bad["properties"][0]["data"]["model"]["shear_moduli_pa"] = json!([4e9, 0., 4e9]);
    assert!(validate_card(&bad).is_err());
    // Scaling all moduli changes units of compliance, not its stability verdict.
    assert!(validate_card(&card(orthotropic([12.; 3], [0.25; 3]))).is_ok());
    assert!(validate_card(&card(orthotropic([1e-300, 1e15, 1.], [0.; 3]))).is_ok());
    assert!(validate_card(&card(orthotropic([1e-309, 1., 1.], [0.; 3]))).is_err());
}

#[test]
fn conditions_missing_values_and_source_claims_remain_explicit() {
    let mut value = card(json!({"kind":"density","kg_m3":600.}));
    let r = validate_card(&value).unwrap();
    assert_eq!(r.unspecified_conditions, vec!["density"]);
    assert_eq!(r.source_review_required, vec!["test"]); // Even an explicit license is unverified.
    assert!(!r.analysis_solver_available && !r.rendering_properties_available);
    assert!(r.isotropic_moduli.is_none());
    value["properties"][0]["conditions"] = json!({"temperature_k":293.15,"moisture_dry_basis_kg_kg":1.5,"state":"synthetic wet specimen"});
    assert!(validate_card(&value).unwrap().unspecified_conditions.is_empty());
    value["family"] = json!("polymer");
    value["properties"][0]["conditions"]["moisture_dry_basis_kg_kg"] = json!(150.);
    value["properties"][0]["conditions"]["strain_rate_per_s"] = json!(1.);
    assert!(validate_card(&value).unwrap().unspecified_conditions.is_empty());
    value["properties"] = json!([]);
    let r = validate_card(&value).unwrap();
    assert!(r.property_kinds.is_empty() && r.isotropic_moduli.is_none());
    let parsed: MaterialCard = serde_json::from_value(value.clone()).unwrap();
    assert_eq!(serde_json::from_value::<MaterialCard>(serde_json::to_value(&parsed).unwrap()).unwrap(), parsed);
}

#[test]
fn thermal_properties_accept_negative_expansion_but_not_negative_conductivity() {
    assert!(validate_card(&card(json!({"kind":"thermal_expansion","per_k":[-1e-6,0.,2e-6],"axes":"material123"}))).is_ok());
    assert!(validate_card(&card(json!({"kind":"thermal_conductivity","w_m_k":[1.,0.,1.],"axes":"material123"}))).is_err());
    assert!(validate_card(&card(json!({"kind":"specific_heat","j_kg_k":500.}))).is_ok());
    for kind in ["tensile_strength", "compressive_strength", "yield_strength", "shear_strength"] {
        assert!(validate_card(&card(json!({"kind":kind,"pa":1e8}))).is_ok());
        assert!(validate_card(&card(json!({"kind":kind,"pa":-1.}))).is_err());
    }
}

#[test]
fn hostile_units_sources_fields_and_budgets_are_rejected() {
    let valid = card(isotropic(12e9, 0.25));
    let mut bad = valid.clone();
    bad["properties"][0]["data"]["model"]["young_modulus_mpa"] = json!(12.);
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["properties"][0]["source_id"] = json!("unknown");
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["properties"] = json!([valid["properties"][0], valid["properties"][0]]);
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["sources"] = json!([valid["sources"][0], valid["sources"][0]]);
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["schema_version"] = json!(2);
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["designation"] = json!("x".repeat(257));
    assert!(validate_card(&bad).is_err());
    let mut bad = valid.clone();
    bad["properties"][0]["conditions"] = json!({"moisture_dry_basis_kg_kg":-1.});
    assert!(validate_card(&bad).is_err());
    assert!(validate_card(&json!(vec![Value::Null; 33])).is_err());
    assert!(validate_card(&json!("x".repeat(256 * 1024 + 1))).is_err());
    let mut deep = Value::Null;
    for _ in 0..14 {
        deep = json!([deep]);
    }
    assert!(validate_card(&deep).is_err());
}
