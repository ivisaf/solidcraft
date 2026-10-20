use serde::de::DeserializeOwned;
use serde_json::{Value, json};

use super::check::schema_keys;
use super::*;
use crate::Session;

fn parse<T: DeserializeOwned>(v: Value) -> Option<T> {
    serde_json::from_value(v).ok()
}

fn box_contract() -> TypedContract<BoxParams> {
    TypedContract::new()
}

fn messages(issues: &[Issue]) -> Vec<String> {
    issues.iter().map(|i| i.message.clone()).collect()
}

#[test]
fn expressions_accept_what_params_expr_accepts() {
    for v in [json!(5), json!(-2.5), json!("w / 2"), json!("  30 mm  "), json!(999_999_999.0)] {
        let want = crate::params::expr(&json!({ "v": v.clone() }), "v");
        assert_eq!(parse::<LengthExpr>(v.clone()).map(|e| e.0), want, "{v}");
        assert!(want.is_some(), "{v}");
    }
    let long = "x".repeat(4097);
    // The cap is in bytes: 2049 two-byte characters are too long (the schema's maxLength counts
    // characters, so it can't say this).
    let wide = "é".repeat(2049);
    for v in [json!(true), json!(null), json!(""), json!("   "), json!(1e9), json!(-1e9), json!([1]), json!({}), json!(long), json!(wide)] {
        assert!(crate::params::expr(&json!({ "v": v.clone() }), "v").is_none(), "{v}");
        assert!(parse::<LengthExpr>(v.clone()).is_none(), "{v}");
        assert!(parse::<AngleExpr>(v.clone()).is_none(), "{v}");
    }
}

#[test]
fn points_accept_what_the_point_readers_accept() {
    assert!(parse::<Point3>(json!([1, 2, 3])).is_some());
    assert!(parse::<Point3>(json!({"x": 1, "y": 2, "z": 3, "label": "extra keys are fine"})).is_some());
    for v in [json!([1, 2]), json!([1, 2, 3, 4]), json!(["a", 2, 3]), json!(null), json!({"x": 1, "y": 2}), json!([1e9, 0, 0])] {
        assert!(parse::<Point3>(v.clone()).is_none(), "{v}");
    }
    assert!(parse::<Point2>(json!([1, 2])).is_some());
    assert!(parse::<Point2>(json!([1, 2, 3])).is_none());
    assert!(parse::<PointExpr3>(json!([1, "w / 2", 3])).is_some());
    assert!(parse::<PointExpr3>(json!({"x": "a", "y": 0, "z": "b + 1"})).is_some());
    for v in [json!([1, 2]), json!(null), json!([true, 1, 2]), json!({"x": 1, "y": 2}), json!("origin")] {
        assert!(parse::<PointExpr3>(v.clone()).is_none(), "{v}");
    }
}

#[test]
fn names_and_lists() {
    assert_eq!(parse::<StringList>(json!("A")).map(|l| l.0), Some(vec!["A".to_string()]));
    assert_eq!(parse::<StringList>(json!(["A", "B"])).map(|l| l.0), Some(vec!["A".to_string(), "B".to_string()]));
    let too_many: Vec<&str> = vec!["x"; 10_001];
    for v in [json!([1]), json!(5), json!(null), json!(too_many.clone())] {
        assert!(parse::<StringList>(v.clone()).is_none(), "{v}");
    }
    let c = TypedContract::<ExtrudeParams>::new();
    assert_eq!(messages(&c.check(&json!({"distance": 5, "targets": too_many}))), vec!["`targets` must have at most 10000 items"]);
    assert!(parse::<IdOrName>(json!(3)).is_some());
    assert!(parse::<IdOrName>(json!("Base")).is_some());
    assert!(parse::<IdOrName>(json!(null)).is_none());
}

#[test]
fn operations_match_operation_parse() {
    for s in [
        "new",
        "join",
        "cut",
        "intersect",
        "newbody",
        "newcomponent",
        "union",
        "add",
        "subtract",
        "difference",
        "intersection",
        "New Body",
        "NEW-body",
        "Cut",
    ] {
        assert!(parse::<OperationParam>(json!(s)).is_some(), "{s}");
        assert!(parse::<ExtrudeOperation>(json!(s)).is_some(), "{s}");
    }
    assert!(parse::<OperationParam>(json!("auto")).is_none(), "box takes no auto");
    assert!(parse::<ExtrudeOperation>(json!("AUTO")).is_some());
    assert!(parse::<ExtrudeOperation>(json!(" auto")).is_none(), "auto is matched without trimming");
    for v in [json!("bogus"), json!(5), json!(null)] {
        assert!(parse::<OperationParam>(v.clone()).is_none(), "{v}");
    }
}

#[test]
fn extrude_keywords() {
    assert!(parse::<DirectionParam>(json!("SYMMETRIC")).is_some());
    assert!(parse::<DirectionParam>(json!("flip")).is_some());
    assert!(parse::<DirectionParam>(json!(" symmetric")).is_none(), "not trimmed");
    assert!(parse::<ExtentKeyword>(json!("Through_All")).is_some());
    assert!(parse::<ExtentKeyword>(json!("to_next")).is_none());
    assert!(parse::<ConventionParam>(json!("total_distance")).is_some());
    assert!(parse::<ConventionParam>(json!("Total_Distance")).is_none(), "serde, case-sensitive");
    assert!(parse::<ThinSideParam>(json!(" Two-Sides ")).is_some(), "ThinSide::parse trims and maps '-'");
    assert!(parse::<ThinSideParam>(json!("middle")).is_none());
}

#[test]
fn profiles_match_features_profiles() {
    for v in [json!("all"), json!("ALL"), json!(0), json!([0, 1]), json!([{"point": [1, 2]}]), json!([["l1", "l2"]]), json!([{"curves": ["l1"]}])] {
        assert!(parse::<ProfilesParam>(v.clone()).is_some(), "{v}");
    }
    for v in [json!("none"), json!([]), json!([true]), json!(null), json!(-1), json!([0, {"point": [1, 2]}])] {
        assert!(parse::<ProfilesParam>(v.clone()).is_none(), "{v}");
    }
    let c = TypedContract::<ExtrudeParams>::new();
    let issue = |profiles: Value| messages(&c.check(&json!({"distance": 5, "profiles": profiles})));
    assert_eq!(issue(json!([])), vec!["`profiles` is empty"]);
    assert_eq!(issue(json!(1.5)), vec!["`profiles` must be \"all\", an index or a list"]);
    assert_eq!(issue(json!(vec![0; 10_001])), vec!["`profiles` must have at most 10000 items"]);
}

#[test]
fn unknown_keys_get_suggestions() {
    let c = box_contract();
    let issues = c.check(&json!({"length": 5, "width": 5, "height": 5, "operaton": "cut", "colour": "red"}));
    assert_eq!(messages(&issues), vec!["unknown key `colour`", "unknown key `operaton`; did you mean `operation`?"]);
    assert_eq!(issues.iter().find(|i| i.path == "operaton").and_then(|i| i.suggestion.clone()), Some("operation".into()));
    // A hostile key is cut in the message.
    let issues = c.check(&json!({"length": 5, "width": 5, "height": 5, "k".repeat(100_000): 1}));
    assert_eq!(messages(&issues), vec![format!("unknown key `{}…`", "k".repeat(64))]);
    // Short keys get a suggestion only one edit away.
    let issues = TypedContract::<ExtrudeParams>::new().check(&json!({"distance": 5, "x": 1, "p0": 1, "ids": 1, "sid": "inside"}));
    assert_eq!(messages(&issues), vec!["unknown key `ids`", "unknown key `p0`", "unknown key `sid`; did you mean `side`?", "unknown key `x`"]);
}

#[test]
fn shape_problems_are_issues() {
    let c = box_contract();
    assert_eq!(messages(&c.check(&json!({"width": 5, "height": 5}))), vec!["`length` is required"]);
    assert_eq!(messages(&c.check(&json!({"length": true, "width": 5, "height": 5}))), vec!["`length` must be a number or an expression"]);
    assert_eq!(
        messages(&c.check(&json!({"length": 5, "width": 5, "height": 5, "corner": null}))),
        vec!["`corner` must be a point [x, y, z] of numbers or length expressions"]
    );
    assert_eq!(messages(&c.check(&json!({"length": 5, "width": 5, "height": 5, "name": 7}))), vec!["`name` must be text"]);
    let e = TypedContract::<ExtrudeParams>::new();
    assert_eq!(messages(&e.check(&json!({"distance": 5, "thin": "true"}))), vec!["`thin` must be true or false"]);
    // serde quotes the caller's value: split at the last ", expected ", and cap the message.
    assert_eq!(messages(&e.check(&json!({"distance": 5, "through_all": "x, expected y"}))), vec!["`through_all` must be true or false"]);
    let long = messages(&e.check(&json!({"distance": 5, "through_all": format!("{}, expected y", "a".repeat(100_000))})));
    assert_eq!(long, vec!["`through_all` must be true or false"]);
    assert_eq!(messages(&c.check(&json!([1, 2, 3]))), vec!["parameters must be an object"]);
    assert!(c.check(&json!(null)).is_empty(), "null is read as {{}}, which the handler then judges");
    let many: serde_json::Map<String, Value> = (0..70).map(|i| (format!("k{i}"), json!(1))).collect();
    assert!(messages(&c.check(&Value::Object(many))).iter().any(|m| m.starts_with("too many parameters")));
}

#[test]
fn schemas_and_the_key_check_agree() {
    for (schema, fields) in [
        (
            input_schema_for::<BoxParams>(),
            vec!["body_name", "body_names", "center", "corner", "height", "length", "name", "operation", "participants", "targets", "width"],
        ),
        (
            input_schema_for::<ExtrudeParams>(),
            vec![
                "body",
                "body_name",
                "body_names",
                "convention",
                "curves",
                "direction",
                "distance",
                "distance2",
                "extent",
                "face",
                "face_body",
                "faces",
                "from",
                "name",
                "operation",
                "participants",
                "profiles",
                "side",
                "sketch",
                "start_offset",
                "taper",
                "targets",
                "thickness",
                "thickness2",
                "thin",
                "through_all",
                "to",
                "to_offset",
            ],
        ),
    ] {
        let v = schema.as_value();
        assert_eq!(schema_keys(v).into_iter().collect::<Vec<_>>(), fields);
        assert_eq!(v.get("additionalProperties"), Some(&json!(false)), "the schema refuses unknown keys too");
        assert_eq!(v.get("$schema").and_then(Value::as_str), Some("https://json-schema.org/draft/2020-12/schema"));
    }
}

#[test]
fn present_null_is_refused_but_absent_is_fine() {
    let base = json!({"length": 5, "width": 5, "height": 5});
    assert!(box_contract().check(&base).is_empty());
    let c: TypedContract<ExtrudeParams> = TypedContract::new();
    assert!(c.check(&json!({"distance": 5})).is_empty());
    // `body` picks the placed instance `face` is read in (frames::to_local), so it is a key.
    assert!(c.check(&json!({"distance": 5, "face": [0, 0, 0], "body": "Part"})).is_empty());
    for key in ["sketch", "profiles", "faces", "convention", "to", "from", "thin"] {
        let mut p = json!({"distance": 5});
        p[key] = Value::Null;
        assert_eq!(c.check(&p).len(), 1, "{key}: null");
    }
    // Keys today's reader treats `null` like absence stay optional.
    for key in ["taper", "direction", "face", "operation", "name"] {
        let mut p = json!({"distance": 5});
        p[key] = Value::Null;
        assert!(c.check(&p).is_empty(), "{key}: null");
    }
}

fn sketch_rectangle(s: &mut Session) {
    for (id, p) in [
        ("sketch.create", json!({"plane": "XY"})),
        ("sketch.rectangle.center", json!({"center": [0, 0], "corner": [10, 10]})),
        ("sketch.finish", json!({})),
    ] {
        if let Err(e) = s.execute(id, &p) {
            panic!("{id}: {e}");
        }
    }
}

fn features(s: &Session) -> usize {
    s.doc.features.len()
}

#[test]
fn off_runs_the_handler_untouched() {
    let mut s = Session::default();
    assert_eq!(s.param_check, ParamCheck::Off);
    let r = s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "operaton": "cut"}));
    let r = r.unwrap_or_else(|e| panic!("{e}"));
    assert!(r.get("param_warnings").is_none());
    assert_eq!(r["created_bodies"], json!(["Body1"]), "the typo is ignored, as before: a new body");
}

#[test]
fn warn_runs_and_reports() {
    let mut s = Session { param_check: ParamCheck::Warn, ..Session::default() };
    let r = s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "operaton": "cut"})).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(r["param_warnings"], json!(["unknown key `operaton`; did you mean `operation`?"]));
    assert_eq!(features(&s), 1);
    let r = s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "corner": [20, 0, 0]})).unwrap_or_else(|e| panic!("{e}"));
    assert!(r.get("param_warnings").is_none(), "a clean call carries no warnings");
}

#[test]
fn strict_refuses_and_leaves_the_design_alone() {
    let mut s = Session { param_check: ParamCheck::Strict, ..Session::default() };
    let e = s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "operaton": "cut"})).err().map(|e| e.to_string());
    assert_eq!(e.as_deref(), Some("invalid parameters for `solid.box`: unknown key `operaton`; did you mean `operation`?"));
    assert_eq!(features(&s), 0);
    sketch_rectangle(&mut s);
    let before = features(&s);
    let e = s.execute("solid.extrude", &json!({"distance": 5, "tapper": "10 deg"})).err().map(|e| e.to_string());
    assert_eq!(e.as_deref(), Some("invalid parameters for `solid.extrude`: unknown key `tapper`; did you mean `taper`?"));
    let e = s.execute("solid.extrude", &json!({"distance": 5, "operation": 5})).err().map(|e| e.to_string());
    assert!(e.as_deref().is_some_and(|e| e.contains("`operation` must be one of")), "{e:?}");
    assert_eq!(features(&s), before);
    s.execute("solid.extrude", &json!({"distance": 5, "direction": "symmetric"})).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(features(&s), before + 1);
}

#[test]
fn nested_runs_are_checked_too() {
    let mut s = Session::default();
    sketch_rectangle(&mut s);
    s.execute("solid.extrude", &json!({"distance": 5, "name": "Slab"})).unwrap_or_else(|e| panic!("{e}"));
    s.param_check = ParamCheck::Strict;
    let e = s
        .execute("timeline.redefine", &json!({"feature": "Slab", "command": "solid.extrude", "params": {"distance": 8, "directon": "symmetric"}}))
        .err()
        .map(|e| e.to_string());
    assert!(e.as_deref().is_some_and(|e| e.contains("unknown key `directon`; did you mean `direction`?")), "{e:?}");
}

#[test]
fn the_level_survives_a_new_design() {
    let mut s = Session { param_check: ParamCheck::Strict, ..Session::default() };
    s.execute("file.new", &json!({})).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(s.param_check, ParamCheck::Strict);
    assert_eq!(s.scratch().param_check, ParamCheck::Strict);
}

#[test]
fn feature_result_serializes_like_the_old_reply() {
    for (warning, ms) in [(None, None), (Some("a Boolean needed a retry".to_string()), Some(12.5)), (None, Some(0.0)), (None, Some(1.0 / 3.0))] {
        let old = json!({"feature": 7, "name": "Box1", "bodies": ["A", "B"], "created_bodies": ["B"], "changed_bodies": [],
            "removed_bodies": ["C"], "warning": warning, "ms": ms});
        let new = FeatureResult {
            feature: 7,
            name: "Box1".into(),
            bodies: vec!["A".into(), "B".into()],
            created_bodies: vec!["B".into()],
            changed_bodies: vec![],
            removed_bodies: vec!["C".into()],
            warning: warning.clone(),
            ms,
            param_warnings: vec![],
        };
        assert_eq!(serde_json::to_value(&new).ok(), Some(old));
    }
}

/// Whether `v` conforms to `node` of the schema `root`, for the keywords these schemas use
/// apart from `pattern`: `type`, `properties`, `required`, `additionalProperties: false`,
/// `items`, `minItems`, `maxItems`, `uniqueItems`, `enum`, `minLength`, `maxLength`, `minimum`,
/// `maximum`, `exclusiveMinimum`, `exclusiveMaximum`, `anyOf`, `not` and `$ref`. (`integer`
/// here is what serde reads as one, so `1.0` is not.)
fn conforms(root: &Value, node: &Value, v: &Value) -> bool {
    if let Some(name) = node.get("$ref").and_then(Value::as_str).and_then(|r| r.strip_prefix("#/$defs/")) {
        return root.get("$defs").and_then(|d| d.get(name)).is_some_and(|t| conforms(root, t, v));
    }
    if let Some(branches) = node.get("anyOf").and_then(Value::as_array)
        && !branches.iter().any(|b| conforms(root, b, v))
    {
        return false;
    }
    if node.get("not").is_some_and(|n| conforms(root, n, v)) {
        return false;
    }
    let is = |t: &str| match t {
        "null" => v.is_null(),
        "boolean" => v.is_boolean(),
        "integer" => v.is_u64() || v.is_i64(),
        "number" => v.is_number(),
        "string" => v.is_string(),
        "array" => v.is_array(),
        "object" => v.is_object(),
        _ => false,
    };
    let typed = match node.get("type") {
        Some(Value::String(t)) => is(t),
        Some(Value::Array(ts)) => ts.iter().filter_map(Value::as_str).any(is),
        _ => true,
    };
    if !typed {
        return false;
    }
    if let (Some(items), Some(a)) = (node.get("items"), v.as_array())
        && !a.iter().all(|x| conforms(root, items, x))
    {
        return false;
    }
    let bound = |k: &str| node.get(k).and_then(Value::as_u64).map(|n| n as usize);
    if let Some(a) = v.as_array() {
        let unique = a.iter().enumerate().all(|(i, x)| !a.iter().skip(i + 1).any(|y| y == x));
        if bound("minItems").is_some_and(|n| a.len() < n)
            || bound("maxItems").is_some_and(|n| a.len() > n)
            || (node.get("uniqueItems") == Some(&json!(true)) && !unique)
        {
            return false;
        }
    }
    if let Some(s) = v.as_str() {
        let n = s.chars().count();
        if bound("minLength").is_some_and(|m| n < m) || bound("maxLength").is_some_and(|m| n > m) {
            return false;
        }
    }
    if let Some(x) = v.as_f64() {
        let limit = |k: &str| node.get(k).and_then(Value::as_f64);
        if limit("minimum").is_some_and(|m| x < m)
            || limit("maximum").is_some_and(|m| x > m)
            || limit("exclusiveMinimum").is_some_and(|m| x <= m)
            || limit("exclusiveMaximum").is_some_and(|m| x >= m)
        {
            return false;
        }
    }
    if let Some(values) = node.get("enum").and_then(Value::as_array)
        && !values.contains(v)
    {
        return false;
    }
    if let Some(o) = v.as_object() {
        let props = node.get("properties").and_then(Value::as_object);
        let mut required = node.get("required").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str);
        if !required.all(|k| o.contains_key(k)) {
            return false;
        }
        for (k, x) in o {
            match props.and_then(|p| p.get(k)) {
                Some(p) if !conforms(root, p, x) => return false,
                None if node.get("additionalProperties") == Some(&json!(false)) => return false,
                _ => {}
            }
        }
    }
    true
}

#[test]
fn replies_match_the_output_schema() {
    let schema = output_schema_for::<FeatureResult>();
    let schema = schema.as_value();
    let required: Vec<&str> = schema.get("required").and_then(Value::as_array).into_iter().flatten().filter_map(Value::as_str).collect();
    assert!(!required.contains(&"param_warnings"));
    let mut s = Session::default();
    sketch_rectangle(&mut s);
    s.param_check = ParamCheck::Warn;
    let replies = [
        s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "corner": [20, 0, 0]})),
        s.execute("solid.extrude", &json!({"distance": 5, "operation": "join"})),
        s.execute("solid.box", &json!({"length": 5, "width": 5, "height": 5, "colour": "red"})),
    ];
    for r in replies {
        let r = r.unwrap_or_else(|e| panic!("{e}"));
        assert!(conforms(schema, schema, &r), "{r} doesn't match {schema}");
    }
    // The check itself catches a wrong type, a wrong item type and a missing key. (The output
    // schema leaves other keys open, so a new result key doesn't break clients.)
    let ok = json!({"feature": 1, "name": "Box1", "bodies": [], "created_bodies": [], "changed_bodies": [], "removed_bodies": [], "warning": null, "ms": null});
    assert!(conforms(schema, schema, &ok));
    for (k, v) in [("feature", json!("1")), ("bodies", json!([1])), ("warning", json!(5)), ("ms", json!("1"))] {
        let mut bad = ok.clone();
        bad[k] = v;
        assert!(!conforms(schema, schema, &bad), "{bad}");
    }
    let mut bad = ok.clone();
    bad.as_object_mut().map(|o| o.remove("name"));
    assert!(!conforms(schema, schema, &bad), "{bad}");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn the_recorder_keeps_a_bounded_number_of_signatures() {
    use super::record::tally;
    let mut seen = std::collections::HashMap::new();
    // Lines are due at counts 1, 2, 4, …
    assert_eq!(tally(&mut seen, "a".into(), 2), Some((false, 1)));
    assert_eq!(tally(&mut seen, "a".into(), 2), Some((false, 2)));
    assert_eq!(tally(&mut seen, "a".into(), 2), None);
    assert_eq!(tally(&mut seen, "b".into(), 2), Some((false, 1)));
    // Full: new signatures are counted under one entry; known ones still count.
    assert_eq!(tally(&mut seen, "c".into(), 2), Some((true, 1)));
    assert_eq!(tally(&mut seen, "d".into(), 2), Some((true, 2)));
    assert_eq!(tally(&mut seen, "a".into(), 2), Some((false, 4)));
    assert_eq!(seen.len(), 3);
}

#[test]
fn describe_returns_both_schemas() {
    let mut s = Session::default();
    let d = s.execute("engine.describe", &json!({"command": "solid.box"})).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(d["id"], "solid.box");
    assert!(d["input"]["properties"].get("length").is_some());
    assert!(d["output"]["properties"].get("created_bodies").is_some());
    // Its own output schema lists the warnings warn mode can add.
    let own = s.execute("engine.describe", &json!({"command": "engine.describe"})).unwrap_or_else(|e| panic!("{e}"));
    assert!(own["output"]["properties"].get("param_warnings").is_some(), "{own}");
    let d = s.execute("engine.describe", &json!({"command": "solid.revolve"})).unwrap_or_else(|e| panic!("{e}"));
    assert!(d["input"].is_null() && d["output"].is_null(), "unconverted commands have no schemas yet");
    let e = s.execute("engine.describe", &json!({"command": "solid.extrud"})).err().map(|e| e.to_string());
    assert!(e.as_deref().is_some_and(|e| e.contains("did you mean `solid.extrude`")), "{e:?}");
}

/// The profiles schema and the contract agree on every listed case; the schema is deliberately
/// narrower on non-text curve ids and loops past the outer one (and describes the app's
/// `whole_boundaries` selection only by its `type`).
#[test]
fn the_profiles_schema_matches_the_contract() {
    let root = input_schema_for::<ExtrudeParams>();
    let root = root.as_value();
    let node = &root["properties"]["profiles"];
    let accepted = |v: &Value| parse::<ProfilesParam>(v.clone()).is_some();
    for v in [
        json!(3),
        json!(-1),
        json!([]),
        json!([0, 1]),
        json!([0, {"point": [1, 2]}]),
        json!([{"point": [1, 2]}, {"point": [3, 4]}]),
        json!([{"point": [1e9, 2]}]),
        json!([["l1"]]),
        json!([[]]),
        json!([{"curves": "l1"}, ["l2"]]),
        json!([{"curves": []}]),
        json!([{"loops": []}]),
        json!([{"loops": [5]}]),
        json!([{"loops": [{"curves": ["l1"]}]}]),
        json!({}),
        json!({"offset_boundaries": ["a"]}),
        json!({"offset_boundaries": []}),
        json!({"offset_boundaries": ["a", "a"]}),
        json!({"type": "offset_boundaries", "associations": ["a", "b"]}),
        json!({"type": "something"}),
        serde_json::from_str("18446744073709551616").unwrap_or_default(),
        // The helper's precedence: all items with `point` are points; `curves` before `loops`;
        // `offset_boundaries` before `type`.
        json!([{"point": null, "curves": ["l1"]}]),
        json!([{"point": [1, 2], "curves": ["l1"]}]),
        json!([{"curves": [], "loops": [{"curves": ["l1"]}]}]),
        json!([{"point": [0, 0], "curves": ["l1"]}, ["l2"]]),
        json!([{"point": [1, 2], "curves": ["a"]}, {"curves": ["b"]}]),
        json!([{"point": [0, 0], "loops": [{"curves": ["l1"]}]}, {"curves": "l2"}]),
        json!({"type": "offset_boundaries", "associations": ["a"], "offset_boundaries": []}),
    ] {
        assert_eq!(conforms(root, node, &v), accepted(&v), "{v}");
    }
    // An index is bounded to what the helper reads as a u64. (`conforms` refuses 2^64 above on
    // type alone, so the bound itself is checked here.)
    let index = |b: &Value| b.get("type") == Some(&json!("integer"));
    let bounds: Vec<&Value> = root["$defs"]["Profiles"]["anyOf"]
        .as_array()
        .into_iter()
        .flatten()
        .flat_map(|b| [b, &b["items"]])
        .filter(|b| index(b))
        .map(|b| &b["maximum"])
        .collect();
    assert_eq!(bounds, [&json!(u64::MAX), &json!(u64::MAX)]);
    // Where the schema is deliberately narrower: non-text ids and loops past the outer one.
    for v in [json!([["l1", 5]]), json!([{"loops": [{"curves": ["l1"]}, 5]}])] {
        assert!(accepted(&v) && !conforms(root, node, &v), "{v}");
    }
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn recorder_lines_have_the_documented_shape() {
    let l = super::record::line("solid.box", &["unknown key `x`"], "ok", "a_test", 4, &json!({"x": "y".repeat(5000)}));
    let keys: Vec<&str> = l.as_object().into_iter().flat_map(|o| o.keys().map(String::as_str)).collect();
    assert_eq!(keys, ["command", "count", "issues", "outcome", "sample_params", "source"]);
    assert_eq!((l["command"].clone(), l["count"].clone(), l["outcome"].clone()), (json!("solid.box"), json!(4), json!("ok")));
    assert!(l["sample_params"].as_str().is_some_and(|s| s.len() <= 1024 + "…".len() && s.ends_with('…')), "{l}");
    // A strict refusal is told apart from a handler's error.
    use super::record::outcome_text;
    assert_eq!(outcome_text(None), "refused");
    assert_eq!(outcome_text(Some(&Ok(json!({})))), "ok");
    assert_eq!(outcome_text(Some(&Err(crate::EngineError::Other("boom".into())))), "error: boom");
    // The recorder is on exactly when built with `param-audit` and the variable names a directory.
    let set = std::env::var_os("SOLIDCRAFT_PARAM_AUDIT").is_some_and(|v| !v.is_empty());
    assert_eq!(super::record::enabled(), cfg!(feature = "param-audit") && set);
}

/// A call that already fails gets the same error in strict as with the check off, apart from
/// the missing-key, missing-instead-of-wrong-type, non-object, profile-index, 10,000-item and
/// over-4-KB messages the report lists.
#[test]
fn strict_keeps_todays_errors() {
    let cases = [
        ("solid.box", json!({"length": 5, "width": 5, "height": 5, "operation": "bogus"})),
        ("solid.box", json!({"length": true, "width": 5, "height": 5})),
        ("solid.box", json!({"length": 5, "width": 5, "height": 5, "corner": [1, 2]})),
        ("solid.extrude", json!({"distance": 5, "operation": "bogus"})),
        ("solid.extrude", json!({"distance": 5, "direction": "sideways"})),
        ("solid.extrude", json!({"distance": 5, "direction": "SIDEWAYS"})),
        ("solid.extrude", json!({"distance": 5, "profiles": [{"point": [1]}]})),
        ("solid.extrude", json!({"distance": 5, "profiles": [["l1"], 7]})),
        ("solid.extrude", json!({"distance": 5, "profiles": []})),
        ("solid.extrude", json!({"distance": 5, "profiles": [true]})),
        ("solid.extrude", json!({"distance": 5, "faces": []})),
        ("solid.extrude", json!({"distance": 5, "faces": null})),
        ("solid.extrude", json!({"distance": 5, "faces": [[1, 2]]})),
        ("solid.extrude", json!({"distance": 5, "profiles": {"offset_boundaries": ["a", "a"]}})),
        ("solid.extrude", json!({"distance": 5, "thin": true, "thickness": 1, "side": "middle"})),
        ("solid.extrude", json!({"distance": 5, "thin": "true"})),
        ("solid.extrude", json!({"distance": 5, "to": [1, 2]})),
        ("solid.extrude", json!({"distance": 5, "from": "x"})),
        ("solid.extrude", json!({"distance": 5, "sketch": true})),
        ("solid.extrude", json!({"distance": 5, "convention": "per_side"})),
        ("solid.extrude", json!({"distance": "  "})),
        ("solid.extrude", json!({"distance": 5, "operation": "x".repeat(1000)})),
        ("engine.describe", json!({"command": 5})),
        ("engine.describe", json!({"command": null})),
        ("engine.describe", json!({"command": "solid.extrud"})),
    ];
    for (id, p) in cases {
        let error = |level: ParamCheck| {
            let mut s = Session { param_check: level, ..Session::default() };
            sketch_rectangle(&mut s);
            s.execute(id, &p).err().map(|e| e.to_string())
        };
        let off = error(ParamCheck::Off);
        assert!(off.is_some(), "{p} should fail today");
        assert_eq!(error(ParamCheck::Strict), off, "{p}");
    }
}

/// Every `default` in the input schemas is a value its own property schema allows.
#[test]
fn schema_defaults_are_valid() {
    for root in [input_schema_for::<BoxParams>(), input_schema_for::<ExtrudeParams>()] {
        let root = root.as_value();
        for (key, property) in root["properties"].as_object().into_iter().flatten() {
            if let Some(d) = property.get("default") {
                assert!(conforms(root, property, d), "`{key}`: default {d} doesn't match {property}");
            }
        }
    }
}

/// The expression schema's "not blank" pattern uses exactly the whitespace Rust's `trim` does.
#[test]
fn the_blank_pattern_matches_trim() {
    let listed = |c: char| super::types::WHITESPACE.iter().any(|&(a, b)| (a..=b).contains(&c));
    for c in (0..=0x10_ffff).filter_map(char::from_u32) {
        assert_eq!(listed(c), c.is_whitespace(), "U+{:04X}", c as u32);
    }
    assert_eq!(super::types::not_blank_pattern(), r"[^\u0009-\u000d\u0020\u0085\u00a0\u1680\u2000-\u200a\u2028-\u2029\u202f\u205f\u3000]");
    assert!(parse::<LengthExpr>(json!("\u{85}")).is_none() && parse::<LengthExpr>(json!("\u{feff}")).is_some());
}

/// A handler-worded message for a key the handler ignores stays bounded in a warning.
#[test]
fn long_values_in_handler_messages_are_capped() {
    let mut s = Session { param_check: ParamCheck::Warn, ..Session::default() };
    sketch_rectangle(&mut s);
    let r = s.execute("solid.extrude", &json!({"distance": 5, "side": "x".repeat(1_000_000)})).unwrap_or_else(|e| panic!("{e}"));
    let w = r["param_warnings"][0].as_str().unwrap_or_default();
    assert!(w.starts_with("unknown side `xxx") && w.len() <= 4096 + "…".len(), "{} bytes", w.len());
}
