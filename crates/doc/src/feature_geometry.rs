//! The geometry a feature holds, each piece with its role: the points that pick faces (moved
//! with the feature, and the source of its persistent face names), other positions, directions,
//! planes, construction references, a primitive's placement point and its profile selections.
//! One walk, written once for `&` and `&mut`, behind `transform_geometry`, `face_points`,
//! `profile_sels` and `point_expr`. Every variant is listed, so a new one has to say what each of
//! its points is.

use solidcraft_geom::Vec3;

use crate::construct::GeoRef;
use crate::{AxisRef, FeatureKind, PatternKind, PlaneRef, PointExpr, ProfileSel};

/// A piece of a feature's geometry: `V` is `&Vec3` or `&mut Vec3`, and so on for the others.
pub(crate) enum Geo<V, P, G, E, S> {
    /// A point that picks a face: it moves with the feature and names the face it picks.
    Pick(V),
    /// A point that picks a face but stays where it is when the feature is moved (an Emboss
    /// cylinder, chosen in its component's frame).
    Pinned(V),
    /// Any other position (an edge pick, an axis origin, a fold line): it moves with the feature.
    Pos(V),
    /// A direction: it turns with the feature.
    Dir(V),
    Plane(P),
    /// A construction axis's or point's reference to the model.
    Construct(G),
    /// A primitive's placement point (length expressions).
    Placement(E),
    /// A profile selection, with its sketch.
    Profiles(u64, S),
}

pub(crate) type GeoItem<'a> = Geo<&'a Vec3, &'a PlaneRef, &'a GeoRef, &'a PointExpr, &'a ProfileSel>;
pub(crate) type GeoItemMut<'a> = Geo<&'a mut Vec3, &'a mut PlaneRef, &'a mut GeoRef, &'a mut PointExpr, &'a mut ProfileSel>;

/// Every piece of a feature's geometry, written once for `&` and `&mut`: `$item` is `GeoItem` or
/// `GeoItemMut`, `$refs` the matching `refs` / `refs_mut`, `$m` is `mut` or nothing. Face picks
/// come in the order the feature's face names follow.
macro_rules! geometry_walk {
    ($name:ident, $item:ident, $refs:ident $(, $m:tt)?) => {
        pub(crate) fn $name<'a>(&'a $($m)? self, f: &mut impl FnMut($item<'a>)) {
            use Geo::{Construct, Dir, Pick, Pinned, Placement, Plane, Pos, Profiles};
            match self {
                FeatureKind::Sketch { plane, .. } | FeatureKind::ConstructionPlane { plane } | FeatureKind::Mirror { plane, .. } => f(Plane(plane)),
                FeatureKind::ConstructionAxis { def } => {
                    for r in def.$refs() {
                        f(Construct(r));
                    }
                }
                FeatureKind::ConstructionPoint { def } => {
                    for r in def.$refs() {
                        f(Construct(r));
                    }
                }
                FeatureKind::Extrude { sketch, profiles, extent, thin, .. } => {
                    // A thin extrude follows curves; its profiles are not used.
                    if thin.is_none() {
                        f(Profiles(*sketch, profiles));
                    }
                    if let Some(p) = & $($m)? extent.to {
                        f(Pick(p));
                    }
                    if let Some(p) = & $($m)? extent.from {
                        f(Pick(p));
                    }
                }
                // Surface Extrude refuses `to` and `from` for now; they would pick faces as for
                // Extrude.
                FeatureKind::SurfaceExtrude { sketch, profiles, extent, .. } => {
                    if let Some(s) = profiles {
                        f(Profiles(*sketch, s));
                    }
                    if let Some(p) = & $($m)? extent.to {
                        f(Pick(p));
                    }
                    if let Some(p) = & $($m)? extent.from {
                        f(Pick(p));
                    }
                }
                FeatureKind::Revolve { sketch, profiles, axis, to, .. } => {
                    f(Profiles(*sketch, profiles));
                    if let AxisRef::Line { origin, dir } = axis {
                        f(Pos(origin));
                        f(Dir(dir));
                    }
                    if let Some(p) = to {
                        f(Pick(p));
                    }
                }
                FeatureKind::SurfaceRevolve { sketch, profiles, axis, .. } => {
                    if let Some(s) = profiles {
                        f(Profiles(*sketch, s));
                    }
                    if let AxisRef::Line { origin, dir } = axis {
                        f(Pos(origin));
                        f(Dir(dir));
                    }
                }
                FeatureKind::Sweep { sketch, profiles, .. } | FeatureKind::SheetBase { sketch, profiles, .. } => f(Profiles(*sketch, profiles)),
                FeatureKind::SurfaceSweep { sketch, profiles, .. } => {
                    if let Some(s) = profiles {
                        f(Profiles(*sketch, s));
                    }
                }
                FeatureKind::Emboss { sketch, profiles, face, .. } => {
                    f(Profiles(*sketch, profiles));
                    if let Some(face) = face {
                        f(Pinned(& $($m)? face.at));
                    }
                }
                FeatureKind::Loft { sections, .. } => {
                    // A section that is a point has no profile.
                    for s in sections {
                        if s.point.is_none() {
                            f(Profiles(s.sketch, & $($m)? s.profiles));
                        }
                    }
                }
                FeatureKind::SurfaceLoft { sections } => {
                    for s in sections {
                        if let Some(p) = & $($m)? s.profiles {
                            f(Profiles(s.sketch, p));
                        }
                    }
                }
                FeatureKind::Patch { sketch, profiles, edges, .. } => {
                    if let Some(sketch) = sketch {
                        f(Profiles(*sketch, profiles));
                    }
                    for e in edges {
                        f(Pos(e));
                    }
                }
                FeatureKind::Fillet { edges, style, .. } => {
                    for e in edges {
                        f(Pos(e));
                    }
                    if let crate::FilletStyle::Variable { start, .. } | crate::FilletStyle::Spline { start, .. } = style {
                        f(Pos(start));
                    }
                }
                FeatureKind::Chamfer { edges, .. } | FeatureKind::SheetHem { edges, .. } | FeatureKind::SurfaceExtend { edges, .. } => {
                    for e in edges {
                        f(Pos(e));
                    }
                }
                FeatureKind::Shell { faces, .. }
                | FeatureKind::OffsetFace { faces, .. }
                | FeatureKind::DeleteFace { faces, .. }
                | FeatureKind::SurfaceOffset { faces, .. } => {
                    for p in faces {
                        f(Pick(p));
                    }
                }
                FeatureKind::Unstitch { faces, .. } | FeatureKind::ReverseNormal { faces, .. } => {
                    for p in faces {
                        f(Pos(p));
                    }
                }
                FeatureKind::BoundaryFill { cells, .. } => {
                    for p in cells {
                        f(Pos(p));
                    }
                }
                FeatureKind::Draft { faces, neutral, pull, .. } => {
                    for p in faces {
                        f(Pick(p));
                    }
                    f(Plane(neutral));
                    f(Dir(pull));
                }
                FeatureKind::ReplaceFace { faces, target, .. } => {
                    for p in faces {
                        f(Pick(p));
                    }
                    f(Plane(target));
                }
                FeatureKind::SplitFace { faces, plane, tool, .. } => {
                    for p in faces {
                        f(Pick(p));
                    }
                    f(Plane(plane));
                    if let Some(t) = tool {
                        f(Pos(& $($m)? t.point));
                    }
                }
                FeatureKind::Split { plane, tool, .. } => {
                    f(Plane(plane));
                    if let Some(t) = tool {
                        f(Pos(& $($m)? t.point));
                    }
                }
                FeatureKind::SurfaceTrim { plane, tool, keep, .. } => {
                    f(Plane(plane));
                    if let Some(t) = tool {
                        f(Pos(& $($m)? t.point));
                    }
                    f(Pos(keep));
                }
                FeatureKind::Box { corner: p, .. } | FeatureKind::Sphere { center: p, .. } | FeatureKind::Torus { center: p, .. } => {
                    f(Placement(p))
                }
                FeatureKind::Cylinder { base, axis, .. } | FeatureKind::Coil { base, axis, .. } => {
                    f(Placement(base));
                    f(Dir(axis));
                }
                FeatureKind::Pattern { pattern, .. } => match pattern {
                    PatternKind::Rectangular { dir1, dir2, .. } => {
                        f(Dir(dir1));
                        if let Some(d) = dir2 {
                            f(Dir(d));
                        }
                    }
                    PatternKind::Circular { origin, axis, .. } => {
                        f(Pos(origin));
                        f(Dir(axis));
                    }
                    // Follows its path sketch.
                    PatternKind::Path { .. } => {}
                },
                FeatureKind::Hole { position, direction, points, to, .. } => {
                    // Placed at sketch points, the hole names no face by its position.
                    if points.is_none() {
                        f(Pick(position));
                    } else {
                        f(Pos(position));
                    }
                    f(Dir(direction));
                    if let Some(p) = to {
                        f(Pick(p));
                    }
                }
                FeatureKind::Thread { face, .. } => f(Pick(face)),
                FeatureKind::SheetFlange { edges, .. } => {
                    for e in edges {
                        f(Pos(& $($m)? e.point));
                    }
                }
                FeatureKind::SheetFold { a, b, fixed, .. } => {
                    f(Pos(a));
                    f(Pos(b));
                    if let Some(p) = fixed {
                        f(Pos(p));
                    }
                }
                FeatureKind::SheetConvert { face, bends, .. } => {
                    // With bend edges the fixed face is found by name (a plate keeps its point).
                    if bends.is_empty() {
                        f(Pos(face));
                    } else {
                        f(Pick(face));
                    }
                    for p in bends {
                        f(Pos(p));
                    }
                }
                FeatureKind::Boss { position, direction, .. } => {
                    f(Pick(position));
                    f(Dir(direction));
                }
                FeatureKind::SnapFit { position, direction, hook, .. } => {
                    f(Pick(position));
                    f(Dir(direction));
                    f(Dir(hook));
                }
                FeatureKind::Rest { position, direction, along, .. } => {
                    f(Pick(position));
                    f(Dir(direction));
                    if let Some(a) = along {
                        f(Dir(a));
                    }
                }
                FeatureKind::Lip { face, .. } => f(Pos(face)),
                FeatureKind::Align { from, to, from_normal, to_normal, .. } => {
                    f(Pos(from));
                    f(Pos(to));
                    if let Some(n) = from_normal {
                        f(Dir(n));
                    }
                    if let Some(n) = to_normal {
                        f(Dir(n));
                    }
                }
                FeatureKind::Scale { origin, .. } => f(Pos(origin)),
                FeatureKind::Move { rotate_axis, .. } => {
                    if let Some(a) = rotate_axis {
                        f(Dir(a));
                    }
                }
                FeatureKind::SheetContour { .. }
                | FeatureKind::SheetPerforate { .. }
                | FeatureKind::SheetUnfold { .. }
                | FeatureKind::Combine { .. }
                | FeatureKind::BoundingSolid { .. }
                | FeatureKind::Pipe { .. }
                | FeatureKind::Rib { .. }
                | FeatureKind::Remove { .. }
                | FeatureKind::Stitch { .. }
                | FeatureKind::Thicken { .. }
                | FeatureKind::Import { .. }
                | FeatureKind::MeshImport { .. }
                | FeatureKind::MeshConvert { .. }
                | FeatureKind::BodyTransfer { .. } => {}
            }
        }
    };
}

impl FeatureKind {
    geometry_walk!(walk_geometry, GeoItem, refs);
    geometry_walk!(walk_geometry_mut, GeoItemMut, refs_mut, mut);

    /// The points that pick faces (shell, draft, offset and replace faces, a hole's placement and
    /// `to`, an extrude's `to` and `from`…), in the order of the feature's persistent face names.
    pub fn face_points_mut(&mut self) -> Vec<&mut Vec3> {
        let mut v = Vec::new();
        self.walk_geometry_mut(&mut |g| {
            if let Geo::Pick(p) | Geo::Pinned(p) = g {
                v.push(p);
            }
        });
        v
    }

    /// Read-only view of [`FeatureKind::face_points_mut`].
    pub fn face_points(&self) -> Vec<Vec3> {
        let mut v = Vec::new();
        self.walk_geometry(&mut |g| {
            if let Geo::Pick(p) | Geo::Pinned(p) = g {
                v.push(*p);
            }
        });
        v
    }

    /// The feature's profile selections, each with its sketch.
    pub fn profile_sels_mut(&mut self) -> Vec<(u64, &mut ProfileSel)> {
        let mut v = Vec::new();
        self.walk_geometry_mut(&mut |g| {
            if let Geo::Profiles(sketch, sel) = g {
                v.push((sketch, sel));
            }
        });
        v
    }

    /// Read-only view of [`FeatureKind::profile_sels_mut`].
    pub fn profile_sels(&self) -> Vec<(u64, &ProfileSel)> {
        let mut v = Vec::new();
        self.walk_geometry(&mut |g| {
            if let Geo::Profiles(sketch, sel) = g {
                v.push((sketch, sel));
            }
        });
        v
    }

    /// A primitive's placement point (length expressions). Its coordinates are not inputs:
    /// they get no parameter names and keep bare numbers in mm whatever the design's units, as
    /// every other position does; renames and users still find them.
    pub fn point_expr_mut(&mut self) -> Option<&mut PointExpr> {
        let mut found = None;
        self.walk_geometry_mut(&mut |g| {
            if let Geo::Placement(p) = g {
                found = Some(p);
            }
        });
        found
    }

    /// Read-only view of [`FeatureKind::point_expr_mut`].
    pub fn point_expr(&self) -> Option<&PointExpr> {
        let mut found = None;
        self.walk_geometry(&mut |g| {
            if let Geo::Placement(p) = g {
                found = Some(p);
            }
        });
        found
    }
}
