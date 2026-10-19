//! Reference triangles belong to a definition, even while its occurrences move. Keep one lazy
//! source mesh and map it into each pose; retain the exact placed B-rep separately in ModelBody.
use std::sync::{Arc, OnceLock};

use solidcraft_geom::{Mesh, Vec3};
use solidcraft_kernel::{self as kernel, Body};

use super::ModelBody;

#[derive(Debug)]
pub(super) struct PlacedMesh {
    body: Body,
    mesh: Arc<OnceLock<Arc<Mesh>>>,
    transform: crate::Mat,
    pub(super) tolerance: f64,
}

impl PlacedMesh {
    /// Share the original definition, never a chain of previously placed meshes. A source's
    /// OnceLock lives with its immutable geometry, so edits get a new source automatically.
    pub(super) fn new(source: &ModelBody, transform: &crate::Mat) -> Option<Self> {
        if !proper_rigid(transform) {
            return None;
        }
        Some(match source.mesh_placement.as_deref() {
            Some(original) => Self {
                body: original.body.clone(),
                mesh: original.mesh.clone(),
                transform: crate::mat_mul(transform, &original.transform),
                tolerance: original.tolerance,
            },
            None => Self { body: source.body.clone(), mesh: source.mesh.clone(), transform: *transform, tolerance: source.display_tol() },
        })
    }

    pub(super) fn mesh(&self) -> Arc<Mesh> {
        let source = reference_mesh(&self.body, &self.mesh, self.tolerance);
        Arc::new(Mesh {
            positions: source.positions.iter().map(|p| crate::apply_point(&self.transform, *p)).collect(),
            normals: source.normals.iter().map(|n| crate::apply_vector(&self.transform, *n)).collect(),
            edges: source.edges.iter().map(|edge| edge.iter().map(|p| crate::apply_point(&self.transform, *p)).collect()).collect(),
            triangles: source.triangles.clone(),
            tri_face: source.tri_face.clone(),
            edge_faces: source.edge_faces.clone(),
            seams: source.seams.clone(),
            smooth_edges: source.smooth_edges.clone(),
            sharp_edges: source.sharp_edges.clone(),
        })
    }
}

pub(super) fn reference_mesh(body: &Body, mesh: &OnceLock<Arc<Mesh>>, tolerance: f64) -> Arc<Mesh> {
    mesh.get_or_init(|| Arc::new(body.reference_display_mesh_angular(tolerance, kernel::DISPLAY_ANGLE).unwrap_or_default())).clone()
}

/// Only proper rotations and translations preserve triangle winding, normal lengths and the
/// reference chord tolerance. Scale, shear, reflection and malformed matrices use the mesher.
fn proper_rigid(m: &crate::Mat) -> bool {
    if m.iter().flatten().any(|v| !v.is_finite()) || m[0][3] != 0. || m[1][3] != 0. || m[2][3] != 0. || m[3][3] != 1. {
        return false;
    }
    let [x, y, z] = [Vec3::new(m[0][0], m[0][1], m[0][2]), Vec3::new(m[1][0], m[1][1], m[1][2]), Vec3::new(m[2][0], m[2][1], m[2][2])];
    [x, y, z].iter().all(|v| (v.dot(*v) - 1.).abs() < 1e-9)
        && x.dot(y).abs() < 1e-9
        && x.dot(z).abs() < 1e-9
        && y.dot(z).abs() < 1e-9
        && (x.cross(y).dot(z) - 1.).abs() < 1e-9
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn placed_mesh_is_lazy_and_composes_from_one_immutable_source() {
        let source = ModelBody::new("Part".into(), kernel::box_solid(Vec3::ZERO, Vec3::new(2., 3., 4.)).unwrap(), 1);
        let a = crate::rigid(Vec3::new(10., 20., 30.), Vec3::ZERO, Vec3::Z, 0.7);
        let b = crate::rigid(Vec3::new(3., -5., 2.), Vec3::ZERO, Vec3::X, 0.4);
        let mut first = ModelBody::new("First".into(), kernel::transform_matrix(&source.body, a).unwrap(), 1);
        first.mesh_placement = Some(Arc::new(PlacedMesh::new(&source, &a).unwrap()));
        let mut nested = ModelBody::new("Nested".into(), kernel::transform_matrix(&first.body, b).unwrap(), 1);
        nested.mesh_placement = Some(Arc::new(PlacedMesh::new(&first, &b).unwrap()));
        assert_eq!(first.display_tol(), source.display_tol());
        assert_eq!(nested.display_tol(), source.display_tol());
        assert!(source.cached_mesh().is_none());
        assert!(first.cached_mesh().is_none());
        let mesh = nested.mesh();
        assert!(source.cached_mesh().is_some(), "the canonical source is warmed on demand");
        assert!(first.cached_mesh().is_none(), "composition does not retain or evaluate intermediate poses");
        let local = source.mesh();
        let matrix = crate::mat_mul(&b, &a);
        for (p, q) in local.positions.iter().zip(&mesh.positions) {
            assert!(crate::apply_point(&matrix, *p).dist(*q) < 1e-8);
        }
        let (other, work) = kernel::profile::capture(|| first.mesh());
        assert_eq!(work.tessellate_ms, 0.);
        assert_eq!(other.tri_face, local.tri_face);
        assert!(Arc::ptr_eq(&mesh, &nested.mesh()));
    }

    #[test]
    fn world_instances_warm_one_definition_and_an_edit_replaces_it() {
        let mut doc = crate::Document::new("instances");
        let component = doc.add_component(Some("Part"), 0).unwrap();
        doc.body_components.insert("Pin".into(), component);
        let first = doc.occurrences[0].id;
        doc.occurrences[0].transform = crate::rigid(Vec3::new(10., 0., 0.), Vec3::ZERO, Vec3::Y, 0.3);
        doc.add_occurrence(component, 0, crate::rigid(Vec3::new(0., 20., 0.), Vec3::ZERO, Vec3::X, 0.6)).unwrap();
        let mut state = crate::ModelState::default();
        state.bodies.push(ModelBody::new("Pin".into(), kernel::cylinder(Vec3::ZERO, Vec3::Z, 2., 8.).unwrap(), 1));
        let mut cache = super::super::PlacedBodies::default();
        let before = super::super::world_state_reusing(&doc, &state, &mut cache);
        assert!(state.bodies[0].cached_mesh().is_none(), "placing bodies must remain lazy");
        let old = before.bodies[0].mesh();
        assert!(state.bodies[0].cached_mesh().is_some());
        let (_, work) = kernel::profile::capture(|| before.bodies[1].mesh());
        assert_eq!(work.tessellate_ms, 0., "the second instance shares the first one's triangulation");
        doc.occurrences.iter_mut().find(|o| o.id == first).unwrap().transform = crate::rigid(Vec3::new(30., 0., 0.), Vec3::ZERO, Vec3::Y, 0.9);
        let moved = super::super::world_state_reusing(&doc, &state, &mut cache);
        let (_, work) = kernel::profile::capture(|| moved.bodies[0].mesh());
        assert_eq!(work.tessellate_ms, 0., "a new pose shares the same definition");
        state.bodies[0] = ModelBody::new("Pin".into(), kernel::cylinder(Vec3::ZERO, Vec3::Z, 4., 8.).unwrap(), 1);
        let edited = super::super::world_state_reusing(&doc, &state, &mut cache);
        assert!(state.bodies[0].cached_mesh().is_none());
        let new = edited.bodies[0].mesh();
        assert!(state.bodies[0].cached_mesh().is_some(), "changed geometry warms its new definition cache");
        assert_ne!(new.bounds(), moved.bodies[0].mesh().bounds());
        assert!(Arc::ptr_eq(&old, &before.bodies[0].mesh()), "editing does not mutate older snapshots");
    }
    #[test]
    fn only_proper_rigid_affine_matrices_reuse_reference_meshes() {
        let source = ModelBody::new("Part".into(), kernel::box_solid(Vec3::ZERO, Vec3::new(2., 3., 4.)).unwrap(), 1);
        assert!(PlacedMesh::new(&source, &crate::IDENTITY).is_some());
        for (col, row, value) in [(0, 0, -1.), (0, 0, 2.), (0, 1, 0.5), (0, 3, 0.1), (3, 3, 2.), (3, 0, f64::NAN), (3, 2, f64::INFINITY)] {
            let mut matrix = crate::IDENTITY;
            matrix[col][row] = value;
            assert!(PlacedMesh::new(&source, &matrix).is_none(), "{matrix:?}");
        }
        // A positive unit determinant alone is insufficient: this scale changes normals and tolerance.
        let mut scale = crate::IDENTITY;
        scale[0][0] = 2.;
        scale[1][1] = 0.5;
        assert!(PlacedMesh::new(&source, &scale).is_none());
    }
}
