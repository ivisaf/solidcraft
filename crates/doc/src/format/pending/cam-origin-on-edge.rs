//! A work origin on the model: a manufacturing setup's `origin` of type `edge`
//! (`manufacture.setups[*].origin`: a body's edge by its persistent name, which of its points,
//! the end nearest `at`, its middle or the centre of its circle, and where along the tool axis,
//! `z`). Older designs need no transformation: their origins are world points or on a box. The
//! last release asks for an update instead of opening a design with this format number.
