//! 3D Z-level finishing (`manufacture.setups[*].operations[*].kind`: `zlevel_3d`, with `step`,
//! `stock_to_leave`, `steep`, `climb` and `rest`): the model's steep walls finished level by level. Older
//! designs need no transformation: they have no such operation. The last release asks for an
//! update instead of opening a design with this format number and failing to read the operation.
