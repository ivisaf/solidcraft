//! 3D parallel finishing (`manufacture.setups[*].operations[*].kind`: `parallel_3d`, with
//! `stepover`, `angle`, `stock_to_leave`, `zigzag` and `rest`): passes side by side following the model's
//! shape. Older designs need no transformation: they have no such operation. The last release
//! asks for an update instead of opening a design with this format number and failing to read
//! the operation.
