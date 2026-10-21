//! A manufacturing operation that chamfers top edges with a V-bit or a drill
//! (`manufacture.setups[*].operations[*].kind`: `chamfer`, with `side`, `climb`, `width`,
//! `tip_offset` and `ramp_angle`). Older designs need no transformation: they have no such
//! operation. The last release asks for an update instead of opening a design with this format
//! number and failing to read the operation.
