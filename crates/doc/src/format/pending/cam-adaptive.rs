//! An adaptive clearing operation (`manufacture.setups[*].operations[*].kind`: `adaptive`, with
//! `stepover_percent`, `climb`, `stock_to_leave`, `finish`, `helix_angle`, `helix_max`,
//! `helix_min` and `rest_from`). Older designs need no transformation: they have no such
//! operation. The last release asks for an update instead of opening a design with this format
//! number and failing to read the operation.
