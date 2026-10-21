//! A manufacturing operation that mills round holes larger than its tool on a helix
//! (`manufacture.setups[*].operations[*].kind`: `bore`, with `climb`, `stock_to_leave` and
//! `pitch`, the depth of one turn). Older designs need no transformation: they have no such
//! operation. The last release asks for an update instead of opening a design with this format
//! number and failing to read the operation.
