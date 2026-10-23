//! A V-carve operation (`manufacture.setups[*].operations[*].kind`: `vcarve`, with `stepover`,
//! `max_depth` and `climb`). Older designs need no transformation: they have no such operation.
//! The last release asks for an update instead of opening a design with this format number and
//! failing to read the operation.
