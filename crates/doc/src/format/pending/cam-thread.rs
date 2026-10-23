//! A thread milling operation (`manufacture.setups[*].operations[*].kind`: `thread`, with
//! `pitch`, `diameter`, `internal`, `climb`, `passes` and `left_hand`). Older designs need no transformation:
//! they have no such operation. The last release asks for an update instead of opening a design
//! with this format number and failing to read the operation.
