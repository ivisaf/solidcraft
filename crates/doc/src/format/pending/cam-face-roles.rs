//! Face roles in 3D operations: a manufacturing operation's `face_roles`
//! (`manufacture.setups[*].operations[*]`), each a picked `face` (`body`, `name`, `at`) and its
//! `role`: `ignore`, `avoid` with a `clearance`, or `leave` with its own `stock_to_leave`. Older
//! designs need no transformation: their operations have none, and every face is machined. The
//! last release asks for an update instead of opening a design with this format number and
//! dropping the roles on the next save.
