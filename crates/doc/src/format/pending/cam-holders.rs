//! Holders by their shape: a manufacturing tool's `holder` (`manufacture.tools[*]`), a copy of a
//! holder from the machine's tool library, its `name` and `sections` (`length`, `diameter`,
//! `top_diameter`) from where the tool comes out of it upward. Older designs need no
//! transformation: their tools have none, and a plain holder (`holder_diameter`) is checked as
//! before. The last release asks for an update instead of opening a design with this format
//! number and dropping a tool's holder on the next save.
