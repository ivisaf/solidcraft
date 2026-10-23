//! A setup's clamps (`manufacture.setups[*].clamps`: boxes `{name?, min: [x, y], max: [x, y], top}`
//! in its work coordinates that no tool may enter; left out when there are none). Older designs
//! need no transformation: without them nothing is checked, as before. The last release asks for
//! an update instead of opening a design whose clamps it would drop.
