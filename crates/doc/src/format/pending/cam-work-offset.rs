//! A setup's work offset (`manufacture.setups[*].work_offset`: 55 to 59 for `G55` to `G59`; left
//! out for `G54`). Older designs need no transformation: without it a setup runs in `G54`, as all
//! did. The last release asks for an update instead of opening a design whose setup it would
//! write in the wrong work offset.
