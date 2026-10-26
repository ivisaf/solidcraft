//! Stock shapes and the part's bodies: a manufacturing setup's `stock.shape`
//! (`manufacture.setups[*]`: `offset`, the box round the part grown by the offsets; `box`, a box
//! of a set size; `round`, a round bar along the tool axis) and its `bodies`, the bodies that are
//! the part. Older designs need no transformation: their stock is the box round every body. The
//! last release asks for an update instead of opening a design with this format number and
//! dropping the shape and the bodies on the next save.
