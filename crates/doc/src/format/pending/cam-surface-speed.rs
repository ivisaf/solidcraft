//! Cutting data as surface speed and chip load: a manufacturing tool's `surface_speed` (m/min)
//! and `chip_load` (mm per tooth), on the tool and in its `cutting` per material
//! (`manufacture.tools[*]`), which operations work their spindle speed and feed out from for the
//! tool's diameter and flutes. Older designs need no transformation: their tools give the spindle
//! speed and feed as they are. The last release asks for an update instead of opening a design
//! with this format number and dropping the two on the next save.
