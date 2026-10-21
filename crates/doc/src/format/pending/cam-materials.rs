//! Cutting data per material: a manufacturing tool's `cutting` (`manufacture.tools[*]`: for each
//! material its spindle speed, feeds, stepdown and stepover, replacing the tool's own where given)
//! and a setup's stock `material` (`manufacture.setups[*]`). Older designs need no transformation:
//! their tools have only their own cutting data and their setups no material. The last release asks
//! for an update instead of opening a design with this format number and dropping the materials on
//! the next save.
