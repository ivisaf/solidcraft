//! Tabs, finishing passes and a number of depth passes for manufacturing operations
//! (`manufacture.setups[*].operations[*]`): a contour's `tabs` (`count`, `width`, `height`), a
//! contour's or a pocket's `finish` allowance, and an operation's `passes` (instead of its
//! stepdown). Older designs need no transformation: their operations have none of these. The last
//! release asks for an update instead of opening a design with this format number and dropping
//! them on the next save.
