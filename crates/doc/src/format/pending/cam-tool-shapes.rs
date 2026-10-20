//! A manufacturing tool's shape beyond its diameter (`manufacture.tools[*]`): the kind `v_bit`,
//! the point's included angle (`tip_angle`, a V-bit's or a drill's), a V-bit's flat at its point
//! (`tip_diameter`), and the shank's diameter and the length out of the holder
//! (`shank_diameter`, `length`). Older designs need no transformation: their tools have none of
//! these, and a drill's point stays 118°. The last release asks for an update instead of opening
//! a design with this format number and dropping a V-bit or its shape on the next save.
