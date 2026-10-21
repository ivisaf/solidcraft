//! Rest machining for pockets (`manufacture.setups[*].operations[*].kind`, a pocket's
//! `rest_from`: the number of the larger tool that cleared the pocket before, so that only what
//! it left is cut). Older designs need no transformation: their pockets have none. The last
//! release asks for an update instead of opening a design with this format number and cutting the
//! whole pocket again.
