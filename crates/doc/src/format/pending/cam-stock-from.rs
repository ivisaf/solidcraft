//! A setup's stock taken from an earlier setup's result (`manufacture.setups[*].stock_from`: that
//! setup's id; left out when the stock is the box round the part). Older designs need no
//! transformation: without it a setup starts from its own stock box, as all did. The last release
//! asks for an update instead of opening a design whose second setup it would simulate from an
//! uncut block.
