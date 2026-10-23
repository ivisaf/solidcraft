//! A contour's lead-in and lead-out arcs, dogbones and onion skin
//! (`manufacture.setups[*].operations[*].kind`: `lead`, `dogbones`, `skin` and `skin_last` on a
//! contour, `dogbones` on a pocket; each left out when not used). Older designs need no
//! transformation: without them an operation cuts as it did. The last release asks for an update
//! instead of opening a design whose operations it would cut without them.
