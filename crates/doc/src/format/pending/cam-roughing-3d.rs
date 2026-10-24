//! 3D adaptive roughing (`manufacture.setups[*].operations[*].kind`: `adaptive` with `model:
//! true`): an adaptive operation that roughs level by level down to the model's shape. Older
//! designs need no transformation: their adaptive operations have no `model` and stay pockets.
//! The last release asks for an update instead of opening a design with this format number and
//! roughing such an operation as a pocket.
