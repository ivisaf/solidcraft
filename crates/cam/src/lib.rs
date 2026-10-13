//! Manufacturing (CAM) foundation for SolidCraft: cutting tools, toolpaths, 2.5D milling and
//! drilling operations on planar regions, G-code output through declarative post processors,
//! and a G-code reader that checks what was written.
//!
//! Everything here works in a setup's work coordinates: millimetres, tool axis along +Z, feeds
//! in mm/min. The crate knows nothing of documents or the B-rep kernel; the engine hands it
//! regions and points.
//!
//! The region offsetting is our own: every boundary piece is offset, corners that open away
//! from the offset side get a joining arc, the pieces are cut where they cross, and a piece is
//! kept when its midpoint is no closer to the boundary than the offset distance. Lines and
//! arcs stay lines and arcs, so walls come out as straight moves and true arcs.
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]
#![forbid(unsafe_code)]

mod check;
mod gcode;
mod offset;
mod ops;
mod path;
mod piece;
mod post;
mod sim;
mod tool;

pub use check::verify;
pub use gcode::{Operation, Program, emit, parse};
pub use offset::offset_regions;
pub use ops::{ContourParams, Cut, DrillParams, Feeds, Heights, PocketParams, Side, contour, drill, face, pocket};
pub use path::{Move, PathStats, Toolpath, arc_sweep};
pub use post::{CommentStyle, LineNumbers, Post, Units};
pub use sim::{Comparison, CutReport, HeightField, MAX_CELLS, compare, simulate};
pub use tool::{Tool, ToolKind};

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CamError {
    #[error("invalid value: {0}")]
    Invalid(String),
    #[error("not supported yet: {0}")]
    Unsupported(String),
    #[error("too large: {0}")]
    TooLarge(String),
    /// A toolpath that breaks a safety rule (a rapid through material, a cut below the
    /// bottom). Operations check their own output, so this reports a defect rather than bad input.
    #[error("unsafe toolpath: {0}")]
    Unsafe(String),
    #[error("G-code line {0}: {1}")]
    Parse(usize, String),
}

pub type Result<T> = std::result::Result<T, CamError>;

pub(crate) fn invalid<T>(why: impl Into<String>) -> Result<T> {
    Err(CamError::Invalid(why.into()))
}

#[cfg(test)]
mod tests;
