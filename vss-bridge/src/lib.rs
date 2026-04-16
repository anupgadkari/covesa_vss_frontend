//! vss-bridge library — COVESA VSS body controller platform (L5).
//!
//! Re-exports all modules so integration tests and the binary can share
//! the same crate root.

pub mod config;
pub mod ipc_message;
pub mod signal_bus;
pub mod signal_ids;
pub mod arbiter;
pub mod adapters;
pub mod features;
pub mod plant_models;
pub mod kuksa_sync;
pub mod sleep_inhibit;
pub mod ws_bridge;
