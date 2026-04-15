//! Plant models — software simulations of physical hardware behavior.
//!
//! In production, the M7 safety core (or smart actuator firmware) drives
//! lamp transistors, reads bulb-out diagnostics, and reports actual lamp
//! state back to the application layer. While running on dev hosts (or
//! before M7 firmware is available), these plant models stand in for
//! that hardware and close the loop on the SignalBus.
//!
//! Plant models subscribe to arbitrated actuator outputs (the "intent"
//! signals) and publish corresponding feedback signals (lamp state,
//! defect flags, etc.) directly to the bus — they do NOT go through
//! the arbiter, since they represent physical hardware.
pub mod blink_relay;
