//! Tier 1 E2E test runner — in-process cucumber-rs with Tokio virtual time.
//!
//! Spawns the full feature stack (arbiter + features + plant model) on
//! a MockBus inside the test process.  Uses `start_paused = true` so
//! `tokio::time::advance()` gives deterministic control over blink
//! cadence without real-time sleeps.
//!
//! Run:  cargo test --test e2e
//!
//! Feature files are read from `../features/` (repo root).

mod steps;

use cucumber::World;
use steps::VssWorld;

fn main() {
    // Manual runtime builder so we can enable `start_paused`.
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .start_paused(true)
        .build()
        .expect("failed to build Tokio runtime");

    rt.block_on(async {
        VssWorld::cucumber()
            .run_and_exit("../features/turn_indicator.feature")
            .await;
    });

    // Second pass for hazard scenarios (separate runtime so virtual
    // time resets cleanly).
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .start_paused(true)
        .build()
        .expect("failed to build Tokio runtime");

    rt.block_on(async {
        VssWorld::cucumber()
            .run_and_exit("../features/hazard.feature")
            .await;
    });
}
