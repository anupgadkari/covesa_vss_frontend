//! ExteriorTrunkButton — capacitive trunk-release button above the
//! rear license plate.
//!
//! Owns both the unlocked-cabin direct path and the locked-cabin
//! authenticated path.  Authenticated lookups route through the
//! [`KeySearchArbiterHandle`], not the legacy continuous Zone signal.
//!
//! # Behaviour
//!
//! On a rising edge of `Body.Trunk.ExteriorButton.IsPressed`, branch
//! on the cached `Cabin.LockStatus` and `Cabin.ValetMode.IsActive`:
//!
//! | Cabin lock state                | Valet | Action                                              |
//! |---------------------------------|:-----:|-----------------------------------------------------|
//! | `UNLOCKED` / `DRIVER_UNLOCKED`  |  ❌   | Pulse trunk arbiter + `trunk_unlock` feedback flash |
//! | `UNLOCKED` / `DRIVER_UNLOCKED`  |  ✅   | Silently denied (log only)                          |
//! | `LOCKED` / `DOUBLE_LOCKED`      |  ❌   | Authenticated trunk-outside scan via arbiter;       |
//! |                                 |       | pulse on a non-empty `keys_found`                   |
//! | `LOCKED` / `DOUBLE_LOCKED`      |  ✅   | Silently denied — short-circuit (the trunk arbiter's|
//! |                                 |       | `ValetGate` would drop the publish anyway)          |
//!
//! Valet mode also gates the trunk arbiter via its `ValetGate`
//! `PhysicalGate` (see `arbiter::trunk_arbiter`).  Our short-circuit
//! here saves the arbiter the wasted publish in both unlocked and
//! locked paths.
//!
//! # No state, no NVM
//!
//! Stateless reader of three signals.  Safe boot defaults: lock
//! status `"LOCKED"` (so a missing-signal stance is "auth required")
//! and valet `false` (don't deny without reason).
//!
//! # Lock-state independence
//!
//! Trunk-only open never publishes `Cabin.LockStatus` or
//! `Cabin.LockStatus.LastRequestor`.  AutoRelock's external-source
//! filter never fires on a trunk press, and the cabin lock indicator
//! stays consistent across a "pop-trunk-while-locked" sequence.
//!
//! # Authentication
//!
//! The locked-cabin path submits
//! `AntennaSet::TrunkOutside + SearchMode::Authenticated +
//! Coalescing::Disallowed` to the arbiter.  Coalescing is forbidden
//! on this path because trunk access is security-critical — a stale
//! cached result might miss a key that walked away between the last
//! scan and the button press.

use std::sync::Arc;

use futures::StreamExt;
use tokio::select;

use crate::arbiter::{ActuatorRequest, DomainArbiter, FEEDBACK_REQUEST, TRUNK_OPEN_CMD};
use crate::features::key_search_arbiter::{
    AntennaSet, Coalescing, KeySearchArbiterHandle, SearchMode,
};
use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

const FEATURE_ID: FeatureId = FeatureId::ExteriorTrunkButton;

const BUTTON: VssPath = "Body.Trunk.ExteriorButton.IsPressed";
const LOCK_STATUS: VssPath = "Cabin.LockStatus";
const VALET_MODE: VssPath = "Cabin.ValetMode.IsActive";

pub struct ExteriorTrunkButton<B: SignalBus> {
    bus: Arc<B>,
    trunk_arb: Arc<DomainArbiter>,
    key_search: KeySearchArbiterHandle,
}

impl<B: SignalBus + Send + Sync + 'static> ExteriorTrunkButton<B> {
    pub fn new(
        bus: Arc<B>,
        trunk_arb: Arc<DomainArbiter>,
        key_search: KeySearchArbiterHandle,
    ) -> Self {
        Self {
            bus,
            trunk_arb,
            key_search,
        }
    }

    pub async fn run(self) {
        tracing::info!("ExteriorTrunkButton feature started");

        let mut button_rx = self.bus.subscribe(BUTTON).await;
        let mut lock_rx = self.bus.subscribe(LOCK_STATUS).await;
        let mut valet_rx = self.bus.subscribe(VALET_MODE).await;

        // Safe defaults — see module docs.  `"LOCKED"` so a missing
        // lock-status signal at boot routes the press to the authed
        // path (where it'll be cleanly rejected if no fob is present)
        // rather than firing an unauthenticated trunk open.
        let mut lock_status = "LOCKED".to_string();
        let mut valet_active = false;

        loop {
            select! {
                Some(val) = button_rx.next() => {
                    if !matches!(val, SignalValue::Bool(true)) {
                        continue;
                    }
                    self.on_press(&lock_status, valet_active).await;
                }
                Some(val) = lock_rx.next() => {
                    if let SignalValue::String(s) = val {
                        lock_status = s;
                    }
                }
                Some(val) = valet_rx.next() => {
                    if let SignalValue::Bool(b) = val {
                        valet_active = b;
                    }
                }
                else => break,
            }
        }
    }

    /// Branch on lock state + valet, fire whichever path is
    /// appropriate.
    async fn on_press(&self, lock_status: &str, valet_active: bool) {
        if valet_active {
            tracing::info!(
                lock_status,
                "ExteriorTrunkButton: press denied — valet mode active"
            );
            return;
        }

        match lock_status {
            "UNLOCKED" | "DRIVER_UNLOCKED" => {
                tracing::info!(
                    lock_status,
                    "ExteriorTrunkButton: cabin unlocked — pulsing trunk arbiter"
                );
                self.pulse_trunk_open().await;
            }
            // LOCKED, DOUBLE_LOCKED, or anything unrecognized — run
            // the authenticated path.
            _ => {
                self.try_authenticated_open(lock_status).await;
            }
        }
    }

    /// Locked-cabin auth path: submit a `TrunkOutside` Authenticated
    /// search to the KeySearch arbiter and pulse the trunk on a
    /// non-empty result.
    async fn try_authenticated_open(&self, lock_status: &str) {
        let result = self
            .key_search
            .submit(
                "ExteriorTrunkButton",
                AntennaSet::TrunkOutside,
                SearchMode::Authenticated,
                Coalescing::Disallowed,
            )
            .await;
        let keys = match result {
            Some(r) => r.keys_found,
            None => {
                tracing::warn!("ExteriorTrunkButton: arbiter dropped the request");
                return;
            }
        };
        if keys.is_empty() {
            tracing::info!(
                lock_status,
                "ExteriorTrunkButton: locked + no paired key at trunk — press denied"
            );
            return;
        }
        tracing::info!(
            lock_status,
            keys = keys.len(),
            "ExteriorTrunkButton: locked-cabin auth passed — pulsing trunk arbiter"
        );
        self.pulse_trunk_open().await;
    }

    /// Pulse `Body.Trunk.OpenCmd` through the trunk arbiter as a
    /// momentary edge: request true, then immediately release so the
    /// arbiter publishes true → false and a subsequent press can
    /// re-fire.  Also fires the `trunk_unlock` lock-feedback flash so
    /// the user gets the same visual confirmation as the RKE
    /// TrunkRelease path.
    async fn pulse_trunk_open(&self) {
        let _ = self
            .trunk_arb
            .request(ActuatorRequest {
                signal: TRUNK_OPEN_CMD,
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FEATURE_ID,
            })
            .await;
        let _ = self.trunk_arb.release(TRUNK_OPEN_CMD, FEATURE_ID).await;

        let _ = self
            .bus
            .publish(FEEDBACK_REQUEST, SignalValue::String("trunk_unlock".into()))
            .await;
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;
    use crate::arbiter::trunk_arbiter;
    use crate::features::key_search_arbiter::KeySearchArbiter;
    use std::time::Duration;

    async fn settle() {
        // KeySearchArbiter run_scan sleeps a real 100 ms for
        // TrunkOutside + Authenticated.  Wait long enough for the
        // arbiter task to return through the oneshot.
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
        for _ in 0..16 {
            tokio::task::yield_now().await;
        }
    }

    /// Spin up bus + trunk arbiter + KeySearch arbiter + ETB.
    async fn setup() -> Arc<MockBus> {
        let bus = Arc::new(MockBus::new());
        let (tarb, tarb_fut) = trunk_arbiter(Arc::clone(&bus));
        tokio::spawn(tarb_fut);
        let tarb = Arc::new(tarb);

        let (ksa, ksa_handle, ksa_rx) = KeySearchArbiter::new_with_rx(Arc::clone(&bus));
        tokio::spawn(
            ksa.with_cadence(Duration::from_millis(20), Duration::from_millis(200))
                .run(ksa_rx),
        );

        let feat = ExteriorTrunkButton::new(Arc::clone(&bus), tarb, ksa_handle);
        tokio::spawn(feat.run());
        for _ in 0..32 {
            tokio::task::yield_now().await;
        }
        bus
    }

    fn trunk_was_pulsed(bus: &MockBus) -> bool {
        bus.history()
            .into_iter()
            .any(|(s, v)| s == "Body.Trunk.OpenCmd" && v == SignalValue::Bool(true))
    }

    fn lock_status_was_published(bus: &MockBus) -> bool {
        bus.history().into_iter().any(|(s, _)| s == LOCK_STATUS)
    }

    fn place_fob_at_trunk(bus: &MockBus, slot: u8) {
        let path = match slot {
            1 => "Body.PEPS.Plant.KeyFob.1.Zone",
            2 => "Body.PEPS.Plant.KeyFob.2.Zone",
            3 => "Body.PEPS.Plant.KeyFob.3.Zone",
            4 => "Body.PEPS.Plant.KeyFob.4.Zone",
            _ => panic!("unknown slot"),
        };
        bus.inject(path, SignalValue::String("Trunk".into()));
    }

    // ── Unlocked-cabin path (unchanged from pre-migration) ──────────────

    #[tokio::test]
    async fn unlocked_press_opens_trunk_directly() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            trunk_was_pulsed(&bus),
            "unlocked + valet off: press must pulse Body.Trunk.OpenCmd=true"
        );
        assert!(
            !lock_status_was_published(&bus),
            "trunk-only open must not mutate Cabin.LockStatus"
        );
    }

    #[tokio::test]
    async fn driver_unlocked_press_opens_trunk_directly() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("DRIVER_UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn valet_active_unlocked_press_does_not_open_trunk() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        bus.inject(VALET_MODE, SignalValue::Bool(true));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "valet active + unlocked: press must be silently denied"
        );
    }

    #[tokio::test]
    async fn falling_edge_is_ignored() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("UNLOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(false));
        settle().await;

        assert!(!trunk_was_pulsed(&bus));
    }

    // ── Locked-cabin auth path (new, replaces PassiveEntry path) ────────

    #[tokio::test]
    async fn locked_press_with_fob_at_trunk_authenticates_and_opens() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        place_fob_at_trunk(&bus, 1);
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            trunk_was_pulsed(&bus),
            "locked + paired fob at Trunk: auth must succeed and pulse"
        );
        assert!(
            !lock_status_was_published(&bus),
            "locked auth path must not mutate Cabin.LockStatus"
        );
    }

    #[tokio::test]
    async fn locked_press_without_key_does_not_open_trunk() {
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "locked + no key in trunk zone: press must be silently denied"
        );
    }

    #[tokio::test]
    async fn locked_press_with_fob_at_driver_door_does_not_open_trunk() {
        // Fob is in the wrong zone — TrunkOutside scan returns empty
        // so the press is denied.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject(
            "Body.PEPS.Plant.KeyFob.1.Zone",
            SignalValue::String("LeftFront".into()),
        );
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(!trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn double_locked_press_with_fob_at_trunk_authenticates_and_opens() {
        // DOUBLE_LOCKED behaves identically to LOCKED on the trunk
        // path — the deterrent is the door lock, not the trunk.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("DOUBLE_LOCKED".into()));
        place_fob_at_trunk(&bus, 1);
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn locked_press_with_unpaired_fob_does_not_open_trunk() {
        // Unpaired fobs (Paired=false) must be filtered by the
        // arbiter's authenticated scan.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject("Body.PEPS.Plant.KeyFob.1.Paired", SignalValue::Bool(false));
        place_fob_at_trunk(&bus, 1);
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "unpaired fob at Trunk: auth must reject"
        );
    }

    #[tokio::test]
    async fn valet_active_locked_press_is_silently_denied() {
        // Valet short-circuits even the locked-auth path so the
        // arbiter isn't wasted on a search that the trunk arbiter
        // would just drop downstream.
        let bus = setup().await;
        bus.inject(LOCK_STATUS, SignalValue::String("LOCKED".into()));
        bus.inject(VALET_MODE, SignalValue::Bool(true));
        place_fob_at_trunk(&bus, 1);
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(!trunk_was_pulsed(&bus));
    }

    #[tokio::test]
    async fn missing_lock_status_at_boot_uses_auth_path() {
        // Boot default is `"LOCKED"`.  Without an explicit lock-status
        // publish, a press must route to the authed path — and with
        // no fob nearby, the press is silently denied.
        let bus = setup().await;
        // Don't inject LOCK_STATUS at all.
        settle().await;
        bus.clear_history();

        bus.inject(BUTTON, SignalValue::Bool(true));
        settle().await;

        assert!(
            !trunk_was_pulsed(&bus),
            "boot-default 'LOCKED' + no fob: press must be denied"
        );
    }
}
