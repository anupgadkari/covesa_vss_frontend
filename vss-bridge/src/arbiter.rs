//! Domain-based Signal Arbiters — per-actuator priority resolution grouped by domain.
//!
//! Each domain (Lighting, Doors, Horn, Comfort) gets its own `DomainArbiter` instance.
//! Within a domain, each actuator signal tracks the current highest-priority request.
//! A new request replaces the winner if its priority >= the current winner's priority.
//!
//! Feature business logic calls `arbiter.request(...)` — the arbiter validates the
//! (feature_id, signal, priority) tuple against the domain's static allow-list before
//! forwarding the arbitrated value to the SignalBus.

use std::collections::HashMap;
use std::sync::Arc;

use tokio::sync::mpsc;
use tracing;

use crate::ipc_message::{FeatureId, Priority, SignalValue};
use crate::signal_bus::{SignalBus, VssPath};

// ---------------------------------------------------------------------------
// ActuatorRequest — what features submit to the arbiter
// ---------------------------------------------------------------------------

/// A request from a feature module to control an actuator signal.
#[derive(Debug, Clone)]
pub struct ActuatorRequest {
    pub signal: VssPath,
    pub value: SignalValue,
    pub priority: Priority,
    pub feature_id: FeatureId,
}

// ---------------------------------------------------------------------------
// AllowEntry — static priority table row
// ---------------------------------------------------------------------------

/// One row in a domain's static allow-list.
/// Defines which (feature, signal, priority) combinations are permitted.
#[derive(Debug, Clone)]
pub struct AllowEntry {
    pub feature_id: FeatureId,
    pub signal: VssPath,
    pub priority: Priority,
}

// ---------------------------------------------------------------------------
// DomainArbiter — one per actuator domain
// ---------------------------------------------------------------------------

/// A domain-scoped arbiter that resolves per-actuator priority conflicts.
///
/// Features submit `ActuatorRequest` via the `request()` method. The arbiter's
/// background loop validates against the allow-list, tracks per-signal winners,
/// and publishes arbitrated values to the `SignalBus`.
pub struct DomainArbiter {
    pub name: &'static str,
    tx: mpsc::Sender<ActuatorRequest>,
}

impl DomainArbiter {
    /// Create a new domain arbiter with its static allow-list.
    ///
    /// Returns the arbiter handle (for features to submit requests) and
    /// a future that must be spawned as a tokio task.
    pub fn new<B: SignalBus>(
        name: &'static str,
        allow_list: Vec<AllowEntry>,
        bus: Arc<B>,
    ) -> (Self, impl std::future::Future<Output = ()>) {
        let (tx, rx) = mpsc::channel::<ActuatorRequest>(256);

        let arbiter = Self { name, tx };
        let loop_fut = arbiter_loop(name, allow_list, bus, rx);

        (arbiter, loop_fut)
    }

    /// Submit an actuator request. Fire-and-forget from the feature's perspective.
    /// Returns an error only if the arbiter loop has been dropped.
    pub async fn request(&self, req: ActuatorRequest) -> anyhow::Result<()> {
        self.tx
            .send(req)
            .await
            .map_err(|_| anyhow::anyhow!("{}: arbiter channel closed", self.name))
    }
}

// ---------------------------------------------------------------------------
// Arbiter resolution loop
// ---------------------------------------------------------------------------

/// Background task that receives requests, validates them, resolves priority,
/// and publishes winning values downstream.
async fn arbiter_loop<B: SignalBus>(
    name: &'static str,
    allow_list: Vec<AllowEntry>,
    bus: Arc<B>,
    mut rx: mpsc::Receiver<ActuatorRequest>,
) {
    // Per-signal current winner: signal → (winning request)
    let mut winners: HashMap<VssPath, ActuatorRequest> = HashMap::new();

    tracing::info!(domain = name, signals = allow_list.len(), "arbiter started");

    while let Some(req) = rx.recv().await {
        // 1. Validate against the allow-list
        let allowed = allow_list.iter().any(|entry| {
            entry.feature_id == req.feature_id
                && entry.signal == req.signal
                && entry.priority == req.priority
        });

        if !allowed {
            tracing::warn!(
                domain = name,
                feature = ?req.feature_id,
                signal = req.signal,
                priority = ?req.priority,
                "request rejected — not in allow-list"
            );
            continue;
        }

        // 2. Priority resolution: new request wins if priority >= current winner
        let should_emit = match winners.get(req.signal) {
            None => true,
            Some(current) => (req.priority as u8) >= (current.priority as u8),
        };

        if should_emit {
            tracing::debug!(
                domain = name,
                feature = ?req.feature_id,
                signal = req.signal,
                value = ?req.value,
                priority = ?req.priority,
                "arbiter: new winner"
            );

            if let Err(e) = bus.publish(req.signal, req.value).await {
                tracing::error!(
                    domain = name,
                    signal = req.signal,
                    error = %e,
                    "failed to publish arbitrated value"
                );
            }

            winners.insert(req.signal, req);
        } else {
            tracing::debug!(
                domain = name,
                feature = ?req.feature_id,
                signal = req.signal,
                priority = ?req.priority,
                "arbiter: lower priority, suppressed"
            );
        }
    }

    tracing::info!(domain = name, "arbiter loop ended");
}

// ---------------------------------------------------------------------------
// Domain factory functions — static priority tables
// ---------------------------------------------------------------------------

/// Create the Lighting domain arbiter.
///
/// Covers: direction indicators, low/high beam, DRL, hazard signaling.
/// Contention on direction indicators: Hazard(3), LockFeedback(3, overlay), Turn(2).
/// LockFeedback uses HIGH to overlay its brief pattern on hazard/turn, then releases.
pub fn lighting_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let allow_list = vec![
        // Direction indicators — 3-way contention
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::TurnIndicator,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::Medium,
        },
        AllowEntry {
            feature_id: FeatureId::TurnIndicator,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::Medium,
        },
        // LockFeedback uses HIGH to overlay its brief lock/unlock pattern
        // on top of active hazard or turn signaling, then self-releases.
        AllowEntry {
            feature_id: FeatureId::LockFeedback,
            signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
            priority: Priority::High,
        },
        AllowEntry {
            feature_id: FeatureId::LockFeedback,
            signal: "Body.Lights.DirectionIndicator.Right.IsSignaling",
            priority: Priority::High,
        },
        // Hazard master signal
        AllowEntry {
            feature_id: FeatureId::Hazard,
            signal: "Body.Lights.Hazard.IsSignaling",
            priority: Priority::High,
        },
        // Low beam
        AllowEntry {
            feature_id: FeatureId::LowBeam,
            signal: "Body.Lights.Beam.Low.IsOn",
            priority: Priority::Medium,
        },
        // High beam
        AllowEntry {
            feature_id: FeatureId::HighBeam,
            signal: "Body.Lights.Beam.High.IsOn",
            priority: Priority::Medium,
        },
        // DRL
        AllowEntry {
            feature_id: FeatureId::Drl,
            signal: "Body.Lights.Running.IsOn",
            priority: Priority::Medium,
        },
    ];

    DomainArbiter::new("Lighting", allow_list, bus)
}

/// Create the Door/Lock domain arbiter.
///
/// Covers: all 4 door lock signals.
/// Contention: Peps(3) > AutoLock(2).
pub fn door_lock_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    let door_signals: &[VssPath] = &[
        "Body.Doors.Row1.Left.IsLocked",
        "Body.Doors.Row1.Right.IsLocked",
        "Body.Doors.Row2.Left.IsLocked",
        "Body.Doors.Row2.Right.IsLocked",
    ];

    let mut allow_list = Vec::new();
    for &signal in door_signals {
        allow_list.push(AllowEntry {
            feature_id: FeatureId::Peps,
            signal,
            priority: Priority::High,
        });
        allow_list.push(AllowEntry {
            feature_id: FeatureId::AutoLock,
            signal,
            priority: Priority::Medium,
        });
    }

    DomainArbiter::new("DoorLock", allow_list, bus)
}

/// Create the Horn domain arbiter.
///
/// Single-feature domain today. Arbiter ensures uniform pattern
/// and validates priority claims for future multi-feature contention.
pub fn horn_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    // No competing features today — pass-through with validation.
    // When a Horn feature is added, its allow entry goes here.
    let allow_list = vec![];

    DomainArbiter::new("Horn", allow_list, bus)
}

/// Create the Comfort domain arbiter.
///
/// Covers: seat heating/ventilation, HVAC, cabin lights, sunroof.
/// No contention today — pass-through with validation. Adding a
/// second feature to any comfort actuator requires only an allow entry here.
pub fn comfort_arbiter<B: SignalBus>(
    bus: Arc<B>,
) -> (DomainArbiter, impl std::future::Future<Output = ()>) {
    // No competing features today — pass-through with validation.
    let allow_list = vec![];

    DomainArbiter::new("Comfort", allow_list, bus)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapters::mock::MockBus;

    /// Helper: spawn a lighting arbiter on MockBus, return the handle.
    async fn setup_lighting() -> (DomainArbiter, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let (arbiter, loop_fut) = lighting_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        // Give the loop a moment to start
        tokio::task::yield_now().await;
        (arbiter, bus)
    }

    #[tokio::test]
    async fn high_priority_wins_over_medium() {
        let (arbiter, bus) = setup_lighting().await;

        // Turn (medium) requests left indicator ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Hazard (high) requests left indicator OFF — should win
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        // First: Turn's true, Second: Hazard's false (overwrites)
        assert_eq!(
            history[0],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(true)
            )
        );
        assert_eq!(
            history[1],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(false)
            )
        );
    }

    #[tokio::test]
    async fn medium_priority_suppressed_by_existing_high() {
        let (arbiter, bus) = setup_lighting().await;

        // Hazard (high) claims left indicator
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Turn (medium) tries the same signal — should be suppressed
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Only Hazard's request was published
        assert_eq!(history.len(), 1);
        assert_eq!(
            history[0],
            (
                "Body.Lights.DirectionIndicator.Left.IsSignaling",
                SignalValue::Bool(true)
            )
        );
    }

    #[tokio::test]
    async fn lock_feedback_overlays_on_active_hazard() {
        let (arbiter, bus) = setup_lighting().await;

        // Hazard (high) claims left indicator ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::Hazard,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // LockFeedback (high, overlay) takes over — should publish (equal priority wins)
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::High,
                feature_id: FeatureId::LockFeedback,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        // Both published: Hazard ON, then LockFeedback OFF (overlay)
        assert_eq!(history.len(), 2);
        assert_eq!(history[0].1, SignalValue::Bool(true));  // Hazard
        assert_eq!(history[1].1, SignalValue::Bool(false)); // LockFeedback overlay
    }

    #[tokio::test]
    async fn different_signals_do_not_interfere() {
        let (arbiter, bus) = setup_lighting().await;

        // LowBeam claims low beam
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.Low.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::LowBeam,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // HighBeam claims high beam — independent signal, both should publish
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.High.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::HighBeam,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        assert_eq!(
            history[0],
            ("Body.Lights.Beam.Low.IsOn", SignalValue::Bool(true))
        );
        assert_eq!(
            history[1],
            ("Body.Lights.Beam.High.IsOn", SignalValue::Bool(true))
        );
    }

    #[tokio::test]
    async fn equal_priority_latest_wins() {
        let (arbiter, bus) = setup_lighting().await;

        // Turn (medium) requests left ON
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // Turn (medium) requests left OFF — same priority, should replace
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(false),
                priority: Priority::Medium,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[1].1, SignalValue::Bool(false));
    }

    #[tokio::test]
    async fn request_rejected_if_not_in_allow_list() {
        let (arbiter, bus) = setup_lighting().await;

        // AutoLock tries to control a lighting signal — not allowed
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.Beam.Low.IsOn",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 0, "rejected request should not publish");
    }

    #[tokio::test]
    async fn wrong_priority_rejected() {
        let (arbiter, bus) = setup_lighting().await;

        // TurnIndicator tries to claim HIGH priority — table says MEDIUM
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Lights.DirectionIndicator.Left.IsSignaling",
                value: SignalValue::Bool(true),
                priority: Priority::High,
                feature_id: FeatureId::TurnIndicator,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 0, "wrong priority should be rejected");
    }

    #[tokio::test]
    async fn door_lock_arbiter_peps_wins_over_autolock() {
        let bus = Arc::new(MockBus::new());
        let (arbiter, loop_fut) = door_lock_arbiter(Arc::clone(&bus));
        tokio::spawn(loop_fut);
        tokio::task::yield_now().await;

        // AutoLock (medium) locks driver door
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Doors.Row1.Left.IsLocked",
                value: SignalValue::Bool(true),
                priority: Priority::Medium,
                feature_id: FeatureId::AutoLock,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        // PEPS (high) unlocks driver door — should win
        arbiter
            .request(ActuatorRequest {
                signal: "Body.Doors.Row1.Left.IsLocked",
                value: SignalValue::Bool(false),
                priority: Priority::High,
                feature_id: FeatureId::Peps,
            })
            .await
            .unwrap();
        tokio::task::yield_now().await;

        let history = bus.history();
        assert_eq!(history.len(), 2);
        assert_eq!(
            history[1],
            (
                "Body.Doors.Row1.Left.IsLocked",
                SignalValue::Bool(false)
            )
        );
    }
}
