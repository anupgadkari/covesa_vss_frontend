//! SleepInhibitManager — tracks active wake claims from features and
//! subsystems to prevent the A53 from powering down while work is in
//! progress.
//!
//! Features acquire a [`SleepInhibitGuard`] via [`SleepInhibitManager::acquire`].
//! The guard holds the claim for its lifetime (RAII). When all guards are
//! dropped, the manager knows the A53 is ready to sleep.
//!
//! Each claim has a name (for logging/diagnostics) and a maximum hold
//! duration. If a guard is held beyond its max duration, the manager
//! force-expires it and logs a warning. This prevents a stuck feature
//! from draining the 12V battery.
//!
//! The manager exposes:
//! - [`active_claims`]: current snapshot of all active inhibitors
//! - [`is_sleep_ready`]: true when no claims are active
//! - [`wait_until_sleep_ready`]: async — resolves when all claims drop
//! - An internal reaper task that force-expires overdue claims
//!
//! Integration:
//! - The power manager (or main loop) calls `wait_until_sleep_ready()`
//!   before signalling the M7 that the A53 is ready to power down.
//! - Features call `acquire()` before starting timed work (AutoRelock
//!   timer, LockFeedback blink, DoorLock in-flight operation, etc.)
//!   and drop the guard when done.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::{watch, Mutex};

/// Global counter for unique claim IDs.
static NEXT_CLAIM_ID: AtomicU64 = AtomicU64::new(1);

/// Metadata for a single active sleep inhibitor claim.
#[derive(Debug, Clone)]
pub struct ClaimInfo {
    /// Human-readable name for logging (e.g., "AutoRelockTimer").
    pub name: String,
    /// When the claim was acquired.
    pub acquired_at: Instant,
    /// Maximum allowed hold duration. After this, the reaper expires it.
    pub max_hold: Duration,
}

/// RAII guard that holds a sleep inhibitor claim. When dropped, the claim
/// is released and the manager re-evaluates sleep readiness.
///
/// Can also be explicitly released via [`SleepInhibitGuard::release`].
pub struct SleepInhibitGuard {
    claim_id: u64,
    manager: Arc<SleepInhibitInner>,
}

impl SleepInhibitGuard {
    /// Explicitly release the claim before the guard goes out of scope.
    /// Equivalent to dropping the guard, but more readable in async code
    /// where drop timing can be subtle.
    pub fn release(self) {
        // Drop triggers the release via the Drop impl.
        drop(self);
    }
}

impl Drop for SleepInhibitGuard {
    fn drop(&mut self) {
        let inner = Arc::clone(&self.manager);
        let claim_id = self.claim_id;

        // We can't do async work in Drop, so spawn a task.
        // The claim removal is quick (just a HashMap remove + watch update).
        tokio::spawn(async move {
            inner.remove_claim(claim_id).await;
        });
    }
}

/// Internal shared state for the manager.
struct SleepInhibitInner {
    claims: Mutex<HashMap<u64, ClaimInfo>>,
    /// Sender side of a watch channel. The current value is the number of
    /// active claims. Receivers can await transitions to 0.
    claim_count_tx: watch::Sender<usize>,
}

impl SleepInhibitInner {
    async fn remove_claim(&self, claim_id: u64) {
        let mut claims = self.claims.lock().await;
        if let Some(info) = claims.remove(&claim_id) {
            tracing::debug!(
                claim = %info.name,
                held_ms = info.acquired_at.elapsed().as_millis(),
                "sleep inhibitor released"
            );
        }
        let count = claims.len();
        // Ignore send error — receiver may have been dropped.
        let _ = self.claim_count_tx.send(count);
    }
}

/// Manages sleep inhibitor claims from features and subsystems.
///
/// Shared via `Arc<SleepInhibitManager>` — clone-cheap, all methods take `&self`.
pub struct SleepInhibitManager {
    inner: Arc<SleepInhibitInner>,
    claim_count_rx: watch::Receiver<usize>,
}

impl SleepInhibitManager {
    /// Create a new manager and spawn the reaper task.
    ///
    /// `reaper_interval` controls how often the reaper checks for overdue
    /// claims. A reasonable default is 1 second.
    pub fn new(reaper_interval: Duration) -> Arc<Self> {
        let (claim_count_tx, claim_count_rx) = watch::channel(0usize);

        let inner = Arc::new(SleepInhibitInner {
            claims: Mutex::new(HashMap::new()),
            claim_count_tx,
        });

        let manager = Arc::new(Self {
            inner: Arc::clone(&inner),
            claim_count_rx,
        });

        // Spawn the reaper task
        let reaper_inner = Arc::clone(&inner);
        tokio::spawn(async move {
            reaper_loop(reaper_inner, reaper_interval).await;
        });

        manager
    }

    /// Acquire a sleep inhibitor claim. Returns a guard that holds the
    /// claim until dropped.
    ///
    /// # Arguments
    /// - `name`: Human-readable identifier (e.g., "AutoRelockTimer")
    /// - `max_hold`: Maximum duration this claim may be held. The reaper
    ///   will force-expire it after this time.
    pub async fn acquire(&self, name: &str, max_hold: Duration) -> SleepInhibitGuard {
        let claim_id = NEXT_CLAIM_ID.fetch_add(1, Ordering::Relaxed);
        let info = ClaimInfo {
            name: name.to_owned(),
            acquired_at: Instant::now(),
            max_hold,
        };

        tracing::debug!(
            claim = %info.name,
            max_hold_secs = max_hold.as_secs(),
            "sleep inhibitor acquired"
        );

        let mut claims = self.inner.claims.lock().await;
        claims.insert(claim_id, info);
        let count = claims.len();
        let _ = self.inner.claim_count_tx.send(count);
        drop(claims);

        SleepInhibitGuard {
            claim_id,
            manager: Arc::clone(&self.inner),
        }
    }

    /// Returns true if no inhibitor claims are active (A53 may sleep).
    pub async fn is_sleep_ready(&self) -> bool {
        self.inner.claims.lock().await.is_empty()
    }

    /// Async wait — resolves when all active claims have been released
    /// (or force-expired by the reaper).
    pub async fn wait_until_sleep_ready(&self) {
        let mut rx = self.claim_count_rx.clone();
        // If already zero, return immediately.
        if *rx.borrow() == 0 {
            return;
        }
        // Wait for the count to reach 0.
        while rx.changed().await.is_ok() {
            if *rx.borrow() == 0 {
                return;
            }
        }
    }

    /// Returns a snapshot of all currently active claims (for diagnostics).
    pub async fn active_claims(&self) -> Vec<ClaimInfo> {
        self.inner.claims.lock().await.values().cloned().collect()
    }

    /// Returns the number of currently active claims.
    pub async fn active_count(&self) -> usize {
        self.inner.claims.lock().await.len()
    }
}

/// Periodically checks for overdue claims and force-expires them.
async fn reaper_loop(inner: Arc<SleepInhibitInner>, interval: Duration) {
    loop {
        tokio::time::sleep(interval).await;

        let mut claims = inner.claims.lock().await;
        let mut expired = Vec::new();

        for (&id, info) in claims.iter() {
            if info.acquired_at.elapsed() > info.max_hold {
                tracing::warn!(
                    claim = %info.name,
                    held_ms = info.acquired_at.elapsed().as_millis(),
                    max_hold_ms = info.max_hold.as_millis(),
                    "sleep inhibitor force-expired (exceeded max hold time)"
                );
                expired.push(id);
            }
        }

        for id in expired {
            claims.remove(&id);
        }

        let count = claims.len();
        let _ = inner.claim_count_tx.send(count);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn acquire_and_release() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));
        assert!(mgr.is_sleep_ready().await);

        let guard = mgr.acquire("TestClaim", Duration::from_secs(10)).await;
        assert!(!mgr.is_sleep_ready().await);
        assert_eq!(mgr.active_count().await, 1);

        guard.release();
        // Give the spawned Drop task a chance to run.
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(mgr.is_sleep_ready().await);
        assert_eq!(mgr.active_count().await, 0);
    }

    #[tokio::test]
    async fn multiple_claims_all_must_release() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));

        let g1 = mgr
            .acquire("AutoRelockTimer", Duration::from_secs(60))
            .await;
        let g2 = mgr
            .acquire("LockFeedbackBlink", Duration::from_secs(5))
            .await;
        let g3 = mgr
            .acquire("DoorLockInFlight", Duration::from_secs(2))
            .await;
        assert_eq!(mgr.active_count().await, 3);

        g1.release();
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert_eq!(mgr.active_count().await, 2);
        assert!(!mgr.is_sleep_ready().await);

        g2.release();
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert_eq!(mgr.active_count().await, 1);

        g3.release();
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(mgr.is_sleep_ready().await);
    }

    #[tokio::test]
    async fn wait_until_sleep_ready_resolves() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));

        let guard = mgr.acquire("ShortTask", Duration::from_secs(10)).await;

        let mgr2 = Arc::clone(&mgr);
        let waiter = tokio::spawn(async move {
            mgr2.wait_until_sleep_ready().await;
            true
        });

        // Release after a short delay
        tokio::time::sleep(Duration::from_millis(50)).await;
        guard.release();

        let result = tokio::time::timeout(Duration::from_millis(200), waiter)
            .await
            .expect("timed out waiting for sleep ready")
            .expect("task panicked");
        assert!(result);
    }

    #[tokio::test]
    async fn wait_when_already_ready() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));

        // Should resolve immediately — no claims active.
        tokio::time::timeout(Duration::from_millis(50), mgr.wait_until_sleep_ready())
            .await
            .expect("should resolve immediately when no claims");
    }

    #[tokio::test]
    async fn reaper_expires_overdue_claims() {
        // Reaper checks every 50ms; claim has max hold of 100ms.
        let mgr = SleepInhibitManager::new(Duration::from_millis(50));

        let _guard = mgr
            .acquire("StuckFeature", Duration::from_millis(100))
            .await;
        assert_eq!(mgr.active_count().await, 1);

        // Wait for the reaper to expire it (100ms max_hold + 50ms reaper interval + margin).
        tokio::time::sleep(Duration::from_millis(250)).await;

        assert!(mgr.is_sleep_ready().await);
        assert_eq!(mgr.active_count().await, 0);
    }

    #[tokio::test]
    async fn reaper_does_not_expire_fresh_claims() {
        let mgr = SleepInhibitManager::new(Duration::from_millis(50));

        let guard = mgr.acquire("FreshClaim", Duration::from_secs(60)).await;

        // Wait for a few reaper cycles — claim should survive.
        tokio::time::sleep(Duration::from_millis(200)).await;
        assert_eq!(mgr.active_count().await, 1);
        assert!(!mgr.is_sleep_ready().await);

        guard.release();
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;
        assert!(mgr.is_sleep_ready().await);
    }

    #[tokio::test]
    async fn drop_releases_claim() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));

        {
            let _guard = mgr.acquire("ScopedClaim", Duration::from_secs(10)).await;
            assert_eq!(mgr.active_count().await, 1);
        }
        // Guard dropped here.
        tokio::task::yield_now().await;
        tokio::time::sleep(Duration::from_millis(10)).await;

        assert!(mgr.is_sleep_ready().await);
    }

    #[tokio::test]
    async fn active_claims_returns_info() {
        let mgr = SleepInhibitManager::new(Duration::from_secs(60));

        let _g1 = mgr
            .acquire("AutoRelockTimer", Duration::from_secs(60))
            .await;
        let _g2 = mgr
            .acquire("LockFeedbackBlink", Duration::from_secs(5))
            .await;

        let claims = mgr.active_claims().await;
        assert_eq!(claims.len(), 2);

        let names: Vec<&str> = claims.iter().map(|c| c.name.as_str()).collect();
        assert!(names.contains(&"AutoRelockTimer"));
        assert!(names.contains(&"LockFeedbackBlink"));
    }
}
