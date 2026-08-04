//! [`IntegrityMonitor`] — DOP / 衛星数 / fix age / cycle-slip flag を統合し
//! 単一の [`IntegrityStatus`] を返す完全性監視。
//!
//! `quality_gate` が「今 fix を採用してよいか」を判定するのに対し、
//! `IntegrityMonitor` は「時系列的に fix ソースは信頼できるか」を評価する。
//! drone / mapping / IoT tracker のいずれの用途でも導入可能。

use crate::gnss_fix::GnssFix;

/// 完全性状態。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegrityStatus {
    /// 信頼できる。
    Nominal,
    /// 劣化しているが動作可能。
    Degraded,
    /// 信頼不能。運航停止を推奨。
    Alert,
}

/// 完全性レポート。
#[derive(Debug, Clone, PartialEq)]
pub struct IntegrityReport {
    pub status: IntegrityStatus,
    /// 詳細理由 (複数列挙可)。
    pub reasons: Vec<&'static str>,
    /// 直近 fix の age (seconds)。
    pub last_fix_age_s: f64,
    /// 直近 fix の PDOP (推定)。
    pub last_pdop: f64,
}

/// 完全性監視の閾値。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct IntegrityMonitor {
    /// 直近 fix からの許容 age (秒)。超えたら Degraded、2 倍で Alert。
    pub max_fix_age_s: f64,
    /// 許容 PDOP。超えたら Degraded、2 倍で Alert。
    pub max_pdop: f64,
    /// 許容 cycle-slip 数 (直近 window 内)。超えたら Degraded、3 倍で Alert。
    pub max_cycle_slips: u32,
    /// Alert に落とす最小衛星数。
    pub min_satellites: u8,
}

impl Default for IntegrityMonitor {
    fn default() -> Self {
        Self {
            max_fix_age_s: 2.0,
            max_pdop: 4.0,
            max_cycle_slips: 3,
            min_satellites: 5,
        }
    }
}

impl IntegrityMonitor {
    /// fix + 現在時刻 + cycle-slip 数から完全性を評価。
    #[must_use]
    pub fn evaluate(&self, fix: &GnssFix, now_unix_s: f64, cycle_slips: u32) -> IntegrityReport {
        let mut reasons: Vec<&'static str> = Vec::new();
        let mut status = IntegrityStatus::Nominal;
        let age = fix.age_seconds(now_unix_s);
        let pdop = fix.pdop();

        if !fix.is_valid() {
            reasons.push("fix invalid");
            status = IntegrityStatus::Alert;
        }
        if fix.satellites_in_use < self.min_satellites {
            reasons.push("satellites below floor");
            status = IntegrityStatus::Alert;
        }
        if age > 2.0 * self.max_fix_age_s {
            reasons.push("fix age critical");
            status = IntegrityStatus::Alert;
        } else if age > self.max_fix_age_s {
            reasons.push("fix age degraded");
            status = merge_worst(status, IntegrityStatus::Degraded);
        }
        if pdop > 2.0 * self.max_pdop {
            reasons.push("PDOP critical");
            status = IntegrityStatus::Alert;
        } else if pdop > self.max_pdop {
            reasons.push("PDOP degraded");
            status = merge_worst(status, IntegrityStatus::Degraded);
        }
        if cycle_slips >= 3 * self.max_cycle_slips {
            reasons.push("cycle-slip critical");
            status = IntegrityStatus::Alert;
        } else if cycle_slips > self.max_cycle_slips {
            reasons.push("cycle-slip degraded");
            status = merge_worst(status, IntegrityStatus::Degraded);
        }

        IntegrityReport {
            status,
            reasons,
            last_fix_age_s: age,
            last_pdop: pdop,
        }
    }
}

const fn merge_worst(a: IntegrityStatus, b: IntegrityStatus) -> IntegrityStatus {
    match (a, b) {
        (IntegrityStatus::Alert, _) | (_, IntegrityStatus::Alert) => IntegrityStatus::Alert,
        (IntegrityStatus::Degraded, _) | (_, IntegrityStatus::Degraded) => IntegrityStatus::Degraded,
        _ => IntegrityStatus::Nominal,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss_fix::FixKind;
    use alice_space::Geodetic;

    fn healthy_fix() -> GnssFix {
        GnssFix::builder()
            .position(Geodetic::from_degrees(35.0, 139.0, 0.0))
            .hdop(1.0)
            .vdop(1.5)
            .satellites_in_use(12)
            .fix_kind(FixKind::QzssL6Augmented)
            .timestamp_unix_s(1000.0)
            .build()
    }

    #[test]
    fn healthy_fix_is_nominal() {
        let mon = IntegrityMonitor::default();
        let report = mon.evaluate(&healthy_fix(), 1000.5, 0);
        assert_eq!(report.status, IntegrityStatus::Nominal);
        assert!(report.reasons.is_empty());
    }

    #[test]
    fn stale_fix_is_alert() {
        let mon = IntegrityMonitor::default();
        // max_fix_age_s=2.0 → 5.0 秒経過は 2.5 倍で critical
        let report = mon.evaluate(&healthy_fix(), 1005.0, 0);
        assert_eq!(report.status, IntegrityStatus::Alert);
    }

    #[test]
    fn moderate_delay_is_degraded() {
        let mon = IntegrityMonitor::default();
        let report = mon.evaluate(&healthy_fix(), 1003.0, 0);
        assert_eq!(report.status, IntegrityStatus::Degraded);
    }

    #[test]
    fn cycle_slip_escalates() {
        let mon = IntegrityMonitor::default();
        let degraded = mon.evaluate(&healthy_fix(), 1000.5, 5);
        assert_eq!(degraded.status, IntegrityStatus::Degraded);
        let alerted = mon.evaluate(&healthy_fix(), 1000.5, 20);
        assert_eq!(alerted.status, IntegrityStatus::Alert);
    }

    #[test]
    fn invalid_fix_is_alert() {
        let mon = IntegrityMonitor::default();
        let fix = GnssFix::builder().build();
        let report = mon.evaluate(&fix, 0.0, 0);
        assert_eq!(report.status, IntegrityStatus::Alert);
    }
}
