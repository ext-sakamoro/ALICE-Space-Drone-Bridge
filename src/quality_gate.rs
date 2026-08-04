//! DOP / 衛星数 / fix 種別を根拠にした飛行 / 動作可否 gate。
//!
//! [`QualityPolicy`] で threshold を設定し、[`quality_gate`] が
//! [`Gate::Allow`] / [`Gate::Warn`] / [`Gate::Deny`] を返す。
//!
//! - Deny: 飛行禁止 (fix 未取得 / HDOP 過大 / 衛星不足)
//! - Warn: 通常より劣化、hover-only や RTL 起動を推奨
//! - Allow: 通常運航可

use crate::gnss_fix::{FixKind, GnssFix};

/// gate 判定結果。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Gate {
    /// 通常運航可。
    Allow,
    /// 劣化状態、注意して運航。
    Warn(&'static str),
    /// 飛行禁止。
    Deny(&'static str),
}

/// gate policy (閾値の束)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QualityPolicy {
    /// Allow の HDOP 上限。
    pub hdop_allow_max: f64,
    /// Warn の HDOP 上限。これを越えたら Deny。
    pub hdop_warn_max: f64,
    /// Allow の VDOP 上限。
    pub vdop_allow_max: f64,
    /// Warn の VDOP 上限。これを越えたら Deny。
    pub vdop_warn_max: f64,
    /// Allow に必要な最小衛星数。
    pub min_satellites_allow: u8,
    /// Warn に許容する最小衛星数。これ未満は Deny。
    pub min_satellites_warn: u8,
    /// Deny 扱いにする fix 種別 (例: `DeadReckoning` のみでの飛行を禁止する場合)。
    pub deny_dead_reckoning: bool,
}

impl Default for QualityPolicy {
    /// 汎用ドローン運用向けデフォルト。
    fn default() -> Self {
        Self {
            hdop_allow_max: 2.0,
            hdop_warn_max: 5.0,
            vdop_allow_max: 3.0,
            vdop_warn_max: 7.0,
            min_satellites_allow: 8,
            min_satellites_warn: 5,
            deny_dead_reckoning: true,
        }
    }
}

impl QualityPolicy {
    /// より厳格な RTK / 精密飛行向け policy。
    #[must_use]
    pub const fn strict_rtk() -> Self {
        Self {
            hdop_allow_max: 1.2,
            hdop_warn_max: 2.5,
            vdop_allow_max: 1.8,
            vdop_warn_max: 3.5,
            min_satellites_allow: 10,
            min_satellites_warn: 7,
            deny_dead_reckoning: true,
        }
    }

    /// 屋外ホビー用途向けの緩い policy。
    #[must_use]
    pub const fn relaxed() -> Self {
        Self {
            hdop_allow_max: 3.0,
            hdop_warn_max: 8.0,
            vdop_allow_max: 4.5,
            vdop_warn_max: 10.0,
            min_satellites_allow: 6,
            min_satellites_warn: 4,
            deny_dead_reckoning: false,
        }
    }
}

/// GnssFix と policy から gate 判定を行う。
#[must_use]
pub fn quality_gate(fix: &GnssFix, policy: &QualityPolicy) -> Gate {
    if !fix.is_valid() {
        return Gate::Deny("fix invalid (NoFix)");
    }
    if policy.deny_dead_reckoning && fix.fix_kind == FixKind::DeadReckoning {
        return Gate::Deny("dead reckoning only, denied by policy");
    }
    if fix.satellites_in_use < policy.min_satellites_warn {
        return Gate::Deny("satellites below warn threshold");
    }
    if fix.hdop > policy.hdop_warn_max {
        return Gate::Deny("HDOP exceeds warn threshold");
    }
    if fix.vdop > policy.vdop_warn_max {
        return Gate::Deny("VDOP exceeds warn threshold");
    }

    if fix.satellites_in_use < policy.min_satellites_allow {
        return Gate::Warn("satellites in warn range");
    }
    if fix.hdop > policy.hdop_allow_max {
        return Gate::Warn("HDOP in warn range");
    }
    if fix.vdop > policy.vdop_allow_max {
        return Gate::Warn("VDOP in warn range");
    }

    Gate::Allow
}

#[cfg(test)]
mod tests {
    use super::*;
    use alice_space::Geodetic;

    fn base_fix() -> GnssFix {
        GnssFix::builder()
            .position(Geodetic::from_degrees(35.0, 139.0, 0.0))
            .hdop(1.0)
            .vdop(1.5)
            .satellites_in_use(12)
            .fix_kind(FixKind::QzssL1sAugmented)
            .timestamp_unix_s(1_735_000_000.0)
            .build()
    }

    #[test]
    fn good_fix_is_allowed() {
        assert_eq!(quality_gate(&base_fix(), &QualityPolicy::default()), Gate::Allow);
    }

    #[test]
    fn no_fix_is_denied() {
        let fix = GnssFix::builder().build();
        matches!(
            quality_gate(&fix, &QualityPolicy::default()),
            Gate::Deny(_)
        );
    }

    #[test]
    fn few_satellites_produce_warn_then_deny() {
        let mut fix = base_fix();
        fix.satellites_in_use = 6;
        assert!(matches!(
            quality_gate(&fix, &QualityPolicy::default()),
            Gate::Warn(_)
        ));
        fix.satellites_in_use = 3;
        assert!(matches!(
            quality_gate(&fix, &QualityPolicy::default()),
            Gate::Deny(_)
        ));
    }

    #[test]
    fn high_hdop_is_denied() {
        let mut fix = base_fix();
        fix.hdop = 10.0;
        assert!(matches!(
            quality_gate(&fix, &QualityPolicy::default()),
            Gate::Deny(_)
        ));
    }

    #[test]
    fn strict_rtk_requires_better_dop() {
        let mut fix = base_fix();
        fix.hdop = 1.5;
        // default では allow 内、strict では warn (1.2 超え、2.5 以下)
        assert_eq!(quality_gate(&fix, &QualityPolicy::default()), Gate::Allow);
        assert!(matches!(
            quality_gate(&fix, &QualityPolicy::strict_rtk()),
            Gate::Warn(_)
        ));
    }

    #[test]
    fn relaxed_policy_permits_dead_reckoning() {
        let mut fix = base_fix();
        fix.fix_kind = FixKind::DeadReckoning;
        assert!(matches!(
            quality_gate(&fix, &QualityPolicy::default()),
            Gate::Deny(_)
        ));
        // relaxed でも警告は付くが Allow または Warn 範囲
        assert!(!matches!(
            quality_gate(&fix, &QualityPolicy::relaxed()),
            Gate::Deny(_)
        ));
    }
}
