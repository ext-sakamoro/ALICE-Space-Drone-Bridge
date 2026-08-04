//! 複数の GNSS 測位ソースを重み付き合成する [`FixAggregator`]。
//!
//! 例えば以下のような複数のソースを同時に受け取り、fix 種別由来の
//! 期待精度を weight として合成する:
//!
//! - RTK Fixed (基準局補正) — 最高精度
//! - QZSS L6 Augmented (センチメータ級)
//! - QZSS L1S Augmented (サブメーター級)
//! - SBAS Augmented (メートル級)
//! - Autonomous (単独測位、fallback)
//!
//! 合成は水平位置 (lat/lon → local ENU 平均) と高度に対して行う。
//! 各 fix はローカル frame で ENU に落として重み付き加算、最後に
//! 逆変換で geodetic に戻す。

use alice_drone::vec3::Vec3;

use crate::coord_convert::LocalFrame;
use crate::gnss_fix::{FixKind, GnssFix};

/// 合成に使う 1 サンプル。
#[derive(Debug, Clone, Copy)]
pub struct WeightedFix {
    pub fix: GnssFix,
    /// 明示 weight (省略時は `FixKind::nominal_horizontal_accuracy_m` の逆数)。
    pub weight_override: Option<f64>,
}

impl WeightedFix {
    #[must_use]
    pub fn new(fix: GnssFix) -> Self {
        Self {
            fix,
            weight_override: None,
        }
    }

    #[must_use]
    pub const fn with_weight(fix: GnssFix, weight: f64) -> Self {
        Self {
            fix,
            weight_override: Some(weight),
        }
    }

    #[must_use]
    pub fn weight(&self) -> f64 {
        self.weight_override.unwrap_or_else(|| {
            let acc = self.fix.fix_kind.nominal_horizontal_accuracy_m();
            if acc.is_finite() && acc > 0.0 {
                1.0 / acc
            } else {
                0.0
            }
        })
    }
}

/// 複数の GNSS 測位ソースを合成する。
#[derive(Debug, Clone, Copy)]
pub struct FixAggregator<'a> {
    frame: &'a LocalFrame,
}

impl<'a> FixAggregator<'a> {
    #[must_use]
    pub const fn new(frame: &'a LocalFrame) -> Self {
        Self { frame }
    }

    /// weighted 加算で合成した fix を返す。
    /// - `NoFix` は自動的に無視 (weight ゼロ扱い)
    /// - 有効 fix ゼロなら `None`
    /// - 合成 fix の `fix_kind` は最良品質の fix の種別を採用
    /// - `hdop` / `vdop` / `satellites_in_use` は最良 fix の値を採用
    /// - `timestamp_unix_s` は最新値
    #[must_use]
    pub fn aggregate(&self, samples: &[WeightedFix]) -> Option<GnssFix> {
        let valid: Vec<&WeightedFix> = samples
            .iter()
            .filter(|s| s.fix.is_valid() && s.weight() > 0.0)
            .collect();
        if valid.is_empty() {
            return None;
        }

        let mut sum_east = 0.0;
        let mut sum_north = 0.0;
        let mut sum_up = 0.0;
        let mut sum_w = 0.0;
        let mut best: &WeightedFix = valid[0];
        let mut latest_ts = f64::NEG_INFINITY;

        for s in &valid {
            let enu = self.frame.geodetic_to_enu(s.fix.position);
            let w = s.weight();
            sum_east += enu.x * w;
            sum_north += enu.y * w;
            sum_up += enu.z * w;
            sum_w += w;

            if s.fix.fix_kind.nominal_horizontal_accuracy_m()
                < best.fix.fix_kind.nominal_horizontal_accuracy_m()
            {
                best = s;
            }
            if s.fix.timestamp_unix_s > latest_ts {
                latest_ts = s.fix.timestamp_unix_s;
            }
        }

        let mean_enu = Vec3::new(sum_east / sum_w, sum_north / sum_w, sum_up / sum_w);
        let mean_geodetic = self.frame.enu_to_geodetic(mean_enu);

        Some(GnssFix {
            position: mean_geodetic,
            hdop: best.fix.hdop,
            vdop: best.fix.vdop,
            satellites_in_use: best.fix.satellites_in_use,
            fix_kind: best.fix.fix_kind,
            timestamp_unix_s: latest_ts,
        })
    }
}

/// helper: 単一 fix の nominal weight (`nominal_horizontal_accuracy_m` の逆数)。
#[must_use]
pub fn nominal_weight(kind: FixKind) -> f64 {
    let acc = kind.nominal_horizontal_accuracy_m();
    if acc.is_finite() && acc > 0.0 {
        1.0 / acc
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alice_space::Geodetic;

    fn fix_at(lat_deg: f64, lon_deg: f64, alt_m: f64, kind: FixKind, ts: f64) -> GnssFix {
        GnssFix::builder()
            .position(Geodetic::from_degrees(lat_deg, lon_deg, alt_m))
            .hdop(1.0)
            .vdop(1.5)
            .satellites_in_use(10)
            .fix_kind(kind)
            .timestamp_unix_s(ts)
            .build()
    }

    #[test]
    fn empty_returns_none() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let agg = FixAggregator::new(&frame);
        assert!(agg.aggregate(&[]).is_none());
    }

    #[test]
    fn single_fix_passes_through() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let agg = FixAggregator::new(&frame);
        let fix = fix_at(35.001, 139.001, 30.0, FixKind::Autonomous, 100.0);
        let out = agg.aggregate(&[WeightedFix::new(fix)]).unwrap();
        assert!((out.position.lat_rad - fix.position.lat_rad).abs() < 1e-9);
        assert!((out.position.lon_rad - fix.position.lon_rad).abs() < 1e-9);
        assert!((out.position.alt_m - fix.position.alt_m).abs() < 1e-3);
    }

    #[test]
    fn best_kind_dominates_via_weight() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let agg = FixAggregator::new(&frame);
        // RTK Fixed (accuracy 0.02m, weight 50) と Autonomous (accuracy 5m, weight 0.2)
        let rtk = fix_at(35.001, 139.001, 30.0, FixKind::RtkFixed, 200.0);
        let auto = fix_at(35.002, 139.002, 60.0, FixKind::Autonomous, 100.0);
        let out = agg
            .aggregate(&[WeightedFix::new(rtk), WeightedFix::new(auto)])
            .unwrap();
        // 合成結果は RTK に大きく寄る
        assert!((out.position.lat_rad - rtk.position.lat_rad).abs() < 1e-4);
        assert_eq!(out.fix_kind, FixKind::RtkFixed);
        assert!((out.timestamp_unix_s - 200.0).abs() < 1e-9);
    }

    #[test]
    fn no_fix_samples_are_ignored() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let agg = FixAggregator::new(&frame);
        let good = fix_at(35.001, 139.001, 30.0, FixKind::QzssL1sAugmented, 100.0);
        let bad = fix_at(35.5, 139.5, 300.0, FixKind::NoFix, 100.0);
        let out = agg
            .aggregate(&[WeightedFix::new(good), WeightedFix::new(bad)])
            .unwrap();
        assert!((out.position.lat_rad - good.position.lat_rad).abs() < 1e-9);
    }

    #[test]
    fn explicit_weight_override_wins() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let agg = FixAggregator::new(&frame);
        let rtk = fix_at(35.001, 139.001, 30.0, FixKind::RtkFixed, 200.0);
        let auto = fix_at(35.002, 139.002, 60.0, FixKind::Autonomous, 100.0);
        // Autonomous を明示的に巨大 weight で採用
        let out = agg
            .aggregate(&[
                WeightedFix::new(rtk),
                WeightedFix::with_weight(auto, 10_000.0),
            ])
            .unwrap();
        // 合成結果は Autonomous 側に寄る
        assert!((out.position.lat_rad - auto.position.lat_rad).abs() < 1e-4);
        // ただし fix_kind は最良品質を採用
        assert_eq!(out.fix_kind, FixKind::RtkFixed);
    }
}
