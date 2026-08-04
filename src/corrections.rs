//! Klobuchar 電離層 / Saastamoinen 対流圏補正を fix 位置に適用する helper。
//!
//! `alice_space::ionosphere` / `alice_space::troposphere` の生モデルは
//! 疑似距離 (擬似距離、meter) の遅延を返す。ここでは擬似距離補正を
//! **概算の垂直方向 (up)** の位置補正として fix に反映する簡易パスを
//! 提供する。精密測位でなく、SBAS / QZSS 補強を使わない autonomous
//! 測位を粗く補正するユースケース向け。
//!
//! 補正は fix の geodetic 高度 (alt_m) に対して適用する。
//! - `IonosphereContext`: Klobuchar 係数と衛星仰角 / 方位角
//! - `TroposphereContext`: 気象 (MetState) + 衛星仰角
//!
//! 精度が必要な用途では `alice_space::sbas` / `qzss_l1s` / `qzss_l6` /
//! `rtcm3` の差分補正を経由することを推奨する。

use alice_space::ionosphere::{klobuchar_delay_m, KlobucharCoefficients};
use alice_space::troposphere::{saastamoinen_delay_m, MetState};

use crate::gnss_fix::GnssFix;

/// 電離層補正の入力コンテキスト。
#[derive(Debug, Clone, Copy)]
pub struct IonosphereContext {
    /// 4+4 Klobuchar 係数 (航法メッセージ由来)。
    pub coeffs: KlobucharCoefficients,
    /// 衛星方位角 (radian、北から時計回り)。
    pub azimuth_rad: f64,
    /// 衛星仰角 (radian)。
    pub elevation_rad: f64,
    /// 測位時刻の GPS 週内秒。
    pub gps_seconds_of_week: f64,
}

/// 対流圏補正の入力コンテキスト。
#[derive(Debug, Clone, Copy)]
pub struct TroposphereContext {
    /// 気象状態。
    pub met: MetState,
    /// 衛星仰角 (radian)。
    pub elevation_rad: f64,
}

impl TroposphereContext {
    /// 気象観測がない場合の default: ISO 2533 標準大気を fix 高度で構築。
    #[must_use]
    pub fn standard_at_fix(fix: &GnssFix, elevation_rad: f64) -> Self {
        Self {
            met: MetState::standard_at_altitude(fix.position.alt_m),
            elevation_rad,
        }
    }
}

/// Klobuchar モデルによる推定遅延 (m) を fix の高度に減算補正した新しい fix を返す。
///
/// 上向き成分に単純減算する近似で、正確な垂直分解には受信機観測に
/// マッピング関数を掛ける必要があるが、ここでは 1st-order 補正として扱う。
#[must_use]
pub fn correct_ionosphere(fix: GnssFix, ctx: &IonosphereContext) -> GnssFix {
    let delay_m = klobuchar_delay_m(
        ctx.coeffs,
        fix.position.lat_rad,
        fix.position.lon_rad,
        ctx.azimuth_rad,
        ctx.elevation_rad,
        ctx.gps_seconds_of_week,
    );
    let mut out = fix;
    out.position.alt_m -= delay_m;
    out
}

/// Saastamoinen モデルによる対流圏遅延を fix の高度に減算補正した新しい fix を返す。
#[must_use]
pub fn correct_troposphere(fix: GnssFix, ctx: &TroposphereContext) -> GnssFix {
    let delay_m = saastamoinen_delay_m(ctx.met, fix.position, ctx.elevation_rad);
    let mut out = fix;
    out.position.alt_m -= delay_m;
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss_fix::{FixKind, GnssFix};
    use alice_space::Geodetic;

    fn base_fix() -> GnssFix {
        GnssFix::builder()
            .position(Geodetic::from_degrees(35.0, 139.0, 100.0))
            .fix_kind(FixKind::Autonomous)
            .satellites_in_use(6)
            .hdop(1.5)
            .vdop(2.0)
            .timestamp_unix_s(1_735_000_000.0)
            .build()
    }

    #[test]
    fn ionosphere_correction_does_not_alter_kind() {
        let fix = base_fix();
        let ctx = IonosphereContext {
            coeffs: KlobucharCoefficients::representative_2020(),
            azimuth_rad: 90.0_f64.to_radians(),
            elevation_rad: 45.0_f64.to_radians(),
            gps_seconds_of_week: 100_000.0,
        };
        let corrected = correct_ionosphere(fix, &ctx);
        assert_eq!(corrected.fix_kind, fix.fix_kind);
        assert_eq!(corrected.timestamp_unix_s, fix.timestamp_unix_s);
    }

    #[test]
    fn ionosphere_correction_reduces_altitude() {
        let fix = base_fix();
        let ctx = IonosphereContext {
            coeffs: KlobucharCoefficients::representative_2020(),
            azimuth_rad: 90.0_f64.to_radians(),
            elevation_rad: 45.0_f64.to_radians(),
            gps_seconds_of_week: 100_000.0,
        };
        let corrected = correct_ionosphere(fix, &ctx);
        // Klobuchar は非負遅延を返すため補正後の高度は元より下がる (もしくは同値)
        assert!(corrected.position.alt_m <= fix.position.alt_m);
    }

    #[test]
    fn troposphere_correction_reduces_altitude() {
        let fix = base_fix();
        let ctx = TroposphereContext::standard_at_fix(&fix, 45.0_f64.to_radians());
        let corrected = correct_troposphere(fix, &ctx);
        assert!(corrected.position.alt_m < fix.position.alt_m);
    }

    #[test]
    fn low_elevation_yields_larger_correction() {
        let fix = base_fix();
        let high = TroposphereContext::standard_at_fix(&fix, 80.0_f64.to_radians());
        let low = TroposphereContext::standard_at_fix(&fix, 10.0_f64.to_radians());
        let corrected_high = correct_troposphere(fix, &high);
        let corrected_low = correct_troposphere(fix, &low);
        // 低仰角ほど遅延大 → 補正後の高度は低くなる
        assert!(corrected_low.position.alt_m < corrected_high.position.alt_m);
    }
}
