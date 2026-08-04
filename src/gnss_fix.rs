//! GNSS 測位 fix 型 ([`GnssFix`] / [`FixKind`]) と builder。
//!
//! `alice_space::Geodetic` を WGS-84 位置とし、DOP / 衛星数 / fix 種別 /
//! Unix timestamp を束ねる。drone 制御 / mapping / IoT tracker のいずれで
//! も統一表現として使える汎用構造。

use alice_space::Geodetic;

/// 測位 fix の種別。補強信号の適用状況を区別する。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FixKind {
    /// 補強なしの単独測位。
    Autonomous,
    /// SBAS 補強 (静止衛星から放送される差分補正)。
    SbasAugmented,
    /// QZSS L1S サブメーター級補強 (SLAS 相当)。
    QzssL1sAugmented,
    /// QZSS L6 cm 級補強 (CLAS 系センチメータ補強)。
    QzssL6Augmented,
    /// RTK 基準局補正、整数アンビギュイティ確定 (fixed)。
    RtkFixed,
    /// RTK 基準局補正、整数アンビギュイティ未確定 (float)。
    RtkFloat,
    /// 補外・慣性合成による推測航法。
    DeadReckoning,
    /// 測位不能。
    NoFix,
}

impl FixKind {
    /// この fix 種別が期待される概略の水平精度 (メートル、経験値)。
    ///
    /// 具体的な数値は運用条件で大きく変わるため、gate や重み付けの初期値
    /// として使う目安。
    #[must_use]
    pub const fn nominal_horizontal_accuracy_m(self) -> f64 {
        match self {
            Self::Autonomous => 5.0,
            Self::SbasAugmented => 1.5,
            Self::QzssL1sAugmented => 1.0,
            Self::QzssL6Augmented => 0.1,
            Self::RtkFixed => 0.02,
            Self::RtkFloat => 0.3,
            Self::DeadReckoning => 20.0,
            Self::NoFix => f64::INFINITY,
        }
    }

    /// 有効な測位解を返しているか。
    #[must_use]
    pub const fn is_valid(self) -> bool {
        !matches!(self, Self::NoFix)
    }
}

/// 単一の GNSS 測位 fix。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GnssFix {
    /// WGS-84 geodetic 位置。
    pub position: Geodetic,
    /// Horizontal Dilution of Precision。
    pub hdop: f64,
    /// Vertical Dilution of Precision。
    pub vdop: f64,
    /// 測位に使用した衛星数。
    pub satellites_in_use: u8,
    /// fix 種別。
    pub fix_kind: FixKind,
    /// Unix timestamp (秒、小数以下も許容)。
    pub timestamp_unix_s: f64,
}

impl GnssFix {
    /// builder を開始。
    #[must_use]
    pub fn builder() -> GnssFixBuilder {
        GnssFixBuilder::default()
    }

    /// PDOP (= sqrt(HDOP² + VDOP²)) の推定。
    #[must_use]
    pub fn pdop(&self) -> f64 {
        (self.hdop * self.hdop + self.vdop * self.vdop).sqrt()
    }

    /// 有効な測位解か。
    #[must_use]
    pub const fn is_valid(&self) -> bool {
        self.fix_kind.is_valid()
    }

    /// この fix が発行されてからの経過秒。負値 (未来 timestamp) は 0 に丸める。
    #[must_use]
    pub fn age_seconds(&self, now_unix_s: f64) -> f64 {
        (now_unix_s - self.timestamp_unix_s).max(0.0)
    }
}

/// [`GnssFix`] の builder。
#[derive(Debug, Clone)]
pub struct GnssFixBuilder {
    position: Geodetic,
    hdop: f64,
    vdop: f64,
    satellites_in_use: u8,
    fix_kind: FixKind,
    timestamp_unix_s: f64,
}

impl Default for GnssFixBuilder {
    fn default() -> Self {
        Self {
            position: Geodetic {
                lat_rad: 0.0,
                lon_rad: 0.0,
                alt_m: 0.0,
            },
            hdop: 99.0,
            vdop: 99.0,
            satellites_in_use: 0,
            fix_kind: FixKind::NoFix,
            timestamp_unix_s: 0.0,
        }
    }
}

impl GnssFixBuilder {
    #[must_use]
    pub fn position(mut self, p: Geodetic) -> Self {
        self.position = p;
        self
    }

    #[must_use]
    pub fn hdop(mut self, hdop: f64) -> Self {
        self.hdop = hdop.max(0.0);
        self
    }

    #[must_use]
    pub fn vdop(mut self, vdop: f64) -> Self {
        self.vdop = vdop.max(0.0);
        self
    }

    #[must_use]
    pub fn satellites_in_use(mut self, n: u8) -> Self {
        self.satellites_in_use = n;
        self
    }

    #[must_use]
    pub fn fix_kind(mut self, k: FixKind) -> Self {
        self.fix_kind = k;
        self
    }

    #[must_use]
    pub fn timestamp_unix_s(mut self, t: f64) -> Self {
        self.timestamp_unix_s = t;
        self
    }

    #[must_use]
    pub fn build(self) -> GnssFix {
        GnssFix {
            position: self.position,
            hdop: self.hdop,
            vdop: self.vdop,
            satellites_in_use: self.satellites_in_use,
            fix_kind: self.fix_kind,
            timestamp_unix_s: self.timestamp_unix_s,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nominal_accuracy_ranks_correctly() {
        assert!(
            FixKind::RtkFixed.nominal_horizontal_accuracy_m()
                < FixKind::QzssL6Augmented.nominal_horizontal_accuracy_m()
        );
        assert!(
            FixKind::QzssL6Augmented.nominal_horizontal_accuracy_m()
                < FixKind::QzssL1sAugmented.nominal_horizontal_accuracy_m()
        );
        assert!(
            FixKind::QzssL1sAugmented.nominal_horizontal_accuracy_m()
                < FixKind::SbasAugmented.nominal_horizontal_accuracy_m()
        );
        assert!(
            FixKind::SbasAugmented.nominal_horizontal_accuracy_m()
                < FixKind::Autonomous.nominal_horizontal_accuracy_m()
        );
        assert!(FixKind::NoFix.nominal_horizontal_accuracy_m().is_infinite());
    }

    #[test]
    fn no_fix_is_invalid() {
        assert!(!FixKind::NoFix.is_valid());
        assert!(FixKind::RtkFixed.is_valid());
    }

    #[test]
    fn builder_defaults_are_no_fix() {
        let fix = GnssFix::builder().build();
        assert_eq!(fix.fix_kind, FixKind::NoFix);
        assert!(!fix.is_valid());
    }

    #[test]
    fn pdop_matches_pythagoras() {
        let fix = GnssFix::builder().hdop(3.0).vdop(4.0).build();
        assert!((fix.pdop() - 5.0).abs() < 1e-9);
    }

    #[test]
    fn age_is_clamped_to_non_negative() {
        let fix = GnssFix::builder().timestamp_unix_s(100.0).build();
        assert_eq!(fix.age_seconds(50.0), 0.0);
        assert_eq!(fix.age_seconds(150.0), 50.0);
    }
}
