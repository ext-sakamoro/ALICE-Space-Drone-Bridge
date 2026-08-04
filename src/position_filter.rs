//! GNSS 位置解のジッタを平滑化するフィルタ。
//!
//! - [`EwmaFilter`]: 指数加重移動平均。応答速度 (`alpha`) を調整可
//! - trait [`PositionFilter`]: 任意のフィルタ実装のための共通 interface
//!
//! ENU / NED どちらの直交座標にも適用可 (`alice_drone::vec3::Vec3`)。

use alice_drone::vec3::Vec3;

/// 位置フィルタの trait。
pub trait PositionFilter {
    /// 新規サンプルを取り込み、平滑化された位置を返す。
    fn update(&mut self, sample: Vec3) -> Vec3;
    /// 直近平滑値。初期化前は `None`。
    fn value(&self) -> Option<Vec3>;
    /// 内部状態を初期化。
    fn reset(&mut self);
}

/// 指数加重移動平均フィルタ (EWMA)。
///
/// `alpha` は 0.0-1.0。`alpha=0` は不変 (完全平滑)、`alpha=1` は生値素通し。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EwmaFilter {
    alpha: f64,
    state: Option<Vec3>,
}

impl EwmaFilter {
    /// alpha を指定して構築。0.0 未満 / 1.0 超は clamp。
    #[must_use]
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            state: None,
        }
    }
}

impl PositionFilter for EwmaFilter {
    fn update(&mut self, sample: Vec3) -> Vec3 {
        let next = match self.state {
            None => sample,
            Some(prev) => Vec3::new(
                prev.x + self.alpha * (sample.x - prev.x),
                prev.y + self.alpha * (sample.y - prev.y),
                prev.z + self.alpha * (sample.z - prev.z),
            ),
        };
        self.state = Some(next);
        next
    }

    fn value(&self) -> Option<Vec3> {
        self.state
    }

    fn reset(&mut self) {
        self.state = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn first_sample_becomes_state() {
        let mut f = EwmaFilter::new(0.5);
        let out = f.update(Vec3::new(1.0, 2.0, 3.0));
        assert!(approx(out.x, 1.0, 1e-9));
        assert_eq!(f.value(), Some(out));
    }

    #[test]
    fn alpha_zero_holds_first_sample() {
        let mut f = EwmaFilter::new(0.0);
        f.update(Vec3::new(1.0, 0.0, 0.0));
        let out = f.update(Vec3::new(100.0, 0.0, 0.0));
        assert!(approx(out.x, 1.0, 1e-9));
    }

    #[test]
    fn alpha_one_passes_sample_through() {
        let mut f = EwmaFilter::new(1.0);
        f.update(Vec3::new(1.0, 0.0, 0.0));
        let out = f.update(Vec3::new(100.0, 0.0, 0.0));
        assert!(approx(out.x, 100.0, 1e-9));
    }

    #[test]
    fn convergence_toward_constant_input() {
        let mut f = EwmaFilter::new(0.3);
        for _ in 0..100 {
            f.update(Vec3::new(10.0, 20.0, 30.0));
        }
        let v = f.value().unwrap();
        assert!(approx(v.x, 10.0, 1e-3));
        assert!(approx(v.y, 20.0, 1e-3));
        assert!(approx(v.z, 30.0, 1e-3));
    }

    #[test]
    fn reset_clears_state() {
        let mut f = EwmaFilter::new(0.5);
        f.update(Vec3::new(1.0, 0.0, 0.0));
        f.reset();
        assert_eq!(f.value(), None);
    }

    #[test]
    fn alpha_is_clamped() {
        let mut f = EwmaFilter::new(-1.0);
        f.update(Vec3::new(5.0, 0.0, 0.0));
        let out = f.update(Vec3::new(100.0, 0.0, 0.0));
        // alpha=0 相当なので第一 sample を保持
        assert!(approx(out.x, 5.0, 1e-9));
    }
}
