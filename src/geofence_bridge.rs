//! 地理座標領域 → `alice_drone::Geofence` の map。
//!
//! - `circular_geofence_at`: 中心 (lat/lon) + 半径 → 円柱ジオフェンス
//! - `bounding_box_geofence`: 緯度経度 min/max + 高度域 → 矩形ジオフェンス

use alice_drone::geofence::Geofence;
use alice_drone::vec3::Vec3;
use alice_space::Geodetic;

use crate::coord_convert::LocalFrame;

/// 地理座標中心の円柱ジオフェンスを生成 (alice_drone Y-up frame 上に配置)。
///
/// `warning_margin_m` は境界からの警告余裕 (m)、`alice_drone::Geofence` の
/// `Warning` 状態を返す幅を制御する。
#[must_use]
pub fn circular_geofence_at(
    frame: &LocalFrame,
    center_geodetic: Geodetic,
    radius_m: f64,
    min_altitude_m: f64,
    max_altitude_m: f64,
    warning_margin_m: f64,
) -> Geofence {
    let local_center = frame.geodetic_to_drone(center_geodetic);
    Geofence::cylinder(
        local_center,
        radius_m,
        min_altitude_m,
        max_altitude_m,
        warning_margin_m,
    )
}

/// 緯度経度の矩形 (min / max) + 高度域 → 矩形ジオフェンス (alice_drone Y-up frame)。
///
/// 緯度・経度は degree、alt / margin はメートル。
/// 返す Box は drone frame で `x`=East, `y`=Up (altitude), `z`=North。
#[must_use]
pub fn bounding_box_geofence(
    frame: &LocalFrame,
    sw_geodetic: Geodetic,
    ne_geodetic: Geodetic,
    min_altitude_m: f64,
    max_altitude_m: f64,
    warning_margin_m: f64,
) -> Geofence {
    let sw = frame.geodetic_to_drone(sw_geodetic);
    let ne = frame.geodetic_to_drone(ne_geodetic);

    let min = Vec3::new(sw.x.min(ne.x), min_altitude_m, sw.z.min(ne.z));
    let max = Vec3::new(sw.x.max(ne.x), max_altitude_m, sw.z.max(ne.z));

    Geofence::bounding_box(min, max, warning_margin_m)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alice_drone::geofence::GeofenceStatus;

    #[test]
    fn point_inside_circular_geofence_is_inside() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let center = Geodetic::from_degrees(35.001, 139.001, 0.0);
        let fence = circular_geofence_at(&frame, center, 200.0, 0.0, 100.0, 10.0);

        let local = frame.geodetic_to_drone(center);
        // drone frame: x=East, y=Up(altitude), z=North
        // 中心の 10m 東で高度 50m は Inside
        let sample = Vec3::new(local.x + 10.0, 50.0, local.z);
        assert_eq!(fence.check(sample), GeofenceStatus::Inside);
    }

    #[test]
    fn point_outside_circular_geofence_is_outside() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let center = Geodetic::from_degrees(35.001, 139.001, 0.0);
        let fence = circular_geofence_at(&frame, center, 50.0, 0.0, 100.0, 5.0);

        let local = frame.geodetic_to_drone(center);
        // 中心から 500m 東は Outside
        let sample = Vec3::new(local.x + 500.0, 50.0, local.z);
        assert_eq!(fence.check(sample), GeofenceStatus::Outside);
    }

    #[test]
    fn bounding_box_geofence_normalises_min_max() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        // 敢えて SW/NE を逆に渡しても min/max が入れ替わって正しく機能する
        let sw = Geodetic::from_degrees(35.002, 139.002, 0.0);
        let ne = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let fence = bounding_box_geofence(&frame, sw, ne, 0.0, 100.0, 5.0);

        let inside_geo = Geodetic::from_degrees(35.001, 139.001, 0.0);
        let inside = frame.geodetic_to_drone(inside_geo);
        let sample = Vec3::new(inside.x, 50.0, inside.z);
        assert_eq!(fence.check(sample), GeofenceStatus::Inside);
    }
}
