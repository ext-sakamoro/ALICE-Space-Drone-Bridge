//! 地理座標 ↔ ローカル ENU / NED / drone Y-up frame の変換。
//!
//! drone は離着陸地点 (home) からの相対座標で waypoint / geofence /
//! 制御を扱うため、WGS-84 geodetic 位置を home 原点の局所直交 frame に
//! 落とす必要がある。ここでは [`LocalFrame`] が home を保持し、以下の
//! 3 系統を提供する。
//!
//! - **純粋 ENU** (East / North / Up): `Vec3::x` = East, `y` = North, `z` = Up
//! - **純粋 NED** (North / East / Down): `Vec3::x` = North, `y` = East, `z` = Down
//! - **alice_drone Y-up drone frame**: `Vec3::x` = East, `y` = **Up (altitude)**, `z` = North
//!   `alice_drone::geofence::Geofence` は `pos.y` を altitude として扱う
//!   Y-up 系を採用しているため、`Waypoint` / `Geofence` に注入する Vec3
//!   はこの drone frame に合わせる必要がある。

use alice_drone::vec3::Vec3;
use alice_space::geodetic::{ecef_to_geodetic, geodetic_to_ecef, Ecef, Geodetic};

/// 原点固定のローカル直交 frame。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalFrame {
    origin_geodetic: Geodetic,
    origin_ecef: Ecef,
    sin_lat: f64,
    cos_lat: f64,
    sin_lon: f64,
    cos_lon: f64,
}

impl LocalFrame {
    /// home 地点を原点とする ENU frame を生成。
    #[must_use]
    pub fn enu_at(origin: Geodetic) -> Self {
        let origin_ecef = geodetic_to_ecef(origin);
        Self {
            origin_geodetic: origin,
            origin_ecef,
            sin_lat: origin.lat_rad.sin(),
            cos_lat: origin.lat_rad.cos(),
            sin_lon: origin.lon_rad.sin(),
            cos_lon: origin.lon_rad.cos(),
        }
    }

    /// 原点 (home) の geodetic 座標。
    #[must_use]
    pub const fn origin(&self) -> Geodetic {
        self.origin_geodetic
    }

    /// geodetic → ENU (East, North, Up)、単位はメートル。
    #[must_use]
    pub fn geodetic_to_enu(&self, target: Geodetic) -> Vec3 {
        let target_ecef = geodetic_to_ecef(target);
        let dx = target_ecef.x_m - self.origin_ecef.x_m;
        let dy = target_ecef.y_m - self.origin_ecef.y_m;
        let dz = target_ecef.z_m - self.origin_ecef.z_m;

        let east = -self.sin_lon * dx + self.cos_lon * dy;
        let north = -self.sin_lat * self.cos_lon * dx - self.sin_lat * self.sin_lon * dy
            + self.cos_lat * dz;
        let up = self.cos_lat * self.cos_lon * dx
            + self.cos_lat * self.sin_lon * dy
            + self.sin_lat * dz;

        Vec3::new(east, north, up)
    }

    /// ENU (East, North, Up) → geodetic。`geodetic_to_enu` の逆変換。
    #[must_use]
    pub fn enu_to_geodetic(&self, enu: Vec3) -> Geodetic {
        let east = enu.x;
        let north = enu.y;
        let up = enu.z;

        // 逆回転行列 (ENU→ECEF diff)
        let dx = -self.sin_lon * east - self.sin_lat * self.cos_lon * north
            + self.cos_lat * self.cos_lon * up;
        let dy = self.cos_lon * east - self.sin_lat * self.sin_lon * north
            + self.cos_lat * self.sin_lon * up;
        let dz = self.cos_lat * north + self.sin_lat * up;

        let ecef = Ecef {
            x_m: self.origin_ecef.x_m + dx,
            y_m: self.origin_ecef.y_m + dy,
            z_m: self.origin_ecef.z_m + dz,
        };
        ecef_to_geodetic(ecef)
    }

    /// geodetic → NED (North, East, Down)、単位はメートル。
    #[must_use]
    pub fn geodetic_to_ned(&self, target: Geodetic) -> Vec3 {
        let enu = self.geodetic_to_enu(target);
        // NED: x=North, y=East, z=Down
        Vec3::new(enu.y, enu.x, -enu.z)
    }

    /// NED (North, East, Down) → geodetic。
    #[must_use]
    pub fn ned_to_geodetic(&self, ned: Vec3) -> Geodetic {
        // NED→ENU
        let enu = Vec3::new(ned.y, ned.x, -ned.z);
        self.enu_to_geodetic(enu)
    }

    /// home 地点との水平距離 (メートル)。
    #[must_use]
    pub fn horizontal_distance_m(&self, target: Geodetic) -> f64 {
        let enu = self.geodetic_to_enu(target);
        (enu.x * enu.x + enu.y * enu.y).sqrt()
    }

    /// home 地点からの相対高度 (メートル、上向き正)。
    #[must_use]
    pub fn relative_altitude_m(&self, target: Geodetic) -> f64 {
        target.alt_m - self.origin_geodetic.alt_m
    }

    /// geodetic → alice_drone Y-up drone frame (x=East, y=**Up (altitude)**, z=North)。
    ///
    /// `alice_drone::geofence::Geofence` / `WaypointNavigator` に注入する Vec3
    /// はこの座標系に揃えること。
    #[must_use]
    pub fn geodetic_to_drone(&self, target: Geodetic) -> Vec3 {
        let enu = self.geodetic_to_enu(target);
        Vec3::new(enu.x, enu.z, enu.y)
    }

    /// drone Y-up frame → geodetic。`geodetic_to_drone` の逆変換。
    #[must_use]
    pub fn drone_to_geodetic(&self, drone: Vec3) -> Geodetic {
        let enu = Vec3::new(drone.x, drone.z, drone.y);
        self.enu_to_geodetic(enu)
    }

    /// ENU Vec3 → alice_drone Y-up Vec3。純粋な軸並び替え (E,N,U) → (E,U,N)。
    #[must_use]
    pub fn enu_to_drone(&self, enu: Vec3) -> Vec3 {
        Vec3::new(enu.x, enu.z, enu.y)
    }

    /// alice_drone Y-up Vec3 → ENU Vec3。純粋な軸並び替え (E,U,N) → (E,N,U)。
    #[must_use]
    pub fn drone_to_enu(&self, drone: Vec3) -> Vec3 {
        Vec3::new(drone.x, drone.z, drone.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn origin_maps_to_zero_enu() {
        let home = Geodetic::from_degrees(35.681_236, 139.767_125, 40.0);
        let frame = LocalFrame::enu_at(home);
        let enu = frame.geodetic_to_enu(home);
        assert!(approx(enu.x, 0.0, 1e-6));
        assert!(approx(enu.y, 0.0, 1e-6));
        assert!(approx(enu.z, 0.0, 1e-6));
    }

    #[test]
    fn moving_east_gives_positive_east() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let east_of_home = Geodetic::from_degrees(35.0, 139.001, 0.0);
        let enu = frame.geodetic_to_enu(east_of_home);
        assert!(enu.x > 0.0, "east should be positive, got {}", enu.x);
        assert!(enu.y.abs() < 1.0, "north should be near 0, got {}", enu.y);
    }

    #[test]
    fn moving_north_gives_positive_north() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let north_of_home = Geodetic::from_degrees(35.001, 139.0, 0.0);
        let enu = frame.geodetic_to_enu(north_of_home);
        assert!(enu.y > 0.0, "north should be positive, got {}", enu.y);
        assert!(enu.x.abs() < 1.0, "east should be near 0, got {}", enu.x);
    }

    #[test]
    fn moving_up_gives_positive_up() {
        let home = Geodetic::from_degrees(35.0, 139.0, 100.0);
        let frame = LocalFrame::enu_at(home);
        let above = Geodetic::from_degrees(35.0, 139.0, 150.0);
        let enu = frame.geodetic_to_enu(above);
        assert!(approx(enu.z, 50.0, 1e-3));
    }

    #[test]
    fn ned_flips_axes() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let east_north_high = Geodetic::from_degrees(35.001, 139.001, 50.0);
        let enu = frame.geodetic_to_enu(east_north_high);
        let ned = frame.geodetic_to_ned(east_north_high);
        assert!(approx(ned.x, enu.y, 1e-9));
        assert!(approx(ned.y, enu.x, 1e-9));
        assert!(approx(ned.z, -enu.z, 1e-9));
    }

    #[test]
    fn horizontal_distance_matches_pythagoras() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let target = Geodetic::from_degrees(35.0005, 139.0005, 0.0);
        let enu = frame.geodetic_to_enu(target);
        let expected = (enu.x * enu.x + enu.y * enu.y).sqrt();
        let got = frame.horizontal_distance_m(target);
        assert!(approx(got, expected, 1e-9));
    }

    #[test]
    fn ned_roundtrip_preserves_geodetic() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let target = Geodetic::from_degrees(35.001, 139.002, 40.0);
        let ned = frame.geodetic_to_ned(target);
        let back = frame.ned_to_geodetic(ned);
        assert!(approx(back.lat_rad, target.lat_rad, 1e-10));
        assert!(approx(back.lon_rad, target.lon_rad, 1e-10));
        assert!(approx(back.alt_m, target.alt_m, 1e-3));
    }

    #[test]
    fn drone_frame_swaps_up_and_north() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let target = Geodetic::from_degrees(35.001, 139.0, 30.0);
        let enu = frame.geodetic_to_enu(target);
        let drone = frame.geodetic_to_drone(target);
        assert!(approx(drone.x, enu.x, 1e-9));
        assert!(approx(drone.y, enu.z, 1e-9)); // Up ↔ altitude
        assert!(approx(drone.z, enu.y, 1e-9)); // North
        assert!(approx(drone.y, 30.0, 1e-3));
    }

    #[test]
    fn drone_roundtrip_preserves_geodetic() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let target = Geodetic::from_degrees(35.001, 139.002, 40.0);
        let drone = frame.geodetic_to_drone(target);
        let back = frame.drone_to_geodetic(drone);
        assert!(approx(back.lat_rad, target.lat_rad, 1e-10));
        assert!(approx(back.lon_rad, target.lon_rad, 1e-10));
        assert!(approx(back.alt_m, target.alt_m, 1e-3));
    }
}
