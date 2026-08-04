//! 地理座標経路点 → `alice_drone::Waypoint` / `WaypointNavigator` の変換。

use alice_drone::waypoint::{Waypoint, WaypointNavigator};
use alice_space::Geodetic;

use crate::coord_convert::LocalFrame;

/// `Waypoint` 生成時のパラメータ。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaypointOptions {
    /// 到達判定半径 (メートル)。
    pub arrival_radius_m: f64,
    /// 通過速度 (m/s)。
    pub cruise_speed_mps: f64,
}

impl Default for WaypointOptions {
    fn default() -> Self {
        Self {
            arrival_radius_m: 5.0,
            cruise_speed_mps: 3.0,
        }
    }
}

/// 単一 geodetic 点 → `Waypoint` (alice_drone Y-up 座標系: x=East, y=Up, z=North)。
#[must_use]
pub fn geodetic_to_waypoint(frame: &LocalFrame, target: Geodetic, opts: WaypointOptions) -> Waypoint {
    let local = frame.geodetic_to_drone(target);
    Waypoint::new(local, opts.arrival_radius_m, opts.cruise_speed_mps)
}

/// 経路点列 → `Waypoint` の Vec。
#[must_use]
pub fn geodetic_route_to_waypoints(
    frame: &LocalFrame,
    route: &[Geodetic],
    arrival_radius_m: f64,
    cruise_speed_mps: f64,
) -> Vec<Waypoint> {
    let opts = WaypointOptions {
        arrival_radius_m,
        cruise_speed_mps,
    };
    route
        .iter()
        .map(|g| geodetic_to_waypoint(frame, *g, opts))
        .collect()
}

/// 経路点列から `WaypointNavigator` を一括構築。
#[must_use]
pub fn build_navigator(
    frame: &LocalFrame,
    route: &[Geodetic],
    opts: WaypointOptions,
) -> WaypointNavigator {
    let mut nav = WaypointNavigator::new();
    for g in route {
        nav.add_waypoint(geodetic_to_waypoint(frame, *g, opts));
    }
    nav
}

#[cfg(test)]
mod tests {
    use super::*;
    use alice_drone::waypoint::NavStatus;

    #[test]
    fn single_conversion_places_waypoint_at_drone_frame() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let target = Geodetic::from_degrees(35.0, 139.001, 30.0);
        let wp = geodetic_to_waypoint(&frame, target, WaypointOptions::default());
        // drone frame: x=East (>0)、y=Up=30、z=North (near 0)
        assert!(wp.position.x > 0.0);
        assert!((wp.position.y - 30.0).abs() < 1e-3);
    }

    #[test]
    fn route_conversion_preserves_order_by_north() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let route = [
            Geodetic::from_degrees(35.001, 139.0, 20.0),
            Geodetic::from_degrees(35.002, 139.0, 30.0),
            Geodetic::from_degrees(35.003, 139.0, 40.0),
        ];
        let waypoints = geodetic_route_to_waypoints(&frame, &route, 5.0, 3.0);
        assert_eq!(waypoints.len(), 3);
        // drone frame: z=North (増加方向)
        assert!(waypoints[0].position.z < waypoints[1].position.z);
        assert!(waypoints[1].position.z < waypoints[2].position.z);
    }

    #[test]
    fn navigator_advances_through_waypoints() {
        let home = Geodetic::from_degrees(35.0, 139.0, 0.0);
        let frame = LocalFrame::enu_at(home);
        let route = [
            Geodetic::from_degrees(35.0001, 139.0, 0.0),
            Geodetic::from_degrees(35.0002, 139.0, 0.0),
        ];
        let mut nav = build_navigator(
            &frame,
            &route,
            WaypointOptions {
                arrival_radius_m: 20.0,
                cruise_speed_mps: 5.0,
            },
        );
        assert_eq!(nav.waypoint_count(), 2);

        // home 地点で update すると最初の waypoint への方向が返る
        // drone frame: z=North なので +North waypoint に対して dir.z > 0
        let (dir, speed, status) = nav.update(alice_drone::vec3::Vec3::zero());
        assert_eq!(status, NavStatus::Navigating);
        assert!(speed > 0.0);
        assert!(dir.z > 0.0);
    }
}
