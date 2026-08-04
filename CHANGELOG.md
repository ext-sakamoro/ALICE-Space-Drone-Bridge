# Changelog

## [0.1.0] - 2026-08-05

### Added

- `gnss_fix::GnssFix` / `FixKind` — WGS-84 geodetic + DOP + 衛星数 + timestamp を束ねる fix 型
- `coord_convert::LocalFrame` — home 地点固定の ENU / NED frame、`geodetic_to_enu` / `enu_to_geodetic` / `geodetic_to_ned` を提供
- `waypoint_bridge::geodetic_route_to_waypoints` / `geodetic_to_waypoint` — 経路点 → `alice_drone::Waypoint`
- `waypoint_bridge::build_navigator` — 経路点列 → `alice_drone::WaypointNavigator` を一括構築
- `geofence_bridge::circular_geofence_at` — 地理座標中心 + 半径 → `alice_drone::Geofence` (円柱)
- `geofence_bridge::bounding_box_geofence` — 緯度経度 min/max + 高度域 → `alice_drone::Geofence` (矩形)
- `quality_gate::quality_gate` / `QualityPolicy` / `Gate` — DOP / 衛星数 threshold での飛行可否判定
- `prelude` — 主要 API の一括 re-export
- example `qzss_waypoint_navigation` — mock QZSS fix stream → ENU 変換 → waypoint 追従の end-to-end 例
- proptest による geodetic ↔ ENU の往復可逆性検証
