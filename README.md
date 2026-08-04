# ALICE-Space-Drone-Bridge

`alice-space` (QZSS / GNSS foundation) と `alice-drone` (UAV flight control) を結線する
決定論的ブリッジクレート

- **QZSS / GNSS 測位 fix (WGS-84 geodetic)** を drone が扱う **local ENU / NED frame** に変換
- **waypoint 変換**: 緯度経度高度で指定した経路点を `alice_drone::Waypoint` (local `Vec3`) に注入
- **geofence bridge**: 緯度経度による地理境界を `alice_drone::Geofence` に map
- **quality gate**: HDOP / VDOP / 衛星数を根拠に飛行可否 (`Gate::Allow` / `Warn` / `Deny`) を判定
- **依存ゼロ (dev-dep 除く)**: `alice-space` + `alice-drone` のみ、両方とも外部依存なし

日本国内の QZSS (準天頂衛星「みちびき」) 補強信号を活用したドローン制御を第一級のユースケースに設計しているが、
API は QZSS に閉じない汎用 GNSS bridge

## Quick start

```rust
use alice_space::Geodetic;
use alice_space_drone_bridge::prelude::*;

// 発着 (Home) 地点
let home = Geodetic::from_degrees(35.681_236, 139.767_125, 0.0); // Tokyo Station
let frame = LocalFrame::enu_at(home);

// QZSS 測位 fix (時刻 + geodetic + DOP + 衛星数)
let fix = GnssFix::builder()
    .position(Geodetic::from_degrees(35.685_5, 139.752_8, 30.0))
    .hdop(1.2)
    .vdop(1.8)
    .satellites_in_use(10)
    .fix_kind(FixKind::QzssAugmented)
    .timestamp_unix_s(1_735_000_000.0)
    .build();

// gate: 飛行して良いか?
match quality_gate(&fix, &QualityPolicy::default()) {
    Gate::Allow => {}
    Gate::Warn(reason) => eprintln!("warn: {reason}"),
    Gate::Deny(reason) => panic!("deny: {reason}"),
}

// drone の Vec3 (alice_drone Y-up: x=East, y=Up, z=North) に変換
// `alice_drone::Geofence` / `Waypoint` に投入する場合はこちらを使う
let local = frame.geodetic_to_drone(fix.position);
println!("east={:.1} m, up={:.1} m, north={:.1} m", local.x, local.y, local.z);

// 純粋 ENU (E, N, U) が必要な場合は geodetic_to_enu を使う
let enu = frame.geodetic_to_enu(fix.position);
println!("E={:.1} N={:.1} U={:.1}", enu.x, enu.y, enu.z);

// waypoint 一括変換
let route = [
    Geodetic::from_degrees(35.686_0, 139.756_0, 40.0),
    Geodetic::from_degrees(35.690_0, 139.760_0, 50.0),
];
let waypoints = geodetic_route_to_waypoints(&frame, &route, 5.0, 3.0);
```

## モジュール

| module | 責務 |
|---|---|
| `gnss_fix` | 測位 fix 型 (`GnssFix` / `FixKind`) |
| `coord_convert` | 地理座標 ↔ ローカル座標変換 (`LocalFrame` / ENU / NED) |
| `waypoint_bridge` | 経路点 → `alice_drone::Waypoint` |
| `geofence_bridge` | 地理領域 → `alice_drone::Geofence` |
| `quality_gate` | DOP / 衛星数 → 飛行 gate 判定 |
| `prelude` | 主要型の再 export |

## Design

- **地理→ローカルの原点固定**: `LocalFrame::enu_at(home)` で決めた原点に対する ENU / NED /
  alice_drone Y-up frame を返す drone が離陸地点からの相対座標で waypoint / geofence /
  control を扱うため
- **座標系の使い分け**: `alice_drone::Geofence` は `pos.y` を altitude として扱う Y-up 系のため、
  `Waypoint` / `Geofence` に注入する Vec3 は `geodetic_to_drone` で得た (E, U, N) を使う
  非 drone 用途 (surveying / mapping) では `geodetic_to_enu` の純粋 ENU (E, N, U) を使う
- **Bowring 経由の高精度往復**: `alice_space::geodetic::{geodetic_to_ecef, ecef_to_geodetic}` を土台に
  ENU 回転を組み合わせ 数 cm 級の可逆性を保つ
- **DOP gate は policy 分離**: `QualityPolicy` で HDOP / VDOP / min satellites を上書き可
- **`no_std` 未対応**: 依存の `alice-drone` `WaypointNavigator` が `Vec` を使うため std 前提

## License

AGPL-3.0-only
