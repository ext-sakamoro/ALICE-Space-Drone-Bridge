//! QZSS 想定 fix stream → ENU 変換 → waypoint 追従の end-to-end 例。
//!
//! GNSS 受信機は外部依存 mock 化ルールに従い、この example では
//! stub な fix stream を用意する。実受信機との統合は
//! `#[cfg(feature = "integration")]` gate 越しに行うことを推奨。
//!
//! 実行:
//! ```bash
//! cargo run --example qzss_waypoint_navigation
//! ```

use alice_drone::vec3::Vec3;
use alice_drone::waypoint::NavStatus;
use alice_space::Geodetic;
use alice_space_drone_bridge::prelude::*;

/// stub QZSS 受信機。順に fix を返す。
struct StubReceiver {
    fixes: Vec<GnssFix>,
    idx: usize,
}

impl StubReceiver {
    fn new(fixes: Vec<GnssFix>) -> Self {
        Self { fixes, idx: 0 }
    }
    fn next_fix(&mut self) -> Option<GnssFix> {
        let f = self.fixes.get(self.idx).copied()?;
        self.idx += 1;
        Some(f)
    }
}

fn main() {
    // Home (Tokyo Station 付近) を基準とする ENU frame。
    let home = Geodetic::from_degrees(35.681_236, 139.767_125, 0.0);
    let frame = LocalFrame::enu_at(home);

    // 経路点を geodetic で定義 (北 100m ステップ、上昇 20m)。
    let route = [
        Geodetic::from_degrees(35.682_000, 139.767_125, 20.0),
        Geodetic::from_degrees(35.682_800, 139.767_125, 30.0),
        Geodetic::from_degrees(35.683_600, 139.767_125, 40.0),
    ];
    let mut nav = build_navigator(
        &frame,
        &route,
        WaypointOptions {
            arrival_radius_m: 10.0,
            cruise_speed_mps: 4.0,
        },
    );
    println!(
        "route loaded: {} waypoints, total distance = {:.1} m",
        nav.waypoint_count(),
        nav.total_distance()
    );

    // Ring 状の geofence: home 中心 500m 円柱、高度 5-80m
    let fence = circular_geofence_at(&frame, home, 500.0, 5.0, 80.0, 20.0);

    // stub 受信機: 経路点方向へ徐々に前進する fix を返す
    let mut receiver = StubReceiver::new(
        (0..10)
            .map(|i| {
                let lat_deg = 35.681_236 + i as f64 * 0.0002;
                GnssFix::builder()
                    .position(Geodetic::from_degrees(lat_deg, 139.767_125, 20.0))
                    .hdop(1.1)
                    .vdop(1.6)
                    .satellites_in_use(11)
                    .fix_kind(FixKind::QzssL1sAugmented)
                    .timestamp_unix_s(1_735_000_000.0 + i as f64 * 0.2)
                    .build()
            })
            .collect(),
    );

    // 位置平滑化フィルタ (受信機ジッタ抑制)。
    let mut filter = EwmaFilter::new(0.4);

    // 完全性監視。
    let monitor = IntegrityMonitor::default();
    let policy = QualityPolicy::default();

    while let Some(fix) = receiver.next_fix() {
        // 1) 完全性チェック
        let now = fix.timestamp_unix_s;
        let integrity = monitor.evaluate(&fix, now, 0);
        // 2) gate 判定
        let gate = quality_gate(&fix, &policy);
        // 3) drone frame (x=East, y=Up, z=North) に変換 + 平滑化
        let raw_local = frame.geodetic_to_drone(fix.position);
        let smoothed = filter.update(raw_local);
        // 4) geofence チェック (drone frame と同じ座標系で照合)
        let fence_status = fence.check(smoothed);
        // 5) waypoint 追従更新
        let (dir, target_speed, nav_status) = nav.update(smoothed);

        println!(
            "t={:.1}s gate={:?} integrity={:?} fence={:?} local=(E{:.1} U{:.1} N{:.1}) dir=(E{:.2} U{:.2} N{:.2}) speed={:.1} nav={:?}",
            fix.timestamp_unix_s - 1_735_000_000.0,
            gate,
            integrity.status,
            fence_status,
            smoothed.x,
            smoothed.y,
            smoothed.z,
            dir.x,
            dir.y,
            dir.z,
            target_speed,
            nav_status,
        );

        if matches!(nav_status, NavStatus::Completed) {
            break;
        }
    }

    // 集約デモ: 複数ソースを合成
    let rtk = GnssFix::builder()
        .position(Geodetic::from_degrees(35.682_500, 139.767_130, 25.0))
        .hdop(0.8)
        .vdop(1.2)
        .satellites_in_use(15)
        .fix_kind(FixKind::RtkFixed)
        .timestamp_unix_s(1_735_000_050.0)
        .build();
    let autonomous = GnssFix::builder()
        .position(Geodetic::from_degrees(35.682_800, 139.767_400, 26.0))
        .hdop(2.4)
        .vdop(3.2)
        .satellites_in_use(7)
        .fix_kind(FixKind::Autonomous)
        .timestamp_unix_s(1_735_000_049.0)
        .build();
    let agg = FixAggregator::new(&frame);
    if let Some(merged) = agg.aggregate(&[WeightedFix::new(rtk), WeightedFix::new(autonomous)]) {
        let drone = frame.geodetic_to_drone(merged.position);
        println!(
            "aggregated fix: kind={:?}, local=(E{:.2} U{:.2} N{:.2}), pdop={:.2}",
            merged.fix_kind,
            drone.x,
            drone.y,
            drone.z,
            merged.pdop()
        );
    }

    // 静的解析回避のダミー参照。
    let _ = Vec3::zero();
}
