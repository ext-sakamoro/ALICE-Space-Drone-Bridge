//! ALICE-Space-Drone-Bridge — GNSS (QZSS 対応) 実用層 + drone 制御ブリッジ
//!
//! `alice-space` の GNSS 基盤 (`qzss_l1s` / `qzss_l6` / `sbas` / `rtcm3` /
//! `geodetic` / `dop` / `ionosphere` / `troposphere` / `cycle_slip` /
//! `multipath`) を、実用パイプラインに束ねる compose 層を提供し、
//! `alice-drone` の飛行制御 (`waypoint` / `geofence` / `vec3`) へ橋渡しする。
//!
//! # 汎用 GNSS utility 層
//!
//! - [`gnss_fix`] — 測位 fix 型 ([`GnssFix`] / [`FixKind`])
//! - [`corrections`] — Klobuchar 電離層 / Saastamoinen 対流圏補正の fix 適用
//! - [`integrity`] — DOP + 衛星数 + fix age + cycle-slip flag を統合した完全性監視
//! - [`position_filter`] — EWMA / 移動平均による GNSS jitter 抑制
//! - [`fix_aggregator`] — 複数の測位ソースを重み付き合成
//! - [`quality_gate`] — 飛行 / 動作可否 gate ([`Gate`] / [`QualityPolicy`])
//!
//! # Drone 制御ブリッジ
//!
//! - [`coord_convert`] — 地理座標 ↔ ローカル ENU / NED frame
//! - [`waypoint_bridge`] — 経路点 → `alice_drone::Waypoint` / `WaypointNavigator`
//! - [`geofence_bridge`] — 地理領域 → `alice_drone::Geofence`
//!
//! # Quick start
//!
//! ```
//! use alice_space::Geodetic;
//! use alice_space_drone_bridge::prelude::*;
//!
//! let home = Geodetic::from_degrees(35.681_236, 139.767_125, 0.0);
//! let frame = LocalFrame::enu_at(home);
//!
//! let fix = GnssFix::builder()
//!     .position(Geodetic::from_degrees(35.685_5, 139.752_8, 30.0))
//!     .hdop(1.2)
//!     .vdop(1.8)
//!     .satellites_in_use(10)
//!     .fix_kind(FixKind::QzssL1sAugmented)
//!     .timestamp_unix_s(1_735_000_000.0)
//!     .build();
//!
//! assert!(matches!(quality_gate(&fix, &QualityPolicy::default()), Gate::Allow));
//! let local = frame.geodetic_to_enu(fix.position);
//! assert!(local.x.abs() < 5000.0 && local.y.abs() < 5000.0);
//! ```

#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::module_name_repetitions,
    clippy::missing_const_for_fn,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::doc_markdown,
    clippy::float_cmp,
    clippy::unreadable_literal,
    clippy::suboptimal_flops,
    clippy::imprecise_flops
)]

pub mod coord_convert;
pub mod corrections;
pub mod fix_aggregator;
pub mod geofence_bridge;
pub mod gnss_fix;
pub mod integrity;
pub mod position_filter;
pub mod prelude;
pub mod quality_gate;
pub mod waypoint_bridge;

pub use coord_convert::LocalFrame;
pub use corrections::{
    correct_ionosphere, correct_troposphere, IonosphereContext, TroposphereContext,
};
pub use fix_aggregator::{FixAggregator, WeightedFix};
pub use geofence_bridge::{bounding_box_geofence, circular_geofence_at};
pub use gnss_fix::{FixKind, GnssFix, GnssFixBuilder};
pub use integrity::{IntegrityMonitor, IntegrityReport, IntegrityStatus};
pub use position_filter::{EwmaFilter, PositionFilter};
pub use quality_gate::{quality_gate, Gate, QualityPolicy};
pub use waypoint_bridge::{
    build_navigator, geodetic_route_to_waypoints, geodetic_to_waypoint, WaypointOptions,
};
