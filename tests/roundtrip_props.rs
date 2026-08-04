//! Geodetic ↔ ENU 往復の可逆性を proptest で検証。

use alice_space::Geodetic;
use alice_space_drone_bridge::LocalFrame;
use proptest::prelude::*;

proptest! {
    // home 付近 ±0.05° / ±500m の任意点で往復
    #[test]
    fn enu_roundtrip_is_reversible(
        home_lat_deg in -60.0f64..60.0f64,
        home_lon_deg in -179.0f64..179.0f64,
        d_lat_deg in -0.05f64..0.05f64,
        d_lon_deg in -0.05f64..0.05f64,
        d_alt_m in -500.0f64..500.0f64,
    ) {
        let home = Geodetic::from_degrees(home_lat_deg, home_lon_deg, 0.0);
        let target = Geodetic::from_degrees(
            home_lat_deg + d_lat_deg,
            home_lon_deg + d_lon_deg,
            d_alt_m,
        );
        let frame = LocalFrame::enu_at(home);
        let enu = frame.geodetic_to_enu(target);
        let back = frame.enu_to_geodetic(enu);
        // ~1e-9 rad = ~6 mm、alt は ~1 mm 相当
        prop_assert!((back.lat_rad - target.lat_rad).abs() < 1e-9);
        prop_assert!((back.lon_rad - target.lon_rad).abs() < 1e-9);
        prop_assert!((back.alt_m - target.alt_m).abs() < 1e-2);
    }

    #[test]
    fn ned_roundtrip_is_reversible(
        home_lat_deg in -60.0f64..60.0f64,
        home_lon_deg in -179.0f64..179.0f64,
        d_lat_deg in -0.05f64..0.05f64,
        d_lon_deg in -0.05f64..0.05f64,
        d_alt_m in -500.0f64..500.0f64,
    ) {
        let home = Geodetic::from_degrees(home_lat_deg, home_lon_deg, 0.0);
        let target = Geodetic::from_degrees(
            home_lat_deg + d_lat_deg,
            home_lon_deg + d_lon_deg,
            d_alt_m,
        );
        let frame = LocalFrame::enu_at(home);
        let ned = frame.geodetic_to_ned(target);
        let back = frame.ned_to_geodetic(ned);
        prop_assert!((back.lat_rad - target.lat_rad).abs() < 1e-9);
        prop_assert!((back.lon_rad - target.lon_rad).abs() < 1e-9);
        prop_assert!((back.alt_m - target.alt_m).abs() < 1e-2);
    }
}
