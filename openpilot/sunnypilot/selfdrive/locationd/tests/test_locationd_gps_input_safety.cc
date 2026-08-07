#define CATCH_CONFIG_MAIN
#include "catch2/catch.hpp"

#include <cmath>
#include <limits>

#include "cereal/messaging/messaging.h"
#include "sunnypilot/common/transformations/coordinates.hpp"
#include "sunnypilot/common/transformations/orientation.hpp"
#include "sunnypilot/selfdrive/locationd/locationd.h"
#include "sunnypilot/selfdrive/locationd/models/live_kf.h"

using namespace Eigen;

namespace {

constexpr double kValidLat = 37.3861;
constexpr double kValidLon = -122.0839;
constexpr double kValidAlt = 10.0;
constexpr double kSensorOffset = 0.095;

struct GpsFields {
  bool has_fix = true;
  double latitude = kValidLat;
  double longitude = kValidLon;
  double altitude = kValidAlt;
  float bearing_deg = 90.0f;
  float horizontal_accuracy = 2.0f;
  float vertical_accuracy = 3.0f;
  float speed_accuracy = 0.5f;
  float bearing_accuracy_deg = 5.0f;
  float vn = 0.0f;
  float ve = 0.0f;
  float vd = 0.0f;
};

void fill_gps(cereal::GpsLocationData::Builder gps, const GpsFields &f) {
  gps.setHasFix(f.has_fix);
  gps.setLatitude(f.latitude);
  gps.setLongitude(f.longitude);
  gps.setAltitude(f.altitude);
  gps.setBearingDeg(f.bearing_deg);
  gps.setHorizontalAccuracy(f.horizontal_accuracy);
  gps.setVerticalAccuracy(f.vertical_accuracy);
  gps.setSpeedAccuracy(f.speed_accuracy);
  gps.setBearingAccuracyDeg(f.bearing_accuracy_deg);
  gps.setUnixTimestampMillis(1'700'000'000'000LL);
  auto vned = gps.initVNED(3);
  vned.set(0, f.vn);
  vned.set(1, f.ve);
  vned.set(2, f.vd);
}

void seed_filter_near_gps(Localizer &loc, double t, const GpsFields &f) {
  Geodetic geo = {f.latitude, f.longitude, f.altitude};
  ECEF ecef = geodetic2ecef(geo);
  VectorXd x = loc.get_state();
  MatrixXdr P = loc.get_cov();
  x.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START) = Vector3d(ecef.x, ecef.y, ecef.z);
  loc.reset_kalman(t, x, P);
}

VectorXd quat_to_vector(const Quaterniond &quat) {
  return Vector4d(quat.w(), quat.x(), quat.y(), quat.z());
}

}  // namespace

TEST_CASE("finite_check requires both x and P finite", "[pr75][finite]") {
  Localizer loc;
  const double t = 10.0;
  loc.reset_kalman(t);

  SECTION("x finite, P finite -> no reset") {
    VectorXd x_before = loc.get_state();
    loc.finite_check(t);
    REQUIRE(loc.get_state().isApprox(x_before));
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }

  SECTION("x NaN, P finite -> reset") {
    VectorXd x = loc.get_state();
    MatrixXdr P = loc.get_cov();
    x(STATE_ECEF_POS_START) = std::numeric_limits<double>::quiet_NaN();
    loc.reset_kalman(t, x, P);
    REQUIRE_FALSE(loc.get_state().array().isFinite().all());
    loc.finite_check(t + 1.0);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }

  SECTION("x finite, P NaN -> reset") {
    VectorXd x = loc.get_state();
    MatrixXdr P = loc.get_cov();
    P(0, 0) = std::numeric_limits<double>::quiet_NaN();
    loc.reset_kalman(t, x, P);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE_FALSE(loc.get_cov().array().isFinite().all());
    loc.finite_check(t + 1.0);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }

  SECTION("x Inf, P finite -> reset") {
    VectorXd x = loc.get_state();
    MatrixXdr P = loc.get_cov();
    x(STATE_ECEF_POS_START) = std::numeric_limits<double>::infinity();
    loc.reset_kalman(t, x, P);
    loc.finite_check(t + 1.0);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }

  SECTION("x finite, P Inf -> reset") {
    VectorXd x = loc.get_state();
    MatrixXdr P = loc.get_cov();
    P(0, 0) = std::numeric_limits<double>::infinity();
    loc.reset_kalman(t, x, P);
    loc.finite_check(t + 1.0);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }

  SECTION("both invalid -> reset") {
    VectorXd x = loc.get_state();
    MatrixXdr P = loc.get_cov();
    x(STATE_ECEF_POS_START) = std::numeric_limits<double>::quiet_NaN();
    P(0, 0) = std::numeric_limits<double>::infinity();
    loc.reset_kalman(t, x, P);
    loc.finite_check(t + 1.0);
    REQUIRE(loc.get_state().array().isFinite().all());
    REQUIRE(loc.get_cov().array().isFinite().all());
  }
}

TEST_CASE("handle_gps rejects non-finite numeric inputs before fusion", "[pr75][gps][nonfinite]") {
  Localizer loc;
  loc.reset_kalman(1.0);
  GpsFields base;
  seed_filter_near_gps(loc, 1.0, base);

  auto require_reject = [&](GpsFields f, GpsInputRejectReason reason) {
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    const uint64_t accepted_before = loc.get_gps_input_stats().accepted;
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == reason);
    REQUIRE(loc.get_gps_input_stats().accepted == accepted_before);
    REQUIRE(loc.get_gps_input_stats().rejected_non_finite >= 1);
  };

  SECTION("latitude NaN") {
    GpsFields f = base;
    f.latitude = std::numeric_limits<double>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("longitude Inf") {
    GpsFields f = base;
    f.longitude = std::numeric_limits<double>::infinity();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("altitude NaN") {
    GpsFields f = base;
    f.altitude = std::numeric_limits<double>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("horizontalAccuracy NaN") {
    GpsFields f = base;
    f.horizontal_accuracy = std::numeric_limits<float>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("verticalAccuracy Inf") {
    GpsFields f = base;
    f.vertical_accuracy = std::numeric_limits<float>::infinity();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("speedAccuracy NaN") {
    GpsFields f = base;
    f.speed_accuracy = std::numeric_limits<float>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("bearingAccuracy NaN") {
    GpsFields f = base;
    f.bearing_accuracy_deg = std::numeric_limits<float>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("bearingDeg Inf") {
    GpsFields f = base;
    f.bearing_deg = std::numeric_limits<float>::infinity();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("vNED north NaN") {
    GpsFields f = base;
    f.vn = std::numeric_limits<float>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("vNED east Inf") {
    GpsFields f = base;
    f.ve = std::numeric_limits<float>::infinity();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("vNED down NaN") {
    GpsFields f = base;
    f.vd = std::numeric_limits<float>::quiet_NaN();
    require_reject(f, GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("non-finite current_time") {
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, base);
    loc.handle_gps(std::numeric_limits<double>::quiet_NaN(), gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("non-finite sensor_time_offset") {
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, base);
    const uint64_t accepted_before = loc.get_gps_input_stats().accepted;
    loc.handle_gps(2.0, gps.asReader(), std::numeric_limits<double>::quiet_NaN());
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NonFiniteInput);
    REQUIRE(loc.get_gps_input_stats().accepted == accepted_before);
  }
}

TEST_CASE("handle_gps UBLOX horizontal accuracy semantics", "[pr75][gps][accuracy][ublox]") {
  Localizer loc(LocalizerGnssSource::UBLOX);
  loc.reset_kalman(1.0);
  GpsFields base;
  seed_filter_near_gps(loc, 1.0, base);

  SECTION("finite positive horizontalAccuracy accepted") {
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, base);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
  }
  SECTION("zero horizontalAccuracy rejected") {
    GpsFields f = base;
    f.horizontal_accuracy = 0.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidHorizontalAccuracy);
  }
  SECTION("negative horizontalAccuracy rejected") {
    GpsFields f = base;
    f.horizontal_accuracy = -1.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidHorizontalAccuracy);
  }
  SECTION("NaN horizontalAccuracy rejected as non-finite") {
    GpsFields f = base;
    f.horizontal_accuracy = std::numeric_limits<float>::quiet_NaN();
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NonFiniteInput);
  }
  SECTION("Inf horizontalAccuracy rejected as non-finite") {
    GpsFields f = base;
    f.horizontal_accuracy = std::numeric_limits<float>::infinity();
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NonFiniteInput);
  }
}

TEST_CASE("QCOM legacy zero horizontalAccuracy remains accepted", "[pr75][gps][qcom]") {
  // qcomgpsd leaves horizontalAccuracy at default 0; QCOM gps_variance_factor is 0 so it is
  // unused in covariance. PR75 must not newly reject this legacy path.
  Localizer loc(LocalizerGnssSource::QCOM);
  loc.reset_kalman(1.0);
  GpsFields f;
  f.horizontal_accuracy = 0.0f;
  f.vertical_accuracy = 1.0f;
  f.speed_accuracy = 1.0f;
  f.bearing_accuracy_deg = 1.0f;
  seed_filter_near_gps(loc, 1.0, f);

  MessageBuilder msg;
  auto gps = msg.initEvent().initGpsLocation();
  fill_gps(gps, f);
  loc.handle_gps(2.0, gps.asReader(), 0.630);
  REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
  REQUIRE(loc.get_gps_input_stats().accepted == 1);
  REQUIRE(loc.get_gps_input_stats().rejected_horizontal_accuracy == 0);
}

TEST_CASE("handle_gps accuracy semantics", "[pr75][gps][accuracy]") {
  Localizer loc;
  loc.reset_kalman(1.0);
  GpsFields base;
  seed_filter_near_gps(loc, 1.0, base);

  SECTION("negative horizontal accuracy rejected on UBLOX") {
    GpsFields f = base;
    f.horizontal_accuracy = -1.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidHorizontalAccuracy);
  }
  SECTION("zero vertical accuracy rejected") {
    GpsFields f = base;
    f.vertical_accuracy = 0.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidVerticalAccuracy);
  }
  SECTION("zero speed accuracy rejected") {
    GpsFields f = base;
    f.speed_accuracy = 0.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidSpeedAccuracy);
  }
  SECTION("zero bearing accuracy rejected") {
    GpsFields f = base;
    f.bearing_accuracy_deg = 0.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidBearingAccuracy);
  }
  SECTION("ordinary valid values accepted") {
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, base);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
    REQUIRE(loc.get_gps_input_stats().accepted == 1);
    REQUIRE(loc.is_gps_ok());
  }
}

TEST_CASE("non-finite current_time does not enter KF recovery", "[pr75][gps][time]") {
  Localizer loc;
  const double t0 = 1.0;
  loc.reset_kalman(t0);

  // Inflate position covariance so determine_gps_mode would otherwise call fake-GPS recovery.
  VectorXd x = loc.get_state();
  MatrixXdr P = loc.get_cov();
  P.block<STATE_ECEF_POS_ERR_LEN, STATE_ECEF_POS_ERR_LEN>(STATE_ECEF_POS_ERR_START, STATE_ECEF_POS_ERR_START).diagonal() =
      Vector3d::Constant(1e7);
  loc.reset_kalman(t0, x, P);

  GpsFields base;
  MessageBuilder msg;
  auto gps = msg.initEvent().initGpsLocationExternal();
  fill_gps(gps, base);

  VectorXd x_before = loc.get_state();
  MatrixXdr P_before = loc.get_cov();
  const uint64_t accepted_before = loc.get_gps_input_stats().accepted;

  loc.handle_gps(std::numeric_limits<double>::quiet_NaN(), gps.asReader(), kSensorOffset);

  REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NonFiniteInput);
  REQUIRE(loc.get_gps_input_stats().accepted == accepted_before);
  REQUIRE(loc.get_state().array().isFinite().all());
  REQUIRE(loc.get_cov().array().isFinite().all());
  REQUIRE(loc.get_state().isApprox(x_before));
  REQUIRE(loc.get_cov().isApprox(P_before));
}

TEST_CASE("handle_gps rejection diagnostics classify reasons", "[pr75][gps][diagnostics]") {
  Localizer loc;
  loc.reset_kalman(1.0);
  GpsFields base;
  seed_filter_near_gps(loc, 1.0, base);

  SECTION("no fix") {
    GpsFields f = base;
    f.has_fix = false;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::NoFix);
    REQUIRE(loc.get_gps_input_stats().rejected_no_fix == 1);
  }
  SECTION("invalid lat/lon/alt") {
    GpsFields f = base;
    f.latitude = 95.0;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::InvalidLatLonAlt);
  }
  SECTION("unreasonable velocity") {
    GpsFields f = base;
    f.vn = 250.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::UnreasonableVelocity);
  }
  SECTION("unreasonable uncertainty") {
    GpsFields f = base;
    f.horizontal_accuracy = 2000.0f;
    f.vertical_accuracy = 2000.0f;
    MessageBuilder msg;
    auto gps = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps, f);
    loc.handle_gps(2.0, gps.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::UnreasonableUncertainty);
  }
}

TEST_CASE("large-position reset heading safety", "[pr75][gps][heading]") {
  Localizer loc;
  const double t0 = 1.0;
  loc.reset_kalman(t0);

  GpsFields gps;
  gps.latitude = kValidLat;
  gps.longitude = kValidLon;
  gps.altitude = kValidAlt;
  gps.bearing_deg = 45.0f;
  gps.bearing_accuracy_deg = 5.0f;

  Geodetic far_geo = {kValidLat + 0.01, kValidLon + 0.01, kValidAlt};
  ECEF far_ecef = geodetic2ecef(far_geo);
  VectorXd x = loc.get_state();
  MatrixXdr P = loc.get_cov();
  x.segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START) = Vector3d(far_ecef.x, far_ecef.y, far_ecef.z);
  Vector3d orient_ned(0.0, 0.0, DEG2RAD(200.0));
  VectorXd orient_quat = quat_to_vector(euler2quat(ecef_euler_from_ned(far_ecef, orient_ned)));
  x.segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START) = orient_quat;
  loc.reset_kalman(t0, x, P);

  auto pos_err_to_gps = [&]() {
    Geodetic geo = {gps.latitude, gps.longitude, gps.altitude};
    ECEF ecef = geodetic2ecef(geo);
    Vector3d gps_pos(ecef.x, ecef.y, ecef.z);
    Vector3d filt_pos = loc.get_state().segment<STATE_ECEF_POS_LEN>(STATE_ECEF_POS_START);
    return (filt_pos - gps_pos).norm();
  };

  SECTION("reliable course at meaningful speed can initialize yaw") {
    gps.vn = 6.0f;
    gps.ve = 0.0f;
    gps.bearing_accuracy_deg = 5.0f;
    VectorXd orient_before = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    MessageBuilder msg;
    auto gps_msg = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps_msg, gps);
    loc.handle_gps(2.0, gps_msg.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
    REQUIRE(loc.gps_course_used_for_last_reset());
    REQUIRE(pos_err_to_gps() < 50.0);
    VectorXd orient_after = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    REQUIRE_FALSE(orient_after.isApprox(orient_before, 1e-6));
  }

  SECTION("low-speed course is not trusted for yaw reset") {
    gps.vn = 0.2f;
    gps.ve = 0.0f;
    gps.bearing_accuracy_deg = 5.0f;
    VectorXd orient_before = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    MessageBuilder msg;
    auto gps_msg = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps_msg, gps);
    loc.handle_gps(2.0, gps_msg.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
    REQUIRE_FALSE(loc.gps_course_used_for_last_reset());
    REQUIRE(pos_err_to_gps() < 50.0);
    VectorXd orient_after = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    REQUIRE(orient_after.isApprox(orient_before, 1e-6));
  }

  SECTION("poor bearing accuracy is not trusted for yaw reset") {
    gps.vn = 8.0f;
    gps.ve = 0.0f;
    gps.bearing_accuracy_deg = 80.0f;
    VectorXd orient_before = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    MessageBuilder msg;
    auto gps_msg = msg.initEvent().initGpsLocationExternal();
    fill_gps(gps_msg, gps);
    loc.handle_gps(2.0, gps_msg.asReader(), kSensorOffset);
    REQUIRE(loc.get_gps_input_stats().last_reason == GpsInputRejectReason::Accepted);
    REQUIRE_FALSE(loc.gps_course_used_for_last_reset());
    REQUIRE(pos_err_to_gps() < 50.0);
    VectorXd orient_after = loc.get_state().segment<STATE_ECEF_ORIENTATION_LEN>(STATE_ECEF_ORIENTATION_START);
    REQUIRE(orient_after.isApprox(orient_before, 1e-6));
  }
}
