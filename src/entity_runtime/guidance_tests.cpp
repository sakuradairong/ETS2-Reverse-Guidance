#include "../world_runtime/reverse_guidance.hpp"
#include <fstream>
#include <iostream>
#include <string>

using namespace reverse_assist;
namespace
{
int failures = 0;
int checks = 0;
void check(bool condition, const char *message)
{
    ++checks;
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; ++failures; }
}
bool close(double a, double b) { return std::abs(a - b) < 1e-8; }
Prediction forecast(double steering, const TractorSpec &tractor,
                    const std::vector<TrailerSpec> &trailers, double distance = 5.0)
{
    return predict_reverse_boxes(steering, tractor.wheelbase, tractor.length, tractor.width,
        trailers, tractor.rear_axle_x, tractor.rear_axle_z, tractor.center_from_rear,
        distance, 0.05, 0.25, tractor.hook_from_rear, tractor.hook_right_from_rear,
        tractor.front_axle_steering);
}
void check_geometry(const GuidanceGeometry &geometry)
{
    check(!geometry.truncated, "supported horizon must preserve every guidance layer");
    check(geometry.count <= maximum_guidance_segments, "bounded segment count");
    for (std::size_t i = 0; i < geometry.count; ++i)
    {
        check(finite_point(geometry.segments[i].from) && finite_point(geometry.segments[i].to),
              "finite guidance coordinates");
        check(static_cast<std::size_t>(geometry.segments[i].style) <
              static_cast<std::size_t>(GuideStyle::count), "valid material selection");
    }
}
void check_policy_data_sources()
{
    // Suspension compensation makes the tyre plane level despite body tilt.
    std::vector<WheelSpec> wheels{
        {-1.0, -3.0, true, true}, {1.0, -3.0, true, true},
        {-1.0, 1.0, false, true}, {1.0, 1.0, false, true}};
    const double pitch = 0.02, roll = 0.015;
    for (auto &wheel : wheels)
    {
        wheel.suspension_deflection_known = true;
        wheel.suspension_deflection = -body_plane_height(wheel.x, wheel.z, pitch, roll);
        wheel.steering_angle_known = wheel.steerable;
        wheel.steering_angle = 0.2;
    }
    const auto plane = estimate_ground_plane(wheels, pitch, roll);
    auto measured = estimate_tractor_spec(wheels);
    auto inputs = make_guidance_policy_inputs(measured, -1.0, true, plane);
    auto policy = evaluate_guidance_policy(inputs, 5.0);
    check(plane.right_fitted && plane.rearward_fitted && close(inputs.slope_degrees, 0.0),
          "policy uses suspension-compensated tyre plane rather than body attitude");
    check(close(inputs.measured_steering, 0.2) && close(policy.distance, 5.0) && policy.endpoint_outline,
          "wheel measurements reach the policy through the production input adapter");

    // Losing wheel telemetry must not leave the finite steering command trusted.
    for (auto &wheel : wheels) wheel.steering_angle_known = false;
    const auto unmeasured = estimate_tractor_spec(wheels);
    inputs = make_guidance_policy_inputs(unmeasured, -1.0, true, plane);
    policy = evaluate_guidance_policy(inputs, 5.0);
    const double steering_command = 0.5;
    const auto fallback_prediction = forecast(steering_command, unmeasured, {}, policy.distance);
    check(!fallback_prediction.boxes.empty() && !std::isfinite(inputs.measured_steering) &&
          !policy.endpoint_outline && close(policy.distance, 2.0),
          "finite steering command cannot promote a fallback wheel-angle prediction to trusted");

    inputs = make_guidance_policy_inputs(measured, -1.0, false, plane);
    check(!std::isfinite(inputs.signed_speed_mps) &&
          !evaluate_guidance_policy(inputs, 5.0).endpoint_outline,
          "invalidated speed is unknown even when the stored speed remains finite");
    GroundPlane trailer_plane;
    trailer_plane.slope_rearward = std::tan(12.0 * pi / 180.0);
    inputs = make_guidance_policy_inputs(measured, -1.0, true, plane, &trailer_plane);
    policy = evaluate_guidance_policy(inputs, 5.0);
    check(close(inputs.slope_degrees, 12.0) && close(policy.distance, 2.0) && !policy.endpoint_outline,
          "steeper trailer contact plane constrains the complete rig");
    trailer_plane.slope_right = std::numeric_limits<double>::quiet_NaN();
    inputs = make_guidance_policy_inputs(measured, -1.0, true, plane, &trailer_plane);
    check(!std::isfinite(inputs.slope_degrees) &&
          !evaluate_guidance_policy(inputs, 5.0).endpoint_outline,
          "invalid trailer plane cannot be masked by a finite truck plane");
    inputs = make_guidance_policy_inputs(measured, -1.0, true, plane);
    check(close(evaluate_guidance_policy(inputs, 5.0).distance, 5.0),
          "detached trailer no longer constrains the horizon");
    GroundPlane diagonal;
    diagonal.slope_right = diagonal.slope_rearward = std::tan(3.5 * pi / 180.0);
    inputs = make_guidance_policy_inputs(measured, -1.0, true, diagonal);
    check(inputs.slope_degrees > 4.0 && !evaluate_guidance_policy(inputs, 5.0).endpoint_outline,
          "combined longitudinal and lateral grade uses total plane inclination");
    diagonal.slope_right = std::numeric_limits<double>::infinity();
    inputs = make_guidance_policy_inputs(measured, -1.0, true, diagonal, &plane);
    check(!std::isfinite(inputs.slope_degrees), "invalid truck plane remains unknown");
}

void export_scene(std::ofstream &stream, const char *scene, double steering,
                  const TractorSpec &tractor, const std::vector<TrailerSpec> &trailers)
{
    const auto prediction = forecast(steering, tractor, trailers);
    const auto geometry = build_reverse_guidance(prediction, tractor, GuidanceOptions{});
    for (std::size_t i = 0; i < geometry.count; ++i)
    {
        const auto &segment = geometry.segments[i];
        stream << scene << ',' << static_cast<std::size_t>(segment.style) << ','
            << segment.from.x << ',' << segment.from.z << ','
            << segment.to.x << ',' << segment.to.z << '\n';
    }
}
}

int main(int argc, char **argv)
{
    check_policy_data_sources();
    TractorSpec tractor;
    const std::vector<TrailerSpec> trailers{{0.0, 7.2, 3.0, 2.55}};
    GuidanceOptions options;
    const auto straight_prediction = forecast(0.0, tractor, trailers);
    const auto straight = build_reverse_guidance(straight_prediction, tractor, options);
    check_geometry(straight);
    check(straight.front_swing_segments == 0, "straight reversing has no outward front swing");
    check(straight.endpoint_segments > 35, "endpoint outline and orientation arrow exist");
    check(straight.articulation_segments > 0 && straight.trend == ArticulationTrend::steady,
          "aligned trailer has a steady angle marker");
    const auto &end = straight_prediction.boxes.back();
    check(close(body_corner(end, 0.0, end.length * 0.5).z, 15.2),
          "analytic straight trailer rear ends at travel + hitch distance + overhang");

    const auto right_prediction = forecast(0.6, tractor, trailers);
    const auto left_prediction = forecast(-0.6, tractor, trailers);
    const auto right = build_reverse_guidance(right_prediction, tractor, options);
    const auto left = build_reverse_guidance(left_prediction, tractor, options);
    check_geometry(right); check_geometry(left);
    check(right.front_swing_segments > 0 && left.front_swing_segments > 0,
          "turning both ways highlights an outward front corner");
    for (std::size_t i = 0; i < right_prediction.boxes.size(); ++i)
    {
        check(close(right_prediction.boxes[i].pose.x, -left_prediction.boxes[i].pose.x) &&
              close(right_prediction.boxes[i].pose.z, left_prediction.boxes[i].pose.z),
              "mirrored steering produces mirrored body poses");
    }
    check(right.trend == ArticulationTrend::folding, "turning away from alignment increases articulation");

    const std::vector<TrailerSpec> angled{{20.0 * pi / 180.0, 7.2, 3.0, 2.55}};
    const auto straightening = build_reverse_guidance(forecast(-0.5, tractor, angled), tractor, options);
    const auto folding = build_reverse_guidance(forecast(0.0, tractor, angled), tractor, options);
    check(straightening.trend == ArticulationTrend::straightening,
          "counter-steering decreases the actual modelled articulation");
    check(folding.trend == ArticulationTrend::folding,
          "reversing an articulated passive trailer with straight steering folds further");

    const auto solo = build_reverse_guidance(forecast(0.4, tractor, {}), tractor, options);
    check_geometry(solo);
    check(solo.endpoint_segments == 0 && solo.articulation_segments == 0,
          "detached trailer has no stale endpoint or articulation marker");
    options.endpoint_outline = options.front_swing = options.articulation_indicator = false;
    const auto baseline = build_reverse_guidance(straight_prediction, tractor, options);
    check(baseline.count == 80, "feature switches retain the original rear-corner tracks");
    options.enabled = false;
    check(build_reverse_guidance(straight_prediction, tractor, options).count == 0,
          "master disable publishes empty guidance");
    options = GuidanceOptions{};

    std::size_t worst = 0;
    for (double distance : {2.0, 5.0, 8.0})
        for (double steer : {-1.0, -0.5, 0.0, 0.5, 1.0})
            for (double degrees : {-170.0, -90.0, -20.0, 0.0, 20.0, 90.0, 170.0})
            {
                const std::vector<TrailerSpec> long_trailer{{degrees * pi / 180.0, 12.0, 4.0, 3.2}};
                const auto geometry = build_reverse_guidance(forecast(steer, tractor, long_trailer, distance),
                                                             tractor, options);
                check_geometry(geometry);
                check(geometry.endpoint_segments > 0 && geometry.articulation_segments > 0,
                      "extreme articulations preserve endpoint and angle layers");
                worst = std::max(worst, geometry.count);
            }

    StableWheelTopologyFilter<TractorSpec> truck_filter;
    WheelTopologySnapshot<TractorSpec> truck_before;
    truck_before.grounded_mask = 3;
    truck_before.geometry.front_axle_steering = 0.0;
    truck_filter.update(truck_before, 0);
    auto truck_lifting = truck_before;
    truck_lifting.grounded_mask = 1;
    truck_lifting.lift_transition_in_progress = true;
    truck_lifting.geometry.wheelbase = 5.0;
    truck_lifting.geometry.front_axle_steering = 0.4;
    truck_filter.update(truck_lifting, 100);
    check(close(truck_filter.stable().geometry.wheelbase, 3.8) &&
          close(truck_filter.stable().geometry.front_axle_steering, 0.4),
          "lifting keeps support geometry stable while steering stays live");
    truck_lifting.lift_transition_in_progress = false;
    truck_filter.update(truck_lifting, 200);
    truck_filter.update(truck_lifting, 601);
    check(close(truck_filter.stable().geometry.wheelbase, 5.0), "settled topology commits after 400 ms");

    StableWheelTopologyFilter<TrailerSpec> trailer_filter;
    WheelTopologySnapshot<TrailerSpec> trailer_before;
    trailer_before.grounded_mask = 3;
    trailer_filter.update(trailer_before, 0);
    auto trailer_lifting = trailer_before;
    trailer_lifting.lift_transition_in_progress = true;
    trailer_lifting.geometry.heading = 0.5;
    trailer_lifting.geometry.axle_steering = 0.2;
    trailer_filter.update(trailer_lifting, 100);
    check(close(trailer_filter.stable().geometry.heading, 0.5) &&
          close(trailer_filter.stable().geometry.axle_steering, 0.2),
          "trailer articulation and axle steering stay live while lifting");

    Prediction workspace;
    std::vector<Pose> axles;
    predict_reverse_boxes_into(workspace, 0.0, 3.8, 7.0, 2.55, trailers,
                              0.0, 0.0, -2.4, 5.0, 0.05, 0.25, 0.0, 0.0,
                              std::numeric_limits<double>::quiet_NaN(), &axles);
    check(workspace.boxes.size() == straight_prediction.boxes.size(), "reusable predictor matches output size");
    for (std::size_t i = 0; i < workspace.boxes.size(); ++i)
        check(close(workspace.boxes[i].pose.x, straight_prediction.boxes[i].pose.x) &&
              close(workspace.boxes[i].pose.z, straight_prediction.boxes[i].pose.z),
              "reusable predictor preserves numeric results");
    const auto capacity = workspace.boxes.capacity();
    predict_reverse_boxes_into(workspace, 0.0, 3.8, 7.0, 2.55, {},
                              0.0, 0.0, -2.4, 5.0, 0.05, 0.25, 0.0, 0.0,
                              std::numeric_limits<double>::quiet_NaN(), &axles);
    check(workspace.boxes.size() == 21 && workspace.boxes.capacity() == capacity && axles.empty(),
          "detach clears stale state while retaining workspace capacity");
    auto invalid = straight_prediction;
    for (auto &box : invalid.boxes) box.pose.heading = std::numeric_limits<double>::quiet_NaN();
    check(build_reverse_guidance(invalid, tractor, options).count == 0, "invalid poses emit no model transforms");

    GuidancePolicyInputs policy_inputs;
    policy_inputs.measured_steering = 0.2;
    policy_inputs.signed_speed_mps = -1.0;
    policy_inputs.slope_degrees = 0.0;
    const auto nominal_policy = evaluate_guidance_policy(policy_inputs, 5.0);
    check(nominal_policy.show && nominal_policy.endpoint_outline &&
          close(nominal_policy.distance, 5.0) && nominal_policy.steering_measured &&
          nominal_policy.speed_known && nominal_policy.slope_known,
          "measured steering at low reverse speed keeps the configured horizon");

    auto unknown_steering = policy_inputs;
    unknown_steering.measured_steering = std::numeric_limits<double>::quiet_NaN();
    const auto blind = evaluate_guidance_policy(unknown_steering, 5.0);
    check(close(blind.distance, 2.0) && !blind.endpoint_outline && blind.show &&
          !blind.steering_measured,
          "unmeasured steering shortens to the fallback horizon without an endpoint");

    auto unknown_speed = policy_inputs;
    unknown_speed.signed_speed_mps = std::numeric_limits<double>::quiet_NaN();
    const auto no_speed = evaluate_guidance_policy(unknown_speed, 5.0);
    check(close(no_speed.distance, 2.0) && !no_speed.endpoint_outline && !no_speed.speed_known,
          "missing speed shortens to the fallback horizon");
    auto infinite_speed = policy_inputs;
    infinite_speed.signed_speed_mps = std::numeric_limits<double>::infinity();
    const auto infinite = evaluate_guidance_policy(infinite_speed, 5.0);
    check(close(infinite.distance, 2.0) && !infinite.endpoint_outline,
          "non-finite speed falls back to the conservative horizon");

    auto brisk = policy_inputs;
    brisk.signed_speed_mps = -8.0 / 3.6;
    const auto reduced = evaluate_guidance_policy(brisk, 5.0);
    check(reduced.distance < 5.0 && reduced.distance > 2.0 && !reduced.endpoint_outline,
          "8 km/h reverse shortens the horizon and drops the endpoint outline");
    auto quick = policy_inputs;
    quick.signed_speed_mps = -12.0 / 3.6;
    const auto floored = evaluate_guidance_policy(quick, 5.0);
    check(close(floored.distance, 2.0) && floored.show,
          "12 km/h reverse keeps the shortest horizon and still shows tracks");

    auto rolling = policy_inputs;
    rolling.signed_speed_mps = 0.8;
    check(!evaluate_guidance_policy(rolling, 5.0).show,
          "rolling forward with reverse selected hides the guidance");
    rolling.signed_speed_mps = 0.05;
    check(evaluate_guidance_policy(rolling, 5.0).show,
          "small forward creep stays inside the motion deadband");

    auto steep = policy_inputs;
    steep.slope_degrees = 12.0;
    const auto graded = evaluate_guidance_policy(steep, 5.0);
    check(close(graded.distance, 2.0) && !graded.endpoint_outline,
          "steep grade shortens the horizon and disables the endpoint outline");
    auto unknown_slope = policy_inputs;
    unknown_slope.slope_degrees = std::numeric_limits<double>::quiet_NaN();
    const auto unmeasured_grade = evaluate_guidance_policy(unknown_slope, 5.0);
    check(close(unmeasured_grade.distance, 2.0) && !unmeasured_grade.endpoint_outline,
          "non-finite slope falls back to the conservative horizon");

    for (double configured : {1.5, 2.0, 3.0, 8.0})
    {
        auto worst_case = policy_inputs;
        worst_case.signed_speed_mps = -20.0 / 3.6;
        worst_case.slope_degrees = 30.0;
        check(evaluate_guidance_policy(worst_case, configured).distance <= configured + 1e-9,
              "policy never extends the configured horizon");
    }

    GuidanceOptions shortened_options;
    shortened_options.distance = blind.distance;
    shortened_options.endpoint_outline =
        shortened_options.endpoint_outline && blind.endpoint_outline;
    const auto shortened = build_reverse_guidance(
        forecast(0.0, tractor, trailers, shortened_options.distance), tractor, shortened_options);
    check(shortened.count > 0 && shortened.endpoint_segments == 0,
          "shortened horizon keeps the rear tracks and drops the endpoint outline");
    GuidanceOptions user_disabled;
    user_disabled.endpoint_outline = false;
    check(build_reverse_guidance(straight_prediction, tractor, user_disabled).endpoint_segments == 0,
          "user-disabled endpoint outline stays off");
    user_disabled.enabled = false;
    check(build_reverse_guidance(straight_prediction, tractor, user_disabled).count == 0,
          "hidden guidance publishes empty geometry");
    if (argc == 2)
    {
        std::ofstream stream(argv[1]);
        stream << "scene,style,x1,z1,x2,z2\n";
        export_scene(stream, "straight", 0.0, tractor, trailers);
        export_scene(stream, "folding", 0.0, tractor, angled);
        export_scene(stream, "straightening", -0.5, tractor, angled);
        check(stream.good(), "geometry preview export succeeded");
    }
    std::cout << "GuidanceTests: " << (failures ? "FAIL" : "PASS")
              << "; 105 horizon/steering/articulation scenarios; worst segments=" << worst
              << "/" << maximum_guidance_segments << "; checks=" << checks
              << "; failures=" << failures << '\n';
    return failures ? 1 : 0;
}
