#pragma once

#include "reverse_kinematics.hpp"
#include <array>
#include <limits>

namespace reverse_assist
{
constexpr std::size_t maximum_guidance_segments = 256;
constexpr std::size_t maximum_guidance_track_points = 40;

enum class GuideStyle : std::size_t
{
    tractor, trailer, preview, straightening, steady, target,
    cue_off, cue_edit, cue_wait, cue_left, cue_right, cue_hold, cue_straight,
    cue_stop, cue_arrived, cue_limit, cue_move, cue_data, cue_slow, count
};

enum class ArticulationTrend { steady, straightening, folding };

enum class JackknifeRisk { none, caution, critical };

enum class JackknifeAdvice { none, countersteer_left, countersteer_right, pull_forward };

enum class GuideLayer : std::uint8_t
{
    tractor_track,
    trailer_track,
    front_swing,
    endpoint,
    articulation,
    corridor,
    target_box,
    target_route,
    cue,
};

struct GuidanceOptions
{
    bool enabled = true;
    bool endpoint_outline = true;
    bool front_swing = true;
    bool articulation_indicator = true;
    bool trailer_corridor = true;
    bool session_latch = true;
    bool jackknife_cues = true;
    bool diagnostics = false;
    double distance = 5.0;
    double corridor_distance = 6.0;
};

struct ReverseSessionLatch
{
    bool engaged = false;
    bool preview_hold = false;
    std::uint64_t last_reverse_ms = 0;
    double anchor_x = 0.0;
    double anchor_z = 0.0;

    static constexpr std::uint64_t hold_timeout_ms = 20000;
    static constexpr double max_hold_speed_mps = 2.8;
    static constexpr double release_speed_mps = 3.2;
    static constexpr double max_hold_distance_m = 12.0;
    static constexpr double preview_hold_horizon_m = 4.5;

    void reset()
    {
        engaged = false;
        preview_hold = false;
        last_reverse_ms = 0;
        anchor_x = 0.0;
        anchor_z = 0.0;
    }

    // Keep reverse guidance live during short neutral pauses or low-speed
    // forward pull-up corrections right after reversing.
    bool update(int gear, double speed_mps, bool speed_valid,
                double world_x, double world_z, std::uint64_t now_ms)
    {
        if (gear < 0)
        {
            engaged = true;
            preview_hold = false;
            last_reverse_ms = now_ms;
            anchor_x = world_x;
            anchor_z = world_z;
            return true;
        }
        if (!engaged)
        {
            preview_hold = false;
            return false;
        }
        const double abs_speed = speed_valid && std::isfinite(speed_mps)
            ? std::abs(speed_mps) : 0.0;
        const double moved = std::hypot(world_x - anchor_x, world_z - anchor_z);
        const bool timed_out = now_ms >= last_reverse_ms &&
            (now_ms - last_reverse_ms) > hold_timeout_ms;
        if (abs_speed > release_speed_mps || moved > max_hold_distance_m || timed_out)
        {
            reset();
            return false;
        }
        if (gear > 0 && abs_speed > max_hold_speed_mps)
        {
            reset();
            return false;
        }
        preview_hold = true;
        return true;
    }
};

// The reverse guide is a display aid drawn from a fixed-horizon kinematic
// model, so it is only defensible while the assumptions behind that model
// hold: a measured steering angle, a walking-pace vehicle and flat ground.
// These limits are deliberately conservative operator-facing heuristics, not
// calibrated vehicle physics, and they only ever shorten or hide the guide.
constexpr double guidance_minimum_distance = 2.0;
constexpr double guidance_speed_intact_kmh = 5.0;
constexpr double guidance_speed_minimum_kmh = 10.0;
constexpr double guidance_forward_creep_mps = 0.3;
constexpr double guidance_slope_intact_degrees = 4.0;
constexpr double guidance_slope_minimum_degrees = 10.0;

struct GuidancePolicyInputs
{
    // Measured effective steering angle in radians; NaN when unmeasured.
    double measured_steering = std::numeric_limits<double>::quiet_NaN();
    // Signed vehicle speed in m/s, negative while reversing; NaN when unknown.
    double signed_speed_mps = std::numeric_limits<double>::quiet_NaN();
    // Largest estimated ground-plane inclination in degrees; NaN when invalid.
    double slope_degrees = std::numeric_limits<double>::quiet_NaN();
};

struct GuidancePolicy
{
    bool show = true;
    bool endpoint_outline = true;
    double distance = 5.0;
    bool steering_measured = false;
    bool speed_known = false;
    bool slope_known = false;
};

inline GuidancePolicyInputs make_guidance_policy_inputs(
    const TractorSpec &tractor, double signed_speed_mps, bool speed_valid,
    const GroundPlane &truck_plane, const GroundPlane *trailer_plane = nullptr)
{
    const auto inclination = [](const GroundPlane &plane) {
        if (!std::isfinite(plane.slope_right) || !std::isfinite(plane.slope_rearward))
            return std::numeric_limits<double>::quiet_NaN();
        return std::atan(std::hypot(plane.slope_right, plane.slope_rearward)) * 180.0 / pi;
    };
    GuidancePolicyInputs inputs;
    // Use the same measured road-wheel angle as the kinematic predictor.
    // A finite normalized steering command does not establish a measurement.
    inputs.measured_steering = tractor.front_axle_steering;
    inputs.signed_speed_mps = speed_valid ? signed_speed_mps :
        std::numeric_limits<double>::quiet_NaN();
    inputs.slope_degrees = inclination(truck_plane);
    if (trailer_plane)
    {
        const double trailer_slope = inclination(*trailer_plane);
        inputs.slope_degrees = std::isfinite(inputs.slope_degrees) && std::isfinite(trailer_slope)
            ? std::max(inputs.slope_degrees, trailer_slope)
            : std::numeric_limits<double>::quiet_NaN();
    }
    return inputs;
}

inline GuidancePolicy evaluate_guidance_policy(const GuidancePolicyInputs &inputs,
                                              double configured_distance)
{
    GuidancePolicy policy;
    const double configured = std::isfinite(configured_distance)
        ? configured_distance
        : guidance_minimum_distance;
    policy.distance = configured;
    // Blend from the configured horizon towards the fallback horizon. The
    // result never exceeds the configured distance.
    const auto shorten = [&policy, configured](double blend) {
        const double scaled = configured * (1.0 - blend) +
            guidance_minimum_distance * blend;
        policy.distance = std::min(policy.distance,
                                   std::max(guidance_minimum_distance, scaled));
    };

    policy.steering_measured = std::isfinite(inputs.measured_steering);
    if (!policy.steering_measured)
    {
        shorten(1.0);
        policy.endpoint_outline = false;
    }

    policy.speed_known = std::isfinite(inputs.signed_speed_mps);
    if (!policy.speed_known)
    {
        shorten(1.0);
        policy.endpoint_outline = false;
    }
    else
    {
        if (inputs.signed_speed_mps > guidance_forward_creep_mps)
            policy.show = false;
        const double speed_kmh = std::abs(inputs.signed_speed_mps) * 3.6;
        if (speed_kmh > guidance_speed_intact_kmh)
        {
            shorten(std::clamp((speed_kmh - guidance_speed_intact_kmh) /
                               (guidance_speed_minimum_kmh - guidance_speed_intact_kmh),
                               0.0, 1.0));
            policy.endpoint_outline = false;
        }
    }

    policy.slope_known = std::isfinite(inputs.slope_degrees);
    if (!policy.slope_known)
    {
        shorten(1.0);
        policy.endpoint_outline = false;
    }
    else
    {
        const double slope = std::abs(inputs.slope_degrees);
        if (slope > guidance_slope_intact_degrees)
        {
            shorten(std::clamp((slope - guidance_slope_intact_degrees) /
                               (guidance_slope_minimum_degrees - guidance_slope_intact_degrees),
                               0.0, 1.0));
            policy.endpoint_outline = false;
        }
    }
    return policy;
}

struct GuideSegment
{
    BoundaryPoint from{};
    BoundaryPoint to{};
    BodyKind body = BodyKind::tractor;
    GuideStyle style = GuideStyle::tractor;
    GuideLayer layer = GuideLayer::tractor_track;
};

struct GuidanceGeometry
{
    std::array<GuideSegment, maximum_guidance_segments> segments{};
    std::size_t count = 0;
    std::size_t endpoint_segments = 0;
    std::size_t front_swing_segments = 0;
    std::size_t articulation_segments = 0;
    std::size_t corridor_segments = 0;
    double articulation_degrees = 0.0;
    double predicted_articulation_degrees = 0.0;
    double jackknife_critical_degrees = 0.0;
    ArticulationTrend trend = ArticulationTrend::steady;
    JackknifeRisk jackknife_risk = JackknifeRisk::none;
    JackknifeAdvice jackknife_advice = JackknifeAdvice::none;
    bool truncated = false;
};

inline BoundaryPoint body_corner(const GroundBox &body,
                                 double lateral, double longitudinal)
{
    return {body.pose.x + std::cos(body.pose.heading) * lateral +
                std::sin(body.pose.heading) * longitudinal,
            0.0,
            body.pose.z - std::sin(body.pose.heading) * lateral +
                std::cos(body.pose.heading) * longitudinal};
}

inline bool finite_point(const BoundaryPoint &point)
{
    return std::isfinite(point.x) && std::isfinite(point.z);
}

inline void add_guide_segment(GuidanceGeometry &output,
                              const BoundaryPoint &from,
                              const BoundaryPoint &to,
                              BodyKind body, GuideStyle style,
                              GuideLayer layer = GuideLayer::tractor_track)
{
    if (!finite_point(from) || !finite_point(to) ||
        std::hypot(to.x - from.x, to.z - from.z) < 0.08)
        return;
    if (output.count == output.segments.size())
    {
        output.truncated = true;
        return;
    }
    output.segments[output.count++] = {from, to, body, style, layer};
}

inline void add_dashed_edge(GuidanceGeometry &output,
                            const BoundaryPoint &from,
                            const BoundaryPoint &to,
                            std::size_t samples, BodyKind body,
                            GuideStyle style,
                            GuideLayer layer = GuideLayer::endpoint)
{
    if (!finite_point(from) || !finite_point(to)) return;
    samples = std::clamp<std::size_t>(samples, 1, 16);
    const double length = std::hypot(to.x - from.x, to.z - from.z);
    if (length < 0.08) return;
    // The shipped line mesh is 0.36 m long. Distribute fixed-length dashes
    // instead of pretending the placement transform can scale the mesh.
    const double half_dash = std::min(0.18, length / (2.0 * samples));
    const double dx = (to.x - from.x) / length;
    const double dz = (to.z - from.z) / length;
    for (std::size_t index = 0; index < samples; ++index)
    {
        const double t = (static_cast<double>(index) + 0.5) / samples;
        const double x = from.x + (to.x - from.x) * t;
        const double z = from.z + (to.z - from.z) * t;
        add_guide_segment(output, {x - dx * half_dash, 0.0, z - dz * half_dash},
                          {x + dx * half_dash, 0.0, z + dz * half_dash}, body, style, layer);
    }
}

inline GuidanceGeometry build_reverse_guidance(
    const Prediction &prediction, const TractorSpec &tractor,
    const GuidanceOptions &options, const TrailerSpec *trailer = nullptr)
{
    GuidanceGeometry output;
    if (!options.enabled || prediction.boxes.empty()) return output;
    struct Track
    {
        std::array<BoundaryPoint, maximum_guidance_track_points> points{};
        std::size_t count = 0;
    };
    std::array<Track, 4> rear_tracks{};
    std::array<Track, 2> front_tracks{};
    const GroundBox *endpoint = nullptr;
    const GroundBox *initial_trailer = nullptr;
    const GroundBox *near_trailer = nullptr;
    const GroundBox *near_tractor = nullptr;
    const auto push_point = [&](Track &track, const BoundaryPoint &point) {
        if (track.count < track.points.size()) track.points[track.count++] = point;
        else output.truncated = true;
    };
    for (const auto &box : prediction.boxes)
    {
        if (!std::isfinite(box.pose.x) || !std::isfinite(box.pose.z) ||
            !std::isfinite(box.pose.heading) || !std::isfinite(box.width) ||
            !std::isfinite(box.length) || box.width <= 0.0 || box.length <= 0.0)
            continue;
        if (box.kind == BodyKind::trailer && box.trailer_index != 0) continue;
        const auto boundary = reverse_leading_boundary(box);
        const std::size_t start = box.kind == BodyKind::tractor ? 0 : 2;
        push_point(rear_tracks[start], boundary.left);
        push_point(rear_tracks[start + 1], boundary.right);
        if (box.kind == BodyKind::tractor)
        {
            push_point(front_tracks[0], body_corner(box, -box.width * 0.5 - 0.18,
                                                   -box.length * 0.5));
            push_point(front_tracks[1], body_corner(box, box.width * 0.5 + 0.18,
                                                   -box.length * 0.5));
            if (!near_tractor && box.prediction_distance >= 1.0 - 1e-6)
                near_tractor = &box;
        }
        else
        {
            if (!initial_trailer) initial_trailer = &box;
            if (!near_trailer && box.prediction_distance >= 1.0 - 1e-6)
                near_trailer = &box;
            endpoint = &box;
        }
    }
    const auto emit_track = [&](Track &track, BodyKind body, GuideStyle style, GuideLayer layer) {
        const auto original = track.points;
        for (std::size_t index = 1; index + 1 < track.count; ++index)
        {
            track.points[index].x = original[index - 1].x * 0.25 +
                original[index].x * 0.5 + original[index + 1].x * 0.25;
            track.points[index].z = original[index - 1].z * 0.25 +
                original[index].z * 0.5 + original[index + 1].z * 0.25;
        }
        for (std::size_t index = 0; index + 1 < track.count; ++index)
            add_guide_segment(output, track.points[index], track.points[index + 1], body, style, layer);
    };
    emit_track(rear_tracks[0], BodyKind::tractor, GuideStyle::tractor, GuideLayer::tractor_track);
    emit_track(rear_tracks[1], BodyKind::tractor, GuideStyle::tractor, GuideLayer::tractor_track);
    emit_track(rear_tracks[2], BodyKind::trailer, GuideStyle::trailer, GuideLayer::trailer_track);
    emit_track(rear_tracks[3], BodyKind::trailer, GuideStyle::trailer, GuideLayer::trailer_track);

    if (options.front_swing && front_tracks[0].count > 1)
    {
        double left_excursion = 0.0, right_excursion = 0.0;
        for (std::size_t index = 1; index < front_tracks[0].count; ++index)
        {
            left_excursion = std::max(left_excursion,
                front_tracks[0].points[0].x - front_tracks[0].points[index].x);
            right_excursion = std::max(right_excursion,
                front_tracks[1].points[index].x - front_tracks[1].points[0].x);
        }
        if (std::max(left_excursion, right_excursion) > 0.05)
        {
            const auto before = output.count;
            emit_track(front_tracks[left_excursion >= right_excursion ? 0 : 1],
                       BodyKind::tractor, GuideStyle::tractor, GuideLayer::front_swing);
            output.front_swing_segments = output.count - before;
        }
    }

    if (endpoint && options.endpoint_outline)
    {
        const auto before = output.count;
        const auto rear_left = body_corner(*endpoint, -endpoint->width * 0.5, endpoint->length * 0.5);
        const auto rear_right = body_corner(*endpoint, endpoint->width * 0.5, endpoint->length * 0.5);
        const auto front_left = body_corner(*endpoint, -endpoint->width * 0.5, -endpoint->length * 0.5);
        const auto front_right = body_corner(*endpoint, endpoint->width * 0.5, -endpoint->length * 0.5);
        const auto edge = [&](const BoundaryPoint &a, const BoundaryPoint &b, std::size_t samples) {
            add_dashed_edge(output, a, b, samples, BodyKind::trailer, GuideStyle::preview, GuideLayer::endpoint);
        };
        edge(front_left, rear_left, 12);
        edge(front_right, rear_right, 12);
        edge(front_left, front_right, 4);
        edge(rear_left, rear_right, static_cast<std::size_t>(std::ceil(endpoint->width / 0.25)));
        // Point toward the physical FRONT of the trailer. +heading in the
        // reverse predictor points at the rear, so the arrow needs -heading.
        const double sx = -std::sin(endpoint->pose.heading);
        const double sz = -std::cos(endpoint->pose.heading);
        const double rx = std::cos(endpoint->pose.heading);
        const double rz = -std::sin(endpoint->pose.heading);
        const BoundaryPoint base{endpoint->pose.x - sx * 0.50, 0.0, endpoint->pose.z - sz * 0.50};
        const BoundaryPoint tip{endpoint->pose.x + sx * 0.75, 0.0, endpoint->pose.z + sz * 0.75};
        edge(base, tip, 5);
        edge({tip.x - sx * 0.40 - rx * 0.28, 0.0, tip.z - sz * 0.40 - rz * 0.28}, tip, 2);
        edge({tip.x - sx * 0.40 + rx * 0.28, 0.0, tip.z - sz * 0.40 + rz * 0.28}, tip, 2);
        output.endpoint_segments = output.count - before;
    }

    if (initial_trailer && near_trailer && near_tractor)
    {
        const double current_angle = normalize_angle(initial_trailer->pose.heading);
        const double future_angle = normalize_angle(near_trailer->pose.heading - near_tractor->pose.heading);
        output.articulation_degrees = current_angle * 180.0 / pi;
        output.predicted_articulation_degrees = future_angle * 180.0 / pi;
        const double axle_to_hitch = trailer
            ? trailer->axle_to_hitch
            : std::clamp(initial_trailer->length * 0.68, 4.5, 11.0);
        const double crit_rad = jackknife_critical_angle(
            tractor.wheelbase, tractor.hook_from_rear, axle_to_hitch);
        output.jackknife_critical_degrees = crit_rad * 180.0 / pi;
        const double change = std::abs(future_angle) - std::abs(current_angle);
        if (change < -1.0 * pi / 180.0) output.trend = ArticulationTrend::straightening;
        else if (change > 1.0 * pi / 180.0) output.trend = ArticulationTrend::folding;

        const double abs_current = std::abs(output.articulation_degrees);
        const double abs_future = std::abs(output.predicted_articulation_degrees);
        const double crit_deg = output.jackknife_critical_degrees;
        if (abs_current >= crit_deg * 0.88 ||
            (output.trend == ArticulationTrend::folding && abs_future >= crit_deg * 0.92))
        {
            output.jackknife_risk = JackknifeRisk::critical;
        }
        else if (abs_current >= std::max(30.0, crit_deg * 0.70) ||
                 (output.trend == ArticulationTrend::folding &&
                  abs_future >= std::max(32.0, crit_deg * 0.78)))
        {
            output.jackknife_risk = JackknifeRisk::caution;
        }

        if (output.jackknife_risk == JackknifeRisk::critical)
            output.jackknife_advice = JackknifeAdvice::pull_forward;
        else if (output.jackknife_risk == JackknifeRisk::caution &&
                 output.trend == ArticulationTrend::folding)
            output.jackknife_advice = current_angle >= 0.0
                ? JackknifeAdvice::countersteer_right
                : JackknifeAdvice::countersteer_left;

        if (options.articulation_indicator)
        {
            const auto before = output.count;
            const auto style = output.jackknife_risk == JackknifeRisk::critical
                ? GuideStyle::trailer
                : (output.trend == ArticulationTrend::straightening ? GuideStyle::straightening :
                   output.trend == ArticulationTrend::folding ? GuideStyle::trailer : GuideStyle::steady);
            // Put the angle marker beside the hitch, outside the tractor body.
            const double cx = tractor.rear_axle_x - tractor.width * 0.5 - 1.1;
            const double cz = tractor.rear_axle_z + tractor.hook_from_rear;
            const auto radial = [&](double angle, double radius) -> BoundaryPoint {
                return {cx + std::sin(angle) * radius, 0.0, cz + std::cos(angle) * radius};
            };
            add_dashed_edge(output, {cx, 0.0, cz}, radial(0.0, 0.75), 3,
                            BodyKind::tractor, style, GuideLayer::articulation);
            if (std::abs(current_angle) > 0.02)
                add_dashed_edge(output, {cx, 0.0, cz}, radial(current_angle, 0.75), 3,
                                BodyKind::tractor, style, GuideLayer::articulation);
            const auto arc_samples = std::clamp<std::size_t>(
                static_cast<std::size_t>(std::ceil(std::abs(current_angle) * 0.75 / 0.20)), 1, 12);
            for (std::size_t index = 0; index < arc_samples; ++index)
                add_guide_segment(output, radial(current_angle * index / arc_samples, 0.75),
                    radial(current_angle * (index + 1) / arc_samples, 0.75),
                    BodyKind::tractor, style, GuideLayer::articulation);
            // Short outer tick marks the forecast angle after 1 m of tractor travel.
            add_dashed_edge(output, radial(future_angle, 0.86), radial(future_angle, 1.11),
                            1, BodyKind::tractor, style, GuideLayer::articulation);
            // Critical jackknife boundary tick on the folding side when caution/critical.
            if (output.jackknife_risk != JackknifeRisk::none &&
                output.count + 1 <= output.segments.size())
            {
                const double sign = current_angle >= 0.0 ? 1.0 : -1.0;
                add_dashed_edge(output, radial(sign * crit_rad, 0.72),
                                radial(sign * crit_rad, 1.08),
                                1, BodyKind::tractor, GuideStyle::trailer, GuideLayer::articulation);
            }
            output.articulation_segments = output.count - before;
        }
    }

    // Straight trailer-heading reference corridor: extends straight behind the
    // trailer's current rear corners so the driver sees where the trailer is
    // currently aimed compared against the steered prediction curve.
    if (options.trailer_corridor && initial_trailer &&
        output.count + 12 <= output.segments.size())
    {
        const auto before = output.count;
        const double corridor_len = std::clamp(
            std::min(options.corridor_distance, options.distance), 2.0, 8.0);
        const auto rear_left = body_corner(
            *initial_trailer, -initial_trailer->width * 0.5, initial_trailer->length * 0.5);
        const auto rear_right = body_corner(
            *initial_trailer, initial_trailer->width * 0.5, initial_trailer->length * 0.5);
        const double dx = std::sin(initial_trailer->pose.heading);
        const double dz = std::cos(initial_trailer->pose.heading);
        const std::size_t spare = output.segments.size() - output.count;
        const std::size_t samples = std::min<std::size_t>(5, spare / 2);
        const double step = corridor_len / static_cast<double>(samples);
        for (std::size_t index = 1; index <= samples; ++index)
        {
            const double d = static_cast<double>(index) * step;
            const double half = 0.18;
            add_guide_segment(output,
                {rear_left.x + dx * (d - half), 0.0, rear_left.z + dz * (d - half)},
                {rear_left.x + dx * (d + half), 0.0, rear_left.z + dz * (d + half)},
                BodyKind::trailer, GuideStyle::steady, GuideLayer::corridor);
            add_guide_segment(output,
                {rear_right.x + dx * (d - half), 0.0, rear_right.z + dz * (d - half)},
                {rear_right.x + dx * (d + half), 0.0, rear_right.z + dz * (d + half)},
                BodyKind::trailer, GuideStyle::steady, GuideLayer::corridor);
        }
        output.corridor_segments = output.count - before;
    }
    return output;
}

// Forward pull-up preview during a latched reverse session: shows where the
// tractor nose swings forward and how the trailer rear/articulation straightens
// when pulling forward with the current measured steering angle.
inline GuidanceGeometry build_pullup_guidance(
    const Prediction &forward_prediction, const TractorSpec &tractor,
    const GuidanceOptions &options, const TrailerSpec *trailer = nullptr)
{
    GuidanceGeometry output;
    if (!options.enabled || forward_prediction.boxes.empty()) return output;
    struct Track
    {
        std::array<BoundaryPoint, maximum_guidance_track_points> points{};
        std::size_t count = 0;
    };
    std::array<Track, 2> tractor_front{};
    std::array<Track, 2> trailer_rear{};
    const GroundBox *initial_trailer = nullptr;
    const GroundBox *end_trailer = nullptr;
    const GroundBox *end_tractor = nullptr;
    const auto push_point = [&](Track &track, const BoundaryPoint &point) {
        if (track.count < track.points.size()) track.points[track.count++] = point;
        else output.truncated = true;
    };
    for (const auto &box : forward_prediction.boxes)
    {
        if (!std::isfinite(box.pose.x) || !std::isfinite(box.pose.z) ||
            !std::isfinite(box.pose.heading) || !std::isfinite(box.width) ||
            !std::isfinite(box.length) || box.width <= 0.0 || box.length <= 0.0)
            continue;
        if (box.kind == BodyKind::trailer && box.trailer_index != 0) continue;
        if (box.kind == BodyKind::tractor)
        {
            push_point(tractor_front[0], body_corner(box, -box.width * 0.5 - 0.18, -box.length * 0.5));
            push_point(tractor_front[1], body_corner(box, box.width * 0.5 + 0.18, -box.length * 0.5));
            end_tractor = &box;
        }
        else
        {
            const auto boundary = reverse_leading_boundary(box);
            push_point(trailer_rear[0], boundary.left);
            push_point(trailer_rear[1], boundary.right);
            if (!initial_trailer) initial_trailer = &box;
            end_trailer = &box;
        }
    }
    if (initial_trailer && end_trailer && end_tractor)
    {
        const double current_angle = normalize_angle(initial_trailer->pose.heading);
        const double future_angle = normalize_angle(end_trailer->pose.heading - end_tractor->pose.heading);
        output.articulation_degrees = current_angle * 180.0 / pi;
        output.predicted_articulation_degrees = future_angle * 180.0 / pi;
        const double axle_to_hitch = trailer
            ? trailer->axle_to_hitch
            : std::clamp(initial_trailer->length * 0.68, 4.5, 11.0);
        const double crit_rad = jackknife_critical_angle(
            tractor.wheelbase, tractor.hook_from_rear, axle_to_hitch);
        output.jackknife_critical_degrees = crit_rad * 180.0 / pi;
        const double change = std::abs(future_angle) - std::abs(current_angle);
        if (change < -1.0 * pi / 180.0) output.trend = ArticulationTrend::straightening;
        else if (change > 1.0 * pi / 180.0) output.trend = ArticulationTrend::folding;
        const double abs_current = std::abs(output.articulation_degrees);
        if (abs_current >= output.jackknife_critical_degrees * 0.88)
            output.jackknife_risk = JackknifeRisk::critical;
        else if (abs_current >= std::max(30.0, output.jackknife_critical_degrees * 0.70))
            output.jackknife_risk = JackknifeRisk::caution;
    }

    const auto emit_track = [&](const Track &track, BodyKind body, GuideStyle style, GuideLayer layer) {
        for (std::size_t index = 0; index + 1 < track.count; ++index)
            add_guide_segment(output, track.points[index], track.points[index + 1], body, style, layer);
    };
    emit_track(tractor_front[0], BodyKind::tractor, GuideStyle::tractor, GuideLayer::tractor_track);
    emit_track(tractor_front[1], BodyKind::tractor, GuideStyle::tractor, GuideLayer::tractor_track);
    const auto trailer_style = output.trend == ArticulationTrend::straightening
        ? GuideStyle::straightening
        : (output.trend == ArticulationTrend::folding ? GuideStyle::trailer : GuideStyle::preview);
    emit_track(trailer_rear[0], BodyKind::trailer, trailer_style, GuideLayer::trailer_track);
    emit_track(trailer_rear[1], BodyKind::trailer, trailer_style, GuideLayer::trailer_track);

    if (initial_trailer && end_trailer && end_tractor && options.articulation_indicator)
    {
        const auto before = output.count;
        const double current_angle = normalize_angle(initial_trailer->pose.heading);
        const double future_angle = normalize_angle(end_trailer->pose.heading - end_tractor->pose.heading);
        const auto style = output.trend == ArticulationTrend::straightening
            ? GuideStyle::straightening
            : (output.trend == ArticulationTrend::folding ? GuideStyle::trailer : GuideStyle::steady);
        const double cx = tractor.rear_axle_x - tractor.width * 0.5 - 1.1;
        const double cz = tractor.rear_axle_z + tractor.hook_from_rear;
        const auto radial = [&](double angle, double radius) -> BoundaryPoint {
            return {cx + std::sin(angle) * radius, 0.0, cz + std::cos(angle) * radius};
        };
        add_dashed_edge(output, {cx, 0.0, cz}, radial(0.0, 0.75), 3,
                        BodyKind::tractor, style, GuideLayer::articulation);
        if (std::abs(current_angle) > 0.02)
            add_dashed_edge(output, {cx, 0.0, cz}, radial(current_angle, 0.75), 3,
                            BodyKind::tractor, style, GuideLayer::articulation);
        const auto arc_samples = std::clamp<std::size_t>(
            static_cast<std::size_t>(std::ceil(std::abs(current_angle) * 0.75 / 0.20)), 1, 12);
        for (std::size_t index = 0; index < arc_samples; ++index)
            add_guide_segment(output, radial(current_angle * index / arc_samples, 0.75),
                radial(current_angle * (index + 1) / arc_samples, 0.75),
                BodyKind::tractor, style, GuideLayer::articulation);
        add_dashed_edge(output, radial(future_angle, 0.86), radial(future_angle, 1.11),
                        1, BodyKind::tractor, style, GuideLayer::articulation);
        output.articulation_segments = output.count - before;
    }

    if (options.trailer_corridor && initial_trailer &&
        output.count + 12 <= output.segments.size())
    {
        const auto before = output.count;
        const double corridor_len = std::clamp(
            std::min(options.corridor_distance, options.distance), 2.0, 8.0);
        const auto rear_left = body_corner(
            *initial_trailer, -initial_trailer->width * 0.5, initial_trailer->length * 0.5);
        const auto rear_right = body_corner(
            *initial_trailer, initial_trailer->width * 0.5, initial_trailer->length * 0.5);
        const double dx = std::sin(initial_trailer->pose.heading);
        const double dz = std::cos(initial_trailer->pose.heading);
        const std::size_t spare = output.segments.size() - output.count;
        const std::size_t samples = std::min<std::size_t>(5, spare / 2);
        const double step = corridor_len / static_cast<double>(samples);
        for (std::size_t index = 1; index <= samples; ++index)
        {
            const double d = static_cast<double>(index) * step;
            const double half = 0.18;
            add_guide_segment(output,
                {rear_left.x + dx * (d - half), 0.0, rear_left.z + dz * (d - half)},
                {rear_left.x + dx * (d + half), 0.0, rear_left.z + dz * (d + half)},
                BodyKind::trailer, GuideStyle::steady, GuideLayer::corridor);
            add_guide_segment(output,
                {rear_right.x + dx * (d - half), 0.0, rear_right.z + dz * (d - half)},
                {rear_right.x + dx * (d + half), 0.0, rear_right.z + dz * (d + half)},
                BodyKind::trailer, GuideStyle::steady, GuideLayer::corridor);
        }
        output.corridor_segments = output.count - before;
    }
    return output;
}
}
