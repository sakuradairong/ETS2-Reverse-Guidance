#pragma once
#include "reverse_planner.hpp"
#include "reverse_guidance.hpp"
#include "chinese_cues.hpp"
#include <string_view>

namespace reverse_assist
{
enum class TargetCue { off, edit, wait, left, right, hold, straight, stop, arrived, limit, move, data, slow };
inline const char *cue_name(TargetCue cue)
{
    constexpr std::array names{"OFF", "EDIT", "WAIT", "LEFT", "RIGHT", "HOLD", "ZERO",
        "STOP", "DONE", "LIMIT", "MOVE", "DATA", "SLOW"};
    return names[static_cast<std::size_t>(cue)];
}
struct TargetInput
{
    bool toggle = false, confirm = false, edit = false, clear = false;
    double right = 0, rearward = 0, rotation = 0;
};
// Reject estimator fallback/clamping and rear-steered tractors for planning.
// Ordinary short-horizon prediction retains its existing permissive policy.
inline bool target_wheels_supported(const std::vector<WheelSpec> &truck, const std::vector<WheelSpec> &trailer,
                                    double truck_hook_x, double truck_hook_z, double trailer_hook_x, double trailer_hook_z)
{
    if (truck.size() < 4 || trailer.size() < 2 || !std::isfinite(truck_hook_x) ||
        !std::isfinite(truck_hook_z) || !std::isfinite(trailer_hook_x) || !std::isfinite(trailer_hook_z) ||
        std::abs(trailer_hook_x) > 0.05) return false;
    const auto known = [](const WheelSpec &w) {
        return w.on_ground_known && w.steerable_known && std::isfinite(w.x) && std::isfinite(w.z);
    };
    if (!std::all_of(truck.begin(), truck.end(), known) || !std::all_of(trailer.begin(), trailer.end(), known)) return false;
    double first_fixed = std::numeric_limits<double>::infinity();
    for (const auto &w : truck) if (w.on_ground && !w.steerable) first_fixed = std::min(first_fixed, w.z);
    double front_z = 0, rear_z = 0, rear_x = 0;
    int fronts = 0, rears = 0;
    for (const auto &w : truck)
    {
        if (!w.on_ground) continue;
        if (w.steerable)
        {
            if (w.z >= first_fixed - 0.20 || !w.steering_angle_known || !std::isfinite(w.steering_angle)) return false;
            front_z += w.z; ++fronts;
        }
        else { rear_z += w.z; rear_x += w.x; ++rears; }
    }
    if (fronts < 2 || rears < 2) return false;
    front_z /= fronts; rear_z /= rears; rear_x /= rears;
    if (rear_z-front_z < 2.4 || rear_z-front_z > 6.8 || truck_hook_z-rear_z < -1.5 ||
        truck_hook_z-rear_z > 2.5 || std::abs(truck_hook_x-rear_x) > 1.5) return false;
    double axle_z = 0, max_z = -std::numeric_limits<double>::infinity();
    int axles = 0;
    for (const auto &w : trailer)
    {
        if (w.steerable || (w.steering_angle_known && (!std::isfinite(w.steering_angle) || std::abs(w.steering_angle)>0.01))) return false;
        max_z = std::max(max_z, w.z);
        if (w.on_ground) { axle_z += w.z; ++axles; }
    }
    if (axles < 2) return false;
    axle_z /= axles;
    return axle_z-trailer_hook_z >= 2.5 && axle_z-trailer_hook_z <= 15.0 && max_z-axle_z+0.95 <= 4.0;
}
// Closed-loop steering feedback around the planned reference trajectory.
// Reduces frequent stop-and-replan interruptions by feeding lateral, trailer-yaw
// and articulation errors back into the lookahead steering command.
inline double compute_closed_loop_steering(const ReverseState &state,
                                           const ReverseSample &nearest_sample,
                                           const ReverseSample &lookahead_sample,
                                           const ReverseRig &rig,
                                           const PlannerLimits &limits = {})
{
    double feedback = 0.0;
    const auto apply_deadband = [](double value, double band) {
        if (std::abs(value) <= band) return 0.0;
        return value > 0.0 ? value - band : value + band;
    };
    if (!nearest_sample.forward)
    {
        const auto ref_rear = reverse_rear(nearest_sample.state, rig);
        const auto act_rear = reverse_rear(state, rig);
        const auto err = to_ground_frame(act_rear, ref_rear);
        const double gamma = normalize_angle(state.trailer_heading - state.tractor.heading);
        const double gamma_ref = normalize_angle(
            nearest_sample.state.trailer_heading - nearest_sample.state.tractor.heading);
        // Feedback-linearized cascade with small deadband for steady straight cues:
        // 1) rear-bumper lateral error sets desired trailer heading;
        // 2) trailer heading error sets desired articulation offset;
        // 3) cancel the open-loop unstable +sin(gamma)/(L2 + h*cos(gamma)) pole while
        //    tracking gamma_des so the driver is prompted to catch the swing in time.
        const double lat_err = apply_deadband(std::clamp(err.x, -2.0, 2.0), 0.03);
        const double desired_trailer_offset = -std::atan(0.22 * lat_err);
        const double trailer_err = normalize_angle(err.heading - desired_trailer_offset);
        const double gamma_des = std::clamp(gamma_ref - 1.50 * trailer_err, -0.45, 0.45);
        const double denom = std::max(1.0, rig.trailer_length + rig.hitch_rear * std::cos(gamma));
        const double k_cancel = -(std::sin(gamma) - std::sin(gamma_ref)) / denom;
        const double k_track = -0.35 * normalize_angle(gamma - gamma_des);
        feedback = apply_deadband(std::atan(rig.wheelbase * (k_cancel + k_track)), 0.4 * pi / 180.0);
    }
    else
    {
        const auto tr_err = to_ground_frame(state.tractor, nearest_sample.state.tractor);
        const double lat_err = apply_deadband(std::clamp(tr_err.x, -1.5, 1.5), 0.05);
        const double yaw_err = apply_deadband(tr_err.heading, 1.0 * pi / 180.0);
        feedback = std::clamp(0.75 * std::atan(0.60 * lat_err) - 0.85 * yaw_err, -0.32, 0.32);
    }
    return std::clamp(lookahead_sample.state.steering + feedback,
                      -limits.search_steering, limits.search_steering);
}

class ReverseTarget
{
    bool active_ = false, editing_ = false, requested_ = false, allow_pullup_ = true;
    ReverseGoal goal_;
    ReversePlanner planner_;
    ReverseState start_;
    std::size_t cursor_ = 0;
    std::uint64_t last_replan_ = 0;
    TargetCue cue_ = TargetCue::off;
    double desired_steering_ = 0.0;
    std::array<double, 6> geometry_{};
public:
    bool active() const { return active_; }
    bool editing() const { return editing_; }
    bool allow_pullup() const { return allow_pullup_; }
    void set_allow_pullup(bool enabled) { allow_pullup_ = enabled; }
    const ReverseGoal &goal() const { return goal_; }
    const ReversePlanner &planner() const { return planner_; }
    std::size_t cursor() const { return cursor_; }
    TargetCue cue() const { return cue_; }
    double desired_steering() const { return desired_steering_; }
    void reset()
    {
        active_ = editing_ = requested_ = false;
        planner_.clear(); cursor_ = 0; last_replan_ = 0; cue_ = TargetCue::off;
        desired_steering_ = 0.0;
    }
    // Backward-compatible bool overload maps reverse_gear into signed gear.
    void update(const ReverseState &state, const ReverseRig &rig, double speed,
                bool reverse_gear, bool supported, std::uint64_t now, const TargetInput &input)
    {
        update(state, rig, speed, reverse_gear ? -1 : 1, supported, now, input);
    }
    // Primary overload accepting real signed gear (<0 reverse, 0 neutral, >0 forward).
    void update(const ReverseState &state, const ReverseRig &rig, double speed,
                int gear, bool supported, std::uint64_t now, const TargetInput &input)
    {
        if (input.clear) { reset(); return; }
        const bool reverse_gear = gear < 0;
        const bool forward_gear = gear > 0;
        const bool stopped = std::isfinite(speed) && std::abs(speed) < 0.05;
        PlannerLimits limits{};
        limits.allow_forward_pullup = allow_pullup_;
        const bool state_valid = finite_pose(state.tractor) && std::isfinite(state.trailer_heading) &&
            std::abs(state.trailer_heading) < 100*pi && std::isfinite(state.steering) &&
            std::abs(state.steering) <= limits.maximum_steering &&
            articulation(state) <= limits.maximum_articulation;
        const std::array geometry{rig.wheelbase, rig.hitch_rear, rig.hitch_right,
            rig.trailer_length, rig.overhang, rig.width};
        if (active_)
            for (std::size_t i = 0; i < geometry.size(); ++i)
                if (!std::isfinite(geometry[i]) || std::abs(geometry[i] - geometry_[i]) > 0.02)
                { reset(); return; }
        if (input.toggle && !active_ && (!supported || !state_valid || !std::isfinite(speed)))
        { cue_ = TargetCue::data; return; }
        if (input.toggle && !active_ && supported && state_valid && std::isfinite(speed) && !stopped)
        { cue_ = TargetCue::stop; return; }
        if (input.toggle && stopped && supported && state_valid && std::isfinite(speed))
        {
            if (active_) { reset(); return; }
            active_ = editing_ = true;
            geometry_ = geometry;
            goal_.rear = reverse_rear(state, rig);
            goal_.rear.x += std::sin(goal_.rear.heading) * 8.0;
            goal_.rear.z += std::cos(goal_.rear.heading) * 8.0;
        }
        if (!active_)
        {
            if (cue_ == TargetCue::data && supported && state_valid && std::isfinite(speed))
                cue_ = TargetCue::off;
            else if (cue_ == TargetCue::stop && stopped)
                cue_ = TargetCue::off;
            return;
        }
        if (!supported || !state_valid || !std::isfinite(speed))
        {
            planner_.clear(); requested_ = !editing_; cue_ = TargetCue::data; return;
        }
        if (input.edit && stopped)
        { editing_ = true; requested_ = false; planner_.clear(); }
        if (editing_)
        {
            cue_ = stopped ? TargetCue::edit : TargetCue::stop;
            if (!stopped) return;
            goal_.rear.x += std::cos(state.tractor.heading) * input.right + std::sin(state.tractor.heading) * input.rearward;
            goal_.rear.z += -std::sin(state.tractor.heading) * input.right + std::cos(state.tractor.heading) * input.rearward;
            goal_.rear.heading = normalize_angle(goal_.rear.heading + input.rotation);
            const auto current_rear = reverse_rear(state, rig);
            if ((input.right != 0 || input.rearward != 0) &&
                std::hypot(goal_.rear.x-current_rear.x, goal_.rear.z-current_rear.z) > 60.0)
            {
                const double angle = std::atan2(goal_.rear.x-current_rear.x, goal_.rear.z-current_rear.z);
                goal_.rear.x = current_rear.x + std::sin(angle)*60.0;
                goal_.rear.z = current_rear.z + std::cos(angle)*60.0;
            }
            if (!input.confirm) return;
            editing_ = false; requested_ = true;
        }
        // Pose tolerance alone means arrival. Leaving reverse after stopping
        // must not clear the route or replace the green box with a stop cue.
        if (goal_reached(state, goal_, rig, limits))
        {
            if (stopped)
            {
                if (planner_.status() == PlanStatus::searching) planner_.clear();
                cue_ = TargetCue::arrived;
                return;
            }
            cue_ = std::abs(speed) * 3.6 > 5.0 ? TargetCue::slow : TargetCue::stop;
            return;
        }

        const bool plan_ready = planner_.status() == PlanStatus::ready &&
            cursor_ < planner_.route().size();
        const bool on_pullup = plan_ready && planner_.route()[cursor_].forward;
        const double speed_kmh = std::abs(speed) * 3.6;
        // Forward pull-up legs allow up to 8 km/h; reverse legs stay capped at 5 km/h.
        const double max_leg_speed_kmh = on_pullup ? 8.0 : 5.0;
        if (speed_kmh > max_leg_speed_kmh)
        { planner_.clear(); requested_ = true; cue_ = TargetCue::slow; return; }

        // Locate the next gear-change point on or ahead of cursor_.
        std::size_t next_switch = 0;
        if (plan_ready)
        {
            for (const auto idx : planner_.gear_change_indices())
            {
                if (idx >= cursor_) { next_switch = idx; break; }
            }
        }
        // At a planned gear-change point, prompt STOP without clearing the route
        // while the driver shifts through neutral into the next leg's gear.
        if (plan_ready && next_switch > 0 && next_switch < planner_.route().size())
        {
            const auto &switch_pose = planner_.route()[next_switch].state;
            const bool at_switch =
                std::hypot(switch_pose.tractor.x - state.tractor.x,
                           switch_pose.tractor.z - state.tractor.z) <= 0.40 &&
                std::abs(normalize_angle(switch_pose.tractor.heading - state.tractor.heading)) <= 8.0 * pi / 180.0;
            if (at_switch && next_switch + 1 < planner_.route().size())
            {
                const bool next_forward = planner_.route()[next_switch + 1].forward;
                const bool next_gear_ok = next_forward ? forward_gear : reverse_gear;
                if (stopped && next_gear_ok)
                {
                    cursor_ = next_switch + 1;
                }
                else
                {
                    cursor_ = next_switch;
                    cue_ = TargetCue::stop;
                    return;
                }
            }
        }

        const bool active_pullup = planner_.status() == PlanStatus::ready &&
            cursor_ < planner_.route().size() && planner_.route()[cursor_].forward;
        if (!active_pullup && (!reverse_gear || speed > 0.10))
        { planner_.clear(); requested_ = true; cue_ = TargetCue::stop; return; }

        if (planner_.status() == PlanStatus::searching)
        {
            // |speed| < 0.05 m/s still counts as stopped, so the position
            // allowance grows with the search. A deliberate yaw or wheel
            // change remains outside the fixed angular allowances.
            const double elapsed = last_replan_ && now >= last_replan_
                ? static_cast<double>(now - last_replan_) / 1000.0 : 0.0;
            const double position_allowance = 0.30 + 0.05 * elapsed;
            const bool moved = !stopped ||
                std::hypot(state.tractor.x - start_.tractor.x, state.tractor.z - start_.tractor.z) > position_allowance ||
                std::abs(normalize_angle(state.tractor.heading - start_.tractor.heading)) > 4.0 * pi / 180.0 ||
                std::abs(normalize_angle(state.trailer_heading - start_.trailer_heading)) > 4.0 * pi / 180.0 ||
                std::abs(state.steering - start_.steering) > 8.0 * pi / 180.0;
            if (moved)
            { planner_.clear(); requested_ = true; cue_ = TargetCue::stop; return; }
            planner_.advance();
        }
        if (requested_)
        {
            cue_ = TargetCue::stop;
            if (!stopped || (last_replan_ && now - last_replan_ < 1200)) return;
            planner_.start(state, goal_, rig, limits); start_ = state;
            last_replan_ = now; cursor_ = 0; requested_ = false;
        }
        switch (planner_.status())
        {
        case PlanStatus::searching: cue_ = TargetCue::wait; return;
        case PlanStatus::budget: cue_ = TargetCue::limit; return;
        case PlanStatus::outside_envelope:
        case PlanStatus::exhausted: cue_ = TargetCue::move; return;
        case PlanStatus::invalid: cue_ = TargetCue::data; return;
        case PlanStatus::ready: break;
        default: cue_ = TargetCue::edit; return;
        }
        const auto &route = planner_.route();
        // Refresh next gear-change boundary after potential switch advance.
        next_switch = 0;
        for (const auto idx : planner_.gear_change_indices())
        {
            if (idx >= cursor_) { next_switch = idx; break; }
        }
        const bool tracking_pullup = cursor_ < route.size() && route[cursor_].forward;
        if (tracking_pullup)
        {
            if (speed < -0.10)
            { planner_.clear(); requested_ = true; cue_ = TargetCue::stop; return; }
            if (!forward_gear)
            {
                // Prompt driver to shift into forward gear and pull up along the blue leg.
                cue_ = TargetCue::move;
                return;
            }
        }
        const std::size_t segment_limit = next_switch > 0 && next_switch < route.size()
            ? next_switch + 1 : route.size();
        double closest_position = std::numeric_limits<double>::infinity();
        double closest_tractor_heading = std::numeric_limits<double>::infinity();
        double closest_trailer_heading = std::numeric_limits<double>::infinity();
        double closest_score = std::numeric_limits<double>::infinity();
        // Restrict progress to the next 3m within the active gear leg.
        std::size_t nearest = cursor_;
        for (std::size_t i = cursor_; i < segment_limit && route[i].distance <= route[cursor_].distance + 3.0; ++i)
        {
            const auto &p = route[i].state;
            const double position = std::hypot(p.tractor.x - state.tractor.x, p.tractor.z - state.tractor.z);
            const double tractor_heading = std::abs(normalize_angle(p.tractor.heading - state.tractor.heading));
            const double trailer_heading = std::abs(normalize_angle(p.trailer_heading - state.trailer_heading));
            const double score = position + tractor_heading + trailer_heading;
            if (score < closest_score)
            {
                closest_score = score;
                closest_position = position;
                closest_tractor_heading = tractor_heading;
                closest_trailer_heading = trailer_heading;
                nearest = i;
            }
        }
        cursor_ = nearest;
        constexpr double tractor_heading_allowance = 26.0 * pi / 180.0;
        constexpr double trailer_heading_allowance = 18.0 * pi / 180.0;
        if (next_switch > 0 && cursor_ >= next_switch)
        {
            cue_ = TargetCue::stop;
            return;
        }
        if (closest_position > 1.5 || closest_tractor_heading > tractor_heading_allowance ||
            closest_trailer_heading > trailer_heading_allowance || cursor_ + 1 == route.size())
        { planner_.clear(); requested_ = true; cue_ = TargetCue::stop; return; }
        // About two metres of lead gives the driver time to follow the wheel cue.
        constexpr double steering_lookahead = 2.0;
        auto ahead = cursor_;
        for (std::size_t i = cursor_; i < segment_limit; ++i)
        {
            ahead = i;
            if (route[i].distance >= route[cursor_].distance + steering_lookahead) break;
        }
        desired_steering_ = compute_closed_loop_steering(
            state, route[cursor_], route[ahead], rig, limits);
        const double difference = desired_steering_ - state.steering;
        if (std::abs(difference) > 3.0 * pi / 180.0)
            cue_ = difference > 0 ? TargetCue::left : TargetCue::right;
        else cue_ = std::abs(desired_steering_) < 3.0 * pi / 180.0 ? TargetCue::straight : TargetCue::hold;
    }
};

// One baked Chinese text model per label. Endpoints encode only placement and
// heading; mesh dimensions are fixed at preparation time, with no scaling hook.
inline void target_label(GuidanceGeometry &out, TargetCue cue, double x, double z,
                         double rotation = 0.0)
{
    const auto index = static_cast<std::size_t>(cue);
    if (index >= chinese_cue_text.size()) return;
    const double dx = 0.18 * std::sin(rotation), dz = 0.18 * std::cos(rotation);
    add_guide_segment(out, {x-dx,0,z-dz}, {x+dx,0,z+dz}, BodyKind::tractor,
        static_cast<GuideStyle>(static_cast<std::size_t>(GuideStyle::cue_off)+index),
        GuideLayer::cue);
}
static_assert(static_cast<std::size_t>(GuideStyle::count) -
    static_cast<std::size_t>(GuideStyle::cue_off) == chinese_cue_text.size());
static_assert(static_cast<std::size_t>(TargetCue::slow)+1 == chinese_cue_text.size());

inline void place_side_cue(GuidanceGeometry &out, TargetCue cue, double tractor_width)
{
    const auto index = static_cast<std::size_t>(cue);
    if (index >= chinese_cue_width.size()) return;
    const double label_x = tractor_width * 0.5 + 0.65 + chinese_cue_width[index] * 0.5;
    target_label(out, cue, label_x, -2.0, pi);
    target_label(out, cue, -label_x, -2.0);
}

inline GuidanceGeometry target_geometry(const ReverseTarget &target, const ReverseRig &rig, const Pose &truck_origin)
{
    GuidanceGeometry output;
    const auto cue = target.cue();
    const auto place_labels = [&]() {
        place_side_cue(output, cue, rig.width);
    };
    // A rejected toggle has no goal yet, but the driver still needs the cue.
    if (!target.active())
    {
        if (cue == TargetCue::data || cue == TargetCue::stop)
            place_labels();
        return output;
    }
    const bool ready = target.planner().status() == PlanStatus::ready;
    const auto style = cue == TargetCue::arrived ? GuideStyle::straightening : GuideStyle::target;
    // Rear-centre + heading defines the whole estimated trailer outline.
    Pose rear = to_ground_frame(target.goal().rear, truck_origin);
    const auto at = [&](double lateral, double longitudinal) {
        return BoundaryPoint{rear.x + std::cos(rear.heading)*lateral + std::sin(rear.heading)*longitudinal, 0,
            rear.z - std::sin(rear.heading)*lateral + std::cos(rear.heading)*longitudinal};
    };
    const double half = rig.width * 0.5, length = rig.trailer_length + rig.overhang;
    // Dense L brackets establish the four corners; sparse rails distinguish
    // the selected bay from the constant-steering endpoint outline.
    for (const double side : {-1.0, 1.0})
        for (const double end : {0.0, -length})
        {
            const double inward = end == 0 ? -1.0 : 1.0;
            add_dashed_edge(output, at(side*half,end), at(side*(half-0.85),end), 2,
                            BodyKind::tractor, style, GuideLayer::target_box);
            add_dashed_edge(output, at(side*half,end), at(side*half,end+inward*0.85), 2,
                            BodyKind::tractor, style, GuideLayer::target_box);
        }
    add_dashed_edge(output, at(-half,-0.85), at(-half,-length+0.85), 5,
                    BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(half,-0.85), at(half,-length+0.85), 5,
                    BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(-half+0.85,0), at(half-0.85,0), 2,
                    BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(-half+0.85,-length), at(half-0.85,-length), 2,
                    BodyKind::tractor, style, GuideLayer::target_box);
    // Rear-centre cross and an external arrow make the target anchor and
    // tail direction explicit even when the long sides are occluded.
    add_dashed_edge(output, at(-0.36,0), at(0.36,0), 2, BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(0,-0.36), at(0,0.36), 2, BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(0,0.55), at(0,1.65), 3, BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(-0.55,1.1), at(0,1.65), 2, BodyKind::tractor, style, GuideLayer::target_box);
    add_dashed_edge(output, at(0.55,1.1), at(0,1.65), 2, BodyKind::tractor, style, GuideLayer::target_box);
    place_labels();
    if (ready)
    {
        const auto &route = target.planner().route();
        const auto begin = target.cursor();
        // Leave at least 96 segments to the actual-steering forecast.
        constexpr std::size_t target_budget = maximum_guidance_segments - 96;
        std::size_t switch_markers = 0;
        for (const auto sw_idx : target.planner().gear_change_indices())
            if (sw_idx >= begin && sw_idx < route.size()) ++switch_markers;
        const std::size_t marker_reserve = switch_markers * 2;
        const std::size_t available = target_budget > output.count + marker_reserve
            ? target_budget - output.count - marker_reserve : 0;
        const auto route_budget = std::min<std::size_t>(36, available);
        for (std::size_t i = 0; i < route_budget && begin+1 < route.size(); ++i)
        {
            const auto first = begin + i*(route.size()-1-begin)/route_budget;
            const auto last = begin + (i+1)*(route.size()-1-begin)/route_budget;
            if (last <= first) continue;
            const bool is_pullup_seg = route[first].forward;
            const auto a = to_ground_frame(
                is_pullup_seg ? route[first].state.tractor : reverse_rear(route[first].state, rig),
                truck_origin);
            const auto b = to_ground_frame(
                is_pullup_seg ? route[last].state.tractor : reverse_rear(route[last].state, rig),
                truck_origin);
            add_dashed_edge(output, {a.x,0,a.z}, {b.x,0,b.z}, 1, BodyKind::tractor,
                            is_pullup_seg ? GuideStyle::tractor : GuideStyle::target,
                            GuideLayer::target_route);
        }
        for (const auto sw_idx : target.planner().gear_change_indices())
        {
            if (sw_idx < begin || sw_idx >= route.size()) continue;
            const auto sw = to_ground_frame(route[sw_idx].state.tractor, truck_origin);
            add_dashed_edge(output, {sw.x - 0.35, 0, sw.z}, {sw.x + 0.35, 0, sw.z}, 1,
                            BodyKind::tractor, GuideStyle::preview, GuideLayer::target_route);
            add_dashed_edge(output, {sw.x, 0, sw.z - 0.35}, {sw.x, 0, sw.z + 0.35}, 1,
                            BodyKind::tractor, GuideStyle::preview, GuideLayer::target_route);
        }
    }
    return output;
}

// Merge current-steering guidance into the remaining 256-segment budget.
// When multiple semantic layers exist and do not all fit, preserve whole
// compact indicators (articulation, corridor, endpoint) by priority instead
// of breaking them into isolated fragments via flat striding.
inline void append_current_guidance(GuidanceGeometry &display, const GuidanceGeometry &current)
{
    if (current.count == 0 || display.count >= display.segments.size()) return;
    const std::size_t budget = display.segments.size() - display.count;
    if (current.count <= budget)
    {
        for (std::size_t i = 0; i < current.count; ++i)
            display.segments[display.count++] = current.segments[i];
        return;
    }

    const auto first_layer = current.segments[0].layer;
    bool multi_layer = false;
    for (std::size_t i = 1; i < current.count; ++i)
    {
        if (current.segments[i].layer != first_layer)
        {
            multi_layer = true;
            break;
        }
    }
    if (!multi_layer)
    {
        for (std::size_t i = 0; i < budget; ++i)
        {
            const auto index = budget > 1 ? i * (current.count - 1) / (budget - 1) : 0;
            display.segments[display.count++] = current.segments[index];
        }
        return;
    }

    auto remaining = [&]() -> std::size_t {
        return display.segments.size() - display.count;
    };
    const auto collect_indices = [&](GuideLayer layer) {
        std::array<std::size_t, maximum_guidance_segments> indices{};
        std::size_t n = 0;
        for (std::size_t i = 0; i < current.count; ++i)
            if (current.segments[i].layer == layer) indices[n++] = i;
        return std::pair{indices, n};
    };
    const auto copy_whole = [&](GuideLayer layer) {
        const auto [indices, n] = collect_indices(layer);
        if (n == 0 || n > remaining()) return;
        for (std::size_t i = 0; i < n; ++i)
            display.segments[display.count++] = current.segments[indices[i]];
    };
    const auto copy_thinned = [&](GuideLayer layer, std::size_t cap) {
        const auto [indices, n] = collect_indices(layer);
        const std::size_t take = std::min({n, cap, remaining()});
        if (take == 0) return;
        for (std::size_t i = 0; i < take; ++i)
        {
            const auto pick = take > 1 ? i * (n - 1) / (take - 1) : 0;
            display.segments[display.count++] = current.segments[indices[pick]];
        }
    };

    // Priority 1: compact whole layers (never shred into broken dots).
    copy_whole(GuideLayer::cue);
    copy_whole(GuideLayer::articulation);
    copy_whole(GuideLayer::corridor);

    // Priority 2: trailer rear tracks (primary reversing reference).
    const auto trailer_n = collect_indices(GuideLayer::trailer_track).second;
    const auto tractor_n = collect_indices(GuideLayer::tractor_track).second;
    const auto rem_tracks = remaining();
    const std::size_t trailer_cap = tractor_n > 0
        ? std::max<std::size_t>(std::min(trailer_n, rem_tracks * 2 / 3),
                                std::min<std::size_t>(trailer_n, 24))
        : rem_tracks;
    copy_thinned(GuideLayer::trailer_track, trailer_cap);

    // Priority 3: whole endpoint outline if it fits cleanly alongside tractor track.
    const auto endpoint_n = collect_indices(GuideLayer::endpoint).second;
    if (endpoint_n > 0 && endpoint_n + std::min<std::size_t>(tractor_n, 20) <= remaining())
        copy_whole(GuideLayer::endpoint);

    // Priority 4: tractor rear tracks.
    copy_thinned(GuideLayer::tractor_track, remaining());

    // Priority 5: endpoint fallback (if still fits whole) and front swing.
    if (endpoint_n > 0 && endpoint_n <= remaining())
    {
        bool already_copied = false;
        for (std::size_t i = 0; i < display.count; ++i)
            if (display.segments[i].layer == GuideLayer::endpoint) { already_copied = true; break; }
        if (!already_copied) copy_whole(GuideLayer::endpoint);
    }
    copy_thinned(GuideLayer::front_swing, remaining());
}
}
