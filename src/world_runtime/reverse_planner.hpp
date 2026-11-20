#pragma once
#include "reverse_kinematics.hpp"
#include <array>
#include <chrono>
#include <unordered_map>

namespace reverse_assist
{
// One passive semitrailer, planar no-slip motion. No obstacle/collision model.
struct ReverseRig
{
    double wheelbase = 3.8, hitch_rear = 0.0, hitch_right = 0.0;
    double trailer_length = 7.2, overhang = 3.0, width = 2.55;
};
struct ReverseState
{
    Pose tractor;
    double trailer_heading = 0.0;
    double steering = 0.0; // measured road-wheel radians, not normalized input
};
struct ReverseGoal { Pose rear; };
struct PlannerLimits
{
    // Measured road-wheel angle that may start a plan. Search commands stay
    // on the coarser search_steering grid so the discrete set does not grow.
    double maximum_steering = 50.0 * pi / 180.0;
    double search_steering = 30.0 * pi / 180.0;
    double steering_per_metre = 0.24;
    double maximum_articulation = 55.0 * pi / 180.0;
    double arrival_articulation = 20.0 * pi / 180.0;
    double position_tolerance = 0.60, heading_tolerance = 4.0 * pi / 180.0;
    double maximum_distance = 40.0, primitive_distance = 0.75;
    std::size_t maximum_expansions = 18000, maximum_nodes = 80000;
    // Allow one initial forward pull-up leg before switching to reverse when
    // pure reverse cannot reach the goal or requires excessive maneuvering.
    bool allow_pullup = true;
    double maximum_pullup_distance = 12.0;
    double gear_switch_penalty = 3.5;
};
enum class PlanStatus { idle, searching, ready, invalid, outside_envelope, exhausted, budget };
struct ReverseSample
{
    ReverseState state;
    double distance = 0.0;
    bool forward = false;
};

inline bool finite_pose(const Pose &p)
{
    return std::isfinite(p.x) && std::isfinite(p.z) && std::isfinite(p.heading) &&
        std::abs(p.x) <= 1e7 && std::abs(p.z) <= 1e7 && std::abs(p.heading) <= 100.0 * pi;
}
inline Pose reverse_rear(const ReverseState &state, const ReverseRig &rig)
{
    const auto hitch = tractor_hitch(state.tractor, rig.hitch_rear, rig.hitch_right);
    return trailer_axle_from_hitch(hitch, state.trailer_heading, rig.trailer_length + rig.overhang);
}
inline double articulation(const ReverseState &state)
{
    return std::abs(normalize_angle(state.trailer_heading - state.tractor.heading));
}
inline bool goal_reached(const ReverseState &state, const ReverseGoal &goal,
                         const ReverseRig &rig, const PlannerLimits &limits)
{
    const auto rear = reverse_rear(state, rig);
    return std::hypot(rear.x - goal.rear.x, rear.z - goal.rear.z) <= limits.position_tolerance &&
        std::abs(normalize_angle(rear.heading - goal.rear.heading)) <= limits.heading_tolerance &&
        articulation(state) <= limits.arrival_articulation;
}
// Bounded ramp, shared rolling constraint and off-axle hitch model with the
// current-steering predictor. Reject limits at every 5 cm, not just endpoints.
inline bool motion_primitive(ReverseState &state, const ReverseRig &rig, double requested,
                             double distance, bool forward, const PlannerLimits &limits)
{
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / 0.05)));
    const double ds = distance / steps;
    const double signed_ds = forward ? -ds : ds;
    for (int i = 0; i < steps; ++i)
    {
        const double next_steering = state.steering + std::clamp(requested - state.steering,
            -limits.steering_per_metre * ds, limits.steering_per_metre * ds);
        const auto previous_hitch = tractor_hitch(state.tractor, rig.hitch_rear, rig.hitch_right);
        state.tractor = tractor_motion_step(state.tractor,
            std::tan((state.steering + next_steering) * 0.5) / rig.wheelbase, signed_ds);
        const auto next_hitch = tractor_hitch(state.tractor, rig.hitch_rear, rig.hitch_right);
        state.trailer_heading = trailer_heading_from_hitch_displacement(state.trailer_heading,
            previous_hitch, next_hitch, rig.trailer_length);
        state.steering = next_steering;
        if (articulation(state) > limits.maximum_articulation) return false;
    }
    return true;
}
inline bool reverse_primitive(ReverseState &state, const ReverseRig &rig, double requested,
                              double distance, const PlannerLimits &limits)
{
    return motion_primitive(state, rig, requested, distance, false, limits);
}
inline Pose to_ground_frame(Pose point, const Pose &origin)
{
    const double x = point.x - origin.x, z = point.z - origin.z;
    return {x * std::cos(origin.heading) - z * std::sin(origin.heading),
            x * std::sin(origin.heading) + z * std::cos(origin.heading),
            normalize_angle(point.heading - origin.heading)};
}
inline Pose from_ground_frame(Pose point, const Pose &origin)
{
    return {origin.x + point.x * std::cos(origin.heading) + point.z * std::sin(origin.heading),
            origin.z - point.x * std::sin(origin.heading) + point.z * std::cos(origin.heading),
            normalize_angle(point.heading + origin.heading)};
}

// Incremental weighted Hybrid A* with optional single initial forward pull-up.
// Exhaustion means no path in this discretized envelope, never a proof of
// physical infeasibility. No work occurs in draw hooks.
class ReversePlanner
{
    struct Key
    {
        std::array<int, 6> v;
        bool operator==(const Key &other) const { return v == other.v; }
    };
    struct Hash
    {
        std::size_t operator()(const Key &k) const
        {
            std::size_t h = 0;
            for (int v : k.v) h ^= std::hash<int>{}(v) + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };
    struct Node
    {
        ReverseState state;
        double distance, pullup_distance, cost, request;
        std::size_t parent;
        bool forward;
        std::uint8_t switches;
    };
    struct Open
    {
        double score;
        std::size_t index;
        bool operator<(const Open &other) const { return score > other.score; }
    };
    std::vector<Node> nodes_;
    // Min-heap by score: Open::operator< treats a smaller score as greater.
    std::vector<Open> open_;
    std::unordered_map<Key, double, Hash> best_;
    std::vector<ReverseSample> route_;
    Pose origin_;
    ReverseRig rig_;
    ReverseGoal goal_;
    PlannerLimits limits_;
    PlanStatus status_ = PlanStatus::idle;
    std::size_t expansions_ = 0;
    std::size_t switch_index_ = 0;
    bool has_pullup_ = false;
    Key key(const ReverseState &s, bool forward, std::uint8_t switches) const
    {
        const auto angle_bin = [](double angle) {
            int value = static_cast<int>(std::lround(normalize_angle(angle) / (pi / 36.0)));
            return value == 36 ? -36 : value;
        };
        const int phase = (forward ? 1 : 0) + 2 * static_cast<int>(switches);
        return {{{static_cast<int>(std::lround(s.tractor.x / 0.4)),
                  static_cast<int>(std::lround(s.tractor.z / 0.4)), angle_bin(s.tractor.heading),
                  angle_bin(s.trailer_heading), static_cast<int>(std::lround(s.steering / 0.10)),
                  phase}}};
    }
    double heuristic(const ReverseState &s) const
    {
        const auto rear = reverse_rear(s, rig_);
        return std::hypot(rear.x - goal_.rear.x, rear.z - goal_.rear.z) +
            5.0 * std::abs(normalize_angle(rear.heading - goal_.rear.heading));
    }
    void finish(std::size_t index)
    {
        std::vector<std::size_t> chain;
        while (index != 0) { chain.push_back(index); index = nodes_[index].parent; }
        std::reverse(chain.begin(), chain.end());
        ReverseState state = nodes_.front().state;
        double distance = 0.0;
        const bool starts_forward = !chain.empty() && nodes_[chain.front()].forward;
        route_.push_back({state, distance, starts_forward});
        has_pullup_ = starts_forward;
        switch_index_ = 0;
        for (const auto child : chain)
        {
            const bool seg_forward = nodes_[child].forward;
            if (has_pullup_ && switch_index_ == 0 && !seg_forward)
                switch_index_ = route_.size() - 1;
            const int pieces = static_cast<int>(std::ceil(limits_.primitive_distance / 0.05));
            const double ds = limits_.primitive_distance / pieces;
            for (int part = 0; part < pieces; ++part)
            {
                motion_primitive(state, rig_, nodes_[child].request, ds, seg_forward, limits_);
                distance += ds;
                route_.push_back({state, distance, seg_forward});
            }
        }
        for (auto &sample : route_)
        {
            sample.state.tractor = from_ground_frame(sample.state.tractor, origin_);
            sample.state.trailer_heading = normalize_angle(sample.state.trailer_heading + origin_.heading);
        }
        status_ = PlanStatus::ready;
    }
public:
    PlanStatus status() const { return status_; }
    const auto &route() const { return route_; }
    std::size_t expansions() const { return expansions_; }
    bool has_pullup() const { return has_pullup_; }
    std::size_t switch_index() const { return switch_index_; }
    void clear()
    {
        nodes_.clear(); open_.clear(); best_.clear(); route_.clear();
        expansions_ = 0; switch_index_ = 0; has_pullup_ = false; status_ = PlanStatus::idle;
    }
    void start(ReverseState state, ReverseGoal goal, ReverseRig rig, PlannerLimits limits = {})
    {
        clear();
        const std::array values{rig.wheelbase, rig.hitch_rear, rig.hitch_right, rig.trailer_length,
            rig.overhang, rig.width, state.trailer_heading, state.steering, limits.maximum_steering,
            limits.search_steering, limits.steering_per_metre, limits.maximum_articulation, limits.arrival_articulation,
            limits.position_tolerance, limits.heading_tolerance, limits.maximum_distance, limits.primitive_distance,
            limits.maximum_pullup_distance, limits.gear_switch_penalty};
        if (!finite_pose(state.tractor) || !finite_pose(goal.rear) ||
            !std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); }) ||
            rig.wheelbase < 2.4 || rig.wheelbase > 6.8 || rig.trailer_length < 2.0 || rig.trailer_length > 16.0 ||
            std::abs(rig.hitch_rear) > 3.0 || std::abs(rig.hitch_right) > 1.5 ||
            std::abs(state.trailer_heading) > 100.0 * pi ||
            rig.overhang < 0.1 || rig.overhang > 6.0 || rig.width < 2.0 || rig.width > 3.0 ||
            limits.maximum_steering <= 0 || limits.maximum_steering > pi / 3 ||
            limits.search_steering <= 0 || limits.search_steering > limits.maximum_steering ||
            std::abs(state.steering) > limits.maximum_steering || limits.steering_per_metre <= 0 ||
            limits.maximum_articulation <= 0 || limits.maximum_articulation > pi / 3 ||
            limits.arrival_articulation < 0 || limits.arrival_articulation > limits.maximum_articulation ||
            limits.position_tolerance <= 0 || limits.heading_tolerance <= 0 || limits.heading_tolerance > pi ||
            limits.maximum_distance <= 0 || limits.maximum_distance > 60 ||
            limits.primitive_distance < 0.1 || limits.primitive_distance > 1.0 ||
            limits.maximum_pullup_distance < 0 || limits.maximum_pullup_distance > 25.0 ||
            limits.gear_switch_penalty < 0 ||
            limits.maximum_expansions > 100000 || limits.maximum_nodes > 200000 ||
            articulation(state) > limits.maximum_articulation)
        { status_ = PlanStatus::invalid; return; }
        rig_ = rig; limits_ = limits; origin_ = state.tractor;
        state.tractor = to_ground_frame(state.tractor, origin_);
        state.trailer_heading = normalize_angle(state.trailer_heading - origin_.heading);
        goal.rear = to_ground_frame(goal.rear, origin_); goal_ = goal;
        if (heuristic(state) > limits.maximum_distance * 2.0 + rig.trailer_length)
        { status_ = PlanStatus::outside_envelope; return; }
        const auto reserve_count = std::min(limits.maximum_nodes,
            static_cast<std::size_t>(1) + limits.maximum_expansions * 5);
        nodes_.reserve(reserve_count);
        best_.reserve(reserve_count);
        open_.reserve(reserve_count);
        nodes_.push_back({state, 0.0, 0.0, 0.0, state.steering, 0, false, 0});
        best_.emplace(key(state, false, 0), 0.0);
        open_.push_back({heuristic(state), 0});
        std::push_heap(open_.begin(), open_.end());
        status_ = PlanStatus::searching;
    }
    // Both work count and wall clock bound each simulation-frame slice.
    void advance(std::size_t work = 48, std::chrono::microseconds slice = std::chrono::microseconds(1500))
    {
        const auto deadline = std::chrono::steady_clock::now() + slice;
        for (std::size_t tick = 0; status_ == PlanStatus::searching && tick < work; ++tick)
        {
            if (std::chrono::steady_clock::now() >= deadline) break;
            if (open_.empty()) { status_ = PlanStatus::exhausted; break; }
            if (expansions_ >= limits_.maximum_expansions || nodes_.size() >= limits_.maximum_nodes)
            { status_ = PlanStatus::budget; break; }
            std::pop_heap(open_.begin(), open_.end());
            const auto item = open_.back();
            open_.pop_back();
            const Node node = nodes_[item.index];
            if (node.cost > best_.at(key(node.state, node.forward, node.switches)) + 1e-6) continue;
            ++expansions_;
            if ((!node.forward || item.index == 0) && goal_reached(node.state, goal_, rig_, limits_))
            { finish(item.index); break; }
            if (node.distance + limits_.primitive_distance > limits_.maximum_distance) continue;

            const auto try_expand = [&](double requested, bool next_forward,
                                        std::uint8_t next_switches, double extra_cost) -> bool {
                auto next = node.state;
                if (!motion_primitive(next, rig_, requested, limits_.primitive_distance,
                                      next_forward, limits_))
                    return true;
                const double step_weight = next_forward ? 1.15 : 1.0;
                const double cost = node.cost + step_weight * limits_.primitive_distance + extra_cost +
                    0.15 * std::abs(next.steering) + 0.4 * std::abs(next.steering - node.state.steering);
                const auto k = key(next, next_forward, next_switches);
                const auto found = best_.find(k);
                if (found != best_.end() && found->second <= cost + 1e-6) return true;
                if (nodes_.size() >= limits_.maximum_nodes) { status_ = PlanStatus::budget; return false; }
                best_[k] = cost;
                const auto index = nodes_.size();
                const double next_pullup = node.pullup_distance +
                    (next_forward ? limits_.primitive_distance : 0.0);
                nodes_.push_back({next, node.distance + limits_.primitive_distance, next_pullup,
                                  cost, requested, item.index, next_forward, next_switches});
                const double h_bias = next_forward ? limits_.gear_switch_penalty : 0.0;
                open_.push_back({cost + 2.5 * heuristic(next) + h_bias, index});
                std::push_heap(open_.begin(), open_.end());
                return true;
            };

            // Reverse transitions (either continuing reverse or switching from pull-up to reverse).
            const bool switching_to_reverse = node.forward;
            const std::uint8_t rev_switches = switching_to_reverse ? 1 : node.switches;
            const double switch_cost = switching_to_reverse ? limits_.gear_switch_penalty : 0.0;
            bool ok = true;
            for (int steer = -2; steer <= 2 && ok; ++steer)
                ok = try_expand(limits_.search_steering * steer / 2.0, false, rev_switches, switch_cost);
            if (!ok) break;

            // Optional single initial forward pull-up leg to realign before reversing.
            if (limits_.allow_pullup &&
                (item.index == 0 || (node.forward && node.switches == 0)) &&
                node.pullup_distance + limits_.primitive_distance <= limits_.maximum_pullup_distance)
            {
                for (int steer : {-2, 0, 2})
                {
                    if (!try_expand(limits_.search_steering * steer / 2.0, true, 0, 0.0))
                        break;
                }
            }
        }
    }
};
}
