#include "../world_runtime/reverse_target.hpp"
#include <iostream>
#include <cstdlib>
#include <fstream>

using namespace reverse_assist;
namespace
{
int checks = 0, failures = 0;
void check(bool condition, const char *name)
{
    ++checks;
    if (!condition) { ++failures; std::cerr << "FAIL: " << name << '\n'; }
}
void solve(ReversePlanner &planner)
{
    while (planner.status() == PlanStatus::searching)
        planner.advance(1000, std::chrono::milliseconds(100));
}
void export_display(std::ofstream &stream, const char *name, const ReverseTarget &target,
                     const ReverseRig &rig, const ReverseState &state, bool &first)
{
    if (!stream.is_open()) return;
    auto display = target_geometry(target, rig, state.tractor);
    const auto selected_count = display.count;
    TractorSpec tractor;
    tractor.wheelbase = rig.wheelbase; tractor.front_axle_steering = state.steering;
    const std::vector<TrailerSpec> trailers{{normalize_angle(state.trailer_heading-state.tractor.heading),
        rig.trailer_length, rig.overhang, rig.width}};
    const auto prediction = predict_reverse_boxes(0, rig.wheelbase, tractor.length, rig.width, trailers,
        0, 0, tractor.center_from_rear, 5, 0.05, 0.25, rig.hitch_rear, rig.hitch_right, state.steering);
    append_current_guidance(display, build_reverse_guidance(prediction, tractor, GuidanceOptions{}));
    if (!first) stream << ',';
    first = false;
    const auto rear = to_ground_frame(target.goal().rear, state.tractor);
    stream << "{\"name\":\"" << name << "\",\"target_segments\":" << selected_count
           << ",\"rear\":[" << rear.x << ',' << rear.z << ',' << rear.heading << "],\"segments\":[";
    for (std::size_t i=0; i<display.count; ++i)
    {
        const auto &s=display.segments[i];
        if (i) stream << ',';
        stream << '[' << s.from.x << ',' << s.from.z << ',' << s.to.x << ',' << s.to.z << ','
               << static_cast<int>(s.style) << ']';
    }
    stream << "]}";
}
void wheel_eligibility()
{
    std::vector<WheelSpec> truck_wheels, trailer_wheels;
    for (int i=0; i<4; ++i)
    {
        WheelSpec wheel;
        wheel.x = i%2 ? 1.0 : -1.0; wheel.z = i<2 ? -3.8 : 0;
        wheel.steerable = i<2; wheel.steerable_known = wheel.on_ground_known = true;
        wheel.steering_angle_known = true;
        truck_wheels.push_back(wheel);
        wheel.steerable = false; wheel.z = i<2 ? 7.0 : 8.0;
        trailer_wheels.push_back(wheel);
    }
    check(target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 0), "measured ordinary rig accepted");
    trailer_wheels[0].steerable = true;
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 0), "steered trailer rejected");
    trailer_wheels[0].steerable = false;
    truck_wheels[2].steerable = true;
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 0), "rear-steered tractor rejected");
    truck_wheels[2].steerable = false;
    truck_wheels[0].steering_angle_known = false;
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 0), "missing measured front steering rejected");
    truck_wheels[0].steering_angle_known = true;
    truck_wheels[2].on_ground = truck_wheels[3].on_ground = false;
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 0), "no grounded fixed rear axle rejected");
    truck_wheels[2].on_ground = truck_wheels[3].on_ground = true;
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, 5, 0, 0), "clamped tractor hitch would be rejected");
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 1, 0), "offset trailer hitch unsupported");
    check(!target_wheels_supported(truck_wheels, trailer_wheels, 0, -0.6, 0, 7), "clamped trailer length would be rejected");
}
void replay(const ReversePlanner &planner, ReverseState initial, ReverseGoal goal, const ReverseRig &rig)
{
    check(planner.status() == PlanStatus::ready, "reachable generated goal");
    if (planner.route().empty()) return;
    const PlannerLimits limits;
    double last_distance = 0;
    for (const auto &sample : planner.route())
    {
        const double ds = sample.distance - last_distance;
        check(ds >= 0 && ds <= 0.051, "route sampling is bounded and reverse-only");
        if (ds > 0)
        {
            const double prior_steering = initial.steering;
            check(reverse_primitive(initial, rig, sample.state.steering, ds, limits), "replay respects articulation");
            check(std::abs(sample.state.steering - prior_steering) <= limits.steering_per_metre * ds + 1e-7,
                "road-wheel steering slew bound");
        }
        check(std::abs(initial.steering) <= limits.maximum_steering + 1e-8, "road-wheel angle bound");
        check(std::hypot(initial.tractor.x - sample.state.tractor.x,
                         initial.tractor.z - sample.state.tractor.z) < 0.002, "independent command replay position");
        check(std::abs(normalize_angle(initial.trailer_heading - sample.state.trailer_heading)) < 0.001,
            "independent command replay heading");
        last_distance = sample.distance;
    }
    check(goal_reached(initial, goal, rig, limits), "replayed rear position AND heading meet goal");
    check(last_distance <= limits.maximum_distance + 1e-6, "bounded route distance");
}
}
int main(int argc, char **argv)
{
    std::ofstream display_export;
    bool first_export = true;
    if (argc == 2)
    {
        display_export.open(argv[1]);
        check(display_export.is_open(), "display export opened");
        display_export << '[';
    }
    wheel_eligibility();
    const ReverseRig rig{3.8, -0.6, 0.12, 7.2, 3.0, 2.55};
    const ReverseState start{{1200, -3500, 0.3}, 0.3, 0};
    const PlannerLimits limits;
    // Exact straight reference, independent of the primitive implementation.
    auto straight = start;
    check(reverse_primitive(straight, rig, 0, 8, limits), "straight primitive valid");
    check(std::abs(straight.tractor.x - start.tractor.x - std::sin(0.3)*8) < 1e-8, "analytic straight X");
    check(std::abs(straight.tractor.z - start.tractor.z - std::cos(0.3)*8) < 1e-8, "analytic straight Z");
    check(std::abs(straight.trailer_heading - 0.3) < 1e-8, "analytic straight trailer yaw");
    auto circle = start; circle.steering = 0.2;
    check(reverse_primitive(circle, rig, 0.2, 3.0, limits), "constant-curvature reference valid");
    const double curvature = std::tan(0.2) / rig.wheelbase;
    const double end_heading = start.tractor.heading - curvature*3.0;
    check(std::abs(circle.tractor.x - start.tractor.x -
        (std::cos(end_heading)-std::cos(start.tractor.heading))/curvature)<1e-5, "analytic circular X");
    check(std::abs(circle.tractor.z - start.tractor.z -
        (std::sin(start.tractor.heading)-std::sin(end_heading))/curvature)<1e-5, "analytic circular Z");
    ReversePlanner planner;
    auto goal = ReverseGoal{reverse_rear(straight, rig)};
    planner.start(start, goal, rig); solve(planner); replay(planner, start, goal, rig);
    // Reachable goals generated with smooth manual left/right/straight sequences.
    for (int sign : {-1, 1})
    {
        auto endpoint = start;
        check(reverse_primitive(endpoint, rig, sign*0.24, 2.25, limits), "curved fixture first leg");
        check(reverse_primitive(endpoint, rig, -sign*0.12, 2.25, limits), "curved fixture second leg");
        check(reverse_primitive(endpoint, rig, 0, 2.25, limits), "curved fixture final leg");
        goal = {reverse_rear(endpoint, rig)};
        planner.start(start, goal, rig); solve(planner);
        std::cout << "Curve " << sign << ": status=" << static_cast<int>(planner.status())
                  << " expansions=" << planner.expansions() << '\n';
        replay(planner, start, goal, rig);
    }
    auto bent_start = start; bent_start.trailer_heading += 0.20;
    auto bent_end = bent_start;
    check(reverse_primitive(bent_end, rig, -0.25, 2.25, limits), "initial articulation recovery leg");
    check(reverse_primitive(bent_end, rig, 0, 3.0, limits), "initial articulation coast leg");
    goal = {reverse_rear(bent_end, rig)};
    planner.start(bent_start, goal, rig); solve(planner); replay(planner, bent_start, goal, rig);
    auto wrap_start = start; wrap_start.tractor.heading = pi-0.01; wrap_start.trailer_heading = -pi+0.01;
    auto wrap_end = wrap_start;
    check(reverse_primitive(wrap_end, rig, 0, 4.5, limits), "wrapped yaw fixture valid");
    goal = {reverse_rear(wrap_end, rig)};
    planner.start(wrap_start, goal, rig); solve(planner); replay(planner, wrap_start, goal, rig);
    // Translation/rotation roundtrip includes an SDK floating-origin cell boundary.
    const Pose origin{513.7, -1025.4, 2.8}, point{521.2, -1019.7, -3.1};
    const auto roundtrip = from_ground_frame(to_ground_frame(point, origin), origin);
    check(std::hypot(roundtrip.x-point.x, roundtrip.z-point.z)<1e-9 &&
        std::abs(normalize_angle(roundtrip.heading-point.heading))<1e-9, "world/local roundtrip");
    auto bad = start; bad.steering = std::numeric_limits<double>::quiet_NaN();
    planner.start(bad, goal, rig); check(planner.status()==PlanStatus::invalid, "NaN steering rejected");
    bad = start; bad.steering = 40.0 * pi / 180.0;
    planner.start(bad, goal, rig);
    check(planner.status()!=PlanStatus::invalid, "measured 40 degree road wheel can start a plan");
    bad.steering = 57.0 * pi / 180.0;
    planner.start(bad, goal, rig);
    check(planner.status()==PlanStatus::invalid, "road wheel past 50 degrees is rejected");
    bad = start; bad.tractor.heading = std::numeric_limits<double>::infinity();
    planner.start(bad, goal, rig); check(planner.status()==PlanStatus::invalid, "infinite heading rejected before normalization");
    bad = start; bad.trailer_heading = 1e300;
    planner.start(bad, goal, rig); check(planner.status()==PlanStatus::invalid, "huge finite heading rejected without looping");
    bad = start; bad.trailer_heading += 1.2;
    planner.start(bad, goal, rig); check(planner.status()==PlanStatus::invalid, "jackknifed start rejected");
    auto invalid_rig = rig; invalid_rig.trailer_length = -1;
    planner.start(start, goal, invalid_rig); check(planner.status()==PlanStatus::invalid, "invalid geometry rejected");
    auto budget = limits; budget.maximum_expansions = 1;
    planner.start(start, goal, rig, budget); solve(planner);
    check(planner.status()==PlanStatus::budget && planner.route().empty(), "budget exhaustion is explicit and has no partial route");
    goal.rear.x += 1000;
    planner.start(start, goal, rig); check(planner.status()==PlanStatus::outside_envelope, "remote target rejected");
    // A goal in front of the starting rear cannot be reached by a tiny reverse envelope.
    goal = {reverse_rear(start, rig)};
    goal.rear.x -= 3*std::sin(start.tractor.heading); goal.rear.z -= 3*std::cos(start.tractor.heading);
    auto small = limits; small.maximum_distance = 0.75;
    planner.start(start, goal, rig, small); solve(planner);
    check(planner.status()==PlanStatus::exhausted, "finite envelope exhaustion distinct from budget");
    goal = {reverse_rear(start, rig)}; goal.rear.heading += 0.5;
    check(!goal_reached(start, goal, rig, limits), "position alone cannot mean arrival");
    auto folded = start; folded.trailer_heading += 0.7;
    goal = {reverse_rear(folded, rig)};
    check(!goal_reached(folded, goal, rig, limits), "arrival also limits articulation");

    ReverseTarget target;
    TargetInput toggle; toggle.toggle = true;
    target.update(start, rig, 0, false, true, 2000, toggle);
    check(target.active() && target.editing(), "target placement allowed stationary in forward/neutral");
    const auto fixed = target.goal().rear;
    auto moved = start; moved.tractor.x += 2;
    target.update(moved, rig, 0, false, true, 2100, {});
    check(target.goal().rear.x == fixed.x && target.goal().rear.z == fixed.z, "goal persists in world coordinates");
    TargetInput edit; edit.right = 0.5; edit.rotation = pi/36;
    target.update(start, rig, 0, false, true, 2200, edit);
    check(std::abs(target.goal().rear.heading - fixed.heading - pi/36)<1e-9, "goal rotation applied");
    check(std::hypot(target.goal().rear.x-fixed.x, target.goal().rear.z-fixed.z)>0.49, "goal translation applied");
    target.reset();
    target.update(start, rig, 0, true, true, 4000, toggle);
    export_display(display_export, "EDIT", target, rig, start, first_export);
    TargetInput confirm; confirm.confirm = true;
    target.update(start, rig, 0, true, true, 4100, confirm);
    for (int i=0; i<1000 && target.cue()==TargetCue::wait; ++i)
        target.update(start, rig, 0, true, true, 4200+i*16, {});
    check(target.planner().status()==PlanStatus::ready, "manual target straight route found");
    const auto draw = target_geometry(target, rig, start.tractor);
    check(draw.count>50 && draw.count<=maximum_guidance_segments-96 && !draw.truncated,
        "target route/labels reserve at least 96 current-forecast segments");
    check(draw.segments.front().style==GuideStyle::target, "selected target uses separate cyan style");
    export_display(display_export, "READY", target, rig, start, first_export);
    for (std::size_t i=0; i<draw.count; ++i)
        check(finite_point(draw.segments[i].from) && finite_point(draw.segments[i].to), "finite target rendering coordinates");
    for (int c = 0; c <= static_cast<int>(TargetCue::slow); ++c)
    {
        GuidanceGeometry label;
        target_label(label, static_cast<TargetCue>(c), 0, 0);
        check(label.count==1 && !label.truncated, "each Chinese cue uses one model pose");
        check(label.segments[0].style == static_cast<GuideStyle>(static_cast<std::size_t>(GuideStyle::cue_off)+c),
            "all thirteen cues map to distinct baked Chinese models");
        check(chinese_cue_text[c].size() >= 6 && static_cast<unsigned char>(chinese_cue_text[c][0]) >= 0xe0 &&
            chinese_cue_width[c] > 0 && chinese_cue_width[c] < 4,
            "Chinese UTF8 display text and physical dimensions present");
        GuidanceGeometry opposite;
        target_label(opposite, static_cast<TargetCue>(c), 0, 0, pi);
        check(label.count==opposite.count, "opposite-view cue has same model count");
        for (std::size_t i=0; i<label.count; ++i)
        {
            const auto &a=label.segments[i]; const auto &b=opposite.segments[i];
            check(std::hypot(a.from.x+b.from.x,a.from.z+b.from.z)<1e-8 &&
                std::hypot(a.to.x+b.to.x,a.to.z+b.to.z)<1e-8, "front and rear cues face opposite directions without mirroring");
            check(std::abs(a.from.z)<=0.40 && std::abs(a.to.z)<=0.40, "cue orientation anchor remains compact");
        }
    }
    GuidanceGeometry full_forecast;
    full_forecast.count = maximum_guidance_segments;
    for (std::size_t i=0; i<full_forecast.count; ++i)
        full_forecast.segments[i].from.x = static_cast<double>(i);
    auto combined = draw;
    append_current_guidance(combined, full_forecast);
    check(combined.count==maximum_guidance_segments && combined.segments[draw.count].from.x==0 &&
        combined.segments[combined.count-1].from.x==255, "shared display retains beginning and tail of current forecast");
    const auto route_copy = target.planner().route();
    check(route_copy.size() > 21, "straight route has an early tracking sample");
    ReverseTarget tracking = target;
    auto nudged = route_copy[20].state;
    nudged.tractor.x += 0.5;
    nudged.tractor.heading = normalize_angle(nudged.tractor.heading + 8.0 * pi / 180.0);
    nudged.trailer_heading = normalize_angle(nudged.trailer_heading + 8.0 * pi / 180.0);
    tracking.update(nudged, rig, -0.4, true, true, 4900, {});
    check(tracking.planner().status()==PlanStatus::ready, "half-metre and eight-degree tracking error stays on the plan");
    for (std::size_t i=5; i+1<route_copy.size(); i+=5)
        target.update(route_copy[i].state, rig, -0.4, true, true, 5000+i*16, {});
    check(target.planner().status()==PlanStatus::ready, "following the route does not trigger unnecessary replans");
    target.update(route_copy.back().state, rig, -0.4, true, true, 5800, {});
    check(target.cue()==TargetCue::stop, "reaching pose while moving requests stop");
    target.update(route_copy.back().state, rig, 0, true, true, 5900, {});
    check(target.cue()==TargetCue::arrived, "arrival requires rear pose and stopped truck");
    target.update(route_copy.back().state, rig, 0, false, true, 5950, {});
    check(target.cue()==TargetCue::arrived && !target.planner().route().empty(),
        "arrival stays after leaving reverse");
    const auto arrived_display = target_geometry(target, rig, start.tractor);
    check(arrived_display.segments[0].style==GuideStyle::straightening, "arrived target turns green");
    export_display(display_export, "DONE", target, rig, route_copy.back().state, first_export);
    target.update(start, rig, -2, true, true, 6000, {});
    check(target.cue()==TargetCue::slow && target.planner().route().empty(), "overspeed hides route/cue");
    target.update(start, rig, 0, true, false, 6100, {});
    check(target.cue()==TargetCue::data && target.planner().route().empty(), "unsupported data never yields steering cue");
    export_display(display_export, "DATA", target, rig, start, first_export);
    target.update(start, rig, 0, true, true, 7500, {});
    check(target.planner().status()==PlanStatus::searching, "restored data can replan after stopping");
    target.reset(); check(!target.active() && target.planner().route().empty(), "scene reset clears target and route");
    // Search tolerates creep at the stop-speed limit. One slice may finish a
    // straight goal, so success is either still searching or a retained route.
    target.update(start, rig, 0, true, true, 8000, toggle);
    target.update(start, rig, 0, true, true, 8100, confirm);
    auto crept = start;
    crept.tractor.x += 0.20;
    crept.steering = 0.05;
    target.update(crept, rig, 0.04, true, true, 8200, {});
    check(target.cue()!=TargetCue::stop && target.planner().status()!=PlanStatus::idle,
        "stop-speed creep and small wheel noise keep the search");
    target.reset();
    target.update(start, rig, 0, true, true, 8300, toggle);
    target.update(start, rig, 0, true, true, 8400, confirm);
    auto turned = start; turned.steering = 0.25;
    target.update(turned, rig, 0, true, true, 8500, {});
    check(target.cue()==TargetCue::stop && target.planner().route().empty(), "wheel change invalidates in-progress search");
    auto changed_rig = rig; changed_rig.hitch_rear += 0.2;
    target.update(start, changed_rig, 0, true, true, 8400, {});
    check(!target.active(), "geometry change invalidates world target and route");
    target.update(start, rig, 0, true, true, 9000, toggle);
    target.update(start, rig, 0, true, true, 9100, confirm);
    for (int i=0; i<1000 && target.cue()==TargetCue::wait; ++i)
        target.update(start, rig, 0, true, true, 9200+i*16, {});
    const auto before_deviation = target.goal().rear;
    auto deviated = start; deviated.tractor.x += 4;
    target.update(deviated, rig, -0.4, true, true, 10000, {});
    check(target.cue()==TargetCue::stop && target.planner().route().empty(), "deviation requests stop before replan");
    check(target.goal().rear.x==before_deviation.x && target.goal().rear.z==before_deviation.z,
        "deviation preserves chosen world target");
    target.update(deviated, rig, 0, true, true, 11000, {});
    check(target.planner().status()==PlanStatus::searching, "deviation replans from current stopped pose");
    ReverseTarget rejected;
    rejected.update(start, rig, 0, true, false, 12000, toggle);
    check(!rejected.active() && rejected.cue()==TargetCue::data, "rejected activation reports missing data");
    const auto rejected_draw = target_geometry(rejected, rig, start.tractor);
    check(rejected_draw.count==2 && rejected_draw.segments[0].style==GuideStyle::cue_data &&
        rejected_draw.segments[1].style==GuideStyle::cue_data,
        "rejected activation shows the data cue without a target");
    rejected.update(start, rig, 1.0, true, true, 12100, toggle);
    check(!rejected.active() && rejected.cue()==TargetCue::stop, "moving activation asks the driver to stop");
    const auto moving_draw = target_geometry(rejected, rig, start.tractor);
    check(moving_draw.count==2 && moving_draw.segments[0].style==GuideStyle::cue_stop,
        "moving activation shows the stop cue");
    ReverseTarget turned_wheels;
    auto turned_state = start; turned_state.steering = 40.0 * pi / 180.0;
    turned_wheels.update(turned_state, rig, 0, false, true, 12200, toggle);
    check(turned_wheels.active() && turned_wheels.cue()!=TargetCue::data,
        "stopped truck with a 40 degree road wheel can place a target");

    // 1-pull-up bidirectional planner: a tight goal with large initial fold that
    // pure reverse cannot reach cleanly is solved via one initial pull-up leg.
    {
        ReverseState folded_start = start;
        folded_start.trailer_heading = normalize_angle(start.tractor.heading + 46.0 * pi / 180.0);
        // Generate an exact 1-pull-up ground-truth goal: pull forward 4.5 m to
        // straighten, then reverse 8.25 m into an aligned bay behind the truck.
        ReverseState pullup_fixture = folded_start;
        check(motion_primitive(pullup_fixture, rig, 0.28, 4.5, true, limits),
              "1-pull-up fixture forward leg valid");
        check(motion_primitive(pullup_fixture, rig, 0.0, 8.25, false, limits),
              "1-pull-up fixture reverse leg valid");
        const ReverseGoal tight_goal{reverse_rear(pullup_fixture, rig)};

        PlannerLimits no_pullup = limits;
        no_pullup.allow_pullup = false;
        ReversePlanner pure_rev;
        pure_rev.start(folded_start, tight_goal, rig, no_pullup);
        solve(pure_rev);

        ReversePlanner bidir;
        bidir.start(folded_start, tight_goal, rig, limits);
        solve(bidir);
        check(bidir.status() == PlanStatus::ready, "1-pull-up planner solves tightly folded bay");
        check(bidir.has_pullup() && bidir.switch_index() > 0 &&
              bidir.switch_index() + 1 < bidir.route().size(),
              "solution uses an initial forward pull-up leg before reversing");
        // Verify full bidirectional kinematic replay of every 5 cm sample.
        ReverseState sim = folded_start;
        double prev_dist = 0.0;
        for (const auto &sample : bidir.route())
        {
            const double ds = sample.distance - prev_dist;
            if (ds > 0.0)
                check(motion_primitive(sim, rig, sample.state.steering, ds, sample.forward, limits),
                      "bidirectional replay step valid");
            check(std::hypot(sim.tractor.x - sample.state.tractor.x,
                             sim.tractor.z - sample.state.tractor.z) < 0.003,
                  "bidirectional replay tractor position");
            check(std::abs(normalize_angle(sim.trailer_heading - sample.state.trailer_heading)) < 0.002,
                  "bidirectional replay trailer heading");
            prev_dist = sample.distance;
        }
        check(goal_reached(sim, tight_goal, rig, limits), "bidirectional replay reaches goal");

        // Walk ReverseTarget state machine through pull-up -> switch STOP -> reverse -> arrived.
        ReverseTarget pullup_target;
        pullup_target.update(folded_start, rig, 0, true, true, 13000, toggle);
        const auto cur_goal = pullup_target.goal().rear;
        TargetInput shift_goal{};
        const auto delta = to_ground_frame(tight_goal.rear, folded_start.tractor);
        const auto cur_local = to_ground_frame(cur_goal, folded_start.tractor);
        shift_goal.right = delta.x - cur_local.x;
        shift_goal.rearward = delta.z - cur_local.z;
        shift_goal.rotation = normalize_angle(tight_goal.rear.heading - cur_goal.heading);
        shift_goal.confirm = true;
        pullup_target.update(folded_start, rig, 0, true, true, 13100, shift_goal);
        for (int i = 0; i < 1500 && pullup_target.cue() == TargetCue::wait; ++i)
            pullup_target.update(folded_start, rig, 0, true, true, 13200 + i * 16, {});
        check(pullup_target.planner().status() == PlanStatus::ready &&
              pullup_target.planner().has_pullup(),
              "ReverseTarget finds 1-pull-up route");
        // Still in reverse gear at start of pull-up leg -> prompts MOVE (pull forward).
        pullup_target.update(folded_start, rig, 0, true, true, 40000, {});
        check(pullup_target.cue() == TargetCue::move &&
              pullup_target.planner().status() == PlanStatus::ready,
              "pull-up leg prompts driver to pull forward while still in reverse gear");
        // Verify visual distinction: pull-up leg blue, switch marker green, reverse leg cyan.
        const auto pullup_draw = target_geometry(pullup_target, rig, folded_start.tractor);
        check(pullup_draw.count <= maximum_guidance_segments - 96 && !pullup_draw.truncated,
              "1-pull-up target display respects 96-segment reserve");
        bool has_blue_route = false, has_cyan_route = false, has_green_switch = false;
        for (std::size_t i = 0; i < pullup_draw.count; ++i)
        {
            if (pullup_draw.segments[i].layer != GuideLayer::target_route) continue;
            if (pullup_draw.segments[i].style == GuideStyle::tractor) has_blue_route = true;
            if (pullup_draw.segments[i].style == GuideStyle::target) has_cyan_route = true;
            if (pullup_draw.segments[i].style == GuideStyle::straightening) has_green_switch = true;
        }
        check(has_blue_route && has_cyan_route && has_green_switch,
              "1-pull-up route renders blue pull-up leg, green switch marker and cyan reverse leg");

        const auto &pr = pullup_target.planner().route();
        const auto sw = pullup_target.planner().switch_index();
        // Shift to forward gear and follow pull-up leg before the 0.40 m switch-stop zone.
        const std::size_t mid_pullup = sw > 12 ? sw - 12 : sw / 2;
        for (std::size_t i = 2; i <= mid_pullup; i += 4)
            pullup_target.update(pr[i].state, rig, 0.5, false, true, 40100 + i * 16, {});
        check(pullup_target.planner().status() == PlanStatus::ready &&
              pullup_target.cue() != TargetCue::stop && pullup_target.cue() != TargetCue::move,
              "pull-up forward driving tracks smoothly with steering cues");
        pullup_target.update(pr[sw].state, rig, 0.3, false, true, 42000, {});
        check(pullup_target.cue() == TargetCue::stop, "reaching switch point requests stop");
        // Stop and shift into reverse gear -> smoothly enters reverse leg.
        pullup_target.update(pr[sw].state, rig, 0.0, true, true, 42100, {});
        check(pullup_target.cursor() > sw && pullup_target.planner().status() == PlanStatus::ready,
              "shifting to reverse at switch point advances into reverse leg");
        for (std::size_t i = sw + 2; i + 1 < pr.size(); i += 5)
            pullup_target.update(pr[i].state, rig, -0.4, true, true, 42200 + i * 16, {});
        pullup_target.update(pr.back().state, rig, 0.0, true, true, 46000, {});
        check(pullup_target.cue() == TargetCue::arrived,
              "completing 1-pull-up route reaches arrived state");
    }

    // Closed-loop feedback-linearized steering converges from mirrored +/-0.45 m
    // lateral perturbations to millimeter-level lateral accuracy without replanning.
    for (double sign : {-1.0, 1.0})
    {
        ReverseTarget cl_target;
        cl_target.update(start, rig, 0, true, true, 50000, toggle);
        TargetInput longer{}; longer.rearward = 10.0; longer.confirm = true;
        cl_target.update(start, rig, 0, true, true, 50100, longer);
        for (int i = 0; i < 1000 && cl_target.cue() == TargetCue::wait; ++i)
            cl_target.update(start, rig, 0, true, true, 50200 + i * 16, {});
        check(cl_target.planner().status() == PlanStatus::ready, "closed-loop straight plan ready");

        ReverseState sim = start;
        sim.tractor.x += std::cos(start.tractor.heading) * (0.45 * sign);
        sim.tractor.z -= std::sin(start.tractor.heading) * (0.45 * sign);
        const auto init_err = to_ground_frame(reverse_rear(sim, rig), cl_target.goal().rear);
        std::uint64_t t = 55000;
        for (int step = 0; step < 400; ++step)
        {
            cl_target.update(sim, rig, -0.4, true, true, t, {});
            t += 50;
            if (goal_reached(sim, cl_target.goal(), rig, limits)) break;
            if (cl_target.planner().status() != PlanStatus::ready) break;
            check(reverse_primitive(sim, rig, cl_target.desired_steering(), 0.05, limits),
                  "closed-loop feedback step respects articulation");
        }
        const auto final_err = to_ground_frame(reverse_rear(sim, rig), cl_target.goal().rear);
        check(std::abs(final_err.x) < 0.10 && std::abs(final_err.x) < std::abs(init_err.x) * 0.25 &&
              std::abs(final_err.heading) < 3.0 * pi / 180.0 &&
              goal_reached(sim, cl_target.goal(), rig, limits),
              "closed-loop steering converges cleanly from lateral perturbation to goal");
    }

    // Semantic layered 256-segment budget preserves compact indicators intact.
    {
        TractorSpec tractor;
        const std::vector<TrailerSpec> tr{{15.0 * pi / 180.0, 7.2, 3.0, 2.55}};
        const auto pred = predict_reverse_boxes(0.4, tractor.wheelbase, tractor.length,
            tractor.width, tr, 0, 0, tractor.center_from_rear, 8.0, 0.05, 0.25,
            0.0, 0.0, 0.35);
        const auto full_cur = build_reverse_guidance(pred, tractor, GuidanceOptions{}, &tr.front());
        GuidanceGeometry tight_display = draw;
        // Pad target display to 156 segments so remaining budget is 100 (< full_cur.count).
        while (tight_display.count < 156)
            add_guide_segment(tight_display, {0, 0, 0}, {0, 0, 0.36}, BodyKind::tractor,
                              GuideStyle::target, GuideLayer::target_route);
        check(full_cur.count > 100, "full multi-layer guidance exceeds tight remaining budget");
        append_current_guidance(tight_display, full_cur);
        check(tight_display.count <= maximum_guidance_segments, "layered merge stays within 256 budget");
        std::size_t kept_artic = 0, kept_corridor = 0, kept_trailer = 0;
        for (std::size_t i = 156; i < tight_display.count; ++i)
        {
            if (tight_display.segments[i].layer == GuideLayer::articulation) ++kept_artic;
            if (tight_display.segments[i].layer == GuideLayer::corridor) ++kept_corridor;
            if (tight_display.segments[i].layer == GuideLayer::trailer_track) ++kept_trailer;
        }
        check(kept_artic == full_cur.articulation_segments &&
              kept_corridor == full_cur.corridor_segments && kept_trailer >= 24,
              "semantic budget preserves whole articulation and corridor layers plus trailer tracks");
    }
    if (display_export.is_open()) display_export << ']';
    std::cout << "PlannerTests checks=" << checks << " failures=" << failures << '\n';
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
