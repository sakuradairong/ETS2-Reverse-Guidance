## 0.3.0-preview.5

- Add exact-build ETS2 1.62.3.3 support while retaining the earlier profiles.
- Relocate four native hooks, four model helpers, and model release/render/finalize methods and vtable checks.
- Keep unsupported builds in telemetry-only mode and keep legacy accessory mutation unavailable on 1.62.
- Wire `trailer_corridor`, `corridor_distance_cm`, `session_latch`, `jackknife_cues`, and `[target] allow_pullup` into INI configuration and runtime caching, and pass measured `TrailerSpec` into `build_reverse_guidance` for accurate jackknife critical angles.
- Add forward pull-up visualization (`build_pullup_guidance` + `ReverseSessionLatch`) and non-target real-time jackknife recovery cues reusing existing baked cue assets.
- Upgrade `ReversePlanner` and `ReverseTarget` with optional 1-pull-up bidirectional Hybrid A*, feedback-linearized closed-loop steering tracking, Alt coarse positioning (`2.0 m / 15 deg`), and semantic layered 256-segment budget merging.
- Extend offline unit tests (`GuidanceTests`: 37,346 checks; `PlannerTests`: 5,166 checks) and `BuildProfileTests` coverage for 1.62.1.0 / 1.62.3.3 helpers and vtable methods; Windows MSVC DLL compilation and in-game ETS2 acceptance remain pending.

## 0.3.0-preview.4

- Add exact-build ETS2 1.62.1.0 support for four native hooks and four model helpers.
- Guard the 1.62 model vtable and release/render/finalize method signatures.
- Keep legacy model activation and optional accessory mutation unavailable on 1.62.
- Static binary analysis and DLL compilation only; no tests, smoke tests, SHA verification or in-game acceptance performed.

# Changelog

## 0.2.2 — 2026-10-16

- Add an exact-hash profile for ETS2 Steam public 1.61.1.9 (`05126873bfae`). Update the four rendering Hook entry points and the four model/scene helpers; retain signature validation and telemetry-only fallback.
- Check all four new model helpers with 16-byte signatures before enabling native rendering. Unknown new executable hashes are not admitted by the new profile.
- Extend compatibility tests to every exact profile, individual Hook/helper signature corruption, unknown new hashes, and the installed PE's tractor/trailer virtual-render dispatch.
- Add `Build.ps1 -GameExecutable` for read-only verification against a local game executable. Package versions now come from `VERSION`.
- Validation boundary: static checks and offline tests do not confirm in-game rendering on 1.61.1.9. Live acceptance remains pending; TruckersMP compatibility remains unverified.

## 0.2.1 — 2026-10-13

- Initial standalone MIT source release and Windows x64 package.
- Reverse tracks, tractor front swing, trailer endpoint outline and articulation trend.
- Conservative horizon policy using measured steering, signed speed and tyre-derived ground planes.
