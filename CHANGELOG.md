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
