#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace reverse_assist::compatibility
{
enum class HookId
{
    model_load,
    vehicle_render_dispatch,
    trailer_visual_update,
    trailer_render,
};

struct HookSpec
{
    HookId id;
    std::string_view name;
    std::uintptr_t rva;
    std::array<std::uint8_t, 16> signature;
};

struct BuildProfile
{
    std::string_view id;
    std::string_view game_version;
    std::string_view executable_sha256;
    bool signature_compatible_fallback;
    std::array<HookSpec, 4> enabled_hooks;
};

constexpr std::array verified_160_1_7_hooks{
    HookSpec{HookId::model_load, "model_load", 0x015184f0,
             {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
              0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch",
             0x00772020,
             {0x40, 0x53, 0x57, 0x48, 0x83, 0xec, 0x28, 0x48,
              0x8b, 0x42, 0x18, 0x48, 0x8b, 0xda, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update",
             0x00614190,
             {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
              0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x006148f0,
             {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
              0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

// Steam public 1.61.1.1 (6949e633e779). All four sites were identified in
// the installed executable; the vehicle dispatch stack frame changed in 1.61.
constexpr std::array verified_161_1_1_hooks{
    HookSpec{HookId::model_load, "model_load", 0x015d99f0,
             {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
              0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch",
             0x00798720,
             {0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83,
              0xec, 0x20, 0x48, 0x8b, 0x42, 0x18, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update",
             0x00637620,
             {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
              0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x00637d90,
             {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
              0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

// Steam public 1.61.1.0 (f02fca2efa75). All four sites were identified in
// the installed executable; the vehicle dispatch stack frame changed in 1.61.
constexpr std::array verified_161_1_0_hooks{
    HookSpec{HookId::model_load, "model_load", 0x015d8f40,
             {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
              0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch",
             0x00798570,
             {0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83,
              0xec, 0x20, 0x48, 0x8b, 0x42, 0x18, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update",
             0x00637500,
             {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
              0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x00637c70,
             {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
              0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

// Steam public 1.61.1.9 (05126873bfae). Entries were checked against the
// on-disk PE, its unwind function boundaries and the render dispatch body.
constexpr std::array verified_161_1_9_hooks{
    HookSpec{HookId::model_load, "model_load", 0x015db820,
             {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
              0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch", 0x00798b30,
             {0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83,
              0xec, 0x20, 0x48, 0x8b, 0x42, 0x18, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update", 0x00637940,
             {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
              0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x006380b0,
             {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
              0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

// Steam public 1.62.1.0 (1afd6ae63840). Located through instruction
// signatures, PE unwind entries, model callers and vehicle render bodies.
constexpr std::array verified_162_1_0_hooks{
    HookSpec{HookId::model_load, "model_load", 0x160ef10,
        {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
         0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch", 0x783c90,
        {0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8b, 0x42, 0x18, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update", 0x62c110,
        {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
         0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x62c880,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
         0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

// Steam public 1.62.3.3 (7173c707daa0). Instruction bodies, PE unwind
// entries, caller argument setup and model/trailer vtables inspected.
constexpr std::array verified_162_3_3_hooks{
    HookSpec{HookId::model_load, "model_load", 0x160f260,
        {0x40, 0x53, 0x56, 0x41, 0x56, 0x48, 0x83, 0xec,
         0x40, 0x80, 0xbc, 0x24, 0x80, 0x00, 0x00, 0x00}},
    HookSpec{HookId::vehicle_render_dispatch, "vehicle_render_dispatch", 0x784170,
        {0x48, 0x89, 0x74, 0x24, 0x20, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8b, 0x42, 0x18, 0x48, 0x8b}},
    HookSpec{HookId::trailer_visual_update, "trailer_visual_update", 0x62c5d0,
        {0x48, 0x8b, 0xc4, 0x48, 0x89, 0x70, 0x10, 0x48,
         0x89, 0x78, 0x18, 0x55, 0x48, 0x8d, 0x68, 0xa1}},
    HookSpec{HookId::trailer_render, "trailer_render", 0x62cd40,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
         0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x30, 0x49}},
};

struct ModelHelperSpec
{
    std::string_view name;
    std::uintptr_t rva;
    std::array<std::uint8_t, 16> signature;
};

// Order: ownership transfer, parameter init, scene parent, scene transform.
constexpr std::array verified_161_1_9_model_helpers{
    ModelHelperSpec{"model_transfer", 0x0033cc00,
                    {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
                     0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48}},
    ModelHelperSpec{"model_parameter_init", 0x0041ccb0,
                    {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c,
                     0x24, 0x18, 0x56, 0x57, 0x41, 0x56, 0x48, 0x83}},
    ModelHelperSpec{"set_parent", 0x013ce580,
                    {0x48, 0x83, 0xec, 0x28, 0x4c, 0x8b, 0xc1, 0x4c,
                     0x8b, 0xca, 0x48, 0x8b, 0x49, 0x40, 0x48, 0x3b}},
    ModelHelperSpec{"set_transform", 0x013ce640,
                    {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
                     0xec, 0x40, 0x48, 0x8b, 0xd9, 0x48, 0x8b, 0xfa}},
};

template <typename SignatureReader>
bool model_helpers_161_1_9_match(SignatureReader &&reader)
{
    for (const auto &helper : verified_161_1_9_model_helpers)
        if (!reader(helper.rva, helper.signature.data(), helper.signature.size()))
            return false;
    return true;
}

constexpr std::array verified_162_1_0_model_helpers{
    ModelHelperSpec{"model_transfer", 0x345f00,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
         0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48}},
    ModelHelperSpec{"model_parameter_init", 0x426a60,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c,
         0x24, 0x18, 0x56, 0x57, 0x41, 0x56, 0x48, 0x83}},
    ModelHelperSpec{"set_parent", 0x1401e10,
        {0x48, 0x83, 0xec, 0x28, 0x4c, 0x8b, 0xc1, 0x4c,
         0x8b, 0xca, 0x48, 0x8b, 0x49, 0x40, 0x48, 0x3b}},
    ModelHelperSpec{"set_transform", 0x1401ed0,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x40, 0x48, 0x8b, 0xd9, 0x48, 0x8b, 0xfa}},
};

// The factory installs this model vtable; the game submits its +0x90
// method with the render context and finalizes through +0x140.
constexpr std::uintptr_t model_vtable_162_1_0_rva = 0x02492a20;
constexpr std::array verified_162_1_0_model_methods{
    ModelHelperSpec{"model_virtual_0x8", 0x1615ad0,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8d, 0x91, 0xf0, 0x02, 0x00}},
    ModelHelperSpec{"model_virtual_0x90", 0x1616320,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8b, 0xda, 0x48, 0x8b, 0xf9}},
    ModelHelperSpec{"model_virtual_0x140", 0x161d5e0,
        {0x40, 0x56, 0x48, 0x83, 0xec, 0x20, 0x8b, 0x81,
         0x3c, 0x01, 0x00, 0x00, 0x48, 0x8b, 0xf1, 0xc1}},
};

template <typename SignatureReader>
bool model_helpers_162_1_0_match(SignatureReader &&reader)
{
    for (const auto &helper : verified_162_1_0_model_helpers)
        if (!reader(helper.rva, helper.signature.data(), helper.signature.size()))
            return false;
    for (const auto &method : verified_162_1_0_model_methods)
        if (!reader(method.rva, method.signature.data(), method.signature.size()))
            return false;
    return true;
}

constexpr std::array verified_162_3_3_model_helpers{
    ModelHelperSpec{"model_transfer", 0x345f40,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74,
         0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x20, 0x48}},
    ModelHelperSpec{"model_parameter_init", 0x426aa0,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x6c,
         0x24, 0x18, 0x56, 0x57, 0x41, 0x56, 0x48, 0x83}},
    ModelHelperSpec{"set_parent", 0x1402160,
        {0x48, 0x83, 0xec, 0x28, 0x4c, 0x8b, 0xc1, 0x4c,
         0x8b, 0xca, 0x48, 0x8b, 0x49, 0x40, 0x48, 0x3b}},
    ModelHelperSpec{"set_transform", 0x1402220,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x40, 0x48, 0x8b, 0xd9, 0x48, 0x8b, 0xfa}},
};

constexpr std::uintptr_t model_vtable_162_3_3_rva = 0x2492a70;
constexpr std::array verified_162_3_3_model_methods{
    ModelHelperSpec{"model_virtual_0x8", 0x1615e20,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8d, 0x91, 0xf0, 0x02, 0x00}},
    ModelHelperSpec{"model_virtual_0x90", 0x1616670,
        {0x48, 0x89, 0x5c, 0x24, 0x08, 0x57, 0x48, 0x83,
         0xec, 0x20, 0x48, 0x8b, 0xda, 0x48, 0x8b, 0xf9}},
    ModelHelperSpec{"model_virtual_0x140", 0x161d930,
        {0x40, 0x56, 0x48, 0x83, 0xec, 0x20, 0x8b, 0x81,
         0x3c, 0x01, 0x00, 0x00, 0x48, 0x8b, 0xf1, 0xc1}},
};

template <typename SignatureReader>
bool model_helpers_162_3_3_match(SignatureReader &&reader)
{
    for (const auto &helper : verified_162_3_3_model_helpers)
        if (!reader(helper.rva, helper.signature.data(), helper.signature.size()))
            return false;
    for (const auto &method : verified_162_3_3_model_methods)
        if (!reader(method.rva, method.signature.data(), method.signature.size()))
            return false;
    return true;
}

// Exact hashes remain required for 1.61 and 1.62 profiles. The final legacy fallback
// admits only binaries with all four verified 1.60 hook signatures.
constexpr std::array build_profiles{
    // Build identity read from the existing 2026-11-19 game log, not recomputed.
    BuildProfile{
        "ets2-1.62.3.3-steam-public-7173c707daa0",
        "1.62.3.3",
        "055EC5343D53BD427DE54B06ECA74937F0D348DBB0C5D71862EDDDCF45E2DB66",
        false,
        verified_162_3_3_hooks,
    },
    // Identity copied from the existing game/plugin log; no SHA operation was
    // performed during this adaptation. Retain the exact-build runtime gate.
    BuildProfile{
        "ets2-1.62.1.0-steam-public-1afd6ae63840",
        "1.62.1.0",
        "A938CB388121F9B884A2E1B0F048C230A7BC19638EF6D641A70775F8C7FEA956",
        false,
        verified_162_1_0_hooks,
    },
    BuildProfile{
        "ets2-1.61.1.9-steam-public-05126873bfae",
        "1.61.1.9",
        "B70C2357CC56EB5834EE47FEA3C60B2F2D4591383FF2AC479CFD3CD254301B8C",
        false,
        verified_161_1_9_hooks,
    },
    BuildProfile{
        "ets2-1.61.1.1-steam-public-6949e633e779",
        "1.61.1.1",
        "2014BFCC850A06108F4CA3C233A5BB6D31D7D83346C5D449F94C8E0675E5199A",
        false,
        verified_161_1_1_hooks,
    },
    BuildProfile{
        "ets2-1.61.1.0-steam-public-f02fca2efa75",
        "1.61.1.0",
        "4DCB548CAAD924254A60AF7C3BD1DB69DCAF42F7ADF19B5BB5D77EBA2814AF21",
        false,
        verified_161_1_0_hooks,
    },
    BuildProfile{
        "ets2-1.60.1.7-steam-public-23966373",
        "1.60.1.7",
        "B7DFFE6B27402C7DB6DFD52CF982CD5BF292584138B35E3EB8EFB311814AB3F8",
        false,
        verified_160_1_7_hooks,
    },
    BuildProfile{
        "ets2-1.60-compatible-hook-layout",
        "1.60.x signature-compatible",
        "",
        true,
        verified_160_1_7_hooks,
    },
};

template <typename SignatureReader>
bool enabled_hook_signatures_match(const BuildProfile &profile,
                                   SignatureReader &&reader,
                                   std::size_t *failed_index = nullptr)
{
    for (std::size_t index = 0; index < profile.enabled_hooks.size(); ++index)
    {
        const auto &hook = profile.enabled_hooks[index];
        if (!reader(hook.rva, hook.signature.data(), hook.signature.size()))
        {
            if (failed_index) *failed_index = index;
            return false;
        }
    }
    return true;
}

template <typename SignatureReader>
const BuildProfile *select_build_profile(std::string_view executable_sha256,
                                         SignatureReader &&reader,
                                         bool *exact_hash_match = nullptr,
                                         std::size_t *failed_index = nullptr)
{
    if (exact_hash_match) *exact_hash_match = false;
    for (const auto &profile : build_profiles)
    {
        if (profile.executable_sha256.empty() ||
            profile.executable_sha256 != executable_sha256)
            continue;
        if (!enabled_hook_signatures_match(profile, reader, failed_index))
            return nullptr;
        if (exact_hash_match) *exact_hash_match = true;
        return &profile;
    }
    for (const auto &profile : build_profiles)
    {
        if (!profile.signature_compatible_fallback) continue;
        if (enabled_hook_signatures_match(profile, reader, failed_index))
            return &profile;
    }
    return nullptr;
}

constexpr const HookSpec *find_hook(const BuildProfile &profile, HookId id)
{
    for (const auto &hook : profile.enabled_hooks)
        if (hook.id == id) return &hook;
    return nullptr;
}
}
