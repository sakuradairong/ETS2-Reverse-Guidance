#include "build_profiles.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

using namespace reverse_assist::compatibility;

bool file_signature_matches(const std::vector<std::uint8_t> &file,
                            std::uintptr_t rva,
                            const std::uint8_t *signature,
                            std::size_t signature_size)
{
    if (file.size() < sizeof(IMAGE_DOS_HEADER)) return false;
    const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(file.data());
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 ||
        static_cast<std::size_t>(dos->e_lfanew) >
            file.size() - sizeof(IMAGE_NT_HEADERS64))
        return false;
    const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS64 *>(
        file.data() + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE ||
        nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC)
        return false;
    const auto *section = IMAGE_FIRST_SECTION(nt);
    for (unsigned index = 0; index < nt->FileHeader.NumberOfSections;
         ++index, ++section)
    {
        const auto start = static_cast<std::uintptr_t>(section->VirtualAddress);
        const auto span = static_cast<std::uintptr_t>(
            std::max(section->Misc.VirtualSize, section->SizeOfRawData));
        if (rva < start || rva - start > span ||
            signature_size > span - (rva - start))
            continue;
        const auto offset = static_cast<std::size_t>(section->PointerToRawData) +
                            static_cast<std::size_t>(rva - start);
        return offset <= file.size() && signature_size <= file.size() - offset &&
               std::memcmp(file.data() + offset, signature, signature_size) == 0;
    }
    return false;
}

int main(int argc, char **argv)
{
    std::size_t image_size = 0x01600000;
    const auto grow_image = [&](const auto &specs) {
        for (const auto &spec : specs)
            image_size = std::max(image_size,
                static_cast<std::size_t>(spec.rva) + spec.signature.size());
    };
    for (const auto &profile : build_profiles)
        grow_image(profile.enabled_hooks);
    grow_image(verified_161_1_9_model_helpers);
    grow_image(verified_162_1_0_model_helpers);
    grow_image(verified_162_1_0_model_methods);
    grow_image(verified_162_3_3_model_helpers);
    grow_image(verified_162_3_3_model_methods);
    std::vector<std::uint8_t> image(image_size, 0xcc);
    const auto stamp_image = [&](const auto &specs) {
        for (const auto &spec : specs)
            std::copy(spec.signature.begin(), spec.signature.end(), image.begin() + spec.rva);
    };
    for (const auto &profile : build_profiles)
        stamp_image(profile.enabled_hooks);
    stamp_image(verified_161_1_9_model_helpers);
    stamp_image(verified_162_1_0_model_helpers);
    stamp_image(verified_162_1_0_model_methods);
    stamp_image(verified_162_3_3_model_helpers);
    stamp_image(verified_162_3_3_model_methods);

    const auto reader = [&](std::uintptr_t rva, const std::uint8_t *bytes,
                            std::size_t size) {
        return rva <= image.size() && size <= image.size() - rva &&
               std::memcmp(image.data() + rva, bytes, size) == 0;
    };

    bool exact = false;
    std::size_t failed = 99;
    for (const auto &profile : build_profiles)
    {
        if (profile.executable_sha256.empty()) continue;
        assert(select_build_profile(profile.executable_sha256, reader, &exact, &failed) == &profile);
        assert(exact);
        for (const auto &hook : profile.enabled_hooks)
        {
            image[hook.rva] ^= 0xff;
            assert(select_build_profile(profile.executable_sha256, reader, &exact, &failed) == nullptr);
            image[hook.rva] ^= 0xff;
        }
    }
    const auto assert_helper_mutations_rejected = [&](const auto &specs, auto matcher) {
        assert(matcher(reader));
        for (const auto &spec : specs)
        {
            image[spec.rva + spec.signature.size() - 1] ^= 0xff;
            assert(!matcher(reader));
            image[spec.rva + spec.signature.size() - 1] ^= 0xff;
        }
    };
    assert_helper_mutations_rejected(verified_161_1_9_model_helpers,
                                     [](auto &&r) { return model_helpers_161_1_9_match(r); });
    assert_helper_mutations_rejected(verified_162_1_0_model_helpers,
                                     [](auto &&r) { return model_helpers_162_1_0_match(r); });
    assert_helper_mutations_rejected(verified_162_1_0_model_methods,
                                     [](auto &&r) { return model_helpers_162_1_0_match(r); });
    assert_helper_mutations_rejected(verified_162_3_3_model_helpers,
                                     [](auto &&r) { return model_helpers_162_3_3_match(r); });
    assert_helper_mutations_rejected(verified_162_3_3_model_methods,
                                     [](auto &&r) { return model_helpers_162_3_3_match(r); });
    // Newer layouts must not become an unrestricted signature-only fallback.
    const auto reject_unknown_hash_for = [&](const auto &hooks) {
        const auto only_reader = [&](std::uintptr_t rva, const std::uint8_t *bytes, std::size_t size) {
            for (const auto &hook : hooks)
                if (rva == hook.rva) return reader(rva, bytes, size);
            return false;
        };
        assert(select_build_profile("UNKNOWN-NEW-BUILD", only_reader, &exact, &failed) == nullptr);
    };
    reject_unknown_hash_for(verified_161_1_9_hooks);
    reject_unknown_hash_for(verified_162_1_0_hooks);
    reject_unknown_hash_for(verified_162_3_3_hooks);

    // A second binary digest is supported by the separately declared
    // signature-compatible profile when all enabled hooks remain verified.
    const auto *compatible_build = select_build_profile(
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
        reader, &exact, &failed);
    assert(compatible_build && compatible_build->signature_compatible_fallback);
    assert(!exact);

    // Bytes at an old, inactive lifecycle helper are intentionally irrelevant.
    image[0x015288d0] ^= 0xff;
    compatible_build = select_build_profile("SECOND-BUILD", reader, &exact, &failed);
    assert(compatible_build && compatible_build->signature_compatible_fallback);

    // Any mismatch in an actually enabled hook causes safe rejection.
    const auto &required = verified_160_1_7_hooks[2];
    image[required.rva] ^= 0xff;
    assert(select_build_profile("UNSUPPORTED", reader, &exact, &failed) == nullptr);
    assert(failed == 2);

    std::cout << "build profile tests passed: 1.60, 1.61.1.0, 1.61.1.1, 1.61.1.9, 1.62.1.0, 1.62.3.3 exact profiles + legacy fallback; "
                 "all hook/helper/vtable-method mutations rejected; unknown-new hashes rejected\n";

    if (argc == 3)
    {
        std::ifstream stream(argv[1], std::ios::binary);
        assert(stream);
        std::vector<std::uint8_t> executable(
            (std::istreambuf_iterator<char>(stream)),
            std::istreambuf_iterator<char>());
        const auto file_reader =
            [&](std::uintptr_t rva, const std::uint8_t *bytes,
                std::size_t size) {
                return file_signature_matches(executable, rva, bytes, size);
            };
        const auto *exact_file = select_build_profile(
            argv[2], file_reader, &exact, &failed);
        assert(exact_file && exact);
        if (exact_file->game_version == "1.61.1.9")
            assert(model_helpers_161_1_9_match(file_reader));
        else if (exact_file->game_version == "1.62.1.0")
            assert(model_helpers_162_1_0_match(file_reader));
        else if (exact_file->game_version == "1.62.3.3")
            assert(model_helpers_162_3_3_match(file_reader));
        const auto *dispatch = find_hook(*exact_file, HookId::vehicle_render_dispatch);
        assert(dispatch);
        // The selected entry must dispatch both trailer and tractor virtual
        // render methods. A similar prologue in a serializer passed a shorter
        // byte-pattern check but was never called in the driving scene.
        const std::array<std::uint8_t, 7> trailer_member{
            0x48, 0x8b, 0x8e, 0xc8, 0x00, 0x00, 0x00};
        const std::array<std::uint8_t, 6> virtual_render{
            0xff, 0x90, 0x00, 0x03, 0x00, 0x00};
        assert(file_signature_matches(executable, dispatch->rva + 0xe1,
                                      trailer_member.data(), trailer_member.size()));
        assert(file_signature_matches(executable, dispatch->rva + 0xfa,
                                      virtual_render.data(), virtual_render.size()));
        assert(file_signature_matches(executable, dispatch->rva + 0x18b,
                                      virtual_render.data(), virtual_render.size()));
        std::cout << "installed executable verified: " << argv[1]
                  << "; exact=" << exact_file->id
                  << "; enabled-hooks=" << exact_file->enabled_hooks.size()
                  << '\n';
    }
    return 0;
}
