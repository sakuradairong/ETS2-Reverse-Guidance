#!/usr/bin/env python3
"""Offline asset preparation and validation tool for ETS2 Reverse Guidance.

Subcommands:
  prepare          Copy bundled base assets and generate guidance colours into
                   package/home/ets2_reverse_posture_tmp; if bundled assets are
                   absent, ETS2_REVERSE_UPSTREAM selects a read-only upstream.
  patch-sources    Retarget the copied MSVC projects (v145/18.0 -> v143/17.0) and
                   rewrite own resource-path constants to the /home prefix.
  package          Copy the freshly built DLL into package/plugins/.
  guidance-assets  Generate endpoint and articulation line styles from verified
                   base assets, without introducing another mesh format.
  validate         Full PMD -> MAT -> TOBJ -> DDS closure check plus malformed
                   layout rejection self-tests.  Exits non-zero on any failure.

Nothing here touches the game install, the game home, the registry, saves or the
original download.  The upstream archive is opened read-only and never modified.
"""

import hashlib
import json
import os
import re
import shutil
import struct
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
HOME_PREFIX = "/home/ets2_reverse_posture_tmp"
PKG_HOME = os.path.join(ROOT, "package", "home", "ets2_reverse_posture_tmp")
PKG_ROOT = os.path.join(ROOT, "package")
REPORTS = os.path.join(ROOT, "reports")
SRC = os.path.join(ROOT, "src")

UPSTREAM = os.environ.get("ETS2_REVERSE_UPSTREAM", "")
UPSTREAM_SCS = os.path.join(UPSTREAM, "mod", "ETS2_Reverse_Posture_Assistant_1.61.scs") if UPSTREAM else ""
UPSTREAM_DLL = os.path.join(UPSTREAM, "runtime", "ETS2ReverseEntityRuntime.dll") if UPSTREAM else ""
SCS_SHA256 = "0CF31C7738F521D2E7E55678EFC73D4D4F8E5A6611EDA5438B61F7F08D00119D"
DLL_SHA256 = "47CC82562E9663D581B6E4234083FDBAA95AD4B8D44AD259CE00B71D013E7E09"

PMD_TAIL_OFF = 112
TOBJ_HEADER = 48
DDS_MAGIC = b"DDS "

PACKAGE_FILES = [
    "model/reverse_assist/accessory_anchor.pmd",
    "model/reverse_assist/accessory_anchor.pmg",
    "model/reverse_assist/sweep_edge.pmd",
    "model/reverse_assist/sweep_edge.pmg",
    "model/reverse_assist/sweep_edge_blue.pmd",
    "model/reverse_assist/sweep_edge_blue.pmg",
    "model/reverse_assist/tractor_box.pmd",
    "model/reverse_assist/tractor_box.pmg",
    "model/reverse_assist/trailer_box.pmd",
    "model/reverse_assist/trailer_box.pmg",
    "model/reverse_assist/guide_blue.tobj",
    "model/reverse_assist/guide_blue.dds",
    "model/reverse_assist/guide_color.tobj",
    "model/reverse_assist/guide_color.dds",
    "model/reverse_assist/guide_mask.tobj",
    "model/reverse_assist/guide_mask.dds",
    "automat/40/408526e9278658d1.mat",
    "automat/5c/5c442fa548af86ab.mat",
    "automat/7f/7fca5e15a5fe7bac.mat",
    "automat/f5/f57370b76733c5aa.mat",
]
BASE_PACKAGE_FILES = tuple(PACKAGE_FILES)
GUIDANCE_COLORS = {
    "preview": (255, 207, 154, 255),
    "straightening": (48, 222, 100, 255),
    "steady": (210, 220, 224, 255),
}
for _style in GUIDANCE_COLORS:
    PACKAGE_FILES.extend([
        "model/reverse_assist/guide_%s.pmd" % _style,
        "model/reverse_assist/guide_%s.pmg" % _style,
        "model/reverse_assist/guide_%s.tobj" % _style,
        "model/reverse_assist/guide_%s.dds" % _style,
        "automat/guidance/%s.mat" % _style,
    ])


class LayoutError(Exception):
    pass


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest().upper()


def sha256_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest().upper()


def u32(b, off):
    return struct.unpack_from("<I", b, off)[0]


def read_upstream_scs():
    got = sha256_file(UPSTREAM_SCS)
    if got != SCS_SHA256:
        raise LayoutError("upstream .scs hash changed: %s" % got)
    return zipfile.ZipFile(UPSTREAM_SCS)


def parse_pmd(blob):
    """Strict PMD model descriptor parser. Returns the material path."""
    if len(blob) < PMD_TAIL_OFF + 1:
        raise LayoutError("pmd too small: %d" % len(blob))
    if u32(blob, 0) != 4:
        raise LayoutError("unexpected pmd version %d" % u32(blob, 0))
    blob_size = u32(blob, 32)
    str_off = u32(blob, 60)
    str_ptr = u32(blob, 108)
    if str_off != PMD_TAIL_OFF or str_ptr != PMD_TAIL_OFF:
        raise LayoutError("unexpected string offset %d/%d" % (str_off, str_ptr))
    if u32(blob, 56) != 108:
        raise LayoutError("unexpected table offset %d" % u32(blob, 56))
    if str_off + blob_size != len(blob):
        raise LayoutError("tail blob size %d does not close file (%d)" % (blob_size, len(blob)))
    tail = blob[str_off:]
    if not tail.endswith(b"\x00") or b"\x00" in tail[:-1]:
        raise LayoutError("pmd tail is not one NUL-terminated string")
    try:
        path = tail[:-1].decode("ascii")
    except UnicodeDecodeError:
        raise LayoutError("pmd tail is not ascii")
    if not path.startswith("/"):
        raise LayoutError("pmd tail is not an absolute resource path: %r" % path)
    return path


def rewrite_pmd(blob, new_path):
    parse_pmd(blob)
    tail = new_path.encode("ascii") + b"\x00"
    out = bytearray(blob[:PMD_TAIL_OFF]) + bytearray(tail)
    struct.pack_into("<I", out, 32, len(tail))
    if u32(bytes(out), 32) != len(tail) or len(out) != PMD_TAIL_OFF + len(tail):
        raise LayoutError("pmd rewrite did not close")
    return bytes(out)


def parse_tobj(blob):
    if len(blob) < TOBJ_HEADER + 1:
        raise LayoutError("tobj too small: %d" % len(blob))
    if blob[:4] != bytes.fromhex("010ab170"):
        raise LayoutError("unexpected tobj magic %s" % blob[:4].hex())
    n = u32(blob, 40)
    if TOBJ_HEADER + n != len(blob):
        raise LayoutError("tobj length field %d does not close file (%d)" % (n, len(blob)))
    raw = blob[TOBJ_HEADER:]
    if b"\x00" in raw:
        raise LayoutError("tobj tail contains NUL")
    try:
        path = raw.decode("ascii")
    except UnicodeDecodeError:
        raise LayoutError("tobj tail is not ascii")
    if not path.startswith("/"):
        raise LayoutError("tobj tail is not an absolute resource path: %r" % path)
    return path


def rewrite_tobj(blob, new_path):
    parse_tobj(blob)
    tail = new_path.encode("ascii")
    out = bytearray(blob[:TOBJ_HEADER]) + bytearray(tail)
    struct.pack_into("<I", out, 40, len(tail))
    return bytes(out)


MAT_RE = re.compile(r'(\bsource\s*:\s*")([^"]+)(")')


def rewrite_mat(text, remap):
    def sub(m):
        return m.group(1) + remap(m.group(2)) + m.group(3)

    return MAT_RE.sub(sub, text)


def relocate(path):
    return HOME_PREFIX + path


def cmd_prepare():
    bundled = os.path.join(ROOT, "assets", "home", "ets2_reverse_posture_tmp")
    if os.path.isdir(bundled):
        for rel in BASE_PACKAGE_FILES:
            source = os.path.join(bundled, rel.replace("/", os.sep))
            target = os.path.join(PKG_HOME, rel.replace("/", os.sep))
            os.makedirs(os.path.dirname(target), exist_ok=True)
            shutil.copyfile(source, target)
        print("prepare: copied %d bundled base assets" % len(BASE_PACKAGE_FILES))
        return cmd_guidance_assets()
    if not UPSTREAM:
        raise LayoutError("Bundled assets missing; set ETS2_REVERSE_UPSTREAM to an upstream package")
    z = read_upstream_scs()
    names = set(z.namelist())
    missing = [p for p in PACKAGE_FILES if p not in names]
    if missing:
        raise LayoutError("upstream archive missing: %s" % missing)
    if any(n.endswith(".pmc") for n in names):
        raise LayoutError("upstream archive unexpectedly contains .pmc data")
    written = []
    for rel in BASE_PACKAGE_FILES:
        raw = z.read(rel)
        dst = os.path.join(PKG_HOME, rel.replace("/", os.sep))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if rel.endswith(".pmd"):
            new = rewrite_pmd(raw, relocate(parse_pmd(raw)))
            if parse_pmd(new) != relocate(parse_pmd(raw)):
                raise LayoutError("pmd relocation did not round-trip: %s" % rel)
        elif rel.endswith(".tobj"):
            new = rewrite_tobj(raw, relocate(parse_tobj(raw)))
            if parse_tobj(new) != relocate(parse_tobj(raw)):
                raise LayoutError("tobj relocation did not round-trip: %s" % rel)
        elif rel.endswith(".mat"):
            text = raw.decode("utf-8")
            out = rewrite_mat(text, relocate)
            stray = [r for _a, r, _b in MAT_RE.findall(out) if not r.startswith(HOME_PREFIX + "/")]
            if stray:
                raise LayoutError("mat still has unrelocated refs %s: %s" % (stray, rel))
            new = out.encode("utf-8")
        else:
            new = raw
        with open(dst, "wb") as f:
            f.write(new)
        written.append({"path": rel, "src_bytes": len(raw), "out_bytes": len(new),
                        "src_sha256": sha256_bytes(raw), "out_sha256": sha256_bytes(new)})
    os.makedirs(REPORTS, exist_ok=True)
    with open(os.path.join(REPORTS, "assets.json"), "w", encoding="utf-8") as f:
        json.dump({"home_prefix": HOME_PREFIX, "files": written}, f, indent=2)
    print("prepare: wrote %d files under %s" % (len(written), PKG_HOME))
    for w in written:
        print("  %-58s %5d -> %5d  %s" % (w["path"], w["src_bytes"], w["out_bytes"],
                                          w["out_sha256"][:12]))
    return cmd_guidance_assets()


def cmd_guidance_assets():
    model_dir = os.path.join(PKG_HOME, "model", "reverse_assist")
    mat_dir = os.path.join(PKG_HOME, "automat", "guidance")
    os.makedirs(mat_dir, exist_ok=True)
    pmd = open(os.path.join(model_dir, "sweep_edge.pmd"), "rb").read()
    tobj = open(os.path.join(model_dir, "guide_color.tobj"), "rb").read()
    dds = open(os.path.join(model_dir, "guide_color.dds"), "rb").read()
    if (len(dds) != 192 or dds[:4] != DDS_MAGIC or
            u32(dds, 12) != 4 or u32(dds, 16) != 4 or u32(dds, 88) != 32):
        raise LayoutError("guidance base DDS must be uncompressed 4x4 RGBA32")
    masks = struct.unpack_from("<4I", dds, 92)
    if set(masks) != {0xff, 0xff00, 0xff0000, 0xff000000}:
        raise LayoutError("guidance base DDS channel masks differ")
    for style, color in GUIDANCE_COLORS.items():
        basename = "guide_" + style
        mat_path = HOME_PREFIX + "/automat/guidance/" + style + ".mat"
        texture_path = HOME_PREFIX + "/model/reverse_assist/" + basename
        pixel = 0
        for component, mask in zip(color, masks):
            shift = (mask & -mask).bit_length() - 1
            pixel |= component << shift
        outputs = {
            os.path.join(model_dir, basename + ".pmd"): rewrite_pmd(pmd, mat_path),
            os.path.join(model_dir, basename + ".tobj"): rewrite_tobj(tobj, texture_path + ".dds"),
            os.path.join(model_dir, basename + ".dds"): dds[:128] + struct.pack("<I", pixel) * 16,
        }
        material = '''effect : "eut2.dif.lum.rfx" {
    additional_ambient : 1.000000
    diffuse : { 1.000000 , 1.000000 , 1.000000 }
    luminance_night : 1.000000
    luminance_output : 180.000000
    reflection : 0.000000
    shininess : 8.000000
    specular : { 0.000000 , 0.000000 , 0.000000 }
    texture : "texture_base" {
        source : "%s.tobj"
    }
}
''' % texture_path
        outputs[os.path.join(mat_dir, style + ".mat")] = material.encode("utf-8")
        for path, blob in outputs.items():
            with open(path, "wb") as stream:
                stream.write(blob)
        shutil.copyfile(os.path.join(model_dir, "sweep_edge.pmg"),
                        os.path.join(model_dir, basename + ".pmg"))
    print("guidance-assets: generated 3 styles / 15 files")
    return 0


TARGET_RE = re.compile(r'"(/model/reverse_assist/[A-Za-z0-9_./-]+)"')


def cmd_patch_sources():
    before = {}
    for base, _dirs, files in os.walk(SRC):
        for name in files:
            p = os.path.join(base, name)
            before[os.path.relpath(p, ROOT).replace(os.sep, "/")] = sha256_file(p)
    changed = 0
    touched = []
    for base, _dirs, files in os.walk(SRC):
        for name in files:
            p = os.path.join(base, name)
            if name.endswith(".vcxproj"):
                text = open(p, encoding="utf-8").read()
                new = (text.replace("<VCProjectVersion>18.0</VCProjectVersion>",
                                    "<VCProjectVersion>17.0</VCProjectVersion>")
                           .replace("<PlatformToolset>v145</PlatformToolset>",
                                    "<PlatformToolset>v143</PlatformToolset>"))
                if new != text:
                    open(p, "w", encoding="utf-8", newline="").write(new)
                    touched.append((os.path.relpath(p, ROOT), "toolset retarget v145->v143"))
                    changed += 1
                continue
            if not name.endswith((".cpp", ".hpp", ".def")):
                continue
            text = open(p, encoding="utf-8", newline="").read()
            hits = sorted({m.group(1) for m in TARGET_RE.finditer(text)})
            if not hits:
                continue
            new = TARGET_RE.sub(lambda m: '"%s"' % relocate(m.group(1)), text)
            open(p, "w", encoding="utf-8", newline="").write(new)
            for h in hits:
                touched.append((os.path.relpath(p, ROOT), "%s -> %s" % (h, relocate(h))))
            changed += 1
    print("patch-sources: %d file(s) modified" % changed)
    for rel, what in touched:
        print("  %-46s %s" % (rel, what))
    # invariants that must survive the patch
    after = {}
    for base, _dirs, files in os.walk(SRC):
        for name in files:
            p = os.path.join(base, name)
            after[os.path.relpath(p, ROOT).replace(os.sep, "/")] = sha256_file(p)
    modified = sorted(k for k in before if before[k] != after.get(k))
    allowed = {"src/entity_runtime/ETS2ReverseEntityRuntime.vcxproj",
               "src/entity_runtime/BuildProfileTests.vcxproj",
               "src/entity_runtime/native_entity_runtime.cpp"}
    cpp_rel = "src/entity_runtime/native_entity_runtime.cpp"
    cpp = open(os.path.join(SRC, "entity_runtime", "native_entity_runtime.cpp"),
               encoding="utf-8", errors="replace").read()
    profiles = open(os.path.join(SRC, "entity_runtime", "build_profiles.hpp"),
                    encoding="utf-8", errors="replace").read()
    invariants = {
        "only_allowed_files_modified": set(modified) <= allowed,
        "cpp_path_literals_relocated": '"/model/reverse_assist/' not in cpp,
        "hook_signature_profiles_untouched": cpp_rel not in modified and
                                             before["src/entity_runtime/build_profiles.hpp"] ==
                                             after["src/entity_runtime/build_profiles.hpp"],
        "compat_gate_preserved": "no BuildProfile matched sha256=" in cpp and
                                 "enabled-hook-signatures" in cpp,
        "telemetry_fallback_preserved": "Compatibility disabled" in cpp,
    }
    print("patch-sources: modified files (hash-diff): %s" % (modified or "none"))
    for k, v in invariants.items():
        print("  invariant %-24s %s" % (k, "OK" if v else "FAILED"))
    if not all(invariants.values()):
        raise LayoutError("source patch broke a required invariant")
    print("patch-sources: build_profiles.hpp digest = %s (unchanged)" %
          before["src/entity_runtime/build_profiles.hpp"][:16])
    return 0


def cmd_package():
    src = os.path.join(ROOT, "build", "entity_runtime", "ETS2ReverseEntityRuntime.dll")
    if not os.path.exists(src):
        raise LayoutError("built DLL not found: %s" % src)
    dst_dir = os.path.join(PKG_ROOT, "plugins")
    os.makedirs(dst_dir, exist_ok=True)
    dst = os.path.join(dst_dir, "ETS2ReverseEntityRuntime.dll")
    shutil.copyfile(src, dst)
    ini = os.path.join(dst_dir, "ETS2ReverseEntityRuntime.ini")
    if not os.path.exists(ini):
        with open(ini, "w", encoding="ascii") as stream:
            stream.write("[guidance]\nenabled=1\nendpoint_outline=1\nfront_swing=1\n"
                         "articulation_indicator=1\nprediction_distance_cm=500\n"
                         "\n[diagnostics]\nenabled=0\n")
    manifest = {
        "schema_version": 1,
        "version": open(os.path.join(ROOT, "VERSION"), encoding="ascii").read().strip(),
        "dll_sha256": sha256_file(dst),
        "dll_size": os.path.getsize(dst),
        "ini_sha256": sha256_file(ini),
        "assets": [{"path": rel, "sha256": sha256_file(os.path.join(PKG_HOME, rel)),
                    "size": os.path.getsize(os.path.join(PKG_HOME, rel))}
                   for rel in PACKAGE_FILES],
    }
    with open(os.path.join(PKG_ROOT, "manifest.json"), "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2)
    print("package: %s  %d bytes  sha256=%s" % (dst, os.path.getsize(dst), sha256_file(dst)))
    print("package: manifest includes %d assets and the startup INI" % len(manifest["assets"]))
    return 0


def resolve_package(path):
    """Map an absolute /home/... resource path to a file inside package/."""
    if not path.startswith("/"):
        raise LayoutError("not absolute: %r" % path)
    return os.path.join(PKG_ROOT, path.lstrip("/").replace("/", os.sep))


def cmd_validate():
    failures = []
    seen = {"pmd": 0, "mat": 0, "tobj": 0, "dds": 0}
    mats_used = set()
    tobjs_used = set()
    for rel in PACKAGE_FILES:
        p = os.path.join(PKG_HOME, rel.replace("/", os.sep))
        if not os.path.exists(p):
            failures.append("missing package file %s" % rel)
    for rel in PACKAGE_FILES:
        if not rel.endswith(".pmd"):
            continue
        seen["pmd"] += 1
        blob = open(os.path.join(PKG_HOME, rel.replace("/", os.sep)), "rb").read()
        try:
            mat_path = parse_pmd(blob)
        except LayoutError as e:
            failures.append("%s: %s" % (rel, e))
            continue
        if not mat_path.startswith(HOME_PREFIX + "/automat/"):
            failures.append("%s: mat path not relocated: %s" % (rel, mat_path))
            continue
        mat_file = resolve_package(mat_path)
        if not os.path.exists(mat_file):
            failures.append("%s: unresolved mat %s" % (rel, mat_path))
            continue
        mats_used.add(mat_path)
        text = open(mat_file, encoding="utf-8").read()
        refs = MAT_RE.findall(text)
        if not refs and "eut2.none.rfx" not in text:
            failures.append("%s: no texture reference (non-\"none\" effect)" % mat_path)
        for _pre, ref, _post in refs:
            if not ref.startswith(HOME_PREFIX + "/"):
                failures.append("%s: tobj ref not relocated: %s" % (mat_path, ref))
                continue
            tobj_file = resolve_package(ref)
            if not os.path.exists(tobj_file):
                failures.append("%s: unresolved tobj %s" % (mat_path, ref))
                continue
            tobjs_used.add(ref)
            tobj_blob = open(tobj_file, "rb").read()
            try:
                dds_path = parse_tobj(tobj_blob)
            except LayoutError as e:
                failures.append("%s: %s" % (ref, e))
                continue
            if not dds_path.startswith(HOME_PREFIX + "/"):
                failures.append("%s: dds path not relocated: %s" % (ref, dds_path))
                continue
            dds_file = resolve_package(dds_path)
            if not os.path.exists(dds_file):
                failures.append("%s: unresolved dds %s" % (ref, dds_path))
                continue
            if open(dds_file, "rb").read(4) != DDS_MAGIC:
                failures.append("%s: not a DDS file" % dds_path)
                continue
            seen["dds"] += 1
    seen["mat"] = len(mats_used)
    seen["tobj"] = len(tobjs_used)

    # every relocated mat/tobj in the package must be reachable from a pmd/mat
    orphans = []
    for base, _dirs, files in os.walk(PKG_HOME):
        for name in files:
            p = os.path.join(base, name)
            rel = "/" + os.path.relpath(p, PKG_ROOT).replace(os.sep, "/")
            if name.endswith(".mat") and rel not in mats_used:
                orphans.append("mat %s" % rel)
            if name.endswith(".tobj") and rel not in tobjs_used:
                orphans.append("tobj %s" % rel)
    print("validate: unreferenced-but-present upstream assets: %s" % (orphans or "none"))

    # scope checks
    strays = []
    for base, _dirs, files in os.walk(PKG_HOME):
        for name in files:
            if name.endswith((".pmc", ".pma", ".pms")):
                strays.append(os.path.join(base, name))
    if strays:
        failures.append("unexpected model cache/extra files: %s" % strays)
    for forbidden in ("def", "ui", "material"):
        if os.path.exists(os.path.join(PKG_HOME, forbidden)):
            failures.append("unexpected %s/ tree in package" % forbidden)

    # malformed layout rejection self-tests
    good_pmd = open(os.path.join(PKG_HOME, "model", "reverse_assist", "accessory_anchor.pmd"), "rb").read()
    good_tobj = open(os.path.join(PKG_HOME, "model", "reverse_assist", "guide_blue.tobj"), "rb").read()
    malformed = []
    b = bytearray(good_pmd); struct.pack_into("<I", b, 0, 5); malformed.append(("pmd wrong version", parse_pmd, bytes(b)))
    b = bytearray(good_pmd); struct.pack_into("<I", b, 60, 120); malformed.append(("pmd shifted string offset", parse_pmd, bytes(b)))
    b = bytearray(good_pmd); struct.pack_into("<I", b, 32, 40); malformed.append(("pmd tail size mismatch", parse_pmd, bytes(b)))
    malformed.append(("pmd truncated", parse_pmd, good_pmd[:100]))
    b = bytearray(good_pmd); b[-1] = 0x41; malformed.append(("pmd unterminated tail", parse_pmd, bytes(b)))
    b = bytearray(good_tobj); struct.pack_into("<I", b, 40, 20); malformed.append(("tobj length mismatch", parse_tobj, bytes(b)))
    b = bytearray(good_tobj); b[0] = 0x02; malformed.append(("tobj bad magic", parse_tobj, bytes(b)))
    malformed.append(("tobj truncated", parse_tobj, good_tobj[:50]))
    rejected = 0
    for label, fn, data in malformed:
        try:
            fn(data)
            failures.append("malformed input accepted: %s" % label)
        except LayoutError:
            rejected += 1
    for label, fn, data in [("pmd", parse_pmd, good_pmd), ("tobj", parse_tobj, good_tobj)]:
        try:
            fn(data)
        except LayoutError as e:
            failures.append("valid %s rejected: %s" % (label, e))

    # original artefacts must be untouched
    orig = {}
    for label, path, want in (("upstream .scs", UPSTREAM_SCS, SCS_SHA256),
                              ("upstream .dll", UPSTREAM_DLL, DLL_SHA256)):
        if os.path.exists(path):
            got = sha256_file(path)
            orig[label] = got
            if got != want:
                failures.append("%s hash changed: %s" % (label, got))
    print("validate: closure pmd=%d mat=%d tobj=%d dds=%d" %
          (seen["pmd"], seen["mat"], seen["tobj"], seen["dds"]))
    print("validate: malformed layouts rejected %d/%d" % (rejected, len(malformed)))
    for k, v in orig.items():
        print("validate: %s unchanged sha256=%s" % (k, v[:16]))
    report = {"closure": seen, "unreferenced_present": orphans, "malformed_rejected": rejected,
              "malformed_total": len(malformed), "originals": orig, "failures": failures}
    os.makedirs(REPORTS, exist_ok=True)
    with open(os.path.join(REPORTS, "validation.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    if failures:
        print("validate: FAIL")
        for f_ in failures:
            print("  - %s" % f_)
        return 1
    print("validate: PASS")
    return 0


COMMANDS = {"prepare": cmd_prepare, "patch-sources": cmd_patch_sources,
            "package": cmd_package, "validate": cmd_validate,
            "guidance-assets": cmd_guidance_assets}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in COMMANDS:
        print(__doc__)
        return 2
    try:
        return COMMANDS[sys.argv[1]]()
    except LayoutError as e:
        print("ABORT (layout): %s" % e)
        return 3


if __name__ == "__main__":
    sys.exit(main())
