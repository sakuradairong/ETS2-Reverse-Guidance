#!/usr/bin/env python3
import hashlib
import json
import math
import os
import shutil
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE_HOME = os.path.join(ROOT, "assets", "home", "ets2_reverse_posture_tmp")
OUT_ROOT = os.path.join(ROOT, "assets_improved")
OUT_HOME = os.path.join(OUT_ROOT, "home", "ets2_reverse_posture_tmp")
HOME_PREFIX = "/home/ets2_reverse_posture_tmp"

TEX_SIZE = 64
DDS_MAGIC = b"DDS "
DDS_OFF_HEIGHT = 12
DDS_OFF_WIDTH = 16
DDS_OFF_PITCH = 20
DDS_OFF_MIPMAPS = 28
DDS_OFF_MASKS = 92
PMD_TAIL_OFF = 112
TOBJ_HEADER = 48

STYLES = [
    ("guide_blue", "automat/7f/7fca5e15a5fe7bac.mat", (72, 168, 255), (20, 86, 214), 1.60, 270.0),
    ("guide_color", "automat/f5/f57370b76733c5aa.mat", (255, 164, 52), (224, 88, 12), 1.65, 285.0),
    ("guide_preview", "automat/guidance/preview.mat", (255, 224, 178), (226, 152, 82), 1.50, 240.0),
    ("guide_straightening", "automat/guidance/straightening.mat", (96, 244, 140), (26, 178, 74), 1.65, 280.0),
    ("guide_steady", "automat/guidance/steady.mat", (232, 240, 244), (146, 162, 172), 1.45, 230.0),
    ("guide_target", "automat/guidance/target.mat", (72, 246, 252), (0, 176, 194), 1.70, 295.0),
]


def sha256_bytes(blob: bytes) -> str:
    return hashlib.sha256(blob).hexdigest().upper()


def rewrite_pmd(blob: bytes, mat_path: str) -> bytes:
    tail = mat_path.encode("ascii") + b"\x00"
    out = bytearray(blob[:PMD_TAIL_OFF]) + bytearray(tail)
    struct.pack_into("<I", out, 32, len(tail))
    return bytes(out)


def rewrite_tobj(blob: bytes, dds_path: str) -> bytes:
    tail = dds_path.encode("ascii")
    out = bytearray(blob[:TOBJ_HEADER]) + bytearray(tail)
    struct.pack_into("<I", out, 40, len(tail))
    return bytes(out)


def make_semantic_dds(template_header: bytes, core_rgb, edge_rgb) -> bytes:
    if len(template_header) < 128 or template_header[:4] != DDS_MAGIC:
        raise ValueError("Invalid DDS template header")
    header = bytearray(template_header[:128])
    struct.pack_into("<I", header, DDS_OFF_HEIGHT, TEX_SIZE)
    struct.pack_into("<I", header, DDS_OFF_WIDTH, TEX_SIZE)
    struct.pack_into("<I", header, DDS_OFF_PITCH, TEX_SIZE * 4)
    struct.pack_into("<I", header, DDS_OFF_MIPMAPS, 0)
    masks = struct.unpack_from("<4I", header, DDS_OFF_MASKS)
    r_shift = (masks[0] & -masks[0]).bit_length() - 1
    g_shift = (masks[1] & -masks[1]).bit_length() - 1
    b_shift = (masks[2] & -masks[2]).bit_length() - 1
    a_shift = (masks[3] & -masks[3]).bit_length() - 1

    pixels = bytearray(TEX_SIZE * TEX_SIZE * 4)
    offset = 0
    for y in range(TEX_SIZE):
        v = (y + 0.5) / TEX_SIZE * 2.0 - 1.0
        for x in range(TEX_SIZE):
            u = (x + 0.5) / TEX_SIZE * 2.0 - 1.0
            dist = math.sqrt((u * u) * 0.65 + (v * v) * 0.65)
            core_w = max(0.0, min(1.0, 1.0 - max(0.0, (dist - 0.30) / 0.55)))
            core_w = core_w * core_w * (3.0 - 2.0 * core_w)
            edge_dist = max(abs(u), abs(v))
            rim_fade = max(0.72, min(1.0, 1.0 - max(0.0, (edge_dist - 0.78) / 0.22) * 0.28))

            r = max(0, min(255, int(round((edge_rgb[0] + (core_rgb[0] - edge_rgb[0]) * core_w) * rim_fade))))
            g = max(0, min(255, int(round((edge_rgb[1] + (core_rgb[1] - edge_rgb[1]) * core_w) * rim_fade))))
            b = max(0, min(255, int(round((edge_rgb[2] + (core_rgb[2] - edge_rgb[2]) * core_w) * rim_fade))))
            a = max(0, min(255, int(round(255 * max(0.84, min(1.0, 1.0 - max(0.0, (edge_dist - 0.85) / 0.15) * 0.16))))))

            px = (r << r_shift) | (g << g_shift) | (b << b_shift) | (a << a_shift)
            struct.pack_into("<I", pixels, offset, px)
            offset += 4
    return bytes(header) + bytes(pixels)


def make_unified_mat(tobj_virtual_path: str, lum_night: float, lum_output: float) -> bytes:
    text = (
        'effect : "eut2.dif.lum.rfx" {\n'
        '    additional_ambient : 1.000000\n'
        '    diffuse : { 1.000000 , 1.000000 , 1.000000 }\n'
        f'    luminance_night : {lum_night:.6f}\n'
        f'    luminance_output : {lum_output:.6f}\n'
        '    reflection : 0.000000\n'
        '    shininess : 8.000000\n'
        '    specular : { 0.000000 , 0.000000 , 0.000000 }\n'
        '    texture : "texture_base" {\n'
        f'        source : "{tobj_virtual_path}"\n'
        '    }\n'
        '}\n'
    )
    return text.encode("utf-8")


def main() -> int:
    model_src = os.path.join(BASE_HOME, "model", "reverse_assist")
    template_dds = open(os.path.join(model_src, "guide_color.dds"), "rb").read()
    template_pmd = open(os.path.join(model_src, "sweep_edge.pmd"), "rb").read()
    template_pmg = open(os.path.join(model_src, "sweep_edge.pmg"), "rb").read()
    template_tobj = open(os.path.join(model_src, "guide_color.tobj"), "rb").read()

    if os.path.exists(OUT_HOME):
        shutil.rmtree(OUT_HOME)
    os.makedirs(OUT_HOME, exist_ok=True)

    manifest_files = []

    def write_file(rel_path: str, data: bytes):
        dst = os.path.join(OUT_HOME, rel_path.replace("/", os.sep))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
        manifest_files.append({
            "path": rel_path,
            "bytes": len(data),
            "sha256": sha256_bytes(data),
        })

    for dds_name, mat_rel, core_rgb, edge_rgb, lum_night, lum_output in STYLES:
        dds_rel = f"model/reverse_assist/{dds_name}.dds"
        tobj_virtual = f"{HOME_PREFIX}/model/reverse_assist/{dds_name}.tobj"
        write_file(dds_rel, make_semantic_dds(template_dds, core_rgb, edge_rgb))
        write_file(mat_rel, make_unified_mat(tobj_virtual, lum_night, lum_output))

        if dds_name == "guide_target":
            write_file("model/reverse_assist/guide_target.pmd",
                       rewrite_pmd(template_pmd, f"{HOME_PREFIX}/{mat_rel}"))
            write_file("model/reverse_assist/guide_target.pmg", template_pmg)
            write_file("model/reverse_assist/guide_target.tobj",
                       rewrite_tobj(template_tobj, f"{HOME_PREFIX}/{dds_rel}"))

    manifest = {
        "schema_version": 1,
        "pack": "assets_improved",
        "texture_resolution": f"{TEX_SIZE}x{TEX_SIZE} BGRA8",
        "mesh_compatible": "0.075x0.055x0.360 m PMG unchanged",
        "files": sorted(manifest_files, key=lambda x: x["path"]),
    }
    manifest_path = os.path.join(OUT_ROOT, "manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    print(f"build_visual_assets: generated {len(manifest_files)} files in assets_improved/ ({TEX_SIZE}x{TEX_SIZE} BGRA8)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
