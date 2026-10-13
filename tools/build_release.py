"""Create a checked Windows release archive from the locally built package."""
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parent.parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main():
    version = (ROOT / "VERSION").read_text().strip()
    package = ROOT / "package"
    manifest = json.loads((package / "manifest.json").read_text())
    if manifest["version"] != version or manifest["schema_version"] != 1:
        raise ValueError("Package manifest/version mismatch")
    checks = [(package / "plugins/ETS2ReverseEntityRuntime.dll", manifest["dll_sha256"]),
              (package / "plugins/ETS2ReverseEntityRuntime.ini", manifest["ini_sha256"])]
    for asset in manifest["assets"]:
        rel = Path(asset["path"])
        if rel.is_absolute() or ".." in rel.parts:
            raise ValueError("Invalid asset path")
        checks.append((package / "home/ets2_reverse_posture_tmp" / rel, asset["sha256"]))
    for path, expected in checks:
        if digest(path) != expected:
            raise ValueError(f"Package hash mismatch: {path.name}")
    dist = ROOT / "dist"
    name = f"ETS2-Reverse-Guidance-v{version}-windows-x64"
    stage = dist / name
    if stage.exists():
        raise ValueError("Release staging directory already exists; use a fresh build directory")
    stage.mkdir(parents=True)
    for directory in ("plugins", "home"):
        shutil.copytree(package / directory, stage / directory)
    for file in ("README.md", "LICENSE", "THIRD_PARTY_NOTICES.md", "VERSION"):
        shutil.copy2(ROOT / file, stage / file)
    shutil.copy2(package / "manifest.json", stage / "manifest.json")
    licenses = stage / "licenses"
    licenses.mkdir()
    shutil.copy2(ROOT / "third_party/scs-sdk-plugin/LICENSE", licenses / "SCS-SDK-LICENSE.txt")
    shutil.copy2(ROOT / "third_party/reshade/deps/minhook/LICENSE.txt", licenses / "MinHook-LICENSE.txt")
    archive = dist / (name + ".zip")
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as output:
        for path in sorted(stage.rglob("*")):
            if path.is_file():
                output.write(path, path.relative_to(dist).as_posix())
    with zipfile.ZipFile(archive) as output:
        if output.testzip() is not None:
            raise ValueError("Release archive CRC verification failed")
        for path, expected in checks:
            entry = f"{name}/{path.relative_to(package).as_posix()}"
            if hashlib.sha256(output.read(entry)).hexdigest().upper() != expected:
                raise ValueError(f"Archived file hash mismatch: {entry}")
    checksum = dist / "SHA256SUMS.txt"
    checksum.write_text(f"{digest(archive)}  {archive.name}\n", encoding="ascii")
    print(f"Release: {archive.name}; package files verified={len(checks)}; bytes={archive.stat().st_size}")
    print(f"SHA256: {digest(archive)}")


if __name__ == "__main__":
    main()
