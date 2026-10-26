#!/usr/bin/env python3
"""Install an already built SDK into a new directory and make a checksummed archive."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New install directory (must not exist)')
    parser.add_argument('--cmake', default='cmake')
    args = parser.parse_args()
    build, output = args.build.resolve(), args.output.resolve()
    repo = Path(__file__).resolve().parents[1]
    if not (build / 'CMakeCache.txt').is_file():
        parser.error('build must be a configured CMake directory')
    cache = (build / 'CMakeCache.txt').read_text()
    if f'CMAKE_HOME_DIRECTORY:INTERNAL={repo}\n' not in cache:
        parser.error('build directory belongs to a different source tree')
    archive = output.with_name(output.name + '.tar.gz')
    if output.exists() or archive.exists():
        parser.error('output directory or archive already exists; choose a new path')
    output.mkdir(parents=True)
    subprocess.run([args.cmake, '--install', str(build), '--prefix', str(output)], check=True)
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip()
    dirty = subprocess.check_output(['git', 'diff', 'HEAD', '--stat'], cwd=repo, text=True).strip()
    (output / 'package-info.json').write_text(json.dumps({
        'source_commit_at_packaging': commit,
        'tracked_changes_at_packaging': dirty,
        'packaging_host': platform.platform(),
        'qualification': 'integration build; see sdk-build-info.txt for target; board acceptance pending',
        'runtime_dependencies': 'OpenCV shared libraries; RKNN-enabled build also requires compatible librknnrt and NPU driver',
    }, indent=2) + '\n')
    hashes = []
    for path in sorted(output.rglob('*')):
        if path.is_file():
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            hashes.append(f'{digest}  {path.relative_to(output).as_posix()}')
    (output / 'SHA256SUMS').write_text('\n'.join(hashes) + '\n')
    shutil.make_archive(str(output), 'gztar', root_dir=output.parent, base_dir=output.name)
    archive_hash = hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_name(archive.name + '.sha256').write_text(f'{archive_hash}  {archive.name}\n')
    print(archive)


if __name__ == '__main__':
    main()
