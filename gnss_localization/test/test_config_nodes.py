#!/usr/bin/env python3
"""检查参数文件顶层节点名与 launch 节点名一致。"""

import pathlib
import sys


EXPECTED_NODES = {
    "gnss_driver.yaml": "gnss_driver",
    "ntrip_client.yaml": "ntrip_client",
    "sdk_client.yaml": "sdk_client",
}


def top_level_keys(path):
    keys = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or line[0].isspace():
            continue
        if stripped.endswith(":"):
            keys.append(stripped[:-1])
    return keys


def main():
    config_dir = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "config")
    failed = False
    for filename, expected in EXPECTED_NODES.items():
        path = config_dir / filename
        keys = top_level_keys(path)
        if keys != [expected]:
            print(
                "[FAIL] {} 顶层节点应为 [{}]，实际为 {}".format(
                    path, expected, keys
                )
            )
            failed = True
        else:
            print("[OK] {} -> {}".format(path, expected))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
