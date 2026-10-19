#!/usr/bin/env python3
"""校验代码里声明的 ROS 参数与 YAML 配置键是否一致。"""

import argparse
import pathlib
import re
import sys

STRUCTURAL_KEYS = {"ros__parameters"}


def declared_params(src_dir: pathlib.Path):
    found = {}
    dynamic = []
    literal = re.compile(r'declare_parameter\s*\(\s*"([^"]+)"')
    any_call = re.compile(r"declare_parameter\s*\(")
    for path in sorted(src_dir.rglob("*.cpp")) + sorted(src_dir.rglob("*.hpp")):
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            names = literal.findall(line)
            for name in names:
                found.setdefault(name, "%s:%d" % (path.name, lineno))
            if not names and any_call.search(line) and not line.strip().startswith("//"):
                dynamic.append("%s:%d: %s" % (path.name, lineno, line.strip()))
    return found, dynamic


def yaml_keys(yaml_files):
    keys = {}
    for path in yaml_files:
        for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if not line[:1].isspace():
                continue
            match = re.match(r"([A-Za-z_][A-Za-z0-9_]*)\s*:", stripped)
            if not match:
                continue
            name = match.group(1)
            if name in STRUCTURAL_KEYS:
                continue
            keys.setdefault(name, "%s:%d" % (path.name, lineno))
    return keys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", default="src")
    parser.add_argument("--yaml", nargs="+", default=None)
    parser.add_argument("--allow", action="append", default=["config_path"])
    parser.add_argument("--allow-prefix", action="append", default=[])
    args = parser.parse_args()

    src_dir = pathlib.Path(args.src)
    if not src_dir.is_dir():
        print("[FAIL] 源码目录不存在: %s" % src_dir)
        return 1

    if args.yaml:
        yaml_files = [pathlib.Path(p) for p in args.yaml]
    else:
        yaml_files = sorted((src_dir.parent / "config").glob("*.yaml"))
    if not yaml_files:
        print("[FAIL] 没找到 yaml 配置")
        return 1
    absent = [p for p in yaml_files if not p.is_file()]
    if absent:
        print("[FAIL] yaml 文件不存在: %s" % ", ".join(str(p) for p in absent))
        return 1

    declared, dynamic = declared_params(src_dir)
    configured = yaml_keys(yaml_files)
    allowed = set(args.allow)
    allowed_prefixes = tuple(args.allow_prefix)

    missing = [n for n in declared if n not in configured and n not in allowed]
    extra = [
        n
        for n in configured
        if n not in declared and not (allowed_prefixes and n.startswith(allowed_prefixes))
    ]

    print("源码: %s" % src_dir)
    print("配置: %s" % ", ".join(str(p) for p in yaml_files))
    print("代码声明参数 %d 个; yaml 键 %d 个; 白名单 %d 个" % (len(declared), len(configured), len(allowed)))
    if dynamic:
        print("注意: 检测到 %d 处非字面量 declare_parameter" % len(dynamic))
    if missing:
        print("\n[缺失] 代码里声明了但 yaml 里没有:")
        for name in missing:
            print("    %-34s (声明于 %s)" % (name, declared[name]))
    if extra:
        print("\n[多余] yaml 里有但代码没声明:")
        for name in extra:
            print("    %-34s (出现于 %s)" % (name, configured[name]))
    if missing or extra:
        print("\n[FAIL] 参数与 yaml 不一致: 缺失 %d 个, 多余 %d 个" % (len(missing), len(extra)))
        return 1

    print("\n[OK] 参数与 yaml 一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
