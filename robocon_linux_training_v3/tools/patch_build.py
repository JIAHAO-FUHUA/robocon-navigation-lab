"""Apply only audited build fixes to the pinned FASTLIO2_ROS2 files.

Unknown file contents are rejected before writing any source file.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHANGES = {
    'fastlio2/CMakeLists.txt': (
        'target_link_libraries(lio_node\n  yaml-cpp',
        'target_link_libraries(lio_node\n  Sophus::Sophus\n  yaml-cpp'),
    'localizer/CMakeLists.txt': (
        'find_package(ament_cmake REQUIRED)',
        'find_package(ament_cmake REQUIRED)\nfind_package(rclcpp REQUIRED)'),
    'localizer/package.xml': (
        '  <depend>std_msgs</depend>',
        '  <depend>rclcpp</depend>\n  <depend>std_msgs</depend>'),
}


def sha(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def apply(source, report):
    versions = json.loads((ROOT / 'config/source_versions.json').read_text(encoding='utf-8'))
    staged, diffs = [], []
    for relative, (before, after) in CHANGES.items():
        path = source / relative
        text = path.read_text(encoding='utf-8')
        expected = versions['upstream_build_file_sha256'][relative]
        if sha(text) == expected:
            if text.count(before) != 1:
                raise RuntimeError(f'Unexpected replacement count: {relative}')
            modified = text.replace(before, after, 1)
            staged.append((path, modified))
            diffs.extend(difflib.unified_diff(text.splitlines(True), modified.splitlines(True),
                         fromfile='a/' + relative, tofile='b/' + relative))
        elif text.count(after) == 1 and sha(text.replace(after, before, 1)) == expected:
            continue
        else:
            raise RuntimeError(f'Unrecognized changes in {relative}; preserve and review local modifications.')
    if diffs:
        report.parent.mkdir(parents=True, exist_ok=True)
        # Keep prior patch evidence. A different report path is needed if it differs.
        patch = ''.join(diffs)
        if report.exists() and report.read_text(encoding='utf-8') != patch:
            raise RuntimeError('Report already exists with different contents; choose a new --report path.')
        for path, modified in staged:
            path.write_text(modified, encoding='utf-8', newline='\n')
        report.write_text(patch, encoding='utf-8', newline='\n')
    return len(staged)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    count = apply(args.source.resolve(), args.report.resolve())
    print(f'Build compatibility files changed: {count}. ICP/localization algorithms were not modified.')


if __name__ == '__main__':
    main()
