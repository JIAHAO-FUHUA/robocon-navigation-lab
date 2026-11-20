"""Hash-guarded IMU unit patch for the pinned FASTLIO2_ROS2 source."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHANGES = [
    ('    bool print_time_cost = false;', '    bool print_time_cost = false;\n    double imu_acc_scale = 9.80665;'),
    ('        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();',
     '        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();\n'
     '        m_node_config.imu_acc_scale = config["imu_acc_scale"] ? config["imu_acc_scale"].as<double>() : 9.80665;'),
    ('msg->linear_acceleration.z) * 10.0,', 'msg->linear_acceleration.z) * m_node_config.imu_acc_scale,'),
]


def apply(source, report):
    path = source/'fastlio2/src/lio_node.cpp'
    original = path.read_text(encoding='utf-8')
    sha = lambda text: hashlib.sha256(text.encode()).hexdigest()
    expected = json.loads((ROOT/'config/source_versions.json').read_text(encoding='utf-8'))['runtime_patch_original_sha256']
    reverted = original
    for before, after in reversed(CHANGES):
        reverted = reverted.replace(after, before, 1)
    if sha(original) != expected:
        if sha(reverted) == expected and all(original.count(after) == 1 for _, after in CHANGES):
            return False
        raise ValueError('Unknown lio_node.cpp; preserve local changes and review the diff')
    updated = original
    for before, after in CHANGES:
        if updated.count(before) != 1:
            raise ValueError('Unexpected replacement count')
        updated = updated.replace(before, after, 1)
    diff = ''.join(difflib.unified_diff(original.splitlines(True), updated.splitlines(True),
                                     fromfile='a/fastlio2/src/lio_node.cpp', tofile='b/fastlio2/src/lio_node.cpp'))
    if report.exists() and report.read_text(encoding='utf-8') != diff:
        raise ValueError('Report exists with different contents; choose another report path')
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(diff, encoding='utf-8', newline='\n')
    path.write_text(updated, encoding='utf-8', newline='\n')
    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    print('IMU scale source changed:', apply(args.source, args.report))
