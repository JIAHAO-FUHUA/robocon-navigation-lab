"""Task-local patch after pinned source/build fixes. Preserve diff in /work/logs."""
from pathlib import Path
import difflib

root=Path('/work/jazzy_ws/src/FASTLIO2_ROS2')
path=root/'localizer/src/localizer_node.cpp'
original=path.read_text()
if 'club_health_pub' in original:
    print('Health patch already applied');raise SystemExit(0)
text=original.replace('#include <queue>','#include <queue>\n#include <std_msgs/msg/string.hpp>\n#include <sstream>\n#include <cmath>')
text=text.replace('        m_map_cloud_pub =',
    '        club_health_pub = this->create_publisher<std_msgs::msg::String>("/club/localization_health", 10);\n        m_map_cloud_pub =')
start=text.index('    void timerCB()')
end=text.index('    void syncCB(',start)
text=text[:start]+r'''
    uint64_t club_request_id = 0;
    bool club_map_loaded = false;
    double club_last_attempt = -1.0;
    double club_last_success = -1.0;
    double stampSec(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }
    void clubHealth(bool ok, double scan_stamp)
    {
        std_msgs::msg::String msg;
        std::ostringstream out;
        Eigen::Quaterniond q(m_state.last_offset_r);
        const auto &t = m_state.last_offset_t;
        auto score = [](double v) { return std::isfinite(v) ? std::to_string(v) : std::string("null"); };
        out.precision(15);
        out << "{\"request_id\":" << club_request_id
            << ",\"match_ok\":" << (ok ? "true" : "false")
            << ",\"scan_stamp\":" << scan_stamp
            << ",\"last_success_stamp\":" << club_last_success
            << ",\"rough_fitness\":" << score(m_localizer->roughFitness())
            << ",\"refine_fitness\":" << score(m_localizer->refineFitness())
            << ",\"offset_translation\":[" << t.x() << "," << t.y() << "," << t.z() << "]"
            << ",\"offset_quaternion\":[" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "]}";
        msg.data=out.str(); club_health_pub->publish(msg);
    }
    void timerCB()
    {
        if (!m_state.message_received || !club_map_loaded) return;
        const double scan_stamp = stampSec(m_state.last_message_time);
        // Each attempt needs a genuinely new scan; simulation pause cannot renew health.
        if (scan_stamp <= club_last_attempt ||
            (club_last_attempt >= 0 && scan_stamp-club_last_attempt < 1.0/m_config.update_hz)) return;
        club_last_attempt = scan_stamp;
        M4F guess=M4F::Identity();
        if (m_state.service_received) guess=m_state.initial_guess;
        else {
            guess.block<3,3>(0,0)=(m_state.last_offset_r*m_state.last_r).cast<float>();
            guess.block<3,1>(0,3)=(m_state.last_offset_r*m_state.last_t+m_state.last_offset_t).cast<float>();
        }
        m_localizer->setInput(m_state.last_cloud);
        const bool result=m_localizer->align(guess);
        m_state.localize_success=result;
        if (result) {
            M3D r=guess.block<3,3>(0,0).cast<double>();
            V3D t=guess.block<3,1>(0,3).cast<double>();
            m_state.last_offset_r=r*m_state.last_r.transpose();
            m_state.last_offset_t=t-m_state.last_offset_r*m_state.last_t;
            m_state.service_received=false;
            club_last_success=scan_stamp;
            sendBroadCastTF(m_state.last_message_time);
            publishMapCloud(m_state.last_message_time);
        }
        clubHealth(result,scan_stamp);
    }
''' +text[end:]
text=text.replace('        std::string pcd_path = request->pcd_path;',
'''        ++club_request_id;
        club_map_loaded=false;
        club_last_success=-1.0;
        club_last_attempt=-1.0;
        m_state.localize_success=false;
        m_state.service_received=false;
        clubHealth(false,stampSec(m_state.last_message_time));
        std::string pcd_path = request->pcd_path;''')
text=text.replace('        response->success = true;', '        club_map_loaded=true;\n        response->success = true;')
text=text.replace('    NodeConfig m_config;', '    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr club_health_pub;\n    NodeConfig m_config;')
path.write_text(text)
diff=list(difflib.unified_diff(original.splitlines(True),text.splitlines(True),fromfile='a/localizer_node.cpp',tofile='b/localizer_node.cpp'))
header=root/'localizer/src/localizers/icp_localizer.h'
original_h=header.read_text()
changed=original_h.replace('    ICPConfig &config()',
'''    double roughFitness() const { return rough_fitness; }
    double refineFitness() const { return refine_fitness; }
    double rough_fitness = std::numeric_limits<double>::infinity();
    double refine_fitness = std::numeric_limits<double>::infinity();
    ICPConfig &config()''').replace('#pragma once','#pragma once\n#include <limits>')
header.write_text(changed)
diff+=difflib.unified_diff(original_h.splitlines(True),changed.splitlines(True),fromfile='a/icp_localizer.h',tofile='b/icp_localizer.h')
cpp=root/'localizer/src/localizers/icp_localizer.cpp';original_c=cpp.read_text()
changed=original_c.replace('    reader.read(path, *cloud);','    if (reader.read(path, *cloud) < 0 || cloud->empty()) return false;')
changed=changed.replace('    CloudType::Ptr aligned_cloud(new CloudType);',
'''    rough_fitness = refine_fitness = std::numeric_limits<double>::infinity();
    CloudType::Ptr aligned_cloud(new CloudType);
    if (m_refine_inp->empty() || m_rough_inp->empty()) return false;''')
changed=changed.replace('    if (!m_rough_icp.hasConverged()', '    rough_fitness=m_rough_icp.getFitnessScore();\n    if (!m_rough_icp.hasConverged()')
changed=changed.replace('    if (!m_refine_icp.hasConverged()', '    refine_fitness=m_refine_icp.getFitnessScore();\n    if (!m_refine_icp.hasConverged()')
cpp.write_text(changed)
diff+=difflib.unified_diff(original_c.splitlines(True),changed.splitlines(True),fromfile='a/icp_localizer.cpp',tofile='b/icp_localizer.cpp')
Path('/work/logs/localizer_health.patch').write_text(''.join(diff))
print('Applied current-request, fresh-scan diagnostics and PCD read checks')
