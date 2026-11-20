from pathlib import Path
import difflib
path=Path('/work/jazzy_ws/src/FASTLIO2_ROS2/localizer/src/localizer_node.cpp')
old=path.read_text()
before='        m_state.last_message_time = cloud_msg->header.stamp;'
after=before+'''\n        // Refresh TF only from genuinely new measurement stamps; never renew ICP health here.
        if (club_map_loaded && m_state.localize_success) sendBroadCastTF(m_state.last_message_time);'''
if after not in old:
    assert old.count(before)==1
    new=old.replace(before,after);path.write_text(new)
    Path('/work/logs/tf_refresh.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/localizer_node.cpp',tofile='b/localizer_node.cpp')))
print('TF refresh ready')
