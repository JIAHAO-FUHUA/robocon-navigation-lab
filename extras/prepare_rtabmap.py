"""Build a TurtleBot3 RGB-D overlay for the ROS 2 Jazzy homework."""
from copy import deepcopy
from pathlib import Path
import shutil
from xml.etree import ElementTree as ET


def add_depth_camera(path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(path, parser=parser)
    model = tree.getroot().find("model")
    link = model.find("link[@name='camera_rgb_frame']")
    rgb = link.find("sensor[@name='camera']")
    link.set("name", "camera_rgb_optical_frame")
    model.append(ET.Element("link", {"name": "camera_rgb_frame"}))
    joint = ET.SubElement(model, "joint", {"name": "camera_rgb_optical_joint", "type": "fixed"})
    ET.SubElement(joint, "parent").text = "camera_rgb_frame"
    ET.SubElement(joint, "child").text = "camera_rgb_optical_frame"
    ET.SubElement(joint, "pose").text = "0 0 0 -1.57079632679 0 -1.57079632679"
    axis = ET.SubElement(joint, "axis")
    ET.SubElement(axis, "xyz").text = "0 0 1"
    rgb.find("gz_frame_id").text = "camera_rgb_optical_frame"
    rgb.find("camera/image/width").text = "640"
    rgb.find("camera/image/height").text = "480"
    depth = deepcopy(rgb)
    depth.set("name", "depth")
    depth.set("type", "depth")
    depth.find("topic").text = "camera/depth/image_raw"
    depth.find("camera").set("name", "intel_realsense_r200_depth")
    depth.find("camera/camera_info_topic").text = "camera/depth/camera_info"
    depth.find("camera").remove(depth.find("camera/noise"))
    link.append(depth)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def build_overlay():
    package = "turtlebot3_gazebo"
    upstream = Path("/opt/ros/jazzy/share") / package
    work = Path(__file__).resolve().parent
    overlay = work / ".rtabmap_overlay"
    if (overlay / "ready").exists():
        return overlay
    if not upstream.is_dir():
        raise SystemExit("Install ros-jazzy-turtlebot3-gazebo and the RTAB-Map demo packages first.")
    share = overlay / "share" / package
    shutil.copytree(upstream, share, dirs_exist_ok=True)
    add_depth_camera(share / "models/turtlebot3_waffle/model.sdf")
    spawn = share / "launch/spawn_turtlebot3.launch.py"
    original = spawn.read_text()
    xy = "'-y', y_pose,"
    if original.count(xy) != 1:
        raise SystemExit("The Jazzy TurtleBot3 launch file changed. Check the upstream RGB-D instructions.")
    spawn.write_text(original.replace(xy, xy + "\n            '-Y', os.environ.get('ROBOCON_SPAWN_YAW', '0.0'),"))
    index = overlay / "share/ament_index/resource_index/packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / package).touch()
    (overlay / "ready").touch()
    return overlay


if __name__ == "__main__":
    print(build_overlay())
