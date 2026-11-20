"""Independent truth-only evaluator; no motion publisher and no control interface."""
import csv
from pathlib import Path
import signal
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

run=Path(sys.argv[1]);rclpy.init();node=Node('club_independent_evaluation')
handle=(run/'evaluation.csv').open('w');writer=csv.writer(handle)
writer.writerow(['source','stamp','x','y','z','qx','qy','qz','qw'])
def write(source,stamp,p):
    writer.writerow([source,stamp.sec+stamp.nanosec/1e9,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]);handle.flush()
node.create_subscription(PoseArray,'/evaluation/ground_truth',lambda m:write('truth',m.header.stamp,m.poses[0]) if m.poses else None,100)
node.create_subscription(Odometry,'/fastlio2/lio_odom',lambda m:write('lio',m.header.stamp,m.pose.pose),100)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:handle.close();node.destroy_node()
