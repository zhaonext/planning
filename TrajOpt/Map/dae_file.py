import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('dae_model_publisher')

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

marker = Marker()
marker.header.frame_id = "map"
marker.type = marker.MESH_RESOURCE
marker.action = marker.ADD
marker.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
marker.scale.x = 1  # 根据实际模型大小调整
marker.scale.y = 1  # 根据实际模型大小调整
marker.scale.z = 1  # 根据实际模型大小调整
marker.color.a = 1.0  # 设置透明度
marker.color.r = 1.0  # 设置颜色（红色）
marker.color.g = 1.0  # 设置颜色（绿色）
marker.color.b = 1.0  # 设置颜色（蓝色）
marker.mesh_resource = "package://my_robot_pkg/models/bmw_x5.dae"  # 修改为正确的包内路径

while not rospy.is_shutdown():
    pub.publish(marker)
    rospy.sleep(1)
