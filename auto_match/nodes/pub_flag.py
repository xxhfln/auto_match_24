import rospy
import rospkg
from std_msgs.msg import String, Header

class PubFlag:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('pub_flag_node', anonymous=True)

        print("========ready to pub_flag===== ")

        
        # 订阅任务控制指令的话题
        self.flag_pub = rospy.Publisher("task_start_flag", String, queue_size=1)
        msg = String()
        msg.data = 'true'
        flag_pub.publish(msg)

pubflag = PubFlag()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("ShutDown")