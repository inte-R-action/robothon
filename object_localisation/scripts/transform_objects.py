import rospy
import tf

class myNode:
    def __init__(self, *args):
        self.tf_listener_ = TransformListener()

    def example_function(self):
        if self.tf.frameExists("/base_link") and self.tf.frameExists("/fingertip"):
            t = self.tf_listener_.getLatestCommonTime("/base_link", "/fingertip")
            p1 = geometry_msgs.msg.PoseStamped()
            p1.header.frame_id = "fingertip"
            p1.pose.orientation.w = 1.0    # Neutral orientation
            p_in_base = self.tf_listener_.transformPose("/base_link", p1)
            print "Position of the fingertip in the robot base:"
            print p_in_base