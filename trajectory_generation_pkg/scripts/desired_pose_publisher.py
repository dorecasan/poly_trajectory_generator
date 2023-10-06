#! /usr/bin/env python3
import pandas as pd
import rospy
from geometry_msgs.msg import PoseArray, Pose


class DesiredPoseNode:
    def __init__(self,topic_name = 'desired_poses'):
        self.topic_name = topic_name
        self.desired_posed_pub = rospy.Publisher(self.topic_name,PoseArray,queue_size=1,latch=True)
        self.poseArray = None
        self.data = None
        
        
    def ReadData(self,file_path):
        self.data = pd.read_csv(file_path,header=None)
        self.data.columns = ['x','y']
        
        self.poseArray = PoseArray()
        self.poseArray.header.frame_id = "map"

        for i in range(0,len(self.data['x']),15):
            pose = Pose()
            pose.position.x = self.data['x'][i]
            pose.position.y = self.data['y'][i]
            self.poseArray.poses.append(pose)
        rospy.loginfo("Total number of points: {}".format(len(self.poseArray.poses)))
        
    def PublishData(self):
        self.desired_posed_pub.publish(self.poseArray)
        rospy.loginfo("Published to topic {}".format(self.topic_name))
    
    def PublishDataToTopic(self,topic_name,rate=1):
        pub = rospy.Publisher(topic_name,PoseArray,queue_size=1)
        r = rospy.Rate(rate)
        while(not rospy.is_shutdown()):
            pub.publish(self.poseArray)
            r.sleep()
        
        

if __name__ == "__main__":
    rospy.init_node("trajectory_publish_node")
    pub_node = DesiredPoseNode("/desired_poses")
    pub_node.ReadData("/home/toannn/Robotics/Robotics/trajectoryGeneration/src/trajectory_generation_pkg/data/road_profile.csv")
    pub_node.PublishData()
    pub_node.PublishDataToTopic("desired_poses_rviz",rate=1)
    rospy.spin()
    
    
        



