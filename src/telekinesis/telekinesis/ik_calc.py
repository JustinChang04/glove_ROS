#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
'''
This takes the glove data, and runs inverse kinematics and then publishes onto LEAP Hand.

Note how the fingertip positions are matching, but the joint angles between the two hands are not.  :) 

Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''
class PybulletIK(Node):
    def __init__(self):
        super().__init__('pyb_ik')  
        # start pybullet
        #clid = p.connect(p.SHARED_MEMORY)
        #clid = p.connect(p.DIRECT)
        p.connect(p.GUI)
        # load right leap hand
        
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
       
        '''
        The end effector indexes refers to the finger tips and DIP 
        These are specific array positions in the URDF loaded through pybullet
        '''
        self.leapEndEffectorIndex = [2, 3, 7, 8, 11, 12, 16, 17, 21, 22]
        
        path_src = os.path.join(os.path.expanduser("~"), "glove_ROS/src/telekinesis/robot_hand/hand.urdf")
        
        # "Leap ID" is copied from the Leap hand's code
        # this is where we load our hand's URDF file to pybullet 
        self.LeapId = p.loadURDF(
            path_src,
            [0, 0, 0],
            p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase = True
        )
 
        #Left hand vs Right hand, which glove you are using 
        if self.is_left:
            #writing to /leaphand_node/cmd_allegro_left
            self.pub_hand = self.create_publisher(JointState, "/left_hand", 10)
            #reading from /glove/l_short
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:  
            self.pub_hand = self.create_publisher(Float32MultiArray, "/right_hand", 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 10)
       ##################

        self.numJoints = p.getNumJoints(self.LeapId)
        # p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        self.create_target_vis()
        
    '''
    We create 5 balls in the simulation to track the movement of the finger tips 
    '''
            
    def create_target_vis(self):
        # Load balls
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        # this array holds the information of the 5 balls which is accessed later
        self.ballMbt = []
        for i in range(0, 5):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition))  # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[1, 0, 1, 1])
        
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[1], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[5], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[9], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[7], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[4])
        p.resetBasePositionAndOrientation(self.ballMbt[4], hand_pos[3], current_orientation)


    '''
    get_glove_data is a subsciber to the Pose Array topic from read_and_send_zmq.py
    Reads the raw data and manipulates it to better map to the actual hand
    '''
    def get_glove_data(self, pose):
        #gets the data converts it and then computes IK and visualizes
        poses = pose.poses

        hand_pos = []  
        for i in range(0,10):
            # hand_pos.append([-poses[i].position.y, -poses[i].position.x, poses[i].position.z])
            hand_pos.append([-poses[i].position.y * 1.2, -poses[i].position.x * 1.2, poses[i].position.z])
        hand_pos[1][2] = hand_pos[1][2] - 0.06  
        hand_pos[1][1] = hand_pos[1][1] + 0.01    
        hand_pos[9][2] = hand_pos[9][2] + 0.04
        # hand_pos[7][0] = hand_pos[7][0] + 0.02
        #hand_pos[2][1] = hand_pos[2][1] + 0.002
        # hand_pos[4][1] = hand_pos[4][1] + 0.002
        # hand_pos[6][1] = hand_pos[6][1] + 0.002

        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
        
    '''
    Computers the Inverse Kinematics 
    '''
        
    def compute_IK(self, hand_pos):
        p.stepSimulation()

        # Thumb: 0, 1
        # Index: 2, 3
        # Middle: 4, 5
        # Ring: 6, 7
        # Pinky: 8, 9   
        
        # assigning specific joint indexes to variable names
        # middle_pos refers to the DIP
        # fingername_pos refers to the tip 

        rightHandPinky_middle_pos = hand_pos[8]
        rightHandPinky_pos = hand_pos[9]

        rightHandIndex_middle_pos = hand_pos[2]
        rightHandIndex_pos = hand_pos[3]
        
        rightHandMiddle_middle_pos = hand_pos[4]
        rightHandMiddle_pos = hand_pos[5]
        
        rightHandRing_middle_pos = hand_pos[6]
        rightHandRing_pos = hand_pos[7]
        
        rightHandThumb_middle_pos = hand_pos[0]
        rightHandThumb_pos = hand_pos[1]
        
        # the x,y,z of each of the end effectors 
        
        leapEndEffectorPos = [
            rightHandThumb_middle_pos,
            rightHandThumb_pos,
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandPinky_middle_pos,
            rightHandPinky_pos
        ]
        
        # here you are calculating each of the joint angles through inverse kinematics 
        # which is held in a jointPoses Array

        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        # Right now: Thumb, Index, Middle, Ring, Pinky
        # 0:4, 4:8, 8:12, 12:16, 16:19
        # combined_jointPoses = (jointPoses[0:3] + (0.0,) + jointPoses[3:7] + (0.0,) + jointPoses[7:11] + (0.0,) + jointPoses[11:15] + (0.0,) + jointPoses[15:19] + (0.0,))
        combined_jointPoses = (jointPoses[0:3] + (0.0,) + jointPoses[3:7] + (0.0,) + jointPoses[7:10] + (0.0,) + jointPoses[10:14] + (0.0,) + jointPoses[14:18] + (0.0,))


        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints in the pybullet simulation 
        for i in range(23):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        real_robot_hand_q = np.array(jointPoses).astype(np.float32)

        # real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        # real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        # real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        
        state = Float32MultiArray()
        state.data = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(state)

        # thread = threading.Thread(target=self.send_serial, args=(real_robot_hand_q,))
        # thread.start()

        # data = struct.pack("<B18f", 0xAB, *real_robot_hand_q)
        # self.pub_hand.write(data)

def main(args=None):
    rclpy.init(args=args)
    pybulletik = PybulletIK()
    rclpy.spin(pybulletik)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pybulletik.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
