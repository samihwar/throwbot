import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera
import time
import math

def keep_it_running(robot, env, sleep_time=0.01):
    print("Simulation running... Press Ctrl+C to stop.")
    
    try:
        while True:
            # Get end-effector's position
            ee_pos = robot.get_joint_obs()['ee_pos']
            
            # Print the position
            # print(f"End-Effector Position: {ee_pos}")
            
            p.stepSimulation()
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("Exiting the simulation...")
        env.close()

      
def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    camera = None
    # robot = Panda((0, 0.5, 0), (0, 0, math.pi))
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    # env.SIMULATION_STEP_DELAY = 0
    while True:
        obs, reward, done, info = env.step(env.read_debug_parameter(), 'end')
        # print(obs, reward, done, info)

def pick_and_lift():
    ycb_models = YCBModels(os.path.join('./data/ycb', '**', 'textured-decmp.obj'))
    camera = Camera((0.2, 0.2, 0.1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 2, (640, 480), 60)
    # camera = None
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera=camera, vis=True)
    env.reset()
    
    ball_position = [0.7, 0.7, 0.05]
    ball_id = env.load_ball(ball_position)
    env.get_the_ball(ball_position)
    env.lift_ball()
    # env.throw_ball([0.5, 0.5,0.5],2,2)

    keep_it_running(robot,env)

def general_control():
    ycb_models = YCBModels(os.path.join('./data/ycb', '**', 'textured-decmp.obj'))
    camera = Camera((1, 1, 1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 5, (320, 320), 40)
    camera = None
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    #   testing move_ee
    
    # starting_joints = {'shoulder_pan_joint': -1.5692913357931622, 'shoulder_lift_joint': -1.5464002711113891,'elbow_joint': 1.3698032438338124,
    #                    'wrist_1_joint': -1.3905061274634392, 'wrist_2_joint': -1.5702364399647457, 'wrist_3_joint': 0.0007070970677205323,'finger_joint': -2.861393527469414e-07,
    #                    'left_inner_finger_joint': 9.464288054072034e-08, 'left_inner_knuckle_joint': -7.804802604672394e-07,
    #                    'right_outer_knuckle_joint': -5.85044992752054e-07, 'right_inner_finger_joint': 8.099090238868875e-07,
    #                    'right_inner_knuckle_joint': 0.4378499984741211, 'gripper_opening_length': 0.03999999910593033}
    # robot.move_ee(starting_joints, 'joint')

    # statring =[0.1, 0.1, 0.3,0,0,0]
    # env.robot.move_ee(statring, 'end')
    # env.get_the_ball(statring)

    print("General_control... Press Ctrl+C to stop.")
    try:
        while True:
            values = env.read_slider_values()
            print(values)
            obs = env.step(values,'joint')
            # robot.debug_joint_positions()
    except KeyboardInterrupt:
        print("Exiting general_control...")
        print(obs)
        env.close()

def throw_it():
    ycb_models = YCBModels(os.path.join('./data/ycb', '**', 'textured-decmp.obj'))
    camera = Camera((0.2, 0.2, 0.1),
                    (0, 0, 0),
                    (0, 0, 1),
                    0.1, 2, (640, 480), 60)
    robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera=camera, vis=True)
    env.reset()


    # Apply torque to a single joint
    single_joint_torques = [242.0] + [0.0] * (robot.arm_num_dofs - 1) #8000
    # robot.move_ee_with_torque(single_joint_torques, control_method="joint") 

    while True:
        robot.move_ee_with_torque(single_joint_torques, control_method="joint") 
        p.stepSimulation()
        time.sleep(1.0 / 240)

    # pitch = math.pi / 2 
    # action = [0.1, 0.6, 0.3, 0, pitch, 0]  # x, y, z 
    # env.move_to_position(action)
    # time.sleep(2)

    # ee_pos = robot.get_joint_obs()['ee_pos']
    # ball_position = [ee_pos[0], ee_pos[1], ee_pos[2]-0.08]
    # ball_id = env.load_ball(ball_position)
    
    # env.grasp_ball()
    # env.lift_ball()
    # # here we have ready grasped ball

    keep_it_running(robot, env)

if __name__ == '__main__':
    # user_control_demo()
    # pick_and_lift()
    # throw_it()
    general_control()