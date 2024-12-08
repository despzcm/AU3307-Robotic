from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math




if __name__ == '__main__':
    env = DofbotEnv()
    env.reset()
    Reward = False


    '''
    constants here
    '''
    GRIPPER_DEFAULT_ANGLE = 20. / 180. * 3.1415
    GRIPPER_CLOSE_ANGLE = -20. / 180. * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE


    initial_jointposes = [1.57, 0., 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.023, -0.023, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.13]
    obj_offset3 = [-0.025, 0.025, 0.09]

    block_pos, block_orn = env.get_block_pose()

    start_time = None
    lift_offset=False
    time.sleep(3)
    while not Reward:

        '''
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        '''

        '''
        code here
        '''
        def check_state(current_joint_state, target_joint_state):
            current= np.array(current_joint_state)
            target= np.array(target_joint_state)
            if np.all(np.abs(current- target) < 0.01):
                return True
            else:
                return False
       
        
        if current_state== INITIAL_STATE:
            target_pos = copy.deepcopy(np.array(block_pos))
            obj_offset_ = np.array(obj_offset)
            target_pos += obj_offset_
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            current_joint_state = env.get_dofbot_jointPoses()[0]
            if check_state(current_joint_state, target_joint_state):
                current_state = GRASP_STATE
        elif current_state== GRASP_STATE:
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            for i in range(10):
                env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
 
            current_state = LIFT_STATE
            
        elif current_state== LIFT_STATE:
            if lift_offset==False:
                target_pos[2] += 0.05
                lift_offset=True
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state = env.get_dofbot_jointPoses()[0]
            if check_state(current_joint_state, target_joint_state):
                current_state = MOVE_STATE
            
        elif current_state== MOVE_STATE:
            target_pos = copy.deepcopy(env.get_target_pose())
            target_pos[2]=block_pos[2]
            obj_offset_2 = np.array(obj_offset2)
            target_pos += obj_offset_2
            
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state = env.get_dofbot_jointPoses()[0]
            if check_state(current_joint_state, target_joint_state):
                current_state = BACK_STATE
            
        elif current_state== BACK_STATE:
            target_pos = copy.deepcopy(env.get_target_pose())
            target_pos[2]=block_pos[2]
            obj_offset_3 = np.array(obj_offset3)
            target_pos += obj_offset_3
            target_joint_state = env.dofbot_setInverseKine(target_pos)
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state = env.get_dofbot_jointPoses()[0]
            if check_state(current_joint_state, target_joint_state):
                env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
                

            
            
        Reward = env.reward()
        
