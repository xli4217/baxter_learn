from rl_pipeline.env.env import Env

import baxter_interface
from baxter_interface import CHECK_VERSION

class BaxterRightArmVelControlEnv(Env):

    def __init__(self, env_params):
        super(BaxterRightArmVelControlEnv, self).__init__(env_params)

        # state space is [current_joint_angles (7 dof), current_joint_vel (7dof), goal_ee_pos (7 dof)]
        # where goal_ee_pos = (x,y,z,wx,wy,wz,w)
        self.state_space = dict(type='float', shape=(21,))
        self.state_ub = np.array([])
        self.state_lb = np.array([])

        self.state = None
        self.ee_goal = None

        # action space is [joint_vel (7dof)]
        self.action_space = dict(type='float', shape=(7,))
        self.actions_ub = np.array([])
        self.actions_lb = np.array([])

        self.env_params = env_params
                
        rospy.init_node("baxter_learn")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()
        self.right_arm = baxter_interface.limb.Limb(arm)
        self.gripper = baxter_interface.gripper.Gripper('right')
        self.joint_names = self.right_arm.joint_names()
        
    def get_state(self):
        pass

    def get_robot_state(self):
        current_joint_positions = self.right_arm.joint_angles()
        current_joint_velocities = self.right_arm.joint_velocities()
        
    def get_reward(self):
        current_ee_pos = self.right_arm.endpoint_pose()
        
    def reset(self):
        self.right_arm.move_to_neutral()
        self.robot_state = self.get_robot_state()
        # Here sample a random ee_goal in the given region

        self.state = np.concatenate([self.robot_state, self.ee_goal])
        return self.state

    def make_vel_cmd(self,joint_names, joint_velocities):
        return dict([(joint,velocity) for (joint, velocity) in zip(joint_names, joint_velocities)])

    def done_condition_check(self):
        # end effector can't move under the table
        pass
        
    def step(self, actions):
        actions = np.clip(actions, self.actions_lb, self.actions_ub)
        
        
    def seed(self):
        pass

    def close(self):
        pass

    @property
    def state_space(self):
        return self.state_space
        
    @property
    def action_space(self):
        return self.action_space
