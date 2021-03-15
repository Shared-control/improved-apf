import numpy as np
import tf.transformations as transmethods
import copy

class AssistancePolicyOneTarget(object):
  """
  Aiistance Policy One Target Class \n
  Args: 
    pose: matrix of goal
  """
  ACTION_APPLY_TIME = 0.01 

  def __init__(self, pose):
    self.goal_pose = pose
    self.goal_quat = transmethods.quaternion_from_matrix(pose)
    self.goal_pos = pose[0:3,3]

  def update(self, robot_state, user_action, robot_state_after_action):
    self.robot_state = robot_state
    self.user_action = user_action
    self.robot_state_after_action = robot_state_after_action

  def get_action(self):
    ee_trans = self.last_ee_trans
    pos_diff = 5.*(self.goal_pose[0:3,3] - ee_trans[0:3,3])

    pos_diff_norm = np.linalg.norm(pos_diff)

    clip_norm_val = 0.02
    if (pos_diff_norm > clip_norm_val):
      pos_diff /= pos_diff_norm/clip_norm_val

    return pos_diff


  #def pose_after_user_action(self, ee_trans, user_action):
  def state_after_user_action(self, robot_state, user_action):
    return robot_state.state_after_action(user_action, self.ACTION_APPLY_TIME)



