#RobotState.py

# keeps track of the state of the robot

import copy
import numpy as np
import rospy

import Utils

#TODO make this dynamic
#NUM_FINGER_DOFS = rospy.get_param('/ada/num_finger_dofs', 2)

class RobotState(object):
  def __init__(self, ee_trans, finger_dofs, mode=0, num_modes=2):
    self.ee_trans = copy.copy(ee_trans)
    self.finger_dofs = copy.copy(finger_dofs)
    self.mode = mode
    self.num_modes = num_modes
    self.myee = copy.copy(ee_trans)

  def get_pos(self):
    return self.ee_trans[0:3,3]

  def get_finger_dofs(self):
    return self.finger_dofs

  def switch_mode(self):
    self.mode = self.next_mode()

  def next_mode(self):
    return (self.mode+1)%self.num_modes

  def set_mode(self, mode):
    assert mode >= 0 and mode <= self.num_modes
    self.mode = mode

  def mode_after_action(self, action):
    if action.is_no_mode_switch():
      return self.mode
    else:
      return action.switch_mode_to

###
  def updateState(self, matrix_pose):
    self.ee_trans = matrix_pose
###
  def state_after_action(self, action, time):
    state_copy = copy.deepcopy(self)
    if not action.is_no_move():
      state_copy.ee_trans = Utils.ApplyTwistToTransform(action.twist, state_copy.ee_trans, time)

    if not action.is_no_mode_switch():
      state_copy.mode = action.switch_mode_to

    self.myee = state_copy
    return state_copy

  def num_finger_dofs(self):
    return len(self.finger_dofs)

  def printPose(self):
    print("STATE ACTION: " +str(self.myee))

  def __str__(self):
    return "STATE ACTION: %s" %(self.myee) 

#actions we can enact on the state
#corresponds to a mode switch and a twist


#TODO handling an unspecified number of fingers is messy here. Handle this better
class Action(object):
  no_mode_switch=-1
  no_move = np.zeros(6)
  no_finger_vel = np.zeros(2)
  def __init__(self, twist=copy.copy(no_move), finger_vel=None, switch_mode_to=no_mode_switch):
    self.twist = twist
    if finger_vel is None:
      self.finger_vel = Action.no_finger_vel
    else:
      self.finger_vel = finger_vel

    self.switch_mode_to = switch_mode_to

  def as_tuple(self):
    return (self.twist, self.switch_mode_to)

  def is_no_action(self):
    return self.twist == self.no_move and self.switch_mode_to == self.no_mode_switch and self.finger_vel == Action.no_finger_vel

  def __eq__(self, other): 
    return self.twist == other.twist and self.switch_mode_to == other.switch_mode_to and self.finger_vel == other.finger_vel

  def is_no_mode_switch(self):
    return self.switch_mode_to == self.no_mode_switch

  def is_no_move(self):
    return np.linalg.norm(self.twist) < 1e-10

  def is_no_finger_move(self):
    return np.linalg.norm(self.finger_vel) < 1e-10
  
  def getTwist(self):
    return self.twist

  # return the parts of this actions move that can be
  # altered by the user given the current robot
#  def move_in_mode(self, mode):
#    if mode == 0:
#      return self.move[:3]
#    elif mode == 1:
#      return self.move[3:]
#    else:
#      return self.move
    
  def __str__(self):
    return 'twist:' + str(self.twist) + ' finger:' + str(self.finger_vel) + ' mode to:' + str(self.switch_mode_to)

  @staticmethod
  def set_no_finger_vel(num_finger_dofs):
    Action.no_finger_vel = np.zeros(num_finger_dofs)
