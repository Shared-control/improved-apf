import numpy as np
import IPython
import AssistancePolicyOneTarget
import tf.transformations as transmethods
from Utils import * 


ACTION_DIMENSION = 6

class HuberAssistancePolicy(AssistancePolicyOneTarget.AssistancePolicyOneTarget):
  """
  Huber Assistance Policy \n
  Args: 
    pose: matrix of goal
  """
  def __init__(self, pose):
    super(HuberAssistancePolicy, self).__init__(pose)
    self.set_constants(self.TRANSLATION_LINEAR_MULTIPLIER, self.TRANSLATION_DELTA_SWITCH, self.TRANSLATION_CONSTANT_ADD, self.ROTATION_LINEAR_MULTIPLIER, self.ROTATION_DELTA_SWITCH, self.ROTATION_CONSTANT_ADD, self.ROTATION_MULTIPLIER)

  def update(self, robot_state, user_action, robot_state_after_action):
    super(HuberAssistancePolicy, self).update(robot_state, user_action, robot_state_after_action)
    self.dist_translation = np.linalg.norm(robot_state.ee_trans[0:3,3] - self.goal_pos)
    self.dist_translation_aftertrans = np.linalg.norm(self.robot_state_after_action.ee_trans[0:3,3] - self.goal_pos)

    self.quat_curr = transmethods.quaternion_from_matrix(robot_state.ee_trans)
    self.quat_after_trans = transmethods.quaternion_from_matrix(self.robot_state_after_action.ee_trans)

    self.dist_rotation = QuaternionDistance(self.quat_curr, self.goal_quat)
    self.dist_rotation_aftertrans = QuaternionDistance(self.quat_after_trans, self.goal_quat)

    #something about mode switch distance???

  # return the action that will be used for this goal
  def get_action(self):
    return -self.get_q_derivative()

  def get_cost(self):
    return self.get_cost_translation() #+ self.get_cost_rotation()

  def get_value(self):
    return self.get_value_translation() #+ self.get_value_rotation()

  def get_qvalue(self):
    return self.get_qvalue_translation() #+ self.get_qvalue_rotation()

  def get_q_derivative(self):
    #q_rot = self.get_qderivative_rotation()
    q_rot = np.zeros(3)
    q_trans = self.get_qderivative_translation()
    return np.append(q_trans, q_rot)  # twist


  #parts split into translation and rotation
  def get_value_translation(self, dist_translation=None):
    if dist_translation is None:
      dist_translation = self.dist_translation

    if dist_translation <= self.TRANSLATION_DELTA_SWITCH:
      return self.TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF * dist_translation*dist_translation + self.TRANSLATION_CONSTANT_ADD*dist_translation
    else:
      return 0.6 *(self.TRANSLATION_LINEAR_COST_MULT_TOTAL * dist_translation - self.TRANSLATION_LINEAR_COST_SUBTRACT)

  def get_cost_translation(self, dist_translation=None):
    if dist_translation is None:
      dist_translation = self.dist_translation

    if dist_translation > self.TRANSLATION_DELTA_SWITCH:
      return self.ACTION_APPLY_TIME * (self.TRANSLATION_LINEAR_COST_MULT_TOTAL)
    else:
      return self.ACTION_APPLY_TIME * (self.TRANSLATION_QUADRATIC_COST_MULTPLIER * dist_translation + self.TRANSLATION_CONSTANT_ADD)

  def get_qvalue_translation(self):
    return self.get_cost_translation() + self.get_value_translation(self.dist_translation_aftertrans)

  def get_qderivative_translation(self):
    translation_diff = self.robot_state_after_action.ee_trans[0:3,3] - self.goal_pos
    dist_to_go = np.linalg.norm(translation_diff)

    if dist_to_go <= self.TRANSLATION_DELTA_SWITCH:
      translation_derivative = self.ACTION_MULT_CLOSE * (self.TRANSLATION_CONSTANT_ADD*(translation_diff/dist_to_go) +
                                    self.TRANSLATION_QUADRATIC_COST_MULTPLIER*translation_diff)
    else:
      translation_derivative = self.ACTION_MULT_FAR * self.TRANSLATION_LINEAR_COST_MULT_TOTAL*(translation_diff/dist_to_go)

    #hacky part to make it not jumpy
    dist_translation_limit = 2e-2;
    if (dist_to_go < dist_translation_limit):
      translation_derivative *= dist_to_go/dist_translation_limit;

    return translation_derivative / self.ROBOT_TRANSLATION_COST_MULTIPLIER 


  def get_value_rotation(self, dist_rotation=None):
    if dist_rotation is None:
      dist_rotation = self.dist_rotation

    if dist_rotation <= self.ROTATION_DELTA_SWITCH:
      return self.ROTATION_MULTIPLIER * (self.ROTATION_QUADRATIC_COST_MULTPLIER_HALF * dist_rotation*dist_rotation + self.ROTATION_CONSTANT_ADD*dist_rotation)
    else:
      return self.ROTATION_MULTIPLIER*(self.ROTATION_LINEAR_COST_MULT_TOTAL * dist_rotation - self.ROTATION_LINEAR_COST_SUBTRACT)

  def get_cost_rotation(self, dist_rotation=None):
    if dist_rotation is None:
      dist_rotation = self.dist_rotation

    if dist_rotation > self.ROTATION_DELTA_SWITCH:
      return self.ACTION_APPLY_TIME * self.ROTATION_MULTIPLIER * self.ROTATION_LINEAR_COST_MULT_TOTAL
    else:
      return self.ACTION_APPLY_TIME * self.ROTATION_MULTIPLIER * (self.ROTATION_QUADRATIC_COST_MULTPLIER * dist_rotation + self.ROTATION_CONSTANT_ADD)

  def get_qvalue_rotation(self):
    return self.get_cost_rotation() + self.get_value_rotation(self.dist_rotation_aftertrans)

  def get_qderivative_rotation(self):
    quat_between = transmethods.quaternion_multiply(self.goal_quat, transmethods.quaternion_inverse(self.quat_after_trans))

    rotation_derivative = quat_between[0:-1]
    rotation_derivative /= np.linalg.norm(rotation_derivative)

    if self.dist_rotation_aftertrans > self.ROTATION_DELTA_SWITCH:
      rotation_derivative_magnitude = self.ROTATION_LINEAR_COST_MULT_TOTAL
    else:
      rotation_derivative_magnitude = self.ROTATION_CONSTANT_ADD
      rotation_derivative_magnitude += self.ROTATION_QUADRATIC_COST_MULTPLIER*self.dist_rotation_aftertrans

    rotation_derivative *= self.ROTATION_MULTIPLIER * rotation_derivative_magnitude / self.ROBOT_ROTATION_COST_MULTIPLIER

    if (np.sum(self.goal_quat * self.quat_after_trans)) > 0:
      rotation_derivative *= -1

    #check to make sure direction is correct
#    quat_thisdir = transition_quaternion(self.quat_curr, rotation_derivative, self.ACTION_APPLY_TIME)
#    val_thisdir = self.get_value_rotation(dist_rotation=QuaternionDistance(quat_thisdir, self.goal_quat))
#    quat_negdir = transition_quaternion(self.quat_curr, -1.*rotation_derivative, self.ACTION_APPLY_TIME)
#    val_negdir = self.get_value_rotation(dist_rotation=QuaternionDistance(quat_negdir, self.goal_quat))
#    if (val_thisdir < val_negdir):
#      print 'ROT DIRECTION ERROR vals this dir: ' + str(val_thisdir) + '  neg dir: ' + str(val_negdir)
#    else:
#      print 'rotdir good'

    #hacky part to make it not jumpy
    dist_rotation_limit = np.pi/12.;
    #print 'dist rotation: ' + str(self.dist_rotation_aftertrans)
    if (self.dist_rotation_aftertrans < dist_rotation_limit):
      rotation_derivative *= self.dist_rotation_aftertrans/dist_rotation_limit;

    return rotation_derivative




  #HUBER CONSTANTS
  #Values used when assistance always on
  TRANSLATION_LINEAR_MULTIPLIER = 1.5
  TRANSLATION_DELTA_SWITCH = 0.10
  TRANSLATION_CONSTANT_ADD = 0.1

  ROTATION_LINEAR_MULTIPLIER = 0.20
  #ROTATION_DELTA_SWITCH = np.pi/7.
  ROTATION_DELTA_SWITCH = np.pi/32.
  ROTATION_CONSTANT_ADD = 0.01
  ROTATION_MULTIPLIER = 0.07

  #ROBOT_TRANSLATION_COST_MULTIPLIER = 14.5
  #ROBOT_ROTATION_COST_MULTIPLIER = 0.10

  ROBOT_TRANSLATION_COST_MULTIPLIER = 40.0
  ROBOT_ROTATION_COST_MULTIPLIER = 0.05



  #HUBER CACHED CONSTANTS that will be calculated soon
  TRANSLATION_LINEAR_COST_MULT_TOTAL = 0.0
  TRANSLATION_QUADRATIC_COST_MULTPLIER = 0.0
  TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF = 0.0
  TRANSLATION_LINEAR_COST_SUBTRACT = 0.0

  ROTATION_LINEAR_COST_MULT_TOTAL = 0.0
  ROTATION_QUADRATIC_COST_MULTPLIER = 0.0
  ROTATION_QUADRATIC_COST_MULTPLIER_HALF = 0.0
  ROTATION_LINEAR_COST_SUBTRACT = 0.0



  def set_constants(self, huber_translation_linear_multiplier, huber_translation_delta_switch, huber_translation_constant_add, huber_rotation_linear_multiplier, huber_rotation_delta_switch, huber_rotation_constant_add, huber_rotation_multiplier, robot_translation_cost_multiplier=None, robot_rotation_cost_multiplier=None):
    self.ACTION_MULT_FAR = 2  # increase module assisted action when far from goal
    self.ACTION_MULT_CLOSE = 8  # increase module assisted action when near goal

    self.TRANSLATION_LINEAR_MULTIPLIER = huber_translation_linear_multiplier
    self.TRANSLATION_DELTA_SWITCH = huber_translation_delta_switch
    self.TRANSLATION_CONSTANT_ADD = huber_translation_constant_add

    self.ROTATION_LINEAR_MULTIPLIER = huber_rotation_linear_multiplier
    self.ROTATION_DELTA_SWITCH = huber_rotation_delta_switch
    self.ROTATION_CONSTANT_ADD = huber_rotation_constant_add
    self.ROTATION_MULTIPLIER = huber_rotation_multiplier

    if robot_translation_cost_multiplier:
      self.ROBOT_TRANSLATION_COST_MULTIPLIER = robot_translation_cost_multiplier
    if robot_rotation_cost_multiplier:
      self.ROBOT_ROTATION_COST_MULTIPLIER = robot_rotation_cost_multiplier
    
    self.calculate_cached_constants()

  #other constants that are cached for faster computation
  def calculate_cached_constants(self):
    self.TRANSLATION_LINEAR_COST_MULT_TOTAL = self.TRANSLATION_LINEAR_MULTIPLIER + self.TRANSLATION_CONSTANT_ADD
    self.TRANSLATION_QUADRATIC_COST_MULTPLIER = self.TRANSLATION_LINEAR_MULTIPLIER/self.TRANSLATION_DELTA_SWITCH
    self.TRANSLATION_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * self.TRANSLATION_QUADRATIC_COST_MULTPLIER
    self.TRANSLATION_LINEAR_COST_SUBTRACT = self.TRANSLATION_LINEAR_MULTIPLIER * self.TRANSLATION_DELTA_SWITCH * 0.5

    self.ROTATION_LINEAR_COST_MULT_TOTAL = self.ROTATION_LINEAR_MULTIPLIER + self.ROTATION_CONSTANT_ADD
    self.ROTATION_QUADRATIC_COST_MULTPLIER = self.ROTATION_LINEAR_MULTIPLIER/self.ROTATION_DELTA_SWITCH
    self.ROTATION_QUADRATIC_COST_MULTPLIER_HALF = 0.5 * self.ROTATION_QUADRATIC_COST_MULTPLIER
    self.ROTATION_LINEAR_COST_SUBTRACT = self.ROTATION_LINEAR_MULTIPLIER * self.ROTATION_DELTA_SWITCH * 0.5


def UserInputToRobotAction(user_input):
  return np.append(user_input, np.zeros(3))


def transition_quaternion(quat, angular_vel, action_apply_time):
  norm_vel = np.linalg.norm(angular_vel)
  return transmethods.quaternion_multiply( transmethods.quaternion_about_axis(action_apply_time*norm_vel, angular_vel/norm_vel) , quat)
