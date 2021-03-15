#Generic assistance policy for multiple goals
import numpy as np
import IPython
import AssistancePolicyOneGoal as GoalPolicy
from RobotState import Action


class AssistancePolicy:
  """
  Assistance Policy Class \n
  Args:
    robot_state: RobotState
    goals: list of GoalAssistance
  """
  ACTION_APPLY_TIME = 0.1  # used when calculating future state
  def __init__(self, robot_state, goals):
    self.goals = goals
    self.robot_state = robot_state
    self.goal_assist_policies = []
    for goal in goals:
      self.goal_assist_policies.append(GoalPolicy.AssistancePolicyOneGoal(goal))


  def update(self, user_action):
    """
    Update assistance policy \n
    Args:
      user_action: Action
    """
    robot_state_after_action = self.robot_state.state_after_action(user_action, self.ACTION_APPLY_TIME)
    #user action corresponds to the effect of direct teleoperation on the robot
    self.user_action = user_action

    for goal_policy in self.goal_assist_policies:
      goal_policy.update(self.robot_state, self.user_action, robot_state_after_action)

  def get_values(self):
    """
    Get v_values and q_values \n
    Return:
      v_values
      q_values
    """
    values = np.ndarray(len(self.goal_assist_policies))
    qvalues = np.ndarray(len(self.goal_assist_policies))
    for ind,goal_policy in enumerate(self.goal_assist_policies):
      values[ind] = goal_policy.get_value()
      qvalues[ind] = goal_policy.get_qvalue()

    return values,qvalues


  def get_probs_last_user_action(self):
    values,qvalues = self.get_values()
    return np.exp(-(qvalues-values))

  def get_assisted_action(self, goal_distribution, fix_magnitude_user_command=True):
    """
    Get assisted action \n
    Args: 
      goal_distribution
    Return: assisted action
    """
    assert goal_distribution.size == len(self.goal_assist_policies)

    action_dimension = GoalPolicy.TargetPolicy.ACTION_DIMENSION
    #TODO how do we handle mode switch vs. not?
    total_action_twist = np.zeros(action_dimension)
    for goal_policy,goal_prob in zip(self.goal_assist_policies, goal_distribution):
      total_action_twist += goal_prob * goal_policy.get_action()

    total_action_twist /= np.sum(goal_distribution)
    to_ret_twist = total_action_twist + self.user_action.twist
    #print "before magnitude adjustment: " + str(to_ret_twist)
    if fix_magnitude_user_command and not np.linalg.norm(self.user_action.twist)==0:
      to_ret_twist *= np.linalg.norm(self.user_action.twist)/np.linalg.norm(to_ret_twist)
    #print "after magnitude adjustment: " + str(to_ret_twist)

    return to_ret_twist
    
      
    

