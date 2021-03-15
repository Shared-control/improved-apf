import numpy as np
import scipy.misc

logsumexp = scipy.misc.logsumexp


class GoalPredictorAssistance(object):
    """
    Goal Predictor Assistance Class \n
    Args:
        goals: list of GoalAssistance
    """
    max_prob_any_goal = 0.99
    log_max_prob_any_goal = np.log(max_prob_any_goal)

    def __init__(self, goals):
        self._goals = goals
        self._log_goal_distribution = np.log((1./len(self._goals))*np.ones(len(self._goals)))  # log( p(g|xi) )
        
    def update_distribution(self, v_values, q_values, w_sc):
        """
        Update distribution \n
        Args:
            v_values
            q_values
            weight
        """
        # log( p(xi|g) ) = log( p(xi|g) )@t-1 + log( exp(V - Q) )@t
        self._log_goal_distribution += w_sc * (v_values - q_values)
        self.normalize_log_distribution()


    def normalize_log_distribution(self):
        # log( sum( p(xi|g) ) )
        log_normalization_val = logsumexp(self._log_goal_distribution)
        # log( p(g|xi) )_i = log( p(xi|g) )_i - log( sum( p(xi|g) ) )
        self._log_goal_distribution = self._log_goal_distribution - log_normalization_val
        self.clip_prob()


    def clip_prob(self):
        if len(self._log_goal_distribution) <= 1:
            return
        #check if any too high
        max_prob_ind = np.argmax(self._log_goal_distribution)
        if self._log_goal_distribution[max_prob_ind] > self.log_max_prob_any_goal:
            #see how much we will remove from probability
            diff = np.exp(self._log_goal_distribution[max_prob_ind]) - self.max_prob_any_goal
            #want to distribute this evenly among other goals
            diff_per = diff/(len(self._log_goal_distribution)-1.)
            #distribute this evenly in the probability space...this corresponds to doing so in log space
            # e^x_new = e^x_old + diff_per, and this is formulate for log addition
            self._log_goal_distribution += np.log( 1. + diff_per/np.exp(self._log_goal_distribution))
            #set old one
            self._log_goal_distribution[max_prob_ind] = self.log_max_prob_any_goal


    def get_distribution(self):
        """
        Get distribution \n
        Return: distribution
        """
        return np.exp(self._log_goal_distribution)

    
    def get_prob(self, index):
        """
        Get probability of index goal \n
        Args:
            index: index of the goal
        Return: probability of index goal
        """
        return np.exp(self._log_goal_distribution[index])


    def get_ind_maxes(self):
        """
        Get the indices of the two major probabilities \n
        Return:
            first max index
            second max index
        """
        amax = np.argmax(self._log_goal_distribution)  # This is the maximum value
        mask = np.zeros_like(self._log_goal_distribution)
        mask[amax] = 1
        a = np.ma.masked_array(self._log_goal_distribution, mask=mask)
        second_max = np.argmax(a)  # Second maximum value
        return amax, second_max