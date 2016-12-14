import random
import numpy as np

# Should use numpy.argmax ?        
def argmax(func, arguments):
    reward_list = map(func, arguments)
    #if reward_list.count(max(reward_list)) > 1:
        
    return arguments[reward_list.index(max(reward_list))]

    
def greedy(Value_func, state, action_list):
    arguments    = [[state, action] for action in action_list]
    return argmax(Value_func, arguments)[1]

    
def epsiron_greedy(Value_func, state, action_list):
    epsiron = 0.5
    num = random.random()
    if(num < epsiron):
        return greedy(Value_func, state, action_list)
    else:
        return random.choice(action_list)

        
def discrete_epsiron_greedy_policy(Q):
    action_index =  epsiron_greedy(Q.function, Q.st_index, Q.action_index_list)
    return Q.action_list[action_index]

    
def policy_from_state_density(state, state_prob, action_list):
    
    # Retern action based on probability density function of state.
    # state_prob is Probability density function of state.
    # state and action_list must be numpy.ndarray.
    # state and action_list[i] must be the same size.

    possible_state_list = state + action_list
    action_prob_list = np.array(map(state_prob, possible_state_list))
    action_prob_list /= action_prob_list.sum()
    prob_range = [action_prob_list[:i+1].sum() for i in xrange(len(action_prob_list))]

    def select_action(prob_range):
        val = random.random()
        for i in xrange(len(prob_range)):
            if prob_range[i] > val:
                return i
                
    return action_list[select_action(prob_range)]
