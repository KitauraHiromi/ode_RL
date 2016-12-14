
def no_reward(state, *args):
    return 0

def tactile_reward(state, *args):
    try:
        tactile_reward.target_tac_index = args[0]
    except IndexError:
        pass
    # If tactile value is near target, the reward will be high.
    target = 0.5
    try:
        reward = 100 * state[tactile_reward.target_tac_index]
    except ZeroDivisionError:
        reward = 100
    return min(reward, 100)

        
def angle_reward(state, *args):
    try:
        angle_reward.target_angle_index = args[0]
    except IndexError:
        pass
    # If tactile value is near target, the reward will be high.
    target = 90
    try:
        reward = 1 / (target - state[angle_reward.target_angle_index])
    except ZeroDivisionError:
        reward = 100
    # print angle_reward, state
    return min(angle_reward, 100)
