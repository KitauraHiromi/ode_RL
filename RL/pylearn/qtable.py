import numpy as np


def prod(float_list):
    return reduce(lambda x,y: x*y, float_list)


class Q_table(object):
    def __init__(self, DOF, TAC_NUM, pitch, ROM, tac_resolution, state_space_type="grid"):

        self.DOF = DOF
        self.TAC_NUM = TAC_NUM
        self.pitch = pitch
        self.ROM = ROM
        self.tac_resolution = tac_resolution
        self.alpha = 0.1
        self.gamma = 0.99
        
        if state_space_type == "grid":

            # grid state space
            self.NUM_state  = int(
                prod([(ROM[2*i+1] - ROM[2*i]) / pitch for i in range(DOF)])
                * ( 1.0/ tac_resolution )
            )
            self.NUM_action = 2 * DOF

            # discrete action
            self.action_list = [[0]*DOF for i in range(self.NUM_action)]
            for i in range(DOF):
                self.action_list[2*i][i]   = pitch
                self.action_list[2*i+1][i] = -pitch
            print self.action_list

            self.action_index_list = range(len(self.action_list))
            
            # constant for encoding, decoding index
            self.n = [int((rom_max - rom_min) / pitch)
                      for rom_max, rom_min in zip(ROM[1::2], ROM[0::2])]
            self.n +=[int(1 / tac_resolution) for i in range(TAC_NUM)]
            self.n.insert(0, 1)

            # index memory
            self.st_index = 0
            self.st_prime_index = 0
            self.at_index = 0

            # initializing table
            self.table_init()
            
            
    def table_init(self):
        self.table = np.zeros((self.NUM_state, self.NUM_action), dtype=np.float)
        

    def function(self, state_action_set):
        state  = state_action_set[0]
        action = state_action_set[1]
        return self.table[state][action]
    
        
    def updateQ(self, reward_function, *args):
        # row state should be given.
        #print self.state_decode(self.st_prime_index)
        reward = reward_function(self.state_decode(self.st_prime_index), *args)
        maxQ   = max(self.table[self.st_prime_index])
        self.table[self.st_index][self.at_index] += self.alpha * (reward + self.gamma * maxQ - self.table[self.st_index][self.at_index])
        return  reward


    def set_state(self, state):
        # update state information
        self.st_index       = self.st_prime_index
        self.st_prime_index = self.state_encode(state)


    def reset_index(self):
        self.st_index = 0
        self.at_index = 0
        self.st_prime_index = 0
        
        
    def state_encode(self, state):
        # encoding state to state_index
        # a < n0, b < n1, c < n2, d < n3
        # A = a*1 + b*n0 + c*n1*n0 + d*n2*n1*n0
        angle_state   = [int((angle - rom_min) / self.pitch * prod(self.n[:i+1]))
                         for angle, rom_min, i in zip(state, self.ROM[0::2], xrange(self.DOF))]
        tacti_state = [int(state[self.DOF] / self.tac_resolution) * prod(self.n[:i+1])
                       for i in xrange(self.DOF, self.DOF+self.TAC_NUM)]
        state_index   = sum(angle_state) + sum(tacti_state)

        #print state, state_index
        
        return state_index

        
    def state_decode(self, state_index):
        # decoding state_index to state
        # d = A % (n3*n2*n1*n0*1) / (n2*n1*n0*1)
        # c = A % (n2*n1*n0*1)    / (n1*n0*1)
        # b = A % (n1*n0*1)       / (n0*1)
        # a = A % (n0*1)          / 1
        
        tacti_state = [ int((state_index % prod(self.n[:len(self.n)-i])
                            / prod(self.n[:len(self.n)-i-1]))) * self.tac_resolution
                        for i in xrange(self.TAC_NUM) ]
        angle_state = [ int((state_index % prod(self.n[:len(self.n)-i])
                            / prod(self.n[:len(self.n)-i-1]))) * self.pitch + rom_min
                        for i, rom_min in zip(xrange(self.TAC_NUM, self.TAC_NUM+self.DOF), self.ROM[0::2])]
        
        tacti_state.reverse()
        angle_state.reverse()
        state = angle_state + tacti_state

        #print state_index, state
                        
        return state

        
    def action_encode(self):
        # encoding action to action_index
        pass

        
    def action_decode(self):
        # decoding action_index to action
        pass
