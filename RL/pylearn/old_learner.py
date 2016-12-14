# -*- coding:utf-8 -*- 
import numpy as np
import re          # regular expression
import os
from tcp import *
import math
from struct import *
from policy import *

LOCALHOST = '127.0.0.1'
PORT = 50007
#filename = 'config_arm_robot1.txt'
INSENSITIVITY = 0.5

def prod(float_list):
    return reduce(lambda x,y: x*y, float_list)

    
def tactile_value_mapping(val):
    # mapping of R (-inf. < R < inf.)  -> R (0 <= R <1)
    if val < 0:
        return math.exp(INSENSITIVITY / val)
    else:
        return 0
        
class Learner(Server):
    def __init__(self):
        super(Learner, self).__init__(LOCALHOST, PORT)
        self.write_file = "state_action.log"
        self.tac_resolution = 0.1
        self.alpha = 0.1
        self.gamma = 0.99
        
        self.reward_function = self.tactile_reward
        self.policy = Epsiron_Greedy
        self.reward = 0


    def run(self):
        #omega = math.pi/360
        with open(self.write_file, 'w') as wf:
            while True:
                self.recieve_state()
                self.updateQ()
                # self.output(wf)
                self.set_action()
                self.send_action()

        
    def restart(self):
        self.st_index       = 0
        self.st_prime_index = 0
        self.at_index       = 0
        # self.angles         = self.initial_angles
        # self.tactiles       = self.initial_tactiles

        
    def load_configure(self, recieve):
        # setting based on the first communication.
        # robot configuration
        self.DOF     = int(recieve[0])
        self.TAC_NUM = int(recieve[1])
        self.pitch   = float(recieve[2])
        #self.initial_angles   = map(float, recieve[2:self.DOF+2])
        self.ROM     = map(float, recieve[3:self.DOF*2+3])
        #self.initial_tactiles = map(tactile_value_mapping, map(float, recieve[self.DOF*2+2:]))
        print self.DOF, self.TAC_NUM, self.pitch, self.ROM

        # the number of state, action
        self.NUM_state  = int(
            prod([(self.ROM[2*i+1] - self.ROM[2*i]) / self.pitch for i in range(self.DOF)])
            * ( 1.0/ self.tac_resolution )
        )
        self.NUM_action = 2 * self.DOF
        
        # action description: DOF*NUM_actionの2次元配列
        self.action_list = [[0]*self.DOF for i in range(self.NUM_action)]
        for i in range(self.DOF):
            self.action_list[2*i][i]   = self.pitch
            self.action_list[2*i+1][i] = -self.pitch
        print self.action_list

        # Q table
        self.Q_table = np.zeros((self.NUM_state, self.NUM_action), dtype=np.float)
        print (self.alpha, self.gamma, self.DOF, self.pitch, self.NUM_state, self.NUM_action)

        # constant for encoding, decoding index
        self.n = [int((rom_max - rom_min) / self.pitch)
                  for rom_max, rom_min in zip(self.ROM[1::2], self.ROM[0::2])]
        self.n +=[int(1 / self.tac_resolution) for i in range(self.TAC_NUM)]
        self.n.insert(0, 1)
        

    def Q_function(self, state_action_set):
        state  = state_action_set[0]
        action = state_action_set[1]
        return self.Q_table[state][action]

        
    def updateQ(self):
        # row state should be given.
        #print self.state_decode(self.st_prime_index)
        reward = self.reward_function(self.state_decode(self.st_prime_index))
        maxQ   = max(self.Q_table[self.st_prime_index])
        self.Q_table[self.st_index][self.at_index] += self.alpha * (reward + self.gamma * maxQ - self.Q_table[self.st_index][self.at_index])
        self.reward += reward

        
    def tactile_reward(self, state):
        # If tactile value is near target, the reward will be high.
        target = 0.5
        try:
            tactile_reward = 100 * state[self.DOF]
        except ZeroDivisionError:
            tactile_reward = 100
        return min(tactile_reward, 100)

        
    def angle_reward(self, state):
        # If tactile value is near target, the reward will be high.
        target = 90
        try:
            angle_reward = 1 / (target - state[0])
        except ZeroDivisionError:
            angle_reward = 100
        # print angle_reward, state
        return min(angle_reward, 100)

        
    def action_encode(self):
        # encoding action to action_index
        pass

        
    def action_decode(self):
        # decoding action_index to action
        pass

        
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

        
    def print_reward(self):
        mean_reward = self.reward / 10000
        print mean_reward
        self.reward = 0
        
        
    def angle_check(self, angles_new):
        # should be more readable
        for i in range(len(angles_new)):
            if self.ROM[2*i+1] * 4 / 5 > angles_new[i] > self.ROM[2*i] * 4 / 5:
                pass
            elif angles_new[i] >= self.ROM[2*i+1] * 4 / 5:
                return i, 'OVER'
            else:
                return i, 'BELOW'
        return -1, 'BETWEEN'

        
    def recieve_state(self):
        self.recieve()
        
        # communication protocol should be written in config file.
        # signal is string. may be ought to be modified.
        if re.match('NEXT_ITER_SIGNAL', self.data):
            self.print_reward()
            self.restart()
            self.recieve_state()            
        elif re.match('FIRST_ITER_SIGNAL', self.data):
            recieve = self.data.split(',')[1:]
            self.load_configure(recieve)
            self.restart()
            self.recieve_state()
        else:
            recieve = map(float, self.data.split(','))
            self.angles = recieve[1:self.DOF+1]
            self.tactiles = map(tactile_value_mapping, recieve[self.DOF+1:self.DOF+self.TAC_NUM+1])
            state = self.angles + self.tactiles
            
            # update state information
            self.st_index       = self.st_prime_index
            self.st_prime_index = self.state_encode(state)

        
    def set_action(self):
        action_index_list = [i for i in range(self.NUM_action)]
        self.at_index = self.policy(self.Q_function, self.st_index, action_index_list)

        
    def send_action(self):
        # apply action
        action = self.action_list[self.at_index]
        angles_new = [angle + delta_angle for angle, delta_angle in zip( self.angles, action )]
        
        # If all elements in angles_new are lower than self.ROM, then apply angles_new.
        n, signal = self.angle_check(angles_new)
        if n >= 0 and signal is 'OVER' and action[n] is 5:
            print 'angle over'
        elif n >= 0 and signal is 'BELOW' and action[n] is -5:
            print 'angle below'
        else:
            self.angles = angles_new
            
        # communication protocol should be written in config file.
        send_buf = str(reduce(lambda x,y: str(x) + ' ' + str(y), self.angles))
        self.send(send_buf)


    def output(self, wf):
        string  = ""
        for angle in  map(str, self.angles):
            string += angle
            string += ' '
        for tactile in  map(str, self.tactiles):
            string += tactile
            string += ' '
        string += str(self.at_index)
        wf.write(string)

                
if __name__ == '__main__':
    learner = Learner()
    learner.run()
