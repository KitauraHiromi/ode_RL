# -*- coding:utf-8 -*- 
import numpy as np
import re          # regular expression
import os
from tcp import *
import math
from struct import *
from policy import discrete_epsiron_greedy_policy
from policy import policy_from_state_density
from qtable import Q_table
from reward import tactile_reward
from reward import no_reward

LOCALHOST = '127.0.0.1'
PORT = 50007
INSENSITIVITY = 0.5

    
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


    def run(self):
        #omega = math.pi/360
        with open(self.write_file, 'w') as wf:
            while True:
                self.recieve_state()
                self.agent_reward += self.Q.updateQ(self.reward_function)
                # self.output(wf)
                self.send_action()
                
        
    def load_configure(self, recieve):
        # setting based on the first communication.
        # robot configuration
        self.DOF     = int(recieve[0])
        self.TAC_NUM = int(recieve[1])
        self.pitch   = float(recieve[2])
        self.ROM     = map(float, recieve[3:self.DOF*2+3])
        print self.DOF, self.TAC_NUM, self.pitch, self.ROM

        tactile_reward.target_tac_index = self.DOF + 0
        self.reward_function = no_reward
        self.policy = discrete_epsiron_greedy_policy
        self.agent_reward = 0
        self.Q = Q_table(self.DOF, self.TAC_NUM, self.pitch, self.ROM, self.tac_resolution, state_space_type="grid")
        

    def restart(self):
        # printing and initializing reward 
        mean_reward = self.agent_reward / 10000
        print mean_reward
        self.agent_reward = 0

        # initializing st_index, st_prime_index and at_index
        self.Q.reset_index()
        
        
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
            self.restart()
            self.recieve_state()            
        elif re.match('FIRST_ITER_SIGNAL', self.data):
            recieve = self.data.split(',')[1:]
            self.load_configure(recieve)
            self.recieve_state()
        else:
            recieve = map(float, self.data.split(','))

            # memory angles and tactiles
            # note that learner has row state data.
            self.angles = recieve[1:self.DOF+1]
            self.tactiles = map(tactile_value_mapping, recieve[self.DOF+1:self.DOF+self.TAC_NUM+1])
            state = self.angles + self.tactiles

            # note that Q_table has only index information.
            self.Q.set_state(state)

        
    def send_action(self):
        # apply action
        # note that action is an angle list
        action = self.policy(self.Q)
        angles_new = [angle + delta_angle for angle, delta_angle in zip( self.angles, action )]
        
        # If all elements in angles_new are lower than self.ROM, then apply angles_new.
        n, signal = self.angle_check(angles_new)
    
        if n >= 0 and signal == "OVER":
            pass
        elif n >= 0 and signal == "BELOW":
            pass
        else:
            self.angles = angles_new
        print self.angles
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
