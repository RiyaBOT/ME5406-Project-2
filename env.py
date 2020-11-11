import numpy as np
import pyglet
import gym
from gym import spaces
from gym.spaces import MultiDiscrete
import random
import math
from PIL import Image
import cv2



# lengths of the links change as you please :)))) 
l1 = 100
l2 = 100

piano_len = 50
piano_wid = 10

# random pos for piano key to test step() 
key_x_start = 500
key_x_end   = 550
key_y_start = 50
key_y_end   = 60

# Actions
# 0 = +1 degree
# 1 = -1 degree
origin_pos_x=10
origin_pos_y=10
link1_pos_x= 0
link1_pos_y= 0
final_pos_y= 25
final_pos_x= 25
'''    ----------------- This is bullshit need to fix ----------------------
def set_piano(piano_x, piano_y):
    valid = False
    
    def isValid():
        # figure out how to determine if it is valid
        return True
        
    while not valid:
        pos_x = random.randint(0, size-piano_size)
        pos_y_start = random.randint(0, size-piano_size)
        pos_y_end = pos_y_start + piano_size
        valid = isValid()
        
    y_range = pos_y_end - pos_y_start
    for i in range (piano_size):
        grid[pos_x, pos_y_start+i] = 1 
    
    return grid, pos_x, pos_y_start, pos_y_end
'''    

class PanicEnv(gym.Env):
    
    metadata = {'render.modes': ['human', 'rgb_array']}  # i think rgb array is needed to render, if not then remove
    
    #grid, goal_x, goal_y_start, goal_y_end = set_piano(size, piano_size)
    
    def __init__(self):
        super(PanicEnv, self).__init__()
        self.reward_range = (-0.1, 1)                         # can change to whatever
        self.action_space = spaces.MultiDiscrete([2,2])     # Action space - increment or decrement the angles by 1
        self.observation_space = spaces.MultiDiscrete([180,180])   # Observation space - angles of theta1 and theta2 (180*180??)

        # storing current angles - random initialisation
        a, b = env.observation_space.sample()
        self.curr_angles = (a, b)
        
    def step(self, action):
        done = False
        reward = 0
        
        state = self.curr_angles
        theta1, theta2 = self.curr_angles
        (a1, a2) = action
        if (a1, a2) == (0, 0):
            theta1 += 1
            theta2 += 1
        elif (a1, a2) == (0, 1):
            theta1 += 1
            theta2 -= 1
        elif (a1, a2) == (1, 0):
            theta1 -= 1
            theta2 += 1
        elif (a1, a2) == (1, 1):
            theta1 -= 1
            theta2 -= 1
        
        # Convert to radians for calculations
        th1 = math.radians(theta1)
        th2 = math.radians(180 - theta2)
        
        ## !!!! All calculations are assuming the origin is at the start of link 1 !!!!! (change acc to render design)
        link1_pos = np.array([l1*np.sin(th1), l1 * np.cos(th1)])
        link1_pos_x= l1*np.sin(th1)
        link1_pos_y= l1 * np.cos(th1)
        final_pos = link1_pos + (np.array([(l2*np.cos(th2)) * np.sin(th1), (l2*np.cos(th2)) * np.cos(th1)]))
        final_pos_x= (l2*np.cos(th2)) * np.sin(th1)
        final_pos_y= (l2*np.cos(th2)) * np.cos(th1)
        self.curr_angles = theta1, theta2
        obs = self.curr_angles
        
        # Get reward and check if done  - change reward structure later
        if (final_pos[0] > key_x_start and final_pos[0] < key_x_end):
            # get some reward?

            if (final_pos[1] > key_y_start and final_pos[1] < key_y_end):
                reward = 1
                done = True

        return obs, reward, done, final_pos

    def reset(self):
        #define reset when theta1 = 45 deg and theta2 = 90 deg(somewhere in the default position that we always draw)
        self.curr_angles = (45, 90)
        return self.curr_angles
        
    def render(self):
        #how to graphically represent this 
        
        # image of the key
        img= Image.new("RGB", (1000, 1000))
        #img = np.zeros((500, 500, 3), dtype="uint8")
        pixels= img.load()
        for i in range (key_x_end- key_x_start):
            for j in range (key_y_end-key_y_start):
                pixels[i+key_x_start, j+key_y_start]= (255, 255, 255)
            
        
        
        # image of the robot link 1 
        for i in range (5):
            for j in range (5):
                pixels[link1_pos_x+i, link1_pos_y+j]= (255, 0, 0)
                
        
        # image for tip of finger
        for i in range (5):
            for j in range (5):
                pixels[final_pos_x+i, final_pos_y+j]= (255, 0, 0)
        
        # image for origin
        for i in range (5):
            for j in range (5):
                pixels[i, j]= (255, 0, 0)
                
        
        
        
        
        
       
        
        
        
        
                
        
        
                
                
        img.show()

        
        pass
      
        
env = PanicEnv()
env.reset()

env.reset()
for i in range(10):
    action = env.action_space.sample()
    print(action)
    next_state, reward, done, final_position = env.step(action) # next_state = angles at next state
    print(next_state, reward, done, final_position)
