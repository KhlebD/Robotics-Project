#!/usr/bin/env python3
# license removed for brevity
# from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
import os
import rospy
import actionlib
import time
from future.standard_library import exclude_local_folder_imports
from geometry_msgs.msg import Point
from task3_env.srv import navigate, navigateResponse, pick, pickResponse, place, placeResponse, info, infoResponse  #
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from environment_functions import init
import numpy as np

toys = ["green", "blue", "black", "red"]
rewards = [10, 20, 10, 40]
loc_init = [[0.1, 0.05, 0.8, 0.05], [0.7, 0.1, 0.1, 0.1], [0.25, 0.25, 0.25, 0.25], [0.25, 0.25, 0.25, 0.25]]
reward_init = [[0.8, 0.05, 0.1, 0.05], [0.1, 0.7, 0.1, 0.1], [0.25, 0.25, 0.25, 0.25], [0.25, 0.25, 0.25, 0.25]]
loc_values = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]
reward_values = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]]

def calc_reward_values():

    global reward_init
    global reward_values
    #green reward values
    for i in range(4):
        reward_values[0][i] = reward_init[0][i]

    #blue reward values
    for j in range(4):
        for k in range(4):
            if(k != j):
                x = reward_init[0][k] * (reward_init[1][j] + reward_init[1][k] / 3.0)
                reward_values[1][j] += x

    #black + red reward values
    for i in range(4): 
        reward_values[2][i] = (1 - (reward_values[0][i] + reward_values[1][i]))/ 2
        reward_values[3][i] = reward_values[2][i]

    
def calc_loc_values():

    global reward_init
    global reward_values
    #green reward values
    for i in range(4):
        loc_values[0][i] = loc_init[0][i]

    #blue reward values
    for j in range(4):
        for k in range(4):
            if(k != j):
                x = loc_init[0][k] * (loc_init[1][j] + loc_init[1][k] / 3.0)
                loc_values[1][j] += x

    #black + red reward values
    for i in range(4): 
        loc_values[2][i] = (1 - (loc_values[0][i] + loc_values[1][i]))/ 2
        loc_values[3][i] = loc_values[2][i]

def calc_next_step():
    #calc best toy and its location
    max_value = 0
    best_nav_location = 0
    best_toy_reward = 0


    for i in range(4):
        for j in range(4):
            current_value = reward_values[i][j] * loc_values[i][j] * rewards[j]
            if(max_value < current_value):
                max_value = current_value
                best_nav_location = j
                best_toy_reward = i
    return [best_nav_location, best_toy_reward]

def update_values(matrix_values, toy, location, found):
    
    #in case a toy was found
    if(found == 1):
        #update the matrix values:
        for i in range(4):
            for j in range(4):
                if(i != toy and j!= location):
                    div_value = 1 - matrix_values[toy][j]
                    matrix_values[i][j] =  matrix_values[i][j] / div_value

        #update the toy itself:
        for i in range(4):
            matrix_values[toy][i] = 0
            matrix_values[i][location] = 0
        matrix_values[toy][location] = 1

    mult_values = [0,0,0,0]

    if(found == 0):
        
        #get div values + update toy row
        for i in range(4):
            if(i != location):
                new_value = matrix_values[toy][i]/ (1 - matrix_values[toy][location])
                mult_values[i] = 1 - (new_value - matrix_values[toy][i])
                matrix_values[toy][i] = new_value
            else:
                mult_values[i] = matrix_values[toy][i]

        matrix_values[toy][location] = 0                   
        #update rest of values
        for i in range(4):
            for j in range(4):
                if(i != toy):
                    if(j != location):
                        
                        matrix_values[i][j] = matrix_values[i][j] * mult_values[j]
                    else:
                        matrix_values[i][j] = matrix_values[i][j] / (1 - mult_values[j])   
            

def navigate_client(location):
    rospy.wait_for_service('navigate')
    try:
        nav = rospy.ServiceProxy('navigate', navigate)
        resp1 = nav(location)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pick_client(color):
    rospy.wait_for_service('pick')
    try:
        pic = rospy.ServiceProxy('pick', pick)
        resp1 = pic(color)
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def place_client():
    rospy.wait_for_service('place')
    try:
        plac = rospy.ServiceProxy('place', place)
        resp1 = plac()
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':

    num_of_picks = 6
    num_of_toys = 4
    is_holding = False
    #Step 1: do all calcualtions:
    calc_loc_values()
    calc_reward_values()

    #Step 2: go to work:
    while((num_of_picks > 0 or is_holding) and num_of_toys > 0):
        
        #Step 2.1: choose best toy:
        best = calc_next_step()

        #Step 2.2: go to best toy
        navigate_client(best[0])
        time.sleep(1)


        pick_success = pick_client(toys[best[1]])
        num_of_picks -= 1
        #Step 2.2.1: pick failed
        if(pick_success == False ):
            update_values(loc_values, best[1], best[0], 0)

        #Step 2.2.1: pick success  
        else:
            update_values(loc_values, best[1], best[0], 1)
            is_holding = True
            navigate_client(4)
            time.sleep(1)
            place_client()
            num_of_toys -= 1
            is_holding = False
            update_values(reward_values, best[1], best[0], 0)






