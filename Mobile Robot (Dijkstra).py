#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import time
import copy


def workspace_check(x_new, y_new):

    """
        
        In this function the given value is checked to see if it lies in the workspace
        
        Parameters
        ----------
        x_new: X coordinate of given input 
        y_new: Y coordinate of given input 
        
        Returns
        -------
        True  : If the input lies inside workspace
        False : If the input lies outside workspace
        
    """
    
    if(x_new >= 0) and (y_new >= 0) and (x_new <= 400) and (y_new <= 300):
        return True
    else: 
        return False
    

def obstacle_space(x_new, y_new, r_robot=10, c_robot=5):
    
    """
        
        In this function the given value is checked to see if it lies in the obstacle space
        
        Parameters
        ----------
        x_new: X coordinate of given input 
        y_new: Y coordinate of given input
        r_robot: radius of mobile robot (default value = 10)
        c_robot: clearance of mobile robot (default value = 5)
        
        Returns
        -------
        True  : If the input lies outside obstacle space
        False : If the input lies inside obstacle space
        
    """
    
    distance = c_robot + r_robot
    
    flag1, flag2, flag3, flag4 = (True, True, True, True)
    
    # For Circle
    if (x_new - 90)**2 + (y_new - 70)**2 - (35 + distance)**2 < 0:
        flag1 = False
        
    # For Ellipse
    if ((x_new - 246)/(60 + distance))**2 + ((y_new - 145)/(30 + distance))**2 - 1 < 0:
        flag2 = False
        
    # For C-shape
    if (y_new > 230 - distance) and (y_new < 280 + distance) and (x_new > 200 - distance) and (x_new < 230 + distance):
        
        if (x_new <= 200) and (y_new >= 280) and ((x_new - 200)**2 + (y_new - 280)**2 >= distance**2):
            flag3 = True
        
        elif (x_new <= 200) and (y_new <= 230) and ((x_new - 200)**2 + (y_new - 230)**2 >= distance**2):
            flag3 = True
            
        elif (x_new >= 230) and (y_new >= 280) and ((x_new - 230)**2 + (y_new - 280)**2 >= distance**2):
            flag3 = True
            
        elif (x_new >= 230) and (y_new <= 230) and ((x_new - 230)**2 + (y_new - 230)**2 >= distance**2):
            flag3 = True
            
        elif (x_new >= 230) and (y_new <= 270) and (y_new >= 270 - distance):
            if (x_new - 230)**2 + (y_new - 270)**2 >= distance**2:
                flag3 = True
            else:
                flag3 = False
                
        elif (x_new >= 230) and (y_new >= 240) and (y_new <= 240 + distance):
            if (x_new - 230)**2 + (y_new - 240)**2 >= distance**2:
                flag3 = True
            else:
                flag3 = False
                
        elif (x_new >= 210) and (x_new <= 230) and (y_new <= 270) and (y_new >= 230):
            if (x_new >= 210 + distance) and (y_new <= 270 - distance) and (y_new >= 240 + distance):
                flag3 = True
            else:
                flag3 = False
                
        else:
            flag3 = False
    
    # For rotated rectangle
    x1, y1 = (48, 108)
    x2, y2 = (x1 - 20*np.cos(11*np.pi/36), y1 + 20*np.sin(11*np.pi/36))
    x4, y4 = (x1 + 150*np.cos(7*np.pi/36), y1 + 150*np.sin(7*np.pi/36))
    x3, y3 = (x4 - 20*np.cos(11*np.pi/36), y4 + 20*np.sin(11*np.pi/36))
    
    m_12 = (y2 - y1)/(x2 - x1)
    m_23 = (y3 - y2)/(x3 - x2)
    m_34 = (y4 - y3)/(x4 - x3)
    m_41 = (y1 - y4)/(x1 - x4)
    
    c_12 = y1 - m_12*x1
    c_23 = y2 - m_23*x2
    c_34 = y3 - m_34*x3
    c_41 = y4 - m_41*x4
      
    l_12 = lambda x, y: m_12*x - y + c_12
    l_23 = lambda x, y: m_23*x - y + c_23
    l_34 = lambda x, y: m_34*x - y + c_34
    l_41 = lambda x, y: m_41*x - y + c_41
    
    theta_12 = np.arctan(m_12)
    theta_23 = np.arctan(m_23)
    theta_34 = np.arctan(m_34)
    theta_41 = np.arctan(m_41)
    
    M_12, C_12 = (m_12, c_12 - distance*(np.cos(theta_12) + m_12*np.sin(theta_12)))
    M_23, C_23 = (m_23, c_23 + distance*(np.cos(theta_23) + m_23*np.sin(theta_23)))
    M_34, C_34 = (m_34, c_34 + distance*(np.cos(theta_34) + m_34*np.sin(theta_34)))
    M_41, C_41 = (m_41, c_41 - distance*(np.cos(theta_41) + m_41*np.sin(theta_41)))
    
    L_12 = lambda x, y: M_12*x - y + C_12
    L_23 = lambda x, y: M_23*x - y + C_23
    L_34 = lambda x, y: M_34*x - y + C_34
    L_41 = lambda x, y: M_41*x - y + C_41
    
    if (C_12*L_12(x_new, y_new) < 0) and (C_23*L_23(x_new, y_new) > 0) and (C_34*L_34(x_new, y_new) > 0) and \
            (C_41*L_41(x_new, y_new) < 0):
              
        if (c_12*l_12(x_new, y_new) >= 0) and (c_41*l_41(x_new, y_new) >= 0) and \
                ((x_new - x1)**2 + (y_new - y1)**2 >= distance**2):
            flag4 = True
            
        elif (c_23*l_23(x_new, y_new) <= 0) and (c_12*l_12(x_new, y_new) >= 0) and \
                ((x_new - x2)**2 + (y_new - y2)**2 >= distance**2):
            flag4 = True
            
        elif (c_34*l_34(x_new, y_new) <= 0) and (c_23*l_23(x_new, y_new) <= 0) and \
                ((x_new - x3)**2 + (y_new - y3)**2 >= distance**2):
            flag4 = True
            
        elif (c_41*l_41(x_new, y_new) >= 0) and (c_34*l_34(x_new, y_new) <= 0) and \
                ((x_new - x4)**2 + (y_new - y4)**2 >= distance**2):
            flag4 = True  
            
        else:
            flag4 = False
              
    flag = flag1 and flag2 and flag3 and flag4 
    
    return flag   

# -----------------------------------------Dijkstra Algorithm Implementation -------------------------------------------


def dijkstra(parent_node, parent_cost):
    
    x_parent, y_parent = node_states[parent_node]    

    # ------------------------------------------ Performing action sets ------------------------------------------------
    
    x_1, y_1, c_1 = (x_parent + 1, y_parent,     1.0)         # right action
    x_2, y_2, c_2 = (x_parent,     y_parent + 1, 1.0)         # top action
    x_3, y_3, c_3 = (x_parent - 1, y_parent,     1.0)         # left action
    x_4, y_4, c_4 = (x_parent,     y_parent - 1, 1.0)         # bottom action
    x_5, y_5, c_5 = (x_parent + 1, y_parent + 1, np.sqrt(2))  # top-right action
    x_6, y_6, c_6 = (x_parent - 1, y_parent + 1, np.sqrt(2))  # top-left action
    x_7, y_7, c_7 = (x_parent - 1, y_parent - 1, np.sqrt(2))  # bottom-left action
    x_8, y_8, c_8 = (x_parent + 1, y_parent - 1, np.sqrt(2))  # bottom-right action
    
    search_pt = [(x_1, y_1), (x_2, y_2), (x_3, y_3), (x_4, y_4), (x_5, y_5), (x_6, y_6), (x_7, y_7), (x_8, y_8)]
    search_travel_cost = [c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8]
    
    # ------------------------------------- Removing Invalid Points from search_pt -------------------------------------

    counter = 0
        
    while counter < len(search_pt):
        pt = search_pt[counter]
        is_valid = workspace_check(pt[0], pt[1]) and obstacle_space(pt[0], pt[1])
        
        if not is_valid:
            search_pt.pop(counter)
            search_travel_cost.pop(counter)
            
        else:
            counter += 1

    # ---------------------------- Appending nodes and travel cost to neighbor_nodes -----------------------------------
    
    neighbor_nodes = []
    
    for j, k in enumerate(search_pt):
        neighbor_node = np.intersect1d(np.where(X == k[0]), np.where(Y == k[1]))[0]
        traveling_cost_to_neighbor = search_travel_cost[j]
        neighbor_nodes.append((neighbor_node, traveling_cost_to_neighbor))

    # ----------------------- Removing Nodes from neighbor_nodes if present in visited_nodes ---------------------------
    
    counter = 0

    while counter < len(neighbor_nodes):

        j, k = neighbor_nodes[counter]

        if j in visited_nodes:
            neighbor_nodes.remove((j, k))

        else:
            counter += 1

    # ------------------------------ Updating Cumulative Cost of neighbor_nodes ----------------------------------------
    
    for j in neighbor_nodes:
        
        neighbor_node = j[0]
        traveling_cost_to_neighbor = j[1]
        
        neighbor_node_index = unvisited_nodes.index(neighbor_node)
        
        if unvisited_nodes_cost[neighbor_node_index] > parent_cost + traveling_cost_to_neighbor:
            unvisited_nodes_cost[neighbor_node_index] = parent_cost + traveling_cost_to_neighbor
            track.update({neighbor_node: parent_node})
            

# ----------------------------------- Finding all the valid nodes for the robot to move --------------------------------

X_ = np.arange(401)
Y_ = np.arange(301)
X, Y = np.meshgrid(X_, Y_)
X = X.flatten()
Y = Y.flatten()
Z = np.stack((X, Y)) 

nodes = []
node_states = {}
nodes_cost = []

for i in range(len(X)):
    if workspace_check(X[i], Y[i]) and obstacle_space(X[i], Y[i]):
        nodes.append(i)
        node_states.update({i: (X[i], Y[i])})
        nodes_cost.append(np.inf)
                            
# ---------------------- Taking User Input for Start Point and checking for its validity -------------------------------

print('\nEnter Start location (X_pt): ')
X_start = int(input())
print('Enter Start location (Y_pt): ')
Y_start = int(input())

start = (X_start, Y_start)

if workspace_check(X_start, Y_start):
    if not obstacle_space(X_start, Y_start):
        print('\n\nInvalid Start Point as it is in Obstacle/ Clearance Space\n\n')
        sys.exit('Exiting ....')

else:
    print('\n\nInvalid Start Point as it is not in Workspace\n\n')
    sys.exit('Exiting ....')
    
    
# ---------------------- Taking User Input for End Point and checking for its validity ---------------------------------

print('\nEnter End location (X_pt): ')
X_end = int(input())
print('Enter End location (Y_pt): ')
Y_end = int(input())

end = (X_end, Y_end)

if workspace_check(X_end, Y_end):
    if not obstacle_space(X_end, Y_end):
        print('\n\nInvalid End Point as it is in Obstacle/ Clearance Space\n\n')
        sys.exit('Exiting ....')
    
else:
    print('\n\nInvalid End Point as it is not in Workspace\n\n')
    sys.exit('Exiting ....')

# finds node numbers for start and end states       
start_node = list(node_states.keys())[list(node_states.values()).index(start)]
goal_node = list(node_states.keys())[list(node_states.values()).index(end)]

print('\nStart Node: ', start_node)
print('Goal Node: ', goal_node)

start_node_index = nodes.index(start_node)
nodes_cost[start_node_index] = 0.0  # assigning cost of start node to zero

visited_nodes = []
visited_nodes_cost = []
unvisited_nodes = copy.deepcopy(nodes)
unvisited_nodes_cost = copy.deepcopy(nodes_cost)
track = {}

iterator = 0

start_time = time.time()

print('\n\nSolving.........')
print('\nIteration # \t Time (mins.)\n')

while goal_node not in visited_nodes:
    unvisited_sorted = [(i, j) for i, j in sorted(zip(unvisited_nodes_cost, unvisited_nodes))]
    next_node_cost = unvisited_sorted[0][0]
    next_node = unvisited_sorted[0][1]
    next_node_index = unvisited_nodes.index(next_node)
    
    if iterator % 5000 == 0 and iterator != 0:
        mid_time = (time.time() - start_time)/60
        print(' {0} \t\t {1:1.3f}'.format(iterator, mid_time))        
    
    if len(unvisited_nodes) > 1:
        dijkstra(next_node, next_node_cost)
        unvisited_nodes.pop(next_node_index)
        unvisited_nodes_cost.pop(next_node_index)
        visited_nodes.append(next_node)
        visited_nodes_cost.append(next_node_cost)
        iterator += 1
    
    elif len(unvisited_nodes) == 1:
        unvisited_nodes.pop(next_node_index)
        unvisited_nodes_cost.pop(next_node_index)
        visited_nodes.append(next_node)
        visited_nodes_cost.append(next_node_cost)
        iterator += 1
    
end_time = time.time()
total_time = (end_time - start_time)/60
print('\n\n Number of iterations taken to reach goal state: {}'.format(iterator))
print('\n Time taken to find optimal (shortest) path: {0:1.3f} min'.format(total_time))

# Visited Node Exploration
x_explore = []
y_explore = []

for node in visited_nodes:
    x_pt, y_pt = node_states[node]
    x_explore.append(x_pt)
    y_explore.append(y_pt)

# Optimal solution trajectory
back_track = []


def traj(child):
    if child != start_node:
        back_track.append(child)
        parent = track[child] 
        return traj(parent)

    else:
        back_track.append(start_node)
        return back_track[::-1]


trajectory = traj(goal_node)

x_solution = []
y_solution = []

for node in trajectory:
    x_pt, y_pt = node_states[node]
    x_solution.append(x_pt)
    y_solution.append(y_pt)

# --------------------------------------- Visualization starts from here -----------------------------------------------

plt.style.use('seaborn-pastel')

fig = plt.figure()

ax = plt.axes(xlim=(0, 400), ylim=(0, 300))  # Defining Workspace limits

# For Plotting Circle
x_circle = np.linspace(55, 125, 2000)
y_circle1 = 70 + (35**2 - (x_circle - 90)**2)**0.5
y_circle2 = 70 - (35**2 - (x_circle - 90)**2)**0.5
ax.plot(x_circle, y_circle1, 'b.', markersize=0.15)
ax.plot(x_circle, y_circle2, 'b.', markersize=0.15)

# For Plotting Ellipse
x_ellipse = np.linspace(186, 306, 2000)
y_ellipse1 = 145 + 30*(1 - ((x_ellipse - 246)/60)**2)**0.5
y_ellipse2 = 145 - 30*(1 - ((x_ellipse - 246)/60)**2)**0.5
ax.plot(x_ellipse, y_ellipse1, 'b.', markersize=0.15)
ax.plot(x_ellipse, y_ellipse2, 'b.', markersize=0.15)

# For C-Shape (assuming uniform thickness)
ax.axhline(y=280, xmin=0.50, xmax=0.575, color='blue')
ax.axvline(x=200, ymin=23/30, ymax=14/15, color='blue')
ax.axhline(y=230, xmin=0.50, xmax=0.575, color='blue')
ax.axvline(x=230, ymin=23/30, ymax=0.80, color='blue')
ax.axhline(y=240, xmin=0.525, xmax=0.575, color='blue')
ax.axvline(x=210, ymin=0.8, ymax=0.9, color='blue')
ax.axhline(y=270, xmin=0.525, xmax=0.575, color='blue')
ax.axvline(x=230, ymin=0.9, ymax=14/15, color='blue')

# For rotated rectangle 
x1_, y1_ = (48, 108)
x2_, y2_ = (x1_ - 20*np.cos(11*np.pi/36), y1_ + 20*np.sin(11*np.pi/36))
x4_, y4_ = (x1_ + 150*np.cos(7*np.pi/36), y1_ + 150*np.sin(7*np.pi/36))
x3_, y3_ = (x4_ - 20*np.cos(11*np.pi/36), y4_ + 20*np.sin(11*np.pi/36))
ax.plot([x1_, x2_], [y1_, y2_], 'b-')
ax.plot([x2_, x3_], [y2_, y3_], 'b-')
ax.plot([x3_, x4_], [y3_, y4_], 'b-')
ax.plot([x1_, x4_], [y1_, y4_], 'b-')

new_node, = ax.plot([], [], 'y.', alpha=0.1)

solution_trajectory, = ax.plot([], [], 'r*') 


def animate(frame_number):
    
    """
        
        In this function, animation is carried out. 
                
        Parameters
        ----------
        frame_number : int type, here frame number serves as an index for the images  
        
        
        Returns
        -------
        None
        
    """
    
    frame_diff = total_frames - frame_number
          
    if frame_diff > 51:  # will run for frame_number = [0, 148]
        first = 0
        last = step1*(frame_number+1)
        x = x_explore[first:last]
        y = y_explore[first:last]
        new_node.set_data(x, y)
        new_node.set_markersize(1)
        return new_node,

    elif frame_diff == 51:  # will run for frame_number = 149 only
        x = x_explore
        y = y_explore
        new_node.set_data(x, y)
        new_node.set_markersize(1)
        return new_node,
    
    elif 51 > frame_diff > 1:  # will run for frame_number = [150, 198]
        first = 0
        last = step2*(frame_number-149)
        x = x_solution[first:last]
        y = y_solution[first:last]  
        solution_trajectory.set_data(x, y)
        solution_trajectory.set_markersize(1.5)
        return solution_trajectory,
        
    else:  # will run for frame_number = 199 only
        x = x_solution
        y = y_solution   
        solution_trajectory.set_data(x, y)
        solution_trajectory.set_markersize(1.5)
        return solution_trajectory,


node_explore_frames = 150
solution_traj_frames = 50  
total_frames = node_explore_frames + solution_traj_frames

step1 = int(len(x_explore)/node_explore_frames)
step2 = int(len(x_solution)/solution_traj_frames)

animation = FuncAnimation(fig, animate, frames=total_frames, interval=30, blit=True, repeat=False)

animation.save('Mobile Robot Visualization (Dijkstra).mp4', dpi=300)

plt.close()

print('\n\n### Writing Visited Nodes to an Output File ###')
fname1 = './Visited_Nodes_Mobile_Robot_Dijkstra.txt'
myfile1 = open(fname1, "w")
myfile1.write('Node # \t X \t Y \t Cumulative Cost\n')

for node, x_pt, y_pt, final_cost in zip(visited_nodes, x_explore, y_explore, visited_nodes_cost):
    myfile1.write('{0} \t {1:1.3f} \t {2:1.3f} \t {3:1.3f}\n'.format(node, x_pt, y_pt, final_cost))

print('\nCompleted !!!')

print('\n\n### Writing the solution trajectory to an Output File ###')

fname2 = './Solution_Path_Mobile_Robot_Dijkstra.txt'
myfile2 = open(fname2, "w")
myfile2.write('Time taken to solve:\t{0:1.3f} minutes\n'.format(total_time))
myfile2.write('Required Solution trajectory\n')
myfile2.write('\nNode # \t X \t Y\n')

for node in trajectory:
    x_pt, y_pt = node_states[node]
    myfile2.write('{0} \t {1:1.3f} \t {2:1.3f}\n'.format(node, x_pt, y_pt))
    
print('\nCompleted !!!')
