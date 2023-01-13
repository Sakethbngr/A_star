
import numpy as np
import math
import cv2 as cv

from math import dist

#Map Space
map = np.ones((250, 400, 3), np.uint8)*255
thresh = 0.5
x_grid = np.arange(0, 400, thresh)
y_grid = np.arange(0, 250, thresh)
theta_grid = np.arange(0, 360, 30)

# Creating nodes and storing in a dictionary
def get_node( pos, theta, parent, cost):
    Node  = {'pos': pos,
             'theta': theta, 
             'parent': parent, 
             'cost': cost}
    return Node

def initial_nodes(start_key):
  open_dict = {}
  for x in x_grid: 
    for y in y_grid:  
        pos = (x, y)
        for theta in theta_grid:  
            open_dict[(pos, theta)] = get_node(pos, theta, None, np.inf)
  open_dict[start_key]['cost'] = 0
  return open_dict

def check_goal(child_node, goal_pos, theta_goal):
    dst = dist(child_node['pos'], goal_pos)
    
    dtheta = np.abs(child_node['theta'] - theta_goal)
    # print(dtheta, child_node['theta'], theta_goal)
    if dst < 1.5 and dtheta <= 30: 
        return  True
    else: 
        return False


#Hexagon points
hexagon = np.array([[240, 80], [240, 120], [200, 145],[160, 120], [160, 80], [200, 55], [240, 80]], np.int32)
hex_center = np.array([200, 100])

##The Polygon is split up into two triangles
triangle1 = np.array([[31,185], [115,215], [85,180], [31,185]])
triangle1center = np.mean(triangle1[:-1], axis = 0)

triangle2 = np.array([[31,185], [105,95], [85,180], [31,185]])
triangle2center = np.mean(triangle2[:-1], axis = 0)

hex_img = np.array([[240, 129.79], [240, 170.20], [200, 195.41],[160, 170.20], [160, 129.79], [200, 104.58]], np.int32)


poly_img = np.array([[31,65], [115,35], [85, 70], [105,155]], np.int32)

cv.circle(map, (300, 65), 40, (0, 0, 0), thickness = -1)


cv.fillPoly(map, [hex_img],(0,0,0))
cv.fillPoly(map, [poly_img], (0,0,0))

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

print("Solution started. Estimated wait time 1 minute")

###Defining obstacle space 
def obstacle(pt):
    x, y = pt

    if (x < 0) or (x >= 400) or (y < 0) or (y >= 250):
        return True

    if np.sqrt((x - 300)**2 + (y - 185)*2) <= 45 :
        return True
    
    ret = False
    for i in range(len(hexagon) - 1):
        ret = ret or intersect(pt, hex_center, hexagon[i], hexagon[i+1])
    if not ret: 
        return True

    ret = False
    for i in range(len(triangle1) - 1):
        ret = ret or intersect(pt, triangle1center, triangle1[i], triangle1[i+1])
    if not ret: 
        return True
    
    ret = False
    for i in range(len(triangle2) - 1):
        ret = ret or intersect(pt, triangle2center, triangle2[i], triangle2[i+1])
    if not ret: 
        return True    
    return False


def Action(node, theta, step_size):
    x, y = node['pos']
    new_theta = (node['theta'] + theta)%360
    x_new = step_size*np.cos(new_theta*np.pi/180) + x # - (y-0)*np.sin(np.deg2rad(degree)) + length
    y_new = step_size*np.sin(new_theta*np.pi/180) + y # + (y-0)*np.cos(np.deg2rad(degree)) + length
    new_pos = ((x_new//thresh)//2, (y_new//thresh)//2)   ###############recheck int vs round############
    return new_pos, new_theta, node['cost'] + 1

######give the input points over here
x_s = int(input('Give x-co-ordinate of the start position: '))
y_s = int(input('Give y-co-ordinate of the start position: '))
x_g = int(input('Give x-co-ordinate of the goal position: '))
y_g = int(input('Give y-co-ordinate of the goal position: '))
theta_start = int(input('Give initial orientation: '))
while (theta_start/30 != 0) and theta_start not in range (-60, 61):
  print('Invalid entry')
  theta_start = int(input('Give initial orientation: '))

theta_goal = int(input('Give goal orientation: '))
while (theta_goal/30 != 0) and theta_goal not in range (-60, 61):
  print('Invalid entry')
  theta_goal = int(input('Give initial orientation: '))
step_size= int(input('Give step size: '))
while step_size not in range(0, 11):
  print('step size should be in the range of 0 to 10. Enter Again')
  step_size= int(input('Give step size: '))

start_pos = (x_s, y_s)
goal_pos = (x_g, y_g)

all_thetas = [-60, -30, 0, 30, 60]
start_key = (start_pos, theta_start)
All_nodes = initial_nodes(start_key)


##A Star Algorithm
def A_star(goal_pos, theta_goal):
    open_dict = {start_key: dist(start_pos, goal_pos)}
    closed_lis = {start_key}
    explore = [All_nodes[start_key]]

    while len(open_dict):
        key = min(open_dict, key = open_dict.get)
        closed_lis.add(key)
        open_dict.pop(key)
        min_node = All_nodes[key]
        if check_goal(min_node, goal_pos, theta_goal): 
            print("solution found")
            return backtrack(child_node), explore
        # gives the minimum position of nodes
        for theta_t in all_thetas:
            pos, theta, cost = Action(min_node, theta_t, step_size)
            if not obstacle(pos) and (pos, theta) not in closed_lis:
                child_node = All_nodes[(pos, theta)]
                if cost < child_node['cost']:
                    child_node['cost'] = cost
                    child_node['parent'] = min_node
                    open_dict[(pos, theta)] = cost + dist(pos, goal_pos)
                    explore.append(child_node)
    # return None

#######BackTracking
def backtrack(node):
    print("Tracking Back")
    path = []
    while node['parent'] is not None:
        path.append(node)
        node = node['parent']
    path.reverse()
    return path

####A = Backtracking
A, explore = A_star(goal_pos, theta_goal)

#visulizing the path explored nodes
def visualize(path, explored, name = 'result'):
        h, w, _ = map.shape
        img = cv.UMat(map)
        # open video writerd
        out = cv.VideoWriter(f'{name}.mp4', cv.VideoWriter_fourcc(*'mp4v'), 30.0, (w, h))
        # visualize exploration
        k = 0
        for node in explored:
            print(node['pos'], node['theta'])
            i, j = node['pos']
            parent = node['parent']
            if parent is None: parent = path[0]
            ip, jp = parent['pos']
            cv.arrowedLine(img, (int(i), int(249 - j)), (int(ip), int(249 - jp)), [0, 80 ,0], 2)
            if k%20 == 0:
                out.write(img)
                cv.imshow('Exploration', img)
                cv.waitKey(1)
            k += 1
            
        # visualise path
        if path is not None:
            for node in path:
                i, j = node['pos']
                parent = node['parent']
                if parent is None: parent = path[0]
                ip, jp = parent['pos']
                cv.arrowedLine(img, (int(i), int(249 - j)), (int(ip), int(249 - jp)), [0, 0, 255], 2)
            for i in range(100): out.write(img)
        out.release()
        #show final image and save video
        cv.imshow('Exploration', img)
        cv.imwrite('final.jpg', img)
        print('\npress any key to exit')
        cv.waitKey(0)
        print(f'Video saved as {name}.mp4')
            

visualize(A, explore)
# print("")



# cv.imshow('Output',map )
# cv.waitKey(0)


