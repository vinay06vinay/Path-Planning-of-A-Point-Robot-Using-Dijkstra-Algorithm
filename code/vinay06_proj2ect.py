import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq as hq
import time
import pygame

def obstacle_space(width,height,canvas):
    obstacle_matrix = {}
    '''
    Creating a obstacle space with clearance of 5mm in the map and 
    initialising all the coordinates in obstacle space with a cost of -1 and 
    all coordinates in free space with a cost of 'inf' initially
    '''
    for x in range(width):
        for y in range(height):
            # #First Rectangle
            r1 = (x>=95 and x<=155) and (y>=0 and y<=105)
            #second Rectangle
            r2 = (x>=95 and x<=155) and (y>=145 and y<=250)
            #Hexagon, representing as sides 's'
            s1 = (0.577*x-y+32.57>=0)
            s2 = (x-230.04>=0)
            s3 = (y+0.57*x-217.14>=0)
            s4 = (0.57*x-y-128.96<=0)
            s5 = (x-369.95 <=0)
            s6 = (y+0.57*x-378.62 <=0)
            #triangle
            l1 = (x-455>=0)
            l2 = (2.01*x- y-915.21<=0)
            l3 = (2.01*x+y-1163.58<=0)
            if(r1 or r2 or (s1 and s2 and s3 and s4 and s5 and s6) or (l1 and l2 and l3)):
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,0,255)
            #Creating Boundary with clearance of 5mm
            elif(((y-5 <=0) or (y-245 >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,255,255)
            elif(((x-5 <=0) or (x-595 >=0))) :
                obstacle_matrix[(x,y)] = -1
                canvas[y,x] = (0,255,255)
            else:
                obstacle_matrix[(x,y)] = math.inf
    return obstacle_matrix,canvas
'''
Get Inputs until valid ones are typed on the console
'''
def get_input(obstacle_matrix):
    
    while True:
        print("Enter the x-coordinate of start node")
        start_x = int(input())
        print("Enter the y-coordinate of start node")
        start_y = int(input())
        if((start_x <0 or start_x >600) ):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif((start_y<0 or start_y>250)):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(start_x,start_y)] == -1):
            print("The entered start node falls in obstacle space")
            print("Please enter a start node values in range x = (5-595) and y=(5,245)")
        else:
            break
    while True:
        print("Enter the x-coordinate of goal node")
        goal_x = int(input())
        print("Enter the y-coordinate of goal node")
        goal_y = int(input())
        if(goal_x <0 or goal_x >600):
            print("The X-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(goal_y<0 and goal_y>250):
            print("The Y-coordinate of start node is out of the map. Please enter the coordinates again")
        elif(obstacle_matrix[(goal_x,goal_y)] == -1):
            print("The entered goal node falls in obstacle space")
            print("Please enter a start node values in range x = (5-595) and y=(5,245)")
        else:
            break
    return (start_x,start_y,goal_x,goal_y)
'''
8 Different Actions can be performed for each new node according to the 8 connected space
Each of the action function below updates whether the node is in obstacle space and returns coordinates of new nodes along with new_cost
'''
def move_left(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1
    return (flag,new_cost,new_node)
def move_right(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1
    return (flag,new_cost,new_node)
def move_top(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1
    return (flag,new_cost,new_node)
def move_bottom(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1
    return (flag,new_cost,new_node)
def move_top_right(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1.4
    return (flag,new_cost,new_node)
def move_top_left(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1.4
    return (flag,new_cost,new_node)
def move_bottom_right(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1.4
    return (flag,new_cost,new_node)
def move_bottom_left(current_cost,action_value,current_node):
    new_node = (current_node[0]+action_value[0],current_node[1]+action_value[1])
    flag=True
    if(obstacle_matrix[new_node] == -1):
        flag = False
    new_cost = current_cost+1.4
    return (flag,new_cost,new_node)
def dijkstra(start_x,start_y,goal_x,goal_y):
    visited_nodes = []
    closed_list = []
    open_list = []
    visited_nodes.append((start_x,start_y))
    child_parent_dict = {}
    cost = 0
    parent_node = (start_x,start_y)
    obstacle_matrix[parent_node] = cost
    hq.heapify(open_list)
    hq.heappush(open_list,[cost,parent_node,(start_x,start_y)])
    count = 0
    goal_reached = False
    while(len(open_list)>0):
        cn=hq.heappop(open_list)
        current_node = cn[2]
        current_cost = cn[0]
        if(goal_reached):
            print("Goal Reached, Starting Back Track")
            break
        closed_list.append(current_node)
        action_set = {move_top:(0,-1),move_bottom:(0,1),move_left:(-1,0),move_right:(1,0),move_top_left:(-1,-1),move_top_right:(1,-1),move_bottom_left:(-1,1),move_bottom_right:(1,1)}
        '''
        For each action, the new node is checked if its in freespace and not in closed list.
        1. If in free space, the new node is pushed to heapQ i.e open list
        2. If, the new node is present in the visited space already, then value of the cost in open list is updated if new cost is less than
           current cost of that node in open list
        '''
        for action,action_value in action_set.items():
            flag,new_cost,new_node = action(current_cost,action_value,current_node)
           
            if(flag):
                if(new_node not in closed_list):
                    if(obstacle_matrix[new_node] == math.inf):
                        obstacle_matrix[new_node] = new_cost
                        hq.heappush(open_list,[new_cost,current_node,new_node])
                        visited_nodes.append(new_node)
                        hq.heapify(open_list)
                        child_parent_dict[new_node] = current_node
                        if(new_node == (goal_x,goal_y)):
                            goal_reached = True
                            break
                    elif (new_node in visited_nodes):
                        node = [i for i in open_list if i[2] == new_node]
                        if(new_cost < round(node[0][0],1)):
                            node[0][0] = new_cost
                            node[0][1] = current_node
                            node[0][2] = new_node
                            hq.heapify(open_list)
                            child_parent_dict[new_node] = current_node
                            obstacle_matrix[new_node] = new_cost

    count += 1
    return(child_parent_dict,goal_reached,closed_list)
'''
Back Tracking with a dictionary where parent vs child node mapping is done while traversing the open list
'''
def back_track(child_parent_dict,start,goal):
    optimal_list = []
    optimal_list.append(goal)
    current = goal
    while(start!=current):
        optimal_list.append(child_parent_dict[current])
        current = child_parent_dict[current]
    print(optimal_list)
    return optimal_list[::-1]
    # for key,value in child_parent_dict.items():
'''
Final Plot using OpenCV of the map with visited nodes, optimal path, obstacle space
'''
def plot(canvas,optimal_path,visited):
    for i in visited:
        canvas[i[1],i[0]] = (0,0,0)
    for i in optimal_path:
        canvas[i[1],i[0]] = (200,255,0)
    plt.imshow(canvas,origin='lower')
'''
Pygame is used in Visualising the Visited Nodes and thereafter the drawing the shortest path
'''
def visualise_pygame(optimal_path,visited):
    screen =  pygame.Surface((600, 250))
    screen.fill((255, 255, 255))
    pygame.draw.rect(screen, (180, 250, 0), pygame.Rect(0, 0,600, 250),5)
    pygame.draw.rect(screen, (0, 0, 150), pygame.Rect(100, 0,50, 100))
    pygame.draw.rect(screen, (0, 0, 150), pygame.Rect(100, 150,50, 100))
    pygame.draw.rect(screen, (180, 250, 0), pygame.Rect(95, 0,60, 105),5)
    pygame.draw.rect(screen, (180, 250, 0), pygame.Rect(95, 145,60, 105),5)
    pygame.draw.polygon(screen, (0, 0, 150), ((235,87.5), (235, 162.5), (300, 200), (364.95, 162.5), (364.95, 87.5),(300,50)))
    pygame.draw.polygon(screen, (180, 250, 0), ((230.04,84.61), (230.04, 165.38), (300, 205.77),(369.95,165.38),  (369.95, 84.61),(300, 44.22)),5)
    pygame.draw.polygon(screen, (0, 0, 150), ((460,25), (460, 225), (510, 125)))
    pygame.draw.polygon(screen, (180, 250, 0), ((455,3.8196), (455, 246.1803), (515.5901, 125)),5)
    pygame.display.set_caption('Path Planner Using Dijkstra')
    
    s1 = pygame.display.set_mode((600, 250))
    # Blit the surface onto the Pygame window
    s1.blit(screen, (0, 0))
    pygame.display.update()
    #First Showing all Visited Nodes
    for i in visited:
        s1.set_at(i,(0,0,0))
        pygame.display.update()
    #Drawing the shortest path
    for i in range(len(optimal_path) - 1):
        pygame.draw.polygon(s1, (200, 0, 200), (optimal_path[i],optimal_path[i+1]),5)
        pygame.time.wait(10)
        pygame.display.update()
    while True:
        events = pygame.event.get()
        keys = pygame.key.get_pressed()
        ctrl_pressed = keys[pygame.K_LCTRL] or keys[pygame.K_RCTRL]
        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q and ctrl_pressed:
                    pygame.quit()
                    return
    
    
start_time = time.time()
canvas = np.ones((250,600,3))
obstacle_matrix,canvas = obstacle_space(600,250,canvas)    
start_x,start_y,goal_x,goal_y = get_input(obstacle_matrix)
# start_x,start_y,goal_x,goal_y = 6,6,100,110
print(f"The start node selected : {(start_x,start_y)}. The goal node selected : {(goal_x,goal_y)}")
child_parent_dict,goal_reached,visited = dijkstra(start_x,start_y,goal_x,goal_y)
if(goal_reached == False):
    print("No Goal Found")
else:
    optimal_path = back_track(child_parent_dict,(start_x,start_y),(goal_x,goal_y))
    print(optimal_path)
    end_time = time.time()  
    print(f"Time Taken By the Algorithm : {end_time - start_time} seconds")
    plot(canvas,optimal_path,visited)
    visualise_pygame(optimal_path,visited)



    