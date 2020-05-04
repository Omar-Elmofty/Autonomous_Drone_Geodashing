#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy


import numpy as np
import matplotlib.pyplot as plt
from poly_interp import polynomial_interp_5ord

from geometry_msgs.msg import TransformStamped



class RRTStar():
    def __init__(self, start, goal):
        # Publishers
        self.pub_des_pos = rospy.Publisher('/aer1217/desired_position', 
                                      TransformStamped, queue_size=32)

        #Initialize msg
        self.desired_position_msg = TransformStamped()

        #Initialize RRT* graph
        self.G = {0:{'parent':None,'position':start,'cost':0}}

        #RRT* parameters
        self.p_goal = goal
        self.r_search = 1
        self.max_iter = 500
        self.step_size = 0.5

        #obstacles definitions
        #               x      y     rad
        obstacles = [[0.618, 8.33, 0.267],
                    [2.21, 7.32, 0.218],
                    [7.54, 7.11, 0.325],
                    [5.82, 6.53, 0.158],
                    [4.51, 3.45, 0.366],
                    [1.40, 2.52, 0.171],
                    [6.20, 1.88, 0.415]]

        self.obs_loc = []
        self.obs_rad = []
        for obs in obstacles:
            self.obs_loc.append(np.array([obs[0],obs[1]]))
            self.obs_rad.append(obs[2])

        #Workspace boundaries
        self.x_bound = [0, 9]
        self.y_bound = [0, 9]
        self.z_bound = [0, 3]

        #set desired speed for trajectory generation
        self.speed = 0.5 #m/s

        #desired publishing period
        self.dt_des = 1/50.0
        self.rate = rospy.Rate(50)

    def check_collision(self, p, p_nearest, margin=0.4):
        
        for j in range(100):
            pi = p_nearest + j/100.0*(p-p_nearest)
            for i in range(len(self.obs_loc)):
                if np.linalg.norm(pi - self.obs_loc[i]) < (self.obs_rad[i]+margin):
                    return True
        return False

    def check_path_collision(self, X, margin = 0.3):
        
        for j in range(X.shape[1]):
            pi = X[0:2,j].flatten()
            for i in range(len(self.obs_loc)):
                if np.linalg.norm(pi - self.obs_loc[i]) < (self.obs_rad[i]+margin):
                    return True
        return False


    def nearby_nodes(self, p, G, r):
        """Function that returns the nearby nodes to random sample within
        a search radius
        Args:
            p: rand
        """
        N = {}
        for key in G.keys():
            d = np.linalg.norm(p-G[key]['position'])
            if d < r:
                N[key]=G[key]

        return N


    def nearest_node(self, p,G):
        """Function that returns the nearest node in a graph
        Args:
            p: random point
            G: rrt* graph
        Returns:
            key_N: the nearest node key
            N: the nearest node to p

        """
        r = float('inf')
        N = None
        key_N = None
        for key in G.keys():
            d = np.linalg.norm(p-G[key]['position'])
            if d < r:
                N = G[key]
                key_N = key
                r = d
        return key_N, N


    def truncate(self, p, p_nearest, step):
        """Function that truncates the line segment between 2 points
        Args:
            p: a random point
            p_nearest: nearest point in the graph
            step: the truncation distance
        Returns:
            p_new: the new point after truncation 
        """
        delta = p-p_nearest
        p_new = p_nearest + step * delta/np.linalg.norm(delta)
        return p_new    

    def rrt_plan(self):
        """Function that performs RRT* planning 
        Returns: The best path from RRT*
        """

        for it in range (self.max_iter):
            #select a random point
            p_rand = np.random.uniform(0,10,2)
            #Find the nearest node
            key_N, N_node = self.nearest_node(p_rand,self.G)
            #Truncate p
            p_trunc = self.truncate(p_rand, N_node['position'], self.step_size)

            #find nearby nodes
            N = self.nearby_nodes(p_trunc,self.G,self.r_search)
            if len(N) == 0:
                N = {key_N:N_node}
            #Sort Nearby nodes by ascending order in cost
            N_sorted = sorted(N.items(), key=lambda x: x[1]['cost'], reverse=False)
            #truncate p in direction of best node
            V_new = it + 1
            parent = N_sorted[0][0]
            cost = np.linalg.norm(N_sorted[0][1]['position']-p_trunc) + self.G[parent]['cost']
            p_nearest = N_sorted[0][1]['position']
            #Check Collision
            if self.check_collision(p_trunc, p_nearest, margin=0.5):
                continue
            self.G[V_new] = {'parent':parent,'position':p_trunc,'cost':cost}
            #remove parent node from Nearest nodes
            N_sorted.pop(0)
    
            #rewire neighbors to new node if cost is lower
            for item in N_sorted:    
                if (item[1]['cost']) > (cost + np.linalg.norm(p_trunc - item[1]['position'])):
                    self.G[item[0]]['parent'] = V_new

        #Connect goal to the lowest cost node
        cost_min = float('inf')   
        best_key = None   
        for key in self.G.keys():
            stage_cost = np.linalg.norm(np.array(self.p_goal) - self.G[key]['position'])
            prev_cost = self.G[key]['cost']
            if self.check_collision(np.array(self.p_goal), self.G[key]['position'], margin=0.5):
                continue
            if (stage_cost + prev_cost) < cost_min:
                best_key = key
                cost_min = (stage_cost + prev_cost)
        #create node for goal
        self.G['goal'] = {'parent':best_key,'position':self.p_goal,'cost':cost_min}

        #populate the lowest cost path
        best_path = []
        run = True
        i = 'goal'
        while run:
            best_path.append(self.G[i])
            i = self.G[i]['parent']
            if i == 0:
                best_path.append(self.G[i])
                run = False

        return best_path


    def lazy_states_contraction(self, best_path):
        """Function that performs lazy states contraction
        Arg's:
            best_path: a list of nodes (output from RRT* forming the best path)
        Returns:
            best_path: the contracted best path with lazy states removed
        """
        #lazy states contraction
        curr_idx = 0
        mid_idx = 1
        next_idx = 2
        while next_idx < len(best_path):
            pos1 = best_path[curr_idx]['position']
            pos2 = best_path[next_idx]['position']
            if self.check_collision(pos1,pos2, margin=0.5):
                curr_idx += 1
                mid_idx = curr_idx + 1
                next_idx = curr_idx + 2
                continue
        
            best_path.pop(mid_idx)

        return best_path

    def generate_traj(self, best_path):
        """Function that creates polynomial segments between nodes
        Arg's:
            best_path: list of nodes creating the best path
        Returns:
            x,y: lists of x, y positions for forming the overall traj
        """

        #reverse the direction of best path
        best_path = best_path[::-1]

        vel = np.zeros(2)
        for i in range(len(best_path)-1):
            #set initial state
            state_init = np.zeros(9)
            state_init[0:2] = best_path[i]['position']
            state_init[3:5] = vel

            #set final state
            state_final = np.zeros(9)
            state_final[0:2] = best_path[i+1]['position']

            #set the velocity at free points
            if i < (len(best_path)-2):
                vel = best_path[i+2]['position'] - best_path[i]['position']
                vel = self.speed * vel/np.linalg.norm(vel)
            else:
                vel = np.zeros(2)

            #assign the velocity to final state
            state_final[3:5] = vel

            #perform fifth order polynomial interpolation
            _, X = polynomial_interp_5ord(state_init, state_final, self.dt_des)

            #store path
            if i==0:
                X_final = X
            else:
                X_final = np.concatenate((X_final, X), axis=1)

        #check final path for collision
        print('Collision :', self.check_path_collision(X, margin=0.3))

        return X_final

    def publish_path(self, X):
        """Function that publishes the desired path 
        """

        print('Publishing Path')

        for i in range(X.shape[1]):
            #set positions
            self.desired_position_msg.transform.translation.x = X[0,i]
            self.desired_position_msg.transform.translation.y = X[1,i]
            self.desired_position_msg.transform.translation.z = 2
            self.desired_position_msg.transform.rotation.z = X[8,i]
            self.desired_position_msg.header.stamp = rospy.Time.now()

            #publish positions
            self.pub_des_pos.publish(self.desired_position_msg)
            self.rate.sleep()



    def plot_results(self, best_path, X):
        #Plotting Results:
        fig, ax = plt.subplots()
        plt.axis([0, 10, 0, 10])
        for i in range(len(self.obs_loc)):
            circle = plt.Circle((self.obs_loc[i][0], self.obs_loc[i][1]), self.obs_rad[i], color='y')
            ax.add_artist(circle)
        for key in self.G.keys():
            pos = self.G[key]['position']
            plt.plot(pos[0],pos[1],'ro')
            parent_key = self.G[key]['parent']
            if parent_key != None:
                parent_pos = self.G[parent_key]['position']
                plt.plot([pos[0],parent_pos[0]],[pos[1],parent_pos[1]],'b')
        #plot the shortest path
        #Plotting Results:
        fig, ax = plt.subplots()
        plt.axis([0, 10, 0, 10])
        for i in range(len(self.obs_loc)):
            circle = plt.Circle((self.obs_loc[i][0], self.obs_loc[i][1]), self.obs_rad[i], color='y')
            ax.add_artist(circle)
       
        for i in range(len(best_path)-1):
            p1 = best_path[i]['position']
            p2 = best_path[i+1]['position']
            ax.plot([p1[0],p2[0]],[p1[1],p2[1]],'y')

        fig, ax = plt.subplots()
        plt.axis([0, 10, 0, 10])
        for i in range(len(self.obs_loc)):
            circle = plt.Circle((self.obs_loc[i][0], self.obs_loc[i][1]), self.obs_rad[i], color='y')
            ax.add_artist(circle)

        ax.plot(X[0,:],X[1,:])

        plt.show()

if __name__ == '__main__':

    #SET REQUIRED SEQUENCE HERE
    sequence = [2,3,1,4]

    rospy.init_node('rrt_planner')
    #Initial starting position
    start = np.array([1,1])
    yaw_init = 0

    sequence_names = ['casa loma', 
                      'cn_tower',
                      'nathan_philips',
                      'princes gates'
                      ]

    #Land mark definitions
    #               x    y   yaw
    landmarks = [[4.27, 1.23, 2.79],
        [0.874, 5.49, 1.425],
        [4.32, 8.05, 2.256],
        [7.68, 4.24, -0.527]]

    #Run RRT* between landmarks
    for i in range(len(sequence)):
        #set goal position
        idx = sequence[i] - 1
        goal = np.zeros(2)
        goal[0] = landmarks[idx][0]
        goal[1] = landmarks[idx][1]
        yaw_fin = landmarks[idx][2]

        #Plan using RRT*
        rrt = RRTStar(start, goal)
        best_path = rrt.rrt_plan()
        best_path = rrt.lazy_states_contraction(best_path)

        #Generate trajectory using polynomial interpolation
        X = rrt.generate_traj(best_path)

        #Add yaw final to path
        yaw = np.linspace(yaw_init, yaw_fin,X.shape[1])
        X[8,:] = yaw

        #store path
        if i==0:
            X_final = X
        else:
            X_final = np.concatenate((X_final, X), axis=1)

        #reset initial positions
        start = goal
        yaw_init = yaw_fin

    #rrt.plot_results(best_path, X_final)
    rrt.publish_path(X_final)
    rospy.spin()










