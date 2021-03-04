#!/usr/bin/env python3

import rospy
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock
import numpy as np

class QLearning(object):
    def __init__(self):
        # set alpha/gamma/epsilon for easy access when optimizing algorithm during testing
        self.alpha = 1
        self.gamma = 0.5
        self.epsilon = 1
        
        # Initialize q_learning node
        rospy.init_node('q_learning')

        # Set publishers
        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.robot_action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=10)
        
        # ROS Subscribe to rewards
        rospy.Subscriber("q_learning/reward", QLearningReward, self.q_algo)

        # Waits for publisher to be attached
        rospy.sleep(3)

        # create a reference for dumbell colors
        self.dumbells = ["red", "green", "blue"]
        
        # create a table that stores the different actions (as a tuple: (db_moved, block_moved_to)) the robot can take
        self.action_table = []
        for db in range(3):
            for block in range(1,4):
                self.action_table.append(self.dumbells[db], block)
        
        # create an array that represents every possible state
        self.state_matrix = []
        for b in range(4):
            for g in range(4):
                for r in range(4):
                    self.state_matrix.append((r, g, b))
        
        # Initialize and publish Q Matrix
        self.q_matrix = QMatrix()
        self.q_matrix.q_matrix = np.full((64, 9), 0)
        self.q_matrix_pub.publish(self.q_matrix)
        
        # intialize iterations
        self.iterations = 0
        self.iterations_converging = 0

        # Initialize action Matrix
        self.init_action_matrix()

        # Start state as 0, next state hasn't been determined yet
        self.curr_state = 0
        self.next_state = -1
    
        # initialialize action chosen (-1 before any have been chosen)
        self.action_choice = -1
        
        # keeps track of whether the q_matrix has converged
        self.converged = False
        
        self.final_action_seq = []

    # initialize action matrix
    def init_action_matrix(self):
        
        # creating the action matrix
        self.action_matrix = []
        self.action_matrix = np.full((64, 64), -1)
        
        for start, col in enumerate(self.action_matrix):
            for next in range(len(col)):
                start_state = self.state_matrix[start]
                next_state = self.state_matrix[next]
                
                # check that transition to next state is valid:
                is_valid_action = True
                # 1) need to check (at least) 1 and only 1 move has been made and that no dumbell has been moved back to origin/between blocks
                # an array of moves in the format (db_color_moved, block_number)
                moves = []
                for i in range(3):
                    if start_state[i] != next_state[i]:
                        if start_state[i] == 0:
                            moves.append((self.dumbells[i], next_state[i]))
                        else:
                            is_valid_action = False
                            break

                    if len(moves) != 1:
                        is_valid_action = False
                        break                
                # 2) check no more than one dumbell at each block
                for db in range(1, 4):
                    num = 0
                    for block in range(0, 3):
                        if next_state[block] == db:
                            num = num + 1
                        if num > 1:
                            is_valid_action = False
                            break
                
                if is_valid_action:
                    # get the action number that matches the move
                    action = 0
                    for i, a in enumerate(self.action_table):
                        if a == moves[0]:
                            action = i
                    # add the action that changes the state from the start_state to next_state
                    self.action_matrix[start][next] = action

    def random_action(self):
        # choose a random action given the robot's current state            
        possible_actions = np.array(self.action_matrix[self.curr_state])        
        self.action_choice = np.random.choice(possible_actions)
        while self.action_choice == -1:
            self.action_choice = np.random.choice(possible_actions)
        action = self.action_table[self.action_choice]

        # publish chosen action
        robot_action = RobotMoveDBToBlock(robot_db = self.dumbells[action[0]], block_id = action[1])
        self.robot_action_pub.publish(robot_action)


    # updates q_matrix based on rewards
    def q_algo(self, data):
    
        # get current and new values of the row
        curr_value = self.q_matrix.q_matrix[self.curr_state].q_matrix_row[self.action_choice]
        self.q_matrix.q_matrix[self.curr_state].q_matrix_row[self.action_choice] = curr_value + (self.alpha * (data.reward + self.gamma * max(self.q_matrix.q_matrix[self.next_state].q_matrix_row)  - curr_value))
        new_value = self.q_matrix.q_matrix[self.curr_state].q_matrix_row[self.action_choice]

        # if q_matrix vals don't change, increase converging count
        if abs(new_value - curr_value) < self.epsilon:
            self.iterations_converging += 1
        # if vals do change, reset convergence count
        else: 
            self.iterations_converging = 0

        # publish q_matrix and update state
        self.q_matrix_pub.publish(self.q_matrix)
        self.curr_state = self.next_state

        # resets state when all 3 dumbells have been moved
        if self.iterations % 3 == 0:
            self.current_state = 0
            rospy.sleep(1)

        # checks if matrix converged
        if self.converged is False:
            # choose next action
            self.random_action()
            self.iterations += 1
            self.check_convergence()
        
    def check_convergence(self):
        # check if iterated minmum amount of times
        if self.iterations > 200:
            if self.iterations_converging >= 300:
                self.converged = True
                self.state = 0
                # find best sequential actions for robot at each stage
                for a in range(1,3):
                    row = self.q_matrix.q_matrix[self.state].q_matrix_row
                    best_action = row.index(max(row))
                    print(a, " best action: ", best_action)

                    # add action to final list of actions
                    self.final_action_seq.append(self.action_table[best_action])
                    
                    # # publish chosen action
                    # robot_action = RobotMoveDBToBlock(robot_db = self.dumbells[best_action[0]], block_id = best_action[1])
                    # self.robot_action_pub.publish(robot_action)

                    # get next state
                    self.state = self.action_matrix[self.state].index(best_action)
                                
                print("matrix converged")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = QLearning()
    node.run()