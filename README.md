# q_learning_project

## Hakim Lahlou, Joyce Passananti


## Q-learning algorithm:
- Executing the algorithm: We plan to implement the Q-learning algorithm provided in class 
    - We will test it by evaluating whether the matrix converges correctly with the correct sequence of actions to maximize reward.
- To determine convergence we will store the state of the matrix before updating, then compare it to the new Q matrix. When negligible difference is found a number (we’ll start at 10 then decide a final value when testing) of times in a row we will stop the algorithm.
    - To test this we will manually input values within this difference threshold to ensure the algorithm converges and recognizes when to stop.
- Determine actions: Using the q matrix, we will determine the best option at each state.
    - We test this by manually checking the state and see if the correct action is predicted.


## Robot perception:
 - To determine the identities and location of the colored dumbbells, we will utilize the /scan package to find the object that falls into the desired color range. From there, we will orient the robot towards the desired object  and have it drive into range of the object. If the dumbbells are not directly in front of the robot, we will have it rotate and scan until it finds its desired object.
    - To test this, we will put different dumbbells around the robot to test if it will find and recognize the right color. We will then move the dumbbell to ensure that the robot can track and follow it.
 - To determine the blocks and their location, we will use the “Cob_read_text” package, which can use the camera feed of the robot to read text. With this tool, we can move towards the boxes (this can be assisted by using scan to look for the box’s color range) and determine if it is the desired number.
    - To test this, we will place blocks in front of the robot to ensure that it can read the text properly. We will then test the robot’s ability to find the desired box by lining up all three options and having the robot find the desired box number.


## Robot manipulation & movement:
- To pick up the dumbbell, we will use the MoveIt package to control the robot’s arm, lowering it to prepare the robot and hoisting the arm into the air once the dumbbell is properly gripped. To ensure that the arm is close enough to the dumbbell, the scan package can be used to determine the exact distance between the two. The same will be done for putting down the dumbbell, as we must ensure there is enough space for the dumbbell and that it is not dropped haphazardly.
    - To test this, we will place a dumbbell in front of the robot for it to lift and have it move a set amount of distance before putting it down again.
- This will incorporate the robot perception portion of the project, and will utilize the knowledge of the desired dumbbell and box locations to direct itself and perform the pick up/ put down action.
    - To test this, we will have the robot exhaust each entry in the action matrix, moving each dumbbell to each location after resetting.


## Timeline:

2/17: Finish Implementation Plan

2/20: Finish Robot Perception

2/23: Finish Robot Manipulation & Movement

2/27: Finish Q- Learning Algorithm

2/28: Finish Testing and Be Ready to Submit



<http://wiki.ros.org/cob_read_text>