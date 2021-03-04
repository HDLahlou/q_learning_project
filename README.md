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

## Writeup

### Objectives Description
The goal of this project is to use a Q-learning algorithm to determine *optimal* actions (actions revolve around moving a certain dumbbell to a certain numbered block) for the robot to take to garner the most points possible given certain conditions. The Q-learning utilizes phantom movements until the Q-matrix converges and will use this matrix to provide action sequences for the robot to execute to reach its goal state. 

### High-level Description
TODO: INSERT Q LEARNING INFO HERE (At a high-level, describe how you used reinforcement learning to solve the task of determining which dumbbells belong in front of each numbered block.
)
After the Q-matrix converges, we extract an action sequence that will maximize the expected *reward* for the robot’s future actions and send it to the robot for it to execute. The robot_action node will receive this action sequence and will break down what it has to do to achieve this goal state. The robot_action node carries out three main tasks which include picking up a specified dumbbell, moving to a specific block location, and putting down a dumbbell. Once the robot is aware of what color dumbbell it must pick up and what numbered block it must move to, the robot will spring into action and ascertain where these two objects are located through scanning and color detection. If the desired object is not immediately in view, the robot will rotate until it finds its goal.

### Q-learning algorithm description
TODO: INSERT Q LEARNING INFO HERE

 Describe how you accomplished each of the following components of the Q-learning algorithm in 1-3 sentences, and also describe what functions / sections of the code executed each of these components(1-3 sentences per function / portion of code):
Selecting and executing actions for the robot (or phantom robot) to take
Updating the Q-matrix
Determining when to stop iterating through the Q-learning algorithm
Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot



### Robot Perception

#### Colored Dumbbell Identification
This was accomplished through use of LaserScan and the camera on top of the robot and interpreting its raw images to search for the desired dumbbell through color recognition. After discerning the desired color, the robot assumes that this is the desired dumbbell and centers itself  on that dumbbell’s location. We don’t determine the dumbbell’s exact location and instead just approach it until we have reached a certain distance from the target

##### Related Functions
- *dumbell_callback* handles the state information related to determining what functions to call next and is what triggers *color_detect_and_move* which handles the logic for identification and locating Colored Dumbbells
- *color_detect_and_move* takes care of the above implementation for locating the specified dumbbell. Based on the desired color, it will select a range for identifying that color and create a mask that will isolate that range within the camera feed input. We rotate the robot until the mask detects the desired color and use its location within the image to help center the robot’s orientation on the identified object. From there, we move towards that location until the scanner has detected that the dumbbell is less than 0.2 units away.

#### Numbered Block Identification
This was accomplished through the use of *keras_ocr*, LaserScan, and the camera on top of the robot for image feeds, determining distance from an object, and recognizing characters within the image. With these tools, we can identify the desired numbered block and find its location by approaching it. To do this, we rotate the robot until it sees the color black (since the numbers on the block are black) and rotate until we see the desired number. From there, we center on that target and approach it until we have reached a certain distance from it.

##### Related Functions
- *char_image_callback* handles the state information for dictating what the robot must do next for properly discerning and approaching the numbered block. Consists of if statements and function calls to below functions.
- *detect_black* has similar functionality to *color_detect_and_move* where it creates a mask that recognizes black colors and rotates until it discerns that color within the image feed. This differs from *color_detect_and_move* as it does not immediately start approaching the discerned target and has a more relaxed threshold for successfully being “centered”.
- *face_cubes* utilizes *pipeline.recognize* on the image feed to create a prediction group of possible “recognized” characters.  It iterates through this list to see if any of the values match the desired numbered block, setting the matching result as the value to be approached. If the desired block is not found, it rotates slightly and searches for it again.
- *approach_cube* takes in the data from *face_cubes* and determines the centerpoint of the prediction’s window / box location and orients the robot to face that center point. It will continually call *face_cubes* to update the prediction’s window location to ensure this reorientation is accurate. From there, once it’s centered, the robot approaches the location until it is less than 0.75 units away.

### Robot Manipulation and Movement

#### Moving to Dumbbell
This movement was fairly simple to implement. Once oriented towards the dumbbell, we move towards the target until we are less than 0.2 units away, which is the ideal range for our arm and its positioning. Since we are constantly orienting the robot to the center of the dumbbell, it is perfectly positioned to grab around its handle.  By this point, the arm is already lowered and positioned and would not interfere with the movement itself.
##### Related Functions
- *color_detect_and_move*, as described above, constantly centers the robot while moving towards the target, which handles moving to the dumbbell and sets the robot up perfectly for picking up the dumbbell

#### Picking up Dumbbell
This action revolves around arm manipulation. The arm starts out in a lowered position, posed and ready to pick up the dumbbell, with joint angles of [0.0, .2, .8, -1.2] and an open grip. From there, it raises the arm and closes the grip, with new joint angles of [0.0, -.2, .1, -0.6], positioned in such a way that the grip cradles the dumbbell and uses gravity to help keep the dumbbell within the robot’s grip.
##### Related Functions
- *manip_arm_pos* handles both lowering the dumbbell and picking up the dumbbell by raising the arm. Based on state values, it determines if it needs to lower or raise the arm, changing the joint values of the arm and gripper.

#### Moving to Desired Destination with Dumbbell
This was a simple task to do once the desired numbered block was found. With this target's location in mind, the robot can drive towards it until it reaches its desired distance from it. This action is performed with the armed raise and would be called after picking up the dumbbell, so it is assumed that moving the robot to the desired destination would also be moving the dumbbell to its desired destination.
##### Related Functions
- *approach_cube* will move towards the desired destination by centering itself on the centerpoint of the target, approaching slowly until it is less than 0.75 units away

#### Putting Down the Dumbbell
This was handled by simply lowering the arm and backing away from the dumbbell, to avoid knocking it over or having any interference during future movement and rotation actions.
##### Related Functions
- *put_down_dumbell* calls *manip_arm_pos* which will put the arm in the lower position by setting the joints as described above. After that, it sets the velocity to -0.2 and sleeps for one second to reverse away from the dumbbell and prepare for the next action.


### Challenges
One of the major challenges that we ran into for *Robot Perception* was the inherent latency revolving around the recognition functionality of *keras_ocr* for Digit Recognition, as by the time the image feed was properly interpreted, the robot would be facing a completely new direction. To counteract this issue, we took a two-prong approach by addressing turning speed and optimizing image interpretation speed. We optimized interpretation speed by reducing the scale of the image, with a quadratic decrease in area, resulting in a decrease in accuracy but less overall processing. With regards to addressing turning speed, we significantly reduced the speed at which the robot was rotating at while searching for the correct block. We also created a complimentary function called detect_black() to ensure that the robot was at least facing the right direction before performing its digit recognition and rotation. Another robot perception was a small issue with discerning the number 1, as it could be interpreted as the letter l. This was quickly remedied by creating alternative values that would satisfy the conditionals for matching.  TODO: INSERT Q LEARNING CHALLENGES HERE

### Future Work
If we had more time, we would determine a method to properly wait for *keras_ocr* recognition results before rotating and scanning more images. This method would revolve around inspecting the prediction groups results and comparing it to previous results for changes in prediction values and box positioning. We would also create a method to help the robot perception determine which face of the block it is recognizing, to help ensure it also orients itself towards the front of the block. TODO: INSERT Q LEARNING FUTURE WORK HERE

### Takeaways
- Robot perception and image analysis is very time and resource dependent, meaning that any implementation involving this must be very accommodating and take these limiting factors into account.
- TODO

### Gifs
