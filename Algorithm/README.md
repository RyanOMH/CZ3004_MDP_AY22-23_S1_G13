# Algorithm Checklist
Simulator should include:

## Robot Movement Area Simulator
Simulator should display:
1. 2m * 2m movement area (Grid Map)
2. Start Zone
3. Locations of obstacles
4. Position of images
5. Position of robot as it moves forward/backward and turns

## Hamiltonian Path Computation Simulator
Simulator should demonstrate: 
1. The implementation of an algorithm that guides the robot to traverse the 2m * 2m movement area, starting from the start zone and visiting each image position once
2. Recognition of each image within time limit

## Shortest-time Hamiltonian Path Computation
Simulator should demonstrate:
1. The robot following a shortest-time Hamiltonian path to recognize the images.

# Procedure for image recognition
1. Send obstacles information from controller app on Android tablet to robot.
2. The robot automatically locate the obstacles and finds the shortest Hamiltonian path to traverse to all obstacles.
3. When the robot reaches each obstacle, it activates the pi-camera to scan the image and detect the symbol.
4. After each image recognition is completed, the robot sends the image id back to the controller app on Android tablet.

# What if the robot does not see the obstacle but it expects to see one:

While the robot is executing the path
Each time the robot reaches the point to scan the image
if Scan image does not return any image id

# Flow of ANDROID/RPI/ALGO/IMAGEREC/STM

Loop For each obstacle in the grid:
    ANDROID send obstacle locations to RPI
    RPI send obstacle locations to ALGO
   ALGO receives obstacle locations
   ALGO calculate path and show on simulator full run (without issues encountered)
   ALGO returns path commands to first obstacle to RPI
   RPI sends path to STM
   STM execute path commands and ROBOT moves to 1st obstacle
   STM returns "COMPLT" (completion of command) to RPI
   RPI request IMAGEREC to scan for image

   if IMAGEREC is able to find image (assume IMAGEREC is always able to identify):
       return image id to RPI to ANDROID
       return IMAGE SUCCESS AKNOWLEDGEMENT to ALGO
   else if IMAGEREC finds a bullseye:
       return IMAGE BULLSEYE to ALGO
   else if IMAGEREC is unable to find image:
       return IMAGE UNABLE TO FIND AKNOWLEDGEMENT to ALGO

   if ALGO receives IMAGE SUCCESS AKNOWLEDGEMENT from IMAGEREC
       ALGO returns next set of path commands to next obstacle to RPI
   else if ALGO receives IMAGE BULLSEYE from IMAGEREC
       execute navigate function
   else if ALGO receives IMAGE UNABLE TO FIND AKNOWLEDGEMENT from IMAGEREC
       execute roam function

# Algo for fastest path
 
Robot moves forward 
If ultrasound detect 20cm (subject to change) away from obstacle, stop robot
Scan image
Image rec return direction to algo
robot turn according to direction
robot go to last obstacle
Scan image
Image rec return direction to algo
robot turn according to direction

# server = Server(socket.gethostbyname(socket.gethostname()), 3004)