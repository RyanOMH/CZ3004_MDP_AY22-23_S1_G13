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