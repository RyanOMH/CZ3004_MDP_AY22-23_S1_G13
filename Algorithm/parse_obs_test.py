from Simulator.simulator_mgr import *

very_old_obstacles = [[15, 185, -90, 0], [65, 125, 90, 1], [105, 75, 0, 2], [155, 165, 180, 3], [195, 95, 180, 4], [135, 25, 0, 5]]

old_obstacles = [{"x":1,"y":18,"direction":-90,"obs_id":0}, 
                 {"x":6,"y":12,"direction":90,"obs_id":1},
                 {"x":10,"y":7,"direction":0,"obs_id":2}, 
                 {"x":15,"y":16,"direction":180,"obs_id":3}, 
                 {"x":19,"y":9,"direction":180,"obs_id":4},
                 {"x":13,"y":2,"direction":0,"obs_id":5}]

new_obstacles = {"obstacle1": [15, 185, -90, 0], 
                 "obstacle2": [65, 125, 90, 1], 
                 "obstacle3": [105, 75, 0, 2], 
                 "obstacle4": [155, 165, 180, 3], 
                 "obstacle5": [185, 95, 180, 4], 
                 "obstacle6": [135, 25, 0, 5]}

print("============================================================================")
print(f"Original Obstacle data: {parse_obstacle_data_old(very_old_obstacles)}")
print("============================================================================")
print(f"Current Android data: {parse_obstacle_data_cur(old_obstacles)}")
print("============================================================================")
print(f"New JSON data: {parse_obstacle_data_new(json.dumps(new_obstacles))}")
print("============================================================================")


"""
# Check if data received is from ANDROID or STM/IMAGE
    if client.is_json(obstacle_data):
        # If receive obstacle data from ANDROID
        client.send_json_aknowledgement()
    else:
"""