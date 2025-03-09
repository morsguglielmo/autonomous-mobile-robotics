Autonomous and Mobile Robotics project - Guglielmo Morselli - 2024

# Task 1
Provided an unknown environment, the robot is supposed to explore it to create its own map.
To do so, the package https://github.com/robo-friends/m-explore-ros2 was used, together with Nav2.

# Task 2
## Localization
Given a map, the robot has to localize itself. In order to do so, it will safely roam in the room, until it's confident enough about its location.

## Navigation
Once the localization is on point, the robot moves to user-defined locations.

# Task 3 
The final task is to sanitize user defined rooms. Given an UV lamp with known power, the robot must move in the room until the UV energy level is above a safe threshold.
The whole room must be sanitized, hence an area coverage algorithm has been developed.

