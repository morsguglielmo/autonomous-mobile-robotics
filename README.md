# Autonomous and Mobile Robotics project 
### Guglielmo Morselli - Unibo - 2024

# Task 1
Provided an unknown environment, the robot is supposed to explore it to create its own map.

To do so, the package https://github.com/robo-friends/m-explore-ros2 was used, together with Nav2.

<p float="left">
  <img src="https://github.com/user-attachments/assets/d9111eef-bb09-469e-805d-7cd2f754aeb7" width=30% height=30%>
  <img src="https://github.com/user-attachments/assets/fcc92308-e038-4653-9fb3-88d2ade386cf" width=30% height=30%> 
  <img src="https://github.com/user-attachments/assets/810c048f-9456-4f35-a441-80b7872820cf" width=30% height=30%>
</p>

# Task 2
## Localization
Given a map, the robot has to localize itself. 

In order to achieve so, it will safely roam in the room, until it's confident enough about its location.

<img src="https://github.com/user-attachments/assets/4a05e2ce-3f11-4f9f-9fe3-54bf00c7c4a4" width=70% height=70%>


## Navigation
Once the localization is on point, the robot moves to user-defined locations.

# Task 3 
The final task is to sanitize user defined rooms. Given an UV lamp with known power, the robot must move in the room until the UV energy level is above a safe threshold.
The whole room must be sanitized, hence an area coverage algorithm has been developed. In this work, the developed policy in inspired by a Model Predictive Control approach.

First, the map is divided into smaller cells. Then, a power map is iteratively updated, identifiyng the cells reached by the UV rays.

<img src="https://github.com/user-attachments/assets/294624b5-2881-43e4-bcbc-9226a401942e" width=30% height=30%>

For each cell an energy map is computed as well, integrating the power over the time.

An area covering algorithm is then defined, ensuring an efficient operation.

Firstly, the the chosen room is divided into a coarser grid (nodes), free cells are detected, and the room subset is chosen:

<p float="left">
  <img src="https://github.com/user-attachments/assets/2e68762d-c429-4da0-9a7d-c3e1278979be" width=20% height=20%>
  <img src="https://github.com/user-attachments/assets/82cc38c1-0108-4348-aa12-6ab3e2c8ab01" width=20% height=20%> 
  <img src="https://github.com/user-attachments/assets/b2eed5f8-1c0c-450c-9c78-3ad3f9864051" width=20% height=20%>
  <img src="https://github.com/user-attachments/assets/dedb1127-7c8f-42bf-9464-25d877841394" width=20% height=20%>
</p>

Secondly, while the room is not fully sanitized, the robot:

1. Computes the nodes yet to be sanitized;

2. Computes the shortest path through those nodes solving a custom traveling salesman problem;

<img src="https://github.com/user-attachments/assets/a4fbee8b-fedf-4795-a696-8dd266e03a58" width=30% height=30%>

3. Actually moves through the first K nodes (where K < total length of the path). TSP path in blue, actual path in green:

<img src="https://github.com/user-attachments/assets/c6d58d28-1518-4957-8e67-3a1aacdf8ecd" width=30% height=30%>






