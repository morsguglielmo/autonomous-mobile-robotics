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
<img src="https://github.com/user-attachments/assets/4a05e2ce-3f11-4f9f-9fe3-54bf00c7c4a4" width=80% height=80%>


## Navigation
Once the localization is on point, the robot moves to user-defined locations.

# Task 3 
The final task is to sanitize user defined rooms. Given an UV lamp with known power, the robot must move in the room until the UV energy level is above a safe threshold.
The whole room must be sanitized, hence an area coverage algorithm has been developed.

power map
![image](https://github.com/user-attachments/assets/294624b5-2881-43e4-bcbc-9226a401942e)

energy map
![image](https://github.com/user-attachments/assets/cab7a048-221f-4874-9551-e8804ac60402)
![image](https://github.com/user-attachments/assets/65505354-18bd-471b-8122-14d9fd5724db)


<p float="left">
  <img src="https://github.com/user-attachments/assets/d9111eef-bb09-469e-805d-7cd2f754aeb7" width=30% height=30%>
  <img src="https://github.com/user-attachments/assets/fcc92308-e038-4653-9fb3-88d2ade386cf" width=30% height=30%> 
  <img src="https://github.com/user-attachments/assets/810c048f-9456-4f35-a441-80b7872820cf" width=30% height=30%>
</p>
