## **Path Planning Project**
#### Abdelkoddous Khamsi
[gif1]: ./writeup_resources/behaviorPlanningDemo.gif "Demo Sample"

The goal of this project is to design and implement a path planner in C++ that is able to create smooth and safe paths for a car to follow in a highway driving scenario. The car shall be able to stay in its lane, avoid collisions with other cars and change lanes whenever stuck behind a slower vehicle.

The work I conducted for this project can be split into 2 major components:
* Car Waypoints Generator
* Car Behavior planning Finite State Machine

### **1. Car Waypoints Generator:**

The goal of this part is to generate waypoints for the car to follow inside its lane. These waypoints should build up to a smooth path.

Please note that for this first part the Project Q&A video was very helpful and the implentation was done along with that session. I state this clearly in my source code in the line **349** of the `main.cpp` file.

* At each step I start with a low resolution path of 5 waypoints: current position of the car, previous position of the car and 3 points ahead of the car.
* Then I used the Spline class to interpolate these widely spaced 5 points, by fitting more waypoints in order to control the speed of the car. 


### **2. Car Behavior planning Finite State Machine:**

At this point the car can travel smoothly along the highway as long as no car is in fonrt of it.
In this part I designed and implemented the strategy for the car to avoid collisions and change lanes.
Here is a brief description of the Behavior Planning strategy I implemented:
* I created a scanning area within a range of 80 meters ahead and behind of the car. The car keeps track of all the cars inside this area and organize them into their corresponding lanes.
* When the car detects a car ahead of it in its lane, it changes its speed to match this car's speed.
* When the car reachs the minimum safety distance it checks if the other lanes are empty within the scanning area, if it is the case it initiates a Lane Change manoeuvre.
* Otherwise, it starts monitoring the potential target lanes for some specific conditions including distance to cars driving in target lanes and their speeds, once this set of conditions (can always be improved) is met the car initiates a Lane change and drive at maximum speed possible in that lane as long as it's below 50mph.


### **3. Demo time:**
![alt text][gif1]  

In the full video I was able to get to up to 18 Miles driving without Incident.

The full (accelerated) video can be found [here](./writeup_resources/MyBehaviourPlanningFullVideo.mp4).