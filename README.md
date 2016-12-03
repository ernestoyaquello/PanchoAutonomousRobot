# Arduino Robot "Pancho"
This is a small project that contains the code of a robot called Pancho that I made in 2013 using an Arduino.

## About the robot
![First version of the robot](https://github.com/ernestoyaquello/robot-pancho/blob/master/readme-pictures/pancho-v1.jpg?raw=true)

This is a robot that can move around on its own avoiding the obstacles in its way. It can also be controlled remotely using an infrared remote, and I actually configured it to accept commands from the remote of my home TV.

### Components
I do not remember the model names of the components that I used, but I know for sure that the robot has all of the following:
* Arduino UNO
* A chassis RP5 with two motors that is used to move the robot forward or backward and to rotate it clockwise or anticlockwise
* A motor contoller that is necessary to use the motors
* A battery that powers the whole system
* An ultrasonic distance sensor to detect the proximity of obstacles
* A servo that can make the ultrasonic sensor rotate in order to read distances not only in front of the robot but also in different angles
* An infrared sensor to read signals emited by a remote controller
* A sensor shield that makes the connection of sensors easier
* A red led and a buzzer to notify the detection of obstacles, the beginning of actions, etctera
* Blue leds used for decoration
* Protoboard and wires
* Some extra materials, such as Meccano pieces, to fit the whole thing together

### How it works
The robot has three operating modes: ```standby```, ```autonomous``` and ```remotely controlled```.

#### Standby Mode
The robot is reading the infrared signals and waiting for the command that will change its operating mode to one of the others.

#### Autonomous Mode
When this mode is active the robot "walks" on its own. In case it finds an obstacle, the robot stops, looks around to find the safest direction and moves towards it. If the obstacle is too close, the robot moves backward and, if necessary, even rotate 180 degrees in order to run away.

#### Remotely Controlled  Mode
When this mode is active, the robot just follows the commands that are received by its infrared sensor, so it actually becomes a slow RC car.

## Shut up and show me the robot!
In the following videos you can see the robot walking around. In each video the robot has a different code as well as different components, so the way it behaves is not the same across the three of them.

#### First version of the robot
It didn't include an infrared sensor yet, so it was just an autonomous robot.

[![First version of the robot](https://github.com/ernestoyaquello/robot-pancho/blob/master/readme-pictures/video1-thumbnail.jpg?raw=true)](http://www.youtube.com/watch?v=KB23F5sOBRM "First version of the robot")

#### Second version of the robot
In order to adapt it to the labyrinth I created, I modified its code so the robot would move in perpendicular directions all the time.

[![Second version of the robot](https://github.com/ernestoyaquello/robot-pancho/blob/master/readme-pictures/video2-thumbnail.jpg?raw=true)](http://www.youtube.com/watch?v=oFyTtHxYDGw "Second version of the robot")

#### Third version of the robot
It is worth noting that the code the robot has in this video is almost the same than the one you can find in this repo. However, the one of this repository is improved so the robot rotates smoothly when being controlled remotely (in the video you can see that the rotation is very harsh because when I recorded it the code was not improved yet).

[![Third version of the robot](https://github.com/ernestoyaquello/robot-pancho/blob/master/readme-pictures/video3-thumbnail.jpg?raw=true)](http://www.youtube.com/watch?v=lSDLfAEq20w "Third version of the robot")

## Do It Yourself
As you can see in this repository, a robot like this one is very easy to make as long as you know a little bit about programming and Arduino. Therefore, even though you can use as much code of this repo as you wish, there is no need to copy it all. Besides, this code might be outdated or even not valid for your components, so the best you can do is to learn and experiment on your own with the materials and abilities you have. Remember that the funny thing here is not to have a robot... is to create it!

## License
This license is here just because it looks cool, but you can do whatever you want with this code and I promise I won't mind :)

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```