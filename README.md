# Roboweek Challenge

This repository features a rough kinematic algorithm developed by me (UFSM, NTNU) and [Guilherme Christmann](https://github.com/guichristmann) (UFSM, NTNU) during a 24h robotics competition held in Brazil, in 2018.

> Challenge: development of a kinematic algorithm for a laser-guided TurtleBot within a maze environment.

# Award

Second place medal awarded to overall individual team winner of a 24h robotics competition (RoboWeek) at the Federal University of Rio Grande (FURG), Brazil.
Event Website: http://roboweek.c3.furg.br/

# Instructions
To reproduce the simulation, check [this Google drive folder](https://drive.google.com/drive/folders/1rIWDC9wHPcT_eNRiRx-XcyBCDKqV4Xsh) for instructions on how to operate the Gazebo simulator using ROS.

![](GazeboSimulator.png)

# Solution 

The ansatz: if the length between each laser sensor is different, send the robot towards the direction in which the laser length is longer (dodging the obstacle ahead)—else—flip a coin, i.e, any right/left turn is a feasible solution. Although this logic seems very straightforward, there is a caveat: the gradient might point towards the start line since there are no transit signs neither object detection implementation to read them. Circumventions must be made.

Below is our TurtleBot completing the entire circuit and leading position in the second to last race. Our implementation was able to complete the entire labyrinth. However, in the finals, our bot slammed into another, consequently overturning, and so we got second place overall according to the distance criteria from the finish line.

![](competition.gif)

# Author

Created and maintained by [@camponogaraviera][1].

[1]: https://github.com/camponogaraviera

# License

This work is licensed under a [Creative Commons Zero Attribution v1.0 Universal](LICENSE) license.
