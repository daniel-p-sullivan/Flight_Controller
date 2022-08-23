# Flight_Controller
A quadcopter Flight Control System.
I started development of this project after I created my inverted pendulum.  The motivation of this project was to learn more about advanced control theory topics.  More advanced control algorithms will be used within this firmware;  Extended Kalman Filters, Model Predictive Control, etc.  Eventually I will use this quadcopter as a robotics platform to develop higher level robotics algorithms (path planning, multi-agent coordination, etc).



Development of this project started in C, and I am currently in the middle of transitioning it into C++.

Ultimately I would like the flight controller code to be not dependent on the underlying board, and I plan on implementing an abstraction layer so that multiple boards/micros can be supported.


