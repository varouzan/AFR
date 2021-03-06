# AFR

**Autonomous Fighting Robot**

[Video](https://drive.google.com/open?id=1qbUIgu0ZMda-dUbzqbQbCqik7pSt9cQw)

<p align="center">
  <img src="https://github.com/varouzan/AFR/blob/master/robo.PNG">
</p>


The robots task was to track down enemy robot or the enemy base, both situated with an IR emitting source while avoiding obstacles at the same time. As an IR source would be detected, robot would allign itself and shoot a ping pong ball.

* Ultasonic sensors were used to get distance of abstacles around the perimeter of the robot.
* Photodiodes were used to detect IR source.
* DC motors and an H-bridge circuit were responsible to drive the wheels and ping pong cannon through a PWM signals provided by mictocontroller.

<p align="center">
  <img src="https://github.com/varouzan/AFR/blob/master/schematic.PNG">
</p>

Firmware was written for STM Nucleo 64 401RE board.

<p align="center">
  <img src="https://github.com/varouzan/AFR/blob/master/pinout.PNG">
</p>


