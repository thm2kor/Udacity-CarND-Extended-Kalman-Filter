# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The objective of this project is to estimate the state of a moving object with noisy measurements from lidar and radar sensors. The state of the moving object is represented by two dimensional position p<sub>x</sub>, p<sub>y</sub> and two dimensional velocity v<sub>x</sub>, v<sub>y</sub>. The **state vector** <img src="https://latex.codecogs.com/gif.latex?\mathit{x}"/> is therefore:
<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_x\\p_y\\v_x\\v_y\end{pmatrix}"/>

**Process Noise:** The uncertainty in the *prediction step*, represented as <img src="https://latex.codecogs.com/gif.latex?\mathit{\nu}"/>, is a gaussian distribution with mean 0 and covariance Q  <img src="https://latex.codecogs.com/gif.latex?\[\nu\sim N(0,Q)\]"/>

**Measurement Noise:** The uncertainty in *sensor measurement*, represented as <img src="https://latex.codecogs.com/gif.latex?\mathit{\omega}"/>, is a gaussian distribution with mean zero and covariance R  <img src="https://latex.codecogs.com/gif.latex?\[\omega \sim N(0,R)\]"/>

We use linear motion model to estimate the new position and velocity:
<img src="https://latex.codecogs.com/gif.latex?\\\acute{p_x} = {p_x} + {v_x}\Delta T + \nu_p_x\"/>

<img src="https://latex.codecogs.com/gif.latex?\\\acute{p_y} = {p_y} + {v_y}\Delta T + \nu_p_y\"/>

<img src="https://latex.codecogs.com/gif.latex?\\\acute{v_x} = {v_x} + {v_y}\Delta T + \nu_v_x\"/>

<img src="https://latex.codecogs.com/gif.latex?\\\acute{p_x} = {v_y} + {v_y}\Delta T + \nu_v_y\"/>

**Process covariance matrix**

**State transition matrix**
<img src="https://latex.codecogs.com/gif.latex?\\\begin{pmatrix}\acute{p_x} \\ \acute{p_y} \\ \acute{v_x} \\ \acute{v_y} \end{pmatrix} = \begin{pmatrix} 1 & 0 & \Delta T & 0 \\ 0 & 1 & 0 & \Delta T\\ 0 & 0 & 1 & 0\\ 0 & 0 & 0 & 1 \end{pmatrix} \begin{pmatrix}p_x\\p_y\\v_x\\v_y\end{pmatrix} + \begin{pmatrix}\nu_p_x\\\nu_p_y\\\nu_v_x\\\nu_v_y\end{pmatrix}\"/>



For every new measurement from a radar or lidar sensor, the following two steps are carried out:
- Estimation function is triggered.
  - At the first iteration of measurement, state transition matrix and covariance matrix are initialized
