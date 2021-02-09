# Extended Kalman Filter Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[image1]: ./images/radar_measurement_parameters.png "radar measurement parameter"
[image2]: ./images/linear_approximation.png "linear approximation"
The objective of this project is to estimate the state of a moving object with noisy measurements from lidar and radar sensors. The state of the moving object is represented by two dimensional position p<sub>x</sub>, p<sub>y</sub> and two dimensional velocity v<sub>x</sub>, v<sub>y</sub>. The **state vector** <img src="https://latex.codecogs.com/gif.latex?\mathit{x}"/> is therefore:
<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_x\\p_y\\v_x\\v_y\end{pmatrix}"/>

Udacity provided simulated radar and lidar measurements from a bicycle that moves around the target vehicle. In addition, Udacity also provided a software framework which :
1. Reads in the data (sensor values as well as ground truth) from the simulator
2. Encapsulates the data into instances of class *MeasurementPackage*
3. Provide a SW design with entry points to:
  - Initialize Kalman filter matrices and variables
  - Predict where the cyclist is going to be after a time step
  - Update where the cyclist is based on sensor measurements
  - Measure how the Kalman filter performs by calculating the root mean squared error
4. Returns the results back to the simulator

The actual contribution of the project is the Step 3, where the standard and extended Kalman Filter functions for predicting and updating the data from Lidar and Radar respectively are implemented.

---

## Kalman Filter
To track the cyclist around the vehicle, it is required to define a :
1. *State Transition function* that models how the state of the cyclist changes between two sensor timestamps.
2. *Measurement function* that models how the measurement is calculated and how close it is to the predicted state.

### State Transition function
Based on the **kinematics equations** of an object in motion, the new 2D position and 2D velocity can be estimated with the following equations:<br>
<img src="https://latex.codecogs.com/gif.latex?{p_x}'={p_x}&plus;{v_x}\Delta&space;t&plus;\frac{a_{x}\Delta&space;t&space;^2}{2}"/><br>
<img src="https://latex.codecogs.com/gif.latex?{p_y}'={p_y}&plus;{v_y}\Delta&space;t&plus;\frac{a_{y}\Delta&space;t&space;^2}{2}"/><br>
<img src="https://latex.codecogs.com/gif.latex?{v_x}'={v_x}&plus;{a_x}\Delta&space;t"/><br>
<img src="https://latex.codecogs.com/gif.latex?{v_y}'={v_y}&plus;{a_y}\Delta&space;t"/><br>

where <img src="https://latex.codecogs.com/gif.latex?\\{a_x}"/> and <img src="https://latex.codecogs.com/gif.latex?\\{a_y}"/> are random acceleration vectors with mean 0 and standard deviations  <img src="https://latex.codecogs.com/gif.latex?\sigma_{a_x}^{2}"/> and <img src="https://latex.codecogs.com/gif.latex?\sigma_{a_y}^{2}"/> respectively. In the project , both values are configured as **9.0**.

The state transition function shown above consists of deterministic part as well as stochastic part represented by the random acceleration vectors. A generalized version of the state transition function is shown below:

<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_x'\\p_y'\\v_x'\\v_y'\end{pmatrix}=\begin{pmatrix}1&0&\Delta&space;t&0\\0&1&0&\Delta&space;t\\0&0&1&0\\0&0&0&1\end{pmatrix}&space;\begin{pmatrix}p_x\\p_y\\v_x\\v_y\end{pmatrix}&space;&plus;&space;\begin{pmatrix}\frac{a_{x}\Delta&space;t&space;^2}{2}\\\frac{a_{y}\Delta&space;t&space;^2}{2}\\a_{x}\Delta&space;t\\a_{y}\Delta&space;t\end{pmatrix}"/>

### Prediction step
This step predicts the new state (x') and uncertainty (P') of the cyclist's position and velocity. The prediction calculation for new state is <br>
<img src="https://latex.codecogs.com/gif.latex?{x}'={F}{x}&plus;\mathit{\nu}"/> <br>
 where **F** is the **state-transition matrix** whose product with the initial state vector gives the state vector at a later time and <img src="https://latex.codecogs.com/gif.latex?\mathit{\nu}"/> refers to the uncertainty in the prediction step or simply **process noise** which is a gaussian distribution with mean 0 and covariance Q  <img src="https://latex.codecogs.com/gif.latex?\[\nu\sim&space;N(0,Q)\]"/>. The F matrix can be derived from the generalized state function:<br>
<img src="https://latex.codecogs.com/gif.latex?\begin{pmatrix}&space;1&0&\Delta&space;t&0\\&space;0&1&0&\Delta&space;t\\&space;0&0&1&0\\&space;0&0&0&1&space;\end{pmatrix}"/>


During the transition of state, the cyclist might have changed direction. This changes the uncertainty of the prediction.

<img src="https://latex.codecogs.com/gif.latex?{P}'={P}{F}{P^T}&plus;{Q}"/><br>
represents this change in uncertainty. The noise variance matrix Q is calculated as :

<img src="https://latex.codecogs.com/gif.latex?Q=\begin{pmatrix}\frac{\Delta&space;t&space;^2}{4}\sigma_{a_x}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t&space;^3}{2}\sigma_{a_x}^{2}&space;&&space;0&space;\\&space;0&space;&&space;\frac{\Delta&space;t&space;^2}{4}\sigma_{a_y}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t&space;^3}{2}\sigma_{a_y}^{2}\\&space;\frac{\Delta&space;t&space;^3}{2}\sigma_{a_x}^{2}&space;&&space;0&space;&&space;{\Delta&space;t&space;^2}\sigma_{a_x}^{2}&space;&&space;0\\&space;0&space;&&space;\frac{\Delta&space;t&space;^3}{2}\sigma_{a_y}^{2}&space;&&space;0&space;&&space;{\Delta&space;t^2}\sigma_{a_y}^{2}&space;\end{pmatrix}"/>

### Update step
#### Define H matrix for Lidar
Lidar sensor measures only two of the four object states (p<sub>x</sub>, p<sub>y</sub>). The **H** matrix projects the belief about the object's current state into the measurement space of the sensor. Multiplying H matrix with x, enables a one to one comparison with new sensor data , z.
```c++
H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
```
##### Define H Matrix for Radar
Incase of Radar sensors, the predicted position and speed are mapped to the polar coordinates of range, bearing and range rate.
![radar_measurement_parameter][image1]

The range **ρ** , is the distance to the pedestrian. The bearing **φ** is the angle between ρ and the x-axis. The range rate,  is the projection of the velocity, v, onto the line, L. In a simplified form, the the **h function** that specifies how the predicted position and speed get mapped to the polar coordinates of range, bearing and range rate is given by the following equation:

<img src="https://latex.codecogs.com/gif.latex?h(x')=\begin{pmatrix}\rho\\\phi\\\dot{\rho}\end{pmatrix}=\begin{pmatrix}\sqrt{{p'}_{x}^{2}&plus;{p'}_{y}^{2}}\\arctan(\frac{{p'}_{y}}{{p'}_{x}})\\\frac{{p'}_{x}{v'}_{x}&plus;{p'}_{y}{v'}_{y}}{\sqrt{{p'}_{x}^{2}&plus;{p'}_{y}^{2}}}\end{pmatrix}"/>

The above matrix consists of non-linear function (Eg:arctan()).
![linear_approximation][image2]

In order to apply Gaussian distributions, this needs to be linearized by a linear function which is tangent to h at the mean location of the original gaussian distribution. This is done with the hep of multivariate Taylor series expansion. A general form of Taylor series expansion is:
<img src="https://latex.codecogs.com/gif.latex?f(x)\approx&space;f(\mu)&plus;&space;\frac{\partial&space;f(\mu)&space;}{\partial&space;x}(x&space;-\mu)" /><br>

As shown in the figure above, when the Taylor expansion is applied, the non-linear function is evaluated first at the mean location μ, followed by a extrapolation with the slope along μ. This slope is given by the first derivate of h, which is called the Jacobian matrix which contains the partial derivatives of H<sub>j</sub>:

<img src="https://latex.codecogs.com/gif.latex?\Large&space;H_j&space;=&space;\begin{bmatrix}&space;\frac{\partial&space;\rho}{\partial&space;p_x}&space;&&space;\frac{\partial&space;\rho}{\partial&space;p_y}&space;&&space;\frac{\partial&space;\rho}{\partial&space;v_x}&space;&&space;\frac{\partial&space;\rho}{\partial&space;v_y}\\&space;\frac{\partial&space;\varphi}{\partial&space;p_x}&space;&&space;\frac{\partial&space;\varphi}{\partial&space;p_y}&space;&&space;\frac{\partial&space;\varphi}{\partial&space;v_x}&space;&&space;\frac{\partial&space;\varphi}{\partial&space;v_y}\\&space;\frac{\partial&space;\dot{\rho}}{\partial&space;p_x}&space;&&space;\frac{\partial&space;\dot{\rho}}{\partial&space;p_y}&space;&&space;\frac{\partial&space;\dot{\rho}}{\partial&space;v_x}&space;&&space;\frac{\partial&space;\dot{\rho}}{\partial&space;v_y}&space;\end{bmatrix}" title="\Large H_j = \begin{bmatrix} \frac{\partial \rho}{\partial p_x} & \frac{\partial \rho}{\partial p_y} & \frac{\partial \rho}{\partial v_x} & \frac{\partial \rho}{\partial v_y}\\ \frac{\partial \varphi}{\partial p_x} & \frac{\partial \varphi}{\partial p_y} & \frac{\partial \varphi}{\partial v_x} & \frac{\partial \varphi}{\partial v_y}\\ \frac{\partial \dot{\rho}}{\partial p_x} & \frac{\partial \dot{\rho}}{\partial p_y} & \frac{\partial \dot{\rho}}{\partial v_x} & \frac{\partial \dot{\rho}}{\partial v_y} \end{bmatrix}" /> <br>

After calculating all the derivates, the final H<sub>j</sub> looks like :
<img src="https://latex.codecogs.com/gif.latex?\Large&space;H_j&space;=&space;\begin{bmatrix}&space;\frac{p_x}{\sqrt[]{p_x^2&space;&plus;&space;p_y^2}}&space;&&space;\frac{p_y}{\sqrt[]{p_x^2&space;&plus;&space;p_y^2}}&space;&&space;0&space;&&space;0\\&space;-\frac{p_y}{p_x^2&space;&plus;&space;p_y^2}&space;&&space;\frac{p_x}{p_x^2&space;&plus;&space;p_y^2}&space;&&space;0&space;&&space;0\\&space;\frac{p_y(v_x&space;p_y&space;-&space;v_y&space;p_x)}{(p_x^2&space;&plus;&space;p_y^2)^{3/2}}&space;&&space;\frac{p_x(v_y&space;p_x&space;-&space;v_x&space;p_y)}{(p_x^2&space;&plus;&space;p_y^2)^{3/2}}&space;&&space;\frac{p_x}{\sqrt[]{p_x^2&space;&plus;&space;p_y^2}}&space;&&space;\frac{p_y}{\sqrt[]{p_x^2&space;&plus;&space;p_y^2}}\\&space;\end{bmatrix}" title="\Large H_j = \begin{bmatrix} \frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}} & 0 & 0\\ -\frac{p_y}{p_x^2 + p_y^2} & \frac{p_x}{p_x^2 + p_y^2} & 0 & 0\\ \frac{p_y(v_x p_y - v_y p_x)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x(v_y p_x - v_x p_y)}{(p_x^2 + p_y^2)^{3/2}} & \frac{p_x}{\sqrt[]{p_x^2 + p_y^2}} & \frac{p_y}{\sqrt[]{p_x^2 + p_y^2}}\\ \end{bmatrix}" />

The function `MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)` performs this calculation. The code for the implementation is taken over from the class notes as-is.

#### Calculate Error
The next step in the measurement process is to calculate the difference between the predicted states from the previous step with what the sensor measurement says.
<img src="https://latex.codecogs.com/gif.latex?{y}={z}{-}{H}{x}"/> <br>

#### Calculate Kalman Gain
The **K** matrix, referred to as the Kalman filter gain, combines the uncertainty of the prediction (**P′**) with the uncertainty of the sensor measurement **R**. The R matrix are generally provided by sensor suppliers. For the current project, the R values were provided by Udacity.
```c++
//measurement covariance matrix - laser
R_laser_ << 0.0225, 0,
            0, 0.0225;

//measurement covariance matrix - radar
R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;
```
<img src="https://latex.codecogs.com/gif.latex?{S}={H}{P}'{H^T}&plus;{R}"/> <br>
<img src="https://latex.codecogs.com/gif.latex?{K}={P}'{H^T}{R}^{-1}"/> <br>

#### Calculate updated object state
With the Kalman gain and the error components, the new state and the uncertainty of the cyclist is derived from the following equations:

<img src="https://latex.codecogs.com/gif.latex?{x}={x}'&plus;{K}{y}"/> <br>
<img src="https://latex.codecogs.com/gif.latex?{P}=\left&space;({I}{-}{K}{H}\right&space;){P}'"/>

## Evaluate Performance
The Root Mean Squared Error (RMSE) method is used to evaluate the performance of the Kalman filter by measuring the deviation of the estimated state from the ground truth. The lower the RMSE, the higher is the accuracy.
The performance evaluation is done in the `tools.cpp`. The code is taken over as-is from the class notes.
```c++
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
```

## Final results
