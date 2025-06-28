### Problem Setting

Given a 9-DoF IMU (gyroscope, accelerometer, magnetometer), we wish to estimate the **orientation** of the sensor in 3D space as it moves. We want the best possible estimate, handling both noise and sensor drift.

---

## **Background Concepts**

### **Orientation in 3D**

**Orientation** tells us "which way am I pointing?" in 3D. There are several ways to represent orientation:

* **Euler angles**: Roll, pitch, yaw (susceptible to gimbal lock).
* **Rotation matrix**: $3 \times 3$ orthonormal matrix.
* **Quaternion**: 4-element vector, more numerically stable for computation.

A **quaternion** is often used in filtering because it avoids singularities and is efficient. It’s a four-dimensional extension of complex numbers, representing a rotation as:

$$
\mathbf{q} = \begin{bmatrix} q_w & q_x & q_y & q_z \end{bmatrix}^T
$$

with the constraint $\lVert \mathbf{q} \rVert = 1$. It encodes both an axis and an angle.

### **Sensor Model**

* **Gyroscope**: Measures angular velocity $\boldsymbol{\omega}$.
* **Accelerometer**: Measures gravity + linear acceleration.
* **Magnetometer**: Measures earth’s magnetic field (for heading).

---

## **Kalman Filter Concept**

The **Kalman Filter** is a recursive state estimator for linear systems, providing the optimal (minimum variance) estimate given Gaussian noise. The **Extended Kalman Filter (EKF)** generalizes this to nonlinear systems, by linearizing around the current estimate at each time step.

It has two main steps:

1. **Prediction:** Predict next state using system dynamics.
2. **Update (Correction):** Correct prediction using new sensor measurements.

---

## **EKF for IMU Orientation**

### **State Vector**

Let the state be:

$$
\mathbf{x} = \begin{bmatrix}
\mathbf{q} \\
\mathbf{b}_g
\end{bmatrix}
$$

where:

* $\mathbf{q}$: orientation quaternion (4D)
* $\mathbf{b}_g$: gyroscope bias (3D)

### **Prediction Step**

Use the gyroscope to predict how the orientation changes:

* Gyroscope gives angular velocity $\boldsymbol{\omega}_m$.
* True angular velocity: $\boldsymbol{\omega} = \boldsymbol{\omega}_m - \mathbf{b}_g$.

Quaternion update (to first order):

$$
\mathbf{q}_{k+1} = \mathbf{q}_k + \frac{1}{2}\mathbf{q}_k \otimes
\begin{bmatrix} 0 \\ \boldsymbol{\omega} \Delta t \end{bmatrix}
$$

where $\otimes$ is the quaternion product.

Bias is assumed to be constant or modeled as random walk:

$$
\mathbf{b}_{g, k+1} = \mathbf{b}_{g, k} + \text{noise}
$$

### **Update (Correction) Step**

We use accelerometer and magnetometer to correct drift:

* **Accelerometer** measures direction of gravity in the sensor frame.
* **Magnetometer** measures earth’s magnetic field direction.

Given our current orientation estimate, we can **predict** what the accelerometer and magnetometer *should* measure (if there was no linear acceleration or magnetic disturbance). If measured values deviate, we update (correct) our orientation estimate.

#### **Measurement Model**

* Predicted gravity in sensor frame: $\mathbf{g}_s = \mathbf{R}^T \mathbf{g}_w$, where $\mathbf{R}$ is the rotation matrix for the quaternion, $\mathbf{g}_w = [0, 0, g]^T$.
* Predicted magnetic field: $\mathbf{m}_s = \mathbf{R}^T \mathbf{m}_w$.

Measured values:

* Accelerometer: $\mathbf{a}_m$
* Magnetometer: $\mathbf{m}_m$

The difference between prediction and measurement forms the **innovation** (error), which is then used to update the state estimate.

---

## **Linearization**

The EKF linearizes the system about the current state estimate, calculating the **Jacobian** of the process and measurement models, so that standard Kalman filter equations can be used for correction.

---

## **Algorithm Summary**

For each IMU update:

1. **Predict**

   * Propagate orientation using gyroscope measurement, accounting for current bias.
   * Predict new state and error covariance.
2. **Update**

   * Calculate expected accelerometer/magnetometer readings based on predicted orientation.
   * Compute error between predicted and measured values.
   * Linearize measurement model about current state (compute Jacobian).
   * Update state estimate and error covariance.

---

## **Pseudocode**

$$
\begin{align*}
\text{Given:} & \ \mathbf{x}_k, \mathbf{P}_k \\
\text{Gyro update:} & \ \mathbf{q}_{k+1} = \mathbf{q}_k + \frac{1}{2} \mathbf{q}_k \otimes [0, \boldsymbol{\omega}_m - \mathbf{b}_g] \Delta t \\
& \ \mathbf{b}_{g, k+1} = \mathbf{b}_{g, k} \\
\text{Prediction:} & \ \mathbf{x}_{k+1|k}, \mathbf{P}_{k+1|k} \\
\text{Measure:} & \ \mathbf{a}_m, \mathbf{m}_m \\
\text{Compute error:} & \ \mathbf{y} = \begin{bmatrix} \mathbf{a}_m - \mathbf{g}_s(\mathbf{q}_{k+1|k}) \\ \mathbf{m}_m - \mathbf{m}_s(\mathbf{q}_{k+1|k}) \end{bmatrix} \\
\text{Update:} & \ \mathbf{x}_{k+1|k+1}, \mathbf{P}_{k+1|k+1} \\
\end{align*}
$$

---

## **Summary**

* The EKF fuses fast gyroscope data (which drifts over time) with slow, noisy but drift-free references from accelerometer and magnetometer.
* The state is orientation (as a quaternion) and gyroscope bias.
* Prediction: propagate orientation using gyroscope.
* Correction: correct orientation with accelerometer/magnetometer readings.
* Linearize nonlinear models for correction (EKF step).

Further details (e.g., explicit quaternion math, implementation) can be given as needed.
