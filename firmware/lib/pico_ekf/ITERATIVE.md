### **Iterated Extended Kalman Filter (IEKF) for IMU Orientation**

The **Iterated Extended Kalman Filter (IEKF)** is a refinement of the standard EKF that improves accuracy, especially in the presence of highly nonlinear measurement models—such as those involved in orientation estimation with IMUs.

---

## **Motivation**

The standard EKF linearizes the measurement model only **once** per correction step, around the *a priori* (predicted) state estimate. If the measurement model is highly nonlinear (as is the case for quaternions, gravity and magnetic field projections), this linearization may be poor, leading to suboptimal correction.

**IEKF** addresses this by repeatedly re-linearizing the measurement model around the *posterior* (corrected) estimate, effectively performing a Gauss-Newton iteration to maximize the likelihood (minimize innovation).

---

## **Algorithm Structure**

Let the state be

$$
\mathbf{x} = \begin{bmatrix} \mathbf{q} \\ \mathbf{b}_g \end{bmatrix}
$$

as before.

### **Step 1: Prediction**

Propagate state and covariance as in the EKF:

$$
\mathbf{x}_{k+1|k} = f(\mathbf{x}_k, \mathbf{u}_k)
$$

$$
\mathbf{P}_{k+1|k} = \mathbf{F}_k \mathbf{P}_k \mathbf{F}_k^T + \mathbf{Q}_k
$$

where $f(\cdot)$ is the nonlinear process (quaternion update), $\mathbf{F}_k$ is the Jacobian, and $\mathbf{Q}_k$ is the process noise.

### **Step 2: Measurement Update (Iterative Correction)**

Given measurement $\mathbf{z}_k$ (accelerometer and/or magnetometer):

#### **Initialization**

* Set initial guess for the corrected state: $\hat{\mathbf{x}}^{(0)} = \mathbf{x}_{k+1|k}$.

#### **Iterate**

For $i = 0, 1, \ldots, N$ (until convergence, typically $N=1$ to $5$):

1. **Linearize** the measurement function $\mathbf{h}$ about $\hat{\mathbf{x}}^{(i)}$:

   $$
   \mathbf{H}^{(i)} = \left. \frac{\partial \mathbf{h}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}^{(i)}}
   $$
2. **Compute Innovation**:

   $$
   \mathbf{y}^{(i)} = \mathbf{z}_k - \mathbf{h}(\hat{\mathbf{x}}^{(i)})
   $$
3. **Kalman Gain**:

   $$
   \mathbf{K}^{(i)} = \mathbf{P}_{k+1|k} \left( \mathbf{H}^{(i)} \right)^T \left( \mathbf{H}^{(i)} \mathbf{P}_{k+1|k} \left( \mathbf{H}^{(i)} \right)^T + \mathbf{R}_k \right)^{-1}
   $$
4. **Update State Estimate**:

   $$
   \hat{\mathbf{x}}^{(i+1)} = \hat{\mathbf{x}}^{(i)} + \mathbf{K}^{(i)} \mathbf{y}^{(i)}
   $$
5. **(Optional) Re-normalize quaternion part** to ensure unit norm.

#### **Convergence**

Stop when $\lVert \hat{\mathbf{x}}^{(i+1)} - \hat{\mathbf{x}}^{(i)} \rVert$ is small or after a fixed number of iterations.

#### **Finalize**

After convergence, update the covariance:

$$
\mathbf{P}_{k+1|k+1} = \left( \mathbf{I} - \mathbf{K}^{(N)} \mathbf{H}^{(N)} \right) \mathbf{P}_{k+1|k}
$$

---

## **Pseudocode Outline**

$$
\begin{align*}
\text{Given:} & \ \mathbf{x}_{k+1|k}, \mathbf{P}_{k+1|k}, \mathbf{z}_k \\
\hat{\mathbf{x}}^{(0)} & = \mathbf{x}_{k+1|k} \\
\text{for } i = 0, 1, \ldots, N: \\
\quad & \mathbf{H}^{(i)} = \left. \frac{\partial \mathbf{h}}{\partial \mathbf{x}} \right|_{\hat{\mathbf{x}}^{(i)}} \\
\quad & \mathbf{y}^{(i)} = \mathbf{z}_k - \mathbf{h}(\hat{\mathbf{x}}^{(i)}) \\
\quad & \mathbf{K}^{(i)} = \mathbf{P}_{k+1|k} \left( \mathbf{H}^{(i)} \right)^T \left( \mathbf{H}^{(i)} \mathbf{P}_{k+1|k} \left( \mathbf{H}^{(i)} \right)^T + \mathbf{R}_k \right)^{-1} \\
\quad & \hat{\mathbf{x}}^{(i+1)} = \hat{\mathbf{x}}^{(i)} + \mathbf{K}^{(i)} \mathbf{y}^{(i)} \\
\quad & \text{(Re-normalize quaternion part)} \\
\text{End For} \\
\mathbf{x}_{k+1|k+1} & = \hat{\mathbf{x}}^{(N)} \\
\mathbf{P}_{k+1|k+1} & = (\mathbf{I} - \mathbf{K}^{(N)} \mathbf{H}^{(N)}) \mathbf{P}_{k+1|k}
\end{align*}
$$

---

## **Remarks**

* Each iteration moves the state estimate closer to the "maximum likelihood" solution for the (nonlinear) measurement.
* This makes the IEKF more robust to initial linearization error, especially with large innovations or strong nonlinearity (as is common with IMU orientation).
* Computational cost is slightly higher, but generally small for typical IMU AHRS problems.
* For small deviations, IEKF reduces to EKF (one iteration).

---

## **Summary Table: EKF vs IEKF**

| Filter | Linearization    | Correction per step | Performance in highly nonlinear settings                        |
| ------ | ---------------- | ------------------- | --------------------------------------------------------------- |
| EKF    | Once (a priori)  | Single              | May be suboptimal if measurement function is strongly nonlinear |
| IEKF   | Multiple (iter.) | Iterative           | More robust, better convergence, higher accuracy                |

---

## **Application to IMU**

The IEKF is well-suited for quaternion-based orientation fusion when using IMUs, since the measurement functions (projection of gravity and magnetic field onto the current body orientation) are strongly nonlinear.

---

If you want a more detailed example (with explicit quaternion math, Jacobians, or code), this can be expanded.
