# 🧍‍♂️ Human Postural Stability Control: A State-Space Approach

![MATLAB](https://img.shields.io/badge/MATLAB-e16737?style=for-the-badge&logo=mathworks&logoColor=white)
![Simulink](https://img.shields.io/badge/Simulink-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)
![ControlTheory](https://img.shields.io/badge/Control_Theory-State_Space_%7C_Luenberger-success?style=for-the-badge)
![Biomechanics](https://img.shields.io/badge/Domain-Biomechanics-purple?style=for-the-badge)

A dynamical systems theory project focused on the mathematical modeling, analysis, and active control of human postural stability. Developed for the *Dynamical Systems Theory* course at Università della Calabria.

This project approximates the human body as an **inverted pendulum hinged at the ankle** and utilizes modern state-space control techniques to stabilize the inherently unstable upright posture against external perturbations.

<p align="center">
  <img src="Immagini\simulink_total.png" width="600" alt="Simulink model overview">
</p>
*Simulink architecture integrating body dynamics, muscle activation, Full State Feedback, and the Luenberger Observer.*

## 🚀 Engineering Highlights

* **Biomechanical Non-Linear Modeling:** Modeled the human body dynamics using anthropometric standards (Winter) combining the Center of Mass (COM) and Center of Pressure (COP) kinematics.
* **Neuromuscular Actuation:** Integrated a first-order differential equation to simulate the muscle activation time constant ($\tau_a$), separating neural commands from physical torque generation.
* **State-Space Linearization:** Extracted equilibrium points and linearized the non-linear equations using Jacobian matrices to enable linear system analysis.
* **Full State Feedback (FSF) Control:** Designed a static state feedback controller via Ackermann's formula to shift the unstable open-loop poles to the left-half complex plane.
* **Asymptotic State Estimation:** Implemented a **Luenberger Observer** to estimate unmeasured internal states (angular velocity and muscle torque) relying solely on the angular position sensor ($\theta$).

## 🧠 System Analysis & Instability

The human upright posture is naturally unstable. Through mathematical modeling, the system was translated into an $I/S/O$ continuous-time representation with three states:
1. $x_1 = \theta$ (Angular position)
2. $x_2 = \omega$ (Angular velocity)
3. $x_3 = \tau_a$ (Muscle activation torque)

The open-loop analysis of the linearized state matrix ($A$) revealed an eigenvalue in the right-half plane ($\lambda \approx 4.09$). The impulse response simulation confirmed that without active neuromuscular control, the system diverges exponentially (the person falls).

<p align="center">
  <img src="Immagini\impulse response.png" width="45%" title="Open Loop - Diverging" />
  <img src="Immagini\impulse_response_cl.png" width="45%" title="Closed Loop - Stabilized" />
</p>
*Left: Unstable open-loop impulse response. Right: Asymptotically stabilized closed-loop response.*

## 🛠️ Control Synthesis

Before synthesizing the controller, structural properties were verified:
* **Reachability:** Verified full controllability ($rank(R) = 3$), allowing arbitrary pole placement.
* **Observability:** Verified full observability ($rank(O) = 3$), ensuring that all states can be reconstructed from the angular position output.

### 1. The Controller (Ackermann's Formula)
To stabilize the pendulum, the closed-loop poles were aggressively placed at $\lambda = [-2.5, -3, -10]$. The resulting feedback gain matrix $K$ effectively limits the angular displacement ($\theta$) and smoothly returns the system to $0$ rad after an initial perturbation of $0.1$ rad.

### 2. The Luenberger Observer
In real-world scenarios, equipping a patient/robot with sensors for every internal state is impractical. An asymptotic observer was designed with faster poles (e.g., $[-15, -20, -30]$) to track the real system. 
The simulations prove that the estimation error rapidly converges to zero $lim_{t \to \infty} e(t) = 0$, allowing the controller to rely entirely on the estimated states $\hat{x}$.


*Performance of the Luenberger Observer: The estimated angular velocity (blue) rapidly converging to the real physical state (yellow).*

## 💻 Repository Structure
* `/MATLAB_Scripts/`: Contains the `.m` files for parameter initialization, equilibrium point extraction (`trim`), linearization (`linmod`), and controller/observer matrix computation.
* `/Simulink_Models/`: Contains the `.slx` files.
  * `Plant_Model.slx`: The non-linear open-loop system.
  * `ControllerFSF.slx`: The closed-loop system with State Feedback.
  * `ControllerObserver.slx`: The complete system integrating the Luenberger observer.
* `/Report/`: Contains the full 40-page PDF project report detailing the mathematical proofs and simulation results.
