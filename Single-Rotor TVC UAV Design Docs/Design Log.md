## Characterization & Modeling
![[FBD.png|center]]
<center> Free Body Diagram for Reference </center>

#### Thrust to Rotational Velocity Model
$$F_t(\omega) = K_f \omega_t^2$$
![[Thrust_to_Rotational_Velocity_Model.png | center]]
Measurement Procedure:
- Plot $F_t$ vs $\omega_t^2$ and find linear fit coefficient $K_f$

#### Rotational Velocity to Control Input Model

Motor is modeled as a 1st-order system: $\omega_t(t) = \omega_{\text{initial}} + \left(K_t u_t - \omega_{\text{initial}}\right)\left(1 - e^{-t / \tau}\right)$
- $K_t$ and $\tau_t$ are constants
- Basically, 1st-order system models linear velocity-to-control-input relationship, but with some time needed to get up to speed due to motor’s inertia and resistance
- For reference, this is what the rotational velocity vs time curve looks like (when $\omega_{\text{initial}} = 0$):
![[Rotational_Velocity_to_Control_Input_Model.png|center]]
- $\omega_t$ isn't incorporated into the state vector, but is an intermediate to calculate $F_t$ from $u_t$: $$F_t[k+1] = K_f \left( \omega_t[k] + \frac{\Delta t}{\tau_t} \left(-\omega_t[k] + K_t u_t[k]\right) \right)^2$$
To measure $K_t$ and $\tau_t$, ramp the motor up from $\omega_{\text{initial}}=0$ to some $\omega$ setpoint and use Matlab or similar to fit a 1st-order model to the curve. $\tau_t$ for a typical 2207 motor is expected to be ~30 ms.

Discrete-time version of the model: $\omega_t[k+1] = \omega_t[k] + \frac{\Delta t}{\tau_t} \left(-\omega_t[k] + K_t u_t\right)$

NOTE: in the worst case, a simplified model may be used where you only model the proportional relationship between $\omega_t$ and $u_t$ and pretend $\tau_t=0$. Then, $\omega_t = K_t u_t$.

#### Thrust Vane Angles to Lift and Drag Forces Model

Servo dynamics are modeled as a single gain. They are assumed to move fast enough (1000 deg/sec) that we basically have direct position control. We just need a conversion factor $K_a$ to go from PWM command to servo angle in radians:
$$ \alpha_n = K_a u_{s_n}$$
The thrust vanes are modeled as air foils generating a lift and drag force, $F_{lift}, F_{drag}$.
$$ F_{lift} = \frac{1}{2} \rho v^2 C_{l \alpha} A_{fin} $$
$$ F_{drag} = \frac{1}{2} \rho v^2 C_{d \alpha} A_{fin} $$
Note that these equations assume $v$, the air speed (a scalar), is in the direction of $\alpha=0$ (zero angle of attack). $F_{lift}$ is perpendicular to $v$ and $F_{drag}$ is parallel to $v$.

$A_{fin}$ is the span of the airfoil times the chord length.

One simplification we will make is to treat $v$, the air speed, as a constant along the entire airfoil. In reality, the air flow velocity varies with radial distance from motor's axis (the propellor tip moves faster than the center of the prop). We simply take the average airflow velocity along the propellor blade, equal to the air flow velocity in middle of a propellor blade (i.e. where the blue arrow is), which should be roughly where the center of the airfoil is.
![[Airflow_Velocity.png|center]]
To actually calculate the air speed, the obvious solution would be to try to model it relative to the rotational velocity of the propellor, but we don't know the exact dynamics of the propellor. Instead, we model the air speed relative to the estimated thrust force. We know the momentum of the air is equal to $p = A_{duct} \rho v$. $F = pv \; \rightarrow \; F_t = A_{duct} \rho v^2$. Therefore, we have this relationship: 
$$v^2 = \frac{F_t}{A_{duct} \rho} = \frac{K_f \omega_t^2}{A_{duct} \rho}$$
$C_{l \alpha}$ and $C_{d \alpha}$ must also be measured. Note that neither of these are constants either; they are functions of the angle of attack $\alpha$ (i.e. the servo angle). airfoiltools.com has a database of common airfoils and their parameters which we will use. It's common, for small $\alpha$, to approximate these as affine functions of $\alpha$.

**Air Foil Choice and Assumptions/Simplifications:**
- NACA 0012 airfoil; common, low drag, symmetrical
- Select Ncrit = 5 (lower = more turbulent air; ideally, Ncrit should be even lower but airfoiltools.com doesn't have data)
- Rough Air Speed estimate: Assuming Motor spins 20,000 RPM (~50% throttle):
	- 20,000 RPM = 2094.4 rad/s
	- Tangential Speed of Propellor Tip $v = \omega r = 133 \; m/s$
		- Simplification 1: same as above, we treat $v$, the air speed, constant.
		- Tangential Speed of Propellor Blade center (i.e. green arrow): $66.5 \; m/s$
	- Assuming axial flow velocity ~ 70% of propellor tip speed:
		- $0.7 * 66.5 = 46.55 \; m/s$ estimated axial air flow velocity
- Reynolds Number Calculation: 
	- Assuming 50mm Air Foil chord length
	- Simplification 2: Reynolds number (and therefore $C_l$ and $C_d$) varies with propellor RPM. We bucket define a few buckets that fall under different Reynolds numbers. The system can use these different system models depending on the operating domain/motor RPM.
		- 679 rad/s --> 50,000 Reynolds Number [Reynolds number calculator](http://airfoiltools.com/calculator/reynoldsnumber?MReNumForm%5Bvel%5D=15.1&MReNumForm%5Bchord%5D=0.05&MReNumForm%5Bkvisc%5D=1.5111E-5&yt0=Calculate)
		- 1359 rad/s --> 100,000 Reynolds Number [Reynolds number calculator](http://airfoiltools.com/calculator/reynoldsnumber?MReNumForm%5Bvel%5D=30.2&MReNumForm%5Bchord%5D=0.05&MReNumForm%5Bkvisc%5D=1.5111E-5&yt0=Calculate)
		- 2722 rad/s --> 200,000 Reynolds Number [Reynolds number calculator](http://airfoiltools.com/calculator/reynoldsnumber?MReNumForm%5Bvel%5D=60.5&MReNumForm%5Bchord%5D=0.05&MReNumForm%5Bkvisc%5D=1.5111E-5&yt0=Calculate)
	- [NACA 0012 AIRFOILS (n0012-il)](http://airfoiltools.com/airfoil/details?airfoil=n0012-il) contains data $C_d$ and $C_l$ data for these Reynolds numbers
![[NACA_0012_Data.png|center]]

#### Motor & Propellor Moment of Inertia Model

When the motor & prop accelerate, a z-axis torque is applied. Their combined MoI ($J_m$) is needed to estimate this torque.
- Tape a piece of string onto the motor bell (w/ prop attached). Wind it up like a spool.
- At the bottom of the string, attach a known mass
- Let the mass free-fall, accelerating the motor and prop. Measure the rate of acceleration
- Calculate $J_m = \frac{\tau_{\text{string+mass}}}{\alpha_{\text{motor+prop}}}$

The resulting torque applied to the UAV (equal to the negative of the torque required to accelerate the motor & prop) is calculated: $$\tau_z^{motor} = -J_m \dot{\omega}_t$$
$\dot{\omega}_t$ can be estimated as $\frac{\omega_t[k+1] - \omega_t[k]}{\Delta t}$. Recall above, we model the motor as a 1st order system: $\omega_t[k+1] = \omega_t[k] + \frac{\Delta t}{\tau_t} \left(-\omega_t[k] + K_t u_t\right)$. Manipulating this equation: $\frac{\omega_t[k+1] - \omega_t[k]}{\Delta t} = \frac{1}{\tau_t} \left(-\omega_t[k] + K_t u_t\right)$. Plugging this into the torque equation yields:
$$ \tau_z^{motor} = -\frac{J_m}{\tau_t} \left(-\omega_t[k] + K_t u_t\right) $$

#### Forces and Torques
##### Applied Forces
Thrust Force:
$$F_t(\omega) = K_f \omega_t^2$$
Thrust Vane Lift Force for vane $n$:
$$ F_n = F_t \frac{C_{l\alpha} A_{{fin}}}{2 A_{{duct}}} \quad \forall n=1,2,3,4 $$
Thrust Vane Drag Force for vane $n$:
$$ F_{dn} = F_t \frac{C_{d \alpha} A_{{fin}}}{2 A_{{duct}}} \quad \forall n=1,2,3,4 $$

##### Body-Frame Force and Torques
Body-Frame Forces:
$$\begin{bmatrix}
F_z \\
F_y \\
F_z
\end{bmatrix}
=
\begin{bmatrix}
F_2 + F_4 \\
F_1 + F_3 \\
F_t - \sum_{n=1}^4 F_{dn}
\end{bmatrix}
-
{}^B \mathbf{R}^W
\begin{bmatrix}
0 \\
0 \\
mg
\end{bmatrix}$$
Body-Frame Torques: 
$$ \begin{bmatrix}
\tau_x \\
\tau_y \\
\tau_z
\end{bmatrix}
=
\begin{bmatrix}
(F_1 + F_3)l \\
-(F_2 + F_4)l \\
(F_1 - F_2 - F_3 + F_4)r + \tau_z^{motor}
\end{bmatrix} $$

##### State Space Form ($\dot{x} = f(x,u)$)

State representation: 
$$ [x, y, z, v_x, v_y, v_z, R_1, R_2, R_3, R_4, R_5, R_6, R_7, R_8, R_9, \omega_x, \omega_y, \omega_z]^\top$$
- Contains position (in world frame), linear velocity (in world frame), Rotation matrix (${}^W R^B$), angular velocity (in body frame)

