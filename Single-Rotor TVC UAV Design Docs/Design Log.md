## Modeling
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
$$ F_{lift} = \frac{1}{2} \rho v^2 C_l A_{fin} $$
$$ F_{drag} = \frac{1}{2} \rho v^2 C_d A_{fin} $$
$C_l$ and $C_d$ must be measured. Note that neither of these are constants either; they are functions of the angle of attack (i.e. the servo angle). airfoiltools.com has a database of common airfoils and their parameters which we will use (for more accuracy, we could use a modeling software like XFOIL, but eh).

**Air Foil Choice and Assumptions/Simplifications:**
- NACA 0012 airfoil; common, low drag, symmetrical
- Select Ncrit = 5 (lower = more turbulent air; ideally, Ncrit should be even lower but airfoiltools.com doesn't have data)
- Rough Air Speed estimate: Assuming Motor spins 20,000 RPM (~50% throttle):
	- 20,000 RPM = 2094.4 rad/s
	- Tangential Speed of Propellor Tip $v = \omega r = 133 \; m/s$
		- Simplification 1: air flow velocity varies with radial distance from motor's axis (the propellor tip moves faster than the center of the prop). We pretend like the air flow velocity is constant, equal to the air flow velocity in the middle of a propellor blade (i.e. where the blue arrow is)
![[Airflow_Velocity.png|center]]
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

