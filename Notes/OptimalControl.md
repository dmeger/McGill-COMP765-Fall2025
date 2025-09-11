Assuming a large amount of knowledge about our robotic system and its goals, it is sensible to think of directly optimizing for the controller that will perform best on a given task. We assume full knowledge of the form and parameters for all of the following:

- The system's linear dynamics: $x_t = Ax_{t-1} + Bu_{t-1}$
- The task's quadratic objective: $J = \sum_{t=0}^H{x_t^TQx_t + u_tRu_t}$

## Claim
The optimal controller for this system is a linear, time-varying matrix: $$u_t^* = -K_tx_t$$ solveable in closed-form. We will verify and demonstrate this fact by simple construction.

## The Start: $Q_{H-1}$ and $K_{H-1}$ 
Define $Q_t(x_t,u_t)$, named the state-action value function, as the task objective from time $t$ onwards until $H$, that is the $t,t+1,...,H-1,H$ terms in the sum that forms $J$, if we could start the system at state $x_t$ and make control $u_t$ at time $t$. 

Our model knowledge allows us to know both the time $t$ term in the objective: $x_t^TQx_t + u_tRu_t$, and the next state, at time $t+1$, $x_{t+1} = Ax_{t} + Bu_t$. This allows expanding:

$$\begin{aligned} Q_t(x_t,u_t) &=& \sum_{t=t}^H{x_t^TQx_t + u_tRu_t}\\
&=& 
\end{aligned}$$

Observe that when $t=H$, there's nothing left for the system to do. We model no control happening at the very last time-step since we'll cut off the system and the future state can't matter. So, we can write out a $Q_H=x_H^TQx_H$. It doesn't affect an instantaneous decision at time $H$, but gives us something concrete to work with one time-step before.

Now let's consider the task objective one time-step from the end of the horizon. 

$$J_{H-1} = \sum_{t=H-1}^H{x_t^TQx_t + u_t^Ru_t} = $$
