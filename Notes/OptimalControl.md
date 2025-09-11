# Starting Point: Linear Quadratic Regulators (LQR)

Assuming a large amount of knowledge about our robotic system and its goals, it is sensible to think of directly optimizing for the controller that will perform best on a given task. To start, we assume full knowledge of a simple form and parameters for all of the following:

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

$$\begin{aligned} J_{H-1} &=& \sum_{t=H-1}^H{x_t^TQx_t + u_t^Ru_t} \\
&=& x_{H-1}^TQx_{H-1} + u_tRu + x_{H}^TQx_{H}.
\end{aligned}$$

These three terms are simple and allow some nice analysis with calculus. First, we can expand $x_H$ using our known linear dynamics, $x_H=Ax_{H-1}+Bu_{H-1}$. This can substitute into the 3rd term above, followed by simple algebraic expansion and factoring:

$$\begin{aligned} J_{H-1} &=& x_{H-1}^TQx_{H-1} + u_tRu + x_{H}^TQx_{H} \\
&=& x_{H-1}^TQx_{H-1} + u_tRu + (x_{H-1}+Bu_{H-1})^TQ(Ax_{H-1}+Bu_{H-1}) \\
&=& x_{H-1}^TQx_{H-1} + u_{H-1}Ru_{H-1} + x_{H-1}^TA^TQAx_{H-1} + 2u_{H-1}^TB^TQAx_{H-1} + u_{H-1}^TB^TQBu_{H-1}. 
\end{aligned}$$

This mess can be made manageable by noticing we only care about finding $u^*$. The mess is at least obviously quadratic in $u$, with positive-definite coefficient matrices. It's minimum occurs where $\frac{\partial{J}}{\partial{u_{H-1}}}=0$. We continue... 

$$\begin{aligned}
\frac{\partial{J}}{\partial{u_{H-1}}} &=&  2Ru_{H-1} + 2B^TQAx_{H-1} + 2B^TQBu_{H-1}. 
\end{aligned}$$

Which is zero when
$$\begin{aligned}
Ru_{H-1} + B^TQBu_{H-1} &=& -B^TQAx_{H-1} \\
 (R + B^TQB)u_{H-1} &=& -B^TQAx_{H-1}  \\
u_{H-1} &=& -(R + B^TQB)^{-1}B^TQAx_{H-1} \\
u_{H-1}^* &=& -K_{H-1}x_{H-1}.
\end{aligned}$$

We have accomplished the form of our claim for one very special time-step! It's time to tidy things up and get ready to attempt the solution for all $H-2$ previous time-steps.

We can plug the new form for $u$ into our expression for $J_{H-1}$. 

$$\begin{aligned} J_{H-1} &=& x_{H-1}^TQx_{H-1} + u_{H-1}Ru_{H-1} + x_{H-1}^TA^TQAx_{H-1} + 2u_{H-1}^TB^TQAx_{H-1} + u_{H-1}^TB^TQBu_{H-1} \\
&=& x_{H-1}^TQx_{H-1} + x_{H-1}^TK^TRKx_{H-1} + x_{H-1}^TA^TQAx_{H-1} - 2x_{H-1}K^TB^TQAx_{H-1} + x_{H-1}K^TB^TQBKx_{H-1} \\
&=& x_{H-1}^T( Q+K^TRK + A^TQA-2K^TB^TQA + K^TB^TQBK )x_{H-1}.
\end{aligned}$$

The final line is a simple quadradic cost in $x_{H-1}$, which we can express as $x_{H-1}^TP_{H-1}x_{H-1}$. Finally we can make use of the previously useless seeming statment, which we will now usefully restate: $J_H = x_H^TP_Hx_H$. The form is the same! 

So, consider what will happen when we write out J_{H-2}. We're going to stop spamming lists of symbols and apply our left brains here:

- J_{H-2} can be formed of three terms, immediate state, immediate control and next state quadratic. The pattern is identical to the one we completed.
- We can apply the known linear dynamics to expand the next state, expand, take derivative and set to zero.
- Our answer will be $u_{H-2}^* = -K_{H-2}x_{H-2}$. 
- We can plug back in to the expanded form of $J_{H-2}$ and factor again to a new quadratic, $x_{H-2}^TP_{H-2}x_{H-2}$.

Generalize. There are two alternating forms that will go back to the start of the episode:

$$\begin{aligned}
J_t &=& x_t^TP_tx_t \\
u_t^* &=& -K_tx_t.
\end{aligned}$$

Our claim is supported!

## Non-linear dynamics, General-form costs and Constraints

Real robots are not linear, as we have discussed previously. The general form for dyanmics is $x_t = f(x_{t-1},u_{t})$. We can still use the technique of dynamic programming we applied above to break-down the overall objective into instantaneous and future sum of costs. The derivative will not allow closed-form solution for $u^*$ in almost any case. Taylor expansion around sensible guesses of the parameters is possible, which leads to a method known as Iterative LQR (similar Differential Dynamic Programming by Jacobsen and Mayne). The normal problems of linearization exist; if we choose the wrong linearization point or update our parameters too far from this point, computations lose accuracy.

An important analysis tool are variational principles that define properties of optimizing solutions. For problems with sufficient structure, these tools can allow direct solution in parametric form, but for arbitrary problems, we must rely on computation.

Therefore, several components and considerations are common:
- How to consider a plausible set of points at which to evaluate our system. Ideas here can be called shooting and co-location.
- How to perform control updates including ideas like line searches, conjugate gradients, relative entropy regularization and projected gradients. 
- How to handle constraints and incorporate them into state trajectories and satisficing controls. Includes concepts such as Lagrange multipliers, dual and slack methods.

We will not dive further into the very large and active area of designing and implementing non-linear optimal control solutions in this introductory portion of the course. As we move to advanced topics, we'll see that today, Deep RL approaches have the potential to be used on the problem, especially when model knowledge is missing or unreliable. However, when we do know aspects of the model well, even a large network with lots of data can be assisted by warm-starting or other guidance from model guidance in some form. More on this to come!

(Ex 2.1) Consider a rocket-powered hockey puck sliding back and forth on 1D ice. The state contains $x$ and $\dot{x}$ and control is a direct acceleration $u=\ddot{x}$. 

$$x_t = \begin{bmatrix} 1 & 1 \\ 0 & 1\end{bmatrix}x_{t-1} + \begin{bmatrix} 0 \\ 1 \end{bmatrix}u_{t}$$

Implement an LQR solution for this problem (or use a Python library). Explore the parameters Q and R and observe their effect on the outcome control solution.

(Ex 2.2) Extend the formulation to a 2D rocket-puck. You must be able to generate controls in any global direction (that is, do not include a model of puck rotation), to maintain the linear nature of the system. What trajectories can you make the puck follow and does it reach the goal at (0,0)?

(Enrichment - Not examinable) Read about how the time varying finite horizon solution we described here can be extended to an infinite horizon solution. 
