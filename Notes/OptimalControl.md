Assuming a large amount of knowledge about our robotic system and its goals, it is sensible to think of directly optimizing for the controller that will perform best on a given task. We assume full knowledge of the form and parameters for all of the following:

- The system's linear dynamics: $$x_t = A*x_t + B*u_t$$
- The task's quadratic objective $$J = \sum{x_t^T*Q*x_t + u_t^*R*u_t}$$
- 
