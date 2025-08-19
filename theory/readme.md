The forward and inverse kinematics problems are solved using the ​Product of Exponentials (PoE) formula, which provides a coordinate-invariant representation of rigid-body motions based on screw theory.
![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/7dof.gif

The inverse dynamics calculation first requires joint angles, angular velocities, and angular accelerations over a time period as inputs. Here, the joint angles are first obtained by solving the inverse kinematics for a heart-shaped curve. The joint angular velocities and accelerations are then computed using the central difference method. Finally, the inverse dynamics computation is performed using the Newton-Euler iteration method from the ModernRobotics-master package.
![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/lovefun.gif

Forward dynamics can be easily solved due to the special properties of the mass matrix and damping matrix. For this second-order differential equation, the backward Euler difference method is employed for numerical solution, all of which are already integrated in the ModernRobotics-master package.

In forward dynamics calculations, given only the initial position, angular velocity, and joint torque curves, we can obtain complete angle and velocity profiles. Here, we compare the results of forward and inverse dynamics computations, with absolute angle errors on the order of 1e-3.

Notably, the backward Euler difference method may sometimes lead to divergent results, failing to produce correct forward dynamics solutions. Potential solutions include reducing the time step size or switching to more stable numerical methods, such as the Runge-Kutta method or BDF methods.

For more detailed theoretical foundations, please refer to the textbook Modern Robotics: Mechanics, Planning, and Control and supplementary online resources. Additionally, I’ve attached the presentation slides from my course project defense, which summarize key implementations and results. Special thanks to my colleague Guan for his excellent work in preparing the slides.
![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/1.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/2.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/3.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/4.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/5.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/6.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/7.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/8.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/9.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/10.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/11.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/12.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/13.png

![image]https://github.com/anonymousob0908/kinematics-and-dynamics-of-robotic-arms/edit/main/theory/14.png

