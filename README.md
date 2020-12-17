## A ROS implementation of a 6-DoF EKF for indoor drone visual SLAM

The current work is part of the final Thesis for my MSc. in Computer Science & Engineering @ Politecnico di Milano (Dec. 2020).
The final document can be found [here](A%20ROS%20implementation%20of%20a%206-DoF%20EKF%20for%20indoor%20drone%20Visual%20SLAM.pdf), and the final presentation can be found [here](Presentation.pptx)
 * **Advisor**: Prof. Matteo Matteucci.
 * **Co-Advisor**: Ing. Simone Mentasti.

### Abstract
An implementation of a localization and mapping algorithm is proposed in the context of the Leonardo Drone Contest, where specific constraints and characteristics apply. The drone’s constraints include the prohibition of usage of GNSS or laser devices, which enforces the need for the localization algorithm to be robust enough. Moreover, the characteristics of the environment make it possible to use a visual-based localization algorithm. On the other hand, the contest requires to map specific landmarks in the environment in order to be able to follow those landmarks mapped in a sequence of take-off and landing in a specific order. In this sense, the algorithm is forced to map these landmarks so their coordinates can be used later on.

The current work proposes an implementation of an EKF-SLAM algorithm, and tries to shed some light on the importance of the usage of these landmarks, while trying to identify implementation details that increase the performance and robustness of the proposed algorithm.
