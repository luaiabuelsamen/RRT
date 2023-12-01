# RRT Algorithm Implementation for Robotics

This project implements the Rapidly-exploring Random Tree (RRT) algorithm for different types of robots in a 2D plane. The implementation focuses on point and line robots and explores various sampling strategies and step sizes.


## Discussion and Results

### A. RRT with Uniform Sampling

![Alt text](./Examples/RRT1)
![Alt text](./Examples/RRT2)
![Alt text](./Examples/RRT3)
- **Objective**: Implement RRT for an omnidirectional point robot using uniform sampling.
- **Results**:
  - Figures 1 and 2 show results for 'shot.png' and 'simple.png' with a point robot.
  - The RRT expands across the map without specific knowledge of the target position.
  - Figure 3 demonstrates guiding point visualization.

### B. RRT with Gaussian Distribution

![Alt text](./Examples/RRT4)
![Alt text](./Examples/RRT5)
- **Objective**: Implement RRT with Gaussian distribution centered on the target.
- **Results**:
  - Figure 4 shows the RRT implementation for a point robot with Gaussian distribution.
  - The implementation focuses more on the target area, as seen in Figure 5.

### C. Varying Timestep with Gaussian Distribution

![Alt text](./Examples/RRT6)
![Alt text](./Examples/RRT7)
- **Objective**: Analyze the effect of varying step size on RRT with Gaussian distribution.
- **Results**:
  - Figures 6 and 7 depict the number of nodes vs. step size and path length vs. step size, respectively.
  - Larger step sizes lead to faster exploration but less smooth paths.

## Section 2: Line Robot

### A. Uniform Sampling with Line Robot

![Alt text](./Examples/Picture1)
![Alt text](./Examples/RRT10)
- **Objective**: Implement RRT for a line robot using uniform sampling.
- **Results**:
  - Figures 8 and 9 show results for 'shot.png' and 'simple.png' with a line robot.
  - The line robot avoids incorrect paths more effectively compared to the point robot.

![Alt text](./Examples/Picture2)
### B. Varying Robot Length for Line Robot

- **Objective**: Study the impact of different robot lengths on the line robot's performance.
- **Results**:
  - Figure 10 demonstrates iterations vs. robot size.
  - Larger robots navigate more efficiently, as indicated by a general decreasing trend in iterations.

## Conclusion

This project demonstrates the versatility of the RRT algorithm in robotic path planning and highlights the effects of different sampling strategies and robot configurations on its performance.
