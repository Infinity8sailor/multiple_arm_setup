# multiple_arm_setup
Docs Reference for moveit tutorial on ur5 multiple arms

Open three shells. in first shell Start gazebo and Spawn the model

  ```bash
  roslaunch multiple_ur_description multiple_ur5_robotiq.launch
  ```

In the second shell, here Start Bring Up file for manipulation

```bash
  roslaunch multiple_ur_manipulation bringup.launch
  ```

In the third shell, Run the cpp Script

```bash
  rosrun multiple_arms_scripts multiple_arm_move
  ```
