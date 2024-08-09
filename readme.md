
---

### Installation and Setup Instructions

**1. Download the `frcobot_ros2` Package:**
   - Clone the package from the GitHub repository:
     ```bash
     git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git
     ```

**2. Download MoveIt2 from Source:**
   - Follow the instructions to download and set up MoveIt2 from source by visiting the MoveIt2 [Getting Started Guide](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).

**3. Build the Packages:**
   - Use `colcon` to build the workspace:
     ```bash
     colcon build
     ```

**4. Source the Workspace:**
   - Source the workspace to overlay it on your environment:
     ```bash
     source install/setup.bash
     ```

**5. Test the Launch File:**
   - Run the following command to test the launch file:
     ```bash
     ros2 launch fairino3_v6_moveit2_config demo.launch.py
     ```

**6. Handle Potential Error (`'capabilities'`):**
   - If you encounter the following error:
     ```
     Caught exception in launch (see debug for traceback): 'capabilities'
     ```
   - Visit the GitHub issue [#2738](https://github.com/moveit/moveit2/issues/2738) and follow the solution provided by user `mink007`.

**7. (Optional) Skip Rebuilding:**
   - **Note:** Rebuilding the entire workspace using `colcon build` can take 10-20 minutes. If you applied the fix mentioned above, you can skip rebuilding. Simply source the workspace again:
     ```bash
     source install/setup.bash
     ```

**8. Re-run the Launch File:**
   - After applying the fix, try running the launch file again:
     ```bash
     ros2 launch fairino3_v6_moveit2_config demo.launch.py
     ```
   - **Note:** You can safely ignore the warning:
     ```
     "Using load_yaml() directly is deprecated. Use xacro.load_yaml() instead."
     ```

**9. Add a `colcon.ignore` File:**
   - If the above command runs successfully, you can add a `colcon.ignore` file in the MoveIt2 folder to avoid rebuilding it in the future. Navigate to the `src` directory and create the file:
     ```bash
     touch src/moveit2/colcon.ignore
     ```

---
