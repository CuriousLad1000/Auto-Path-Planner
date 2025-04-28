
# Auto Path Planner

<br/>

<p align="Center">
  <img src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/a0f1ebb0611a2ccf48f890b86eb23e76334fb5f6.png">
</p>

<br/>



https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/16b15b4e-7563-43c9-8b2e-35f60a291b7f

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/663e523b-eca2-4ad2-bb70-720181fc7315

<br/>

## What is it?

This tool can generate target trajectories by automatically planning
path around any unknown object based on its shape. It can move a
robot\'s EEF around any object based on its shape.

Franka panda robot was used for the testing along with the Intel
Realsense D435 depth camera. The program presents an interactive GUI to
the operator where he can fine-tune settings according to their specific
needs and visualize or replay the generated targets.

There are two separate versions of this program available. Simulated
version where the Franka, Rviz, Moveit and Gazebo are used in simulated
environment. Physical version is optimized to function with actual
Franka robot. Both are mostly identical with some minor platform
specific changes.

## Prerequisites

-   ROS Noetic (may work with any other version as long as required tools are there.)

-   Moveit (See installation for instructions on how to install)

-   Gazebo ROS

**Note** All codes were tested on Ubuntu 20.04.6 LTS with ROS Noetic installed.

## Installation


-   Install Moveit and test with Franka panda
    [Tutorial](https://ros-planning.github.io/moveit_tutorials/index.html)

-   Make sure you can launch Rviz and Gazebo and able to use Moveit
    planner with Panda summoned in Gazebo and Rviz environment.

-   Test with roslaunch panda_moveit_config demo_gazebo.launch

- Open new Terminal

  - ```console
    mkdir -p ~/ws_Auto_path_planner
    ```


-   Download / Clone this repository
    - ```console
      git clone https://github.com/CuriousLad1000/Auto-Path-Planner.git
      ```
-   Copy the **src** folder to **ws_Auto_path_planner** created earlier

-   Configure folder to prepare for build.

    - ```console
      cd ~/ws_Auto_path_planner
      ```
    - ```console
      rosdep install -y --from-paths . --ignore-src --rosdistro noetic
      ```
    - **Note** In case an upstream package is not (yet) available from the standard ROS repositories or if you experience any build errors in those packages, please try to fetch the latest release candidates from the ROS testing repositories instead. [Source](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#install-ros-and-catkin)

        - ```console
          sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
          ```
        - ```console
          sudo apt update
          ```
    - ```console
      cd ~/ws_Auto_path_planner
      ```

    - ```console
      catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```

    - ```console
      catkin build
      ```




<br/>

### To run Simulated version,

-   In First terminal

    - ```console
      source ~/ws_Auto_path_planner/devel/setup.bash
      ```

    - ```console
      cd ~/ws_Auto_path_planner/src/Visual_Inspection/ && conda activate && jupyter notebook
      ```

-   In Second Terminal

    - ```console
      source ~/ws_Auto_path_planner/devel/setup.bash
      ```

    - ```console
      roslaunch panda_moveit_config demo_gazebo.launch
      ```


-   Run **"Spawn_a\_model_gazebo.ipynb"** Notebook to summon various STL
    models to Gazebo.

-   Run the \"**ROS_Simulation_Camera_Open3D_GUI.ipynb\"** Notebook

<br/>

### To run the Physical version, (used with real robot...)

-   In Second Terminal, Connect Intel Realsense D435 camera to your PC and Launch its node using

    - ```console
      source ~/ws_Auto_path_planner/devel/setup.bash
      ```

    - ```console
      roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_fps:=30 color_width:=640 color_height:=480
      ```

-   In Third Terminal, Launch Panda Moveit (make sure to change ip address to ip of your Robot)

    - ```console
      source ~/ws_Auto_path_planner/devel/setup.bash
      ```

    - ```console
      roslaunch panda_moveit_config franka_control.launch robot_ip:=192.208.0.5 load_gripper:=true use_rviz:=true
      ```

-   Run the \"**ROS_Physical_Camera_Open3D_GUI.ipynb\"** Notebook


## Interface

The Script provides an interactive GUI with different features and
configurations.


<br/>

<p align="Center">
  <img src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/7cac7d211552478168b4dbd7de0297cc37bcbf11.png">
</p>

<br/>
<br/>

Choose from 4 different Options.

-   Run/Preview/Modify previously saved targets.

-   **Manual Mode**: User can define initial pose and can keep control
    over entire process of Path planning, gives option to preview,
    select correct object, generate targets with a lot more flexibility
    and control with minimal intervention from user.

-   **Automatic Mode**: User initially follows the Manual mode process
    and defines number of iterations for which the program should run.
    After defining, Program takes over and continues with defined
    settings without user's intervention. Useful in case doing path
    planning around Larger structures like tunnels large planes, pipes
    etc.. where the FOV of camera can't capture entire object's shape at
    once.

-   **Settings:** Can be changed to get even fine-tuned control over the
    process including object selection, clustering, Point cloud
    resolution, target generation, offsets, Debugging etc...

## Hardware used

-   Franka Emika Panda

-   Realsense D435

## Software used

-   ROS Noetic

-   Gazebo

-   Rviz-Moveit

-   Jupyter notebook to run script.

-   Software written in Python.

## Process explained in pics....

### Select initial pose of robot


<br/>

<p align="Center">
  <img width="700" height="406" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/9d307c8e2eac97f69df1542c9046285171bd45fa.png">
</p>

<br/>
<br/>

### Filtered Pointcloud image


<br/>

<p align="Center">
  <img width="400" height="293" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/be459ad338073b51da8496216d67c640a1250d56.png">
</p>

<br/>
<br/>

### Select correct Object and Scan motion.


<br/>

<p align="Center">
  <img width="500" height="413" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/0931ff80f5ebde008d4f82d4bf81eb4c5db80f77.png">
</p>

<br/>
<br/>

### Preview generated profile


<br/>

<p align="Center">
  <img width="400" height="400" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/37493c3d84ad73398af86403affd1fd4deb82017.png">
</p>

<br/>
<br/>

### Preview Targets


<br/>

<p align="Center">
  <img width="400" height="342" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/ed576b26fb66bc6847189acb0be677238cc62ff0.png">
</p>

<br/>
<br/>

### Robot moves through the targets.


<br/>

<p align="Center">
  <img width="1000" height="334" src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/c674317ab8cb46211229207dcfef358f140fd2b0.png">
</p>

<br/>
<br/>

## Real test images

<br/>

<p align="Center">
  <img src="https://raw.githubusercontent.com/wiki/CuriousLad1000/Auto-Path-Planner/images/c9a66f8d7c662560f23bdbd144f9c895dd5d5927.png">
</p>

<br/>

## Videos

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/d8c298fc-8b49-4cbb-b7d1-7dc01e255154

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/8c949989-bd81-43a3-9bbd-5da5aa749a19

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/adb00ead-a902-46a5-a32c-858be3ea8546

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/c7539826-1340-44d6-8d6b-4b27b45be970

https://github.com/CuriousLad1000/Auto-Path-Planner/assets/115222760/34ceb36c-4d9b-41d3-b90f-619d24a6a0fb

Please check the Demo_Photos_and_Videos directory for full videos

<br/>

## Acknowledgements

- This tool was created by implementing certain functions from Open3D library. [Open3D](http://www.open3d.org/)

```bib
@article{Zhou2018,
      author  = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
      title   = {{Open3D}: {A} Modern Library for {3D} Data Processing},
      journal = {arXiv:1801.09847},
      year    = {2018},
    }
```

- The STL models were downloaded from [Thingiverse](https://www.thingiverse.com/) under Creative Commons Licence
  - [Subaru Impreza for FDM by derben](https://www.thingiverse.com/thing:1995220)
  - [Small Submarine by nmartin](https://www.thingiverse.com/thing:11758)
  - [(armageddon film) sateillite by joshuajacobson95](https://www.thingiverse.com/thing:4583121)

- This project was supported and funded by [Tampere Univeristy](https://www.tuni.fi/en), Finland and [AI HUB Tampere](https://research.tuni.fi/aihubtampere/) , [Tampere AI](https://tampere.ai/en/events/)


## Publication and Citation

Research paper: (https://arxiv.org/abs/2312.02603)

If you like my work and would like to support me, please cite this work as:

```bib
@INPROCEEDINGS{10711320,
  author={Tasneem, Osama and Pieters, Roel},
  booktitle={2024 IEEE 20th International Conference on Automation Science and Engineering (CASE)}, 
  title={Automatic Robot Path Planning for Active Visual Inspection on Free-Form Surfaces}, 
  year={2024},
  volume={},
  number={},
  pages={173-180},
  keywords={Point cloud compression;Training;Visualization;Shape;Robot vision systems;Training data;Inspection;Cameras;Path planning;Hardware},
  doi={10.1109/CASE59546.2024.10711320}}
```

<br/>

Intersted in my work? Want to discuss something? Want to hire me?

You can connect with me on [LinkedIn](https://www.linkedin.com/in/osama-tasneem/) 
