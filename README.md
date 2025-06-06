# TactileAloha_Hardware
#### Main Project Website: https://guningquan.github.io/TactileAloha

This repository is one of the repositories in the TactileAloha project.  
We organize the code of the TactileAloha project as follows:  

- **This repository** contains the hardware-related code, which is used for launching ROS hardware and should be placed in the ROS workspace.  
- **Another repository,** [TactileAloha_ML](https://github.com/guningquan/act-triple-plus), focuses on robot teleoperation, dataset collection, and imitation learning algorithms. It can be placed anywhere on your computer.

<!-- This repository contains the hardware-related code for the TactileAloha project.  
The TactileAloha project includes two parts: this repository, which focuses on hardware, and another repository related to robot teleoperation, dataset collection, and imitation learning algorithm [TactileAloha_ML](https://github.com/guningquan/act-triple-plus). -->

## 📂 Repo Structure
- **`aloha/`** - Contains core components related to the TactileAloha module, including the TactileAloha launch and the 3D printed model.
- **`digit_interface/`** - An interface for GelSight tactile sensing that can launch and publish GelSight tactile information.
- **`tactile_aloha_monitor/`** - Monitoring tools for TactileAloha.
- **`README.md`** - Project documentation and setup instructions.  

---

## 🏗️ Quick Start Guide

### 🖥️ Software Selection – OS Compatibility

This project has been tested and confirmed to work with the following configuration:  

- ✅ **Ubuntu 20.04 + ROS 1 Noetic** (Fully tested and verified)  

Other configurations may work as well, but they have not been tested yet. If you successfully run this project on a different setup, feel free to contribute by sharing your experience! 🚀

### 🛠️ Software installation - ROS:
1. Install ROS and Interbotix Software: You can install ROS and the Interbotix software by following the official documentation: [Interbotix X-Series Arms Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/). 
Alternatively, you can run the following commands to automate the installation:
    ```sh
    sudo apt install curl
    curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' -o xsarm_amd64_install.sh
    chmod +x xsarm_amd64_install.sh
    ./xsarm_amd64_install.sh -d noetic
    ```
2. This will create the directory ``~/interbotix_ws`` which contains ``src``.
3. Clone our repository inside `~/interbotix_ws/src`， and go to ``~/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py``, find function ``publish_positions``. Change ``self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)`` to ``self.T_sb = None``.
4. ``source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh``.
5. ``sudo apt-get install ros-noetic-usb-cam && sudo apt-get install ros-noetic-cv-bridge``.
6. run ``catkin_make`` inside ``~/interbotix_ws``, make sure the build is successful.

### 🧱 Material Preparation
1. Ensure you have the original [ALOHA](https://github.com/tonyzhaozh/aloha) system, which includes at least 4 robots and 3 cameras.
2. Prepare the additional materials required for TactileAloha. The bill of materials, detailing components costing approximately $450 USD beyond the original ALOHA setup, is available [here](https://docs.google.com/spreadsheets/d/1tX9UN2hUqN1ftezxETeFLI59bT0_g0PbhREX1-UMxE8/edit?usp=sharing).
3. 3D print the gripper designed for the tactile sensor. The STL file can be found at aloha/3d_models/gelfinger_v2.stl within this repository.


### 🔧 Hardware Installation
The goal of this section is to run `roslaunch aloha tactile_aloha.launch`, which starts  
communication with **4 robots**, **3 cameras**, and a **Gelsight tactile sensor**.  
Follow these steps to ensure it works properly:

1. We assume you already have the original ALOHA system.

2. `❗` Camera Focus Configuration (Not described in ALOHA):

   The cameras in the ALOHA series are set to **fixed focus** in ROS launch. 
The focus value is configured through `aloha.launch` in `aloha/launch`:  
    ```xml
    <param name="focus" value="40"/>
    ```
   It is necessary to determine the appropriate focus value for each camera; otherwise, the camera image may appear blurry during manipulation.

    The recommended procedure to find a suitable focus value is:
    - To check the available video devices, run the following command: 
   ``` bash
     ls /dev/CAM_*  # or ls /dev/video*
   ```   
    If you have set your camera serial numbers according to [ALOHA](https://github.com/tonyzhaozh/aloha), you can see your camera list as follows:
   ``` bash
     /dev/CAM_HIGH  /dev/CAM_LEFT_WRIST   /dev/CAM_RIGHT_WRIST
   ```  
   
    - Use the following command to open the camera and adjust the focus: 
   ``` bash
     guvcview -d /dev/CAM_HIGH
   ```
    - Test and note the appropriate focus value for each camera ;

3. `❗` Disable Auto Focus:

   - You must disable the continuous autofocus by setting the focus_automatic_continuous control parameter as follows:
    ```bash
      v4l2-ctl -d /dev/CAM_HIGH --set-ctrl focus_automatic_continuous=0
    ```
    - For other cameras please modifiy `/dev/CAM_HIGH` to `/dev/CAM_LEFT_WRIST`, etc.
    - The way to check whether we have set the camera correctly is to run `roslaunch aloha aloha.launch` and ensure that no warning like this appears:
    ```bash
    Error setting controls: Permission denied
    VIDIOC_S_EXT_CTRLS: failed: Permission denied
    ```
   - You can add the following to your `.bashrc`, which allows you to conveniently run `cameras-autofocus` from the command line.
   ```bash
   # Define the cameras-autofocus function to disable autofocus
   cameras-autofocus() {
       # List of camera devices to configure
       cameras=("/dev/CAM_HIGH" "/dev/CAM_LEFT_WRIST" "/dev/CAM_RIGHT_WRIST")
   
       # Function to disable autofocus for a single camera
       disable_autofocus() {
           local camera=$1
           echo "Disabling autofocus for $camera..."
           
           # Disable automatic continuous focus
           v4l2-ctl -d "$camera" --set-ctrl focus_automatic_continuous=0
   
           echo "Autofocus disabled for $camera."
       }
   
       # Main loop to process all cameras
       echo "Configuring autofocus for cameras..."
       for cam in "${cameras[@]}"; do
           if [ -e "$cam" ]; then
               disable_autofocus "$cam"
           else
               echo "Camera $cam not found. Skipping."
           fi
       done
       echo "Autofocus configuration complete."
   }
    ```
   Note: You will need to reapply the focus_automatic_continuous=0 setting whenever you reboot the computer or unplug and replug the cameras.


4. Install the GelSight tactile sensor with the 3D-printed model in the `/aloha/3d_models` directory. We recommend using the [right-angle micro USB cable](https://www.amazon.co.jp/dp/B00ENZDFQ4?ref=ppx_yo2ov_dt_b_fed_asin_title) to connect the sensor with computer, which could reduce collision risk during manipulation. 
Moreover, we install the [noise absorbers](https://www.amazon.co.jp/-/en/gp/product/B0CP491LJR/ref=ewc_pr_img_2?smid=A19BPWQK30Q73P&th=1) for the tactile sensor cable.

5. ⚠️ After installing the tactile sensor, you need to update the `sensor_id` in `tactile_aloha.launch` with a series string like `D20982`.  
In our project, we only utilize one GelSight sensor. If you would like to use multiple GelSight sensors, you can set the `sensor_id` like `D20982_D20983`.  
Meanwhile, you need to use the `setup_digit_multi` function in `digit_interface/scripts/digit.py` along with other corresponding modifications.

6. Install the conda environment for TactileAloha:
    ```sh
    conda create -n aloha python=3.8.10
    conda activate aloha
    pip install -r requirements.txt
    ```
7. You may need to set your aloha conda environment path for `digit.py` in `/digit_interface/scripts/`. For example:
    ```python
    #!/home/ubuntu20/miniforge3/envs/aloha/bin/python
    ```

### ▶️ Testing teleoperation
Before running the commands below, ensure that:  
- **All four robots** are placed in their **sleep positions**.  
- The **master robot's gripper** is **open**.  
    ``` ROS
    # ROS terminal
    conda deactivate
    source /opt/ros/noetic/setup.sh && source ~/interbotix_ws/devel/setup.sh
    roslaunch aloha tactile_aloha.launch
    ```
You can see the three cameras and a real-time GelSight observation on your screen.  
For the teleoperation test, you need to clone the [TactileAloha_ML](https://github.com/guningquan/act-triple-plus) repository and run the command.
The [TactileAloha_ML](https://github.com/guningquan/act-triple-plus) repository can be placed anywhere on your computer; there is no need to place it inside the `interbotix_ws` directory as done in the previous ALOHA series.
   
    # Right hand terminal
    conda activate aloha
    source ~/interbotix_ws/devel/setup.sh
    cd act-triple-plus/aloha_scripts
    python one_side_teleop.py right
    
    # Left hand terminal
    conda activate aloha
    source ~/interbotix_ws/devel/setup.sh
    cd act-triple-plus/aloha_scripts
    python one_side_teleop.py left
    

The teleoperation will start when the master side gripper is closed.

For more details about teleoperation, dataset collection, data visualization, algorithm training, and development, please refer to the [TactileAloha_ML](https://github.com/guningquan/act-triple-plus) repository.

## 🙏 Acknowledgements
   This project codebase is built based on [ALOHA](https://github.com/tonyzhaozh/aloha) and [ACT](https://github.com/tonyzhaozh/act).