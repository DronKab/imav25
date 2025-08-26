# imav24

Indoor ROS2 nodes for IMAV25.

## Pre-requisites

 - ROS2 Humble installed. Steps can be found *[here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)*.
 - After install ROS2 Humble, source it with
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
 ```

 - Last version of setuptools working without warning on Ros2 Humble : 

```sh
sudo apt install python3-pip
pip install setuptools==58.2.0
```

 - MicroXRCE-DDS Client installed, to do so you can run : 
 
 ```sh
 git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
 ```
 
 Other installations steps and more details are described *[here](https://docs.px4.io/main/en/middleware/uxrce_dds.html)*.

  - Aruco-OpenCV ROS2 Package :

  ```sh
  sudo apt install ros-humble-aruco-opencv
  ```
  Package *[page](https://index.ros.org/p/aruco_opencv/)* on ROS Index is provided as reference.
  - Aruco-OpenCV ROS2 Package :
  ```sh
  sudo apt install ros-humble-joy
  ```

  - Video for Linux 2 package (for transmiting camera image on raspberrypi)
  ```sh
  sudo apt-get install ros-humble-v4l2-camera
  ```
Package based on this *[GitHub page](https://gitlab.com/boldhearts/ros2_v4l2_camera)

## Installation

 1. Create a new workspace folder.

 ```sh
 cd
 mkdir ~/imav25_ws
 mkdir ~/imav25_ws/src
 ```

2. Clone px4_msgs repo

```sh
cd ~/imav25_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```
**_NOTE_** : The px4_msgs version should match the major version firmware on your PX4 SITL or your PX4 Hardware.
### Cloning this repository via SSH

To securely clone this repository via SSH:

#### a. Check or create an SSH key

```sh
ls ~/.ssh/id_ed25519.pub
```

If it does not exist, generate one:

```sh
ssh-keygen -t ed25519 -C "your_email@example.com"
```

Press Enter to confirm defaults.

#### b. Add your key to GitHub

- Copy the public key:

```sh
cat ~/.ssh/id_ed25519.pub
```

- Go to [https://github.com/settings/keys](https://github.com/settings/keys)
- Click **"New SSH key"**, paste the key and save

#### c. Clone the repository

```sh
cd imav25_ws/src/
git clone git@github.com:DronKab/imav25.git
```

3. Build the workspace, this might take some minutes the first time.
- First build the px4 messages package (this might take some minutes the first time)
```sh
cd ~/imav25_ws
colcon build --packages-select px4_msgs
```
- Then build the imav25 package with symlink
```sh
cd ~/imav25_ws
colcon build --packages-select imav25 --symlink-install 
```

**_NOTE_** : Don't forget to source your ros2 installation if you haven't. 

You can add the new package to your .bashrc after the build if it didn't fail. This will auto-source the package on every new terminal you open.

```sh
echo "source ~/imav25_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc
```

 ## How to run

1. Start your SITL/Gazebo, check *[this repo](https://github.com/DronKab/imav25_sim.git)* for that. Then, run the launch file with :

```sh
ros2 launch imav25 simulador.launch.py 
```

This will run px4_driver node, Aruco detection node, and UXRCE agent. Let it run in the background.
