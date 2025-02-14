# TIS USB3 CAMERA SETUP FOR ROS

## DESCRIPTION
This document covers the installation and operation of the **Tis 37AUX462** model camera with **Ubuntu 20.04** and **ROS Noetic**. The tests were conducted on an **AMD64** architecture.

---

## STEPS
1. [Camera Files Installation](#1-camera-files-installation)
2. [Required Package Installation](#2-required-package-installation)
3. [Running the Package](#3-running-the-package)

---

## 1. Camera Files Installation
The Imaging Source SDK installation will be carried out using the following steps:

```bash
git clone https://github.com/TheImagingSource/tiscamera.git
cd tiscamera

# Only works on Debian-based systems like Ubuntu
sudo ./scripts/dependency-manager install

mkdir build
cd build

# Disable Aravis as it is not used
cmake -DTCAM_BUILD_ARAVIS=OFF ..

make
sudo make install
```

---

## 2. Required Package Installation
Run the following commands in the terminal to install the necessary packages:

```bash
git clone https://github.com/alperkoc13/tis_cam_driver

# Install dutils package for additional TIS camera features
tar -xvf tis_cam_driver.tar.gz
cd tis_cam_driver/src/Package
sudo dpkg -i tcamdutils_1.0.0.560_amd64.deb

# Compile the ROS workspace
cd ../..
catkin_make
```

---

## 3. Running the Package
Ensure that ROS environment variables are loaded before running the package:

```bash
source devel/setup.bash
rosrun camera camera_node
```

---

By following these steps, you can integrate and use the **Tis 37AUX462** camera with ROS. If you encounter any errors or issues, check the error messages and make the necessary corrections accordingly.

# tis_cam_driver
