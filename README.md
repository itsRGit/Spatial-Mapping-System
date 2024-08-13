# 🗺️ Spatial Mapping System

An advanced spatial mapping system developed using a Simple Link MSP-EXP432E401Y microcontroller, a Time-of-Flight (ToF) distance sensor, and a stepper motor. This system is designed to capture and visualize 3D environments in real-time, making it ideal for applications in robotics, AR/VR, and autonomous navigation.

## 🧭 Overview

The Spatial Mapping System utilizes a ToF sensor mounted on a stepper motor to perform 360° scans of the environment. The system captures distance measurements at precise intervals, which are then processed and visualized on a connected PC using Open3D software. The data is transmitted from the microcontroller to the PC via UART communication, where it is rendered into a 3D point cloud.

## ✨ Features

- **Real-Time 3D Mapping:** Captures spatial data in real-time, enabling immediate 3D visualization.
- **Rotational Scanning:** Uses a stepper motor to rotate the ToF sensor, capturing distance measurements at 45° intervals.
- **High Precision:** Employs advanced algorithms to calculate x, y, and z coordinates from ToF sensor data.
- **Interactive Visualization:** Renders 3D point clouds using Open3D, allowing for detailed inspection of the scanned environment.
- **User-Controlled Sampling:** Provides user control over data collection with an onboard button and stop button for manual operation.

## 🛠️ Technologies Used

- **Simple Link MSP-EXP432E401Y:** 32-bit ARM Cortex-M4F based microcontroller.
- **VL53LIX Time-of-Flight Sensor:** Utilized for precise distance measurements.
- **MOT-28BYJ48 Stepper Motor:** Enables rotational scanning of the environment.
- **Open3D:** Used for rendering 3D visualizations of the collected data.
- **Python & UART:** Facilitates communication between the microcontroller and the PC for data transfer and processing.

## 🚀 Getting Started

To set up and run the Spatial Mapping System, follow these steps:

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/itsRGit/Spatial-Mapping-System.git
    cd Spatial-Mapping-System
    ```

2. **Install Dependencies:**
   Ensure you have installed the necessary software, including Open3D, Python 3.9, and the required Python libraries (serial, numpy, open3d, math).

3. **Build the System:**
    - Compile and load the Keil code onto the MSP-EXP432E401Y microcontroller.
    - Set up the circuit as described in the documentation and connect the ToF sensor to the stepper motor.

4. **Run the System:**
    - Connect the MCU to your PC via USB.
    - Press the onboard PJ1 button to start the stepper motor and begin collecting data.
    - Run the `Python Code.py` script to process and visualize the data in real-time.

## 📚 Documentation

For a detailed explanation of the system's architecture, algorithms, and setup, refer to the `docs/` directory. This includes the circuit schematic, programming flowchart, and step-by-step instructions for device setup and operation.

## 🌍 Applications

This Spatial Mapping System is suitable for various applications, such as:
- **Robotics:** Facilitates autonomous navigation and obstacle detection.
- **AR/VR:** Enhances virtual experiences by integrating real-world spatial data.
- **Construction:** Useful for site surveying and creating accurate 3D models.
- **Healthcare:** Assists in surgeries and rehabilitation with precise spatial awareness.
