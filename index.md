---
layout: custom
title: Home
---

# Formula Racing Simulation for Ben Gurion University

[About](about.md)

## Introduction
Welcome to the Formula Racing Simulation project, designed for simulating the FSAE challenge within Unreal Engine using Airsim. 
This project was created in 2021 and is optimized for that year's challenge specifications.
It runs on Windows 10 with Python 3.9, Airsim 1.5.0, and Unreal Engine 4.26.2. 
Please note that newer versions of Airsim may produce an error in the simGetImages function, which may be resolvable.

For detailed information, refer to the project PDF file included in the repository.

## Getting Started
### Prerequisites
To set up and use this project, you will need the following:
Unreal Engine 4.26.2 (Install using official instructions)
Airsim 1.5.0 (Follow official instructions)

### Installation Steps
1. Unreal Engine Installation:
# Formula Racing Simulation for Ben Gurion University
## Introduction
Welcome to the Formula Racing Simulation project, designed for simulating the FSAE challenge within Unreal Engine using Airsim. 
This project was created in 2021 and is optimized for that year's challenge specifications.
It runs on Windows 10 with Python 3.9, Airsim 1.5.0, and Unreal Engine 4.26.2. 
Please note that newer versions of Airsim may produce an error in the simGetImages function, which may be resolvable.

For detailed information, refer to the project PDF file included in the repository.

## Getting Started
### Prerequisites
To set up and use this project, you will need the following:
Unreal Engine 4.26.2 (Install using official instructions)
Airsim 1.5.0 (Follow official instructions)

### Installation Steps
1. Unreal Engine Installation:
  * Install Unreal Engine 4.26.2.

2. Download Project Files:
  * Download the Unreal Project from the following [link](https://drive.google.com/drive/folders/1_BdXtkc-P8FzvNqy38et9genC3dfL_1K?usp=sharing "project files location").

3. Install Airsim:
  * Ensure Airsim binaries are available under the project plugins.

4. Python Environment:
  * Create a Python Virtual Environment (Venv / Anaconda recommended).
  * Activate the virtual environment and run:
    pip install wheel
    pip install -r requirements.txt

5. IDE Setup:
  * Create a PyCharm project in the repo folder.
  * Mark the utils subfolder as Sources Root or add it to Python path using 'sys'.

6. Configuration:
  * Copy settings.json into the Documents/Airsim folder. Backup and overwrite if necessary.

7. Launch Unreal Project:
     * Open the StudentRacing project in Unreal Engine.
     * Ensure the Airsim plugin is enabled.
     * Click "Play" and wait for all shaders to compile.

8. Run Main Program:
  * Play the project in Unreal Engine.
  * Run the main_program.py script in PyCharm/Python.

## Features
The main program comprises three high-level missions:

1. Cone Detection:
  * Discovering cones using LIDAR.
  * Classifying cones using cameras.
  * Saving cone locations.

2. Path Generation:
  * Creating a path spline from detected cone positions.

3. Control Loop:
  * Closing a control loop over the generated spline.

Additional features include:
  * Data logging for exporting statistics.
  * Customizable UE cone courses.
  * Independent testing and tweaking of sub-missions with the right parameters.

## Documentation
  * For a deep dive into the project, review the thesis PDF.

## Contributing
We welcome contributions! Please follow these steps:
  1. Fork the repository.
  2. Create a new branch (git checkout -b feature-branch).
  3. Commit your changes (git commit -m 'Add some feature').
  4. Push to the branch (git push origin feature-branch).
  5. Open a Pull Request.

## Contact
  For questions or support, please contact [marikf98@gmail.com].

