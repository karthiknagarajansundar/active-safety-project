# Active safety project

## Objective
The focus of this project is on a specific type of safety-critical situation called a rear-end collision. A rear-end collision occurs when a vehicle hits another vehicle from behind. This crash type, often caused by an inattentive or distracted driver, is one of the most frequently occurring worldwide. Active safety systems that can warn the driver or autonomously brake the vehicle have been developed to avoid or reduce injuries and fatalities from these crashes. 

## Tasks
1) Analyze the kinematics of a rear-end situation.
2) Process and use experimental data to understand drivers’ braking behaviour in a critical rear-end situation.
3) Propose a Forward Collision Warning (FCW) system and an Autonomous Emergency Braking (AEB) system.

## Results

### A hypothetical rear-end conflict plot
<img src="https://github.com/karthiknagarajansundar/active-safety-project/blob/main/Images/rear-end-conflict.jpeg" width="500" height="400">

### A test-run with the start and end of breaking maneuver
<img src="https://github.com/karthiknagarajansundar/active-safety-project/blob/main/Images/Test-run_with_BM.jpg" width="500" height="400">

### Driver behaviour analysis: Speed at Break Onset vs Time To Collision at Break Onset
<img src="https://github.com/karthiknagarajansundar/active-safety-project/blob/main/Images/speedvsTTC.jpeg" width="500" height="400">
Each colour represents each driver's behaviour towards the estimated TTC at the provided speed of the vehicle.

### Active safety system design
<img src="https://github.com/karthiknagarajansundar/active-safety-project/blob/main/Images/Visualization.jpeg" width="500" height="400">
For the hypothetical rear-end conflict situation an Active safety system is designed with Forward Collision Warning (FCW) system and an Autonomous Emergency Braking (AEB) system, along with variants like conservative model and aggressive model.

## Conclusion
Thus in order to avoid the rear-end collision, the Active safety system firstly provides a warning to the driver so that the driver takes required action in two scenarios where the warning is in advance (conservative) and near the last moment (aggressive). If the driver doesn't take any action, then the Active safety system initiates the AEB system in order to stop the collision automatically.

## Others
* Basic analysis of LiDAR data, coordinate transformation and mapping
<img src="https://github.com/karthiknagarajansundar/active-safety-project/blob/main/Data/LiDAR.gif" width="500" height="400">
* Analysis of CAN data
* Signal processing to filter data
* Naturalistic data analysis
* Field data acquisition and analysis
* Human factors of automated driving
