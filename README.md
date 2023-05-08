# FarmBeast_IMU
GUI za FarmBeast IMU


 Samo od sebe nebo zalaufalo

Za zagon (potrebno)
-Linux Ubuntu 20.04 
-ROS Noetic - Catkin

Po Pullu:

1. cd .. do CMakeList.txt 
2. dodaj datoteke ki si jih pullu
3. cd .. do catkin-ws/
4. catkin_make/
5. cd do farmBeast_simulation/src/
6. v posebnem terminalu poženi senzor phidgets Spatial

(
cd catkin_ws/
komanda: roslaunch phidgets_spatial spatial.launch 
)
pocaki cca 5s da se stabilizira

v drugem terminalu pridi do farmbeast_simulation/src/

komanda za zagon posaeznega pythonscripta u ROS-u:  rosrun farmbeast_simulation <ime_datoteke>.py

primer:  rosrun farmbeast_simulation nevem.py
