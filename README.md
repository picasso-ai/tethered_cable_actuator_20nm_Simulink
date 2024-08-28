# 20Nm Tethered Cable Actuator 
# Prerequisite
Hardware Parts
Electronics Parts

**Simulink Setup**
1. Install Simulink Real-Time Target Support Package
    * In MATLAB, **Home** > **Add-Ons** > **Get Add-Ons**
    * Search for Simulink Real-Time Target Support Package
    * Download the package
2. Install with the Speedgoat package given. (Requires Simulink Real-Time)
    * Go to the folder the package in MATLAB
    * Run speedgoat_setup file
    * Restart MATLAB
3. Power on the Speedgoat and connect to the PC with an Ethernet cable.
4. Go to the Windows Network and Sharing Center
5. Right-click on the Ethernet controller and change the Internet Protocol Version 4 Properties to the following
    * IP address: 192.168.7.2
    * Subnet mask: 255.255.255.0
6. Open MATLAB and type in slrtExplorer in the MATLAB Command Window
7. In the Target Configuration tab, set the IP address to 192.168.7.5 then click Update Software
8. Connect to Speedgoat by selecting TargetPC1 and click on the connect button under.
   
**Note:** After configuring Simulink ports and address, you can connect to Speedgoat by just clicking on the connect button

**Teensy Setup**
1. Download Arduino IDE
2. Open Arduino IDE
     * Click on **File** > **Preferences**
     * Go to "Additional boards manager URLs"
     * Add in this link: https://www.pjrc.com/teensy/package_teensy_index.json
     * Close **Preference**
     * Click on **Tools** > **Boards** > **Board Manager**
     * Search and install Teensy
3. Download (based on your OS) Teensy Loader from this link: https://www.pjrc.com/teensy/loader.html **Note** These steps should only need to be done when setting up a new computer
4. Select Teensy from the board
5. Select the port Teensy is connected to
6. Upload the .ino code
7. Open up serial monitor by clicking on the magnifying glass in the upper right or under **Tools** > ""Serial Monitor**
8. Calibrate loadcell
    * Gather a bar of known length and weights of known mass
    * Lock the actuator by using a long screw at tapped holes at the outer rims to stop the rotor from turning
    * Attach the bar to the actuator
    * Hang the weights
    * Check if the reading on the serial monitor is similar to the torque reading expected
9. Alter the gain value accordingly to match the correct torque reading displayed in the serial monitor
# Operation
