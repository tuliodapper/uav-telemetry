# uav-telemetry
The purpose of this work is to provide a cost-effective method to gather important information of a flight for project validation. Because some of the data need to be gathered at a particular condition, it is desirable that the flight is constantly monitored in order to help the pilot to achieve a specified condition, e.g., altitude. To sum up, this work aims to build a platform responsible for collecting airspeed, orientation and altitude of a fixed-wing UAV flight, and current drained by the radio receiver and the servo actuators. In addition, the low-cost instrumentation system should be capable of acquiring, storing and transmitting these relevant flight parameters to a ground control station.

## Folders

### doc
Contains report, articles, presentation and video.

### software
Contains MATLAB code for the control panel interface, which aims to monitor the data collected from the acquisition board and transmitted to the ground station. Use MATLAB R2015a or later versions.

### firmware
Contains Arduino code and libraries for the firmware of the acquisition board.

### hardware
Contains the board and schematic EAGLE files for the hardware of the acquisition board.
