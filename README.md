# Kinect Recorder for Skeleton Joints
program used in bachelor's thesis of Johanna Peham, June 2019
under supervision of Andreas Kimmig (KIT-IMI) and Kinemic GmbH

## required/ used during thesis work:
IDE - Visual Studio
Environment - Anaconda, download here: https://www.anaconda.com/products/individual 
Kinect for Windows SDK v2 - https://developer.microsoft.com/de-de/windows/kinect/
comtypes - https://github.com/enthought/comtypes/ - to install you can use: pip install comtypes

##Use-Case:
model car from LEGO parts as example of assembly with tracking of depth sensor and IMU, for later analysis and prediction

used Kinectv2 as depth-sensor and Kinemic Band as IMU

code uses PyKinectV2 to access Kinect and to get skeleton data
see: https://github.com/Kinect/PyKinect2 
used Anaconda as interpreter

Kinect skeleton data consists of 25 joints, here finger tips and wrist are used
Kinect works with 30 Hz
for every frame the coordinates of the skeleton and points-of-interest are drawn onto the live-feed and saved into CSV-files after the recording ended
points-of-interest (POI) mark specific points, necessary for use-case: the position of the tires of the lego car and of fields, that contain extra parts used in the assembly
the position of those POI can be set with the keys 1 and 2 for tires, and 3 and 4 for the fields and the position of the finger tips of the right hand.

after positioning of POIs the subject would go through the different motions of assembly while marking the time-slices of those actions with a key logger
in the thesis a foot pedal was used, that acts as a key from the keyboard (in this case the letter 'b')
at the beginning of a single motion the pedal=key is pressed down, and the code logs the event 'keydown' with its timestamp. after the motion ends, and the pedal=key is released, the event 'keyup' is saved and now marks the time-slice of that action

additionally the subject would press a specific letter on the keyboard to mark specific milestones of the recording, for easier analysis of the data. for example, 'a' for the beginning of recording of the motions after positioning of POIs, but due to the logging of the time-slices with the foot pedal not necessary

all data is saved into individual CSV-files for each frame with a counter and a unix timestamp

##Kinemic:
Initially this code was used with additional code from Kinemic GmbH to record the IMU data of a wristband.
During the thesis this was done with an app, a websocket, and a smartphone, and the full recording with the Kinect all together on a laptop. This allowed all comunication to go over a single network.
After the thesis, when bringing the project back into KIT, it was found that this system does not work on the KIT pool computers because of different networks for LAN and WLAN, and the missing option to switch to Wi-Fi completely like before.
If wanted, it is important to check for a potential offset of timestamps, depending on the device of recording and the time it takes to process the data before logging.
It was found easiest, to find a possible offset by producing extremes in data, by clapping of bumping the tabletop.

