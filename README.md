# Created by Vinay on 10.00 PM, 4 June 2017
This repository contains code for making slit-scan panorama using video or camera on pc. This approach can be used to make real-time panorama on successive images or videos.  

# Dependencies
OpenCV 3.x or higher (not tested on OpenCV 2.x yet)

# Installation
Go to root directory. And run following commands:
$ chmod +x compile.sh
$ ./compile.sh

This will compile the libraries and save the executable naming 'panorama` in 'bin' folder.

# Usage
This executable can be used to make panorama images from videos or live camera. 
Use the following command to make panorama image from a video:
$ ./bin/panaroma input_video_path output_image_name

Use the following command to make panorama image from live camera stream:
$ ./bin/panaroma camera_index output_image_name
(Note: camera_index can be 0/1/2 only)

# Example
For video:
$ ./bin/panaroma data/pano_raw.mp4 pano.jpg

For camera:
$ ./bin/panaroma 0 pano.jpg

The output images will be saved in 'output' folder
# Hope you do good with slit-scan panorama. Report any issue on the github page.
