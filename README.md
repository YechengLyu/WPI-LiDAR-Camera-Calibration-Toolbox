# WPI LiDAR - Camera calibration toolbox

This toolbox is developed by Yecheng Lyu who is at WPI [Embedded Computing Lab](http://computing.wpi.edu/) to calibrate the LiDAR to camera transformation. This MATLAB based toolbox searches for corner points in the LiDAR frame sequence, request for annotation in the corresponding images and then estimates the transformation parameters using the generic algorithm. 

#### Features:
* It does need any checkerboard or a special object, the only thing needed is a piece of cardboard ( Amazon box is good enough ). 
* With a simple threshold filter typed in, the toolbox automatically detects the upper corner of the cardboard and search for the corresponding image. 
* To label the corner for each image you can simply click on the image and press Enter, that's it. 
* Calibration result is extracted in a common format. You can use the projection function in the toolbox or load it to other places.

The source code is released under an [MIT license](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/LICENSE).

<a href="http://computing.wpi.edu/index.html"><img src="http://computing.wpi.edu/public/images/chip.jpg" height="200" title="WPI Embedded Computing Lab"></a>
<a href="http://www.linkedin.com/in/yecheng-lyu/"><img src="http://computing.wpi.edu/public/images/yecheng.jpg" height="200" title="Yecheng Lyu"></a>
# How to use
#### Before using the toolbox
Before applying the toolbox you need to record a rosbag containing the LiDAR and camera frames. Though a professional checkerboard is not in need, you still need a piece of cardboard with a distinguishable upper corner. In our test, we rotated the checkerboard by 45&deg; so that one of the corners is up to the sky.

Then you can record the rosbag. It is recommended to set camera frame rate 1.5x faster than LiDAR frame rate to eliminate the time offset between the selected LiDAR frame and the corresponding camera frame. It is also recommended to record the rosbag in an open space so that the field of view (FOV) in the toolbox can be set easily.

#### Using the toolbox for calibration
1. To the toolbox please open MATLAB and run Calibration_toolbox.m.
1. Type in the rosbag path. It is recommended to move the rosbag under the toolbox folder since it searches for the latest rosbag as default input.
1. After the toolbox analyzes the rosbag, type in the topic name of LiDAR and camera frames. The toolbox is tested using Velodyne VLP-16 LiDAR and PointGrey Chameleon3 camera. For other sensors, you can modify the fun_read_pc.m and fun_read_image.m to read the rosmessages.
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_pc.png)
1. After the toolbox reads the rosmessages, it will plot one of the LiDAR frames to help set the FOV. You can type in the filter using expressions of LiDAR coordinate (x,y,z) and distance to the LiDAR center (r). Please type one filter each time. You can see the result of the current filter(s) and choose to add another filter, drop the latest filter or finish FOV setting. You can also edit fun_pc_sel.m if you find the FOV is not good in the labeling step.  
An available examples: x>0; x<2; y>z; z./x>1.6/8; r<9.
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_pc_filtered.png)
1. After setting the FOV, the toolbox scans the LiDAR frames one by one and looks for the corners. Each time it finds a corner, it opens the corresponding camera frame to label the corner position in the image. You can simply move the data cursor to the corner and press Enter in the command window. The toolbox will record it and turn to the next frame until the last LiDAR frame is reached.
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_pc_label.png)
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_img_label.png)
1. Before the toolbox starts estimation, it asks you to select if you want to calibrate pinhole camera model and fisheye camera model. You can select either of them or both and type the lower boundaries and upper boundaries. 
1. The toolbox then executes the genetic algorithm and estimate the parameters of the extrinsic and intrinsic matrix. The result will be saved in a .mat file and a txt file. You can check param_pinhole.mat and calib_LiDAR_to_cam_pinhole.txt for pinhole model estimation, and param_fisheye.mat and calib_LiDAR_to_cam_fisheye.txt for fisheye model estimation.
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_ga.png)
![](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/blob/master/doc/example_result.png)

# Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/YechengLyu/WPI-LiDAR-Camera-Calibration-Toolbox/issues).