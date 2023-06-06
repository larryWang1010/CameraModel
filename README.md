# CameraModel

Here is two camera models commonly used in **SLAM/VO**, it consists of MEI and Pinhole. It can rectify image or points, lift from pixel to unit plane and project from world to pixel. It gives both implementation based on **OpenCV** and **no-OpenCV**, so that we can explore all of the algorithm.

More details see [blog](https://blog.csdn.net/OKasy/article/details/90665534)

![Before](https://github.com/alalagong/CameraModel/blob/master/data/1.png) ![After](https://github.com/alalagong/CameraModel/blob/master/data/1_undist_pinhole.png)

## 1. Dependencies

It's tested in **Ubuntu 16.04**

### 1.1 OpenCV

​	[OpenCV 3.2.0](https://docs.opencv.org/3.2.0/d2/d75/namespacecv.html) is used in the code.

### 1.2 Eigen

​	we use Eigen 3.2.29.

### 1.3 GLOG

​	This is used to record log in `log` folder.

> sudo apt-get install libgoogle-glog-dev

## 2. Build

​	After clone, run:

> mkdir build  
> cd build  
> cmake ..  
> make 


## 3. Usage
The calibration files are stored in `calib` folder, and the format of *yaml* can refer to the examples.
The images are stored in `data` folder.

### 3.1 Run test
​	We can test the code using commands:

> ./bin/test ./calib/cam0_tumvio_mei.yaml ./data/1.png

### 3.2 Output

​	After finish running, corrected image will be put to `data` folder, and name format is:

> original name + \_ undist \_  + model(pinhole or mei) + original extension

​	Examples: 

> hello.jpg   ==> hello_undist_mei.jpg



## REFERENCE

1. kb4模型
```
J. Kannala and S. Brandt (2006). A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses, IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 28, no. 8, pp. 1335-1340
```
2. mei模型
```
Mei, Christopher, and Patrick Rives. "Single view point omnidirectional camera calibration from planar grids." Proceedings 2007 IEEE International Conference on Robotics and Automation. IEEE, 2007.
```
3. ds模型
```
Usenko, Vladyslav, Nikolaus Demmel, and Daniel Cremers. "The double sphere camera model." 2018 International Conference on 3D Vision (3DV). IEEE, 2018.
```
4. eucm模型
```
Khomutenko, Bogdan, Gaëtan Garcia, and Philippe Martinet. "An enhanced unified camera model." IEEE Robotics and Automation Letters 1.1 (2015): 137-144.
```

5. omni模型
Scaramuzza, Davide, and Katsushi Ikeuchi. “Omnidirectional camera.” (2014): 552-560.



Reference

https://blog.51cto.com/u_13157605/6038101

https://github.com/ethz-asl/kalibr/wiki/supported-models

OpenCV中相机标定相关的文档

https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html
https://docs.opencv.org/4.5.4/d4/d94/tutorial_camera_calibration.html

