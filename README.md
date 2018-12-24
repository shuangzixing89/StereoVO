# StereoVO
a small Stereo VO  

![image](https://github.com/robotlee1997/StereoVO/blob/master/demo/demo.jpeg)

#### Requirements
- OpenCV
- Ceres(if you do not want to optimize the PNP result, you can remove all ceres code)  

#### How to compile?
```
mkdir build
cd build
cmake ..
make
```
#### How to run?
```
cd ${PROJECT_SOURCE_DIR}/bin
./KITTI
```
#### Before you run 
In order to run this algorithm, you need to have either your own data, or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) . In order to run this algorithm on your own data, you must modify the intrinsic calibration page path in the code.