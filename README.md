# PointCloudHandler
[***This is currently a demo project***]  
## Requirements  
 - C++11
 - flann
 - Eigen  
 
 Eigen and FLANN are header-only, to simplify the compilation on different platforms.  
## Introduction  
Photogrammetry typically generates point cloud data sets with different point densities. In addition, measurement errors lead to rare outliers that further distort the results. This complicates working with the point cloud and makes it difficult to determine certain characteristics (for example, relative size). Some of these irregularities can be resolved by performing a statistical analysis of the neighborhood of each point and cutting off those that do not meet a certain criterion.
This project presents the work of two algorithms for automatic removal of extra points:
- Remove Statistical Outliers
- Remove Radius Outliers  

I took this idea from https://pcl-tutorials.readthedocs.io/

## Remove Statistical Outliers 
This removal of sparse outliers is based on calculating the distribution of distances between points and neighbors in the input dataset. For each point, we calculate the average distance from it to all its neighbors. Assuming that the resulting distribution is Gaussian with mean and standard deviation, all points whose mean distances are outside the interval defined by the global mean distance and standard deviation can be considered outliers and cut off from the cloud-dataset.  
There are 2 input parameters:  
 - ***nbPoints*** - the number of neighbors to analyze;
 - ***stdRatio*** - the standard deviation multiplier.  
 ## Remove Radius Outliers
 It removes each point that has few neighbors in a given sphere around them.  
 So there are 2 input parameters too:  
  - ***radius*** - radius of the sphere that will be used for counting the neighbors;  
  - ***nbPoints*** - minimum count of points which this analyzing sphere should contain.  
  
 ## Outputs  
 One of my tasks at work was related to the fact that using photogrammetry we received a point cloud, but which contained noise and garbage points. And it was necessary to automate the clearing of the point cloud.  
 ### Input [girl bag point cloud]
 ![jpg](https://github.com/serjik85kg/PointCloudHandler/blob/main/examples/girl_bag.jpg)  
 ### Result [filtered]
 ![jpg](https://github.com/serjik85kg/PointCloudHandler/blob/main/examples/girl_bag_filtered.jpg)  
 __________________________________________________________________________________________________
 ## Remark
 I added a few unused functions and methods that could potentially be used in the future. In the future, I will expand this library and add a couple of new features. However, I don't have time to refactor this code right now.
 
# TO DO:  
 - Add examples from 2D
 - Add .obj pointcloud examples
 - Edit main.cpp
 - Add new features to this project (in the development)
