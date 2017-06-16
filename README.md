eeeee## GOOD: a Global Orthographic Object Descriptor for 3D object recognition and manipulation
* **Hamidreza Kasaei**, Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira
* http://wiki.ieeta.pt/wiki/index.php/Hamidreza_Kasaei

The Global Orthographic Object Descriptor (GOOD) has been designed to be robust, descriptive and efficient to compute and use. GOOD descriptor has two outstanding characteristics: 

1. Providing a good trade-off among :
  1. descriptiveness,
  2. robustness,
  3. computation tim
  4. memory usage.
2. Allowing concurrent object recognition and pose estimation for manipulation.

The performance of the proposed object descriptor is compared with the main state-of-the-art descriptors. Experimental results show that the overall classification performance obtained with GOOD is comparable to the best performances obtained with the state-of-the-art descriptors. Concerning memory and computation time, GOOD clearly outperforms the other descriptors. Therefore, GOOD is especially suited for real-time applications.
The current implementation of GOOD descriptor supports several functionalities for 3D object recognition and Object Manipulation.




In the current distribution you can find 
  	
	- an implementation of the GOOD descriptor (good.h and good.cpp)
  	- a test that demonstrates (test_GOOD_descriptor.cpp) how to use it on a test object


** CITING

This is an implementation of the GOOD descriptor which has been presented in the following papers:

	[1] Kasaei, S. Hamidreza,  Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira 
	"GOOD: A global orthographic object descriptor for 3D object recognition and manipulation." 
	Pattern Recognition Letters 83 (2016): 312-320.
	http://dx.doi.org/10.1016/j.patrec.2016.07.006

	[2] Kasaei, S. Hamidreza, Luís Seabra Lopes, Ana Maria Tomé, Miguel Oliveira 
	"An orthographic descriptor for 3D object learning and recognition." 
	2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, 
	pp. 4158-4163. doi: 10.1109/IROS.2016.7759612

Please adequately refer to this work any time this code is being used by citing above papers.
If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the above papers in your publications.



**DEPENDENCIES and BUILD

The required dependencies are Point Cloud Library (www.pointclouds.org), Boost (www.boost.org) and Eigen (eigen.tuxfamily.org).

PCL (http://www.pointclouds.org/downloads/)
```bash

	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all

Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
	
	Download Eigen
	cd build
	cmake ..
	sudo make install

We provide a CMakeLists.txt, therefore, the code can be build over different compilers and platforms by making use of CMake (www.cmake.org).

	mkdir build
	cd build
	cmake ..
	make


**How TO RUN THE SAMPLE CODE 

First of all you will need a point cloud in PCD or PLY format. 
For your convenience, we have provided a set of sample point clouds in both PLY and PCD format in a folder namely "point_cloud". 

The syntax for runing the sample code is: 

	"test_GOOD_descriptor <path/file_name.pcd> [--nogui]" or "test_GOOD_descriptor <path/file_name.ply> [--nogui]"

Use [--nogui] option to disable visualization

***Example:

	GOOD_descriptor/build$ ./test_GOOD_descriptor ../point_cloud/vase.pcd 
	GOOD_descriptor/build$ ./test_GOOD_descriptor ../point_cloud/vase.pcd --nogui



 **REAl-TIME DEMONSTRATION 

To show all the functionalities and properties of the GOOD descriptor, a real demonstration was performed. 
A video of this demonstration is available in: https://youtu.be/iEq9TAaY9u8



**CONTACT INFORMATION 

Please use the following email address, if you have questions or want to contribute to this project:

- Hamidreza Kasaei <seyed.Hamidreza@ua.pt> 
- Hamidreza Kasaei <kasaei.hamidreza@gmail.com> 

**LICENSE 

The GOOD descriptor code is released under the BSD License. A version of this code under a different licensing agreement, intended for commercial use, is also available. 
Please contact me if interested.
