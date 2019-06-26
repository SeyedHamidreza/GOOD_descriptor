# GOOD: A Global Orthographic Object Descriptor for 3D Object Recognition and Manipulation
###### [Hamidreza Kasaei](http://wiki.ieeta.pt/wiki/index.php/Hamidreza_Kasaei), Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira
##
The **G**lobal **O**rthographic **O**bject **D**escriptor (**GOOD**) has been designed to be robust, descriptive and efficient to compute and use. GOOD descriptor has two outstanding characteristics: 

1. **Providing a good trade-off among :**
	- descriptiveness,
  	- robustness,
  	- computation time,
  	- memory usage.
2. **Allowing concurrent object recognition and pose estimation for manipulation.**

The performance of the proposed object descriptor is compared with the main state-of-the-art descriptors. Experimental results show that the overall classification performance obtained with GOOD is comparable to the best performances obtained with the state-of-the-art descriptors. Concerning memory and computation time, GOOD clearly outperforms the other descriptors. Therefore, GOOD is especially suited for real-time applications.
The current implementation of GOOD descriptor supports several functionalities for 3D object recognition and Object Manipulation.


## CITING
This is an implementation of the GOOD descriptor, which has been presented in the following papers.
Please adequately refer to the papers any time this code is being used. 
If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the following papers in your publications.

	@article{GOODPRL,
		title = "GOOD: A global orthographic object descriptor for 3D object recognition and manipulation",
		journal = "Pattern Recognition Letters",
		volume = "83",
		pages = "312 - 320",
		year = "2016",
		note = "Efficient Shape Representation, Matching, Ranking, and its Applications",
		issn = "0167-8655",
		doi = "http://dx.doi.org/10.1016/j.patrec.2016.07.006",
		url = "http://www.sciencedirect.com/science/article/pii/S0167865516301684",
		author = "S. Hamidreza Kasaei and Ana Maria Tomé and Luís Seabra Lopes and Miguel Oliveira",}

	@INPROCEEDINGS{GOODIROS, 
		author={S. H. Kasaei and L. Seabra Lopes and A. M. Tomé and M. Oliveira}, 
		booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
		title={An orthographic descriptor for 3D object learning and recognition}, 
		year={2016}, 
		pages={4158-4163}, 
		doi={10.1109/IROS.2016.7759612}, 
		url="http://ieeexplore.ieee.org/document/7759612/"
		month={Oct},}
	


## LICENSE 
The GOOD descriptor code is released under the BSD License. A version of this code under a different licensing agreement, intended for commercial use, is also available. Please contact if interested.


## DEPENDENCIES
The required dependencies are [Point Cloud Library](www.pointclouds.org), [Boost](www.boost.org) and [Eigen](eigen.tuxfamily.org):

1. **[PCL](http://www.pointclouds.org/downloads/)** (http://www.pointclouds.org/downloads/)
	```bash
	sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
	sudo apt-get update
	sudo apt-get install libpcl-all
	```
2. **[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)** (http://eigen.tuxfamily.org/index.php?title=Main_Page)
	```bash
	hg clone https://bitbucket.org/eigen/eigen/
	cd build
	cmake ..
	sudo make install
	```

## BUILD
We provide a CMakeLists.txt, therefore, the code can be build over different compilers and platforms by making use of [CMake](www.cmake.org).
```bash
mkdir build
cd build
cmake ..
make
```

## CODE 

In the current distribution you can find:  
- an implementation of the GOOD descriptor (good.h and good.cpp)
- a sample test code (test_GOOD_descriptor.cpp) that demonstrates how to use it on a test object

### How to run the sample code
First of all you will need a point cloud in PCD or PLY format. For your convenience, we have provided a set of sample point clouds in both PLY and PCD format in a folder named "point_cloud". 

The syntax for runing the sample code is: 
```bash
./test_GOOD_descriptor <path/file_name.pcd> [--nogui]	
./test_GOOD_descriptor <path/file_name.ply> [--nogui]
```

- Use [--nogui] option to disable visualization.

### Example:

	./test_GOOD_descriptor ../point_cloud/vase.pcd 
	./test_GOOD_descriptor ../point_cloud/vase.pcd --nogui



## REAl-TIME DEMONSTRATION 

To show all the functionalities and properties of the GOOD descriptor, a demonstration in real time was performed.

:movie_camera: A video of this demonstration is available in: https://youtu.be/iEq9TAaY9u8

## CONTACT INFORMATION 

1. Please use the following email addresses if you have questions or want to contribute to this project:
	- :email: <hamidreza.kasaei@rug.nl> 
	- :email: <kasaei.hamidreza@gmail.com> 
2. Follow us on our [website](www.ai.rug.nl/hkasaei) for updates and to get information about other works:
	- :satellite: www.ai.rug.nl/hkasaei
