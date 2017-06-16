/*________________________________
|                                 |
|           INCLUDES              |
|_________________________________| */

#include "good.cpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <pcl/visualization/histogram_visualizer.h>

typedef pcl::PointXYZRGBA PointT;  

void
visualizationPointCloud(boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud, std::string name_of_window)
{
  // visualization point cloud
  pcl::visualization::PCLVisualizer viewer1 (name_of_window.c_str());
  viewer1.addPointCloud (point_cloud, "original");
  viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "original");
  viewer1.addCoordinateSystem (0.12);
  viewer1.setBackgroundColor (255, 255, 255);
  while (!viewer1.wasStopped ())
  { viewer1.spinOnce (100);}	
}


int
readPointCloud(std::string object_path,  boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud)
{
  std::string extension = boost::filesystem::extension(object_path);
  if (extension == ".pcd" || extension == ".PCD")
  { 
    if (pcl::io::loadPCDFile(object_path.c_str() , *point_cloud) == -1)
    {
      std::cout << "\n Cloud reading failed." << std::endl;
      return (-1);
    }
  }
  else if (extension == ".ply" || extension == ".PLY")
  {   
    if (pcl::io::loadPLYFile(object_path , *point_cloud) == -1)
    {
      std::cout << "\n Cloud reading failed." << std::endl;
      return (-1);
    }
  }
  else 
  {
      std::cout << "\n file extension is not correct. Syntax is: test_GOOD_descriptor <path/file_name.pcd> [--nogui] or test_GOOD_descriptor <path/file_name.ply> [--nogui]" << std::endl;
      return -1;
  }
  return 1;
}

int main(int argc, char* argv[])
{  
  bool gui_flag = true;
  
  if (argc < 2 ||argc > 3) 
  {         
    std::cout << "\n Syntax is: test_GOOD_descriptor <path/file_name.pcd> [--nogui] or test_GOOD_descriptor <path/file_name.ply> [--nogui]" << std::endl;
    return 0;
  }
  if (argc ==3 && strcmp(argv[2],"--nogui") != 0 )
  {
    std::cout << "\n Syntax is: test_GOOD_descriptor <path/file_name.pcd> [--nogui] or test_GOOD_descriptor <path/file_name.ply> [--nogui]" << std::endl;
    return 0;
  }
  else if (argc ==3)
  {
    gui_flag = false;    
  }
    
  std::string object_path =  argv[1];
  
  
  pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>);
  if (readPointCloud( object_path,  object)==-1)
    return -1;
  
  if (gui_flag)
    visualizationPointCloud(object, "Original point cloud and camera reference frame");
   
  std::vector< float > object_description;
  std::vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
  boost::shared_ptr<pcl::PointCloud<PointT> > transformed_object (new pcl::PointCloud<PointT>);

  Eigen::Matrix4f transformation;
  pcl::PointCloud<PointT>::Ptr transformed_object_and_projected_views (new pcl::PointCloud<PointT>);
  pcl::PointXYZ center_of_bounding_box;
  pcl::PointXYZ bounding_box_dimensions;
  std::string order_of_projected_planes;
    
  // Setup the GOOD descriptor
  // GOOD can also be setup in a line:  GOODEstimation test_GOOD_descriptor (5, 0.0015); 
  GOODEstimation<PointT> test_GOOD_descriptor; 
  test_GOOD_descriptor.setNumberOfBins(5);
  test_GOOD_descriptor.setThreshold(0.0015);  
    
  // Provide the original point cloud
  test_GOOD_descriptor.setInputCloud(object);
  
  // Compute GOOD discriptor for the given object
  test_GOOD_descriptor.compute(object_description);
  std::cout <<"\n GOOD = ["; 
  for (size_t i =0; i< object_description.size()-1; i ++)
    std::cout << object_description.at(i)<<",";
  std::cout << object_description.back() <<"]"<<std::endl;  
  
  /*_________________________________________
  |                                          |
  | Functionalities for Object Manipulation  |
  |__________________________________________| */   
  // The following functinalities of GOOD are usefull for manipulation tasks:

  // Get objec point cloud in local reference frame
  test_GOOD_descriptor.getTransformedObject (transformed_object);

  // Get three orthographic projects and transformation matrix 
  test_GOOD_descriptor.getOrthographicProjections (vector_of_projected_views);  
  test_GOOD_descriptor.getTransformationMatrix (transformation);
  std::cout << "\n transofrmation matrix =\n"<<transformation << std::endl;
  
  // Get object bounding box information 
  test_GOOD_descriptor.getCenterOfObjectBoundingBox (center_of_bounding_box); 
  test_GOOD_descriptor.getObjectBoundingBoxDimensions(bounding_box_dimensions);
  std::cout<<"\n center_of_bounding_box = " << center_of_bounding_box<<std::endl;
  std::cout<<"\n bounding_box_dimensions = " << bounding_box_dimensions <<std::endl;
  
  // Get the order of the three projected planes 
  test_GOOD_descriptor.getOrderOfProjectedPlanes(order_of_projected_planes);
  std::cout << "\n order of projected planes = "<<order_of_projected_planes << std::endl;

  if (gui_flag)
  {
    // Visualizing the transformed object, local reference fram and three orthographic projections
    *transformed_object_and_projected_views += *vector_of_projected_views.at(0);
    *transformed_object_and_projected_views += *vector_of_projected_views.at(1);
    *transformed_object_and_projected_views += *vector_of_projected_views.at(2);
    *transformed_object_and_projected_views += *transformed_object;    
    visualizationPointCloud(transformed_object_and_projected_views, "Transformed object and projected views");
  }
  return 0;
}
