/*
 * Software License Agreement (BSD License)
 *
 *  	Hamidreza Kasaei - http://wiki.ieeta.pt/wiki/index.php/Hamidreza_Kasaei
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */


#ifndef GOOD_H_
#define GOOD_H_

#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>

/** \brief GOOD: a Global Orthographic Object Descriptor for 3D object recognition and manipulation.
  * GOOD descriptor has been designed to be robust, descriptive and efficient to compute and use. 
  * It has two outstanding characteristics: 
  * 
  * (1) Providing a good trade-off among :
  *	- descriptiveness,
  *	- robustness,
  *	- computation time,
  *	- memory usage.
  * 
  * (2) Allowing concurrent object recognition and pose estimation for manipulation.
  * 
  * \note This is an implementation of the GOOD descriptor which has been presented in the following papers:
  * 
  *	[1] Kasaei, S. Hamidreza,  Ana Maria Tomé, Luís Seabra Lopes, Miguel Oliveira 
  *	"GOOD: A global orthographic object descriptor for 3D object recognition and manipulation." 
  *	Pattern Recognition Letters 83 (2016): 312-320.http://dx.doi.org/10.1016/j.patrec.2016.07.006
  *
  *	[2] Kasaei, S. Hamidreza, Luís Seabra Lopes, Ana Maria Tomé, Miguel Oliveira 
  * 	"An orthographic descriptor for 3D object learning and recognition." 
  *	2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, 2016, 
  *	pp. 4158-4163. doi: 10.1109/IROS.2016.7759612
  * 
  * Please adequately refer to this work any time this code is being used by citing above papers.
  * If you do publish a paper where GOOD descriptor helped your research, we encourage you to cite the above papers in your publications.
  * 
  * \author Hamidreza Kasaei (Seyed.Hamidreza[at]ua[dot]pt)
  */
  
template <typename PointInT>
class GOODEstimation
{
  public:     

    GOODEstimation();     
    GOODEstimation(unsigned int, float);
    
    /** \brief Empty destructor */
    virtual ~GOODEstimation () {}
    
    void
    setInputCloud (boost::shared_ptr<pcl::PointCloud<PointInT> > cloud);  
    
    void
    setNumberOfBins (unsigned int number_of_bins);  
    
    void
    setThreshold (float threshold);
    
    void
    compute (std::vector< float > &object_description );

    void
    getOrthographicProjections (std::vector < boost::shared_ptr<pcl::PointCloud<PointInT> > > &vector_of_projected_views);
    
    void
    getTransformedObject (boost::shared_ptr<pcl::PointCloud<PointInT> > &transformed_point_cloud);
    
    void
    getCenterOfObjectBoundingBox (pcl::PointXYZ &center_of_bbox);
     
    void
    getObjectBoundingBoxDimensions(pcl::PointXYZ &bbox_dimensions);
    
    void 
    getOrderOfProjectedPlanes(std::string &order_of_projected_plane);
    
    void 
    getTransformationMatrix( Eigen::Matrix4f &transformation);
    
  private:

    unsigned int number_of_bins_; 
    float threshold_;      
    int sign_;
    boost::shared_ptr<pcl::PointCloud<PointInT> > input_ ;
    boost::shared_ptr<pcl::PointCloud<PointInT> > transformed_point_cloud_;  
    pcl::PointXYZ bbox_dimensions_;
    pcl::PointXYZ center_of_bbox_;
    std::vector < boost::shared_ptr<pcl::PointCloud<PointInT> > > vector_of_projected_views_;
    std::string projections_ordered_;
    std::string order_of_projected_plane_;    
    Eigen::Matrix4f transformation_;
    
    void 
    computeBoundingBoxDimensions (boost::shared_ptr<pcl::PointCloud<PointInT> > pc, pcl::PointXYZ& dimensions);

    void
    projectPointCloudToPlane (boost::shared_ptr<pcl::PointCloud<PointInT> > pc_in, boost::shared_ptr<pcl::ModelCoefficients> coefficients, boost::shared_ptr<pcl::PointCloud<PointInT> > pc_out);

    void 
    convert2DHistogramTo1DHistogram (std::vector <std::vector <unsigned int> >  histogram_2D, std::vector <unsigned int>  &histogram);

    void
    signDisambiguationXAxis (boost::shared_ptr<pcl::PointCloud<PointInT> >  XoZ_projected_view, float threshold, int &sign );
    
    void 
    signDisambiguationYAxis (boost::shared_ptr<pcl::PointCloud<PointInT> >  YoZ_projected_view, float threshold, int &sign );

    void
    create2DHistogramFromYOZProjection (boost::shared_ptr<pcl::PointCloud<PointInT> >  YOZ_projected_view,
			double largest_side, unsigned int number_of_bins, int sign, std::vector < std::vector<unsigned int> > &YOZ_histogram);

    void 
    create2DHistogramFromXOZProjection ( boost::shared_ptr<pcl::PointCloud<PointInT> >  XOZ_projected_view, double largest_side, 
					  unsigned int number_of_bins, int sign, std::vector < std::vector<unsigned int> > &XOZ_histogram);
    void 
    create2DHistogramFromXOYProjection (boost::shared_ptr<pcl::PointCloud<PointInT> >  XOY_projected_view,	double largest_side,
				      unsigned int number_of_bins, int sign, std::vector < std::vector<unsigned int> > &XOY_histogram);

    void
    normalizingHistogram (std::vector <unsigned int> histogram, std::vector <float> &normalized_histogram);

    void
    viewpointEntropy (std::vector <float> normalized_histogram, float &entropy);

    void
    findMaxViewPointEntropy (std::vector <float> view_point_entropy, int &index);

    void
    averageHistograms (std::vector< float> histogram1, std::vector< float> historam2, std::vector< float> historam3, std::vector< float> &average);

    void
    meanOfHistogram (std::vector< float> histogram, float &mean);

    void
    varianceOfHistogram (std::vector< float> histogram, float mean, float &variance);
    
    void
    objectViewHistogram (int maximum_entropy_index, std::vector< std::vector<float> >normalized_projected_views,
		    std::vector< float> &sorted_normalized_projected_views,
		    std::string &name_of_sorted_projected_plane /*debug*/);

    void
    computeLargestSideOfBoundingBox (pcl::PointXYZ dimensions, double &largest_side );

    void
    computeDistanceBetweenProjections (std::vector <std::vector <float> > projection1, std::vector <std::vector <float> > projection2, float &distance);

    inline std::vector < std::vector<unsigned int> > initializing2DHistogram (unsigned int number_of_bins);
    
};

#endif
