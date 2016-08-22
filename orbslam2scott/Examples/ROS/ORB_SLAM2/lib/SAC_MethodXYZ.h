#ifndef SAC_METHOD_H_
#define SAC_METHOD_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_types.h>
#include <iostream>

class SAC_MethodXYZ {
private:
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;

	/////////////////////////////////////////
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	Eigen::Vector3f axis;
public:
	int *inPlane;

public:
	SAC_MethodXYZ();
	pcl::PointIndices::Ptr applyRANSAC(
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float distThreshold,
			float epsAngle, int axisVal);
	void printPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	virtual ~SAC_MethodXYZ();
};

#endif /* SAC_METHOD_H_ */
