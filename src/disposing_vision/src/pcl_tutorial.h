#define PCL_TUTORIAL_H
#ifdef PCL_TUTORIAL_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointT;

class Pcl_tutorial
{
private:

public:
    Pcl_tutorial();
    ~Pcl_tutorial();

    void cylinder_segmentation(pcl::PointCloud<PointT>::Ptr cloud
                              ,pcl::PointCloud<PointT> &cloud_cylinder
                              ,pcl::PointCloud<PointT> &cloud_plane);
};

#endif //PCL_TUTORIAL_H