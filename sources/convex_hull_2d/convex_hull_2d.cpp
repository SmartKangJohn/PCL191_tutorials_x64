#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;
  reader.read ("../data/table_scene_mug_stereo_textured.pcd", *cloud);

  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  //参数模型
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  //有效点云
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;  //采样一致性分割
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;  //投影滤波：使用一个模型和一组的内点的索引，将内点投影到模型形成新的一个独立点云
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_filtered);
  proj.setIndices (inliers);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  // Create a Convex Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;  //凸包检测对象
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("../dataOut/table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}
