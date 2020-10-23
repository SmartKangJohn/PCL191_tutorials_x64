#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

//可视化
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

using namespace std;
//viewer
void showCloudsWithPcd(vector<string>& pcdNames);

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  std::string pcd_r = "../data/table_scene_mug_stereo_textured.pcd";
  reader.read (pcd_r, *cloud);
  // Build a filter to remove spurious NaNs
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  std::string pcd_w = "../dataOut/table_scene_mug_stereo_textured_hull.pcd";
  writer.write (pcd_w, *cloud_hull, false);

  //可视化点云
  vector<string> pcdNames;
  pcdNames.push_back(pcd_r);
  pcdNames.push_back(pcd_w);
  showCloudsWithPcd(pcdNames);

  return (0);
}

void showCloudsWithPcd(vector<string>& pcdNames)
{
	int nSize = pcdNames.size();
	vector<pcl::PointCloud<pcl::PointXYZ>> clouds(nSize);
	vector<int> v_ports(nSize);
	for (int i = 0; i < nSize; i++)
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>(pcdNames[i], clouds[i]);
		v_ports[i] = i;
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int viewSize = 6;
	double xmin, ymin, xmax, ymax;
	double offsetx = 1.0 * 2.0 / viewSize, offsety = 1.0 / 2.0;
	for (int i = 0; i < viewSize; i++)
	{

		if ((double)i / 2 > 1)
		{
			xmin = offsetx * (i - 3), ymin = offsety;
			xmax = xmin + offsetx, ymax = ymin + offsety;
		}
		else
		{
			xmin = offsetx * i, ymin = 0.0;
			xmax = xmin + offsetx, ymax = offsety;
		}
		if (xmax > 1.0) { xmax = 1.0; }
		if (ymax > 1.0) { ymax = 1.0; }

		viewer->createViewPort(xmin, ymin, xmax, ymax, v_ports[i]);//(Xmin,Ymin,Xmax,Ymax)设置窗口坐标
		viewer->setBackgroundColor(0.1 + i * 0.1, 0.1 + i * 0.1, 0.1 + i * 0.1, v_ports[i]);//设置背景
		string text = "original" + to_string(i);
		string id = "v__ " + to_string(i);
		clouds[i].header.frame_id = id;
		viewer->addText(text, 10, 10, id, v_ports[i]);//设置视口名称
		id += to_string(i);
		if (i < nSize)
		{
			viewer->addPointCloud<pcl::PointXYZ>(clouds[i].makeShared(), id, v_ports[i]);//添加点云

			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 3.0, id, v_ports[i]);
		}
	}
	while (!viewer->wasStopped())
	{

		viewer->spin();
	}

}
