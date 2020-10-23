#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

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
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("../data/table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "../dataOut/cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  //可视化点云
  vector<string> pcdNames;
  pcdNames.push_back("../data/table_scene_lms400.pcd");
  pcdNames.push_back("../dataOut/cloud_cluster_0.pcd");
  pcdNames.push_back("../dataOut/cloud_cluster_1.pcd");
  pcdNames.push_back("../dataOut/cloud_cluster_2.pcd");
  pcdNames.push_back("../dataOut/cloud_cluster_3.pcd");
  pcdNames.push_back("../dataOut/cloud_cluster_4.pcd");
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

	double xmin, ymin, xmax, ymax;
	double offsetx = 1.0 * 2.0 / nSize, offsety = 1.0 / 2.0;
	for (int i = 0; i < nSize; i++)
	{

		if ((double)i/2 > 1)
		{
			xmin = offsetx * (i-3), ymin = offsety;
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
		viewer->addPointCloud<pcl::PointXYZ>(clouds[i].makeShared(), id, v_ports[i]);//添加点云

		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 3.0, id, v_ports[i]);
	}
	while (!viewer->wasStopped())
	{

		viewer->spin();
	}

}
