#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>   // 包含相关头文件
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> //直方图的可视化
#include <pcl/visualization/pcl_plotter.h>// 直方图的可视化 方法2
#include "resolution.h" // 用于计算模型分辨率
typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT; 
typedef pcl::FPFHSignature33 FeatureT;
int main(int argc, char** argv)
{
	// 读取点云
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("/home/blue/ply2pcd/water/water50_515_pxy_in.pcd", *cloud);
	// 读取关键点，也可以用之前提到的方法计算
	pcl::PointCloud<PointT>::Ptr keys(new pcl::PointCloud<PointT>);
	pcl::io::loadPCDFile("/home/blue/ply2pcd/water/water50_515_keypoint_harris.pcd", *keys);
	double resolution = computeCloudResolution(cloud);
	// 法向量
	pcl::NormalEstimation<PointT, PointNT> nest;
	//	nest.setRadiusSearch(10 * resolution);
	nest.setKSearch(10);
	nest.setInputCloud(cloud);
	nest.setSearchSurface(cloud);
	pcl::PointCloud<PointNT>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	nest.compute(*normals);
	std::cout << "compute normal\n";
	// 关键点计算PFH描述子
	pcl::PointCloud<FeatureT>::Ptr features(new pcl::PointCloud<FeatureT>);
	pcl::FPFHEstimation<PointT, PointNT, FeatureT> fest;
	fest.setRadiusSearch(18 * resolution);
	fest.setSearchSurface(cloud);
	fest.setInputCloud(keys);
	fest.setInputNormals(normals);
	fest.compute(*features);
	std::cout << "compute feature\n";
	
	pcl::visualization::PCLVisualizer viewer;
	viewer.addPointCloud(cloud, "cloud");
	//int level = 1;
	//float scale = 0.1;
	//viewer.addPointCloudNormals<PointNT>(normals, level, scale, "normals");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red_src(keys, 0, 255, 0);
	viewer.addPointCloud(keys, red_src, "keys"); 
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys");

	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*features, 300); //設定的橫座標長度，該值越大，則顯示的越細緻
	plotter.plot();
	
	viewer.spin();
	system("pause");
	return 0;
}
