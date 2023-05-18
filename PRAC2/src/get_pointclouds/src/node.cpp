#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <vector>
#include <iostream>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/keypoints/iss_3d.h>

// for x x1
// FPFH pa sacar features
// kdtreeFLANN pa encontrar iguales
// RANSAC pa filtrar los q no valen
// Encontrar la transformacion 
// Aplicar transf.






pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vector;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
Eigen::Matrix4f transformacion_total = Eigen::Matrix4f::Identity();
void simpleVis ()
{
  	pcl::visualization::PCLVisualizer viewer("Normals");
	
	while(!viewer.wasStopped())
	{
		cout << visu_pc->size() << endl;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(visu_pc, 0, 255, 255);
		viewer.removeAllPointClouds();
		viewer.addPointCloud(visu_pc, cloud_color_handler);
		//viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(visu_pc, cloud_normals, 1, 0.3, "yepa");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
		//viewer.addCoordinateSystem (1.0);
		viewer.initCameraParameters ();
		viewer.resetCamera();
		viewer.spinOnce(5000);
		//boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
	}
	// pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	// cout << visu_pc->size() << endl;
	// while(!viewer.wasStopped())
	// {
	//   viewer.showCloud (visu_pc);
	//   boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	// }

}

void normalVis(){
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	//viewer.addPointCloud<pcl::PointXYZRGB>(visu_pc);
	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(visu_pc, cloud_normals);
	while(!viewer.wasStopped())
	{

	  viewer.spinOnce();
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void visualizarCorrespondencias(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& c1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& c2, pcl::CorrespondencesPtr correspondences){
	pcl::visualization::PCLVisualizer viewer("ISS Keypoints");
	viewer.addPointCloud(c1, "yepa");
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 3) = 5.0;  // Translate by 10m in the X-axis
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*c2, *transformed_cloud, transformation_matrix);
	viewer.addPointCloud(transformed_cloud, "yepa2");

	int counter=0;
	for(size_t i = 0; i < correspondences->size(); ++i){
		counter++;
		pcl::Correspondence correspondence = (*correspondences)[i];
		int index_query = correspondence.index_query; // index of the point in the source (first) point cloud
		int index_match = correspondence.index_match;

		pcl::PointXYZRGB source_point = c1->points[correspondence.index_query];
		pcl::PointXYZRGB target_point = transformed_cloud->points[correspondence.index_match];
		cout << target_point << endl;
		viewer.addLine(source_point, target_point, 255, 0, 0, std::to_string(counter)); 
	}

	viewer.spinOnce(5000);
}

void normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	

	normal_estimation.setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setRadiusSearch(0.03); // Adjust the radius based on your data AJUSTAR
	normal_estimation.compute(*cloud_normals); // VISUALIZAR NORMALES
	visu_pc = cloud;
	cout << cloud->size() << "yee" << endl;
	//normalVis();

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	float iss_salient_radius = 0.05;   // Radio de influencia para puntos salientes
	float iss_non_max_radius = 0.04;   // Radio de no máxima supresión
	float iss_gamma_21 = 0.2;                       // Umbral de variabilidad
	float iss_gamma_32 = 0.5;                       // Umbral de variabilidad
	float iss_min_neighbors = 4;                      // Mínimo de vecinos
	int iss_threads = 8;  

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	normal_estimation.setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setRadiusSearch(0.03); // Adjust the radius based on your data AJUSTAR
	normal_estimation.compute(*normals); // VISUALIZAR NORMALES

	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
	//iss_detector.setSearchMethod(tree);
	iss_detector.setInputCloud(cloud);
	iss_detector.setNormals(normals);
	iss_detector.setSalientRadius(iss_salient_radius);
	iss_detector.setNonMaxRadius(iss_non_max_radius);
	iss_detector.setThreshold21(iss_gamma_21);
	iss_detector.setThreshold32(iss_gamma_32);
	iss_detector.setMinNeighbors(iss_min_neighbors);
	iss_detector.setNumberOfThreads(iss_threads);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	iss_detector.compute(*keypoints);

	// pcl::visualization::PCLVisualizer viewer("ISS Keypoints");
	// viewer.addPointCloud(cloud, "cloud");
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler(keypoints, 0, 255, 0);
	// viewer.addPointCloud<pcl::PointXYZRGB>(keypoints, keypoints_color_handler, "keypoints");
	// viewer.spinOnce(1000);
	return keypoints;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
	cout << "YEPA2";
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	normal_estimation.setInputCloud(cloud);
	normal_estimation.setSearchMethod(tree);
	normal_estimation.setRadiusSearch(0.03); // Adjust the radius based on your data AJUSTAR
	normal_estimation.compute(*normals); // VISUALIZAR NORMALES

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);

	fpfh.setInputCloud(cloud);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(tree);
	fpfh.setRadiusSearch(0.06); // Adjust the radius based on your data AJUSTAR
	// Porque tiene que ser mayor que el radio de las normales
	fpfh.compute(*fpfh_features);
	cout << "YEPA3";
	return fpfh_features;

}



void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{

	if(cloud_vector.size() > 2){
		float distance_threshold = 1;
		for(int i=0; i<cloud_vector.size()-1; i++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr x0 = cloud_vector[i];
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr x1 = cloud_vector[i+1];
			//sacar keypoints
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr key0 = iss(x0);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr key1 = iss(x1);
			//normals(x0);
			//normals(x1)
			cout << "KEY0: " << key0->size() << " KEY1: " << key1->size() << endl;
			
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh0 = fpfh(key0);
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1 = fpfh(key1);
			//assert(x0->size() == fpfh0->size());
			//assert(x1->size() == fpfh1->size());
			// Perform nearest-neighbor search
			// pcl::KdTreeFLANN<pcl::FPFHSignature33> match_search;
			// match_search.setInputCloud(fpfh1); // Set the feature descriptors of the first point cloud
			pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
			
			//Find the closest matches for each feature in the second point cloud
			pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimator;
			estimator.setInputSource(fpfh0);
			estimator.setInputTarget(fpfh1);
			estimator.determineCorrespondences(*correspondences);

			

			// for (size_t i = 0; i < fpfh0->size(); ++i) {
			// 	std::vector<int> indices(1);
			// 	std::vector<float> sqr_distances(1);
			// 	//cout << "ITERACION KSEARCH" << endl;
			// 	// Buscar el vecino más cercano en la nube de puntos objetivo
			// 	int num_neighbors_found = match_search.nearestKSearch((*fpfh0)[i], 1, indices, sqr_distances);

			// 	pcl::Correspondence correspondence;
			// 	correspondence.index_query = i; // Índice del punto de origen
			// 	correspondence.index_match = indices[0]; // Índice del punto objetivo
			// 	correspondence.distance = sqr_distances[0]; // Distancia al cuadrado entre los descriptores FPFH
			// 	correspondences->push_back(correspondence);
				
				
			// }
			visualizarCorrespondencias(x0,x1, correspondences);

			cout << "ANTES DE RANSAC " << correspondences->size() << "\n";
			cout << "YEPA";
			pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> ransac;
			ransac.setInputSource(key0); // MINIMA CANTIDAD DE PARAMETROS 4
			ransac.setInputTarget(key1);
			ransac.setInlierThreshold(0.05);
			ransac.setMaximumIterations(10000);
			ransac.setRefineModel(true);
			ransac.setInputCorrespondences(correspondences);
			pcl::CorrespondencesPtr inlier_correspondences(new pcl::Correspondences());
			ransac.getCorrespondences(*inlier_correspondences);
			//visualizarCorrespondencias(x0,x1, inlier_correspondences);
			// Verificar si se encontraron suficientes correspondencias inliers
			cout << "PUNTOS DESPUES DE RANSAC " << inlier_correspondences->size() << "\n";
			if (inlier_correspondences->size() < 3) {
				std::cerr << "No se encontraron suficientes correspondencias inliers" << std::endl;

			}
			else{
				Eigen::Matrix4f transformation_matrix = ransac.getBestTransformation();
				transformacion_total = transformacion_total * transformation_matrix;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::transformPointCloud(*x1, *cloud_transformed, transformacion_total);

				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZRGB>);
				*cloud_merged = *cloud_merged + *x0 + *cloud_transformed;
				visu_pc = cloud_merged;
				//simpleVis();
			}
		}
		cloud_vector.clear();

	}


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB > vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.02f, 0.02f, 0.02f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VG: " << cloud_filtered->size() << endl;
	cout << "YEPA4";
	cloud_vector.push_back(cloud_filtered);

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);

  //boost::thread t(simpleVis);
  //boost::thread t2(normalVis);

  while(ros::ok())
  {
	ros::spinOnce();
  }

}
