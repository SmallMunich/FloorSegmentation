#include "GPF.h"

bool compare_cloudZ(pcl::PointXYZI a, pcl::PointXYZI b)
{
	return a.z < b.z;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr floor_seeds_(new pcl::PointCloud<pcl::PointXYZI>());


FloorSegment::FloorSegment()
{
	sensor_model_ = 16;
	num_seg_ = 1;
	num_iter_ = 3;
	num_lpr_ = 20;

	sensor_height_ = 2.5;
	th_seeds_ = 1.2;
	th_dist_ = 0.3;

	threshold_ = -1.5;
}

FloorSegment::~FloorSegment()
{

}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated 
    according to mean ground points.

    @param g_ground_pc:global ground pointcloud ptr.
    
*/
void FloorSegment::estimate_plane(void)
{

	Eigen::Matrix3f cov;
	Eigen::Vector4f pc_mean;

	pcl::computeMeanAndCovarianceMatrix(*floor_seeds_, cov, pc_mean);

	// Singular Value Decomposition: SVD
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);

	// use the least singular vector as normal
	normal_ = (svd.matrixU().col(2));

	// mean ground seeds value
	Eigen::Vector3f seeds_mean = pc_mean.head<3>();

	// according to normal.T * [x,y,z] = -d
	d_ = -(normal_.transpose()*seeds_mean)(0,0);

	// set distance threhold to `th_dist - d`
	th_dist_d_ = th_dist_ - d_;
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
*/

void FloorSegment::extract_initial_seeds(const pcl::PointCloud<pcl::PointXYZI>& p_sorted)
{
	// LPR is the mean of low point representative
	double sum = 0;
	int cnt = 0;

	// Calculate the mean height value.
	for(size_t i = 0; i < p_sorted.points.size() && cnt < num_lpr_; ++i)
	{
		sum += p_sorted.points[i].z;
		cnt ++;
	}

	double lpr_height = (cnt != 0 ? sum/cnt : 0);

	floor_seeds_->clear();
	// iterate pointcloud, filter those height is less than lpr.height + th_seeds_
	for(size_t i = 0; i < p_sorted.points.size(); ++i)
	{
		if (p_sorted.points[i].z < lpr_height + th_seeds_)
		{
			floor_seeds_->points.push_back(p_sorted.points[i]);
		}
	}

}


void FloorSegment::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZI>::Ptr nofloor_cloud, 
						pcl::PointCloud<pcl::PointXYZI>::Ptr onlyfloor_cloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

	for(size_t i = 0; i < cloudIn->points.size(); ++i)
	{
		pcl::PointXYZI point;
		point.x = cloudIn->points[i].x;
		point.y = cloudIn->points[i].y;
		point.z = cloudIn->points[i].z;
		point.intensity = cloudIn->points[i].intensity;
		cloud->points.push_back(point);
	}

	/// 1. sort on z-axis values
	sort((*cloud).points.begin(), (*cloud).points.end(), compare_cloudZ);

	/// 2. error point removal.
	// As there are some error mirror reflection under the ground,
	// here regardless point under 2 * sensor_height, 
	// sort point according to height, here uses z-axis in default.
	pcl::PointCloud<pcl::PointXYZI>::iterator it = (*cloud).points.begin();

	for(size_t i = 0; i < (*cloud).points.size(); ++i)
	{
		if ((*cloud).points[i].z < threshold_ * sensor_height_)
			++it;
		else
			break;
	}

	(*cloud).points.erase((*cloud).points.begin(), it);
	
	// 3. extract init ground seeds.
	extract_initial_seeds(*cloud);

	// onlyfloor_cloud = floor_seeds_; // ground points cloud.

	// 4. ground plane fit mainloop.
	for(int i = 0; i < num_iter_; ++i)
	{
		estimate_plane();

		onlyfloor_cloud->clear();
		nofloor_cloud->clear();

		// pointcloud to matrix
		Eigen::MatrixXf points(cloudIn->points.size(), 3);
		int j = 0;
		for(auto p : (*cloudIn).points)
		{
			points.row(j++) << p.x, p.y, p.z;
		}

		// ground plane model
		Eigen::VectorXf result = points * normal_;

		// threshold filter.
		for(int k = 0; k < result.rows(); ++k)
		{
			if (result[k] < th_dist_d_)
			{
				pcl::PointXYZI point;
				point.x = cloudIn->points[k].x;
				point.y = cloudIn->points[k].y;
				point.z = cloudIn->points[k].z;
				point.intensity = cloudIn->points[k].intensity;
				onlyfloor_cloud->points.push_back(point);
			}
			else
			{
				pcl::PointXYZI point;
				point.x = cloudIn->points[k].x;
				point.y = cloudIn->points[k].y;
				point.z = cloudIn->points[k].z;
				point.intensity = cloudIn->points[k].intensity;
				nofloor_cloud->points.push_back(point);
			}
		}
	}

}
