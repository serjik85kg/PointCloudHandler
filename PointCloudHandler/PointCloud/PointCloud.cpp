#include "PointCloud.hpp"

#include <Eigen/Dense>
#include <numeric>

#include "../KDTreeFlann/KDTreeFlann.hpp"


namespace pointcloudhandler 
{

		PointCloud& PointCloud::Clear() 
		{
			mPoints.clear();
			return *this;
		}

		bool PointCloud::IsEmpty() const 
		{ 
			return !HasPoints(); 
		}

		Eigen::Vector3d PointCloud::GetMinBound() const 
		{
			return ComputeMinBound(mPoints);
		}

		Eigen::Vector3d PointCloud::GetMaxBound() const 
		{
			return ComputeMaxBound(mPoints);
		}

		Eigen::Vector3d PointCloud::GetCenter() const 
		{ 
			return ComputeCenter(mPoints); 
		}

		PointCloud& PointCloud::Transform(const Eigen::Matrix4d &transformation) 
		{
			TransformPoints(transformation, mPoints);
			return *this;
		}

		PointCloud& PointCloud::Translate(const Eigen::Vector3d &translation, bool relative) 
		{
			TranslatePoints(translation, mPoints, relative);
			return *this;
		}

		PointCloud& PointCloud::Scale(const double scale, bool center) 
		{
			ScalePoints(scale, mPoints, center);
			return *this;
		}

		PointCloud& PointCloud::Rotate(const Eigen::Matrix3d &R, bool center) 
		{
			RotatePoints(R, mPoints, center);
			return *this;
		}

		PointCloud& PointCloud::operator+=(const PointCloud &cloud) 
		{
			if (cloud.IsEmpty()) 
				return (*this);
			size_t oldVertNum = mPoints.size();
			size_t addVertNum = cloud.mPoints.size();
			size_t newVertNum = oldVertNum + addVertNum;
			mPoints.resize(newVertNum);
			for (size_t i = 0; i < addVertNum; i++)
				mPoints[oldVertNum + i] = cloud.mPoints[i];
			return (*this);
		}

		PointCloud PointCloud::operator+(const PointCloud &cloud) const 
		{
			return (PointCloud(*this) += cloud);
		}

		std::vector<double> PointCloud::ComputePointCloudDistance(const PointCloud &target) 
		{
			std::vector<double> distances(mPoints.size());
			KDTreeFlann kdtree;
			kdtree.SetGeometry(target);
			for (int i = 0; i < (int)mPoints.size(); i++) 
			{
				std::vector<int> indices(1);
				std::vector<double> dists(1);
				if (kdtree.SearchKNN(mPoints[i], 1, indices, dists) == 0) 
				{
					/*Found a point without neighbors.");*/
					distances[i] = 0.0;
				}
				else 
				{
					distances[i] = std::sqrt(dists[0]);
				}
			}
			return distances;
		}

		PointCloud& PointCloud::RemoveNoneFinitePoints(bool removeNan, bool removeInfinite) {
			size_t oldPointNum = mPoints.size();
			size_t k = 0;                                 // new index
			for (size_t i = 0; i < oldPointNum; i++) 
			{  // old index
				bool isNan = removeNan &&
					(std::isnan(mPoints[i](0)) || std::isnan(mPoints[i](1)) || std::isnan(mPoints[i](2)));
				bool is_infinite = removeInfinite && 
					(std::isinf(mPoints[i](0)) || std::isinf(mPoints[i](1)) ||std::isinf(mPoints[i](2)));
				if (!isNan && !is_infinite) 
				{
					mPoints[k] = mPoints[i];
					k++;
				}
			}
			mPoints.resize(k);
			return *this;
		}

		std::tuple<Eigen::Vector3d, Eigen::Matrix3d>PointCloud::ComputeMeanAndCovariance() const 
		{
			if (IsEmpty()) 
			{
				return std::make_tuple(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
			}
			Eigen::Matrix<double, 9, 1> cumulants;
			cumulants.setZero();
			for (const auto &point : mPoints) 
			{
				cumulants(0) += point(0);
				cumulants(1) += point(1);
				cumulants(2) += point(2);
				cumulants(3) += point(0) * point(0);
				cumulants(4) += point(0) * point(1);
				cumulants(5) += point(0) * point(2);
				cumulants(6) += point(1) * point(1);
				cumulants(7) += point(1) * point(2);
				cumulants(8) += point(2) * point(2);
			}
			cumulants /= (double)mPoints.size();
			Eigen::Vector3d mean;
			Eigen::Matrix3d covariance;
			mean(0) = cumulants(0);
			mean(1) = cumulants(1);
			mean(2) = cumulants(2);
			covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
			covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
			covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
			covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
			covariance(1, 0) = covariance(0, 1);
			covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
			covariance(2, 0) = covariance(0, 2);
			covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
			covariance(2, 1) = covariance(1, 2);
			return std::make_tuple(mean, covariance);
		}

		std::vector<double> PointCloud::ComputeMahalanobisDistance() const 
		{
			std::vector<double> mahalanobis(mPoints.size());
			Eigen::Vector3d mean;
			Eigen::Matrix3d covariance;
			std::tie(mean, covariance) = ComputeMeanAndCovariance();
			Eigen::Matrix3d covInv = covariance.inverse();

			for (int i = 0; i < (int)mPoints.size(); i++) 
			{
				Eigen::Vector3d p = mPoints[i] - mean;
				mahalanobis[i] = std::sqrt(p.transpose() * covInv * p);
			}
			return mahalanobis;
		}

		std::vector<double> PointCloud::ComputeNearestNeighborDistance() const 
		{
			std::vector<double> nnDis(mPoints.size());
			KDTreeFlann kdtree(*this);
			for (int i = 0; i < (int)mPoints.size(); i++) 
			{
				std::vector<int> indices(2);
				std::vector<double> dists(2);
				if (kdtree.SearchKNN(mPoints[i], 2, indices, dists) <= 1) 
				{
					/*Found a point without neighbors.");*/
					nnDis[i] = 0.0;
				}
				else 
				{
					nnDis[i] = std::sqrt(dists[1]);
				}
			}
			return nnDis;
		}

}  // namespace pointcloudhandler