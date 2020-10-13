#pragma once
#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <vector>

#include "Geometry3D.hpp"
#include "../KDTreeFlann/KDTreeSearchParam.hpp"

namespace pointcloudhandler 
{

		class PointCloud : public Geometry3D 
		{
		public:
			PointCloud() : Geometry3D(Geometry::GeometryType::PointCloud) {}
			PointCloud(const std::vector<Eigen::Vector3d>& points)
				: Geometry3D(Geometry::GeometryType::PointCloud), mPoints(points) {}
			~PointCloud() override {}

		public:
			PointCloud& Clear() override;
			bool IsEmpty() const override;
			Eigen::Vector3d GetMinBound() const override;
			Eigen::Vector3d GetMaxBound() const override;
			Eigen::Vector3d GetCenter() const override;

			PointCloud& Transform(const Eigen::Matrix4d & transformation) override;
			PointCloud& Translate(const Eigen::Vector3d & translation, bool relative = true) override;
			PointCloud& Scale(const double scale, bool center = true) override;
			PointCloud& Rotate(const Eigen::Matrix3d & R, bool center = true) override;

			PointCloud& operator+=(const PointCloud & cloud);
			PointCloud operator+(const PointCloud & cloud) const; 

			bool HasPoints() const { return mPoints.size() > 0; }

			/// Remove all points fromt he point cloud that have a nan entries, or infinite entries.
			PointCloud& RemoveNoneFinitePoints(bool removeNan = true, bool removeInfinite = true);

			/// Function to select points from \param input pointcloud into
			/// \return output pointcloud
			/// Points with indices in \param indices are selected.
			std::shared_ptr<PointCloud> SelectReducedPts(const std::vector<size_t>& indices, bool invert = false) const;

			/// Function to remove points that have less than \param nb_points in a
			/// sphere of radius \param search_radius
			std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
				RemoveRadiusOutliers(size_t nbPoints, double searchRadius) const;

			/// Function to remove points that are further away from their
			/// \param nb_neighbor neighbors in average.
			std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
				RemoveStatisticalOutliers(size_t nbNeighbors, double stdRatio) const;


			/// Function to compute the point to point distances between point clouds
			/// \param source is the first point cloud.
			/// \param target is the second point cloud.
			/// \return the output distance. It has the same size as the number
			/// of points in \param source
			std::vector<double> ComputePointCloudDistance(const PointCloud & target);

			/// Function to compute the mean and covariance matrix
			/// of an \param input point cloud
			std::tuple<Eigen::Vector3d, Eigen::Matrix3d> ComputeMeanAndCovariance() const;

			/// Function to compute the Mahalanobis distance for points
			/// in an \param input point cloud
			/// https://en.wikipedia.org/wiki/Mahalanobis_distance
			std::vector<double> ComputeMahalanobisDistance() const;

			/// Function to compute the distance from a point to its nearest neighbor in
			/// the \param input point cloud
			std::vector<double> ComputeNearestNeighborDistance() const;

			size_t CloudSize() const {
				return mPoints.size();
			}

			std::vector<Eigen::Vector3d> Points() const {
				return mPoints;
			}

		public:
			std::vector<Eigen::Vector3d> mPoints;
		};

}  // namespace pointcloudhandler