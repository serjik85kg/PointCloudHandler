#include "Geometry3D.hpp"

#include <Eigen/Dense>
#include <numeric>

namespace pointcloudhandler {

		Eigen::Vector3d Geometry3D::ComputeMinBound(const std::vector<Eigen::Vector3d>& points) const 
		{
			if (points.empty()) 
			{
				return Eigen::Vector3d(0.0, 0.0, 0.0);
			}
			return std::accumulate(points.begin(), points.end(), points[0],
				[](const Eigen::Vector3d& a, const Eigen::Vector3d& b) 
			{
				return a.array().min(b.array()).matrix();
			});
		}

		Eigen::Vector3d Geometry3D::ComputeMaxBound(const std::vector<Eigen::Vector3d>& points) const 
		{
			if (points.empty()) {
				return Eigen::Vector3d(0.0, 0.0, 0.0);
			}
			return std::accumulate(points.begin(), points.end(), points[0],
				[](const Eigen::Vector3d& a, const Eigen::Vector3d& b) 
			{
				return a.array().max(b.array()).matrix();
			});
		}

		Eigen::Vector3d Geometry3D::ComputeCenter(const std::vector<Eigen::Vector3d>& points) const 
		{
			Eigen::Vector3d center(0, 0, 0);
			if (points.empty()) 
			{
				return center;
			}
			center = std::accumulate(points.begin(), points.end(), center);
			center /= double(points.size());
			return center;
		}

		void Geometry3D::TransformPoints(const Eigen::Matrix4d& transformation, std::vector<Eigen::Vector3d>& points) const 
		{
			for (auto& point : points) 
			{
				Eigen::Vector4d newPoint = transformation * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
				point = newPoint.head<3>() / newPoint(3);
			}
		}

		void Geometry3D::TranslatePoints(const Eigen::Vector3d& translation, std::vector<Eigen::Vector3d>& points, bool relative) const 
		{
			Eigen::Vector3d transform = translation;
			if (!relative) 
			{
				transform -= ComputeCenter(points);
			}
			for (auto& point : points) 
			{
				point += transform;
			}
		}

		void Geometry3D::ScalePoints(const double scale, std::vector<Eigen::Vector3d>& points, bool center) const 
		{
			Eigen::Vector3d pointsCenter(0, 0, 0);
			if (center && !points.empty()) 
			{
				pointsCenter = ComputeCenter(points);
			}
			for (auto& point : points) 
			{
				point = (point - pointsCenter) * scale + pointsCenter;
			}
		}

		void Geometry3D::RotatePoints(const Eigen::Matrix3d& R, std::vector<Eigen::Vector3d>& points, bool center) const 
		{
			Eigen::Vector3d pointsCenter(0, 0, 0);
			if (center && !points.empty()) 
			{
				pointsCenter = ComputeCenter(points);
			}
			for (auto& point : points) 
			{
				point = R * (point - pointsCenter) + pointsCenter;
			}
		}


}  // namespace pointcloudfilter