#pragma once
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Geometry.hpp"

#include <vector>

namespace pointcloudhandler {

		class Geometry3D : public Geometry {
		public:
			~Geometry3D() override 
			{
			}

		protected:
			Geometry3D(GeometryType type) : Geometry(type, 3) 
			{
			}

		public:
			Geometry3D& Clear() override = 0;
			bool IsEmpty() const override = 0;
			virtual Eigen::Vector3d GetMinBound() const = 0;
			virtual Eigen::Vector3d GetMaxBound() const = 0;
			virtual Eigen::Vector3d GetCenter() const = 0;

			virtual Geometry3D& Transform(const Eigen::Matrix4d& transformation) = 0;
			virtual Geometry3D& Translate(const Eigen::Vector3d& translation, bool relative = true) = 0;
			virtual Geometry3D& Scale(const double scale, bool center = true) = 0;
			virtual Geometry3D& Rotate(const Eigen::Matrix3d& R, bool center = true) = 0;

		protected:
			Eigen::Vector3d ComputeMinBound(const std::vector<Eigen::Vector3d>& points) const;
			Eigen::Vector3d ComputeMaxBound(const std::vector<Eigen::Vector3d>& points) const;
			Eigen::Vector3d ComputeCenter(const std::vector<Eigen::Vector3d>& points) const;

			void TransformPoints(const Eigen::Matrix4d& transformation,std::vector<Eigen::Vector3d>& points) const;
			void TranslatePoints(const Eigen::Vector3d& translation,std::vector<Eigen::Vector3d>& points,bool relative) const;
			void ScalePoints(const double scale, std::vector<Eigen::Vector3d>& points, bool center) const;
			void RotatePoints(const Eigen::Matrix3d& R, std::vector<Eigen::Vector3d>& points, bool center) const;
		};

}  // namespace pointcloudhandler