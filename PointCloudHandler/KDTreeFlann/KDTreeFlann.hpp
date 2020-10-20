#pragma once
#include <Eigen/Core>
#include <memory>
#include <vector>

#include "../PointCLoud/Geometry.hpp"
#include "KDTreeSearchParam.hpp"

namespace flann {
	template <typename T>
	class Matrix;
	template <typename T>
	struct L2;
	template <typename T>
	class Index;
}  // namespace flann

namespace pointcloudhandler 
{

		class KDTreeFlann {
		public:
			KDTreeFlann();
			KDTreeFlann(const Eigen::MatrixXd& data);
			KDTreeFlann(const Geometry& geometry);
			~KDTreeFlann();
			KDTreeFlann(const KDTreeFlann& ) = delete;
			KDTreeFlann& operator=(const KDTreeFlann& ) = delete;

		public:
			bool SetMatrixData(const Eigen::MatrixXd& data);
			bool SetGeometry(const Geometry& geometry);

			template <typename T>
			int Search(const T& query, const KDTreeSearchParam& param,
				std::vector<int>& indices, std::vector<double>& distance2) const;

			template <typename T>
			int SearchKNN(const T& query, int knn,
				std::vector<int>& indices, std::vector<double>& distance2) const;

			template <typename T>
			int SearchRadius(const T& query, double radius,
				std::vector<int>& indices, std::vector<double>& distance2) const;

		private:
			bool SetRawData(const Eigen::Map<const Eigen::MatrixXd>& data);

		protected:
			std::vector<double> mData;
			std::unique_ptr<flann::Matrix<double>> mFlannDataset;
			std::unique_ptr<flann::Index<flann::L2<double>>> mFlannIndex;
			size_t mDimension = 0;
			size_t mDatasetSize = 0;
		};

}  // namespace pointcloudhandler