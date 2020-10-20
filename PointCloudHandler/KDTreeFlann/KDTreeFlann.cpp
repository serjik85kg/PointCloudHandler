#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4267)
#endif

#include "KDTreeFlann.hpp"

#include <flann/flann.hpp>

#include "../PointCloud/PointCloud.hpp"

namespace pointcloudhandler {

		KDTreeFlann::KDTreeFlann() 
		{
		}

		KDTreeFlann::KDTreeFlann(const Eigen::MatrixXd& data) 
		{ 
			SetMatrixData(data); 
		}

		KDTreeFlann::KDTreeFlann(const Geometry& geometry) 
		{ 
			SetGeometry(geometry);
		}

		KDTreeFlann::~KDTreeFlann() 
		{
		}

		bool KDTreeFlann::SetMatrixData(const Eigen::MatrixXd& data) 
		{
			return SetRawData(Eigen::Map<const Eigen::MatrixXd>(data.data(), data.rows(), data.cols()));
		}

		// @ TO DO: add 2D pointclouds and other types
		bool KDTreeFlann::SetGeometry(const Geometry& geometry) 
		{
			switch (geometry.GetGeometryType()) 
			{
			case Geometry::GeometryType::PointCloud:
				return SetRawData(Eigen::Map<const Eigen::MatrixXd>(
					(const double *)((const PointCloud&)geometry).mPoints.data(),
					3, 
					((const PointCloud&)geometry).mPoints.size()));
			case Geometry::GeometryType::Unknown:
			default:
				printf("[KDTreeFlann::SetGeometry] Unsupported Geometry type.");
				return false;
			}
		}

		template <typename T>
		int KDTreeFlann::Search(const T& query, const KDTreeSearchParam& param,
			std::vector<int>& indices, std::vector<double>& distance2) const 
		{
			switch (param.GetSearchType()) 
			{
			case KDTreeSearchParam::SearchType::Knn:
				return SearchKNN(query, ((const KDTreeSearchParamKNN&)param).mKNN, indices, distance2);
			case KDTreeSearchParam::SearchType::Radius:
				return SearchRadius(query, ((const KDTreeSearchParamRadius&)param).mRadius, indices, distance2);
			default:
				return -1;
			}
			return -1;
		}

		template <typename T>
		int KDTreeFlann::SearchKNN(const T& query, int knn,
			std::vector<int>& indices, std::vector<double> & distance2) const 
		{
			if (mData.empty() || mDatasetSize <= 0 ||
				size_t(query.rows()) != mDimension || knn < 0) 
			{
				return -1;
			}
			flann::Matrix<double> queryFlann((double *)query.data(), 1, mDimension);
			indices.resize(knn);
			distance2.resize(knn);
			flann::Matrix<int> indicesFlann(indices.data(), queryFlann.rows, knn);
			flann::Matrix<double> distsFlann(distance2.data(), queryFlann.rows, knn);
			int k = mFlannIndex->knnSearch(queryFlann, indicesFlann, distsFlann, knn, flann::SearchParams(-1, 0.0));
			indices.resize(k);
			distance2.resize(k);
			return k;
		}

		template <typename T>
		int KDTreeFlann::SearchRadius(const T& query,double radius,
			std::vector<int>& indices, std::vector<double>& distance2) const 
		{
			if (mData.empty() || mDatasetSize <= 0 ||
				size_t(query.rows()) != mDimension) 
			{
				return -1;
			}
			flann::Matrix<double> queryFlann((double *)query.data(), 1, mDimension);
			flann::SearchParams param(-1, 0.0);
			param.max_neighbors = -1;
			std::vector<std::vector<int>> indicesVec(1);
			std::vector<std::vector<double>> distsVec(1);
			int k = mFlannIndex->radiusSearch(queryFlann, indicesVec, distsVec, float(radius * radius), param);
			indices = indicesVec[0];
			distance2 = distsVec[0];
			return k;
		}


		bool KDTreeFlann::SetRawData(const Eigen::Map<const Eigen::MatrixXd>& data) {
			mDimension = data.rows();
			mDatasetSize = data.cols();
			if (mDimension == 0 || mDatasetSize == 0) {
				printf("[KDTreeFlann::SetRawData] Failed due to no data.");
				return false;
			}
			mData.resize(mDatasetSize * mDimension);
			memcpy(mData.data(), data.data(), mDatasetSize * mDimension * sizeof(double));
			mFlannDataset.reset(new flann::Matrix<double>((double *)mData.data(), mDatasetSize, mDimension));
			mFlannIndex.reset(new flann::Index<flann::L2<double>>(*mFlannDataset, flann::KDTreeSingleIndexParams(15)));
			mFlannIndex->buildIndex();
			return true;
		}

		template int KDTreeFlann::Search<Eigen::Vector3d>(
			const Eigen::Vector3d& query,
			const KDTreeSearchParam& param,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

		template int KDTreeFlann::SearchKNN<Eigen::Vector3d>(
			const Eigen::Vector3d& query,
			int knn,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

		template int KDTreeFlann::SearchRadius<Eigen::Vector3d>(
			const Eigen::Vector3d& query,
			double radius,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

		template int KDTreeFlann::Search<Eigen::VectorXd>(
			const Eigen::VectorXd& query,
			const KDTreeSearchParam& param,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

		template int KDTreeFlann::SearchKNN<Eigen::VectorXd>(
			const Eigen::VectorXd& query,
			int knn,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

		template int KDTreeFlann::SearchRadius<Eigen::VectorXd>(
			const Eigen::VectorXd& query,
			double radius,
			std::vector<int>& indices,
			std::vector<double>& distance2) const;

}  // namespace pointcloudhandler

#ifdef _MSC_VER
#pragma warning(pop)
#endif