#include <numeric>
#include <unordered_map>

#include "../KDTreeFlann/KDTreeFlann.hpp"
#include "PointCloud.hpp"
#include <iostream>

namespace pointcloudhandler 
{
		std::shared_ptr<PointCloud> PointCloud::SelectReducedPts(const std::vector<size_t> &indices, bool invert /* = false */) const 
		{
			auto output = std::make_shared<PointCloud>();

			std::vector<bool> mask = std::vector<bool>(mPoints.size(), invert);
			for (size_t i : indices) 
			{
				mask[i] = !invert;
			}

			for (size_t i = 0; i < mPoints.size(); i++) 
			{
				if (mask[i]) 
				{
					output->mPoints.push_back(mPoints[i]);
				}
			}

			std::cout << "Pointcloud reduced from " << (int)mPoints.size() << " points to " << (int)output->mPoints.size() << " points." << std::endl;
			return output;
		}


		std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
			PointCloud::RemoveRadiusOutliers(size_t nbPoints, double searchRadius) const 
		{
			if (nbPoints < 1 || searchRadius <= 0) 
			{
				printf("[RemoveRadiusOutliers] Illegal input parameters,"
					"number of points and radius must be positive");
			}
			KDTreeFlann kdtree;
			kdtree.SetGeometry(*this);
			std::vector<bool> mask = std::vector<bool>(mPoints.size());

			for (int i = 0; i < int(mPoints.size()); i++) 
			{
				//printf("ok\n");
				std::vector<int> tmpIndices;
				std::vector<double> dist;
				size_t nbNeighbors = kdtree.SearchRadius(mPoints[i], searchRadius,
					tmpIndices, dist);
				mask[i] = (nbNeighbors > nbPoints);
			}
			std::vector<size_t> indices;
			for (size_t i = 0; i < mask.size(); i++) 
			{
				if (mask[i]) 
				{
					indices.push_back(i);
				}
			}
			return std::make_tuple(SelectReducedPts(indices), indices);
		}

		std::tuple<std::shared_ptr<PointCloud>, std::vector<size_t>>
			PointCloud::RemoveStatisticalOutliers(size_t nbNeighbors, double stdRatio) const 
		{
			if (nbNeighbors < 1 || stdRatio <= 0) {
				printf("[RemoveStatisticalOutliers] Illegal input parameters, number "
					"of neighbors and standard deviation ratio must be positive");
			}
			if (mPoints.size() == 0) 
			{
				return std::make_tuple(std::make_shared<PointCloud>(), std::vector<size_t>());
			}
			KDTreeFlann kdtree;
			kdtree.SetGeometry(*this);
			std::vector<double> avgDistances = std::vector<double>(mPoints.size());
			std::vector<size_t> indices;
			size_t validDistances = 0;
			for (int i = 0; i < int(mPoints.size()); i++) 
			{
				std::vector<int> tmpIndices;
				std::vector<double> dist;
				kdtree.SearchKNN(mPoints[i], int(nbNeighbors), tmpIndices, dist);
				double mean = -1.0;
				if (dist.size() > 0u) 
				{
					validDistances++;
					std::for_each(dist.begin(), dist.end(),
						[](double &d) 
					{ 
						d = std::sqrt(d); 
					});
					mean = std::accumulate(dist.begin(), dist.end(), 0.0) / dist.size();
				}
				avgDistances[i] = mean;
			}
			if (validDistances == 0) 
			{
				return std::make_tuple(std::make_shared<PointCloud>(), std::vector<size_t>());
			}
			double cloudMean = std::accumulate(avgDistances.begin(), avgDistances.end(), 0.0,
				[](double const &x, double const &y) 
			{ 
				return y > 0 ? x + y : x; 
			});
			cloudMean /= validDistances;
			double sqSum = std::inner_product(avgDistances.begin(), avgDistances.end(), avgDistances.begin(), 0.0,
				[](double const &x, double const &y) 
			{ 
				return x + y; 
			},
				[cloudMean](double const &x, double const &y) 
			{
				return x > 0 ? (x - cloudMean) * (y - cloudMean) : 0;
			});
			// Bessel's correction
			double stdDev = std::sqrt(sqSum / (validDistances - 1));
			double distanceThreshold = cloudMean + stdRatio * stdDev;
			for (size_t i = 0; i < avgDistances.size(); i++) 
			{
				if (avgDistances[i] > 0 && avgDistances[i] < distanceThreshold) 
				{
					indices.push_back(i);
				}
			}
			return std::make_tuple(SelectReducedPts(indices), indices);
		}

}  // namespace pointcloudhandler