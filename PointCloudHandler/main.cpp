#include "../PointCloudHandler/PointCloud/PointCloud.hpp"
#include "FilehandlerUtils/Filehandler.h"

#include <opencv2\opencv.hpp>
#include <opencv2\core\eigen.hpp>

using namespace pointcloudhandler;

void filter3D(const std::string& inputPath, const std::string& outputPath);

int main()
{
	const std::string girlbag = "../examples/girl_bag.obj";
	const std::string outputtest = "../examples/test.obj";
	filter3D(girlbag, outputtest);
	system("pause");
}

void filter3D(const std::string& inputPath, const std::string& outputPath)
{
	FileHandler * fileHandler = new FileHandler(new DataObj);

	auto pc = fileHandler->ReadFrom(inputPath);

	auto statfi = pc.RemoveStatisticalOutliers(5, 10);
	std::shared_ptr<PointCloud> pcstat;
	std::vector<size_t> szstat;
	std::tie(pcstat, szstat) = statfi;
	auto pcFiltered = *pcstat;
	std::cout << "check" << std::endl;
	fileHandler->SaveTo(pcFiltered, outputPath);
}