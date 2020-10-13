#include "Filehandler.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace pointcloudhandler
{
	PointCloud DataObj::ReadFrom(const std::string& inputFilename)
	{
		std::vector<Eigen::Vector3d> points; // @ TO DO check filesize
		std::ifstream file(inputFilename.c_str());

		if (!file.is_open())
		{
			std::cerr << "Cannot open " + inputFilename + " file." << std::endl;
		}
		else
		{
			std::cout << "File " + inputFilename + " opened." << std::endl;
			std::string line;
			while (getline(file, line))
			{
				std::istringstream in(line);
				std::string v;
				in >> v;
				if (v != "v") continue;

				float x, y, z;
				in >> x >> y >> z;
				points.emplace_back(x, y, z); // push_back({x, y, z});
			}
		}
		file.close();

		std::cout << "Points size: " << points.size() << std::endl;

		PointCloud cloud(points);

		return cloud;
	}

	// @ TO DO: add extension checker
	void DataObj::SaveTo(const PointCloud& pc, const std::string& outputFilename)
	{
		std::ofstream os(outputFilename.c_str());

		auto points = pc.Points();

		for (size_t i = 0; i < points.size(); ++i)
		{
			os << "v ";
			os << points[i][0] << " ";
			os << points[i][1] << " ";
			os << points[i][2] << "\n";
		}
		os.close();
	}
}