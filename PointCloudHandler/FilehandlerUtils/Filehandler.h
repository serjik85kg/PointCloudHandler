#pragma once

#include "../PointCloud/PointCloud.hpp"

namespace pointcloudhandler 
{

	class Data
	{
	public:
		enum class DataType
		{
			Unknown = 0,
			Obj = 1, // 
			PCL = 2 // Point Cloud Library https://pointclouds.org/
		};
		virtual ~Data() {}
		virtual PointCloud ReadFrom(const std::string & file) = 0;
		virtual void SaveTo(const PointCloud& pc, const std::string& file) = 0;
	protected:
		Data(DataType type) : mDataType(type) {}
	private:
		DataType mDataType;
	};

	class DataObj : public Data
	{
	public:
		DataObj() : Data(DataType::Obj) {}
		PointCloud ReadFrom(const std::string& file) override;
		void SaveTo(const PointCloud& pc, const std::string& file) override;
	};

	class FileHandler
	{
	public:

		FileHandler(Data * p) : mData(p) {}

		~FileHandler() { delete mData; }

		PointCloud ReadFrom(const std::string& filepath)
		{
			return mData->ReadFrom(filepath);
		}
		void SaveTo(const PointCloud& pc, const std::string& filepath)
		{
			mData->SaveTo(pc, filepath);
		}
	private:
		Data * mData;
	};

	// class DataPCL


}