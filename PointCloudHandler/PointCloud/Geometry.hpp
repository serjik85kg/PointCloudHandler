#pragma once

namespace pointcloudhandler 
{

		class Geometry 
		{
		public:
			enum class GeometryType 
			{
				Unknown = 0,
				PointCloud = 1,
			};

		public:
			virtual ~Geometry() {}

		protected:
			Geometry(GeometryType type, int dimension)
				: mGeometryType(type), mDimension(dimension) 
			{
			}

		public:
			virtual Geometry& Clear() = 0;
			virtual bool IsEmpty() const = 0;
			GeometryType GetGeometryType() const 
			{ 
				return mGeometryType; 
			}
			int Dimension() const 
			{ 
				return mDimension; 
			}

		private:
			GeometryType mGeometryType = GeometryType::Unknown;
			int mDimension = 3;
		};

} // namespace pointcloudhandler