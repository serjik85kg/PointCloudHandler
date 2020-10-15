#pragma once

namespace pointcloudhandler 
{

		class KDTreeSearchParam 
		{
		public:
			enum class SearchType 
			{
				Knn = 0,
				Radius = 1,
			};

		public:
			virtual ~KDTreeSearchParam() {}

		protected:
			KDTreeSearchParam(SearchType type) : mSearchType(type) 
			{}

		public:
			SearchType GetSearchType() const 
			{ 
				return mSearchType;
			}

		private:
			SearchType mSearchType;
		};

		class KDTreeSearchParamKNN : public KDTreeSearchParam 
		{
		public:
			KDTreeSearchParamKNN(int knn = 30)
				: KDTreeSearchParam(SearchType::Knn), mKNN(knn) {}

		public:
			int mKNN;
		};

		class KDTreeSearchParamRadius : public KDTreeSearchParam {
		public:
			KDTreeSearchParamRadius(double radius)
				: KDTreeSearchParam(SearchType::Radius), mRadius(radius) {}

		public:
			double mRadius;
		};

}  // namespace pointcloudhandler