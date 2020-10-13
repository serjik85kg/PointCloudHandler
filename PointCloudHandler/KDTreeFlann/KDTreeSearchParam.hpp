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
				Hybrid = 2,
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

		class KDTreeSearchParamHybrid : public KDTreeSearchParam {
		public:
			KDTreeSearchParamHybrid(double radius, int maxNN)
				: KDTreeSearchParam(SearchType::Hybrid),
				mRadius(radius),
				mMaxNN(maxNN) {}

		public:
			double mRadius;
			int mMaxNN;
		};

}  // namespace pointcloudhandler