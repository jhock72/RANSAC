//  circular_ransac.hpp



#ifndef circular_ransac_hpp
#define circular_ransac_hpp

#include <vector>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "read_csv.hpp"
#include "ransac.hpp"

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class CircularRansac : public Ransac
{
public:
    explicit CircularRansac(const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter);
    
    virtual std::ostream& print(std::ostream& out) const override final;
    
private:
        std::vector<point> data_points_copy_;
    
        struct ModelInfo
        {
            point center;
            double radius;
            unsigned int numb_inliers;
            double total_error;
            std::vector<point> inliers;
            std::vector <double> inlier_error;
        };
    
        // Containers to hold information about models
        ModelInfo proposed_, better_, best_;
    
        virtual std::vector<point>& GetPossibleInliers (const ModelType& type) override final;
        virtual void CalculateParameters (const ModelType& type, const std::vector<point>& data_to_fit) override final;
        virtual bool CalculateError (const ModelType type, const std::vector<point>& data_to_fit, bool find_inliers) override final;
        virtual void SaveBetterModelAsBest() override final;
    
        // A set of functions that are utilized to retieve error values from derived classes for comparison purposes
        virtual double GetBestError() const override final;
        virtual double GetBetterError() const override final;
        virtual unsigned int GetNumbInliersProposed() const override final;
        virtual void SetBestError(double err) override final;
    
        // Defaults or characteritics of the model
        const int kNumbDataPointsForCircularModel{3};
        const double kBigInitialError{10000};
        const int kPointsPerCircle{360};

        std::mt19937 rn_engine_;
};

#endif /* circular_ransac_hpp */
