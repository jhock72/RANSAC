//  linear_ransac.hpp



#ifndef linear_ransac_hpp
#define linear_ransac_hpp

#include <vector>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "read_csv.hpp"
#include "ransac.hpp"

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class LinearRansac : public Ransac
{
public:
    
    // Read a data file and set algorithm parameters
    // @param ro Object of ReadCSV class
    // @param min_inliers The minimum number of inliers required to assert that a model fits well to the data
    // @param thres Threshold value to determine data points that are fit well by the model
    // @param numb_iter Number of times a model will be estimated and its fitness with the data exampined
     explicit LinearRansac (const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter);
    ~LinearRansac();
    
    virtual std::ostream& print(std::ostream& out) const override final;
        
private:
    std::vector<point> data_points_copy_;
    
    struct ModelInfo
    {
        double slope;
        double y_int;
        double total_error;
        unsigned int numb_inliers;
        std::vector<point> inliers;
        std::vector<double> inlier_errors;
    };
    
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
    const double kAussumedRatioInliersToDatapoints{.5}, kDefaultInlierThreshold{.5};
    const int kNumbDataPointsForLinearModel{2};

    // Containers to hold information about models
    ModelInfo proposed_, better_, best_;
    
    //
    std::mt19937 rn_engine_;
    
};


#endif /* linear_ransac_hpp */


