//  ransac.hpp



#ifndef ransac_hpp
#define ransac_hpp

#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "read_csv.hpp"

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class Ransac
{
public:
    
    // Read a data file and set algorithm parameters
    // @param ro Object of ReadCSV class
    explicit Ransac (const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter);
    virtual ~Ransac();

    enum class ModelType
      {
          PROPOSED,
          BETTER,
          BEST,
      };
    
    bool IterativeFit();
    
    void SetNumbIterations(unsigned int numb_iter);
    void SetNumbInliers (unsigned int min_numb_inliers);
    void SetInlierErrorThres (double inlier_error_thres);
    
    unsigned int GetNumberIterations () const;
    unsigned int GetMinNumberInliers () const;
    double GetInlierErrorThreshold () const;
    
    virtual std::ostream& print(std::ostream &out) const;
    
    
private:
    
    virtual std::vector<point>& GetPossibleInliers (const ModelType& type)=0;
    
    virtual bool ComputeModel (const ModelType& type);
    
    virtual void CalculateParameters (const ModelType& type, const std::vector<point>& data_to_fit)=0;
    
    virtual bool CalculateError (const ModelType, const std::vector<point>& data_to_fit, bool find_inliers)=0;
    
    // Checks if the proposed model has the minimum number of inliers required
    bool CheckProposedModel();

    // Compares the error value contained in better_ to best_
    // Overwrites best_ if better_ has a lower error
    void CompareBetterToBest();
    virtual void SaveBetterModelAsBest()=0;
        
    // A set of functions that are utilized to retieve error values from derived classes for comparison purposes
    virtual double GetBestError() const =0;
    virtual double GetBetterError() const =0;
    virtual unsigned int GetNumbInliersProposed() const =0;
    virtual void SetBestError(double err)=0;
    
    
protected:
    ReadCsv ro_;
    
    // Variables that control algorithm implementation
    unsigned int numb_of_iterations_;
    unsigned int min_numb_inliers_;
    double inlier_error_threshold_;
    const double kBigInitialError{10000};
    
    point GetDataPoint(const unsigned int& index);
    
};

std::ostream& operator<<(std::ostream &out, const Ransac &x);

#endif /* ransac_hpp */
