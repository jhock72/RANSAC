//  linear_ransac.cpp



#include <iostream>
#include <cmath>
#include <vector>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "linear_ransac.hpp"
#include "read_csv.hpp"
#include "ransac.hpp"


typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

using namespace std;

LinearRansac::LinearRansac (const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter)
: Ransac(ro, min_inliers, thres, numb_iter), rn_engine_(static_cast<unsigned int>(time(0))), best_({0}), better_({0}), data_points_copy_(ro.GetData())
{
    SetBestError(kBigInitialError);
}

LinearRansac::~LinearRansac()
{
    
}

std::vector<point>& LinearRansac::GetPossibleInliers (const ModelType& type)
{
    if(type == ModelType::PROPOSED)
        return data_points_copy_;
    else if (type == ModelType::BETTER)
        return proposed_.inliers;
    else
        return best_.inliers;
}

void LinearRansac::CalculateParameters (const ModelType& type, const std::vector<point>& data_to_fit)
{
    ModelInfo* model{nullptr};
    unsigned int max{static_cast<unsigned int> (data_to_fit.size())};
    uniform_int_distribution <unsigned int> random_int (1,static_cast<unsigned int> (max));
    
    if (type == ModelType::PROPOSED) model = &proposed_;
    else if (type == ModelType::BETTER) model = &better_;

    std::array<unsigned int, 2> index {{0,0}};
    int temp;
    for (int i{0}; i<kNumbDataPointsForLinearModel; i++)
    {
        temp = random_int(rn_engine_);
        while (temp == index[0] || temp == index[kNumbDataPointsForLinearModel-1])
        {
            temp = random_int(rn_engine_);
        }
        index[i] = temp;
    }
    model->inliers.push_back(data_to_fit[index[0]]);
    model->inliers.push_back(data_to_fit[index[1]]);
    
    auto P1 = model->inliers[0];
    auto P2 = model->inliers[1];
        
    // Determines the model parameters using equation of a line
    model->slope = (P2.get<1>() - P1.get<1>())/(P2.get<0>() - P1.get<0>());
    model->y_int = P2.get<1>() - model->slope*P2.get<0>();
}

bool LinearRansac::CalculateError (const ModelType type, const std::vector<point>& data_to_fit, bool find_inliers)
{
    ModelInfo* model;
    
    if (type == ModelType::PROPOSED) model = &proposed_;
    else if (type == ModelType::BETTER) model = &better_;
    else return false;
        
    double error;
    const int x{0},y{1};
    auto P1 = model->inliers[0];
    auto P2 = model->inliers[1];
    boost::geometry::model::polygon<point> triangle;
    
    if (find_inliers == true)
    {
        model->inliers.clear();
        model->inlier_errors.clear();
    }
    else if (find_inliers == false)
    {
        model->inliers.clear();
        model->inliers = proposed_.inliers;
        model->inlier_errors.clear();
    }
    
    for (auto i:data_to_fit)
    {
        triangle.clear();
        boost::geometry::append(triangle.outer(), P1);
        boost::geometry::append(triangle.outer(), P2);
        boost::geometry::append(triangle.outer(), i);
        boost::geometry::append(triangle.outer(), P1);
        
        error = abs(2*boost::geometry::area(triangle))/boost::geometry::distance(P1,P2);

        if (error <= inlier_error_threshold_)
        {
            if (find_inliers == true) model->inliers.push_back({i.get<x>(),i.get<y>()});
            model->inlier_errors.push_back({error});
        }
    }

    model->numb_inliers = static_cast<unsigned int> (model->inliers.size());
     for (auto i:model->inlier_errors)
     {
         model->total_error += i;
     }
    
    return true;
}

void LinearRansac::SaveBetterModelAsBest()
{
    best_ = better_;
}

std::ostream& LinearRansac::print(std::ostream &out) const
{
    
    return out << "The slope for the best model is: " << best_.slope << "\n" << "The y_int for the best model is: " << best_.y_int << "\n" << "The fitness of the best model is: " << best_.total_error << "\n" << "The number of inliers for the best model is : " << best_.numb_inliers << "\n" ;
}

double LinearRansac::GetBestError() const
{
    return best_.total_error;
}

double LinearRansac::GetBetterError() const
{
    return better_.total_error;
}

unsigned int LinearRansac::GetNumbInliersProposed() const
{
    return proposed_.numb_inliers;
}

void LinearRansac::SetBestError(double err)
{
    best_.total_error = err;
}
