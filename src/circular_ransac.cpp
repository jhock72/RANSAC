//  circular_ransac.cpp



#include <iostream>
#include <cmath>
#include <array>
#include <vector>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "ransac.hpp"
#include "circular_ransac.hpp"


namespace bg = boost::geometry;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;

using namespace std;

CircularRansac::CircularRansac (const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter)
: Ransac(ro, min_inliers, thres, numb_iter), rn_engine_(static_cast<unsigned int>(time(0))), data_points_copy_(ro_.GetData())
{
    SetBestError(kBigInitialError);
}

std::ostream& CircularRansac::print(std::ostream &out) const
{
    return out << "The center points is: " << "(" <<best_.center.get<0>() << ","<< best_.center.get<1>() << ")" << "\n" << "The radius is: " << best_.radius << "\n" << "The fitness of the best model is: " << best_.total_error << "\n" << "The number of inliers for the best model is : " << best_.numb_inliers << "\n" ;
}

std::vector<point>& CircularRansac::GetPossibleInliers (const ModelType& type)
{
    if(type == ModelType::PROPOSED)
        return data_points_copy_;
    else if (type == ModelType::BETTER)
        return proposed_.inliers;
    else 
        return better_.inliers;
}

void CircularRansac::CalculateParameters (const ModelType& type, const std::vector<point>& data_to_fit)
{
    ModelInfo* model{nullptr};
    unsigned int max{static_cast<unsigned int> (data_to_fit.size())};
    
    if (type == ModelType::PROPOSED)
    {
        model=&proposed_;
        model->inliers.clear();
    }
    else if (type == ModelType::BETTER)
    {
        model= &better_;
        model->inliers.clear();
    }
        
     std::array<unsigned int, 3> index {{0,0,0}};
     uniform_int_distribution <unsigned int> random_int (1,static_cast<unsigned int>(max-1));
     
     int temp;
     for (int i{0}; i<kNumbDataPointsForCircularModel; i++ )
     {
         temp = random_int(rn_engine_);
         while (temp == index[0] || temp == index[kNumbDataPointsForCircularModel-2] || temp == index[kNumbDataPointsForCircularModel-1])
         {
             temp = random_int(rn_engine_);
         }
         index[i] = temp;
     }
    
     model->inliers.push_back(data_to_fit[index[0]]);
     model->inliers.push_back(data_to_fit[index[1]]);
     model->inliers.push_back(data_to_fit[index[2]]);
     
     auto P1 = model->inliers[0];
     auto P2 = model->inliers[1];
     auto P3 = model->inliers[2];
  
     bg::model::polygon<point> poly{{P1,P2,P3}};
     bg::centroid(poly, model->center);
     model->radius = bg::distance(P3, model->center);
}

bool CircularRansac::CalculateError (const ModelType type, const std::vector<point>& data_to_fit, bool find_inliers)
{
        ModelInfo* model{nullptr};
       
        if (type == ModelType::PROPOSED) model = &proposed_;
        else if (type == ModelType::BETTER)  model = &better_;
        else return false;
    
        bg::strategy::buffer::side_straight side_strat;
        bg::strategy::buffer::join_round join_strat(kPointsPerCircle);
        bg::strategy::buffer::end_round end_strat(kPointsPerCircle);
        bg::strategy::buffer::point_circle point_strat(kPointsPerCircle);
        bg::strategy::buffer::distance_symmetric<double> dist_strat{model->radius};
        bg::model::multi_polygon<bg::model::polygon<point>> result_circle;
        bg::buffer(model->center, result_circle, dist_strat, side_strat, join_strat, end_strat, point_strat);
        
        double error;
        
        if (find_inliers == true)
        {
            model->inliers.clear();
            model->inlier_error.clear();
        }
        else if (find_inliers == false)
        {
            model->inliers.clear();
            model->inliers = proposed_.inliers;
            model->inlier_error.clear();
        }
        
        for (auto i:data_to_fit)
          {
              if(bg::within(i,result_circle[0]))
              {
                  error = model->radius- bg::distance(i, model->center);
              }
              else
              {
                  error = bg::distance(result_circle[0], i);
              }
              
              if (error <= inlier_error_threshold_)
              {
                  if (find_inliers == true){
                    model->inliers.push_back({i.get<0>(),i.get<1>()});}
                  model->inlier_error.push_back({error});
              }
          }
          model->numb_inliers = static_cast<unsigned int> (model->inliers.size());
          for (auto i:model->inlier_error)
          {
              model->total_error += i;
          }
         
          return true;
}

void CircularRansac::SaveBetterModelAsBest()
{
    best_= better_;
}

double CircularRansac::GetBestError() const
{
    return best_.total_error;
}

double CircularRansac::GetBetterError() const
{
    return better_.total_error;
}

unsigned int CircularRansac::GetNumbInliersProposed() const
{
    return proposed_.numb_inliers;
}

void CircularRansac::SetBestError(double err)
{
    best_.total_error = err;
}
