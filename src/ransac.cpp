//  ransac.cpp



#include <iostream>
#include <cmath>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "ransac.hpp"
#include "read_csv.hpp"

using namespace std;

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

Ransac::Ransac(const ReadCsv &ro, unsigned int min_inliers, double thres, unsigned int numb_iter)
: ro_(ro)
{
   SetNumbIterations(numb_iter);
   SetNumbInliers(min_inliers);
   SetInlierErrorThres(thres);    
}

Ransac::~Ransac()
{
    
}

bool Ransac::IterativeFit()
{

    for (int i = 1; i <= numb_of_iterations_; i++)
    {
        ComputeModel(ModelType::PROPOSED);

        if(CheckProposedModel())
        {
            ComputeModel(ModelType::BETTER);
            CompareBetterToBest();
        }
    }
    
    if (GetBestError()< kBigInitialError )
    {
        return true;
    }
    else
    {
        cout << " The fitting failed because a model having the minimum number of inliers with the specified error threshold could not be found." << endl;
        return false; 
    }
}

bool Ransac::ComputeModel(const ModelType& type)
{
    bool find_inliers;
 
    switch (type)
    {
        case ModelType::PROPOSED:
            find_inliers = true;
            CalculateParameters (ModelType::PROPOSED, GetPossibleInliers(ModelType::PROPOSED));
            return CalculateError (ModelType::PROPOSED, GetPossibleInliers(ModelType::PROPOSED), find_inliers);
        
        case ModelType::BETTER:
            find_inliers = false;
            CalculateParameters(ModelType::BETTER, GetPossibleInliers(ModelType::BETTER));
            return CalculateError(ModelType::BETTER, GetPossibleInliers(ModelType::BETTER), find_inliers);
        
        default: return false;
    }
}

bool Ransac::CheckProposedModel()
{
    if (GetNumbInliersProposed()> min_numb_inliers_) {return true;}
    else {return false;}
}

void Ransac::CompareBetterToBest()
{
    if (GetBetterError() < GetBestError()) {SaveBetterModelAsBest();}
}

void Ransac::SetNumbIterations(unsigned int numb_iter)
{
    numb_of_iterations_ = numb_iter;
}

void Ransac::SetNumbInliers (unsigned int min_numb_inliers)
{
    min_numb_inliers_ = min_numb_inliers;
    
}

void Ransac::SetInlierErrorThres (double inlier_error_thres)
{
    inlier_error_threshold_ = inlier_error_thres;
}

unsigned int Ransac::GetNumberIterations () const
{
    return numb_of_iterations_;
}

unsigned int Ransac::GetMinNumberInliers () const
{
    return min_numb_inliers_;
}

double Ransac::GetInlierErrorThreshold () const
{
    return inlier_error_threshold_;
}

std::ostream& operator<<(std::ostream &out, const Ransac &x)
{
    return x.print(out);
}

std::ostream& Ransac::print(std::ostream &out) const
{
    return out << "The inlier error threshold is " << inlier_error_threshold_ << "\n" << "The min numb of inliers is " << min_numb_inliers_ << "\n" << "The number of iterations is  " << numb_of_iterations_ << "\n";
}

point Ransac::GetDataPoint(const unsigned int& index)
{
    point temp {ro_.GetData()[index].get<0>(), ro_.GetData()[index].get<1>()};
    return temp;
}

