//  main.cpp



#include <iostream>
#include <string>

#include "read_csv.hpp"
#include "linear_ransac.hpp"
#include "circular_ransac.hpp"

using namespace std;

int main(int argc, const char * argv[]) {
    
    
//  LinearRansac
    unsigned int min_inliers = 25;
    double thres = .80;
    unsigned int numb_iter = 600;
    
    LinearRansac lo {ReadCsv ("../data/point_data_linear.csv"), min_inliers, thres, numb_iter};
    Ransac *ro = &lo;
    if(ro->IterativeFit())
    {
        cout << lo << endl;
    };
    
    
//  Circular Ransac 
    unsigned int min_inliers_1 = 15;
    double thres_1 = .85;
    unsigned int numb_iter_1 = 250;
    
    CircularRansac co {ReadCsv("../data/point_data_circular.csv"), min_inliers_1, thres_1, numb_iter_1};
    Ransac *r1 = &co;
    if(r1->IterativeFit())
    {
        cout << co << endl;
    };
    
     
    return 0;
}
