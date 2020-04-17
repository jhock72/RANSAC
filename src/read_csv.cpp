//  read_csv.cpp



#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

#include "read_csv.hpp"

using namespace std;
typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

// Initialize the filename
ReadCsv::ReadCsv(const string &file_name)
: file_name_(file_name)
{
    switch(IngestData())
    {
        case FailReason::FAIL:
            if(RequestCorrectedFileName())
            {
                if(GetFileName()) IngestData();
            }else exit(1);                          // exit if the user doesnt want to renter the file name
            break;
        case FailReason::BAD:
            exit(1);
            break;
        case FailReason::EOFF:
            cout << "The file: " << file_name_ << " was empty.\n";
            if(RequestCorrectedFileName())
            {
                if(GetFileName()) IngestData();
            }else exit(1);                          // exit if the user doesnt want to renter the file name
            break;
        case FailReason::NONE:
        default:
            break;
    }
}

ReadCsv::~ReadCsv()
{
    
}

std::vector<point> ReadCsv::GetData () const
{
    return data_points_;
}

bool ReadCsv::RequestCorrectedFileName()
{
    char answer;
    cout << "\n Would you like to re-enter the filename?(y/n)";
    
    while (!(cin >> answer) || !CheckAnswer(answer))
    {
        if(!cin.good()) cout << "Invalid Entry.\n";
        cin.clear(std::ios_base::goodbit);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "\n Would you like to re-enter the filename?(y/n)";
    }
    
    if (answer == 'y') return true;
    else return false;
}

bool ReadCsv::CheckAnswer (const char& ans)
{
 switch (ans)
    {
    case 'y':
         return true;
    case 'n':
         return true;
    default:
         return false;
    }
}

// Opens the file provided in the constructor and reads the data into a vector
ReadCsv::FailReason ReadCsv::IngestData()
{
    ifstream csv_file (file_name_, ios::in);
    
    if (!csv_file)
    {
        cerr << "\n Couldn't open file: " << file_name_ << '\n';
        
        if(csv_file.fail()) return FailReason::FAIL;
        if(csv_file.bad()) return FailReason::BAD;
    }
    
    point j;
    string x;
    size_t comma_pos;
    bool check_line;
    int count{0};
    
    
    csv_file >> x;
    // Read each line of the file until the eof is encountered
    while (!csv_file.eof())
    {
         
         check_line = x.find_first_of(kDigits_)==string::npos;
         if (!check_line)
         {
            count +=1;
            j.set<0>(stof(x, &comma_pos));
            j.set<1>(stof(x.substr(comma_pos+1)));
            data_points_.push_back(j);
         }
        csv_file >>x;
    }
    if(csv_file.fail()) return FailReason::NONE;
    else if (count ==0 ) return FailReason::EOFF;
    else if (csv_file.bad()) return FailReason::BAD;
    else return FailReason::NONE;
}

bool ReadCsv::GetFileName()
{
    string filename;
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    cout << "Please enter a new filename: ";
    
    while (!(cin>> filename))
    {
        
        if(!cin.good()) cout << "Invalid Entry.\n";
        cin.clear(std::ios_base::goodbit);
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "\n Would you like to re-enter the filename?(y/n)";
    }
    if (cin.good())
    {
        file_name_ = filename;
        return true;
    }
    else return false;
}
