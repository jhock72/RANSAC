//  read_csv.hpp



#ifndef read_csv_hpp
#define read_csv_hpp

#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>

typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point;

class ReadCsv
{
public:
    explicit ReadCsv (const std::string &);
    ~ReadCsv();
    std::vector<point> GetData () const;
    
private:
    std::string file_name_;
    const std::string kDigits_{"1234567890"};
    std::vector<point> data_points_;
    
    bool RequestCorrectedFileName();
    bool CheckAnswer(const char&);
    bool GetFileName();
    
    enum class FailReason
    {
        NONE,
        BAD,
        FAIL,
        EOFF,
    };
    
    FailReason IngestData();
};

#endif /* read_csv_hpp */
