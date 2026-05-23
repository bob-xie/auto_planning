#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include "common/Path.h"

namespace AD {
namespace Planning {
namespace Smooth {

class PathSmoother {
public:
    virtual ~PathSmoother() = default;
    
    virtual bool init(double param1 = 0.0, double param2 = 0.0) = 0;
    
    virtual Path smooth(const Path& input_path) = 0;
    
    virtual std::string getName() const = 0;
};

}
}
}

#endif