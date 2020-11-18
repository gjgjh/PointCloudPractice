#ifndef UTILS_H
#define UTILS_H

#include "common.h"

PtCloud::Ptr readVelodyneBin(const std::string &filepath);

void visualize(const PtCloud::Ptr &ptCloud, double pointSize = 1);

#endif // UTILS_H