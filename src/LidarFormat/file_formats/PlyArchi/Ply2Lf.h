/*
 * *
 * * Created on: 29 nov. 2013
 * * Author: Bruno
 * */

#ifndef PLY2LF_H_
#define PLY2LF_H_

#include <string>
#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/geometry/LidarCenteringTransfo.h"

namespace Lidar
{

// conversion from ply to lf format types
std::string Ply2Lf(std::string ply_type);

// conversion from lf to ply format types
std::string Lf2Ply(EnumLidarDataType lf_type);

// write the xml header corresponding to the ply file
// returns the name of the header
std::string WritePlyXmlHeader(const std::string& ply_filename, bool debug=false);

// directly read a ply file in lf by generating a lf .xml header (cf function above) then reading it
void ReadPly(const std::string& ply_filename,
             LidarDataContainer& container,
             LidarCenteringTransfo& transfo);

// save the container and centering as a ply file
void SavePly(const LidarDataContainer& container,
             const LidarCenteringTransfo& transfo,
             const std::string& ply_filename);

// save the container as a ply file
void SavePly(const LidarDataContainer& container,
             const std::string& ply_filename);

} // namespace Lidar

#endif // PLY2LF_H_
