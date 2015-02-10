/*
 * *
 * * Created on: 09/02/2015
 * * Author: Bruno Vallet
 * */

#ifndef LAS2LF_H_
#define LAS2LF_H_

#include <string>
#include "LidarFormat/LidarDataContainer.h"

namespace Lidar
{
// write the xml header corresponding to the las file
// returns the name of the header
std::string WriteLasXmlHeader(const std::string& las_filename, bool debug=false);

// directly read a ply file in lf by generating a lf .xml header (cf function above) then reading it
void ReadLas(const std::string& ply_filename,
             LidarDataContainer& container,
             LidarCenteringTransfo& transfo);

// save the container and centering as a ply file
//void SaveLas(const LidarDataContainer& container,
//             const LidarCenteringTransfo& transfo,
//             const std::string& ply_filename);

// save the container as a ply file
//void SaveLas(const LidarDataContainer& container,
//             const std::string& ply_filename);

} // namespace Lidar

#endif // PLY2LF_H_
