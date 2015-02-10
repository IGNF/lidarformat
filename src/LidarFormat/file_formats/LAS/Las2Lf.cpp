
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include "LidarFormat/LidarFile.h"
#include "Las2Lf.h"

namespace Lidar
{

inline std::string RemoveExtention(std::string path)
{
    int pointpos = path.find_last_of('.')+1;
    return path.substr(0, pointpos);
}

inline std::string GetFilename(std::string path)
{
    int slashpos = path.find_last_of('/')+1;
    return path.substr(slashpos, path.size()-slashpos);
}

std::string WriteLasXmlHeader(const std::string& las_filename, bool debug)
{
    // attempt to create xml header
    bool need_absolute = false;
    std::string xml_filename = RemoveExtention(las_filename) + "xml";
    std::ofstream ofs(xml_filename.c_str());
    if(!ofs.good())
    {
        std::cout << "Failed to create " << xml_filename << " -> attempting in cwd" << std::endl;
        need_absolute = true; // if xml is local, it is not with its data file so the data file path should be absolute
        xml_filename = GetFilename(xml_filename); // remove path
        ofs.open(xml_filename.c_str());
        if(!ofs.good())
        {
            std::cout << "Failed to open " << xml_filename << std::endl;
            return "";
        }
    }

    // write header
    std::cout << "Writing LF .xml header: " << xml_filename << std::endl;
    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n";
    ofs << "<LidarData xmlns=\"cs\">\n";

    int slashpos = las_filename.find_last_of('/')+1;
    std::string las_filename_to_write = las_filename;
    if(!need_absolute) las_filename_to_write = las_filename.substr(slashpos, las_filename.size()-slashpos);
    ofs << "  <Attributes DataFormat=\"las\" DataSize=\"0\" DataFileName=\"" << las_filename_to_write << "\">\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"x\"/>\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"y\"/>\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"z\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"intensity\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"classification\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"returnNumber\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"numberOfReturns\"/>\n";
    ofs << "  </Attributes>\n</LidarData>\n";
    return xml_filename;
}

void ReadLas(const std::string& las_filename,
             Lidar::LidarDataContainer& container,
             Lidar::LidarCenteringTransfo& transfo)
{
    std::string xml_filename = WriteLasXmlHeader(las_filename);
    if(xml_filename == "") return;
    LidarFile file(xml_filename);
    file.loadData(container);
    file.loadTransfo(transfo);
}

} // namespace Lidar
