
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "LidarFormat/LidarFile.h"
#include "Las2Lf.h"

namespace Lidar
{

std::string WriteLasXmlHeader(const std::string& las_filename, bool debug)
{
    // attempt to create xml header
    bool need_absolute = false;
    boost::filesystem::path las_filepath(las_filename), lasxml_filepath(las_filename);
    lasxml_filepath.replace_extension(".xml");
    std::ofstream ofs(lasxml_filepath.string().c_str());
    if(!ofs.good())
    {
        std::cout << "Failed to create " << lasxml_filepath << " -> attempting in cwd" << std::endl;
        need_absolute = true; // if xml is local, it is not with its data file so the data file path should be absolute
        lasxml_filepath = lasxml_filepath.filename(); // remove path
        ofs.open(lasxml_filepath.string().c_str());
        if(!ofs.good())
        {
            std::cout << "Failed to open " << lasxml_filepath << std::endl;
            return "";
        }
    }

    // write header
    std::cout << "Writing LF .xml header for a LAS file: " << lasxml_filepath << std::endl;
    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n";
    ofs << "<LidarData xmlns=\"cs\">\n";

    if(!need_absolute) las_filepath = las_filepath.filename();
    ofs << "  <Attributes DataFormat=\"las\" DataSize=\"0\" DataFileName=" << las_filepath << ">\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"x\"/>\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"y\"/>\n";
    ofs << "    <Attribute DataType=\"float64\" Name=\"z\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"intensity\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"classification\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"returnNumber\"/>\n";
    ofs << "    <Attribute DataType=\"uint32\" Name=\"numberOfReturns\"/>\n";
    ofs << "  </Attributes>\n</LidarData>\n";
    return lasxml_filepath.string();
}

void ReadLas(const std::string& las_filename,
             Lidar::LidarDataContainer& container,
             Lidar::LidarCenteringTransfo& transfo)
{
    std::string xml_filename = WriteLasXmlHeader(las_filename);
    if(xml_filename == "") return;
    container.load(xml_filename);
}

} // namespace Lidar
