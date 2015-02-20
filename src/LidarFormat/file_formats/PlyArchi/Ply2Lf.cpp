
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "LidarFormat/LidarFile.h"
#include "Ply2Lf.h"

namespace Lidar
{

std::string Ply2Lf(std::string ply_type)
{
    if(ply_type == "char") return "int8";
    if(ply_type == "uchar") return "uint8";
    if(ply_type == "short") return "int16";
    if(ply_type == "ushort") return "uint16";
    if(ply_type == "int") return "int32";
    if(ply_type == "uint") return "uint32";
    if(ply_type == "float") return "float32";
    if(ply_type == "double") return "float64";
    return ply_type;
}

struct ply_attrib_info
{
    ply_attrib_info(std::string type_, std::string name_):
        type(type_), name(name_),
        bounds(false), min(0.), max(0.){}
    std::string type, name;
    bool bounds;
    double min, max;
};

void RobustGetLine(std::ifstream & ifs, std::string & line)
{
    std::getline(ifs, line);
    if(line[line.size()-1] == '\r')
        line.erase(line.size()-1);
}

std::string WritePlyXmlHeader(const std::string& ply_filename, bool debug)
{
    // load ply header
    std::ifstream ifs(ply_filename.c_str());
    if(!ifs.good())
    {
        std::cout << "Failed to open " << ply_filename << std::endl;
        return "";
    }
    std::string line;
    RobustGetLine(ifs, line);
    { // line == "ply" does not always work (weird char at end of line)
        std::istringstream iss(line);
        std::string word;
        iss >> word;
        if(word != "ply")
        {
            std::cout << "not a PLY file: starts with " << word << std::endl;
            return "";
        }
    }

    // attempt to create xml header
    bool need_absolute = false;
    boost::filesystem::path ply_filepath(ply_filename), plyxml_filepath(ply_filename);
    plyxml_filepath.replace_extension(".xml");
    std::ofstream ofs(plyxml_filepath.string().c_str());
    if(!ofs.good())
    {
        std::cout << "Failed to create " << plyxml_filepath << " -> attempting in cwd" << std::endl;
        need_absolute = true; // if xml is local, it is not with its data file so the data file path should be absolute
        plyxml_filepath = plyxml_filepath.filename(); // remove path
        ofs.open(plyxml_filepath.string().c_str());
        if(!ofs.good())
        {
            std::cout << "Failed to open " << plyxml_filepath << std::endl;
            return "";
        }
    }

    if(debug) std::cout << "Loading ply header from " << ply_filename << std::endl;

    int data_size = 0;
    double tx=0., ty=0., tz=0.;
    std::vector< ply_attrib_info > v_ply_attrib_info;
    std::string element="";
    int i_line=0;
    bool end_reached = false;
    while(!ifs.eof() && !end_reached && i_line++<100)
    {
        RobustGetLine(ifs, line);
        if(debug) std::cout << line;
        std::istringstream iss(line);
        std::string word;
        iss >> word;
        if(word == "format")
        {
            if(line != "format binary_little_endian 1.0")
                std::cout << "->only binary_little_endian 1.0 format supported, use at your own risks" << std::endl;
        }
        else if(word == "comment")
        {
            iss >> word;
            if(word == "IGN")
            {
                iss >> word;
                if(word == "offset" || word == "Offset")
                {
                    iss >> word;
                    if(word == "GPS")
                    {
                        if(debug) std::cout << "->GPS Offset not handled";
                    }
                    else if(word == "Pos")
                    {
                        iss >> tx >> ty >> tz;
                        if(debug) std::cout << "->tx=" << tx << ", ty=" << ty << ", tz=" << tz;
                    }
                    else if(debug) std::cout << "->unknown IGN Offset";
                }
                else if(word == "bounds")
                {
                    if(v_ply_attrib_info.empty()) if(debug) std::cout << "->Bounds without attrib, dropping";
                    double min, max;
                    iss >> min >> max;
                    if(debug) std::cout << "->min=" << min << ", max=" << max;
                    v_ply_attrib_info.back().min=min;
                    v_ply_attrib_info.back().max=max;
                    v_ply_attrib_info.back().bounds=true;
                }
                else if(word == "BBox" && debug) std::cout << "->Old BBox format, only new one supported";
                else if(debug) std::cout << "->unknown IGN comment";
            }
            else if(debug) std::cout << "->unknown comment";
        }
        else if(word == "element")
        {
            iss >> element;
            if(element != "vertex" && debug)
                std::cout << "->only vertex supported, the following will be ignored";
            else
            {
                iss >> data_size;
                if(debug) std::cout << "->data_size=" << data_size;
            }
        }
        else if(word == "property")
        {
            if(element == "vertex")
            {
                std::string type, name;
                iss >> type >> name;
                v_ply_attrib_info.push_back(ply_attrib_info(type, name));
            } else if(debug) std::cout << "->ignored";
        }
        else if(word == "end_header")
        {
            if(debug) std::cout << "->stopping";
            end_reached = true;
        }
        else
        {
            if(debug) std::cout << "->not recognized";
        }
        if(debug) std::cout << std::endl;
    }

    // write header
    std::cout << "Writing LF .xml header: " << plyxml_filepath << std::endl;
    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n";
    ofs << "<LidarData xmlns=\"cs\">\n";

    if(!need_absolute) ply_filepath = ply_filepath.filename();
    ofs << "  <Attributes DataFormat=\"plyarchi\" DataSize=\"" << data_size
        << "\" DataFileName=" << ply_filepath << ">\n"; // << path adds the quotes
    for(unsigned int i=0; i<v_ply_attrib_info.size(); i++)
    {
        ofs << "    <Attribute DataType=\"" << Ply2Lf(v_ply_attrib_info[i].type)
            << "\" Name=\"" << v_ply_attrib_info[i].name << "\"";
        if(v_ply_attrib_info[i].bounds)
            ofs << " min=\"" << v_ply_attrib_info[i].min <<
                   "\" max=\"" << v_ply_attrib_info[i].max << "\" ";
        ofs << "/>\n";
    }
    if(tx != 0. || ty != 0.)
        ofs << "    <CenteringTransfo tx=\"" << tx << "\" ty=\"" << ty << "\"/>\n";
    ofs << "  </Attributes>\n</LidarData>\n";
    return plyxml_filepath.string();
}

void ReadPly(const std::string& ply_filename,
             Lidar::LidarDataContainer& container,
             Lidar::LidarCenteringTransfo& transfo)
{
    std::string xml_filename = WritePlyXmlHeader(ply_filename);
    if(xml_filename == "") return;
    LidarFile file(xml_filename);
    file.loadData(container);
    file.loadTransfo(transfo);
}

void SavePly(const LidarDataContainer& container,
             const LidarCenteringTransfo& transfo,
             const std::string& ply_filename)
{
    std::ofstream fileOut(ply_filename.c_str());
    if(!fileOut.good())
    {
        std::cout << "Cannot open " + ply_filename + " for writing\n";
        return;
    }
    // write text header
    fileOut << "ply\nformat binary_little_endian 1.0" << std::endl;
    fileOut << "comment LidarFormat export" << std::endl;
    fileOut << "comment IGN offset Pos " << transfo.x() << " " << transfo.y() << " 0" << std::endl;
    fileOut << "element vertex " << container.size() << std::endl;
    std::vector<std::string> attrib_liste;
    container.getAttributeList(attrib_liste);
    for(std::vector<std::string>::iterator it = attrib_liste.begin(); it != attrib_liste.end();it++)
    {
        fileOut << "property " << OldName(container.getAttributeType(*it)) << " " << *it << std::endl;
        double min=0., max=0.;
        if(container.getAttributeBounds(*it, min, max))
            fileOut << "comment IGN bounds " << min << " " << max << std::endl;
    }
    fileOut << "end_header" << std::endl;
    fileOut.write(container.rawData(), container.size() * container.pointSize());
    fileOut.close();
    // WritePlyXmlHeader(ply_filename); // header should be written by LidarFile::Save()
}

void SavePly(const LidarDataContainer& container, const std::string& ply_filename)
{
    SavePly(container, LidarCenteringTransfo(), ply_filename);
}

} // namespace Lidar
