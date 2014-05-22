
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
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

std::string Lf2Ply(EnumLidarDataType lf_type)
{
    if(lf_type == LidarDataType::int8) return "char";
    if(lf_type == LidarDataType::uint8) return "uchar";
    if(lf_type == LidarDataType::int16) return "short";
    if(lf_type == LidarDataType::uint16) return "ushort";
    if(lf_type == LidarDataType::int32) return "int";
    if(lf_type == LidarDataType::uint32) return "uint";
    if(lf_type == LidarDataType::int64) return "long";
    if(lf_type == LidarDataType::uint64) return "ulong";
    if(lf_type == LidarDataType::float32) return "float";
    if(lf_type == LidarDataType::float64) return "double";
    return "unknown";
}

std::string RemoveExtention(std::string path)
{
    int pointpos = path.find_last_of('.')+1;
    return path.substr(0, pointpos);
}

std::string GetFilename(std::string path)
{
    int slashpos = path.find_last_of('/')+1;
    return path.substr(slashpos, path.size()-slashpos);
}

std::string WriteXmlHeader(const std::string& ply_filename)
{
    // load ply header
    std::ifstream ifs(ply_filename.c_str());
//    if(!ifs.good())
//    {
//        std::cout << "Failed to open " << ply_filename << std::endl;
//        return "";
//    }
    std::string line;
    std::getline(ifs, line);
    if(line != "ply")
    {
        std::cout << "not a PLY file: starts with " << line << std::endl;
        return "";
    }

    // attempt to create xml header
    bool need_absolute = false;
    std::string xml_filename = RemoveExtention(ply_filename) + "xml";
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

    std::cout << "Loading ply header from " << ply_filename << std::endl;

    int data_size = 0;
    double tx=0., ty=0., tz=0.;
    std::vector< std::pair<std::string, std::string> > v_type_name;
    std::string element="";
    while(!ifs.eof() && line != "end_header")
    {
        std::getline(ifs, line);
        std::cout << line;
        std::istringstream iss(line);
        std::string word;
        iss >> word;
        if(word == "format")
        {
            if(line != "format binary_little_endian 1.0")
                std::cout << "->only binary_little_endian 1.0 format supported, use at your own risks";
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
                        std::cout << "->GPS Offset not handled";
                    }
                    else if(word == "Pos")
                    {
                        iss >> tx >> ty >> tz;
                        std::cout << "->tx=" << tx << ", ty=" << ty << ", tz=" << tz;
                    }
                    else std::cout << "->unknown IGN Offset";
                }
                else std::cout << "->unknown IGN comment";
            }
            else std::cout << "->unknown comment";
        }
        else if(word == "element")
        {
            iss >> element;
            if(element != "vertex")
                std::cout << "->only vertex supported, the following will be ignored";
            else
            {
                iss >> data_size;
                std::cout << "->data_size=" << data_size;
            }
        }
        else if(word == "property")
        {
            if(element == "vertex")
            {
                std::string type, name;
                iss >> type >> name;
                v_type_name.push_back(std::pair<std::string, std::string>(type, name));
            } else std::cout << "->ignored";
        }
        else if(word != "end_header")
        {
            std::cout << "->not recognized";
        }
        std::cout << std::endl;
    }

    // write header
    std::cout << "Writing LF .xml header: " << xml_filename << std::endl;
    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n";
    ofs << "<LidarData xmlns=\"cs\">\n";

    int slashpos = ply_filename.find_last_of('/')+1;
    std::string ply_filename_to_write = ply_filename;
    if(!need_absolute) ply_filename_to_write = ply_filename.substr(slashpos, ply_filename.size()-slashpos);
    ofs << "  <Attributes DataFormat=\"plyarchi\" DataSize=\"" << data_size
        << "\" DataFileName=\"" << ply_filename_to_write << "\">\n";
    for(unsigned int i=0; i<v_type_name.size(); i++)
    {
        ofs << "    <Attribute DataType=\"" << Ply2Lf(v_type_name[i].first)
            << "\" Name=\"" << v_type_name[i].second << "\"/>\n";
    }
    if(tx != 0. || ty != 0.)
        ofs << "    <CenteringTransfo tx=\"" << tx << "\" ty=\"" << ty << "\"/>\n";
    ofs << "  </Attributes>\n</LidarData>\n";
    return xml_filename;
}

void ReadPly(const std::string& ply_filename,
             Lidar::LidarDataContainer& container,
             Lidar::LidarCenteringTransfo& transfo)
{
    std::string xml_filename = WriteXmlHeader(ply_filename);
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
        fileOut << "property " << Lf2Ply(container.getAttributeType(*it)) << " " << *it << std::endl;
    }
    fileOut << "end_header" << std::endl;
    fileOut.write(container.rawData(), container.size() * container.pointSize());
}

void SavePly(const LidarDataContainer& container, const std::string& ply_filename)
{
    SavePly(container, LidarCenteringTransfo(), ply_filename);
}

} // namespace Lidar
