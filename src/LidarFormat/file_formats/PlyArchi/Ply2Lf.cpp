
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "LidarFormat/LidarFile.h"
#include "Ply2Lf.h"

using namespace std;

namespace Lidar
{

string Ply2Lf(string ply_type)
{
    if(ply_type == "char") return "int8";
    if(ply_type == "uchar") return "uint8";
    if(ply_type == "short") return "int16";
    if(ply_type == "ushort") return "uint16";
    if(ply_type == "int") return "int32";
    if(ply_type == "uint") return "uint32";
    if(ply_type == "long") return "int64";
    if(ply_type == "ulong") return "uint64";
    if(ply_type == "float") return "float32";
    if(ply_type == "double") return "float64";
    return ply_type;
}

// ply supports both old typenames (char, float,...) and new typenames (int8, float32,...)
EnumLidarDataType Ply2LfEnum(string ply_type)
{
    if(ply_type == "char" || ply_type == "int8") return LidarDataType::int8;
    if(ply_type == "uchar" || ply_type == "uint8") return LidarDataType::uint8;
    if(ply_type == "short" || ply_type == "int16") return LidarDataType::int16;
    if(ply_type == "ushort" || ply_type == "uint16") return LidarDataType::uint16;
    if(ply_type == "int" || ply_type == "int32") return LidarDataType::int32;
    if(ply_type == "uint" || ply_type == "uint32") return LidarDataType::uint32;
    if(ply_type == "long" || ply_type == "int64") return LidarDataType::int64;
    if(ply_type == "ulong" || ply_type == "uint64") return LidarDataType::uint64;
    if(ply_type == "float" || ply_type == "float32") return LidarDataType::float32;
    if(ply_type == "double" || ply_type == "float64") return LidarDataType::float64;
    return LidarDataType::float32; // by default
}

void RobustGetLine(ifstream & ifs, string & line)
{
    getline(ifs, line);
    if(line[line.size()-1] == '\r')
        line.erase(line.size()-1);
}

boost::shared_ptr<cs::LidarDataType> PlyHeaderToLidarDataType(const string& ply_filename, bool debug)
{
    //debug=true;
    // load ply header
    ifstream ifs(ply_filename.c_str());
    if(!ifs.good()) throw logic_error("Failed to open " + ply_filename +"\n");
    string line;
    RobustGetLine(ifs, line);
    { // line == "ply" does not always work (weird char at end of line)
        istringstream iss(line);
        string word;
        iss >> word;
        if(word != "ply") throw logic_error("not a PLY file: starts with " + word +"\n");
    }

    if(debug) cout << "Loading ply header from " << ply_filename << endl;
    cs::LidarDataType::AttributesType attributes(0,cs::DataFormatType::plyarchi); // we do not know the data size yet
    attributes.dataFileName() = ply_filename;
    int i_line=0;
    bool end_reached = false;
    string element="";
    while(!ifs.eof() && !end_reached && i_line++<1000)
    {
        RobustGetLine(ifs, line);
        if(debug) cout << line;
        istringstream iss(line);
        string word="";
        iss >> word;
        if(word == "format")
        {
            if(line == "format binary_little_endian 1.0") attributes.dataFormat() = cs::DataFormatType::plyarchi;
            else if(line == "format ascii 1.0") attributes.dataFormat() = cs::DataFormatType::plyascii;
            else cout << "->only binary_little_endian and ascii 1.0 formats supported, use at your own risks" << endl;
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
                        if(debug) cout << "->GPS Offset not handled";
                    }
                    else if(word == "Pos")
                    {
                        double tx=0., ty=0., tz=0.;
                        iss >> tx >> ty >> tz;
                        if(debug) cout << "->tx=" << tx << ", ty=" << ty << ", tz=" << tz;
                        if(tx != 0. || ty != 0.) attributes.centeringTransfo(cs::CenteringTransfoType(tx, ty));
                    }
                    else if(debug) cout << "->unknown IGN Offset";
                }
                else if(word == "bounds")
                {
                    //if(v_ply_attrib_info.empty()) if(debug) cout << "->Bounds without attrib, dropping";
                    double min, max;
                    iss >> min >> max;
                    if(debug) cout << "->min=" << min << ", max=" << max;
                    // todo: min/max
                    //v_ply_attrib_info.back().min=min;
                    //v_ply_attrib_info.back().max=max;
                    //v_ply_attrib_info.back().bounds=true;
                }
                else if(word == "BBox" && debug) cout << "->Old BBox format, only new one supported";
                else if(debug) cout << "->unknown IGN comment";
            }
            else if(debug) cout << "->unknown comment";
        }
        else if(word == "element")
        {
            iss >> element;
            if(element != "vertex")
            {
                if(debug) cout << "->only vertex supported, the following will be ignored";
            }
            else
            {
                int data_size=0;
                iss >> data_size;
                if(debug) cout << "->data_size=" << data_size;
                attributes.dataSize() = data_size;
            }
        }
        else if(word == "property")
        {
            if(element == "vertex")
            {
                string type, name;
                iss >> type >> name;
                attributes.attribute().push_back(cs::AttributeContainerType::AttributeType(Ply2LfEnum(type), name));
            } else if(debug) cout << "->ignored";
        }
        else if(word == "end_header")
        {
            if(debug) cout << "->stopping";
            end_reached = true;
        }
        else
        {
            if(debug) cout << "->not recognized";
        }
        if(debug) cout << endl;
    }
    boost::shared_ptr<cs::LidarDataType> xmlStructure(new cs::LidarDataType(attributes));
    return xmlStructure;
}


string WritePlyXmlHeader(const string& ply_filename, bool debug)
{
    boost::shared_ptr<cs::LidarDataType> xmlStructure = PlyHeaderToLidarDataType(ply_filename, debug);
    boost::filesystem::path plyxml_filepath(ply_filename);
    plyxml_filepath.replace_extension(".xml");
    xml_schema::NamespaceInfomap map;
    map[""].name = "cs";
    std::ofstream ofs(plyxml_filepath.string().c_str());
    cs::lidarData(ofs, *xmlStructure, map);
    return plyxml_filepath.string();
}

void ReadPly(const string& ply_filename,
             Lidar::LidarDataContainer& ldc,
             Lidar::LidarCenteringTransfo& transfo)
{
    string xml_filename = WritePlyXmlHeader(ply_filename);
    if(xml_filename == "") return;
    LidarFile file(xml_filename);
    file.loadData(ldc);
    file.loadTransfo(transfo);
}

// assumes ldc is already structured
void LoadPlyAsciiData(const string& ply_filename,
                      Lidar::LidarDataContainer& ldc)
{
    using namespace std;
    ifstream ply_ifs(ply_filename.c_str());
    if(!ply_ifs.good()) throw std::logic_error(std::string(__FUNCTION__) + ": Failed to open " + ply_filename +"\n");
    string word="";
    while(!ply_ifs.eof() && word != "end_header") {ply_ifs >> word;}
    AttributeMapType attrib_map = ldc.getAttributeMap();
    for(LidarDataContainer::iterator it=ldc.begin(); it != ldc.end(); it++)
    {
        int old_decalage=-2, decalage=-1, ival=0;
        for(AttributeMapType::iterator it_att = attrib_map.begin(); it_att != attrib_map.end(); it_att++)
        {
            old_decalage = decalage;
            decalage = it_att->second.decalage;
            if(decalage <= old_decalage) cout << "ERROR: non increasing decalage" << endl;
            switch(it_att->second.dataType())
            {
            case Lidar::LidarDataType::float32: ; ply_ifs >> it.value<float32>(decalage); break;
            case Lidar::LidarDataType::float64: ply_ifs >> it.value<float64>(decalage); break;
            case Lidar::LidarDataType::int8: ply_ifs >> ival; it.value<int8>(decalage)=ival; break;
            case Lidar::LidarDataType::int16: ply_ifs >> ival; it.value<int16>(decalage)=ival; break;
            case Lidar::LidarDataType::int32: ply_ifs >> it.value<int32>(decalage); break;
            case Lidar::LidarDataType::int64: ply_ifs >> it.value<int64>(decalage); break;
            case Lidar::LidarDataType::uint8: ply_ifs >> ival; it.value<uint8>(decalage)=ival; break;
            case Lidar::LidarDataType::uint16: ply_ifs >> ival; it.value<uint16>(decalage)=ival; break;
            case Lidar::LidarDataType::uint32: ply_ifs >> it.value<uint32>(decalage); break;
            case Lidar::LidarDataType::uint64: ply_ifs >> it.value<uint64>(decalage); break;
            default: cout << "Unknown data type " << it_att->second.dataType() << endl;
            }
        }
    }
}

void SavePly(const LidarDataContainer& ldc,
             const LidarCenteringTransfo& transfo,
             const string& ply_filename,
             bool binary)
{
    ofstream fileOut(ply_filename.c_str());
    if(!fileOut.good())
    {
        cout << "Cannot open " + ply_filename + " for writing\n";
        return;
    }
    // write text header
    fileOut << "ply\nformat ";
    if(binary) fileOut << "binary_little_endian";
    else fileOut << "ascii";
    fileOut << " 1.0\ncomment LidarFormat export" << endl;
    fileOut << "comment IGN offset Pos " << transfo.x() << " " << transfo.y() << " 0" << endl;
    fileOut << "element vertex " << ldc.size() << endl;
    vector<string> attrib_liste;
    ldc.getAttributeList(attrib_liste);
    for(vector<string>::iterator it = attrib_liste.begin(); it != attrib_liste.end();it++)
    {
        fileOut << "property " << Name(ldc.getAttributeType(*it)) << " " << *it << endl;
        double min=0., max=0.;
        if(ldc.getAttributeBounds(*it, min, max))
            fileOut << "comment IGN bounds " << min << " " << max << endl;
    }
    fileOut << "end_header" << endl;
    if(binary) fileOut.write(ldc.rawData(), ldc.size() * ldc.pointSize());
    else
    {
        AttributeMapType attrib_map = ldc.getAttributeMap();
        for(LidarDataContainer::const_iterator it = ldc.begin(); it != ldc.end(); it++)
        {
            for(AttributeMapType::iterator it_att = attrib_map.begin(); it_att != attrib_map.end(); it_att++)
            {
                if(it_att != attrib_map.begin()) fileOut << " ";
                int decalage = it_att->second.decalage;
                switch(it_att->second.dataType())
                {
                case Lidar::LidarDataType::float32: ; fileOut.precision(9); fileOut << it.value<float32>(decalage); break;
                case Lidar::LidarDataType::float64: fileOut.precision(15); fileOut << it.value<float64>(decalage); break;
                case Lidar::LidarDataType::int8: fileOut << (int)it.value<int8>(decalage); break;
                case Lidar::LidarDataType::int16: fileOut << (int)it.value<int16>(decalage); break;
                case Lidar::LidarDataType::int32: fileOut << it.value<int32>(decalage); break;
                case Lidar::LidarDataType::int64: fileOut << it.value<int64>(decalage); break;
                case Lidar::LidarDataType::uint8: fileOut << (int)it.value<uint8>(decalage); break;
                case Lidar::LidarDataType::uint16: fileOut << (int)it.value<uint16>(decalage); break;
                case Lidar::LidarDataType::uint32: fileOut << it.value<uint32>(decalage); break;
                case Lidar::LidarDataType::uint64: fileOut << it.value<uint64>(decalage); break;
                default: cout << "Unknown data type " << it_att->second.dataType() << endl;
                }
            }
            fileOut << endl;
        }
    }
    fileOut.close();
}

void SavePly(const LidarDataContainer& ldc, const string& ply_filename, bool binary)
{
    double x=0.,y=0.; ldc.getCenteringTransfo(x,y);
    SavePly(ldc, LidarCenteringTransfo(x,y), ply_filename, binary);
}

} // namespace Lidar
