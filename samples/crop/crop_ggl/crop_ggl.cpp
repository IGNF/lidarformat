/*!
 * \file crop_ggl.cpp
 * \brief
 * \author Nicolas DAVID
 * \date 23 juillet. 2009
 */

#include <iostream>
#include <iterator>
#include <algorithm>
#include <map>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include "ggl/ggl.hpp"
#include "ggl/geometries/cartesian2d.hpp"
#include "ggl/io/wkt/read_wkt.hpp"
#include "ggl/io/wkt/stream_wkt.hpp"



#include "LidarFormat/LidarDataContainer.h"
#include "LidarFormat/LidarFile.h"
#include "LidarFormat/LidarIOFactory.h"
#include "LidarEcho_ggl_adapter.hpp"

using namespace Lidar;
using namespace std;
using namespace ggl;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef std::map< std::string,  polygon_2d> crop_region;

// A helper function to simplify the main part.
template<class T>
ostream& operator<<(ostream& os, const vector<T>& v)
{
    copy(v.begin(), v.end(), ostream_iterator<T>(cout, " "));
    return os;
}

int read_crop_region(std::string filename, crop_region& crop)
{
	std::cout<<" ***** initialisation des regions de crop **** "<<std::endl;
	std::cout<<" \t fichier : "<<filename<<std::endl;
	boost::filesystem::path crop_file_path(filename);
	std::string crop_extension = extension(crop_file_path);
	//std::cout<<" crop_extension "<<crop_extension;
	if(!(crop_extension==".txt"))
	{
		std::cout<<" mauvaise extension de fichier de crop, doit être un fichier *.txt "<<std::endl;
		return 1;
	}
	if ( !fs::exists( crop_file_path ) )
	{
	    std::cout << "\nNot found: " << crop_file_path.file_string() << std::endl;
	    return 1;
	}
	std::ifstream fileIn(filename.c_str());
	if(!fileIn.good())
	{
		std::cout<<"Erreur au chargement du fichier de crop le fichier n'existe pas ou n'est pas accessible en lecture ! \n";
		return 1;
	}
	std::string tmp_buf, name_tmp, wkt_string;
	bool first_crop=true;
	while(fileIn.good())
	{
		fileIn>>tmp_buf;
		if(tmp_buf=="NAME" && !first_crop)
		{
			polygon_2d poly_crop;
			ggl::read_wkt(wkt_string, poly_crop);
			crop.insert( pair<std::string, polygon_2d >(name_tmp,poly_crop));
			wkt_string.clear();
			name_tmp.clear();
			fileIn>>name_tmp;
		}
		if(tmp_buf=="END" )
		{
				polygon_2d poly_crop;
				ggl::read_wkt(wkt_string, poly_crop);
				crop.insert( pair<std::string, polygon_2d >(name_tmp,poly_crop));
				wkt_string.clear();
				name_tmp.clear();
				break;
		}
		if(tmp_buf=="NAME" && first_crop)
		{
			fileIn>>name_tmp;
			first_crop=false;
		}
		if(!(tmp_buf=="NAME") && !(tmp_buf=="END"))
		{
			if(wkt_string.empty()){
				wkt_string.append(tmp_buf);
			}
			else{
				wkt_string.append(" ");
				wkt_string.append(tmp_buf);
			}
		}
	}
	typedef crop_region::iterator crop_ite_type;
	crop_ite_type crop_ite=crop.begin();
	for(crop_ite; crop_ite!=crop.end(); crop_ite++)
	{
		std::cout<<"\t region de crop : "<<(*crop_ite).first<<std::endl;
		std::cout<<"\t\t"<<(*crop_ite).second<<std::endl;
	}
	std::cout<<" ********************************************** "<<std::endl;
	return 0;
}

boost::shared_ptr<LidarDataContainer> LoadInputData(std::string input_filename)
{
	boost::filesystem::path input_file_path(input_filename);
	std::string input_extension = extension(input_file_path);
	if(!(input_extension==".xml"))
	{
		std::cout<<" mauvaise extension de fichier lidar, doit être un fichierde metadonnée *.xml "<<std::endl;
		return boost::shared_ptr<LidarDataContainer>();
	}
	if ( !fs::exists( input_file_path ) )
	{
		    std::cout << "\nNot found: " << input_file_path.file_string() << std::endl;
		    return boost::shared_ptr<LidarDataContainer>();
	}
	std::cout<<" on initialize un LidarFile "<<input_filename<<std::endl;
	LidarFile file(input_filename);
	boost::shared_ptr<LidarDataContainer> lidarData;
	lidarData = boost::shared_ptr<LidarDataContainer> (new LidarDataContainer());
	file.loadData(*lidarData);
	std::cout << "\n Header : " << std::endl;
	lidarData->printHeader(std::cout);
	return lidarData;
}

boost::shared_ptr<LidarDataContainer> InitOutputData(std::string input_filename, std::string& output_format)
{
	boost::filesystem::path input_file_path(input_filename);
	std::string input_extension = extension(input_file_path);
	if(!(input_extension==".xml"))
	{
		std::cout<<" mauvaise extension de fichier lidar, doit être un fichierde metadonnée *.xml "<<std::endl;
		return boost::shared_ptr<LidarDataContainer>();
	}
	if ( !fs::exists( input_file_path ) )
	{
		    std::cout << "\nNot found: " << input_file_path.file_string() << std::endl;
		    return boost::shared_ptr<LidarDataContainer>();
	}
	std::cout<<" on initialize un LidarFile "<<input_filename<<std::endl;
	LidarFile file(input_filename);
	output_format=file.getFormat();
	boost::shared_ptr<LidarDataContainer> lidarData;
	lidarData = boost::shared_ptr<LidarDataContainer> (new LidarDataContainer());
	file.loadData(*lidarData);
	std::cout << "\n Header : " << std::endl;
	lidarData->printHeader(std::cout);
	lidarData->clear();
	return lidarData;
}

void add_crop(polygon_2d poly, boost::shared_ptr<LidarDataContainer> m_lidarInput,boost::shared_ptr<LidarDataContainer> m_lidarOutput )
{
	std::cout<<" ajout des points "<<std::endl;
	AttributeMapType att_input_map= m_lidarInput->getAttributeMap();
	if( !( (att_input_map.find("x"))!= att_input_map.end()) )
	{
		std::cout<<" pas d'attribut x trouve...arrêt du crop sur le fichier input courant "<<std::endl;
		return;
	}
	if( !( (att_input_map.find("y"))!= att_input_map.end()) )
	{
			std::cout<<" pas d'attribut y trouve...arrêt du crop sur le fichier input courant "<<std::endl;
			return;
	}
	if( !( (att_input_map.find("z"))!= att_input_map.end()) )
	{
			std::cout<<" pas d'attribut z trouve...arrêt du crop sur le fichier input courant "<<std::endl;
			return;
	}
	EnumLidarDataType coord_type=m_lidarInput->getAttributeType("x");
	if( coord_type == LidarDataType::float32 )
	{
		std::cout<<" coord type float"<<std::endl;
		add_crop<float>(poly,m_lidarInput,m_lidarOutput);
	}
	if( coord_type == LidarDataType::float64 )
	{
		std::cout<<" coord type double"<<std::endl;
		add_crop<double>(poly,m_lidarInput,m_lidarOutput);
	}

}

int main(int ac, char* av[])
{
    std::string crop_filename;
    std::vector< std::string > input_files;
    std::cout<<" ***** initialisation arguments ***************"<<std::endl;
    try {

        po::options_description desc("Allowed options");
        desc.add_options()
			("help", "produce help message")
			("input-file", po::value< vector<string> >(), "input files, multiple input-file are enable.")
			("crop-file", po::value<string>(), "name of the file containing cropping polygon (wkt format), must be unique")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << desc << "\n";
            return 1;
        }
        if (vm.count("input-file"))
      	{
         	cout << "Input files are: "
           		 << vm["input-file"].as< vector<string> >() << "\n";
         	input_files=vm["input-file"].as< vector<string> >();
      	}
        if (vm.count("input-file"))
         {
            cout << "crop file is: "
                << vm["crop-file"].as<string>() << "\n";
             crop_filename=vm["crop-file"].as<string>();
         }
    }
    catch(std::exception& e) {
        cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        cerr << "Exception of unknown type!\n";
    }
    std::cout<<std::endl;

    crop_region crop;
    boost::shared_ptr<LidarDataContainer> m_lidarOutput=boost::shared_ptr<LidarDataContainer>();
    if( !read_crop_region(crop_filename, crop )==0)
    {
		std::cout<<" invalid crop, exit programme "<<std::endl;
    	return 1;
    }

    typedef crop_region::iterator crop_ite_type;
    crop_ite_type crop_ite=crop.begin();
    for(crop_ite; crop_ite!=crop.end(); crop_ite++)
    {
    	polygon_2d poly=(*crop_ite).second;
		typedef std::vector< std::string>::iterator ite_file_type;
		ite_file_type input_file_ite=input_files.begin();
		std::string output_format;
		m_lidarOutput=InitOutputData(*input_file_ite, output_format);
		for ( input_file_ite; input_file_ite!=input_files.end(); input_file_ite++)
		{
			boost::shared_ptr<LidarDataContainer> m_lidarInput;
			m_lidarInput=LoadInputData(*input_file_ite);
			add_crop(poly, m_lidarInput, m_lidarOutput );
		}
		boost::shared_ptr<LidarFileIO> writer=LidarIOFactory::instance().createObject(output_format);
		std::string output_name=(*crop_ite).first;
		std::cout<<" write crop file "<<output_name<<std::endl;
		try{
			writer->save(*m_lidarOutput,output_name);
		}
		catch( std::exception &e )
		{
				std::cout << e.what() << std::endl;
				std::cout << "invalide erreur !!\n";
		}
		std::cout<<" fichier ecrit "<<output_name<<std::endl;

    }
    return 0;
}
