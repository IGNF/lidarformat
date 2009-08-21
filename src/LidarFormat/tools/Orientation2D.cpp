/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. LidarFormat is 
distributed under the CeCILL-B licence. See Licence_CeCILL-B_V1-en.txt 
or http://www.cecill.info for more details.


Homepage: 

	https://fullanalyze.ign.fr/trac/LidarFormat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Author: 

	Adrien Chauve
	
Contributors:

	Nicolas David, Olivier Tournaire



This software is governed by the CeCILL-B license under French law and
abiding by the rules of distribution of free software.  You can  use, 
modify and/ or redistribute the software under the terms of the CeCILL-B
license as circulated by CEA, CNRS and INRIA at the following URL
"http://www.cecill.info". 

As a counterpart to the access to the source code and  rights to copy,
modify and redistribute granted by the license, users are provided only
with a limited warranty  and the software's author,  the holder of the
economic rights,  and the successive licensors  have only  limited
liability. 

In this respect, the user's attention is drawn to the risks associated
with loading,  using,  modifying and/or developing or reproducing the
software by the user in light of its specific status of free software,
that may mean  that it is complicated to manipulate,  and  that  also
therefore means  that it is reserved for developers  and  experienced
professionals having in-depth computer knowledge. Users are therefore
encouraged to load and test the software's suitability as regards their
requirements in conditions enabling the security of their systems and/or 
data to be ensured and,  more generally, to use and operate it in the 
same conditions as regards security. 

The fact that you are presently reading this means that you have had
knowledge of the CeCILL-B license and that you accept its terms.
 
***********************************************************************/

#include <fstream>
#include <stdexcept>
#include <sstream>

#include <boost/filesystem.hpp>
#include "LidarFormat/tools/Orientation2D.h"


namespace Lidar
{

Orientation2D::Orientation2D() :
	m_originX(-1), m_originY(-1), m_step(-1), m_zoneCarto(-1), m_sizeX(-1), m_sizeY(-1)
{
}


Orientation2D::Orientation2D(const double origineX, const double origineY, const double step, const unsigned int zoneCarto, const unsigned int tailleX, const unsigned int tailleY):
	m_originX(origineX), m_originY(origineY), m_step(step), m_zoneCarto(zoneCarto), m_sizeX(tailleX), m_sizeY(tailleY)
{

}

void Orientation2D::ReadOriFromImageFile(const std::string &filename)
{
	if ( !boost::filesystem::exists(filename) )
	{
		std::ostringstream oss;
		oss << "Le fichier image demandé n'existe pas : "<<filename<< " ! " << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}

	std::string basename = boost::filesystem::basename(filename);
	std::string path = boost::filesystem::path(filename).branch_path().string();

	try
	{
		ReadOriFromOriFile(path+"/"+basename+".ori");
	}
	catch (const std::logic_error &e)
	{
		// Si on catche cette erreur, cela signifie que le .ori n'existe pas
		// On essaie de lire un .tfw
		ReadOriFromTFWFile(path+"/"+basename+".tfw");
	}
}

void Orientation2D::ReadOriFromOriFile(const std::string &filename)
{
	if ( !boost::filesystem::exists(filename) )
	{
		std::ostringstream oss;
		oss << "Le fichier ori demandé n'existe pas : "<<filename<< " ! " << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}

	std::ifstream fileOri(filename.c_str() , std::ifstream::in );

#ifndef WIN32
	//Renvoit des exceptions en cas d'erreur de lecture/ouverture de fichier
	// Ca, sous windows, ca me lance direct une exception ...
	fileOri.exceptions ( std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit );
#endif

	//Verification de la presence du carto
	std::string carto;
	fileOri >> carto;

	// On suppose un formattage comme suit :
	// CARTO
	// OriginX (en m)   OriginY (en m)
	// Zone carto
	// TailleX    TailleY   (taille de l'image en pixels)
	// Pas en X (>0 - en m)    Pas en Y (<0 - en m)

	if (carto != "CARTO")
	{
		std::ostringstream oss;
		oss << "Fichier ORI au mauvais format !" << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}

	fileOri >> m_originX >> m_originY;
	fileOri >> m_zoneCarto;
	fileOri >> m_sizeX >> m_sizeY;
	double pasX, pasY;
	fileOri >> pasX >> pasY;

	if(pasX != pasY)
	{
		std::ostringstream oss;
		oss << "Erreur orientation non supportee : pas en X different de pas en Y !" << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}
	else
	m_step = pasX;

	fileOri.close();

}

void Orientation2D::ReadOriFromTFWFile(const std::string &filename)
{
	if ( !boost::filesystem::exists(filename) )
	{
		std::ostringstream oss;
		oss << "Le fichier TFW demandé n'existe pas : "<<filename<< " ! " << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}

	std::ifstream fileTFW(filename.c_str() , std::ifstream::in );

#ifndef WIN32
	//Renvoit des exceptions en cas d'erreur de lecture/ouverture de fichier
	// Ca, sous windows, ca me lance direct une exception ...
	fileTFW.exceptions ( std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit );
#endif

	// On suppose un formattage comme suit :
	// Pas en X (>0 - en m)
	// Sais pas
	// Sais pas
	// Pas en Y (<0 - en m)
	// OriginX (en m)
	// OriginY (en m)

	double temp, pasX, pasY;
	fileTFW >> pasX;
//	pasX *= 1000.;
	fileTFW >> temp;
	fileTFW >> temp;
	fileTFW >> pasY;
	pasY *= -1.;
	fileTFW >> m_originX;
//	m_originX *= 1000.;
	fileTFW >> m_originY;
//	m_originY *= 1000.;

	if(pasX != pasY)
	{
		std::ostringstream oss;
		oss << "Erreur orientation non supportee : pas en X different de pas en Y !" << std::endl;
		oss << "File : " <<__FILE__ << std::endl;
		oss << "Line : " << __LINE__ << std::endl;
		oss << "Function : " << __FUNCTION__ << std::endl;
		throw std::logic_error( oss.str() );
	}
	else
		m_step = pasX;

	fileTFW.close();
}


void Orientation2D::SaveOriToFile(const std::string &filename)
{
	std::ofstream fileOri(filename.c_str() , std::ofstream::out);

	fileOri<<"CARTO\n";
	fileOri<< m_originX << "\t" << m_originY << std::endl;
	fileOri<< m_zoneCarto << std::endl;
	fileOri<< m_sizeX << "\t" << m_sizeY << std::endl;
	fileOri<< m_step << "\t" << m_step << std::endl;

	fileOri.close();
}

std::string Orientation2D::Affiche() const
{
	std::ostringstream result;
	result << "Orientation : \n";
	result << "[Origine] : ("<<m_originX<<","<<m_originY<<"\n";
	result << "[Taille] : ("<<m_sizeX<<","<<m_sizeY<<"\n";
	result << "[Pas] : "<<m_step<<"\n";
	result << "[Zone] : "<<m_zoneCarto<<"\n";
	return result.str();
}

}

