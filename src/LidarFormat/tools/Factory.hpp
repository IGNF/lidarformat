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

/*
 * Factory.hpp
 *
 *  Created on: 28 oct. 2008
 *      Author: achauve
 */

#ifndef FACTORY_HPP_
#define FACTORY_HPP_

#include <map>
#include <string>
#include <stdexcept>

//#include <iostream> //debug

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

/**
* @brief Classe de Factory.
*
* Implémentation d'une factory très simple à partir de "Modern C++ Design" d'Alexandrescu.
* (simple = AssocMap non opimisée, pas de gestionnaire d'erreur... à améliorer !)
* A noter l'utilisation de boost::function pour pouvoir récupérer le résultat d'un bind comme un creator,
* ce qui n'est pas possible directement avec un TAbstractProduct* (*) () classique.
*
* Version avec shared_ptr : attention ne convient pas pour stocker un objet wx !
*
* @author Adrien Chauve
*
*/

template
<
	class TAbstractProduct,
	typename TIdentifierType = std::string,
	typename TProductCreator = boost::function<boost::shared_ptr<TAbstractProduct> ()>
>
class Factory
{

	public:
		bool Register(const TIdentifierType& id, TProductCreator creator)
		{
//			std::cout << "Factyory register: " << id << "\n";
//			for(typename AssocMapType::const_iterator it=m_associations.begin(); it!=m_associations.end(); ++it)
//			{
//				std::cout << "contenu : " << it->first << "\n";
//			}
			return m_associations.insert( typename AssocMapType::value_type(id, creator) ).second;
		}

		bool Unregister(const TIdentifierType& id)
		{
			return m_associations.erase(id) == 1;
		}

		virtual boost::shared_ptr<TAbstractProduct> createObject(const TIdentifierType& id) const
		{
			typename AssocMapType::const_iterator i = m_associations.find(id);
			if(i != m_associations.end())
			{
//				std::cout << "Factyory ok: creator trouvé !\n";
				return (i->second)();
			}
//			std::cout << "Factory pas ok: creator introuvable !! : " << id <<"\n";
//			std::cout << "Taille de la factory : " << m_associations.size() <<"\n";
//			for(typename AssocMapType::const_iterator it=m_associations.begin(); it!=m_associations.end(); ++it)
//			{
//				std::cout << "contenu : " << it->first << "\n";
//			}

			throw std::logic_error("Unknown object type passed to factory !\n");
		}

	private:
		typedef std::map<TIdentifierType, TProductCreator> AssocMapType;
		AssocMapType m_associations;

};

#endif /* FACTORY_HPP_ */
