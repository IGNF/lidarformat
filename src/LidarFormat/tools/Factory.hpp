/***********************************************************************

This file is part of the LidarFormat project source files.

LidarFormat is an open source library for efficiently handling 3D point 
clouds with a variable number of attributes at runtime. 


Homepage: 

	http://code.google.com/p/lidarformat
	
Copyright:
	
	Institut Geographique National & CEMAGREF (2009)

Author: 

	Adrien Chauve
	
Contributors:

	Nicolas David, Olivier Tournaire
	
	

    LidarFormat is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LidarFormat is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public 
    License along with LidarFormat.  If not, see <http://www.gnu.org/licenses/>.
 
***********************************************************************/


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

// 08/01/2009: [Olivier Tournaire] FactoryErrorPolicy

template <class TIdentifierType , class ProductType>
class ThrowFactoryErrorPolicy
{
    public:
    static boost::shared_ptr<ProductType> OnUnknowType(const TIdentifierType& id)
    {
        throw std::logic_error("Unknown object type passed to factory !\n");
    }
};

template
<
	class TAbstractProduct,
	typename TIdentifierType = std::string,
	typename TProductCreator = boost::function<boost::shared_ptr<TAbstractProduct> ()>,
        template <typename , class> class FactoryErrorPolicy = ThrowFactoryErrorPolicy
>
class Factory : public FactoryErrorPolicy<TIdentifierType,TAbstractProduct>
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

                        //throw std::logic_error("Unknown object type passed to factory !\n");
                        return OnUnknowType(id);
		}

	private:
		typedef std::map<TIdentifierType, TProductCreator> AssocMapType;
		AssocMapType m_associations;

};

#endif /* FACTORY_HPP_ */
