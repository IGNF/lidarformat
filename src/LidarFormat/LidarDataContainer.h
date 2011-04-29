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


#ifndef LIDARDATACONTAINER_H_
#define LIDARDATACONTAINER_H_

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <iterator>
//#include <map>
#include <cassert>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>



#include "LidarFormat/AttributesInfo.h"
#include "LidarFormat/LidarIteratorAttribute.h"
#include "LidarFormat/LidarIteratorEcho.h"
#include "LidarFormat/LidarIteratorXYZ.h"

#include "LidarFormat/LidarDataFormatTypes.h"


namespace Lidar
{

using boost::shared_ptr;


class LidarDataContainer
{
	public:
		typedef LidarEcho									value_type;
		typedef std::size_t									size_type;
		typedef LidarIteratorEcho							iterator;
		typedef LidarConstIteratorEcho						const_iterator;

		typedef iterator::difference_type					difference_type;

		typedef iterator::reference							reference;
		typedef const_iterator::reference					const_reference;
		typedef const_iterator::pointer						const_pointer;

	    typedef std::reverse_iterator<const_iterator>		const_reverse_iterator;
	    typedef std::reverse_iterator<iterator>				reverse_iterator;


		typedef unsigned int IndexType;


		LidarDataContainer();
		LidarDataContainer(const LidarDataContainer&);
		LidarDataContainer& operator=(const LidarDataContainer&);



		///Interface semblable au vector :
		bool empty() const;
		void push_back(const LidarEcho& echo);
		void reserve(const std::size_t nbEchos);
		void resize(const std::size_t nbEchos);
		std::size_t capacity() const;
		std::size_t size() const;
		std::size_t max_size() const;

		void append(const LidarDataContainer& rhs);

		unsigned int erase(const unsigned int position);
		unsigned int erase(const unsigned int first, const unsigned int last);
		LidarIteratorEcho erase(const LidarIteratorEcho& position);
		LidarIteratorEcho erase(const LidarIteratorEcho& first, const LidarIteratorEcho& last);
		template<typename T> LidarIteratorXYZ<T> erase(const LidarIteratorXYZ<T>& position);
		template<typename T> LidarIteratorXYZ<T> erase(const LidarIteratorXYZ<T>& first, const LidarIteratorXYZ<T>& last);
		template<typename T> LidarIteratorAttribute<T> erase(const LidarIteratorAttribute<T>& position);
		template<typename T> LidarIteratorAttribute<T> erase(const LidarIteratorAttribute<T>& first, const LidarIteratorAttribute<T>& last);

		///TODO attention efface les données mais garde les maps d'attibuts en mémoire...
		void clear();


		reference operator[](const unsigned int index);
		const_reference operator[](const unsigned int index) const;



		///Affichage de la liste des attributs sur une ligne
		std::ostream & printHeader(std::ostream &os) const;


		///Itérateurs
		template<typename T> LidarIteratorAttribute<T> beginAttribute(const std::string &attributeName);
		template<typename T> LidarIteratorAttribute<T> endAttribute(const std::string &attributeName);
		template<typename T> LidarConstIteratorAttribute<T> beginAttribute(const std::string &attributeName) const;
		template<typename T> LidarConstIteratorAttribute<T> endAttribute(const std::string &attributeName) const;

		template<typename T> LidarIteratorXYZ<T> beginXYZ();
		template<typename T> LidarIteratorXYZ<T> endXYZ();
		template<typename T> LidarConstIteratorXYZ<T> beginXYZ() const;
		template<typename T> LidarConstIteratorXYZ<T> endXYZ() const;

		iterator begin();
		iterator end();
		const_iterator begin() const;
		const_iterator end() const;

		reverse_iterator rbegin();
		reverse_iterator rend();
		const_reverse_iterator rbegin() const;
		const_reverse_iterator rend() const;

		///Ajoute un attribut avec son type dans le container
		///  ATTENTION : l'appel à cette fonction rend obsolète tous les itérateurs en cours, et rend incompatible les anciens LidarEcho avec les nouveaux
		///  La fonction retourne false si l'attribut était déjà présent dans le container
		bool addAttribute(const std::string& attributeName, const EnumLidarDataType type);
		void addAttributeList(const std::vector<std::pair<std::string, EnumLidarDataType> > attributes);

		bool delAttribute(const std::string& attributeName);
		void delAttributeList(const std::vector<std::string>& attributeNames);

		bool checkAttributeIsPresent(const std::string& attributeName);

		bool checkAttributeIsPresentAndType(const std::string& attributeName, const EnumLidarDataType type);

		EnumLidarDataType getAttributeType(const std::string &attributeName) const;

		const unsigned int pointSize() const { return pointSize_; }

		void getAttributeList(std::vector<std::string> &liste) const;

		///ATTENTION ! Renvoit un pointeur sur les données du conteneur... dangereux ! (utilisé par la classe LidarFile pour charger les données binaires en une fois)
		char* rawData() { return &lidarData_.front(); }
		const char* rawData() const { return &lidarData_.front(); }

		const AttributeMapType& getAttributeMap() const { return *attributeMap_; }

		LidarEcho createEcho() const;

		///Attention !
		void push_back(const char* echo);
		///Attention !
		const char* rawData(const unsigned int index) const { return rawData() + index*pointSize();}
		char* rawData(const unsigned int index) { return rawData() + index*pointSize();}

		unsigned int getDecalage(const std::string &attributeName) const
		{
			return attributeMap_->find(attributeName)->second.decalage;
		}

	private:
		void copy(const LidarDataContainer& rhs);

		///Update container content after having added attributes
		void updateAttributeContent(const unsigned int oldPointSize);
		bool addAttributeHelper(const std::string& attributeName, const EnumLidarDataType type);


		/////Structure interne
		typedef char BaseType;
		typedef std::vector<BaseType> LidarDataContainerType;

		LidarIteratorEcho createLidarIteratorEcho(char* data) const
		{
			return LidarIteratorEcho(data, pointSize(), attributeMap_);
		}

		LidarConstIteratorEcho createLidarConstIteratorEcho(char* data) const
		{
			return LidarConstIteratorEcho(data, pointSize(), attributeMap_);
		}



		bool isAttribute(const std::string &attributeName);


		///data
		mutable LidarDataContainerType lidarData_;
		shared_ptr<AttributeMapType> attributeMap_; //infos sur les attributs

		unsigned int pointSize_;


};



///////////////IMPLEMENTATION TEMPLATE et INLINE


inline LidarEcho LidarDataContainer::createEcho() const
{
	return LidarEcho(pointSize(), boost::shared_array<BaseType>(new BaseType[pointSize()]).get(), attributeMap_);
}


template<typename T>
inline LidarIteratorAttribute<T> LidarDataContainer::beginAttribute(const std::string &attributeName)
{
	return LidarIteratorAttribute<T>(rawData() + getDecalage(attributeName), pointSize());
}

template<typename T>
inline LidarIteratorAttribute<T> LidarDataContainer::endAttribute(const std::string &attributeName)
{
	return LidarIteratorAttribute<T>(rawData(size()) + getDecalage(attributeName), pointSize());
}

template<typename T>
inline LidarConstIteratorAttribute<T> LidarDataContainer::beginAttribute(const std::string &attributeName) const
{
	return LidarConstIteratorAttribute<T>(&lidarData_.front() + getDecalage(attributeName), pointSize());
}

template<typename T>
inline LidarConstIteratorAttribute<T> LidarDataContainer::endAttribute(const std::string &attributeName) const
{
	return LidarConstIteratorAttribute<T>(&lidarData_.front() + pointSize()*size() + getDecalage(attributeName), pointSize());
}



template<typename T>
inline LidarIteratorXYZ<T> LidarDataContainer::beginXYZ()
{
	return LidarIteratorXYZ<T>(rawData() + getDecalage("x"), pointSize());
}

template<typename T>
inline LidarIteratorXYZ<T> LidarDataContainer::endXYZ()
{
	return LidarIteratorXYZ<T>(rawData(size()) + getDecalage("x"), pointSize());
}

template<typename T>
inline LidarConstIteratorXYZ<T> LidarDataContainer::beginXYZ() const
{
	return LidarConstIteratorXYZ<T>(&lidarData_.front() + getDecalage("x"), pointSize());
}

template<typename T>
inline LidarConstIteratorXYZ<T> LidarDataContainer::endXYZ() const
{
	return LidarConstIteratorXYZ<T>(&lidarData_.front() + pointSize()*size() + getDecalage("x"), pointSize());
}





inline LidarIteratorEcho LidarDataContainer::begin()
{
	return createLidarIteratorEcho(rawData());
}

inline LidarIteratorEcho LidarDataContainer::end()
{
	return createLidarIteratorEcho(rawData(size()));
}

inline LidarConstIteratorEcho LidarDataContainer::begin() const
{
	return createLidarConstIteratorEcho(&lidarData_.front());
}

inline LidarConstIteratorEcho LidarDataContainer::end() const
{
	return createLidarConstIteratorEcho(&lidarData_.front() + pointSize()*size());
}

inline LidarDataContainer::reverse_iterator LidarDataContainer::rbegin()
{
	return reverse_iterator(end());
}

inline LidarDataContainer::reverse_iterator LidarDataContainer::rend()
{
	return reverse_iterator(begin());
}

inline LidarDataContainer::const_reverse_iterator LidarDataContainer::rbegin() const
{
	return const_reverse_iterator(end());
}

inline LidarDataContainer::const_reverse_iterator LidarDataContainer::rend() const
{
	return const_reverse_iterator(begin());
}

inline std::size_t LidarDataContainer::size() const
{
	return lidarData_.size()/pointSize();
}

inline std::size_t LidarDataContainer::max_size() const
{
	return lidarData_.max_size()/pointSize();
}

inline std::size_t LidarDataContainer::capacity() const
{
	return lidarData_.capacity()/pointSize();
}

inline bool LidarDataContainer::empty() const
{
	return size()==0;
}

inline void LidarDataContainer::resize(const std::size_t nbEchos)
{
	lidarData_.resize(nbEchos*pointSize());
}


inline LidarDataContainer::reference LidarDataContainer::operator[](const unsigned int index)
{
	return begin()[index];
}


inline LidarDataContainer::const_reference LidarDataContainer::operator[](const unsigned int index) const
{
	return begin()[index];
}


inline void LidarDataContainer::push_back(const LidarEcho& echo)
{
	assert(echo.size() == pointSize());

	resize(size()+1);

	//recopie de l'écho
	*(end()-1) = echo;
}


inline void LidarDataContainer::push_back(const char* echo)
{
	resize(size()+1);

	//recopie de l'écho
	memcpy(rawData() + (size()-1)*pointSize(), echo, pointSize());
}

inline void LidarDataContainer::reserve(const std::size_t nbEchos)
{
	lidarData_.reserve(nbEchos*pointSize());
}

inline unsigned int LidarDataContainer::erase(const unsigned int position)
{
	LidarDataContainerType::iterator erase_pos = lidarData_.erase(lidarData_.begin() + position*pointSize(), lidarData_.begin() + (position+1)*pointSize());
	return  erase_pos - lidarData_.begin();
}

inline unsigned int LidarDataContainer::erase(const unsigned int first, const unsigned int last)
{
	LidarDataContainerType::iterator erase_pos = lidarData_.erase(lidarData_.begin() + first*pointSize(), lidarData_.begin() + last*pointSize());
	return erase_pos - lidarData_.begin();
}

inline LidarIteratorEcho LidarDataContainer::erase(const LidarIteratorEcho& position)
{
	const LidarIteratorEcho beg = begin();
	const unsigned int index = position - beg;
	const unsigned int pos_erase = erase(index);
	return begin() + pos_erase;
}

inline LidarIteratorEcho LidarDataContainer::erase(const LidarIteratorEcho& first, const LidarIteratorEcho& last)
{
	const LidarIteratorEcho beg = begin();
	const unsigned int indexFirst = first - beg;
	const unsigned int indexLast = last - beg;
	const unsigned int pos_erase = erase(indexFirst, indexLast);
	return begin() + pos_erase;
}

template<typename T>
inline LidarIteratorXYZ<T> LidarDataContainer::erase(const LidarIteratorXYZ<T>& position)
{
	const LidarIteratorXYZ<T> beg = beginXYZ<T>();
	const unsigned int index = position - beg;
	const unsigned int pos_erase = erase(index);
	return beginXYZ<T>() + pos_erase;
}

template<typename T>
inline LidarIteratorXYZ<T> LidarDataContainer::erase(const LidarIteratorXYZ<T>& first, const LidarIteratorXYZ<T>& last)
{
	const LidarIteratorXYZ<T> beg = beginXYZ<T>();
	const unsigned int indexFirst = first - beg;
	const unsigned int indexLast = last - beg;
	const unsigned int pos_erase = erase(indexFirst, indexLast);
	return beginXYZ<T>() + pos_erase;
}

template<typename T>
inline LidarIteratorAttribute<T> LidarDataContainer::erase(const LidarIteratorAttribute<T>& position)
{
	const LidarIteratorAttribute<T> beg = beginAttribute<T>();
	const unsigned int index = position - beg;
	const unsigned int pos_erase = erase(index);
	return beginAttribute<T>() + pos_erase;
}

template<typename T>
inline LidarIteratorAttribute<T> LidarDataContainer::erase(const LidarIteratorAttribute<T>& first, const LidarIteratorAttribute<T>& last)
{
	const LidarIteratorAttribute<T> beg = beginAttribute<T>();
	const unsigned int indexFirst = first - beg;
	const unsigned int indexLast = last - beg;
	const unsigned int pos_erase = erase(indexFirst, indexLast);
	return beginAttribute<T>() + pos_erase;
}

inline bool LidarDataContainer::checkAttributeIsPresent(const std::string& attributeName)
{
	return !(attributeMap_->find(attributeName) == attributeMap_->end());
}

inline bool LidarDataContainer::checkAttributeIsPresentAndType(const std::string& attributeName, const EnumLidarDataType type)
{
	AttributeMapType::const_iterator it = attributeMap_->find(attributeName);

	if(it != attributeMap_->end())
	{
		return it->second.type == type;
	}

	return false;
}

inline EnumLidarDataType LidarDataContainer::getAttributeType(const std::string &attributeName) const
{
	assert(attributeMap_->find(attributeName) != attributeMap_->end());
	return attributeMap_->find(attributeName)->second.type;
}



} //namespace Lidar


#endif /* LIDARDATACONTAINER_H_ */
