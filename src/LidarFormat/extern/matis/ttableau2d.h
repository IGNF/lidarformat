#ifndef __TTABLEAU2D_HPP__
#define __TTABLEAU2D_HPP__


/*
 * 	Code from MATIS library
 */


#include <vector>
#include "LidarFormat/extern/matis/tpoint2d.h"

/** Tableau a deux dimensions.
 *
 * Cette classe est basee sur les vector de la STL
 */
template<class T>
class TTableau2D
{
	public:
		/** Le type iterateur */
typedef		typename std::vector<T>::iterator iterator;
		/** Le type iterateur constant */
		typedef typename std::vector<T>::const_iterator const_iterator;

		typedef typename std::vector<T>::reference reference;
		typedef typename std::vector<T>::const_reference const_reference;

		TTableau2D ()
		{	Construire (0, 0);}
		TTableau2D (int t, int u)
		{	Construire (t, u);}
		TTableau2D (TPoint2D<int> const & p)
		{	Construire (p.x, p.y);}
		TTableau2D (int t, int u, T v)
		{	Construire (t, u); operator = (v);}
		TTableau2D (TTableau2D<T> const & t)
		{	Construire (t.GetTaille().x, t.GetTaille().y); operator = (t);}
		~TTableau2D ()
		{	Detruire ();}
		const TPoint2D<int> & GetTaille() const
		{	return m_taille;}

		void SetTaille (int t, int u)
		{
			if (t!=m_taille.x || u!=m_taille.y)
			{
				//Detruire () ;
				Construire (t, u);
			}
		}
		void SetTaille (TPoint2D<int> const & p)
		{	SetTaille (p.x, p.y);}

		//subscripting: pointer to row i
		inline T * operator[](int i)
		{	return &(*m_ligne[i]);}
		//subscripting: pointer to row i
		inline const T * operator[](int i) const
		{	return &(*m_ligne[i]);}

		inline reference operator () (int x, int y)
		{
			return *(m_ligne[x] + y);
		}

		inline const_reference operator () (int x, int y) const
		{
			return *(m_ligne[x] + y);
		}

		TTableau2D<T> & operator = ( T v)
		{
			iterator it=begin(), fin = end();
			for (; it != fin; ++it)
			*it = v;
			return (*this);
		}

		TTableau2D<T> & operator = (TTableau2D<T> const & src)
		{
			SetTaille(src.GetTaille());
			iterator it=begin(), fin = end();
			const_iterator itsrc=src.begin();
			for (; it != fin; ++it, ++itsrc)
			*it = *itsrc;
			return (*this);
		}

		//-- Fonctions obsoletes.
		void vReformat (int t, int u)
		{	SetTaille (t, u);}
		void vReformat (TPoint2D<int> const & p)
		{	vReformat (p.x, p.y);}
		void clear()
		{	Detruire();}

		/** L'iterateur pointant vers le debut */
		inline iterator begin ()
		{
			return m_data.begin();
		}

		/** L'iterateur constant pointant vers le debut */
		inline const_iterator begin () const
		{
			return m_data.begin();
		}

		/** L'iterateur pointant vers la fin */
		inline iterator end ()
		{
			return m_data.end();
		}

		/** L'iterateur constant pointant vers la fin */
		inline const_iterator end () const
		{
			return m_data.end();
		}

		protected :

		TPoint2D<int> m_taille;
		std::vector<T> m_data;
		std::vector< iterator> m_ligne;

		void Construire (int x, int y)
		{
			if (x<=0 || y<=0)
			{
				m_taille.x=m_taille.y = 0;
			}
			else
			{
				m_taille.x=x;
				m_taille.y=y;
			}
			m_data.resize(m_taille.x * m_taille.y);
			m_ligne.resize(m_taille.x);
			for (int i=0; i<m_taille.x; ++i)
			m_ligne[i]= m_data.begin() + i * y;
		}

		void Detruire ()
		{
			m_data.clear();
			m_ligne.clear();
			m_taille.x = m_taille.y = 0;
		}
	};

template<class T>
std::ostream& operator <<( std::ostream& sortie, TTableau2D< T > const & p )
{
	sortie << "[ ";
	int x, y;
	for (x = 0; x < p.GetTaille().x; ++x)
	{
		sortie << "[ ";
		for (y = 0; y < p.GetTaille().y; ++y)
		{
			sortie << p( x, y ) << " ";
		}
		sortie << " ]" << std::endl;
	}
	sortie << " ]";
	return ( sortie );
}

#endif // __TTABLEAU2D_HPP__
