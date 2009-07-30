#ifndef __TPOINT3D_H__
#define __TPOINT3D_H__


/*
 * 	Code from MATIS library
 */


#include "tpoint2d.h"


// ----------------------------------------------------
// Point3D
// ----------------------------------------------------
#define MACRO_CONST_RECOP_P3D(U) TPoint3D ( TPoint3D < U > const &src) { x = (T) src.x ; y = (T) src.y ; z = (T) src.z ; }
#define MACRO_AFFEC_P3D(U) TPoint3D < T > & operator = ( TPoint2D < U > const &src) { x = (T) src.x ; y = (T) src.y ; z = (T) 0 ; return *this ; }


/**
* Point en 3 dimensions.
*/
template <class T>
class TPoint3D
{
  public :
    /** Les 3 coordonnées */
    //@{
    ///
    T x ;
  ///
  T y ;
  ///
  T z ;
  //@}

  /** Les typedefs */
  //@{
  ///
  typedef TPoint3D < T > self ;
  //@}

  /** Constructeurs et destructeur */
  //@{
  ///
  TPoint3D () {} ;
  ///
  TPoint3D (const T a, const T b, const T c) { x=a;y=b;z=c ; }
  ///
  ~TPoint3D () {}
  //@}

  /** Accès et affectation */
  //@{
  ///
  inline void setx(const double v) { x=(T)v;}
  ///
  inline void sety(const double v) { y=(T)v;}
  ///
  inline void setz(const double v) { z=(T)v;}
  ///
  void set (const T X, const T Y, const T Z) { x=X; y=Y; z=Z; }
  ///
  inline double X () const { return (double)x ; }
  ///
  inline double Y () const { return (double)y ; }
  ///
  inline double Z () const { return (double)z ; }

  /// Operateur d'affectation
  TPoint3D<T> & operator = (TPoint3D<T> const &p) {
    set(p.x,p.y,p.z) ;
    return (*this) ;
  }

  /// Operateur d'affectation
  void operator = (T const &val)
  {
    x = y = z = val ;
  }
  ///
  T & operator [] (const int n) {
    if (n==0) return x ;
    if (n==1) return y ;
    if (n==2) return z ;
    return x ;
  }
  ///
  T  operator [] (const int n) const {
    if (n==0) return x ;
    if (n==1) return y ;
    if (n==2) return z ;
    return x ;
  }


  //@}

//  MACRO_CONST_RECOP_P3D(Int1);
//  MACRO_CONST_RECOP_P3D(Int2);
//  MACRO_CONST_RECOP_P3D(Int4);
//  MACRO_CONST_RECOP_P3D(Float4);
//  MACRO_CONST_RECOP_P3D(Float8);
//  MACRO_AFFEC_P3D(Int1);
//  MACRO_AFFEC_P3D(Int2);
//  MACRO_AFFEC_P3D(Int4);
//  MACRO_AFFEC_P3D(Float4);
//  MACRO_AFFEC_P3D(Float8);

    /** Comparaison */
    //@{
    ///
  bool operator == (const TPoint3D<T> & p) const{ return ( std::abs(x-p.x)<10E-9 && std::abs(y-p.y)<10E-9 && std::abs(z-p.z)<10E-9) ; }
  ///
  bool operator != (const TPoint3D<T> & p) const{ return (x!=p.x || y!=p.y || z!=p.z) ; }
  /// tri lexicographique sur les coordonnees
  bool operator < (const self & p) const { return ( (x<p.x) || ((x==p.x)&&(y<p.y)) || ((x==p.x)&&(y==p.y)&&(z<p.z))) ; }
  /// tri lexicographique sur les coordonnees
  bool operator > (const self & p) const { return ( (x>p.x) || ((x==p.x)&&(y>p.y)) || ((x==p.x)&&(y==p.y)&&(z>p.z))) ; }

  //@}

  /** Norme et normalisation */
  //@{
  ///
  inline double norme () const
  {
    double d ;
    d  = (double) x * (double) x ;
    d += (double) y * (double) y ;
    d += (double) z * (double) z ;
    return sqrt (d) ;
  }
  ///
  inline TPoint3D<T> & normaliser ()
  {
    T d = (T) norme ();
    if (d) operator /= (d) ;
    return (*this) ;
  }
  //@}

  /** Arithmetique */
  //@{
  ///
  TPoint3D<T> operator / (const T t) const {
#ifdef _DEBUG
    if (t==(T)0) throw Erreur ("[TPoint2D] Division par zero") ;
#endif
    return 	TPoint3D<T> (x/t, y/t,z/t) ;
  }
  ///
  inline void operator /= (const T d) {
    x /= d ;
    y /= d ;
    z /= d ;
  }
  ///
  TPoint3D<T> operator * (const T c) const {
    return TPoint3D<T> (c*x,c*y,c*z) ;
  }
  ///
  void operator *= (const T c) {
    x=c*x;
    y=c*y;
    z=c*z;
  }
  ///
  TPoint3D<T> operator - (const TPoint3D<T> &b) const {
    return TPoint3D<T> (x-b.x, y-b.y, z-b.z) ;
  }
  ///
  TPoint3D<T> operator - () const {
    return TPoint3D<T> (-x, -y, -z) ;
  }
  ///
  TPoint3D<T> operator + (const  TPoint3D<T> &q) const {
    return TPoint3D<T> (x+q.x,y+q.y,z+q.z) ;
  }
  ///
  TPoint3D < T > & operator += (const TPoint3D<T> &b) {
    x += b.x ;
    y += b.y ;
    z += b.z ;
    return *this ;
  }
  //@}
  //-- Produit vectoriel
  TPoint3D<T> operator ^ ( const TPoint3D<T>& b ) const{
    return TPoint3D<T> (
			y * b.z -  z * b.y,
			z * b.x -  x * b.z,
			x * b.y -  y * b.x) ;
  }


  T operator * (const TPoint3D<T>& b) const {
    return (x*b.x+y*b.y+z*b.z) ;
  }
} ;

/** Affichage d'un TPoint3D aiu format (x, y, z).*/
template <class T>
std::ostream& operator << ( std::ostream &os, const TPoint3D<T> & p) {
  return (os << "(" << p.x << "," << p.y << "," << p.z << ")") ;
}

/** Soustraction de deux TPoint3D de types hétérogènes */
template < class T, class U >
TPoint3D<T> & operator -= (TPoint3D<T> &a, const TPoint3D<U> &b) {
	a.x -= b.x ;
	a.y -= b.y ;
	a.z -= b.z ;

	return a ;
}

/// Multiplication d'un TPoint3D à gauche par un scalaire
template < class T >
TPoint3D<T> operator * (const T t, TPoint3D < T > const & p) {
  return p*t ;
}

#undef MACRO_CONST_RECOP_P3D
#undef MACRO_AFFEC_P3D


#endif // __TPOINT3D_H__
