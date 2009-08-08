#ifndef __TPOINT2D_H__
#define __TPOINT2D_H__

/*
 * 	Code from MATIS library
 */





#include <cmath>
#include <ostream>



// -----------------------------------------------------
// Point2D
// -----------------------------------------------------
#define MACRO_CONST_RECOP_P2D(U) TPoint2D ( TPoint2D < U > const &src) { x = (T) src.x ; y = (T) src.y ; }
#define MACRO_AFFEC_P2D(U) TPoint2D < T > & operator = ( TPoint2D < U > const &src) { x = (T) src.x ; y = (T) src.y ; return *this ; }

/**
* Point en 2 dimensions.
* 
* Cette classe n'est pr�vue pour fonctionner qu'avec des types signes.
* Pour certaines methodes, on voit aussi les points comme des vecteurs.
*/
template <class T>
class TPoint2D {
  public :
   /** @name Les coordonnees du point. */
  //@{
  ///
  T x ;
  ///
  T y ;
  //@}
  
  /** Les typedefs */
  //@{
  typedef TPoint2D < T > self ;
  //@}
  
  
  /** Constructeurs et destructeur
   *
   */
  //@{
  ///
  TPoint2D () {  }
  ///
  TPoint2D (const T a, const T b) { x=a; y=b; }
  ///
  TPoint2D (const T t)  { x=t; y=t; }
  ///
  ~TPoint2D () {
  }
  
  
  
  /** Affectation et acces
   *
   * Ces methodes 'travaillent' en double pour etre compatibles avec certains codes. Pour travailler dans le type {\b T}, utilisez directement les membres {\b .x} et {\b .y} de la classe.
   */ 
  //@{
  ///
  inline void setx(const double v) { x=(T)v;}
  ///
  inline void sety(const double v) { y=(T)v;}
  ///
  inline double X () const { return ((double)x) ; }
  ///
  inline double Y () const { return ((double)y) ; }
  
  /// Affectation directe des 2 coordonnees.
  inline void set (const T a, const T b) {
    x = a ;
    y = b ;
  }

  ///
  T & operator [] (const int n) {
    if (n==0) return x ;
    if (n==1) return y ;
    return x ;
  }
  ///
  T  operator [] (const int n) const {
    if (n==0) return x ;
    if (n==1) return y ;
    return x ;
  }


  //@}  
  
  MACRO_CONST_RECOP_P2D(signed char) ;
  MACRO_CONST_RECOP_P2D(signed short) ;
  MACRO_CONST_RECOP_P2D(signed int) ;
  MACRO_CONST_RECOP_P2D(signed long) ;
  MACRO_CONST_RECOP_P2D(float) ;
  MACRO_CONST_RECOP_P2D(double) ;
  MACRO_AFFEC_P2D(signed char) ;
  MACRO_AFFEC_P2D(signed short) ;
  MACRO_AFFEC_P2D(signed int) ;
  MACRO_AFFEC_P2D(signed long) ;
  MACRO_AFFEC_P2D(float) ;
  MACRO_AFFEC_P2D(double) ;
    
  /** Comparaison de points */
  //@{
  ///
  bool operator == (const self & p) const { return (x==p.x && y==p.y) ; }
  ///
  bool operator != (const self & p) const { return (x!=p.x || y!=p.y) ; }
  /// tri lexicographique sur les coordonnees
  bool operator < (const self & p) const { return ( (x<p.x) || ((x==p.x)&&(y<p.y)) ) ; }
  /// tri lexicographique sur les coordonnees
  bool operator > (const self & p) const { return ( (x>p.x) || ((x==p.x)&&(y>p.y)) ) ; }
  //@}

  /** Arithm�tique des points et vecteurs */
  //@{
  ///
  self operator - (const self &b) const {
    return TPoint2D<T> (x-b.x, y-b.y) ;
  }
  ///
  self operator - () const {
    return TPoint2D<T> (-x, -y) ;
  }

  ///
  self operator + (const self &b) const {
    return self (x+b.x, y+b.y) ;
  }
  ///
  void operator += (const self &b) {
    x += b.x ;
    y += b.y ;
  }
  
  ///
  self operator / (const T t) const {
    return self (x/t, y/t) ;
  }

  ///
  self operator * (const T t) const {
    return self ( x*t, y*t) ;
  }

  ///
  self& operator /= (const T t) {
    x /= t ;
    y /= t ;
    return (*this) ;
  }
  
  ///
  self & operator *= (const T t) {
    x *= t ;
    y *= t ;
    return (*this) ;
  }
  //@}

  /** Produit scalaire et vectoriel */
  //@{
  ///
  T operator * (const self & p) const {
    return (x*p.x+y*p.y) ;
  }

  /// Produit vectoriel (d�terminant)
  T operator ^ (const self& p ) const { return x*p.y - y*p.x; }
  //@}


  /** Norme, normalisation */
  //@{
  /// Norme
  double norme() const {
	  return (sqrt ((double)x*x+(double)y*y)) ;
  }
  /** Normalisation
   *
   * Si la norme est nulle, rien n'est fait.
   */
  TPoint2D<T> & normaliser () 
    {
      double d=norme() ;
      if (d) 
	{
	  x= (T) ((double)x/d) ;
	  y= (T) ((double)y/d) ;
	}
      return (*this);
    }
  //@}


  
  

  /** Rotations et sym�trie */
  //@{
  ///
  self tourne(const double& a) const {
    double ca = cos(a);
    double sa = sin(a);
    return (TPoint2D<T> ((T) (x*ca-y*sa), (T) (x*sa+y*ca)) );
  } 
  ///
  self quart2tour() const {
    return (TPoint2D<T> (-y,x));
  }
  ///
  self symox() const {
    return (TPoint2D<T> (x,-y));
  }
  ///
  self symoy() const {
    return (TPoint2D<T> (-x,y));
  }
  ///
  self swapxy() const {
    return (TPoint2D<T> (y,x));
  }
  //@}
} ;




template < class T, class U > 
TPoint2D<T> & operator -= (TPoint2D<T> &a, const TPoint2D<U> &b) {
	a.x -= b.x ;
	a.y -= b.y ;
	return a ;
}

template <class T>
inline std::ostream& operator << ( std::ostream &os, const TPoint2D<T>&p ) {
  return (os << "(" << p.x << "," << p.y << ")") ;
}



/** Multiplication � gauche par un scalaire pour TPoint2D */
template < class T >
TPoint2D<T> operator * (const T t, TPoint2D < T > const & p) {
  return p*t ;
}






#undef MACRO_CONST_RECOP_P2D
#undef MACRO_AFFEC_P2D

#endif  // __TPOINT2D_H__


