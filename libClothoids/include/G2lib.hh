/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2017                                                      |
 |                                                                          |
 |         , __                 , __                                        |
 |        /|/  \               /|/  \                                       |
 |         | __/ _   ,_         | __/ _   ,_                                | 
 |         |   \|/  /  |  |   | |   \|/  /  |  |   |                        |
 |         |(__/|__/   |_/ \_/|/|(__/|__/   |_/ \_/|/                       |
 |                           /|                   /|                        |
 |                           \|                   \|                        |
 |                                                                          |
 |      Enrico Bertolazzi                                                   |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: enrico.bertolazzi@unitn.it                                   |
 |                                                                          |
\*--------------------------------------------------------------------------*/

///
/// file: G2lib.hh
///

/*!

\mainpage

Clothoids
=========

G1 and G2 fitting with clothoids, spline of clothods, circle arc and biarc

by Enrico Bertolazzi and Marco Frego

for the documentation see `manual.md` or
[Doxygen documentation: http://ebertolazzi.github.io/Clothoids/](http://ebertolazzi.github.io/Clothoids/)

Authors:
-------

  Enrico Bertolazzi and Marco Frego
  Department of Industrial Engineering
  University of Trento
  enrico.bertolazzi@unitn.it
  m.fregox@gmail.com

 */

#ifndef G2LIB_HH
#define G2LIB_HH

#include <iostream>
#include <cmath>
#include <cfloat>
#include <sstream>
#include <stdexcept>
#include <limits>

#include <vector>
#include <utility>

#ifndef G2LIB_DO_ERROR
  #define G2LIB_DO_ERROR(MSG)              \
    {                                      \
      std::ostringstream ost;              \
      ost << "On line: " << __LINE__       \
          << " file: " << __FILE__         \
          << '\n' << MSG << '\n';          \
      throw std::runtime_error(ost.str()); \
    }
#endif

#ifndef G2LIB_ASSERT
  #define G2LIB_ASSERT(COND,MSG) if ( !(COND) ) G2LIB_DO_ERROR(MSG)
#endif

// select computer architecture
#if defined(__APPLE__) && defined(__MACH__)
  // osx architecture
  #define G2LIB_OS_OSX 1
  #if defined(__i386__)
    #define G2LIB_ARCH32 1
  #elif defined(__x86_64__)
    #define G2LIB_ARCH64 1
  #endif
#elif defined(__unix__)
  // linux architecture
  #define G2LIB_OS_LINUX 1
  #if defined(__i386__)
    #define G2LIB_ARCH32 1
  #elif defined(__x86_64__)
    #define G2LIB_ARCH64 1
  #endif
#elif defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64)
  // windows architecture
  #define G2LIB_OS_WINDOWS 1
  #if defined(_M_X64) || defined(_M_AMD64)
    #define G2LIB_ARCH64 1
  #else
    #define G2LIB_ARCH32 1
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #error "unsupported OS!"
#endif

// check if compiler is C++11
#if (defined(_MSC_VER) &&  _MSC_VER >= 1800) || \
    (defined(__cplusplus) && __cplusplus > 199711L)
  #ifndef G2LIB_DO_NOT_USE_CXX11
    #define G2LIB_USE_CXX11
  #endif
#else
  // not C++11 compiler
  #ifndef nullptr
    #define nullptr NULL
  #endif
#endif

#define G2LIB_PURE_VIRTUAL = 0
#ifdef G2LIB_USE_CXX11
  #define G2LIB_OVERRIDE override
#else
  #define G2LIB_OVERRIDE
#endif

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wpadded"
#endif
#ifdef __clang__
#pragma clang diagnostic ignored "-Wpadded"
#pragma clang diagnostic ignored "-Wc++98-compat"
#endif

//! Clothoid computations routine
namespace G2lib {

  typedef std::basic_ostream<char> ostream_type;

  typedef double real_type;
  typedef int    int_type;

  extern real_type const machepsi;     //!< machine espilon \f$ \varepsilon \f$
  extern real_type const machepsi10;   //!< \f$ 10\varepsilon \f$
  extern real_type const machepsi100;  //!< \f$ 100\varepsilon \f$
  extern real_type const machepsi1000; //!< \f$ 1000\varepsilon \f$
  extern real_type const sqrtMachepsi; //!< \f$ \sqrt{\varepsilon} \f$
  extern real_type const m_pi;         //!< \f$ \pi \f$
  extern real_type const m_pi_2;       //!< \f$ \pi/2 \f$
  extern real_type const m_2pi;        //!< \f$ 2\pi \f$
  extern real_type const m_1_pi;       //!< \f$ 1/\pi \f$
  extern real_type const m_1_sqrt_pi;  //!< \f$ 1/\sqrt{\pi} \f$
  extern bool            intersect_with_AABBtree;

  #ifdef G2LIB_COMPATIBILITY_MODE

  extern bool            use_ISO;

  static
  inline
  void
  lib_use_ISO()
  { use_ISO = true; }

  static
  inline
  void
  lib_use_SAE()
  { use_ISO = false; }

  #endif

  //! disable AABB tree in computation
  static
  inline
  void
  noAABBtree()
  { intersect_with_AABBtree = false; }

  //! enable AABB tree in computation
  static
  inline
  void
  yesAABBtree()
  { intersect_with_AABBtree = true; }

  //! check if cloating point number `x` is zero
  static
  inline
  bool
  isZero( real_type x )
  { return FP_ZERO == std::fpclassify(x); }

  //! check if cloating point number `x` is finite
  static
  inline
  bool
  isInfinite( real_type x )
  { return FP_INFINITE == std::fpclassify(x); }

  //! check if cloating point number `x` is Not A Number
  static
  inline
  bool
  isNaN( real_type x )
  { return FP_NAN == std::fpclassify(x); }

  //! check if cloating point number `x` is regural (i.e. finite and not NaN)
  static
  inline
  bool
  isRegular( real_type x )
  { return !( FP_INFINITE == std::fpclassify(x) ||
              FP_NAN      == std::fpclassify(x) ); }

  /*
  // sin(x)/x
  */
  real_type Sinc( real_type x );     //!< \f$ \frac{\sin x}{x} \f$
  real_type Sinc_D( real_type x );   //!< \f$ \frac{\mathrm{d}}{\mathrm{d}x} \frac{\sin x}{x} \f$
  real_type Sinc_DD( real_type x );  //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^2 \frac{\sin x}{x} \f$
  real_type Sinc_DDD( real_type x ); //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^3 \frac{\sin x}{x} \f$

  /*
  // (1-cos(x))/x
  */
  real_type Cosc( real_type x );     //!< \f$ \frac{1-\cos x}{x} \f$
  real_type Cosc_D( real_type x );   //!< \f$ \frac{\mathrm{d}}{\mathrm{d}x} \frac{1-\cos x}{x} \f$
  real_type Cosc_DD( real_type x );  //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^2 \frac{1-\cos x}{x} \f$
  real_type Cosc_DDD( real_type x ); //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^3 \frac{1-\cos x}{x} \f$

  /*
  // atan(x)/x
  */
  real_type Atanc( real_type x );     //!< \f$ \frac{\arctan x}{x} \f$
  real_type Atanc_D( real_type x );   //!< \f$ \frac{\mathrm{d}}{\mathrm{d}x} \frac{\arctan x}{x} \f$
  real_type Atanc_DD( real_type x );  //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^2 \frac{\arctan x}{x} \f$
  real_type Atanc_DDD( real_type x ); //!< \f$ \left(\frac{\mathrm{d}}{\mathrm{d}x}\right)^3 \frac{\arctan x}{x} \f$

  //! Add or remove multiple of \f$ 2\pi \f$ to an angle  in order to put it in the range \f$ [-\pi,\pi]\f$.
  void rangeSymm( real_type & ang );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //! return minumum and maximum of three numbers
  inline
  void
  minmax3(
    real_type   a,
    real_type   b,
    real_type   c,
    real_type & vmin,
    real_type & vmax
  ) {
    vmin = vmax = a;
    if ( b < vmin ) vmin = b;
    else            vmax = b;
    if      ( c < vmin ) vmin = c;
    else if ( c > vmax ) vmax = c;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*!
   * project point `(qx,qy)` to the circle arc passing from `(x0,y0)`
   * with tangent direction `(c0,s0)` curvature `k` length `L`
   */
  real_type
  projectPointOnCircleArc(
    real_type x0,
    real_type y0,
    real_type c0, //!< cos(theta0)
    real_type s0, //!< sin(theta0)
    real_type k,
    real_type L,
    real_type qx,
    real_type qy
  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*!
   * project point `(qx,qy)` to the circle passing from `(x0,y0)`
   * with tangent direction `(c0,s0)` and curvature `k`
   */
  real_type
  projectPointOnCircle(
    real_type x0,
    real_type y0,
    real_type theta0,
    real_type k,
    real_type qx,
    real_type qy
  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*!
   * check if point `(qx,qy)` is inside the circle passing from `(x0,y0)`
   * with tangent direction `(c0,s0)` and curvature `k`
   * \return true if point is inside
   */

  inline
  bool
  pointInsideCircle(
    real_type x0,
    real_type y0,
    real_type c0, //!< cos(theta0)
    real_type s0, //!< sin(theta0)
    real_type k,
    real_type qx,
    real_type qy
  ) {
    real_type cx  = x0 - s0/k;
    real_type cy  = y0 + c0/k;
    real_type dst = hypot( qx - cx, qy - cy );
    return dst*k <= 1;
  }

  /*!
   * Solve the nonlinear system
   *
   * \f[ A x + B y = C \f]
   * \f[ a x^2 + b y^2 = c \f]
   *
   * \param A first parameter of the linear equation
   * \param B second parameter of the linear equation
   * \param C third parameter of the linear equation
   * \param a first parameter of the quadratic equation
   * \param b second parameter of the quadratic equation
   * \param c third parameter of the quadratic equation
   * \return the number of solution 0, 1 or 2
   *
   */

  int_type
  solveLinearQuadratic(
    real_type A,
    real_type B,
    real_type C,
    real_type a,
    real_type b,
    real_type c,
    real_type x[],
    real_type y[]
  );

  /*!
   * Solve the nonlinear system
   *
   * \f[ A x + B y = C \f]
   * \f[ x^2 + y^2 = 1 \f]
   *
   * \param A first parameter of the linear equation
   * \param B second parameter of the linear equation
   * \param C third parameter of the linear equation
   * \return the number of solution 0, 1 or 2
   *
   */

  int_type
  solveLinearQuadratic2(
    real_type A,
    real_type B,
    real_type C,
    real_type x[],
    real_type y[]
  );

  /*!
   * Intersect the parametric arc
   *
   * \f[ x = x_1+\frac{\sin(\kappa_1 s+\theta_1)-sin(\theta_1)}{\kappa_1} \f]
   * \f[ y = y_1+\frac{\cos(\theta_1)-\cos(\kappa_1 s+\theta_1)}{\kappa_1} \f]
   *
   * with the parametric arc
   * \f[ x = x_2+\frac{\sin(\kappa_2 s+\theta_2)-sin(\theta_2)}{\kappa_2} \f]
   * \f[ y = y_2+\frac{\cos(\theta_2)-\cos(\kappa_2 s+\theta_2)}{\kappa_2} \f]
   *
   * \param x1     x-origin of the first arc
   * \param y1     y-origin of the first arc
   * \param theta1 initial angle of the first arc
   * \param kappa1 curvature of the first arc
   * \param x2     x-origin of the second arc
   * \param y2     y-origin of the second arc
   * \param theta2 initial angle of the second arc
   * \param kappa2 curvature of the second arc
   * \param s1     parameter2 of intersection for the first circle arc
   * \param s2     parameter2 of intersection for the second circle arc
   *
   * \return the number of solution 0, 1 or 2
   *
   */

  int_type
  intersectCircleCircle(
    real_type x1,
    real_type y1,
    real_type theta1,
    real_type kappa1,
    real_type x2,
    real_type y2,
    real_type theta2,
    real_type kappa2,
    real_type s1[],
    real_type s2[]
  );

  /*\
   |   ____        _           ____       ____
   |  / ___|  ___ | |_   _____|___ \__  _|___ \
   |  \___ \ / _ \| \ \ / / _ \ __) \ \/ / __) |
   |   ___) | (_) | |\ V /  __// __/ >  < / __/
   |  |____/ \___/|_| \_/ \___|_____/_/\_\_____|
  \*/
  /*!
   * Class that solve a 2x2 linear system using Pseudo inverse
   * to manage singular and near singular cases
   */
  //! Class that solve a 2x2 linear system
  class Solve2x2 {
    int_type  i[2], j[2];
    real_type LU[2][2];
    real_type epsi;
    bool      singular;

  public:

    Solve2x2() : epsi(1e-10) {}
    bool factorize( real_type A[2][2] );
    bool solve( real_type const b[2], real_type x[2] ) const;
  };

  /*!
   *  return +1 = CounterClockwise
   *  return -1 = Clockwise
   *  return  0 = flat
   *
   *  CounterClockwise:
   *    the path P1->P2->P3 turns Counter-Clockwise, i.e.,
   *    the point P3 is located "on the left" of the line P1-P2.
   *  Clockwise:
   *    the path turns Clockwise, i.e.,
   *    the point P3 lies "on the right" of the line P1-P2.
   *  flat:
   *    the point P3 is located on the line segment [P1 P2].
   *
   *  Algorithm from FileExchage geom2d adapated from Sedgewick's book.
   */

  int_type
  isCounterClockwise(
    real_type const P1[2],
    real_type const P2[2],
    real_type const P3[2]
  );

  /*!
   *  return +1 = Inside
   *  return -1 = Outsize
   *  return  0 = on border
   */
  int_type
  isPointInTriangle(
    real_type const pt[2],
    real_type const P1[2],
    real_type const P2[2],
    real_type const P3[2]
  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*!
   * Given a vector `Xvec` and a real number `x` update `lastInterval` in such a way
   * `Xvec[lastInterval] <= x <= Xvec[lastInterval+1]`
   *
   * \param[in,out] lastInterval index of the interval to be updated
   * \param[in]     x            point used to search interval
   * \param[in]     Xvec         vector of the interval extremas
   * \param[in]     npts         dimension of `Xvec`
   */
  void
  updateInterval(
    int_type      & lastInterval,
    real_type       x,
    real_type const Xvec[],
    int_type        npts
  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*\
   |  ____                  ____
   | | __ )  __ _ ___  ___ / ___|   _ _ ____   _____
   | |  _ \ / _` / __|/ _ \ |  | | | | '__\ \ / / _ \
   | | |_) | (_| \__ \  __/ |__| |_| | |   \ V /  __/
   | |____/ \__,_|___/\___|\____\__,_|_|    \_/ \___|
  \*/

  typedef enum {
    G2LIB_LINE=0,
    G2LIB_POLYLINE,
    G2LIB_CIRCLE,
    G2LIB_BIARC,
    G2LIB_BIARC_LIST,
    G2LIB_CLOTHOID,
    G2LIB_CLOTHOID_LIST
  } CurveType;

  extern char const *CurveType_name[];

  typedef std::pair<real_type,real_type> Ipair;
  typedef std::vector<Ipair>             IntersectList;

  /*\
   |   _       _                          _
   |  (_)_ __ | |_ ___ _ __ ___  ___  ___| |_
   |  | | '_ \| __/ _ \ '__/ __|/ _ \/ __| __|
   |  | | | | | ||  __/ |  \__ \  __/ (__| |_
   |  |_|_| |_|\__\___|_|  |___/\___|\___|\__|
  \*/

  class BaseCurve;

  /*!
   * return `true` if the two curves intersect
   *
   * \param[in] C1 first curve
   * \param[in] C2 second curve
   */
  bool
  collision( BaseCurve const & C1, BaseCurve const & C2 );

  /*!
   * return `true` the the two curves intersect
   *
   * \param[in] C1      first curve
   * \param[in] offs_C1 offset of the first curve
   * \param[in] C2      second curve
   * \param[in] offs_C2 offset of the second curve
   */
  bool
  collision_ISO(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2
  );

  /*!
   * return `true` the the two curves intersect
   *
   * \param[in] C1      first curve
   * \param[in] offs_C1 offset of the first curve
   * \param[in] C2      second curve
   * \param[in] offs_C2 offset of the second curve
   */
  inline
  bool
  collision_SAE(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2
  ) {
    return collision_ISO( C1, -offs_C1, C2, -offs_C2 );
  }

  #ifdef G2LIB_COMPATIBILITY_MODE
  /*!
   * return `true` the the two curves intersect
   *
   * \param[in] C1      first curve
   * \param[in] offs_C1 offset of the first curve
   * \param[in] C2      second curve
   * \param[in] offs_C2 offset of the second curve
   */
  inline
  bool
  collision(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2
  ) {
    if ( G2lib::use_ISO ) return collision_ISO( C1, offs_C1, C2, offs_C2 );
    else                  return collision_SAE( C1, offs_C1, C2, offs_C2 );
  }
  #endif

  /*!
   * collect the intersection of the two curve
   *
   * \param[in]  C1          first curve
   * \param[in]  C2          second curve
   * \param[out] ilist       list of the intersection (as parameter on the curves)
   * \param[out] swap_s_vals if true store `(s2,s1)` instead of `(s1,s2)` for each
   *                         intersection
   */
  void
  intersect(
    BaseCurve const & C1,
    BaseCurve const & C2,
    IntersectList   & ilist,
    bool              swap_s_vals
  );

  /*!
   * collect the intersections of the two curve
   *
   * \param[in]  C1          first curve
   * \param[in]  offs_C1     offset of the first curve
   * \param[in]  C2          second curve
   * \param[in]  offs_C2     offset of the second curve
   * \param[out] ilist       list of the intersection (as parameter on the curves)
   * \param[out] swap_s_vals if true store `(s2,s1)` instead of `(s1,s2)` for each
   *                         intersection
   */

  void
  intersect_ISO(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2,
    IntersectList   & ilist,
    bool              swap_s_vals
  );

  /*!
   * collect the intersections of the two curve
   *
   * \param[in]  C1          first curve
   * \param[in]  offs_C1     offset of the first curve
   * \param[in]  C2          second curve
   * \param[in]  offs_C2     offset of the second curve
   * \param[out] ilist       list of the intersection (as parameter on the curves)
   * \param[out] swap_s_vals if true store `(s2,s1)` instead of `(s1,s2)` for each
   *                         intersection
   */

  inline
  void
  intersect_SAE(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2,
    IntersectList   & ilist,
    bool              swap_s_vals
  ) {
    intersect_ISO( C1, -offs_C1, C2, -offs_C2, ilist, swap_s_vals );
  }

  #ifdef G2LIB_COMPATIBILITY_MODE
  /*!
   * collect the intersections of the two curve
   *
   * \param[in]  C1          first curve
   * \param[in]  offs_C1     offset of the first curve
   * \param[in]  C2          second curve
   * \param[in]  offs_C2     offset of the second curve
   * \param[out] ilist       list of the intersection (as parameter on the curves)
   * \param[out] swap_s_vals if true store `(s2,s1)` instead of `(s1,s2)` for each
   *                         intersection
   */

  inline
  void
  intersect(
    BaseCurve const & C1,
    real_type         offs_C1,
    BaseCurve const & C2,
    real_type         offs_C2,
    IntersectList   & ilist,
    bool              swap_s_vals
  ) {
    if ( G2lib::use_ISO ) intersect_ISO( C1, offs_C1, C2, offs_C2, ilist, swap_s_vals );
    else                  intersect_SAE( C1, offs_C1, C2, offs_C2, ilist, swap_s_vals );
  }
  #endif

  //! base classe for all the curve ìs in the library
  class BaseCurve {

    // block default constructor
    BaseCurve( BaseCurve const & );

  protected:
    CurveType _type;

  public:

    BaseCurve( CurveType const & __type )
    : _type(__type)
    {}

    virtual
    ~BaseCurve() {}

    //! \return name of the curve type
    CurveType type() const { return _type; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    //! \return length of the curve
    virtual real_type length() const G2LIB_PURE_VIRTUAL;

    //! \return length of the curve with offset
    virtual real_type length_ISO( real_type offs ) const G2LIB_PURE_VIRTUAL;

    //! \return length of the curve with offset
    real_type
    length_SAE( real_type offs ) const
    { return this->length_ISO(-offs); }

    #ifdef G2LIB_COMPATIBILITY_MODE
    //! \return length of the curve with offset
    real_type
    length( real_type offs ) const
    { return G2lib::use_ISO ? this->length_ISO(offs) : this->length_SAE(offs); }
    #endif

    /*\
     |   _     _
     |  | |__ | |__   _____  __
     |  | '_ \| '_ \ / _ \ \/ /
     |  | |_) | |_) | (_) >  <
     |  |_.__/|_.__/ \___/_/\_\
    \*/

    /*!
     * Compute the bounding box of the curve
     * \param[out] xmin left bottom
     * \param[out] ymin left bottom
     * \param[out] xmax right top
     * \param[out] ymax right top
     */
    virtual
    void
    bbox(
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const G2LIB_PURE_VIRTUAL;

    /*!
     * Compute the bounding box of the curve with offset
     * \param[out] xmin left bottom
     * \param[out] ymin left bottom
     * \param[out] xmax right top
     * \param[out] ymax right top
     */

    virtual
    void
    bbox_ISO(
      real_type   offs,
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const G2LIB_PURE_VIRTUAL;

    /*!
     * Compute the bounding box of the curve
     * \param[out] xmin left bottom
     * \param[out] ymin left bottom
     * \param[out] xmax right top
     * \param[out] ymax right top
     */
    void
    bbox_SAE(
      real_type   offs,
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const {
      this->bbox_ISO( -offs, xmin, ymin, xmax, ymax );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     * Compute the bounding box of the curve
     * \param[out] xmin left bottom
     * \param[out] ymin left bottom
     * \param[out] xmax right top
     * \param[out] ymax right top
     */
    void
    bbox(
      real_type   offs,
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const {
      if ( G2lib::use_ISO ) this->bbox_ISO( offs, xmin, ymin, xmax, ymax );
      else                  this->bbox_SAE( offs, xmin, ymin, xmax, ymax );
    }
    #endif

    /*\
     |   ____             _          _______           _
     |  | __ )  ___  __ _(_)_ __    / / ____|_ __   __| |
     |  |  _ \ / _ \/ _` | | '_ \  / /|  _| | '_ \ / _` |
     |  | |_) |  __/ (_| | | | | |/ / | |___| | | | (_| |
     |  |____/ \___|\__, |_|_| |_/_/  |_____|_| |_|\__,_|
     |              |___/
    \*/

    virtual real_type thetaBegin() const { return this->theta(0); }
    virtual real_type thetaEnd()   const { return this->theta(this->length()); }
    virtual real_type kappaBegin() const { return this->kappa(0); }
    virtual real_type kappaEnd()   const { return this->kappa(this->length()); }
    virtual real_type xBegin()     const { return this->X(0); }
    virtual real_type yBegin()     const { return this->Y(0); }
    virtual real_type xEnd()       const { return this->X(this->length()); }
    virtual real_type yEnd()       const { return this->Y(this->length()); }

    virtual real_type xBegin_ISO( real_type offs ) const { return this->X_ISO(0,offs); }
    virtual real_type yBegin_ISO( real_type offs ) const { return this->Y_ISO(0,offs); }
    virtual real_type xEnd_ISO  ( real_type offs ) const { return this->X_ISO(this->length(),offs); }
    virtual real_type yEnd_ISO  ( real_type offs ) const { return this->Y_ISO(this->length(),offs); }

    real_type xBegin_SAE( real_type offs ) const { return this->xBegin_ISO(-offs); }
    real_type yBegin_SAE( real_type offs ) const { return this->yBegin_ISO(-offs); }
    real_type xEnd_SAE  ( real_type offs ) const { return this->xEnd_ISO(-offs); }
    real_type yEnd_SAE  ( real_type offs ) const { return this->yEnd_ISO(-offs); }

    #ifdef G2LIB_COMPATIBILITY_MODE
    real_type
    xBegin( real_type offs ) const
    { return G2lib::use_ISO ? this->xBegin_ISO(offs) : this->xBegin_SAE(offs); }
    
    real_type
    yBegin( real_type offs ) const
    { return G2lib::use_ISO ? this->yBegin_ISO(offs) : this->yBegin_SAE(offs); }
    
    real_type
    xEnd( real_type offs ) const
    { return G2lib::use_ISO ? this->xEnd_ISO(offs) : this->xEnd_SAE(offs); }
    
    real_type
    yEnd( real_type offs ) const
    { return G2lib::use_ISO ? this->yEnd_ISO(offs) : this->yEnd_SAE(offs); }
    #endif

    virtual real_type tx_Begin() const { return this->tx(0); }
    virtual real_type ty_Begin() const { return this->ty(0); }
    virtual real_type tx_End()   const { return this->tx(this->length()); }
    virtual real_type ty_End()   const { return this->ty(this->length()); }

    virtual real_type nx_Begin_ISO() const { return -this->ty(0); }
    virtual real_type ny_Begin_ISO() const { return this->tx(0); }
    virtual real_type nx_End_ISO()   const { return -this->ty(this->length()); }
    virtual real_type ny_End_ISO()   const { return this->tx(this->length()); }

    real_type nx_Begin_SAE() const { return -nx_Begin_ISO(); }
    real_type ny_Begin_SAE() const { return -ny_Begin_ISO(); }
    real_type nx_End_SAE()   const { return -nx_End_ISO(); }
    real_type ny_End_SAE()   const { return -ny_End_ISO(); }

    #ifdef G2LIB_COMPATIBILITY_MODE
    real_type
    nx_Begin() const
    { return G2lib::use_ISO ? this->nx_Begin_ISO() : this->nx_Begin_SAE(); }

    real_type
    ny_Begin() const
    { return G2lib::use_ISO ? this->ny_Begin_ISO() : this->ny_Begin_SAE(); }

    real_type
    nx_End() const
    { return G2lib::use_ISO ? this->nx_End_ISO() : this->nx_End_SAE(); }

    real_type
    ny_End() const
    { return G2lib::use_ISO ? this->ny_End_ISO() : this->ny_End_SAE(); }
    #endif

    /*\
     |  _   _          _
     | | |_| |__   ___| |_ __ _
     | | __| '_ \ / _ \ __/ _` |
     | | |_| | | |  __/ || (_| |
     |  \__|_| |_|\___|\__\__,_|
    \*/

    virtual real_type theta    ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type theta_D  ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type theta_DD ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type theta_DDD( real_type s ) const G2LIB_PURE_VIRTUAL;

    /*\
     |   _
     |  | | ____ _ _ __  _ __   __ _
     |  | |/ / _` | '_ \| '_ \ / _` |
     |  |   < (_| | |_) | |_) | (_| |
     |  |_|\_\__,_| .__/| .__/ \__,_|
     |            |_|   |_|
    \*/

    real_type kappa   ( real_type s ) const { return theta_D(s); }
    real_type kappa_D ( real_type s ) const { return theta_DD(s); }
    real_type kappa_DD( real_type s ) const { return theta_DDD(s); }

    /*\
     |  _____                   _   _   _
     | |_   _|   __ _ _ __   __| | | \ | |
     |   | |    / _` | '_ \ / _` | |  \| |
     |   | |   | (_| | | | | (_| | | |\  |
     |   |_|    \__,_|_| |_|\__,_| |_| \_|
    \*/

    virtual real_type tx    ( real_type s ) const;
    virtual real_type ty    ( real_type s ) const;
    virtual real_type tx_D  ( real_type s ) const;
    virtual real_type ty_D  ( real_type s ) const;
    virtual real_type tx_DD ( real_type s ) const;
    virtual real_type ty_DD ( real_type s ) const;
    virtual real_type tx_DDD( real_type s ) const;
    virtual real_type ty_DDD( real_type s ) const;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    real_type nx_ISO    ( real_type s ) const { return -ty(s); }
    real_type nx_ISO_D  ( real_type s ) const { return -ty_D(s); }
    real_type nx_ISO_DD ( real_type s ) const { return -ty_DD(s); }
    real_type nx_ISO_DDD( real_type s ) const { return -ty_DDD(s); }

    real_type ny_ISO    ( real_type s ) const { return tx(s); }
    real_type ny_ISO_D  ( real_type s ) const { return tx_D(s); }
    real_type ny_ISO_DD ( real_type s ) const { return tx_DD(s); }
    real_type ny_ISO_DDD( real_type s ) const { return tx_DDD(s); }

    real_type nx_SAE    ( real_type s ) const { return ty(s); }
    real_type nx_SAE_D  ( real_type s ) const { return ty_D(s); }
    real_type nx_SAE_DD ( real_type s ) const { return ty_DD(s); }
    real_type nx_SAE_DDD( real_type s ) const { return ty_DDD(s); }

    real_type ny_SAE    ( real_type s ) const { return -tx(s); }
    real_type ny_SAE_D  ( real_type s ) const { return -tx_D(s); }
    real_type ny_SAE_DD ( real_type s ) const { return -tx_DD(s); }
    real_type ny_SAE_DDD( real_type s ) const { return -tx_DDD(s); }

    #ifdef G2LIB_COMPATIBILITY_MODE
    real_type nx( real_type s ) const
    { return G2lib::use_ISO ? this->nx_ISO(s) : this->nx_SAE(s); }

    real_type nx_D( real_type s ) const
    { return G2lib::use_ISO ? this->nx_ISO_D(s) : this->nx_SAE_D(s); }

    real_type nx_DD( real_type s ) const
    { return G2lib::use_ISO ? this->nx_ISO_DD(s) : this->nx_SAE_DD(s); }

    real_type nx_DDD( real_type s ) const
    { return G2lib::use_ISO ? this->nx_ISO_DDD(s) : this->nx_SAE_DDD(s); }
    
    real_type ny( real_type s ) const
    { return G2lib::use_ISO ? this->ny_ISO(s) : this->ny_SAE(s); }

    real_type ny_D( real_type s ) const
    { return G2lib::use_ISO ? this->ny_ISO_D(s) : this->ny_SAE_D(s); }

    real_type ny_DD( real_type s ) const
    { return G2lib::use_ISO ? this->ny_ISO_DD(s) : this->ny_SAE_DD(s); }

    real_type ny_DDD( real_type s ) const
    { return G2lib::use_ISO ? this->ny_ISO_DDD(s) : this->ny_SAE_DDD(s); }
    #endif

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    tg( real_type s, real_type & tg_x, real_type & tg_y ) const {
      tg_x = this->tx(s);
      tg_y = this->ty(s);
    }

    virtual
    void
    tg_D( real_type s, real_type & tg_x_D, real_type & tg_y_D ) const {
      tg_x_D = this->tx_D(s);
      tg_y_D = this->ty_D(s);
    }

    virtual
    void
    tg_DD( real_type s, real_type & tg_x_DD, real_type & tg_y_DD ) const {
      tg_x_DD = this->tx_DD(s);
      tg_y_DD = this->ty_DD(s);
    }

    virtual
    void
    tg_DDD( real_type s, real_type & tg_x_DDD, real_type & tg_y_DDD ) const {
      tg_x_DDD = this->tx_DDD(s);
      tg_y_DDD = this->ty_DDD(s);
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    void
    nor_ISO( real_type s, real_type & nx, real_type & ny ) const
    { tg( s, ny, nx ); nx = -nx; }

    void
    nor_ISO_D( real_type s, real_type & nx_D, real_type & ny_D ) const
    { tg_D( s, ny_D, nx_D ); nx_D = -nx_D; }

    void
    nor_ISO_DD( real_type s, real_type & nx_DD, real_type & ny_DD ) const
    { tg_DD( s, ny_DD, nx_DD ); nx_DD = -nx_DD; }

    void
    nor_ISO_DDD( real_type s, real_type & nx_DDD, real_type & ny_DDD ) const
    { tg_DDD( s, ny_DDD, nx_DDD ); nx_DDD = -nx_DDD; }

    void
    nor_SAE( real_type s, real_type & nx, real_type & ny ) const
    { tg( s, ny, nx ); ny = -ny; }

    void
    nor_SAE_D( real_type s, real_type & nx_D, real_type & ny_D ) const
    { tg_D( s, ny_D, nx_D ); ny_D = -ny_D; }

    void
    nor_SAE_DD( real_type s, real_type & nx_DD, real_type & ny_DD ) const
    { tg_DD( s, ny_DD, nx_DD ); ny_DD = -ny_DD; }

    void
    nor_SAE_DDD( real_type s, real_type & nx_DDD, real_type & ny_DDD ) const
    { tg_DDD( s, ny_DDD, nx_DDD ); ny_DDD = -ny_DDD; }

    #ifdef G2LIB_COMPATIBILITY_MODE
    void
    nor( real_type s, real_type & nx, real_type & ny ) const {
      if ( G2lib::use_ISO ) this->nor_ISO(s,nx,ny);
      else                  this->nor_SAE(s,nx,ny);
    }

    void
    nor_D( real_type s, real_type & nx_D, real_type & ny_D ) const {
      if ( G2lib::use_ISO ) this->nor_ISO_D(s,nx_D,ny_D);
      else                  this->nor_SAE_D(s,nx_D,ny_D);
    }

    void
    nor_DD( real_type s, real_type & nx_DD, real_type & ny_DD ) const {
      if ( G2lib::use_ISO ) this->nor_ISO_DD(s,nx_DD,ny_DD);
      else                  this->nor_SAE_DD(s,nx_DD,ny_DD);
    }

    void
    nor_DDD( real_type s, real_type & nx_DDD, real_type & ny_DDD ) const {
      if ( G2lib::use_ISO ) this->nor_ISO_DDD(s,nx_DDD,ny_DDD);
      else                  this->nor_SAE_DDD(s,nx_DDD,ny_DDD);
    }
    #endif

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    evaluate(
      real_type   s,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const {
      eval( s, x, y );
      th = theta( s );
      k  = theta_D( s );
    }

    virtual
    void
    evaluate_ISO(
      real_type   s,
      real_type   offs,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const {
      eval_ISO( s, offs, x, y );
      th = theta( s );
      k  = theta_D( s );
      k /= 1+offs*k; // scale curvature
    }

    virtual
    void
    evaluate_SAE(
      real_type   s,
      real_type   offs,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const {
      eval_SAE( s, offs, x, y );
      th = theta( s );
      k  = theta_D( s );
      k /= 1-offs*k; // scale curvature
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    void
    evaluate(
      real_type   s,
      real_type   offs,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const {
      if ( G2lib::use_ISO ) this->evaluate_ISO( s, offs, th, k, x, y );
      else                  this->evaluate_SAE( s, offs, th, k, x, y );
    }
    #endif

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual real_type X    ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type Y    ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type X_D  ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type Y_D  ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type X_DD ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type Y_DD ( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type X_DDD( real_type s ) const G2LIB_PURE_VIRTUAL;
    virtual real_type Y_DDD( real_type s ) const G2LIB_PURE_VIRTUAL;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval( real_type s, real_type & x, real_type & y ) const
    G2LIB_PURE_VIRTUAL;

    virtual
    void
    eval_D( real_type s, real_type & x_D, real_type & y_D ) const
    G2LIB_PURE_VIRTUAL;

    virtual
    void
    eval_DD( real_type s, real_type & x_DD, real_type & y_DD ) const
    G2LIB_PURE_VIRTUAL;

    virtual
    void
    eval_DDD( real_type s, real_type & x_DDD, real_type & y_DDD ) const
    G2LIB_PURE_VIRTUAL;

    /*\
     |         __  __          _
     |   ___  / _|/ _|___  ___| |_
     |  / _ \| |_| |_/ __|/ _ \ __|
     | | (_) |  _|  _\__ \  __/ |_
     |  \___/|_| |_| |___/\___|\__|
    \*/

    virtual real_type X_ISO    ( real_type s, real_type offs ) const;
    virtual real_type Y_ISO    ( real_type s, real_type offs ) const;
    virtual real_type X_ISO_D  ( real_type s, real_type offs ) const;
    virtual real_type Y_ISO_D  ( real_type s, real_type offs ) const;
    virtual real_type X_ISO_DD ( real_type s, real_type offs ) const;
    virtual real_type Y_ISO_DD ( real_type s, real_type offs ) const;
    virtual real_type X_ISO_DDD( real_type s, real_type offs ) const;
    virtual real_type Y_ISO_DDD( real_type s, real_type offs ) const;

    real_type X_SAE    ( real_type s, real_type offs ) const { return this->X_ISO(s,-offs); }
    real_type Y_SAE    ( real_type s, real_type offs ) const { return this->Y_ISO(s,-offs); }
    real_type X_SAE_D  ( real_type s, real_type offs ) const { return this->X_ISO_D(s,-offs); }
    real_type Y_SAE_D  ( real_type s, real_type offs ) const { return this->Y_ISO_D(s,-offs); }
    real_type X_SAE_DD ( real_type s, real_type offs ) const { return this->X_ISO_DD(s,-offs); }
    real_type Y_SAE_DD ( real_type s, real_type offs ) const { return this->Y_ISO_DD(s,-offs); }
    real_type X_SAE_DDD( real_type s, real_type offs ) const { return this->X_ISO_DDD(s,-offs); }
    real_type Y_SAE_DDD( real_type s, real_type offs ) const { return this->Y_ISO_DDD(s,-offs); }

    #ifdef G2LIB_COMPATIBILITY_MODE
    real_type
    X( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->X_ISO( s, offs ) : this->X_SAE( s, offs ); }

    real_type
    Y( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->Y_ISO( s, offs ) : this->Y_SAE( s, offs ); }

    real_type
    X_D( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->X_ISO_D( s, offs ) : this->X_SAE_D( s, offs ); }

    real_type
    Y_D( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->Y_ISO_D( s, offs ) : this->Y_SAE_D( s, offs ); }

    real_type
    X_DD( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->X_ISO_DD( s, offs ) : this->X_SAE_DD( s, offs ); }

    real_type
    Y_DD( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->Y_ISO_DD( s, offs ) : this->Y_SAE_DD( s, offs ); }

    real_type
    X_DDD( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->X_ISO_DDD( s, offs ) : this->X_SAE_DDD( s, offs ); }

    real_type
    Y_DDD( real_type s, real_type offs ) const
    { return G2lib::use_ISO ? this->Y_ISO_DDD( s, offs ) : this->Y_SAE_DDD( s, offs ); }
    #endif

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     *  Compute curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x     coordinate
     * \param[out] y     coordinate
     */

    virtual
    void
    eval_ISO(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const;

    /*!
     *  Compute curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x     coordinate
     * \param[out] y     coordinate
     */

    void
    eval_SAE(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const {
      this->eval_ISO( s, -offs, x, y );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     *  Compute curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x     coordinate
     * \param[out] y     coordinate
     */

    void
    eval(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const {
      if ( G2lib::use_ISO ) this->eval_ISO( s, offs, x, y );
      else                  this->eval_SAE( s, offs, x, y );
    }
    #endif

    /*!
     *  Compute derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_D   coordinate
     * \param[out] y_D   coordinate
     */

    virtual
    void
    eval_ISO_D(
      real_type   s,
      real_type   offs,
      real_type & x_D,
      real_type & y_D
    ) const;

    /*!
     *  Compute derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_D   coordinate
     * \param[out] y_D   coordinate
     */

    void
    eval_SAE_D(
      real_type   s,
      real_type   offs,
      real_type & x_D,
      real_type & y_D
    ) const {
      this->eval_ISO_D( s, -offs, x_D, y_D );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     *  Compute derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_D   coordinate
     * \param[out] y_D   coordinate
     */

    void
    eval_D(
      real_type   s,
      real_type   offs,
      real_type & x_D,
      real_type & y_D
    ) const {
      if ( G2lib::use_ISO ) this->eval_ISO_D( s, offs, x_D, y_D );
      else                  this->eval_SAE_D( s, offs, x_D, y_D );
    }
    #endif

    /*!
     *  Compute second derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DD  coordinate
     * \param[out] y_DD  coordinate
     */

    virtual
    void
    eval_ISO_DD(
      real_type   s,
      real_type   offs,
      real_type & x_DD,
      real_type & y_DD
    ) const;

    /*!
     *  Compute second derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DD  coordinate
     * \param[out] y_DD  coordinate
     */

    void
    eval_SAE_DD(
      real_type   s,
      real_type   offs,
      real_type & x_DD,
      real_type & y_DD
    ) const {
      this->eval_ISO_DD( s, -offs, x_DD, y_DD );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     *  Compute second derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DD  coordinate
     * \param[out] y_DD  coordinate
     */

    void
    eval_DD(
      real_type   s,
      real_type   offs,
      real_type & x_DD,
      real_type & y_DD
    ) const {
      if ( G2lib::use_ISO ) this->eval_ISO_DD( s, offs, x_DD, y_DD );
      else                  this->eval_SAE_DD( s, offs, x_DD, y_DD );
    }
    #endif

    /*!
     *  Compute third derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DDD coordinate
     * \param[out] y_DDD coordinate
     */

    virtual
    void
    eval_ISO_DDD(
      real_type   s,
      real_type   offs,
      real_type & x_DDD,
      real_type & y_DDD
    ) const;

    /*!
     *  Compute third derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DDD coordinate
     * \param[out] y_DDD coordinate
     */

    void
    eval_SAE_DDD(
      real_type   s,
      real_type   offs,
      real_type & x_DDD,
      real_type & y_DDD
    ) const {
      this->eval_ISO_DDD( s, -offs, x_DDD, y_DDD );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     *  Compute third derivative curve at position `s` with offset `offs`
     *
     * \param[in]  s     parameter on the curve
     * \param[in]  offs  offset of the curve
     * \param[out] x_DDD coordinate
     * \param[out] y_DDD coordinate
     */

    void
    eval_DDD(
      real_type   s,
      real_type   offs,
      real_type & x_DDD,
      real_type & y_DDD
    ) const {
      if ( G2lib::use_ISO ) this->eval_ISO_DDD( s, offs, x_DDD, y_DDD );
      else                  this->eval_SAE_DDD( s, offs, x_DDD, y_DDD );
    }
    #endif

    /*\
     |  _                        __
     | | |_ _ __ __ _ _ __  ___ / _| ___  _ __ _ __ ___
     | | __| '__/ _` | '_ \/ __| |_ / _ \| '__| '_ ` _ \
     | | |_| | | (_| | | | \__ \  _| (_) | |  | | | | | |
     |  \__|_|  \__,_|_| |_|___/_|  \___/|_|  |_| |_| |_|
    \*/

    virtual
    void
    translate( real_type tx, real_type ty ) G2LIB_PURE_VIRTUAL;

    virtual
    void
    rotate( real_type angle, real_type cx, real_type cy ) G2LIB_PURE_VIRTUAL;

    virtual
    void
    scale( real_type sc ) G2LIB_PURE_VIRTUAL;

    virtual
    void
    reverse() G2LIB_PURE_VIRTUAL;

    virtual
    void
    changeOrigin( real_type newx0, real_type newy0 ) G2LIB_PURE_VIRTUAL;

    virtual
    void
    trim( real_type s_begin, real_type s_end ) G2LIB_PURE_VIRTUAL;

    /*\
     |   _       _                          _
     |  (_)_ __ | |_ ___ _ __ ___  ___  ___| |_
     |  | | '_ \| __/ _ \ '__/ __|/ _ \/ __| __|
     |  | | | | | ||  __/ |  \__ \  __/ (__| |_
     |  |_|_| |_|\__\___|_|  |___/\___|\___|\__|
    \*/

    bool
    collision( BaseCurve const & C ) const
    { return G2lib::collision( *this, C ); }

    bool
    collision_ISO(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C
    ) const {
      return G2lib::collision_ISO( *this, offs, C, offs_C );
    }

    bool
    collision_SAE(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C
    ) const {
      return G2lib::collision_SAE( *this, offs, C, offs_C );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    bool
    collision(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C
    ) const {
      if ( G2lib::use_ISO )
        return G2lib::collision_ISO( *this, offs, C, offs_C );
      else
        return G2lib::collision_SAE( *this, offs, C, offs_C );
    }
    #endif

    void
    intersect(
      BaseCurve const & C,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const {
      G2lib::intersect( *this, C, ilist, swap_s_vals );
    }

    void
    intersect_ISO(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const {
      G2lib::intersect_ISO( *this, offs, C, offs_C, ilist, swap_s_vals );
    }

    void
    intersect_SAE(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const {
      G2lib::intersect_SAE( *this, offs, C, offs_C, ilist, swap_s_vals );
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    void
    intersect(
      real_type         offs,
      BaseCurve const & C,
      real_type         offs_C,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const {
      if ( G2lib::use_ISO )
        G2lib::intersect_ISO( *this, offs, C, offs_C, ilist, swap_s_vals );
      else
        G2lib::intersect_SAE( *this, offs, C, offs_C, ilist, swap_s_vals );
    }
    #endif

    /*\
     |      _ _     _
     |   __| (_)___| |_ __ _ _ __   ___ ___
     |  / _` | / __| __/ _` | '_ \ / __/ _ \
     | | (_| | \__ \ || (_| | | | | (_|  __/
     |  \__,_|_|___/\__\__,_|_| |_|\___\___|
    \*/

    /*!
     * \param  qx  x-coordinate of the point
     * \param  qy  y-coordinate of the point
     * \param  x   x-coordinate of the projected point on the curve
     * \param  y   y-coordinate of the projected point on the curve
     * \param  s   parameter on the curve of the projection
     * \param  t   curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */
    virtual
    int_type
    closestPoint_ISO(
      real_type   qx,
      real_type   qy,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const G2LIB_PURE_VIRTUAL;

    /*!
     * \param  qx  x-coordinate of the point
     * \param  qy  y-coordinate of the point
     * \param  x   x-coordinate of the projected point on the curve
     * \param  y   y-coordinate of the projected point on the curve
     * \param  s   parameter on the curve of the projection
     * \param  t   curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */
    int_type
    closestPoint_SAE(
      real_type   qx,
      real_type   qy,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const {
      int_type res = this->closestPoint_ISO( qx, qy, x, y, s, t, dst );
      t = -t;
      return res;
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     * \param  qx  x-coordinate of the point
     * \param  qy  y-coordinate of the point
     * \param  x   x-coordinate of the projected point on the curve
     * \param  y   y-coordinate of the projected point on the curve
     * \param  s   parameter on the curve of the projection
     * \param  t   curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */
    int_type
    closestPoint(
      real_type   qx,
      real_type   qy,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const {
      if ( G2lib::use_ISO )
        return this->closestPoint_ISO( qx, qy, x, y, s, t, dst );
      else
        return this->closestPoint_SAE( qx, qy, x, y, s, t, dst );
    }
    #endif

    /*!
     * \param  qx   x-coordinate of the point
     * \param  qy   y-coordinate of the point
     * \param  offs offset of the curve
     * \param  x    x-coordinate of the projected point on the curve
     * \param  y    y-coordinate of the projected point on the curve
     * \param  s    parameter on the curve of the projection
     * \param  t    curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst  distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */
    virtual
    int_type // true if projection is unique and orthogonal
    closestPoint_ISO(
      real_type   qx,
      real_type   qy,
      real_type   offs,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const G2LIB_PURE_VIRTUAL;

    /*!
     * \param  qx   x-coordinate of the point
     * \param  qy   y-coordinate of the point
     * \param  offs offset of the curve
     * \param  x    x-coordinate of the projected point on the curve
     * \param  y    y-coordinate of the projected point on the curve
     * \param  s    parameter on the curve of the projection
     * \param  t    curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst  distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */

    int_type
    closestPoint_SAE(
      real_type   qx,
      real_type   qy,
      real_type   offs,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const {
      int_type res = this->closestPoint_ISO( qx, qy, -offs, x, y, s, t, dst );
      t = -t;
      return res;
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    /*!
     * \param  qx   x-coordinate of the point
     * \param  qy   y-coordinate of the point
     * \param  offs offset of the curve
     * \param  x    x-coordinate of the projected point on the curve
     * \param  y    y-coordinate of the projected point on the curve
     * \param  s    parameter on the curve of the projection
     * \param  t    curvilinear coordinate of the point x,y (if orthogonal projection)
     * \param  dst  distance point projected point
     * \return 1 = point is projected orthogonal
     *         0 = more than one projection (first returned)
     *        -1 = minimum point is not othogonal projection to curve
     */

    int_type
    closestPoint(
      real_type   qx,
      real_type   qy,
      real_type   offs,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const {
      if ( G2lib::use_ISO )
        return this->closestPoint_ISO( qx, qy, offs, x, y, s, t, dst );
      else
        return this->closestPoint_SAE( qx, qy, offs, x, y, s, t, dst );
    }
    #endif

    virtual
    real_type
    distance( real_type qx, real_type qy ) const {
      real_type x, y, s, t, dst;
      closestPoint_ISO( qx, qy, x, y, s, t, dst );
      return dst;
    }

    real_type
    distance_ISO(
      real_type qx,
      real_type qy,
      real_type offs
    ) const {
      real_type x, y, s, t, dst;
      this->closestPoint_ISO( qx, qy, offs, x, y, s, t, dst );
      return dst;
    }

    real_type
    distance_SAE(
      real_type qx,
      real_type qy,
      real_type offs
    ) const {
      real_type x, y, s, t, dst;
      this->closestPoint_SAE( qx, qy, offs, x, y, s, t, dst );
      return dst;
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    virtual
    real_type
    distance(
      real_type qx,
      real_type qy,
      real_type offs
    ) const {
      real_type x, y, s, t, dst;
      this->closestPoint( qx, qy, offs, x, y, s, t, dst );
      return dst;
    }
    #endif

    /*\
     |    __ _           _ ____ _____
     |   / _(_)_ __   __| / ___|_   _|
     |  | |_| | '_ \ / _` \___ \ | |
     |  |  _| | | | | (_| |___) || |
     |  |_| |_|_| |_|\__,_|____/ |_|
    \*/

    bool
    findST_ISO(
      real_type   x,
      real_type   y,
      real_type & s,
      real_type & t
    ) const {
      real_type X, Y, dst;
      int_type icode = this->closestPoint_ISO( x, y, X, Y, s, t, dst );
      return icode >= 0;
    }

    bool
    findST_SAE(
      real_type   x,
      real_type   y,
      real_type & s,
      real_type & t
    ) const {
      real_type X, Y, dst;
      int_type icode = this->closestPoint_SAE( x, y, X, Y, s, t, dst );
      return icode >= 0;
    }

    #ifdef G2LIB_COMPATIBILITY_MODE
    bool
    findST(
      real_type   x,
      real_type   y,
      real_type & s,
      real_type & t
    ) const {
      real_type X, Y, dst;
      int_type icode = this->closestPoint( x, y, X, Y, s, t, dst );
      return icode >= 0;
    }
    #endif

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    virtual
    void
    info( ostream_type & stream ) const G2LIB_PURE_VIRTUAL;

  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*\
   |  __  ____   __  _          _____ _          _
   |  \ \/ /\ \ / / | |_ ___   |_   _| |__   ___| |_ __ _
   |   \  /  \ V /  | __/ _ \    | | | '_ \ / _ \ __/ _` |
   |   /  \   | |   | || (_) |   | | | | | |  __/ || (_| |
   |  /_/\_\  |_|    \__\___/    |_| |_| |_|\___|\__\__,_|
  \*/

  void
  xy_to_guess_angle(
    int_type        npts,
    real_type const x[],
    real_type const y[],
    real_type       theta[],
    real_type       theta_min[],
    real_type       theta_max[],
    real_type       omega[],
    real_type       len[]
  );

}

#endif

///
/// eof: G2lib.hh
///
