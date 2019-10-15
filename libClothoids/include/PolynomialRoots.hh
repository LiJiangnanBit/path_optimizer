/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2014                                                      |
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

#ifndef POLYNOMIAL_ROOTS_HH
#define POLYNOMIAL_ROOTS_HH

#include <cmath>
#include <cfloat>
#include <complex>
#include <iostream>

/*!

\mainpage

Polynomial Roots Solver
=======================

Port to C++ of Jenkins-Traub real
polynomial root finder and the algorithm
of Norbert Flocke for polynoms up to degree 4.

[Doxygen documentation: http://ebertolazzi.github.io/quarticRootsFlocke/](http://ebertolazzi.github.io/quarticRootsFlocke/)

[Repository: https://github.com/ebertolazzi/quarticRootsFlocke](https://github.com/ebertolazzi/quarticRootsFlocke)

Usage
-----

~~~~~~~~~~~~~~~~~~~~~
  #include "PolynomialRoots.hh"
  ....

  double coeffs[] = { 8, -8, 16,-16, 8,-8 }; // polynomial coeffs

  double zeror[5], zeroi[5];
  int    info[5];
  int    degree = 5;

  int ok = PolynomialRoots::roots( coeffs, degree, zeror, zeroi ); // ok < 0 failed
  cout << " ok = " << ok << '\n' ;
  for ( int i = 0 ; i < degree ; ++i )
    cout << zeror[i] << " + I* " << zeroi[i] << '\n';
~~~~~~~~~~~~~~~~~~~~~

To solve quadratic, cubic or quartic use specialized classes

~~~~
  Quadratic qsolve(a,b,c);
  qsolve.info(cout);

  Cubic csolve(a,b,c,d);
  csolve.info(cout);

  Quartic q4(a,b,c,d,e);
  q4.info(cout);
~~~~

look at the class definition to see how to access the computed roots.

References
----------

- Algorithm 954: An Accurate and Efficient Cubic and Quartic
  Equation Solver for Physical Applications, ACM TOMS,
  vol 41, n.4, 2015

- A Three-Stage Algorithm for Real Polynomials Using Quadratic Iteration
  M. A.   Jenkins and J. F. Traub
  SIAM Journal on Numerical Analysis,
  Vol. 7, No. 4 (Dec., 1970), pp. 545-566

Author
------

Enrico Bertolazzi<br>
Dipartimento di Ingegneria Industriale<br>
Universita` degli Studi di Trento<br>
email: enrico.bertolazzi@unitn.it

*/

//! implementation of Flocke algorithm for roots of 3rd and 4th degree polynomials
namespace PolynomialRoots {

  typedef double valueType;
  typedef int    indexType;
  typedef std::complex<valueType> complexType;

  //! check if cloating point number `x` is zero
  static
  inline
  bool
  isZero( valueType x )
  { return FP_ZERO == std::fpclassify(x); }

  //! check if cloating point number `x` is finite
  static
  inline
  bool
  isInfinite( valueType x )
  { return FP_INFINITE == std::fpclassify(x); }

  //! check if cloating point number `x` is Not A Number
  static
  inline
  bool
  isNaN( valueType x )
  { return FP_NAN == std::fpclassify(x); }

  //! check if cloating point number `x` is regural (i.e. finite and not NaN)
  static
  inline
  bool
  isRegular( valueType x )
  { return !( FP_INFINITE == std::fpclassify(x) ||
              FP_NAN      == std::fpclassify(x) ); }

  //! evaluate real polynomial
  valueType
  evalPoly(
    valueType const op[],
    indexType       Degree,
    valueType       x
  );

  //! evaluate real polynomial with complex value
  std::complex<valueType>
  evalPolyC(
    valueType const                 op[],
    indexType                       Degree,
    std::complex<valueType> const & x
  );

  //! find roots of a generic polinomial using Jenkins-Traub method
  int
  roots(
    valueType const op[],
    indexType       Degree,
    valueType       zeror[],
    valueType       zeroi[]
  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*\
   |    ___                  _           _   _
   |   / _ \ _   _  __ _  __| |_ __ __ _| |_(_) ___
   |  | | | | | | |/ _` |/ _` | '__/ _` | __| |/ __|
   |  | |_| | |_| | (_| | (_| | | | (_| | |_| | (__
   |   \__\_\\__,_|\__,_|\__,_|_|  \__,_|\__|_|\___|
   |
   |  A * x^2 + B * x + C
  \*/
  //! Quadratic polynomial class
  class Quadratic {
    valueType ABC[3];
    valueType r0, r1;
    indexType nrts;
    bool      cplx;
    bool      dblx;

    void findRoots();

  public:

    Quadratic() : nrts(0), cplx(false), dblx(false) {}
    Quadratic( valueType _a, valueType _b, valueType _c )
    : nrts(0), cplx(false), dblx(false) {
      valueType & A = ABC[0];
      valueType & B = ABC[1];
      valueType & C = ABC[2];
      A = _a; B = _b; C = _c;
      findRoots();
    }

    //! compute the roots of quadratic polynomial \f$ a x^2 + b x + c \f$
    /*!
     *
     * \param[in] _a coefficient of \f$ x^2 \f$
     * \param[in] _b coefficient of \f$ x   \f$
     * \param[in] _c coefficient of \f$ x^0   \f$
     *
     */
    void
    setup( valueType _a, valueType _b, valueType _c ) {
      valueType & A = ABC[0];
      valueType & B = ABC[1];
      valueType & C = ABC[2];
      A = _a; B = _b; C = _c;
      findRoots();
    }

    indexType numRoots()     const { return nrts; } //!< number of found roots
    bool      complexRoots() const { return cplx; } //!< are roots complex?
    bool      doubleRoot()   const { return dblx; } //!< are roots a double root?

    //! get the real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of real roots, 0, 1 or 2
     */
    indexType getRealRoots( valueType r[] ) const;

    //! get positive real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of positive real roots, 0, 1 or 2
     */
    indexType getPositiveRoots( valueType r[] ) const;

    //! get negative real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of negative real roots, 0, 1 or 2
     */
    indexType getNegativeRoots( valueType r[] ) const;

    valueType real_root0() const { return r0; } //!< first real root
    valueType real_root1() const { return r1; } //!< second real root

    complexType
    root0() const
    { return cplx ? complexType(r0,r1) : complexType(r0,0); }

    complexType
    root1() const
    { return cplx ? complexType(r0,-r1) : complexType(r1,0); }

    void
    getRoot0( valueType & re, valueType & im ) const {
      if ( cplx ) { re = r0; im = r1; }
      else        { re = r0; im = 0;  }
    }

    void
    getRoot0( complexType & r ) const {
      if ( cplx ) r = complexType(r0,r1);
      else        r = complexType(r0,0);
    }

    void
    getRoot1( valueType & re, valueType & im ) const {
      if ( cplx ) { re = r0; im = -r1; }
      else        { re = r1; im = 0;   }
    }

    void
    getRoot1( complexType & r ) const {
      if ( cplx ) r = complexType(r0,-r1);
      else        r = complexType(r1,0);
    }

    //! evalute the quadratic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$
     * \return the value \f$ p(x) \f$
     */
    valueType
    eval( valueType x ) const
    { return evalPoly( ABC, 2, x ); }

    //! evalute the quadratic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$, x complex
     * \return the value \f$ p(x) \f$
     */
    complexType
    eval( complexType const & x ) const
    { return evalPolyC( ABC, 2, x ); }

    //! evalute the polynomial with its derivative
    /*!
     * \param x   value where compute \f$ p(x) \f$
     * \param p   value \f$ p(x) \f$
     * \param dp  value \f$ p'(x) \f$
     */
    void eval( valueType x, valueType & p, valueType & dp ) const;

    //! print info of the roots of the polynomial
    void
    info( std::ostream & s ) const;

    //! check tolerenace and quality of the computed roots
    bool
    check( std::ostream & s ) const;

  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*\
   |    ____      _     _
   |   / ___|   _| |__ (_) ___
   |  | |  | | | | '_ \| |/ __|
   |  | |__| |_| | |_) | | (__
   |   \____\__,_|_.__/|_|\___|
   |
   |  A * x^3 + B * x^2 + C * x + D
  \*/
  //! Cubic polynomial class
  class Cubic {
    valueType ABCD[4];
    valueType r0, r1, r2;
    indexType nrts, iter;
    bool      cplx; // complex root
    bool      dblx; // double root
    bool      trpx; // triple root

    void findRoots();

  public:

    Cubic() : nrts(0), iter(0), cplx(false), trpx(false) {}
    Cubic( valueType _a, valueType _b, valueType _c, valueType _d )
    : nrts(0), iter(0), cplx(false), trpx(false) {
      valueType & A = ABCD[0];
      valueType & B = ABCD[1];
      valueType & C = ABCD[2];
      valueType & D = ABCD[3];
      A = _a; B = _b; C = _c; D = _d;
      findRoots();
    }

    //! compute the roots of cubic polynomial \f$ a x^3 + b x^2 + c x + d \f$
    /*!
     *
     * \param[in] _a coefficient of \f$ x^3 \f$
     * \param[in] _b coefficient of \f$ x^2 \f$
     * \param[in] _c coefficient of \f$ x   \f$
     * \param[in] _d coefficient of \f$ x^0 \f$
     *
     */
    void
    setup( valueType _a, valueType _b, valueType _c, valueType _d ) {
      valueType & A = ABCD[0];
      valueType & B = ABCD[1];
      valueType & C = ABCD[2];
      valueType & D = ABCD[3];
      A = _a; B = _b; C = _c; D = _d;
      findRoots();
    }

    indexType numRoots()     const { return nrts; } //!< number of found roots
    bool      complexRoots() const { return cplx; } //!< has complex roots?
    bool      doubleRoot()   const { return dblx; } //!< has a double root?
    bool      tripleRoot()   const { return trpx; } //!< has a triple root?

    //! get the real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of real roots, 0, 1, 2 or 3
     */
    indexType getRealRoots( valueType r[] ) const;

    //! get positive real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of positive real roots, 0, 1, 2 or 3
     */
    indexType getPositiveRoots( valueType r[] ) const;

    //! get negative real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of negative real roots, 0, 1, 2 or 3
     */
    indexType getNegativeRoots( valueType r[] ) const;

    valueType real_root0() const { return r0; } //!< first real real
    valueType real_root1() const { return r1; } //!< second real real
    valueType real_root2() const { return r2; } //!< third real real

    complexType
    root0() const
    { return cplx ? complexType(r0,r1) : complexType(r0,0); }

    complexType
    root1() const
    { return cplx ? complexType(r0,-r1) : complexType(r1,0); }

    complexType
    root2() const
    { return complexType(r2,0); }

    void
    getRoot0( valueType & re, valueType & im ) const {
      if ( cplx ) { re = r0; im = r1; }
      else        { re = r0; im = 0;  }
    }

    void
    getRoot0( complexType & r ) const {
      if ( cplx ) r = complexType(r0,r1);
      else        r = complexType(r0,0);
    }

    void
    getRoot1( valueType & re, valueType & im ) const {
      if ( cplx ) { re = r0; im = -r1; }
      else        { re = r1; im = 0;   }
    }

    void
    getRoot1( complexType & r ) const {
      if ( cplx ) r = complexType(r0,-r1);
      else        r = complexType(r1,0);
    }

    void
    getRoot2( valueType & re, valueType & im ) const
    { re = r2; im = 0; }

    void
    getRoot2( complexType & r ) const
    { r = complexType(r2,0); }

    //! evalute the cubic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$
     * \return the value \f$ p(x) \f$
     */
    valueType
    eval( valueType x ) const
    { return evalPoly( ABCD, 3, x ); }

    //! evalute the cubic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$, x complex
     * \return the value \f$ p(x) \f$
     */
    complexType
    eval( complexType const & x ) const
    { return evalPolyC( ABCD, 3, x ); }

    //! evalute the polynomial with its derivative
    /*!
     * \param x   value where compute \f$ p(x) \f$
     * \param p   value \f$ p(x) \f$
     * \param dp  value \f$ p'(x) \f$
     */
    void eval( valueType x, valueType & p, valueType & dp ) const;

    //! print info of the roots of the polynomial
    void
    info( std::ostream & s ) const;

    //! check tolerenace and quality of the computed roots
    bool
    check( std::ostream & s ) const;

  };

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  /*\
   |    ___                   _   _
   |   / _ \ _   _  __ _ _ __| |_(_) ___
   |  | | | | | | |/ _` | '__| __| |/ __|
   |  | |_| | |_| | (_| | |  | |_| | (__
   |   \__\_\\__,_|\__,_|_|   \__|_|\___|
   |
   |  A * x^3 + B * x^2 + C * x + D
  \*/
  //! Quartic polynomial class
  class Quartic {
    valueType ABCDE[5];
    valueType r0, r1, r2, r3;
    indexType iter, nreal, ncplx;

    void findRoots();

    bool cplx0() const { return ncplx > 0; }
    bool cplx1() const { return ncplx > 0; }
    bool cplx2() const { return ncplx > 2; }
    bool cplx3() const { return ncplx > 2; }

  public:

    Quartic() : iter(0), nreal(0), ncplx(0) {}
    Quartic(
      valueType _a,
      valueType _b,
      valueType _c,
      valueType _d,
      valueType _e
    )
    : iter(0), nreal(0), ncplx(0) {
      valueType & A = ABCDE[0];
      valueType & B = ABCDE[1];
      valueType & C = ABCDE[2];
      valueType & D = ABCDE[3];
      valueType & E = ABCDE[4];
      A = _a; B = _b; C = _c; D = _d; E = _e;
      findRoots();
    }

    //! compute the roots of quartic polynomial \f$ a x^4 + b x^3 + c x^2 + d x + e \f$
    /*!
     *
     * \param[in] _a coefficient of \f$ x^4 \f$
     * \param[in] _b coefficient of \f$ x^3 \f$
     * \param[in] _c coefficient of \f$ x^2  \f$
     * \param[in] _d coefficient of \f$ x   \f$
     * \param[in] _e coefficient of \f$ x^0 \f$
     *
     */
    void
    setup(
      valueType _a,
      valueType _b,
      valueType _c,
      valueType _d,
      valueType _e
    ) {
      valueType & A = ABCDE[0];
      valueType & B = ABCDE[1];
      valueType & C = ABCDE[2];
      valueType & D = ABCDE[3];
      valueType & E = ABCDE[4];
      A = _a; B = _b; C = _c; D = _d; E = _e;
      findRoots();
    }

    indexType numRoots()        const { return nreal+ncplx; } //!< number of found roots
    indexType numRealRoots()    const { return nreal; } //!< number of real roots
    indexType numComplexRoots() const { return ncplx; } //!< number of complex roots

    //! get the real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of real roots, 0, 1, 2, 3 or 4
     */
    indexType getRealRoots( valueType r[] ) const;

    //! get positive real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of positive real roots, 0, 1, 2, 3 or 4
     */
    indexType getPositiveRoots( valueType r[] ) const;

    //! get negative real roots
    /*!
     * \param[out] r vector that will be filled with the real roots
     * \return the total number of negative real roots, 0, 1, 2, 3 or 4
     */
    indexType getNegativeRoots( valueType r[] ) const;

    valueType real_root0() const { return r0; } //!< first real real
    valueType real_root1() const { return r1; } //!< second real real
    valueType real_root2() const { return r2; } //!< third real real
    valueType real_root3() const { return r3; } //!< fourth real real

    complexType
    root0() const
    { return cplx0() ? complexType(r0,r1) : complexType(r0,0); }

    complexType
    root1() const
    { return cplx1() ? complexType(r0,-r1) : complexType(r1,0); }

    complexType
    root2() const
    { return cplx2() ? complexType(r2,r3) : complexType(r2,0); }

    complexType
    root3() const
    { return cplx3() ? complexType(r2,-r3) : complexType(r3,0); }

    void
    getRoot0( valueType & re, valueType & im ) const {
      if ( cplx0() ) { re = r0; im = r1; }
      else           { re = r0; im = 0;  }
    }

    void
    getRoot0( complexType & r ) const {
      if ( cplx0() ) r = complexType(r0,r1);
      else           r = complexType(r0,0);
    }

    void
    getRoot1( valueType & re, valueType & im ) const {
      if ( cplx1() ) { re = r0; im = -r1; }
      else           { re = r1; im = 0;   }
    }

    void
    getRoot1( complexType & r ) const {
      if ( cplx1() ) r = complexType(r0,-r1);
      else           r = complexType(r1,0);
    }

    void
    getRoot2( valueType & re, valueType & im ) const {
      if ( cplx2() ) { re = r2; im = r3; }
      else           { re = r2; im = 0;  }
    }

    void
    getRoot2( complexType & r ) const {
      if ( cplx2() ) r = complexType(r2,r3);
      else           r = complexType(r2,0);
    }

    void
    getRoot3( valueType & re, valueType & im ) const {
      if ( cplx3() ) { re = r2; im = -r3; }
      else           { re = r3; im = 0;   }
    }

    void
    getRoot3( complexType & r ) const {
      if ( cplx3() ) r = complexType(r2,-r3);
      else           r = complexType(r3,0);
    }

    //! evalute the quartic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$, x complex
     * \return the value \f$ p(x) \f$
     */
    valueType
    eval( valueType x ) const
    { return evalPoly( ABCDE, 4, x ); }

    //! evalute the quartic polynomial
    /*!
     * \param x   value where compute \f$ p(x) \f$, x complex
     * \return the value \f$ p(x) \f$
     */
    complexType
    eval( complexType const & x ) const
    { return evalPolyC( ABCDE, 4, x ); }

    //! print info of the roots of the polynomial
    void
    info( std::ostream & s ) const;

    //! check tolerenace and quality of the computed roots
    bool
    check( std::ostream & s ) const;

  };

  /*\
   |   _   _ _   _ _
   |  | | | | |_(_) |___
   |  | | | | __| | / __|
   |  | |_| | |_| | \__ \
   |   \___/ \__|_|_|___/
  \*/

  // x^3 + a*x^2 + b*x + c
  static
  inline
  valueType
  evalMonicCubic(
    valueType x,
    valueType a,
    valueType b,
    valueType c
  ) {
    valueType p;
    p = x + a;
    p = p * x + b;
    p = p * x + c;
    return p;
  }

  static
  inline
  void
  evalMonicCubic(
    valueType   x,
    valueType   a,
    valueType   b,
    valueType   c,
    valueType & p,
    valueType & dp
  ) {
    p  = x + a;
    dp = x + p;
    p  = p  * x + b;
    dp = dp * x + p;
    p  = p  * x + c;
  }

  // 3*x^2 + 2*a*x + b
  // 6*x + 2*a
  static
  inline
  void
  evalMonicCubic(
    valueType   x,
    valueType   a,
    valueType   b,
    valueType   c,
    valueType & p,
    valueType & dp,
    valueType & ddp
  ) {
    p   = x + a;
    dp  = x + p;      // 2*x + a
    p   = p  * x + b; // x^2 + a * x + b
    ddp = 2*(x + dp);
    dp  = dp * x + p;
    p   = p  * x + c;
  }

  // x^4 + a*x^3 + b*x^2 + c*x + d
  static
  inline
  valueType
  evalMonicQuartic(
    valueType x,
    valueType a,
    valueType b,
    valueType c,
    valueType d
  ) {
    valueType p;
    p = x + a;     // x + a
    p = p * x + b; // x^2+ a*x + b
    p = p * x + c; // x^3+ a*x^2 + b*x + c
    p = p * x + d; // x^4+ a*x^3 + b*x^2 + c*x + d
    return p;
  }

  static
  inline
  void
  evalMonicQuartic(
    valueType   x,
    valueType   a,
    valueType   b,
    valueType   c,
    valueType   d,
    valueType & p,
    valueType & dp
  ) {
    p  = x + a;      // x + a
    dp = x + p;      // 2*x + a
    p  = p  * x + b; // x^2+ a*x + b
    dp = dp * x + p; // 3*x^2 + 2*a*x + b
    p  = p  * x + c; // x^3+ a*x^2 + b*x + c
    dp = dp * x + p; // 4*x^3 + 3*a*x^2 + 2*b*x + c
    p  = p  * x + d; // x^4+ a*x^3 + b*x^2 + c*x + d
  }

  static
  inline
  void
  evalMonicQuartic(
    valueType   x,
    valueType   a,
    valueType   b,
    valueType   c,
    valueType   d,
    valueType & p,
    valueType & dp,
    valueType & ddp
  ) {
    // p_{n+1}(x)   = x * p_{n}(x) + b_{n}
    // p'_{n+1}(x)  = x * p'_{n}(x) + p_{n}(x)
    // p''_{n+1}(x) = x * p''_{n}(x) + 2*p'_{n}(x)
    // ddp = 0;
    // dp  = 1;
    p   = x + a;     // x + a

    ddp = 2;
    dp  = x + p;
    p   = p * x + b;

    ddp = ddp * x + 2 * dp;
    dp  = dp * x + p;
    p   = p * x + c;

    ddp = ddp * x + 2 * dp;
    dp  = dp * x + p;
    p   = p * x + d;
  }

}

#endif
