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
/// file: Triangle2D.hh
///

#ifndef TRIANGLE2D_HH
#define TRIANGLE2D_HH

#include "G2lib.hh"
#include <vector>

//! Clothoid computations routine
namespace G2lib {

  /*\
   |   _____     _                   _      ____  ____
   |  |_   _| __(_) __ _ _ __   __ _| | ___|___ \|  _ \
   |    | || '__| |/ _` | '_ \ / _` | |/ _ \ __) | | | |
   |    | || |  | | (_| | | | | (_| | |  __// __/| |_| |
   |    |_||_|  |_|\__,_|_| |_|\__, |_|\___|_____|____/
   |                           |___/
  \*/
  //! \brief Class to manage Triangle for BB of clothoid curve

  class Triangle2D {
    real_type p1[2], p2[2], p3[2];
    real_type s0;
    real_type s1;
    int_type  icurve;

  public:

    Triangle2D( Triangle2D const & t )
    { *this = t; }

    Triangle2D( ) {
      this->p1[0] = this->p1[1] =
      this->p2[0] = this->p2[1] =
      this->p3[0] = this->p3[1] = 0;
      this->s0     = 0;
      this->s1     = 0;
      this->icurve = 0;
    }

    Triangle2D(
      real_type x1,  real_type y1,
      real_type x2,  real_type y2,
      real_type x3,  real_type y3,
      real_type _s0, real_type _s1,
      int_type  _icurve
    ) {
      p1[0] = x1; p1[1] = y1;
      p2[0] = x2; p2[1] = y2;
      p3[0] = x3; p3[1] = y3;
      this->s0     = _s0;
      this->s1     = _s1;
      this->icurve = _icurve;
    }

    Triangle2D(
      real_type const _p1[2],
      real_type const _p2[2],
      real_type const _p3[2],
      real_type       _s0,
      real_type       _s1,
      int_type        _icurve
    ) {
      p1[0] = _p1[0]; p1[1] = _p1[1];
      p2[0] = _p2[0]; p2[1] = _p2[1];
      p3[0] = _p3[0]; p3[1] = _p3[1];
      this->s0     = _s0;
      this->s1     = _s1;
      this->icurve = _icurve;
    }

    ~Triangle2D() {}

    Triangle2D const &
    operator = ( Triangle2D const & t ) {
      this->p1[0] = t.p1[0]; this->p1[1] = t.p1[1];
      this->p2[0] = t.p2[0]; this->p2[1] = t.p2[1];
      this->p3[0] = t.p3[0]; this->p3[1] = t.p3[1];
      this->s0     = t.s0;
      this->s1     = t.s1;
      this->icurve = t.icurve;
      return *this;
    }

    void
    build(
      real_type const _p1[2],
      real_type const _p2[2],
      real_type const _p3[2],
      real_type       _s0,
      real_type       _s1,
      int_type        _icurve
    ) {
      p1[0] = _p1[0]; p1[1] = _p1[1];
      p2[0] = _p2[0]; p2[1] = _p2[1];
      p3[0] = _p3[0]; p3[1] = _p3[1];
      this->s0     = _s0;
      this->s1     = _s1;
      this->icurve = _icurve;
    }

    void
    build(
      real_type x1, real_type y1,
      real_type x2, real_type y2,
      real_type x3, real_type y3,
      real_type _s0,
      real_type _s1,
      int_type  _icurve
    ) {
      p1[0] = x1; p1[1] = y1;
      p2[0] = x2; p2[1] = y2;
      p3[0] = x3; p3[1] = y3;
      this->s0     = _s0;
      this->s1     = _s1;
      this->icurve = _icurve;
    }

    int_type  Icurve() const { return this->icurve; }

    real_type x1() const { return this->p1[0]; }
    real_type y1() const { return this->p1[1]; }

    real_type x2() const { return this->p2[0]; }
    real_type y2() const { return this->p2[1]; }

    real_type x3() const { return this->p3[0]; }
    real_type y3() const { return this->p3[1]; }

    real_type S0() const { return this->s0; }
    real_type S1() const { return this->s1; }

    void
    translate( real_type tx, real_type ty ) {
      this->p1[0] += tx; this->p2[0] += tx; this->p3[0] += tx;
      this->p1[1] += ty; this->p2[1] += ty; this->p3[1] += ty;
    }

    void
    rotate( real_type angle, real_type cx, real_type cy );

    void
    scale( real_type sc ) {
      this->p1[0] *= sc; this->p1[1] *= sc;
      this->p2[0] *= sc; this->p2[1] *= sc;
      this->p3[0] *= sc; this->p3[1] *= sc;
    }

    void
    bbox(
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const {
      minmax3( p1[0], p2[0], p3[0], xmin, xmax );
      minmax3( p1[1], p2[1], p3[1], ymin, ymax );
    }

    real_type baricenterX() const { return (p1[0]+p2[0]+p3[0])/3; }
    real_type baricenterY() const { return (p1[1]+p2[1]+p3[1])/3; }

    real_type const * P1() const { return p1; }
    real_type const * P2() const { return p2; }
    real_type const * P3() const { return p3; }

    bool overlap( Triangle2D const & ) const;

    /*!
     *  return +1 = CounterClockwise
     *  return -1 = Clockwise
     *  return  0 = degenerate triangle
     */
    int_type
    isCounterClockwise() const {
      return G2lib::isCounterClockwise( p1, p2, p3 );
    }

    /*!
     *  return +1 = inside
     *  return -1 = outside
     *  return  0 = on the border
     */
    int_type
    isInside( real_type x, real_type y ) const {
      real_type const pt[2] = {x,y};
      return isPointInTriangle( pt, p1, p2, p3 );
    }

    int_type
    isInside( real_type const pt[2] ) const {
      return isPointInTriangle( pt, p1, p2, p3 );
    }

    real_type
    distMin( real_type x, real_type y ) const;

    real_type
    distMax( real_type x, real_type y ) const;

    void
    info( ostream_type & stream ) const
    { stream << "Triangle2D\n" << *this << '\n'; }

    friend
    ostream_type &
    operator << ( ostream_type & stream, Triangle2D const & c );

  };

}

#endif

///
/// eof: Triangle2D.hh
///
