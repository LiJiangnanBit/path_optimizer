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
/// file: Circle.hh
///

#ifndef CIRCLE_HH
#define CIRCLE_HH

#include "G2lib.hh"
#include "Triangle2D.hh"
#include "Line.hh"

//! Clothoid computations routine
namespace G2lib {

  /*\
   |    ____ _          _         _
   |   / ___(_)_ __ ___| | ___   / \   _ __ ___
   |  | |   | | '__/ __| |/ _ \ / _ \ | '__/ __|
   |  | |___| | | | (__| |  __// ___ \| | | (__
   |   \____|_|_|  \___|_|\___/_/   \_\_|  \___|
  \*/

  //! \brief Class to manage Clothoid Curve
  class CircleArc : public BaseCurve {

    friend class Biarc;

    real_type x0;     //!< initial x coordinate of the clothoid
    real_type y0;     //!< initial y coordinate of the clothoid
    real_type theta0; //!< initial angle of the clothoid
    real_type c0;     //!< initial cos(angle) of the clothoid
    real_type s0;     //!< initial sin(angle) of the clothoid
    real_type k;      //!< curvature

    real_type L;      //!< length of the circle segment

  public:

    #include "BaseCurve_using.hxx"

    //explicit
    CircleArc()
    : BaseCurve(G2LIB_CIRCLE)
    , x0(0)
    , y0(0)
    , theta0(0)
    , c0(1)
    , s0(0)
    , k(0)
    , L(0)
    {}

    //explicit
    CircleArc( CircleArc const & s )
    : BaseCurve(G2LIB_CIRCLE)
    { copy(s); }

    //! construct a circle curve with the standard parameters
    explicit
    CircleArc(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _k,
      real_type _L
    )
    : BaseCurve(G2LIB_CIRCLE)
    , x0(_x0)
    , y0(_y0)
    , theta0(_theta0)
    , c0(cos(_theta0))
    , s0(sin(_theta0))
    , k(_k)
    , L(_L)
    {}

    //! construct a circle curve with the standard parameters
    explicit
    CircleArc( LineSegment const & LS )
    : BaseCurve(G2LIB_CIRCLE)
    , x0(LS.xBegin())
    , y0(LS.yBegin())
    , theta0(LS.theta0)
    , c0(LS.c0)
    , s0(LS.s0)
    , k(0)
    , L(LS.length())
    {}

    void
    copy( CircleArc const & c ) {
      this->x0     = c.x0;
      this->y0     = c.y0;
      this->theta0 = c.theta0;
      this->c0     = c.c0;
      this->s0     = c.s0;
      this->k      = c.k;
      this->L      = c.L;
    }

    explicit
    CircleArc( BaseCurve const & C );

    CircleArc const &
    operator = ( CircleArc const & s )
    { copy(s); return *this; }

    //! construct a circle with the standard parameters
    void
    build(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _k,
      real_type _L
    ) {
      x0     = _x0;
      y0     = _y0;
      theta0 = _theta0;
      k      = _k;
      L      = _L;
    }

    //! build a circle by solving the hermite G1 problem
    bool
    build_G1(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _x1,
      real_type _y1
    );

    //! build a circle passing by 3 points
    bool
    build_3P(
      real_type _x0,
      real_type _y0,
      real_type _x1,
      real_type _y1,
      real_type _x2,
      real_type _y2
    );

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    //! get the bounding box triangle (if angle variation less that pi/3)
    bool
    bbTriangle(
      real_type & x0, real_type & y0,
      real_type & x1, real_type & y1,
      real_type & x2, real_type & y2
    ) const;

    //! get the bounding box triangle (if angle variation less that pi/3)
    bool
    bbTriangle_ISO(
      real_type   offs,
      real_type & x0, real_type & y0,
      real_type & x1, real_type & y1,
      real_type & x2, real_type & y2
    ) const;

    //! get the bounding box triangle (if angle variation less that pi/3)
    bool
    bbTriangle_SAE(
      real_type   offs,
      real_type & _x0, real_type & _y0,
      real_type & _x1, real_type & _y1,
      real_type & _x2, real_type & _y2
    ) const {
      return this->bbTriangle_ISO( -offs, _x0, _y0, _x1, _y1, _x2, _y2 );
    }

    bool
    bbTriangle(
      real_type p0[2],
      real_type p1[2],
      real_type p2[2]
    ) const {
      return bbTriangle( p0[0], p0[1], p1[0], p1[1], p2[0], p2[1] );
    }

    bool
    bbTriangle_ISO(
      real_type offs,
      real_type p0[2],
      real_type p1[2],
      real_type p2[2]
    ) const {
      return bbTriangle_ISO( offs, p0[0], p0[1], p1[0], p1[1], p2[0], p2[1] );
    }

    bool
    bbTriangle_SAE(
      real_type offs,
      real_type p0[2],
      real_type p1[2],
      real_type p2[2]
    ) const {
      return bbTriangle_SAE( offs, p0[0], p0[1], p1[0], p1[1], p2[0], p2[1] );
    }

    bool
    bbTriangle(
      Triangle2D & t,
      real_type    ss0    = 0,
      real_type    ss1    = 0,
      int_type     icurve = 0
    ) const {
      real_type p0[2], p1[2], p2[2];
      bool ok = bbTriangle( p0, p1, p2 );
      if ( ok ) t.build( p0, p1, p2, ss0, ss1, icurve );
      return ok;
    }

    bool
    bbTriangle_ISO(
      real_type    offs,
      Triangle2D & t,
      real_type    ss0    = 0,
      real_type    ss1    = 0,
      int_type     icurve = 0
    ) const {
      real_type p0[2], p1[2], p2[2];
      bool ok = bbTriangle_ISO( offs, p0, p1, p2 );
      if ( ok ) t.build( p0, p1, p2, ss0, ss1, icurve );
      return ok;
    }

    bool
    bbTriangle_SAE(
      real_type    offs,
      Triangle2D & t,
      real_type    ss0    = 0,
      real_type    ss1    = 0,
      int_type     icurve = 0
    ) const {
      return this->bbTriangle_ISO( -offs, t, ss0, ss1, icurve );
    }

    void
    bbTriangles(
      std::vector<Triangle2D> & tvec,
      real_type max_angle = m_pi/18,
      real_type max_size  = 1e100,
      int_type  icurve    = 0
    ) const; // 10 degree

    void
    bbTriangles_ISO(
      real_type offs,
      std::vector<Triangle2D> & tvec,
      real_type max_angle = m_pi/18,
      real_type max_size  = 1e100,
      int_type  icurve    = 0
    ) const; // 10 degree

    void
    bbTriangles_SAE(
      real_type offs,
      std::vector<Triangle2D> & tvec,
      real_type max_angle = m_pi/18,
      real_type max_size  = 1e100,
      int_type  icurve    = 0
    ) const {
      this->bbTriangles_ISO( -offs, tvec, max_angle, max_size, icurve );
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    bbox(
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const G2LIB_OVERRIDE;

    virtual
    void
    bbox_ISO(
      real_type   offs,
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    length() const G2LIB_OVERRIDE
    { return L; }

    virtual
    real_type
    length_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return L*(1+k*offs); }

    virtual
    real_type
    thetaBegin() const G2LIB_OVERRIDE
    { return theta0; }

    virtual
    real_type
    xBegin() const G2LIB_OVERRIDE
    { return x0; }

    virtual
    real_type
    yBegin() const G2LIB_OVERRIDE
    { return y0; }

    virtual
    real_type
    tx_Begin() const G2LIB_OVERRIDE
    { return c0; }

    virtual
    real_type
    ty_Begin() const G2LIB_OVERRIDE
    { return s0; }

    virtual
    real_type
    nx_Begin_ISO() const G2LIB_OVERRIDE
    { return s0; }

    virtual
    real_type
    ny_Begin_ISO() const G2LIB_OVERRIDE
    { return -c0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    theta( real_type s ) const G2LIB_OVERRIDE
    { return theta0 + s*k; }

    virtual
    real_type
    theta_D( real_type ) const G2LIB_OVERRIDE
    { return k; }

    virtual
    real_type
    theta_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    theta_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    evaluate(
      real_type   s,
      real_type & th,
      real_type & kappa,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE {
      eval( s, x, y );
      th     = this->theta0 + s*this->k;
      kappa  = this->k;
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual real_type X( real_type s ) const G2LIB_OVERRIDE;
    virtual real_type Y( real_type s ) const G2LIB_OVERRIDE;

    virtual real_type X_D( real_type ) const G2LIB_OVERRIDE;
    virtual real_type Y_D( real_type ) const G2LIB_OVERRIDE;

    virtual real_type X_DD( real_type ) const G2LIB_OVERRIDE;
    virtual real_type Y_DD( real_type ) const G2LIB_OVERRIDE;

    virtual real_type X_DDD( real_type ) const G2LIB_OVERRIDE;
    virtual real_type Y_DDD( real_type ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval(
      real_type   s,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_D(
      real_type,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_DD(
      real_type,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_DDD(
      real_type,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE;

    /*\
     |  _____                   _   _   _
     | |_   _|   __ _ _ __   __| | | \ | |
     |   | |    / _` | '_ \ / _` | |  \| |
     |   | |   | (_| | | | | (_| | | |\  |
     |   |_|    \__,_|_| |_|\__,_| |_| \_|
    \*/

    virtual
    real_type
    tx( real_type s ) const G2LIB_OVERRIDE
    { return cos(theta(s)); }

    virtual
    real_type
    tx_D( real_type s ) const G2LIB_OVERRIDE
    { return -sin(theta(s))*k; }

    virtual
    real_type
    tx_DD( real_type s ) const G2LIB_OVERRIDE
    { return -cos(theta(s))*k*k; }

    virtual
    real_type
    tx_DDD( real_type s ) const G2LIB_OVERRIDE
    { return sin(theta(s))*k*k*k; }

    virtual
    real_type
    ty( real_type s ) const G2LIB_OVERRIDE
    { return sin(theta(s)); }

    virtual
    real_type
    ty_D( real_type s ) const G2LIB_OVERRIDE
    { return cos(theta(s))*k; }

    virtual
    real_type
    ty_DD( real_type s ) const G2LIB_OVERRIDE
    { return -sin(theta(s))*k*k; }

    virtual
    real_type
    ty_DDD( real_type s ) const G2LIB_OVERRIDE
    { return -cos(theta(s))*k*k*k; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    tg( real_type s, real_type & tx, real_type & ty ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_D( real_type s, real_type & tx_D, real_type & ty_D ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_DD( real_type s, real_type & tx_DD, real_type & ty_DD ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_DDD( real_type s, real_type & tx_DDD, real_type & ty_DDD ) const G2LIB_OVERRIDE;

    /*\
     |  _                        __
     | | |_ _ __ __ _ _ __  ___ / _| ___  _ __ _ __ ___
     | | __| '__/ _` | '_ \/ __| |_ / _ \| '__| '_ ` _ \
     | | |_| | | (_| | | | \__ \  _| (_) | |  | | | | | |
     |  \__|_|  \__,_|_| |_|___/_|  \___/|_|  |_| |_| |_|
    \*/

    virtual
    void
    translate( real_type tx, real_type ty ) G2LIB_OVERRIDE
    { x0 += tx; y0 += ty; }

    virtual
    void
    rotate( real_type angle, real_type cx, real_type cy ) G2LIB_OVERRIDE;

    virtual
    void
    reverse() G2LIB_OVERRIDE;

    virtual
    void
    changeOrigin( real_type newx0, real_type newy0 ) G2LIB_OVERRIDE
    { x0 = newx0; y0 = newy0; }

    virtual
    void
    scale( real_type s ) G2LIB_OVERRIDE;

    virtual
    void
    trim( real_type s_begin, real_type s_end ) G2LIB_OVERRIDE;

    /*\
     |        _                     _   ____       _       _
     |    ___| | ___  ___  ___  ___| |_|  _ \ ___ (_)_ __ | |_
     |   / __| |/ _ \/ __|/ _ \/ __| __| |_) / _ \| | '_ \| __|
     |  | (__| | (_) \__ \  __/\__ \ |_|  __/ (_) | | | | | |_
     |   \___|_|\___/|___/\___||___/\__|_|   \___/|_|_| |_|\__|
     |
    \*/

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
    ) const G2LIB_OVERRIDE;

    virtual
    int_type
    closestPoint_ISO(
      real_type   qx,
      real_type   qy,
      real_type   offs,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst
    ) const G2LIB_OVERRIDE;

    virtual
    void
    info( ostream_type & stream ) const G2LIB_OVERRIDE
    { stream << "CircleArc\n" << *this << '\n'; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*\
     |             _ _ _     _
     |    ___ ___ | | (_)___(_) ___  _ __
     |   / __/ _ \| | | / __| |/ _ \| '_ \
     |  | (_| (_) | | | \__ \ | (_) | | | |
     |   \___\___/|_|_|_|___/_|\___/|_| |_|
    \*/

    bool
    collision( CircleArc const & ) const;

    bool
    collision_ISO(
      real_type         offs,
      CircleArc const & C,
      real_type         offs_obj
    ) const;

    /*\
     |   _       _                          _
     |  (_)_ __ | |_ ___ _ __ ___  ___  ___| |_
     |  | | '_ \| __/ _ \ '__/ __|/ _ \/ __| __|
     |  | | | | | ||  __/ |  \__ \  __/ (__| |_
     |  |_|_| |_|\__\___|_|  |___/\___|\___|\__|
    \*/

    void
    intersect(
      CircleArc const & obj,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const;

    void
    intersect_ISO(
      real_type         offs,
      CircleArc const & C,
      real_type         offs_obj,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const;

    real_type sinTheta0() const { return sin(theta0); }
    real_type cosTheta0() const { return cos(theta0); }
    real_type curvature() const { return k; }

    // return the length of the arc that can approximated
    // by a line segment
    real_type lenTolerance( real_type tol ) const;

    real_type
    delta_theta() const
    { return L*k; }

    real_type
    thetaTotalVariation() const
    { return std::abs(L*k); }

    real_type
    thetaMinMax( real_type & thMin, real_type & thMax ) const;

    real_type
    deltaTheta() const
    { real_type thMin, thMax; return thetaMinMax( thMin, thMax ); }

    void
    changeCurvilinearOrigin( real_type s0, real_type newL );

    void
    center( real_type & cx, real_type & cy ) const;

    real_type ray() const { return 1/std::abs(k); }

    /*\
     |   _   _ _   _ ____  ____ ____
     |  | \ | | | | |  _ \| __ ) ___|
     |  |  \| | | | | |_) |  _ \___ \
     |  | |\  | |_| |  _ <| |_) |__) |
     |  |_| \_|\___/|_| \_\____/____/
    \*/

    void
    paramNURBS( int_type & n_knots, int_type & n_pnts ) const;

    void
    toNURBS( real_type knots[], real_type Poly[][3] ) const;

    friend
    ostream_type &
    operator << ( ostream_type & stream, CircleArc const & c );

    friend class ClothoidCurve;

  };

}

#endif

///
/// eof: Circle.hh
///
