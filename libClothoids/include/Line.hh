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
/// file: Line.hh
///

#ifndef LINE_HH
#define LINE_HH

#include "G2lib.hh"

namespace G2lib {

  /*\
   |   _     _
   |  | |   (_)_ __   ___
   |  | |   | | '_ \ / _ \
   |  | |___| | | | |  __/
   |  |_____|_|_| |_|\___|
  \*/

  //! Class to manage a straight segment
  class LineSegment : public BaseCurve {

    friend class CircleArc;
    friend class PolyLine;

    real_type x0;     //!< initial x coordinate of the line
    real_type y0;     //!< initial y coordinate of the line
    real_type theta0; //!< angle of the line

    real_type c0;     //!< `cos(theta0)`
    real_type s0;     //!< `sin(theta0)`
    real_type L;      //!< length of the segment

  public:

    #include "BaseCurve_using.hxx"

    //explicit
    LineSegment()
    : BaseCurve(G2LIB_LINE)
    , x0(0)
    , y0(0)
    , theta0(0)
    , c0(1)
    , s0(0)
    , L(0)
    {}

    //explicit
    LineSegment( LineSegment const & s )
    : BaseCurve(G2LIB_LINE)
    { copy(s); }

    explicit
    LineSegment( BaseCurve const & C );

    //! construct a circle curve with the standard parameters
    explicit
    LineSegment(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _L
    )
    : BaseCurve(G2LIB_LINE)
    , x0(_x0)
    , y0(_y0)
    , theta0(_theta0)
    , c0(cos(_theta0))
    , s0(sin(_theta0))
    , L(_L)
    {}

    void
    copy( LineSegment const & c ) {
      x0     = c.x0;
      y0     = c.y0;
      theta0 = c.theta0;
      c0     = c.c0;
      s0     = c.s0;
      L      = c.L;
    }

    LineSegment const & operator = ( LineSegment const & s )
    { copy(s); return *this; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    length() const G2LIB_OVERRIDE
    { return L; }

    virtual
    real_type
    length_ISO( real_type ) const G2LIB_OVERRIDE
    { return L; }

    /*\
     |   _     _
     |  | |__ | |__   _____  __
     |  | '_ \| '_ \ / _ \ \/ /
     |  | |_) | |_) | (_) >  <
     |  |_.__/|_.__/ \___/_/\_\
    \*/

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

    /*\
     |   ____             _          _______           _
     |  | __ )  ___  __ _(_)_ __    / / ____|_ __   __| |
     |  |  _ \ / _ \/ _` | | '_ \  / /|  _| | '_ \ / _` |
     |  | |_) |  __/ (_| | | | | |/ / | |___| | | | (_| |
     |  |____/ \___|\__, |_|_| |_/_/  |_____|_| |_|\__,_|
     |              |___/
    \*/

    virtual real_type tx_Begin() const G2LIB_OVERRIDE { return this->c0; }
    virtual real_type ty_Begin() const G2LIB_OVERRIDE { return this->s0; }
    virtual real_type tx_End()   const G2LIB_OVERRIDE { return this->c0; }
    virtual real_type ty_End()   const G2LIB_OVERRIDE { return this->s0; }

    virtual real_type nx_Begin_ISO() const G2LIB_OVERRIDE { return -s0; }
    virtual real_type ny_Begin_ISO() const G2LIB_OVERRIDE { return c0; }
    virtual real_type nx_End_ISO()   const G2LIB_OVERRIDE { return -s0; }
    virtual real_type ny_End_ISO()   const G2LIB_OVERRIDE { return c0; }

    virtual real_type xBegin() const G2LIB_OVERRIDE { return x0; }
    virtual real_type yBegin() const G2LIB_OVERRIDE { return y0; }
    virtual real_type xEnd()   const G2LIB_OVERRIDE { return x0+L*c0; }
    virtual real_type yEnd()   const G2LIB_OVERRIDE { return y0+L*s0; }

    virtual
    real_type
    xBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return x0+offs*nx_Begin_ISO(); }

    virtual
    real_type
    yBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return y0+offs*ny_Begin_ISO(); }

    virtual
    real_type
    xEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return xEnd()+offs*nx_Begin_ISO(); }

    virtual
    real_type
    yEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return yEnd()+offs*ny_Begin_ISO(); }

    /*\
     |  _   _          _
     | | |_| |__   ___| |_ __ _
     | | __| '_ \ / _ \ __/ _` |
     | | |_| | | |  __/ || (_| |
     |  \__|_| |_|\___|\__\__,_|
    \*/

    virtual
    real_type
    theta( real_type ) const G2LIB_OVERRIDE
    { return theta0; }

    virtual
    real_type
    theta_D( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    theta_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    theta_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }


    /*\
     |  _____                   _   _   _
     | |_   _|   __ _ _ __   __| | | \ | |
     |   | |    / _` | '_ \ / _` | |  \| |
     |   | |   | (_| | | | | (_| | | |\  |
     |   |_|    \__,_|_| |_|\__,_| |_| \_|
    \*/

    virtual
    real_type
    tx( real_type ) const G2LIB_OVERRIDE
    { return c0; }

    virtual
    real_type
    ty( real_type ) const G2LIB_OVERRIDE
    { return s0; }

    virtual
    real_type
    tx_D( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    ty_D( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    tx_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    ty_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    tx_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    ty_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    tg( real_type, real_type & tx, real_type & ty ) const G2LIB_OVERRIDE
    { tx = c0; ty = s0; }

    virtual
    void
    tg_D( real_type, real_type & tx_D, real_type & ty_D ) const G2LIB_OVERRIDE
    { tx_D = ty_D = 0; }

    virtual
    void
    tg_DD( real_type, real_type & tx_DD, real_type & ty_DD ) const G2LIB_OVERRIDE
    { tx_DD = ty_DD = 0; }

    virtual
    void
    tg_DDD( real_type, real_type & tx_DDD, real_type & ty_DDD ) const G2LIB_OVERRIDE
    { tx_DDD = ty_DDD = 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    X( real_type s ) const G2LIB_OVERRIDE
    { return x0+s*c0; }

    virtual
    real_type
    Y( real_type s ) const G2LIB_OVERRIDE
    { return y0+s*s0; }

    virtual
    real_type
    X_D( real_type ) const G2LIB_OVERRIDE
    { return c0; }

    virtual
    real_type
    Y_D( real_type ) const G2LIB_OVERRIDE
    { return s0; }

    virtual
    real_type
    X_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    Y_DD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    X_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    Y_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval(
      real_type   s,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE {
      x = x0+s*c0;
      y = y0+s*s0;
    }

    virtual
    void
    eval_D(
      real_type,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE {
      x_D = c0;
      y_D = s0;
    }

    virtual
    void
    eval_DD(
      real_type,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE {
      x_DD = 0;
      y_DD = 0;
    }

    virtual
    void
    eval_DDD(
      real_type,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE {
      x_DDD = 0;
      y_DDD = 0;
    }

    /*\
     |         __  __          _
     |   ___  / _|/ _|___  ___| |_
     |  / _ \| |_| |_/ __|/ _ \ __|
     | | (_) |  _|  _\__ \  __/ |_
     |  \___/|_| |_| |___/\___|\__|
    \*/

    virtual
    real_type
    X_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return x0 + s*c0 + offs*nx_Begin_ISO(); }

    virtual
    real_type
    Y_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return y0 + s*s0 + offs*ny_Begin_ISO(); }

    virtual
    real_type
    X_ISO_D( real_type, real_type ) const G2LIB_OVERRIDE
    { return c0; }

    virtual
    real_type
    Y_ISO_D( real_type, real_type ) const G2LIB_OVERRIDE
    { return s0; }

    virtual
    real_type
    X_ISO_DD( real_type, real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    Y_ISO_DD( real_type, real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    X_ISO_DDD( real_type, real_type ) const G2LIB_OVERRIDE
    { return 0; }

    virtual
    real_type
    Y_ISO_DDD( real_type, real_type ) const G2LIB_OVERRIDE
    { return 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval_ISO(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE {
      x = x0 + s*c0 + offs*nx_Begin_ISO();
      y = y0 + s*s0 + offs*ny_Begin_ISO();
    }

    virtual
    void
    eval_ISO_D(
      real_type,
      real_type,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE {
      x_D = c0;
      y_D = s0;
    }

    virtual
    void
    eval_ISO_DD(
      real_type,
      real_type,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE {
      x_DD = y_DD = 0;
    }

    virtual
    void
    eval_ISO_DDD(
      real_type,
      real_type,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE {
      x_DDD = y_DDD = 0;
    }

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
    scale( real_type sc ) G2LIB_OVERRIDE
    { L *= sc; }

    virtual
    void
    trim( real_type s_begin, real_type s_end ) G2LIB_OVERRIDE {
      x0 += c0 * s_begin;
      y0 += s0 * s_begin;
      L   = s_end - s_begin;
    }

    /*\
     |      _ _     _
     |   __| (_)___| |_ __ _ _ __   ___ ___
     |  / _` | / __| __/ _` | '_ \ / __/ _ \
     | | (_| | \__ \ || (_| | | | | (_|  __/
     |  \__,_|_|___/\__\__,_|_| |_|\___\___|
    \*/

    /*!
     *  \brief compute the point at minimum distance from a point `[x,y]` and the line segment
     *
     *  \param qx  x-coordinate
     *  \param qy  y-coordinate
     *  \param x   x-coordinate of the closest point
     *  \param y   y-coordinate of the closest point
     *  \param s   param of the closest point
     *  \param t   signed distance if projection is orthogonal to segment
     *  \param dst signed distance from the segment
     *  \return 1 = point is projected orthogonal
     *          0 = more than one projection (first returned)
     *         -1 = minimum point is not othogonal projection to curve
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
    { stream << "LineSegment\n" << *this << '\n'; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    void
    build(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _L
    ) {
      x0     = _x0;
      y0     = _y0;
      theta0 = _theta0;
      c0     = cos(_theta0);
      s0     = sin(_theta0);
      L      = _L;
    }

    //! construct a clothoid with the standard parameters
    void
    build_2P(
      real_type _x0,
      real_type _y0,
      real_type _x1,
      real_type _y1
    );

    //! construct a clothoid with the standard parameters
    void
    build_2P( real_type const p0[2], real_type const p1[2] )
    { build_2P( p0[0], p0[1], p1[0], p1[1] ); }

    void
    p1p2( real_type p1[2], real_type p2[2] ) const {
      p1[0] = x0;
      p1[1] = y0;
      p2[0] = x0+L*c0;
      p2[1] = y0+L*s0;
    }

    bool
    intersect(
      LineSegment const & S,
      real_type         & s1,
      real_type         & s2
    ) const;

    bool
    intersect_ISO(
      real_type           offs,
      LineSegment const & S,
      real_type           S_offs,
      real_type         & s1,
      real_type         & s2
    ) const;

    void
    intersect(
      LineSegment const & LS,
      IntersectList     & ilist,
      bool                swap_s_vals
    ) const {
      real_type s1, s2;
      bool ok = this->intersect( LS, s1, s2 );
      if ( ok ) {
        if ( swap_s_vals ) ilist.push_back( Ipair(s2, s1) );
        else               ilist.push_back( Ipair(s1, s2) );
      }
    }

    void
    intersect_ISO(
      real_type           offs,
      LineSegment const & LS,
      real_type           offs_LS,
      IntersectList     & ilist,
      bool                swap_s_vals
    ) const {
      real_type s1, s2;
      bool ok = this->intersect_ISO( offs, LS, offs_LS, s1, s2 );
      if ( ok ) {
        if ( swap_s_vals ) ilist.push_back( Ipair(s2, s1) );
        else               ilist.push_back( Ipair(s1, s2) );
      }
    }

    bool
    collision( LineSegment const & S ) const;

    bool
    collision_ISO(
      real_type           offs,
      LineSegment const & S,
      real_type           S_offs
    ) const;

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

    virtual
    void
    toBS( real_type knots[], real_type Poly[][2] ) const;

    friend
    ostream_type &
    operator << ( ostream_type & stream, LineSegment const & c );

    friend class ClothoidCurve;

  };

}

#endif

///
/// eof: Line.hh
///
