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
/// file: Clothoid.hh
///

#ifndef CLOTHOID_HH
#define CLOTHOID_HH

#include "Fresnel.hh"
#include "Line.hh"
#include "Circle.hh"
#include "Triangle2D.hh"
#include "AABBtree.hh"

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>

//! Clothoid computations routine
namespace G2lib {

  using std::vector;

  /*\
   |    ____ _       _   _           _     _  ____
   |   / ___| | ___ | |_| |__   ___ (_) __| |/ ___|   _ _ ____   _____
   |  | |   | |/ _ \| __| '_ \ / _ \| |/ _` | |  | | | | '__\ \ / / _ \
   |  | |___| | (_) | |_| | | | (_) | | (_| | |__| |_| | |   \ V /  __/
   |   \____|_|\___/ \__|_| |_|\___/|_|\__,_|\____\__,_|_|    \_/ \___|
  \*/
  //! \brief Class to manage Clothoid Curve
  class ClothoidCurve : public BaseCurve {
    friend class ClothoidList;
  private:

    ClothoidData CD;  //!< clothoid data
    real_type    L;   //!< length of clothoid segment

    void
    optimized_sample_internal_ISO(
      real_type           s_begin,
      real_type           s_end,
      real_type           offs,
      real_type           ds,
      real_type           max_angle,
      vector<real_type> & s
    ) const;

    void
    bbTriangles_internal_ISO(
      real_type            offs,
      vector<Triangle2D> & tvec,
      real_type            s0,
      real_type            s1,
      real_type            max_angle,
      real_type            max_size,
      int_type             icurve
    ) const;

    void
    closestPoint_internal_ISO(
      real_type   s_begin,
      real_type   s_end,
      real_type   qx,
      real_type   qy,
      real_type   offs,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & dst
    ) const;

    static int_type  max_iter;
    static real_type tolerance;

    mutable bool               aabb_done;
    mutable AABBtree           aabb_tree;
    mutable real_type          aabb_offs;
    mutable real_type          aabb_max_angle;
    mutable real_type          aabb_max_size;
    mutable vector<Triangle2D> aabb_tri;

    bool
    aabb_intersect_ISO(
      Triangle2D    const & T1,
      real_type             offs,
      ClothoidCurve const * pC,
      Triangle2D    const & T2,
      real_type             C_offs,
      real_type           & ss1,
      real_type           & ss2
    ) const;

    class T2D_approximate_collision {
      ClothoidCurve const * pC1;
      ClothoidCurve const * pC2;
    public:
      T2D_approximate_collision(
        ClothoidCurve const * _pC1,
        ClothoidCurve const * _pC2
      )
      : pC1(_pC1)
      , pC2(_pC2)
      {}

      bool
      operator () ( BBox::PtrBBox ptr1, BBox::PtrBBox ptr2 ) const {
        Triangle2D const & T1 = pC1->aabb_tri[size_t(ptr1->Ipos())];
        Triangle2D const & T2 = pC2->aabb_tri[size_t(ptr2->Ipos())];
        return T1.overlap(T2);
      }
    };

    class T2D_collision_ISO {
      ClothoidCurve const * pC1;
      real_type     const   offs1;
      ClothoidCurve const * pC2;
      real_type     const   offs2;
    public:
      T2D_collision_ISO(
        ClothoidCurve const * _pC1,
        real_type     const   _offs1,
        ClothoidCurve const * _pC2,
        real_type     const   _offs2
      )
      : pC1(_pC1)
      , offs1(_offs1)
      , pC2(_pC2)
      , offs2(_offs2)
      {}

      bool
      operator () ( BBox::PtrBBox ptr1, BBox::PtrBBox ptr2 ) const {
        Triangle2D const & T1 = pC1->aabb_tri[size_t(ptr1->Ipos())];
        Triangle2D const & T2 = pC2->aabb_tri[size_t(ptr2->Ipos())];
        real_type ss1, ss2;
        return pC1->aabb_intersect_ISO( T1, offs1, pC2, T2, offs2, ss1, ss2 );
      }
    };

  public:

    #include "BaseCurve_using.hxx"

    //explicit
    ClothoidCurve()
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    {
      CD.x0     = 0;
      CD.y0     = 0;
      CD.theta0 = 0;
      CD.kappa0 = 0;
      CD.dk     = 0;
      L         = 0;
    }

    //explicit
    ClothoidCurve( ClothoidCurve const & s )
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    { copy(s); }

    //! construct a clothoid with the standard parameters
    explicit
    ClothoidCurve(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _k,
      real_type _dk,
      real_type _L
    )
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    {
      CD.x0     = _x0;
      CD.y0     = _y0;
      CD.theta0 = _theta0;
      CD.kappa0 = _k;
      CD.dk     = _dk;
      L         = _L;
    }

    //! construct a clothoid by solving the hermite G1 problem
    explicit
    ClothoidCurve(
      real_type const P0[],
      real_type       theta0,
      real_type const P1[],
      real_type       theta1
    )
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    {
      build_G1( P0[0], P0[1], theta0, P1[0], P1[1], theta1 );
    }

    void
    copy( ClothoidCurve const & c ) {
      CD = c.CD;
      L  = c.L;
      aabb_done = false;
      aabb_tree.clear();
    }

    explicit
    ClothoidCurve( LineSegment const & LS )
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    {
      CD.x0     = LS.x0;
      CD.y0     = LS.y0;
      CD.theta0 = LS.theta0;
      CD.kappa0 = 0;
      CD.dk     = 0;
      L         = LS.L;
    }

    explicit
    ClothoidCurve( CircleArc const & C )
    : BaseCurve(G2LIB_CLOTHOID)
    , aabb_done(false)
    {
      CD.x0     = C.x0;
      CD.y0     = C.y0;
      CD.theta0 = C.theta0;
      CD.kappa0 = C.k;
      CD.dk     = 0;
      L         = C.L;
    }

    explicit
    ClothoidCurve( BaseCurve const & C );

    ClothoidCurve const & operator = ( ClothoidCurve const & s )
    { copy(s); return *this; }

    /*\
     |  _         _ _    _
     | | |__ _  _(_) |__| |
     | | '_ \ || | | / _` |
     | |_.__/\_,_|_|_\__,_|
    \*/
    //! construct a clothoid with the standard parameters
    void
    build(
      real_type _x0,
      real_type _y0,
      real_type _theta0,
      real_type _k,
      real_type _dk,
      real_type _L
    ) {
      CD.x0     = _x0;
      CD.y0     = _y0;
      CD.theta0 = _theta0;
      CD.kappa0 = _k;
      CD.dk     = _dk;
      L         = _L;
      aabb_done = false;
      aabb_tree.clear();
    }

    /*!
     *  \brief build a clothoid by solving the hermite G1 problem
     *
     *  \param x0     initial x position \f$ x_0      \f$
     *  \param y0     initial y position \f$ y_0      \f$
     *  \param theta0 initial angle      \f$ \theta_0 \f$
     *  \param x1     final x position   \f$ x_1      \f$
     *  \param y1     final y position   \f$ y_1      \f$
     *  \param theta1 final angle        \f$ \theta_1 \f$
     *  \return number of iteration performed
     */
    int
    build_G1(
      real_type x0,
      real_type y0,
      real_type theta0,
      real_type x1,
      real_type y1,
      real_type theta1,
      real_type tol = 1e-12
    ) {
      aabb_done = false;
      aabb_tree.clear();
      return CD.build_G1( x0, y0, theta0, x1, y1, theta1, tol, L );
    }

    /*!
     *  \brief build a clothoid by solving the hermite G1 problem
     *
     *  \param x0     initial x position \f$ x_0      \f$
     *  \param y0     initial y position \f$ y_0      \f$
     *  \param theta0 initial angle      \f$ \theta_0 \f$
     *  \param x1     final x position   \f$ x_1      \f$
     *  \param y1     final y position   \f$ y_1      \f$
     *  \param theta1 final angle        \f$ \theta_1 \f$
     *  \return number of iteration performed
     */
    int
    build_G1_D(
      real_type x0,
      real_type y0,
      real_type theta0,
      real_type x1,
      real_type y1,
      real_type theta1,
      real_type L_D[2],
      real_type k_D[2],
      real_type dk_D[2],
      real_type tol = 1e-12
    ) {
      aabb_done = false;
      aabb_tree.clear();
      return CD.build_G1( x0, y0, theta0, x1, y1, theta1, tol, L,
                          true, L_D, k_D, dk_D );
    }

    /*!
     *  \brief build a clothoid by solving the forward problem
     *
     *  \param x0     initial x position \f$ x_0      \f$
     *  \param y0     initial y position \f$ y_0      \f$
     *  \param theta0 initial angle      \f$ \theta_0 \f$
     *  \param kappa0 initial curvature  \f$ \kappa_0 \f$
     *  \param x1     final x position   \f$ x_1      \f$
     *  \param y1     final y position   \f$ y_1      \f$
     */
    bool
    build_forward(
      real_type x0,
      real_type y0,
      real_type theta0,
      real_type kappa0,
      real_type x1,
      real_type y1,
      real_type tol = 1e-12
    ) {
      aabb_done = false;
      aabb_tree.clear();
      return CD.build_forward( x0, y0, theta0, kappa0, x1, y1, tol, L );
    }

    /*!
     *  \brief build a clothoid from a line segment
     *
     *  \param LS line segment object
     */
    void
    build( LineSegment const & LS ) {
      CD.x0     = LS.x0;
      CD.y0     = LS.y0;
      CD.theta0 = LS.theta0;
      CD.kappa0 = 0;
      CD.dk     = 0;
      L         = LS.L;
      aabb_done = false;
      aabb_tree.clear();
    }

    /*!
     *  \brief build a clothoid from a line segment
     *
     *  \param C line segment object
     */
    void
    build( CircleArc const & C ) {
      CD.x0     = C.x0;
      CD.y0     = C.y0;
      CD.theta0 = C.theta0;
      CD.kappa0 = C.k;
      CD.dk     = 0;
      L         = C.L;
      aabb_done = false;
      aabb_tree.clear();
    }

    void
    Pinfinity( real_type & x, real_type & y, bool plus = true ) const
    { CD.Pinfinity( x, y, plus ); }

    real_type
    dkappa() const
    { return CD.dk; }

    /*!
     *  \return clothoid total variation
     */
    real_type
    thetaTotalVariation() const;

    real_type
    thetaMinMax( real_type & thMin, real_type & thMax ) const;

    /*!
     *  \return clothoid angle range
     */
    real_type
    deltaTheta() const
    { real_type thMin, thMax; return thetaMinMax( thMin, thMax ); }

    real_type
    curvatureMinMax( real_type & kMin, real_type & kMax ) const;

    /*!
     *  \return clothoid total curvature variation
     */
    real_type curvatureTotalVariation() const;

    real_type integralCurvature2() const;

    real_type integralJerk2() const;

    real_type integralSnap2() const;

    /*!
     *  Return a vector of optimized sample parameters
     *  \param offs      offset of the sampled curve
     *  \param npts      suggested minimum number of sampled points
     *  \param max_angle maximum angle variation between two sampled points
     *  \param s         vector of computed parameters
     */
    void
    optimized_sample_ISO(
      real_type                offs,
      int_type                 npts,
      real_type                max_angle,
      std::vector<real_type> & s
    ) const;

    void
    optimized_sample_SAE(
      real_type                offs,
      int_type                 npts,
      real_type                max_angle,
      std::vector<real_type> & s
    ) const {
      optimized_sample_ISO( -offs, npts, max_angle, s );
    }

    /*\
     |     _ _    _
     |  __| (_)__| |_ __ _ _ _  __ ___
     | / _` | (_-<  _/ _` | ' \/ _/ -_)
     | \__,_|_/__/\__\__,_|_||_\__\___|
    \*/
    /*!
     * \brief Compute the point on clothoid at minimal distance from a given point
     *
     * \param  qx x-coordinate of the given point
     * \param  qy y-coordinate of the given point
     * \param  X  x-coordinate of the point on clothoid at minimal distance
     * \param  Y  y-coordinate of the point on clothoid at minimal distance
     * \param  S  curvilinear coordinate of the point (X,Y) on the clothoid
     * \return the distance of the
     *
     */
    real_type
    closestPointBySample(
      real_type   ds,
      real_type   qx,
      real_type   qy,
      real_type & X,
      real_type & Y,
      real_type & S
    ) const;

    real_type
    distanceBySample(
      real_type   ds,
      real_type   qx,
      real_type   qy,
      real_type & S
    ) const {
      real_type X, Y;
      return closestPointBySample( ds, qx, qy, X, Y, S );
    }

    real_type
    distanceBySample(
      real_type ds,
      real_type qx,
      real_type qy
    ) const {
      real_type X, Y, S;
      return closestPointBySample( ds, qx, qy, X, Y, S );
    }

    /*\
     |  _    _   _____    _                _
     | | |__| |_|_   _| _(_)__ _ _ _  __ _| |___
     | | '_ \ '_ \| || '_| / _` | ' \/ _` | / -_)
     | |_.__/_.__/|_||_| |_\__,_|_||_\__, |_\___|
     |                               |___/
    \*/

    //! get the triangle bounding box (if angle variation less that pi/2)
    bool
    bbTriangle(
      real_type & xx0, real_type & yy0,
      real_type & xx1, real_type & yy1,
      real_type & xx2, real_type & yy2
    ) const {
      return CD.bbTriangle( L, xx0, yy0, xx1, yy1, xx2, yy2 );
    }

    //! get the triangle bounding box (if angle variation less that pi/2)
    bool
    bbTriangle_ISO(
      real_type offs,
      real_type & xx0, real_type & yy0,
      real_type & xx1, real_type & yy1,
      real_type & xx2, real_type & yy2
    ) const {
      return CD.bbTriangle_ISO( L, offs, xx0, yy0, xx1, yy1, xx2, yy2 );
    }

    //! get the triangle bounding box (if angle variation less that pi/2)
    bool
    bbTriangle_SAE(
      real_type offs,
      real_type & xx0, real_type & yy0,
      real_type & xx1, real_type & yy1,
      real_type & xx2, real_type & yy2
    ) const {
      return CD.bbTriangle_SAE( L, offs, xx0, yy0, xx1, yy1, xx2, yy2 );
    }

    bool
    bbTriangle( Triangle2D & t, int_type icurve = 0 ) const {
      real_type x0, y0, x1, y1, x2, y2;
      bool ok = CD.bbTriangle( L, x0, y0, x1, y1, x2, y2 );
      if ( ok ) t.build( x0, y0, x1, y1, x2, y2, 0, 0, icurve );
      return ok;
    }

    bool
    bbTriangle_ISO( real_type offs, Triangle2D & t, int_type icurve = 0 ) const {
      real_type x0, y0, x1, y1, x2, y2;
      bool ok = CD.bbTriangle_ISO( L, offs, x0, y0, x1, y1, x2, y2 );
      if ( ok ) t.build( x0, y0, x1, y1, x2, y2, 0, 0, icurve );
      return ok;
    }

    bool
    bbTriangle_SAE( real_type offs, Triangle2D & t, int_type icurve = 0 ) const {
      real_type x0, y0, x1, y1, x2, y2;
      bool ok = CD.bbTriangle_SAE( L, offs, x0, y0, x1, y1, x2, y2 );
      if ( ok ) t.build( x0, y0, x1, y1, x2, y2, 0, 0, icurve );
      return ok;
    }

    void
    bbTriangles_ISO(
      real_type                 offs,
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100,
      int_type                  icurve    = 0
    ) const;

    void
    bbTriangles_SAE(
      real_type                 offs,
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100,
      int_type                  icurve    = 0
    ) const {
      this->bbTriangles_ISO( -offs, tvec, max_angle, max_size, icurve );
    }

    void
    bbTriangles(
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100,
      int_type                  icurve    = 0
    ) const {
      this->bbTriangles_ISO( 0, tvec, max_angle, max_size, icurve );
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    bbox(
      real_type & xmin,
      real_type & ymin,
      real_type & xmax,
      real_type & ymax
    ) const G2LIB_OVERRIDE {
      bbox_ISO( 0, xmin, ymin, xmax, ymax );
    }

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
    length_ISO( real_type ) const G2LIB_OVERRIDE {
      G2LIB_DO_ERROR( "Offset length not available for Clothoids" )
      return 0;
    }

    virtual
    real_type
    thetaBegin() const G2LIB_OVERRIDE
    { return CD.theta0; }

    virtual
    real_type
    thetaEnd() const G2LIB_OVERRIDE
    { return CD.theta(L); }

    virtual
    real_type
    xBegin() const G2LIB_OVERRIDE
    { return CD.x0; }

    virtual
    real_type
    xEnd() const G2LIB_OVERRIDE
    { return CD.X(L); }

    virtual
    real_type
    yBegin() const G2LIB_OVERRIDE
    { return CD.y0; }

    virtual
    real_type
    yEnd() const G2LIB_OVERRIDE
    { return CD.Y(L); }

    virtual
    real_type
    tx_Begin() const G2LIB_OVERRIDE
    { return CD.tg0_x(); }

    virtual
    real_type
    ty_Begin() const G2LIB_OVERRIDE
    { return CD.tg0_y(); }

    virtual
    real_type
    nx_Begin_ISO() const G2LIB_OVERRIDE
    { return CD.nor0_x_ISO(); }

    virtual
    real_type
    ny_Begin_ISO() const G2LIB_OVERRIDE
    { return CD.nor0_y_ISO(); }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
    { return CD.tg_x( s ); }

    virtual
    real_type
    ty( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_y( s ); }

    virtual
    real_type
    tx_D( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_x_D( s ); }

    virtual
    real_type
    ty_D( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_y_D( s ); }

    virtual
    real_type
    tx_DD( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_x_DD( s ); }

    virtual
    real_type
    ty_DD( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_y_DD( s ); }

    virtual
    real_type
    tx_DDD( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_x_DDD( s ); }

    virtual
    real_type
    ty_DDD( real_type s ) const G2LIB_OVERRIDE
    { return CD.tg_y_DDD( s ); }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    tg(
      real_type   s,
      real_type & tx,
      real_type & ty
    ) const G2LIB_OVERRIDE
    { CD.tg( s, tx, ty ); }

    virtual
    void
    tg_D(
      real_type   s,
      real_type & tx_D,
      real_type & ty_D
    ) const G2LIB_OVERRIDE
    { CD.tg_D( s, tx_D, ty_D ); }

    virtual
    void
    tg_DD(
      real_type   s,
      real_type & tx_DD,
      real_type & ty_DD
    ) const G2LIB_OVERRIDE
    { CD.tg_DD( s, tx_DD, ty_DD ); }

    virtual
    void
    tg_DDD(
      real_type   s,
      real_type & tx_DDD,
      real_type & ty_DDD
    ) const G2LIB_OVERRIDE
    { CD.tg_DDD( s, tx_DDD, ty_DDD ); }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief get clothoid angle at curvilinear cooordinate `s`
     *
     * \param  s curvilinear cooordinate
     * \return angle (radiant) at curvilinear cooordinate `s`
     */
    real_type
    theta( real_type s ) const G2LIB_OVERRIDE
    { return CD.theta(s); }

    /*!
     * \brief get clothoid angle derivative (=curvature) at curvilinear cooordinate `s`
     *
     * \param  s curvilinear cooordinate
     * \return angle derivative (radiant/s) at curvilinear cooordinate `s`
     */
    real_type
    theta_D( real_type s ) const G2LIB_OVERRIDE
    { return CD.kappa(s); }

    /*!
     * \brief get clothoid angle second derivative at curvilinear cooordinate `s`
     *
     * \return angle second derivative (radiant/s^2) at curvilinear cooordinate `s`
     */
    real_type
    theta_DD( real_type ) const G2LIB_OVERRIDE
    { return CD.dk; }

    /*!
     * \brief get clothoid angle third derivative at curvilinear cooordinate `s`
     *
     * \return angle third derivative (radiant/s^3) at curvilinear cooordinate `s`
     */
    real_type
    theta_DDD( real_type ) const G2LIB_OVERRIDE
    { return 0; }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    #ifdef G2LIB_COMPATIBILITY_MODE
    virtual
    void
    evaluate(
      real_type   s,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE
    { CD.evaluate( s, th, k, x, y ); }
    #endif

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief clothoid X coordinate at curvilinear coordinate `s`
     * \param s curvilinear coordinate
     * \return clothoid X coordinate
     */
    real_type
    X( real_type s ) const G2LIB_OVERRIDE
    { return CD.X(s); }

    real_type
    X_D( real_type s ) const G2LIB_OVERRIDE
    { return CD.X_D(s); }

    real_type
    X_DD( real_type s ) const G2LIB_OVERRIDE
    { return CD.X_DD(s); }

    real_type
    X_DDD( real_type s ) const G2LIB_OVERRIDE
    { return CD.X_DDD(s); }

    /*!
     * \brief clothoid Y coordinate at curvilinear coordinate `s`
     * \param s curvilinear coordinate
     * \return clothoid Y coordinate
     */
    real_type
    Y( real_type s ) const G2LIB_OVERRIDE
    { return CD.Y(s); }

    real_type
    Y_D( real_type s ) const G2LIB_OVERRIDE
    { return CD.Y_D(s); }

    real_type
    Y_DD ( real_type s ) const G2LIB_OVERRIDE
    { return CD.Y_DD(s); }

    real_type
    Y_DDD( real_type s ) const G2LIB_OVERRIDE
    { return CD.Y_DDD(s); }

    /*!
     * \brief clothoid X coordinate at curvilinear coordinate `s`
     * \param s    curvilinear coordinate
     * \param offs lateral offset
     * \return     clothoid X coordinate
     */
    real_type
    X_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.X_ISO(s,offs); }

    real_type
    X_ISO_D( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.X_ISO_D(s,offs); }

    real_type
    X_ISO_DD( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.X_ISO_DD(s,offs); }

    real_type
    X_ISO_DDD( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.X_ISO_DDD(s,offs); }

    /*!
     * \brief clothoid Y coordinate at curvilinear coordinate `s`
     * \param s curvilinear coordinate
     * \param offs lateral offset
     * \return clothoid Y coordinate
     */
    real_type
    Y_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.Y_ISO(s,offs); }

    real_type
    Y_ISO_D( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.Y_ISO_D(s,offs); }

    real_type
    Y_ISO_DD( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.Y_ISO_DD(s,offs); }

    real_type
    Y_ISO_DDD( real_type s, real_type offs ) const G2LIB_OVERRIDE
    { return CD.Y_ISO_DDD(s,offs); }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval(
      real_type   s,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE
    { CD.eval( s, x, y ); }

    virtual
    void
    eval_D(
      real_type   s,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE
    { CD.eval_D( s, x_D, y_D ); }

    virtual
    void
    eval_DD(
      real_type   s,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE
    { CD.eval_DD( s, x_DD, y_DD ); }

    virtual
    void
    eval_DDD(
      real_type   s,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE
    { CD.eval_DDD( s, x_DDD, y_DDD ); }

    virtual
    void
    eval_ISO(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE
    { CD.eval_ISO( s, offs, x, y ); }

    virtual
    void
    eval_ISO_D(
      real_type   s,
      real_type   offs,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE
    { CD.eval_ISO_D( s, offs, x_D, y_D ); }

    virtual
    void
    eval_ISO_DD(
      real_type   s,
      real_type   offs,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE
    { CD.eval_ISO_DD( s, offs, x_DD, y_DD ); }

    virtual
    void
    eval_ISO_DDD(
      real_type   s,
      real_type   offs,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE
    { CD.eval_ISO_DDD( s, offs, x_DDD, y_DDD ); }

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
    { CD.x0 += tx; CD.y0 += ty; }

    virtual
    void
    rotate( real_type angle, real_type cx, real_type cy ) G2LIB_OVERRIDE
    { CD.rotate( angle, cx, cy ); }

    virtual
    void
    scale( real_type s ) G2LIB_OVERRIDE {
      CD.kappa0 /= s;
      CD.dk     /= s*s;
      L         *= s;
    }

    virtual
    void
    reverse() G2LIB_OVERRIDE
    { CD.reverse(L); }

    virtual
    void
    changeOrigin( real_type newx0, real_type newy0 ) G2LIB_OVERRIDE
    { CD.x0 = newx0; CD.y0 = newy0; }

    virtual
    void
    trim( real_type s_begin, real_type s_end ) G2LIB_OVERRIDE {
      CD.origin_at( s_begin );
      L = s_end - s_begin;
    }

    void
    changeCurvilinearOrigin( real_type s0, real_type newL ) {
      CD.origin_at( s0 );
      L = newL;
    }

    /*\
     |        _                     _   ____       _       _
     |    ___| | ___  ___  ___  ___| |_|  _ \ ___ (_)_ __ | |_
     |   / __| |/ _ \/ __|/ _ \/ __| __| |_) / _ \| | '_ \| __|
     |  | (__| | (_) \__ \  __/\__ \ |_|  __/ (_) | | | | | |_
     |   \___|_|\___/|___/\___||___/\__|_|   \___/|_|_| |_|\__|
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

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*\
     |             _ _ _     _
     |    ___ ___ | | (_)___(_) ___  _ __
     |   / __/ _ \| | | / __| |/ _ \| '_ \
     |  | (_| (_) | | | \__ \ | (_) | | | |
     |   \___\___/|_|_|_|___/_|\___/|_| |_|
    \*/

    void
    build_AABBtree_ISO(
      real_type offs,
      real_type max_angle = m_pi/18, // 10 degree
      real_type max_size  = 1e100
    ) const;

    // collision detection
    bool
    approximate_collision_ISO(
      real_type             offs,
      ClothoidCurve const & c,
      real_type             c_offs,
      real_type             max_angle, //!< maximum angle variation
      real_type             max_size   //!< curve offset
    ) const;

    bool
    collision( ClothoidCurve const & C ) const;

    bool
    collision_ISO(
      real_type             offs,
      ClothoidCurve const & C,
      real_type             offs_C
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
      ClothoidCurve const & C,
      IntersectList       & ilist,
      bool                  swap_s_vals
    ) const {
      intersect_ISO( 0, C, 0, ilist, swap_s_vals );
    }

    void
    intersect_ISO(
      real_type               offs,
      ClothoidCurve const   & C,
      real_type               offs_C,
      IntersectList         & ilist,
      bool                    swap_s_vals
    ) const;

    void
    info( ostream_type & stream ) const G2LIB_OVERRIDE
    { stream << "Clothoid\n" << *this << '\n'; }

    friend
    ostream_type &
    operator << ( ostream_type & stream, ClothoidCurve const & c );

  };

}

#endif

///
/// eof: Clothoid.hh
///
