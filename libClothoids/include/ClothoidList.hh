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
/// file: ClothoidList.hh
///

#ifndef CLOTHOID_LIST_HH
#define CLOTHOID_LIST_HH

#include "G2lib.hh"
#include "Clothoid.hh"
#include "PolyLine.hh"
#include "Biarc.hh"
#include "BiarcList.hh"

//! Clothoid computations routine
namespace G2lib {

  using std::vector;

  /*\
   |    ____ ____            _           ____
   |   / ___|___ \ ___  ___ | |_   _____|___ \ __ _ _ __ ___
   |  | |  _  __) / __|/ _ \| \ \ / / _ \ __) / _` | '__/ __|
   |  | |_| |/ __/\__ \ (_) | |\ V /  __// __/ (_| | | | (__
   |   \____|_____|___/\___/|_| \_/ \___|_____\__,_|_|  \___|
  \*/

  //! computation of the G2 fitting with 2 clothoid arc
  class G2solve2arc {

    real_type tolerance;
    int_type  maxIter;

    real_type x0;
    real_type y0;
    real_type theta0;
    real_type kappa0;

    real_type x1;
    real_type y1;
    real_type theta1;
    real_type kappa1;

    // standard problem
    real_type lambda, phi, xbar, ybar;
    real_type th0, th1;
    real_type k0, k1;
    real_type DeltaK;
    real_type DeltaTheta;

    ClothoidCurve S0, S1;

    void
    evalA(
      real_type   alpha,
      real_type   L,
      real_type & A
    ) const;

    void
    evalA(
      real_type   alpha,
      real_type   L,
      real_type & A,
      real_type & A_1,
      real_type & A_2
    ) const;

    void
    evalG(
      real_type alpha,
      real_type L,
      real_type th,
      real_type k,
      real_type G[2]
    ) const;

    void
    evalG(
      real_type alpha,
      real_type L,
      real_type th,
      real_type k,
      real_type G[2],
      real_type G_1[2],
      real_type G_2[2]
    ) const;

    void
    evalF( real_type const vars[2], real_type F[2] ) const;

    void
    evalFJ(
      real_type const vars[2],
      real_type       F[2],
      real_type       J[2][2]
    ) const;

    void
    buildSolution( real_type alpha, real_type L );

  public:

    G2solve2arc()
    : tolerance(1e-10)
    , maxIter(20)
    , x0(0)
    , y0(0)
    , theta0(0)
    , kappa0(0)
    , x1(0)
    , y1(0)
    , theta1(0)
    , kappa1(0)
    , lambda(0)
    , phi(0)
    , xbar(0)
    , ybar(0)
    , th0(0)
    , th1(0)
    , k0(0)
    , k1(0)
    {}

    ~G2solve2arc() {}

    int
    build(
      real_type x0, real_type y0, real_type theta0, real_type kappa0,
      real_type x1, real_type y1, real_type theta1, real_type kappa1
    );

    void
    setTolerance( real_type tol );

    void
    setMaxIter( int tol );

    int
    solve();

    ClothoidCurve const & getS0() const { return S0; }
    ClothoidCurve const & getS1() const { return S1; }

  };

  /*\
   |    ____ ____            _            ____ _     ____
   |   / ___|___ \ ___  ___ | |_   _____ / ___| |   / ___|
   |  | |  _  __) / __|/ _ \| \ \ / / _ \ |   | |  | |
   |  | |_| |/ __/\__ \ (_) | |\ V /  __/ |___| |__| |___
   |   \____|_____|___/\___/|_| \_/ \___|\____|_____\____|
  \*/

  //! computation of the G2 fitting with 2 clothoid and one line segment
  class G2solveCLC {

    real_type tolerance;
    int       maxIter;

    real_type x0;
    real_type y0;
    real_type theta0;
    real_type kappa0;
    real_type x1;
    real_type y1;
    real_type theta1;
    real_type kappa1;

    // standard problem
    real_type lambda, phi, xbar, ybar;
    real_type th0, th1;
    real_type k0, k1;

    ClothoidCurve S0, SM, S1;

    bool
    buildSolution( real_type sM, real_type thM );

  public:

    G2solveCLC()
    : tolerance(1e-10)
    , maxIter(20)
    , x0(0)
    , y0(0)
    , theta0(0)
    , kappa0(0)
    , x1(0)
    , y1(0)
    , theta1(0)
    , kappa1(0)
    , lambda(0)
    , phi(0)
    , xbar(0)
    , ybar(0)
    , th0(0)
    , th1(0)
    , k0(0)
    , k1(0)
    {}

    ~G2solveCLC() {}

    int
    build(
      real_type x0, real_type y0, real_type theta0, real_type kappa0,
      real_type x1, real_type y1, real_type theta1, real_type kappa1
    );

    void
    setTolerance( real_type tol );

    void
    setMaxIter( int tol );

    int
    solve();

    ClothoidCurve const & getS0() const { return S0; }
    ClothoidCurve const & getSM() const { return SM; }
    ClothoidCurve const & getS1() const { return S1; }

  };

  /*\
   |    ____ ____            _           _____
   |   / ___|___ \ ___  ___ | |_   _____|___ /  __ _ _ __ ___
   |  | |  _  __) / __|/ _ \| \ \ / / _ \ |_ \ / _` | '__/ __|
   |  | |_| |/ __/\__ \ (_) | |\ V /  __/___) | (_| | | | (__
   |   \____|_____|___/\___/|_| \_/ \___|____/ \__,_|_|  \___|
  \*/
  // Clothoid-clothoid-clothoid with G2 continuity
  //! computation of the G2 fitting with 3 clothoid arcs
  class G2solve3arc {

    ClothoidCurve S0, SM, S1;

    real_type tolerance;
    int       maxIter;

    // G2 interpolation data
    real_type x0;
    real_type y0;
    real_type theta0;
    real_type kappa0;
    real_type x1;
    real_type y1;
    real_type theta1;
    real_type kappa1;

    // standard scaled problem
    real_type phi, Lscale;
    real_type th0, th1;
    real_type s0, s1;

    // precomputed values
    real_type K0, K1, c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14;

    void
    evalFJ(
      real_type const vars[2],
      real_type       F[2],
      real_type       J[2][2]
    ) const;

    void
    evalF( real_type const vars[2], real_type F[2] ) const;

    void
    buildSolution( real_type sM, real_type thM );

    int
    solve( real_type sM_guess, real_type thM_guess );

  public:

    G2solve3arc()
    : tolerance(1e-10)
    , maxIter(100)
    {}

    ~G2solve3arc() {}

    void setTolerance( real_type tol );
    void setMaxIter( int miter );

    /*!
     *  Compute the 3 arc clothoid spline that fit the data
     *
     *  \param[in] x0      initial `x` position
     *  \param[in] y0      initial `y` position
     *  \param[in] theta0  initial angle
     *  \param[in] kappa0  initial curvature
     *  \param[in] x1      final `x` position
     *  \param[in] y1      final `y` position
     *  \param[in] theta1  final angle
     *  \param[in] kappa1  final curvature
     *  \param[in] Dmax    rough desidered maximum angle variation, if 0 computed automatically
     *  \param[in] dmax    rough desidered maximum angle divergence from guess, if 0 computed automatically
     *  \return number of iteration, -1 if fails
     *
     */
    int
    build(
      real_type x0,
      real_type y0,
      real_type theta0,
      real_type kappa0,
      real_type x1,
      real_type y1,
      real_type theta1,
      real_type kappa1,
      real_type Dmax = 0,
      real_type dmax = 0
    );

    /*!
     *  Compute the 3 arc clothoid spline that fit the data
     *
     *  \param[in] s0      length of the first segment
     *  \param[in] x0      initial `x` position
     *  \param[in] y0      initial `y` position
     *  \param[in] theta0  initial angle
     *  \param[in] kappa0  initial curvature
     *  \param[in] s1      length of the last segment
     *  \param[in] x1      final `x` position
     *  \param[in] y1      final `y` position
     *  \param[in] theta1  final angle
     *  \param[in] kappa1  final curvature
     *  \return number of iteration, -1 if fails
     *
     */
    int
    build_fixed_length(
      real_type s0,
      real_type x0,
      real_type y0,
      real_type theta0,
      real_type kappa0,
      real_type s1,
      real_type x1,
      real_type y1,
      real_type theta1,
      real_type kappa1
    );

    //! \return get the first clothoid for the 3 arc G2 fitting
    ClothoidCurve const & getS0() const { return S0; }

    //! \return get the last clothoid for the 3 arc G2 fitting
    ClothoidCurve const & getS1() const { return S1; }

    //! \return get the middle clothoid for the 3 arc G2 fitting
    ClothoidCurve const & getSM() const { return SM; }

    //! \return get the length of the 3 arc G2 fitting
    real_type
    totalLength() const {
      return S0.length() + S1.length() + SM.length();
    }

    //! \return get the total angle variation of the 3 arc G2 fitting
    real_type
    thetaTotalVariation() const {
      return S0.thetaTotalVariation() +
             S1.thetaTotalVariation() +
             SM.thetaTotalVariation();
    }

    //! \return get the total curvature variation of the 3 arc G2 fitting
    real_type
    curvatureTotalVariation() const {
      return S0.curvatureTotalVariation() +
             S1.curvatureTotalVariation() +
             SM.curvatureTotalVariation();
    }

    //! \return get the integral of the curvature squared of the 3 arc G2 fitting
    real_type
    integralCurvature2() const {
      return S0.integralCurvature2() +
             S1.integralCurvature2() +
             SM.integralCurvature2();
    }

    //! \return get the integral of the jerk squared of the 3 arc G2 fitting
    real_type
    integralJerk2() const {
      return S0.integralJerk2() +
             S1.integralJerk2() +
             SM.integralJerk2();
    }

    //! \return get the integral of the snap squared of the 3 arc G2 fitting
    real_type
    integralSnap2() const {
      return S0.integralSnap2() +
             S1.integralSnap2() +
             SM.integralSnap2();
    }

    /*!
     *  \param[out] thMin minimum angle in the 3 arc G2 fitting curve
     *  \param[out] thMax maximum angle in the 3 arc G2 fitting curve
     *  \return the difference of `thMax` and `thMin`
     */
    real_type
    thetaMinMax( real_type & thMin, real_type & thMax ) const;

    /*!
     *  \return the difference of maximum-minimum angle in the 3 arc G2 fitting curve
     */
    real_type
    deltaTheta() const
    { real_type thMin, thMax; return thetaMinMax( thMin, thMax ); }

    /*!
     *  \param[out] kMin minimum curvature in the 3 arc G2 fitting curve
     *  \param[out] kMax maximum curvature in the 3 arc G2 fitting curve
     *  \return the difference of `kMax` and `kMin`
     */
    real_type
    curvatureMinMax( real_type & kMin, real_type & kMax ) const;

    /*!
     *  \return angle as a function of curvilinear coordinate
     */
    real_type theta( real_type s ) const;

    /*!
     *  \return angle derivative (curvature) as a function of curvilinear coordinate
     */
    real_type theta_D( real_type s ) const;

    /*!
     *  \return angle second derivative (curvature derivative) as a function of curvilinear coordinate
     */
    real_type theta_DD( real_type s ) const;

    /*!
     *  \return angle third derivative as a function of curvilinear coordinate
    \*/
    real_type theta_DDD( real_type s ) const;

    /*!
     *  \return x coordinate of the3 arc clothoid as a function of curvilinear coordinate
     */
    real_type X( real_type s ) const;

    /*!
     *  \return y coordinate of the3 arc clothoid as a function of curvilinear coordinate
     */
    real_type Y( real_type s ) const;

    /*!
     *  \return initial x coordinate of the 3 arc clothoid
     */
    real_type xBegin() const { return S0.xBegin(); }

    /*!
     *  \return initial y coordinate of the 3 arc clothoid
     */
    real_type yBegin() const { return S0.yBegin(); }

    /*!
     *  \return initial curvature of the 3 arc clothoid
     */
    real_type kappaBegin() const { return S0.kappaBegin(); }

    /*!
     *  \return initial angle of the 3 arc clothoid
     */
    real_type thetaBegin() const { return S0.thetaBegin(); }

    /*!
     *  \return final x coordinate of the 3 arc clothoid
     */
    real_type xEnd()const { return S1.xEnd(); }

    /*!
     *  \return final y coordinate of the 3 arc clothoid
     */
    real_type yEnd() const { return S1.yEnd(); }

    /*!
     *  \return final curvature of the 3 arc clothoid
     */
    real_type kappaEnd() const { return S1.kappaEnd(); }

    /*!
     *  \return final angle of the 3 arc clothoid
     */
    real_type thetaEnd() const { return S1.thetaEnd(); }

    /*!
     *  Compute parameters of 3 arc clothoid at curvilinear coordinate `s`
     *
     *  \param[in]  s     curvilinear coordinate of where curve is computed
     *  \param[out] theta the curve angle
     *  \param[out] kappa the curve curvature
     *  \param[out] x     the curve x-coordinate
     *  \param[out] y     the curve y-coordinate
     */
    void
    eval(
      real_type   s,
      real_type & theta,
      real_type & kappa,
      real_type & x,
      real_type & y
    ) const;

    void eval( real_type s, real_type & x, real_type & y ) const;
    void eval_D( real_type s, real_type & x_D, real_type & y_D ) const;
    void eval_DD( real_type s, real_type & x_DD, real_type & y_DD ) const;
    void eval_DDD( real_type s, real_type & x_DDD, real_type & y_DDD ) const;

    // offset curve
    void eval_ISO( real_type s, real_type offs, real_type & x, real_type & y ) const;
    void eval_ISO_D( real_type s, real_type offs, real_type & x_D, real_type & y_D ) const;
    void eval_ISO_DD( real_type s, real_type offs, real_type & x_DD, real_type & y_DD ) const;
    void eval_ISO_DDD( real_type s, real_type offs, real_type & x_DDD, real_type & y_DDD ) const;

    void
    rotate( real_type angle, real_type cx, real_type cy ) {
      S0.rotate( angle, cx, cy );
      S1.rotate( angle, cx, cy );
      SM.rotate( angle, cx, cy );
    }

    void
    translate( real_type tx, real_type ty ){
      S0.translate( tx, ty );
      S1.translate( tx, ty );
      SM.translate( tx, ty );
    }

    void
    reverse() {
      ClothoidCurve tmp(S0); S1 = S0; S0 = tmp;
      S0.reverse();
      S1.reverse();
      SM.reverse();
    }

    friend
    ostream_type &
    operator << ( ostream_type & stream, ClothoidCurve const & c );

  };

  /*\
   |   ____ _       _   _           _     _ _     _     _
   |  / ___| | ___ | |_| |__   ___ (_) __| | |   (_)___| |_
   | | |   | |/ _ \| __| '_ \ / _ \| |/ _` | |   | / __| __|
   | | |___| | (_) | |_| | | | (_) | | (_| | |___| \__ \ |_
   |  \____|_|\___/ \__|_| |_|\___/|_|\__,_|_____|_|___/\__|
   |
  \*/
  //! \brief Class to manage a list of clothoid curves (not necessarily G2 or G1 connected)
  class ClothoidList : public BaseCurve {

    vector<real_type>     s0;
    vector<ClothoidCurve> clotoidList;
    mutable int_type      last_idx;

    mutable bool               aabb_done;
    mutable AABBtree           aabb_tree;
    mutable real_type          aabb_offs;
    mutable real_type          aabb_max_angle;
    mutable real_type          aabb_max_size;
    mutable vector<Triangle2D> aabb_tri;

    class T2D_collision_list_ISO {
      ClothoidList const * pList1;
      real_type    const   offs1;
      ClothoidList const * pList2;
      real_type    const   offs2;
    public:
      T2D_collision_list_ISO(
        ClothoidList const * _pList1,
        real_type    const   _offs1,
        ClothoidList const * _pList2,
        real_type    const   _offs2
      )
      : pList1(_pList1)
      , offs1(_offs1)
      , pList2(_pList2)
      , offs2(_offs2)
      {}

      bool
      operator () ( BBox::PtrBBox ptr1, BBox::PtrBBox ptr2 ) const {
        Triangle2D    const & T1 = pList1->aabb_tri[size_t(ptr1->Ipos())];
        Triangle2D    const & T2 = pList2->aabb_tri[size_t(ptr2->Ipos())];
        ClothoidCurve const & C1 = pList1->get(T1.Icurve());
        ClothoidCurve const & C2 = pList2->get(T2.Icurve());
        real_type ss1, ss2;
        return C1.aabb_intersect_ISO( T1, offs1, &C2, T2, offs2, ss1, ss2 );
      }
    };

  public:

    #include "BaseCurve_using.hxx"

    //explicit
    ClothoidList()
    : BaseCurve(G2LIB_CLOTHOID_LIST)
    , last_idx(0)
    , aabb_done(false)
    {}

    virtual
    ~ClothoidList() G2LIB_OVERRIDE {
      s0.clear();
      clotoidList.clear();
      aabb_tri.clear();
    }

    //explicit
    ClothoidList( ClothoidList const & s )
    : BaseCurve(G2LIB_CLOTHOID_LIST)
    , last_idx(0)
    , aabb_done(false)
    { copy(s); }

    void init();
    void reserve( int_type n );
    void copy( ClothoidList const & L );

    ClothoidList const & operator = ( ClothoidList const & s )
    { copy(s); return *this; }

    explicit
    ClothoidList( LineSegment const & LS );

    explicit
    ClothoidList( CircleArc const & C );

    explicit
    ClothoidList( Biarc const & B );

    explicit
    ClothoidList( BiarcList const & BL );

    explicit
    ClothoidList( ClothoidCurve const & CL );

    explicit
    ClothoidList( PolyLine const & PL );

    explicit
    ClothoidList( BaseCurve const & C );

    void push_back( LineSegment   const & c );
    void push_back( CircleArc     const & c );
    void push_back( Biarc         const & c );
    void push_back( BiarcList     const & c );
    void push_back( ClothoidCurve const & c );
    void push_back( PolyLine      const & c );


    void push_back( real_type kappa0, real_type dkappa, real_type L );
    void push_back( real_type x0,     real_type y0,     real_type theta0,
                    real_type kappa0, real_type dkappa, real_type L );

    void push_back_G1( real_type x1, real_type y1, real_type theta1 );
    void push_back_G1( real_type x0, real_type y0, real_type theta0,
                       real_type x1, real_type y1, real_type theta1 );

    bool
    build_G1(
      int_type        n,
      real_type const x[],
      real_type const y[]
    );

    bool
    build_G1(
      int_type        n,
      real_type const x[],
      real_type const y[],
      real_type const theta[]
    );

    bool
    build(
      real_type       x0,
      real_type       y0,
      real_type       theta0,
      int_type        n,
      real_type const s[],
      real_type const kappa[]
    );

    bool
    build(
      real_type                 x0,
      real_type                 y0,
      real_type                 theta0,
      vector<real_type> const & s,
      vector<real_type> const & kappa
    ) {
      if ( s.size() != kappa.size() ) return false;
      return build(
        x0, y0, theta0,
        int_type(s.size()),
        &s.front(), &kappa.front()
      );
    }

    ClothoidCurve const & get( int_type idx ) const;
    ClothoidCurve const & getAtS( real_type s ) const;

    int_type numSegment() const { return int_type(clotoidList.size()); }

    bool findAtS( real_type s ) const;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    length() const G2LIB_OVERRIDE;

    virtual
    real_type
    length_ISO( real_type offs ) const G2LIB_OVERRIDE;

    real_type
    segment_length( int_type nseg ) const;

    real_type
    segment_length_ISO( int_type nseg, real_type offs ) const;

    real_type
    segment_length_SAE( int_type nseg, real_type offs ) const
    { return segment_length_ISO( nseg, -offs ); }

    /*\
     |  _    _   _____    _                _
     | | |__| |_|_   _| _(_)__ _ _ _  __ _| |___
     | | '_ \ '_ \| || '_| / _` | ' \/ _` | / -_)
     | |_.__/_.__/|_||_| |_\__,_|_||_\__, |_\___|
     |                               |___/
    \*/

    void
    bbTriangles_ISO(
      real_type                 offs,
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100
    ) const;

    void
    bbTriangles_SAE(
      real_type                 offs,
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100
    ) const {
      this->bbTriangles_ISO( -offs, tvec, max_angle, max_size );
    }

    void
    bbTriangles(
      std::vector<Triangle2D> & tvec,
      real_type                 max_angle = m_pi/6, // 30 degree
      real_type                 max_size  = 1e100
    ) const {
      bbTriangles_ISO( 0, tvec, max_angle, max_size );
    }

    void
    build_AABBtree_ISO(
      real_type offs,
      real_type max_angle = m_pi/6, // 30 degree
      real_type max_size  = 1e100
    ) const;

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

    /*\
     |   ____             _          _______           _
     |  | __ )  ___  __ _(_)_ __    / / ____|_ __   __| |
     |  |  _ \ / _ \/ _` | | '_ \  / /|  _| | '_ \ / _` |
     |  | |_) |  __/ (_| | | | | |/ / | |___| | | | (_| |
     |  |____/ \___|\__, |_|_| |_/_/  |_____|_| |_|\__,_|
     |              |___/
    \*/

    virtual
    real_type
    thetaBegin() const G2LIB_OVERRIDE
    { return clotoidList.front().thetaBegin(); }

    virtual
    real_type
    thetaEnd() const G2LIB_OVERRIDE
    { return clotoidList.back().thetaEnd(); }

    virtual
    real_type
    xBegin() const G2LIB_OVERRIDE
    { return clotoidList.front().xBegin(); }

    virtual
    real_type
    yBegin() const G2LIB_OVERRIDE
    { return clotoidList.front().yBegin(); }

    virtual
    real_type
    xEnd() const G2LIB_OVERRIDE
    { return clotoidList.back().xEnd(); }

    virtual
    real_type
    yEnd() const G2LIB_OVERRIDE
    { return clotoidList.back().yEnd(); }

    virtual
    real_type
    xBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return clotoidList.front().xBegin_ISO( offs ); }

    virtual
    real_type
    yBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return clotoidList.front().yBegin_ISO( offs ); }

    virtual
    real_type xEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return clotoidList.back().xEnd_ISO( offs ); }

    virtual
    real_type
    yEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return clotoidList.back().yEnd_ISO( offs ); }

    virtual
    real_type tx_Begin() const G2LIB_OVERRIDE
    { return clotoidList.front().tx_Begin(); }

    virtual
    real_type
    ty_Begin() const G2LIB_OVERRIDE
    { return clotoidList.front().ty_Begin(); }

    virtual
    real_type
    tx_End() const G2LIB_OVERRIDE
    { return clotoidList.back().tx_End(); }

    virtual
    real_type
    ty_End() const G2LIB_OVERRIDE
    { return clotoidList.back().ty_End(); }

    virtual
    real_type
    nx_Begin_ISO() const G2LIB_OVERRIDE
    { return clotoidList.front().nx_Begin_ISO(); }

    virtual
    real_type
    ny_Begin_ISO() const G2LIB_OVERRIDE
    { return clotoidList.front().ny_Begin_ISO(); }

    virtual
    real_type
    nx_End_ISO() const G2LIB_OVERRIDE
    { return clotoidList.back().nx_End_ISO(); }

    virtual
    real_type
    ny_End_ISO() const G2LIB_OVERRIDE
    { return clotoidList.back().ny_End_ISO(); }

    /*\
     |  _   _          _
     | | |_| |__   ___| |_ __ _
     | | __| '_ \ / _ \ __/ _` |
     | | |_| | | |  __/ || (_| |
     |  \__|_| |_|\___|\__\__,_|
    \*/

    virtual
    real_type
    theta( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    theta_D( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    theta_DD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    theta_DDD( real_type s ) const G2LIB_OVERRIDE;

    /*\
     |  _____                   _   _   _
     | |_   _|   __ _ _ __   __| | | \ | |
     |   | |    / _` | '_ \ / _` | |  \| |
     |   | |   | (_| | | | | (_| | | |\  |
     |   |_|    \__,_|_| |_|\__,_| |_| \_|
    \*/

    virtual
    real_type
    tx( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    ty( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    tx_D( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    ty_D( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    tx_DD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    ty_DD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    tx_DDD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    ty_DDD( real_type s ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    tg(
      real_type   s,
      real_type & tg_x,
      real_type & tg_y
    ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_D(
      real_type   s,
      real_type & tg_x_D,
      real_type & tg_y_D
    ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_DD(
      real_type   s,
      real_type & tg_x_DD,
      real_type & tg_y_DD
    ) const G2LIB_OVERRIDE;

    virtual
    void
    tg_DDD(
      real_type   s,
      real_type & tg_x_DDD,
      real_type & tg_y_DDD
    ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    evaluate(
      real_type   s,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    evaluate_ISO(
      real_type   s,
      real_type   offs,
      real_type & th,
      real_type & k,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    real_type
    X( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_D( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_D( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_DD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_DD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_DDD( real_type s ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_DDD( real_type s ) const G2LIB_OVERRIDE;

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
      real_type   s,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_DD(
      real_type   s,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_DDD(
      real_type   s,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE;

    /*\
     |         __  __          _
     |   ___  / _|/ _|___  ___| |_
     |  / _ \| |_| |_/ __|/ _ \ __|
     | | (_) |  _|  _\__ \  __/ |_
     |  \___/|_| |_| |___/\___|\__|
    \*/

    virtual
    real_type
    X_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_ISO( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_ISO_D( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_ISO_D( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_ISO_DD( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_ISO_DD( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    X_ISO_DDD( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    virtual
    real_type
    Y_ISO_DDD( real_type s, real_type offs ) const G2LIB_OVERRIDE;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    virtual
    void
    eval_ISO(
      real_type   s,
      real_type   offs,
      real_type & x,
      real_type & y
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_ISO_D(
      real_type   s,
      real_type   offs,
      real_type & x_D,
      real_type & y_D
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_ISO_DD(
      real_type   s,
      real_type   offs,
      real_type & x_DD,
      real_type & y_DD
    ) const G2LIB_OVERRIDE;

    virtual
    void
    eval_ISO_DDD(
      real_type   s,
      real_type   offs,
      real_type & x_DDD,
      real_type & y_DDD
    ) const G2LIB_OVERRIDE;

    /*\
     |  _                        __
     | | |_ _ __ __ _ _ __  ___ / _| ___  _ __ _ __ ___
     | | __| '__/ _` | '_ \/ __| |_ / _ \| '__| '_ ` _ \
     | | |_| | | (_| | | | \__ \  _| (_) | |  | | | | | |
     |  \__|_|  \__,_|_| |_|___/_|  \___/|_|  |_| |_| |_|
    \*/

    virtual
    void
    translate( real_type tx, real_type ty ) G2LIB_OVERRIDE;

    virtual
    void
    rotate( real_type angle, real_type cx, real_type cy ) G2LIB_OVERRIDE;

    virtual
    void
    scale( real_type sc ) G2LIB_OVERRIDE;

    virtual
    void
    reverse() G2LIB_OVERRIDE;

    virtual
    void
    changeOrigin( real_type newx0, real_type newy0 ) G2LIB_OVERRIDE;

    virtual
    void
    trim( real_type s_begin, real_type s_end ) G2LIB_OVERRIDE;

    /*\
     |      _ _     _
     |   __| (_)___| |_ __ _ _ __   ___ ___
     |  / _` | / __| __/ _` | '_ \ / __/ _ \
     | | (_| | \__ \ || (_| | | | | (_|  __/
     |  \__,_|_|___/\__\__,_|_| |_|\___\___|
    \*/

    /*!
     *  \param  qx  x-coordinate of the point
     *  \param  qy  y-coordinate of the point
     *  \param  x   x-coordinate of the projected point on the curve
     *  \param  y   y-coordinate of the projected point on the curve
     *  \param  s   parameter on the curve of the projection
     *  \param  t   curvilinear coordinate of the point x,y (if orthogonal projection)
     *  \param  dst distance point projected point
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

    /*!
     *  \param  qx  x-coordinate of the point
     *  \param  qy  y-coordinate of the point
     *  \param  offs offset of the curve
     *  \param  x   x-coordinate of the projected point on the curve
     *  \param  y   y-coordinate of the projected point on the curve
     *  \param  s   parameter on the curve of the projection
     *  \param  t   curvilinear coordinate of the point x,y (if orthogonal projection)
     *  \param  dst distance point projected point
     *  \return 1 = point is projected orthogonal
     *          0 = more than one projection (first returned)
     *         -1 = minimum point is not othogonal projection to curve
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
    ) const G2LIB_OVERRIDE;

    /*\
     |      _ _     _
     |   __| (_)___| |_ __ _ _ __   ___ ___
     |  / _` | / __| __/ _` | '_ \ / __/ _ \
     | | (_| | \__ \ || (_| | | | | (_|  __/
     |  \__,_|_|___/\__\__,_|_| |_|\___\___|
    \*/

    /*!
     *  \param  qx  x-coordinate of the point
     *  \param  qy  y-coordinate of the point
     *  \return the segment at minimal distance from point (qx,qy)
     */
    int_type
    closestSegment( real_type qx, real_type qy ) const;

    int_type
    closestPointInRange_ISO(
      real_type   qx,
      real_type   qy,
      int_type    icurve_begin,
      int_type    icurve_end,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst,
      int_type  & icurve
    ) const;

    int_type
    closestPointInRange_SAE(
      real_type   qx,
      real_type   qy,
      int_type    icurve_begin,
      int_type    icurve_end,
      real_type & x,
      real_type & y,
      real_type & s,
      real_type & t,
      real_type & dst,
      int_type  & icurve
    ) const {
      int_type res = this->closestPointInRange_ISO(
        qx, qy, icurve_begin, icurve_end, x, y, s, t, dst, icurve
      );
      t = -t;
      return res;
    }

    virtual
    void
    info( ostream_type & stream ) const G2LIB_OVERRIDE
    { stream << "ClothoidList\n" << *this << '\n'; }

    friend
    ostream_type &
    operator << ( ostream_type & stream, ClothoidList const & CL );

    void
    getSK( real_type s[], real_type kappa[] ) const;

    void
    getSK(
      std::vector<real_type> & s,
      std::vector<real_type> & kappa
    ) const {
      s.resize( clotoidList.size()+1 );
      kappa.resize( clotoidList.size()+1 );
      getSK( &s.front(), &kappa.front() );
    }

    void
    getSTK(
      real_type s[],
      real_type theta[],
      real_type kappa[]
    ) const;

    void
    getSTK(
      std::vector<real_type> & s,
      std::vector<real_type> & theta,
      std::vector<real_type> & kappa
    ) const {
      s.resize( clotoidList.size()+1 );
      theta.resize( clotoidList.size()+1 );
      kappa.resize( clotoidList.size()+1 );
      getSTK( &s.front(), &theta.front(), &kappa.front() );
    }

    void
    getXY( real_type x[], real_type y[] ) const;

    void
    getDeltaTheta( real_type deltaTheta[] ) const;

    void
    getDeltaKappa( real_type deltaKappa[] ) const;

    /*!
     *  \brief Find parametric coordinate.
     *
     *  \param  x    x-coordinate point
     *  \param  y    y-coordinate point
     *  \param  s    value \f$ s \f$
     *  \param  t    value \f$ t \f$
     *  \return idx  the segment with point at minimal distance, otherwise
     *               -(idx+1) if (x,y) cannot be projected orthogonally on the segment
     *
     */
    int_type
    findST1(
      real_type   x,
      real_type   y,
      real_type & s,
      real_type & t
    ) const;

    /*!
     *  \brief Find parametric coordinate.
     *
     *  \param  ibegin initial segment to compute the distance
     *  \param  iend   final segment to compute the distance
     *  \param  x      x-coordinate point
     *  \param  y      y-coordinate point
     *  \param  s      value \f$ s \f$
     *  \param  t      value \f$ t \f$
     *  \return idx    the segment with point at minimal distance, otherwise
     *                 -(idx+1) if (x,y) cannot be projected orthogonally on the segment
     */
    int_type
    findST1(
      int_type    ibegin,
      int_type    iend,
      real_type   x,
      real_type   y,
      real_type & s,
      real_type & t
    ) const;

    /*\
     |             _ _ _     _
     |    ___ ___ | | (_)___(_) ___  _ __
     |   / __/ _ \| | | / __| |/ _ \| '_ \
     |  | (_| (_) | | | \__ \ | (_) | | | |
     |   \___\___/|_|_|_|___/_|\___/|_| |_|
    \*/

    bool
    collision( ClothoidList const & C ) const;

    bool
    collision_ISO(
      real_type            offs,
      ClothoidList const & CL,
      real_type            offs_C
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
      ClothoidList const & CL,
      IntersectList      & ilist,
      bool                 swap_s_vals
    ) const {
      intersect_ISO( 0, CL, 0, ilist, swap_s_vals );
    }

    void
    intersect_ISO(
      real_type            offs,
      ClothoidList const & CL,
      real_type            offs_obj,
      IntersectList      & ilist,
      bool                 swap_s_vals
    ) const;

    /*! \brief Save Clothoid list to a stream
     *
     * \param stream stream to save
     */
    void
    export_table( ostream_type & stream ) const;

    /*! \brief Save Clothoid list to a stream
     *
     * \param stream streamstream to save
     */
    void
    export_ruby( ostream_type & stream ) const;

  };

  /*\
   |
   |    ___ _     _   _        _    _ ___      _ _           ___ ___
   |   / __| |___| |_| |_  ___(_)__| / __|_ __| (_)_ _  ___ / __|_  )
   |  | (__| / _ \  _| ' \/ _ \ / _` \__ \ '_ \ | | ' \/ -_) (_ |/ /
   |   \___|_\___/\__|_||_\___/_\__,_|___/ .__/_|_|_||_\___|\___/___|
   |                                     |_|
  \*/

  //! Class for the computation of G2 spljne of clothoids
  class ClothoidSplineG2 {
  public:
    typedef enum { P1 = 1, P2, P3, P4, P5, P6, P7, P8, P9 } TargetType;

  private:

    vector<real_type> x;
    vector<real_type> y;
    TargetType        tt;
    real_type         theta_I;
    real_type         theta_F;
    int_type          npts;

    // work vector
    mutable vector<real_type> k, dk, L, kL, L_1, L_2, k_1, k_2, dk_1, dk_2;

    real_type
    diff2pi( real_type in ) const {
      return in-m_2pi*round(in/m_2pi);
    }

  public:

    ClothoidSplineG2() : tt(P1) {}
    ~ClothoidSplineG2() {}

    void
    setP1( real_type theta0, real_type thetaN )
    { tt = P1; theta_I = theta0; theta_F = thetaN; }

    void setP2() { tt = P2; }
    void setP3() { tt = P3; }
    void setP4() { tt = P4; }
    void setP5() { tt = P5; }
    void setP6() { tt = P6; }
    void setP7() { tt = P7; }
    void setP8() { tt = P8; }
    void setP9() { tt = P9; }

    void
    build(
      real_type const xvec[],
      real_type const yvec[],
      int_type        npts
    );

    int_type numPnts() const { return npts; }
    int_type numTheta() const;
    int_type numConstraints() const;

    void
    guess(
      real_type theta_guess[],
      real_type theta_min[],
      real_type theta_max[]
    ) const;

    bool
    objective( real_type const theta[], real_type & f ) const;

    bool
    gradient( real_type const theta[], real_type g[] ) const;

    bool
    constraints( real_type const theta[], real_type c[] ) const;

    int_type
    jacobian_nnz() const;

    bool
    jacobian_pattern( int_type i[], int_type j[] ) const;

    bool
    jacobian_pattern_matlab( real_type i[], real_type j[] ) const;

    bool
    jacobian( real_type const theta[], real_type vals[] ) const;

    void
    info( ostream_type & stream ) const
    { stream << "ClothoidSplineG2\n" << *this << '\n'; }

    friend
    ostream_type &
    operator << ( ostream_type & stream, ClothoidSplineG2 const & c );

  };

}

#endif

///
/// eof: ClothoidList.hh
///
