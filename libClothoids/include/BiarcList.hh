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
/// file: BiarcList.hh
///

#ifndef BIARC_LIST_HH
#define BIARC_LIST_HH

#include "PolyLine.hh"
#include "Biarc.hh"

//! Clothoid computations routine
namespace G2lib {

  using std::vector;

  class ClothoidList;

  /*\
   |  ____  _                _     _     _
   | | __ )(_) __ _ _ __ ___| |   (_)___| |_
   | |  _ \| |/ _` | '__/ __| |   | / __| __|
   | | |_) | | (_| | | | (__| |___| \__ \ |_
   | |____/|_|\__,_|_|  \___|_____|_|___/\__|
  \*/
  //! \brief Class to manage a list of biarc Curve (not necessarily G2 or G1 connected)
  class BiarcList : public BaseCurve {

    friend class ClothoidList;

    vector<real_type> s0;
    vector<Biarc>     biarcList;
    mutable int_type  last_idx;

    mutable bool               aabb_done;
    mutable AABBtree           aabb_tree;
    mutable real_type          aabb_offs;
    mutable real_type          aabb_max_angle;
    mutable real_type          aabb_max_size;
    mutable vector<Triangle2D> aabb_tri;

    class T2D_collision_list_ISO {
      BiarcList const * pList1;
      real_type const   offs1;
      BiarcList const * pList2;
      real_type const   offs2;
    public:
      T2D_collision_list_ISO(
        BiarcList const * _pList1,
        real_type const   _offs1,
        BiarcList const * _pList2,
        real_type const   _offs2
      )
      : pList1(_pList1)
      , offs1(_offs1)
      , pList2(_pList2)
      , offs2(_offs2)
      {}

      bool
      operator () ( BBox::PtrBBox ptr1, BBox::PtrBBox ptr2 ) const {
        Triangle2D const & T1 = pList1->aabb_tri[size_t(ptr1->Ipos())];
        Triangle2D const & T2 = pList2->aabb_tri[size_t(ptr2->Ipos())];
        Biarc      const & C1 = pList1->get(T1.Icurve());
        Biarc      const & C2 = pList2->get(T2.Icurve());
        return C1.collision_ISO( offs1, C2, offs2 );
      }
    };

  public:

    #include "BaseCurve_using.hxx"

    //explicit
    BiarcList()
    : BaseCurve(G2LIB_BIARC_LIST)
    , last_idx(0)
    , aabb_done(false)
    {}

    virtual
    ~BiarcList() G2LIB_OVERRIDE {
      s0.clear();
      biarcList.clear();
      aabb_tri.clear();
    }

    //explicit
    BiarcList( BiarcList const & s )
    : BaseCurve(G2LIB_BIARC_LIST)
    , last_idx(0)
    , aabb_done(false)
    { copy(s); }

    void init();
    void reserve( int_type n );
    void copy( BiarcList const & L );

    BiarcList const & operator = ( BiarcList const & s )
    { copy(s); return *this; }

    explicit
    BiarcList( LineSegment const & LS );

    explicit
    BiarcList( CircleArc const & C );

    explicit
    BiarcList( Biarc const & C );

    explicit
    BiarcList( PolyLine const & pl );

    explicit
    BiarcList( BaseCurve const & C );

    void push_back( LineSegment const & c );
    void push_back( CircleArc const & c );
    void push_back( Biarc const & c );
    void push_back( PolyLine const & c );

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

    Biarc const & get( int_type idx ) const;
    Biarc const & getAtS( real_type s ) const;

    int_type numSegment() const { return int_type(biarcList.size()); }

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
      bbTriangles_ISO( -offs, tvec, max_angle, max_size );
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

    void
    build_AABBtree_SAE(
      real_type offs,
      real_type max_angle = m_pi/6, // 30 degree
      real_type max_size  = 1e100
    ) const {
      build_AABBtree_ISO( -offs, max_angle, max_size );
    }

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
    { return biarcList.front().thetaBegin(); }

    virtual
    real_type
    thetaEnd() const G2LIB_OVERRIDE
    { return biarcList.back().thetaEnd(); }

    virtual
    real_type
    xBegin() const G2LIB_OVERRIDE
    { return biarcList.front().xBegin(); }

    virtual
    real_type
    yBegin() const G2LIB_OVERRIDE
    { return biarcList.front().yBegin(); }

    virtual
    real_type
    xEnd() const G2LIB_OVERRIDE
    { return biarcList.back().xEnd(); }

    virtual
    real_type
    yEnd() const G2LIB_OVERRIDE
    { return biarcList.back().yEnd(); }

    virtual
    real_type
    xBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return biarcList.front().xBegin_ISO( offs ); }

    virtual
    real_type
    yBegin_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return biarcList.front().yBegin_ISO( offs ); }

    virtual
    real_type
    xEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return biarcList.back().xEnd_ISO( offs ); }

    virtual
    real_type
    yEnd_ISO( real_type offs ) const G2LIB_OVERRIDE
    { return biarcList.back().yEnd_ISO( offs ); }

    virtual
    real_type
    tx_Begin() const G2LIB_OVERRIDE
    { return biarcList.front().tx_Begin(); }

    virtual
    real_type
    ty_Begin() const G2LIB_OVERRIDE
    { return biarcList.front().ty_Begin(); }

    virtual
    real_type
    tx_End() const G2LIB_OVERRIDE
    { return biarcList.back().tx_End(); }

    virtual
    real_type
    ty_End() const G2LIB_OVERRIDE
    { return biarcList.back().ty_End(); }

    virtual
    real_type
    nx_Begin_ISO() const G2LIB_OVERRIDE
    { return biarcList.front().nx_Begin_ISO(); }

    virtual
    real_type
    ny_Begin_ISO() const G2LIB_OVERRIDE
    { return biarcList.front().ny_Begin_ISO(); }

    virtual
    real_type
    nx_End_ISO() const G2LIB_OVERRIDE
    { return biarcList.back().nx_End_ISO(); }

    virtual
    real_type
    ny_End_ISO() const G2LIB_OVERRIDE
    { return biarcList.back().ny_End_ISO(); }

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

    virtual
    void
    info( ostream_type & stream ) const G2LIB_OVERRIDE
    { stream << "BiarcList\n" << *this << '\n'; }

    friend
    ostream_type &
    operator << ( ostream_type & stream, BiarcList const & CL );

    void
    getSTK(
      real_type s[],
      real_type theta[],
      real_type kappa[]
    ) const;

    void
    getXY( real_type x[], real_type y[] ) const;

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
    collision( BiarcList const & C ) const;

    bool
    collision_ISO(
      real_type         offs,
      BiarcList const & CL,
      real_type         offs_C
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
      BiarcList const & CL,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const {
      intersect_ISO( 0, CL, 0, ilist, swap_s_vals );
    }

    void
    intersect_ISO(
      real_type         offs,
      BiarcList const & CL,
      real_type         offs_obj,
      IntersectList   & ilist,
      bool              swap_s_vals
    ) const;

  };

}

#endif

///
/// eof: BiarcList.hh
///
