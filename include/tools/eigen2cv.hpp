/**
 * @brief   Mapping functions from Eigen data types to OpenCV
 * @author  Eugene Khvedchenya <ekhvedchenya@gmail.com>
 * @details This header file contains code snippet for easy mapping Eigen types
 * to OpenCV and back with minimal overhead.
 * @more    computer-vision.talks.com/articles/mapping-eigen-to-opencv/
 * Features:
 *  - Mapping plain data types with no overhead (read/write access)
 *  - Mapping expressions via evaluation (read only acess)
 *
 * Known issues:
 *  - Does not support .transpose()
 */
#ifndef INTERNAL_GRID_MAP_EIGEN2CV_H
#define INTERNAL_GRID_MAP_EIGEN2CV_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace details {
struct Const {};
struct Mutable {};
}

template <typename T>
struct opencv_matrix;

template <>
struct opencv_matrix<double> {
    static const int type = CV_64FC1;
};
template <>
struct opencv_matrix<float> {
    static const int type = CV_32FC1;
};
template <>
struct opencv_matrix<int> {
    static const int type = CV_32SC1;
};
template <>
struct opencv_matrix<unsigned char> {
    static const int type = CV_8UC1;
};
template <>
struct opencv_matrix<bool> {
    static_assert(sizeof(bool) == 1, "Requires bool to be 1 byte");
    static const int type = CV_8UC1;
};

/**
 * PlainObjectBase<D> - map directly (read/write)
 * ArrayWrapper/MatrixWrapper on PlainObjectBase - map directly (read/write)
 * Block<D> - recursively instantiate. Can be read-only (derived is expression)
 * or read/write (derived is planar)
 * Expression - evaluate (read-only)
 */
template <typename Derived, typename Base, typename ConstPolicy,
          typename StorageKind =
                  typename Eigen::internal::traits<Derived>::StorageKind>
class Eigen2CV;

class Eigen2CVBase {
 public:
    operator cv::Mat() const { return mBody; }

    operator cv::_InputArray() const { return cv::_InputArray(mBody); }

 protected:
    template <typename Derived>
    void mapPlaneMemory(const Derived& src) {
        const bool isRowMajor = int(Derived::Flags) & Eigen::RowMajorBit;
        const int stride = src.outerStride() * sizeof(typename Derived::Scalar);

        if (isRowMajor)
            this->mapPlaneMemoryRowMajor(src.data(), src.rows(), src.cols(),
                                         stride);
        else
            this->mapPlaneMemoryColMajor(src.data(), src.rows(), src.cols(),
                                         stride);
    }

    template <typename Scalar>
    void mapPlaneMemoryRowMajor(const Scalar* planeData, int rows, int cols,
                                int stride) {
        this->mBody = cv::Mat(rows, cols, opencv_matrix<Scalar>::type,
                              const_cast<Scalar*>(planeData), stride);
    }

    template <typename Scalar>
    void mapPlaneMemoryColMajor(const Scalar* planeData, int rows, int cols,
                                int stride) {
        this->mBody = cv::Mat(cols, rows, opencv_matrix<Scalar>::type,
                              const_cast<Scalar*>(planeData), stride);
    }

    template <typename Derived, typename T>
    void assignMatrix(Eigen::DenseBase<Derived>& dst, const cv::Mat_<T>& src) {
        typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic,
                              Eigen::RowMajor>
                PlainMatrixType;

        dst = Eigen::Map<PlainMatrixType>((T*)src.data, src.rows, src.cols)
                      .template cast<typename Derived::Scalar>();
    }

    cv::Mat mBody;
};

template <typename Derived>
class Eigen2CV<Derived, Eigen::PlainObjectBase<Derived>, details::Const>
        : public Eigen2CVBase {
 public:
    typedef typename Derived::Scalar Scalar;
    typedef Eigen2CV<Derived, Eigen::PlainObjectBase<Derived>, details::Mutable>
            Self;

    Eigen2CV(const Eigen::PlainObjectBase<Derived>& src) : mMappedView(src) {
        this->mapPlaneMemory(mMappedView);
    }

 private:
    const Eigen::PlainObjectBase<Derived>& mMappedView;
};

template <typename Derived>
class Eigen2CV<Derived, Eigen::PlainObjectBase<Derived>, details::Mutable>
        : public Eigen2CVBase {
 public:
    typedef typename Derived::Scalar Scalar;
    typedef Eigen2CV<Derived, Eigen::PlainObjectBase<Derived>, details::Mutable>
            Self;

    Eigen2CV(Eigen::PlainObjectBase<Derived>& src) : mMappedView(src) {
        this->mapPlaneMemory(mMappedView);
    }

    template <typename T>
    Self& operator=(const cv::Mat_<T>& src) {
        assignMatrix<Derived, T>(mMappedView, src);
        return *this;
    }

    /**
     * @brief Assignment operator to copy OpenCV Mat data to mapped Eigen
     * object.
     */
    Self& operator=(const cv::Mat& m) {
        switch (m.type()) {
            case CV_8U:
                return *this = (cv::Mat_<uint8_t>)m;
            case CV_16U:
                return *this = (cv::Mat_<uint16_t>)m;
            case CV_16S:
                return *this = (cv::Mat_<int16_t>)m;
            case CV_32S:
                return *this = (cv::Mat_<int32_t>)m;
            case CV_32F:
                return *this = (cv::Mat_<float>)m;
            case CV_64F:
                return *this = (cv::Mat_<double>)m;
            default:
                throw std::runtime_error("Unsupported OpenCV matrix type");
        };
    }

    operator cv::_InputOutputArray() {
        return cv::_InputOutputArray(this->mBody);
    }

 protected:
 private:
    Eigen::PlainObjectBase<Derived>& mMappedView;
};

template <typename Derived>
class Eigen2CV<Derived, Eigen::Block<Derived>, details::Mutable, Eigen::Dense>
        : public Eigen2CVBase {
 public:
    typedef typename Derived::Scalar Scalar;
    typedef Eigen2CV<Derived, Eigen::Block<Derived>, details::Mutable> Self;

    Eigen2CV(const Eigen::Block<Derived>& src) : mMappedView(src) {
        this->mapPlaneMemory(mMappedView);
    }

    operator cv::_OutputArray() { return cv::_OutputArray(this->mBody); }

    template <typename T>
    Self& operator=(const cv::Mat_<T>& src) {
        assignMatrix<Derived, T>(mMappedView, src);
        return *this;
    }

    /**
     * @brief Assignment operator to copy OpenCV Mat data to mapped Eigen
     * object.
     */
    Self& operator=(const cv::Mat& m) throw() {
        switch (m.type()) {
            case CV_8U:
                return *this = (cv::Mat_<uint8_t>)m;
            case CV_16U:
                return *this = (cv::Mat_<uint16_t>)m;
            case CV_16S:
                return *this = (cv::Mat_<int16_t>)m;
            case CV_32S:
                return *this = (cv::Mat_<int32_t>)m;
            case CV_32F:
                return *this = (cv::Mat_<float>)m;
            case CV_64F:
                return *this = (cv::Mat_<double>)m;
            default:
                throw std::runtime_error("Unsupported OpenCV matrix type");
        };
    }

 private:
    const Eigen::Block<Derived>& mMappedView;
};

/**
 * @brief Most generic and most inefficient mapper - it evaluates input object
 * into a local copy.
 */
template <typename Derived>
class Eigen2CV<Derived, Eigen::EigenBase<Derived>, details::Const>
        : public Eigen2CVBase {
 public:
    typedef typename Derived::Scalar Scalar;
    typedef typename Eigen::internal::plain_matrix_type<Derived>::type
            StorageType;

    Eigen2CV(const Eigen::EigenBase<Derived>& src) {
        mStorage = src;
        this->mapPlaneMemory(mStorage);
    }

 private:
    StorageType mStorage;
};

template <typename E>
Eigen2CV<E, Eigen::PlainObjectBase<E>, details::Mutable> eigen2cv(
        Eigen::PlainObjectBase<E>& src) {
    return Eigen2CV<E, Eigen::PlainObjectBase<E>, details::Mutable>(src);
}

template <typename E>
Eigen2CV<E, Eigen::PlainObjectBase<E>, details::Const> eigen2cv(
        const Eigen::PlainObjectBase<E>& src) {
    return Eigen2CV<E, Eigen::PlainObjectBase<E>, details::Const>(src);
}

template <typename E>
Eigen2CV<E, Eigen::Block<E>, details::Mutable> eigen2cv(
        const Eigen::Block<E>& src) {
    return Eigen2CV<E, Eigen::Block<E>, details::Mutable>(src);
}

template <typename E>
auto eigen2cv(const Eigen::ArrayWrapper<E>& src)
        -> decltype(eigen2cv(src.nestedExpression().eval())) {
    return eigen2cv(src.nestedExpression().eval());
}

template <typename E>
auto eigen2cv(const Eigen::MatrixWrapper<E>& src)
        -> decltype(eigen2cv(src.nestedExpression().eval())) {
    return eigen2cv(src.nestedExpression().eval());
}

template <typename E>
Eigen2CV<E, Eigen::EigenBase<E>, details::Const> eigen2cv(
        const Eigen::EigenBase<E>& src) {
    return Eigen2CV<E, Eigen::EigenBase<E>, details::Const>(src);
}  // namespace hmpl
#endif  // INTERNAL_GRID_MAP_EIGEN2CV_H
