#ifndef _ABSTRACT_CAMERA_HPP_
#define _ABSTRACT_CAMERA_HPP_

#include <vector>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>
// #include <boost/noncopyable.hpp>

#include <eigen3/Eigen/StdVector>

class noncopyable
{
protected:
    noncopyable() = default;
    ~noncopyable() = default;

    noncopyable(const noncopyable&) = delete;
    noncopyable &operator=(const noncopyable&) = delete;
};

namespace camera_model 
{
using namespace Eigen;
using namespace cv;

class AbstractCamera : public noncopyable 
{
public:

    enum Model {
        UNKNOW      = -2,
        ABSTRACT    = -1,
        PINHOLE     = 0,
        MEI        = 1
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::shared_ptr<AbstractCamera> Ptr;

    AbstractCamera();

    AbstractCamera(Model model = ABSTRACT);

    AbstractCamera(int width, int height, Model model = ABSTRACT);

    AbstractCamera(int width, int height, double fx, double fy, double cx, double cy, Model model = ABSTRACT);

    virtual ~AbstractCamera() {};

    static Model checkCameraModel(std::string calib_file);

    inline const int getWidth() const { return width_; }

    inline const int getHeight() const { return height_; }

    inline const double getFx() const { return fx_; };

    inline const double getFy() const { return fy_; };

    inline const double getCx() const { return cx_; };

    inline const double getCy() const { return cy_; };

    inline const cv::Mat getK() const { return K_; };

    inline const cv::Mat getD() const { return D_; };

    inline const Model getCamModel() const { return model_; }

    virtual const double xi() const;

    virtual Vector3d unproject(const Vector2d& pix) const;

    virtual Vector3d unproject(double x_pix, double y_pix) const;

    virtual Vector2d project(const Vector3d& xyz) const;

    virtual Vector2d project(double x, double y, double z=1) const;

    virtual void undistortPoints(const std::vector<cv::Point2f> &pts_dist, std::vector<cv::Point2f> &pts_udist) const;

    virtual void undistortImage(const cv::Mat& img_dist, cv::Mat& img_udist) const;

    inline bool isInFrame(const Vector2d& obs, int boundary = 0) const {
        if (obs[0] >= boundary && obs[0] < getWidth() - boundary && obs[1] >= boundary &&
            obs[1] < getHeight() - boundary)
            return true;
        return false;
    }

    inline bool isInFrame(const Vector2d& obs, int boundary, int level) const {
        if (obs[0] >= boundary && obs[0] < (getWidth() >> level) - boundary && obs[1] >= boundary &&
            obs[1] < (getHeight() >> level) - boundary)
            return true;
        return false;
    }

protected:
    const Model model_;
    int width_;
    int height_;
    double fx_, fy_, cx_, cy_;
    cv::Mat K_, D_;
    bool distortion_; 

};

} // end namespace


#endif  // _ABSTRACT_CAMERA_HPP_