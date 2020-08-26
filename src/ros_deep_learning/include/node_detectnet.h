#ifndef NODE_DETECTNET_H
#define NODE_DETECTNET_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <jetson-inference/detectNet.h>
#include <jetson-utils/cudaMappedMemory.h>

#include "image_converter.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
namespace detect_object {

struct point{
    float x;
    float y;
    float z;
};

struct pixel{
    float v;
    float u;
};

struct object_pixels{
    pixel center;
    float width;
    float height;
};

class DetectObject{
public:
    DetectObject(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~DetectObject();
    bool initializeNode();
private:
    void readNodeParameters();
    void initializeSubscribers();
    void initializePublishers();

    void getCameraIntrinsicMatrix(const cv::Mat& intr_rect_ir);
    void publishOverlay( detectNet::Detection* detections, int numDetections );
    void imgCallback( const sensor_msgs::ImageConstPtr& input );
    void calcObjectsDepth(std::vector<object_pixels> detections_vector);
    void calcObjectXYZ(const cv::Mat& depth_img_rect, pixel &pix, point &pt);
    void depthCallback(const sensor_msgs::ImageConstPtr& input);

    ros::NodeHandle nh_,private_nh_;

    ros::Subscriber img_sub;
    ros::Subscriber depth_sub;
    ros::Publisher overlay_pub;

    std::string class_labels_path;
    std::string prototxt_path;
    std::string model_path;
    detectNet* 	 net ;
    imageConverter* input_cvt   = NULL;
    imageConverter* overlay_cvt = NULL;

    double cx,cy,fx_inv,fy_inv;
    bool get_intrinsic_param = false;
    float mean_pixel;
    float threshold;
    cv::Mat depth_img;
    bool got_depth;
    int require_class_id;


};
}
#endif // NODE_DETECTNET_H
