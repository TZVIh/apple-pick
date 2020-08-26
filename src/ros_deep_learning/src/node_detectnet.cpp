#include "node_detectnet.h"

namespace detect_object {

DetectObject::DetectObject(ros::NodeHandle nh, ros::NodeHandle private_nh){
    nh_ = nh;
    private_nh_ = private_nh;
}

DetectObject::~DetectObject(){
            delete net;
            delete input_cvt;
            delete overlay_cvt;
}
void DetectObject::readNodeParameters() {
    private_nh_.param<std::string>("prototxt_path", prototxt_path,"  ");
    private_nh_.param<std::string>("model_path", model_path," ");
    private_nh_.param<float>("mean_pixel_value", mean_pixel,0.0);
    private_nh_.param<float>("threshold", threshold,0.5);
    private_nh_.param<std::string>("class_labels_path", class_labels_path," ");
    private_nh_.param<int>("require_class_id", require_class_id,19);
}
void DetectObject::initializeSubscribers() {
     img_sub = nh_.subscribe("image_in", 1, &DetectObject::imgCallback,this);
     depth_sub = nh_.subscribe("depth_in", 1, &DetectObject::depthCallback,this);
     //        ros::Subscriber img_sub = private_nh.subscribe("/camera/color/image_raw", 1, imgCallback);
     //        ros::Subscriber depth_sub = private_nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthCallback);
}

void DetectObject::initializePublishers() {
    overlay_pub = nh_.advertise<sensor_msgs::Image>("deep_overlay", 0);
}

bool DetectObject::initializeNode(){
    net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), mean_pixel, class_labels_path.c_str(), threshold);
    if( !net ){
            ROS_ERROR("failed to load detectNet model");
            return 0;
    }
    input_cvt = new imageConverter();
    overlay_cvt = new imageConverter();
    if( !input_cvt || !overlay_cvt  ){
            ROS_ERROR("failed to create imageConverter object");
            return 0;
    }
    got_depth = false;
    return true;
}
void DetectObject::getCameraIntrinsicMatrix(const cv::Mat& intr_rect_ir){
    if (!get_intrinsic_param){
        cx = intr_rect_ir.at<double>(0,2);
        cy = intr_rect_ir.at<double>(1,2);
        fx_inv = 1.0 / intr_rect_ir.at<double>(0,0);
        fy_inv = 1.0 / intr_rect_ir.at<double>(1,1);
    }
}
bool DetectObject::publishOverlay( detectNet::Detection* detections, int numDetections )
{
        // get the image dimensions
        const uint32_t width  = input_cvt->GetWidth();
        const uint32_t height = input_cvt->GetHeight();

        // assure correct image size
        if( !overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
                return false;

        // generate the overlay
        if( !net->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height,
                                   imageConverter::InternalFormat, detections, numDetections, overlay_flags) )
        {
                return false;
        }

        // populate the message
        sensor_msgs::Image msg;

        if( !overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat) )
                return false;

        // populate timestamp in header field
        msg.header.stamp = ROS_TIME_NOW();

        // publish the message
        overlay_pub->publish(msg);
        ROS_DEBUG("publishing %ux%u overlay image", width, height);
}


// input image subscriber callback
void DetectObject::imgCallback( const sensor_msgs::ImageConstPtr& input )
{
        // convert the image to reside on GPU
        if( !input_cvt || !input_cvt->Convert(input) )
        {
                ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
                return;
        }

        // classify the image
        detectNet::Detection* detections = NULL;
        const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

        // verify success
        if( numDetections < 0 )
        {
                ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
                return;
        }

        // if objects were detected, send out message
        if( numDetections > 0 )
        {
                ROS_INFO("detected %i objects in %ux%u image", numDetections, input->width, input->height);
                // create a detection for each bounding box
                std::vector<object_pixels> detections_vector;
                for( int n=0; n < numDetections; n++ )
                {
                        detectNet::Detection* det = detections + n;
                        if (det->ClassID == require_class_id){
                            printf("object %i class #%u (%s)  confidence=%f\n", n, det->ClassID, net->GetClassDesc(det->ClassID), det->Confidence);
                            printf("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height());

                            float cx, cy;
                            det->Center(&cx, &cy);

                            object_pixels p;
                            p.center.x =cx;
                            p.center.y =cy;
                            p.height = det->Height();
                            p.width = det->Width();
                            detections_vector.push_back(p);
                        }
                }

        }
        if (got_depth){
            calcObjectsDepth(detections_vector);
        }
}
void DetectObject::calcObjectsDepth(std::vector<object_pixels> detections_vector){
    for (int i=0;i<detections_vector.size();i++){
        point pt;
        calcObjectXYZ(depth_img,detections_vector.at(i).center,pt);
    }
}

void DetectObject::calcObjectXYZ(const cv::Mat& depth_img_rect,
  pixel &pix,
  point &pt)
{
    uint16_t z = depth_img_rect.at<uint16_t>(pix.v, pix.u);

    if (z != 0)
    {
      double z_metric = z * 0.001;
      pt.x = z_metric * ((pix.u - cx) * fx_inv);
      pt.y = z_metric * ((pix.v - cy) * fy_inv);
      pt.z = z_metric;
    }
    else
    {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    }
}

void DetectObject::depthCallback(const sensor_msgs::ImageConstPtr& input){
    cv_bridge::CvImagePtr cv_ptr;
        try
           {
             cv_ptr = cv_bridge::toCvCopy(input,sensor_msgs::image_encodings::TYPE_16UC1);
             depth_img = cv_ptr->image;
             got_depth = true;
           }
        catch (cv_bridge::Exception& e)
           {
             ROS_ERROR("cv_bridge exception: %s", e.what());
             return;
           }
}
}
