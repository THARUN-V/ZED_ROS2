#include <stdio.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <image_transport/image_transport.hpp>

#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <camera_info_manager/camera_info_manager.hpp>

#define WIDTH_ID 3
#define HEIGHT_ID 4
#define FPS_ID 5

namespace arti
{
  class StereoCamera
  {
    public:
      /**
       * @brief      { stereo camera driver }
       *
       * @param[in]  resolution  The resolution
       * @param[in]  frame_rate  The frame rate
       */
      StereoCamera(std::string device_name, int resolution, double frame_rate) : frame_rate_(30.0)
      {
        camera_ = new cv::VideoCapture(device_name);
        cv::Mat raw;
        cv::Mat left_image;
        cv::Mat right_image;
        setResolution(resolution);
        // // this function doesn't work very well in current Opencv 2.4, so, just use ROS to control frame rate.
        // setFrameRate(frame_rate);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Stereo Camera Set Resolution %d, width %f, height %f", resolution, camera_->get(WIDTH_ID),
                camera_->get(HEIGHT_ID));
      }

      ~StereoCamera()
      {
        // std::cout << "Destroy the pointer" << std::endl;
        delete camera_;
      }

      /**
       * @brief      Sets the resolution.
       *
       * @param[in]  type  The type
       */
      void setResolution(int type)
      {
        switch (type)
        {
          case 0:
            width_ = 4416;
            height_ = 1242;
            break;
          case 1:
            width_ = 3840;
            height_ = 1080;
            break;
          case 2:
            width_ = 2560;
            height_ = 720;
            break;
          case 3:
            width_ = 1344;
            height_ = 376;
            break;
          default:
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"Unknow resolution passed to camera: %d", type);
        }

        camera_->set(WIDTH_ID, width_);
        camera_->set(HEIGHT_ID, height_);
        // make sure that the number set are right from the hardware
        width_ = camera_->get(WIDTH_ID);
        height_ = camera_->get(HEIGHT_ID);
      }

      /**
       * @brief      Sets the frame rate.
       *
       * @param[in]  frame_rate  The frame rate
       */
      void setFrameRate(double frame_rate)
      {
        camera_->set(FPS_ID, frame_rate);
        frame_rate_ = camera_->get(FPS_ID);
      }

      /**
       * @brief      Gets the images.
       *
       * @param      left_image   The left image
       * @param      right_image  The right image
       *
       * @return     The images.
       */
      bool getImages(cv::Mat& left_image, cv::Mat& right_image)
      {
        cv::Mat raw;
        if (camera_->grab())
        {
          camera_->retrieve(raw);
          cv::Rect left_rect(0, 0, width_ / 2, height_);
          cv::Rect right_rect(width_ / 2, 0, width_ / 2, height_);
          left_image = raw(left_rect);
          right_image = raw(right_rect);
          cv::waitKey(10);
          return true;
        }
        else
        {
          return false;
        }
      }

    private:
      cv::VideoCapture* camera_;
      int width_;
      int height_;
      double frame_rate_;
      bool cv_three_;
  };

  /**
   * @brief       the camera ros warpper class
  */

  class ZedCameraROS
  {

    private:
      int resolution_;
      std::string device_name_,left_frame_id_, right_frame_id_,config_file_location_,encoding_;
      double frame_rate_;
      bool show_image_, use_zed_config_;
      double width_, height_;
      
    public:
      /**
     * @brief      { function_description }
     *
     * @param[in]  resolution  The resolution
     * @param[in]  frame_rate  The frame rate
     */

      ZedCameraROS()
      {
        rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("zed_camera");

        // Declare Parameter
        nh->declare_parameter("resolution",1);
        nh->declare_parameter("frame_rate",30.0);
        nh->declare_parameter("config_file_location","/home/tharun/ros2_ws/src/zed_cpu_ros2/config/SN1267.conf");
        nh->declare_parameter("left_frame_id","left_camera");
        nh->declare_parameter("right_frame_id","right_camera");
        nh->declare_parameter("show_img",false);
        nh->declare_parameter("use_zed_config",true);
        nh->declare_parameter("device_name","/dev/video0");
        nh->declare_parameter("encoding","bgr8");

        //Get Parameter
        resolution_           = nh->get_parameter("resolution").as_int();
        frame_rate_           = nh->get_parameter("frame_rate").as_double();
        config_file_location_ = nh->get_parameter("config_file_location").as_string();
        left_frame_id_        = nh->get_parameter("left_frame_id").as_string();
        right_frame_id_       = nh->get_parameter("right_frame_id").as_string();
        show_image_           = nh->get_parameter("show_img").as_bool();
        use_zed_config_       = nh->get_parameter("use_zed_config").as_bool();
        device_name_          = nh->get_parameter("device_name").as_string();
        encoding_             = nh->get_parameter("encoding").as_string();


        correctFramerate(resolution_, frame_rate_);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Try to initialize the camera");
        StereoCamera zed(device_name_, resolution_, frame_rate_);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Initialized the camera");


        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        image_transport::Publisher left_image_pub = it.advertise("left/image_raw", 1);
        image_transport::Publisher right_image_pub = it.advertise("right/image_raw", 1);

        auto left_cam_info_pub  = nh->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
        auto right_cam_info_pub = nh->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);

        sensor_msgs::msg::CameraInfo left_info, right_info;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Try load camera calibration files");

        if (use_zed_config_)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Loading from zed calibration files");
          // get camera info from zed
          if (!config_file_location_.empty())
          {
            try
            {
              getZedCameraInfo(config_file_location_, resolution_, left_info, right_info);
            }
            catch (std::runtime_error& e)
            {
              RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Can't load camera info");
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"%s", e.what());
              throw e;
            }
          }
          else
          {
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"Please input zed config file path");
          }
        }
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Loading from ROS calibration files");
          // here we just use camera infor manager to load info
          // get config from the left, right.yaml in config
          // ros::NodeHandle left_nh("left");
          // ros::NodeHandle right_nh("right");
          // camera_info_manager::CameraInfoManager left_info_manager(left_nh, "camera/left",
          //                                                         "package://zed_cpu_ros/config/left.yaml");
          // left_info = left_info_manager.getCameraInfo();

          // camera_info_manager::CameraInfoManager right_info_manager(right_nh, "camera/right",
          //                                                           "package://zed_cpu_ros/config/right.yaml");
          // right_info = right_info_manager.getCameraInfo();

          // left_info.header.frame_id = left_frame_id_;
          // right_info.header.frame_id = right_frame_id_;

          rclcpp::Node left_nh("left");
          rclcpp::Node right_nh("right");

          camera_info_manager::CameraInfoManager left_info_manager(&left_nh,"camera/left","package://zed_cpu_ros2/config/left.yaml");
          left_info = left_info_manager.getCameraInfo();

          camera_info_manager::CameraInfoManager right_info_manager(&right_nh,"camera/right","package://zed_cpu_ros2/config/right.yaml");
          right_info = right_info_manager.getCameraInfo();

          left_info.header.frame_id = left_frame_id_;
          right_info.header.frame_id = right_frame_id_;

        }

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Got camera calibration files");

        // loop to publish images;
        cv::Mat left_image, right_image;
        rclcpp::Rate r(frame_rate_);

        while (rclcpp::ok())
        {
          // ros::Time now = ros::Time::now();
          rclcpp::Time now = nh->now();

          if (!zed.getImages(left_image, right_image))
          {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"),"Can't find camera");
          }
          else
          {
            RCLCPP_INFO_ONCE(rclcpp::get_logger("rclcpp"),"Success, found camera");
          }
          if (show_image_)
          {
            cv::imshow("left", left_image);
            cv::imshow("right", right_image);
          }
          if (left_image_pub.getNumSubscribers() > 0)
          {
            publishImage(left_image, left_image_pub, "left_frame", now);
          }
          if (right_image_pub.getNumSubscribers() > 0)
          {
            publishImage(right_image, right_image_pub, "right_frame", now);
          }
          if (left_cam_info_pub->get_subscription_count() > 0)
          {
            publishCamInfo(left_cam_info_pub, left_info, now);
          }
          if (right_cam_info_pub->get_subscription_count() > 0)
          {
            publishCamInfo(right_cam_info_pub, right_info, now);
          }
          r.sleep();
          // since the frame rate was set inside the camera, no need to do a ros sleep
        }

      }
      
      /**
      * @brief      Correct frame rate according to resolution
      *
      * @param[in]  resolution          The resolution
      * @param      frame_rate   			 The camera frame rate
      */
      void correctFramerate(int resolution, double& frame_rate)
      {
        double max_frame_rate;
        std::string reso_str = "";
        switch (resolution)
        {
          case 0:
            max_frame_rate = 15;
            reso_str = "2K";
            break;
          case 1:
            max_frame_rate = 30;
            reso_str = "FHD";
            break;
          case 2:
            max_frame_rate = 60;
            reso_str = "HD";
            break;
          case 3:
            max_frame_rate = 100;
            reso_str = "VGA";
            break;
          default:
            RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),"Unknow resolution passed");
            return;
        }
        if (frame_rate > max_frame_rate)
          RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"frame_rate(%fHz) too high for resolution(%s), downgraded to %fHz", frame_rate, reso_str.c_str(),max_frame_rate);

        frame_rate = max_frame_rate;
      }

      /**
      * @brief      Gets the camera information From Zed config.
      *
      * @param[in]  config_file         The configuration file
      * @param[in]  resolution          The resolution
      * @param[in]  left_cam_info_msg   The left camera information message
      * @param[in]  right_cam_info_msg  The right camera information message
      */
      void getZedCameraInfo(std::string config_file, int resolution, sensor_msgs::msg::CameraInfo& left_info,sensor_msgs::msg::CameraInfo& right_info)
      {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(config_file, pt);
        std::string left_str = "LEFT_CAM_";
        std::string right_str = "RIGHT_CAM_";
        std::string reso_str = "";

        switch (resolution)
        {
          case 0:
            reso_str = "2K";
            break;
          case 1:
            reso_str = "FHD";
            break;
          case 2:
            reso_str = "HD";
            break;
          case 3:
            reso_str = "VGA";
            break;
        }
        // left value
        double l_cx = pt.get<double>(left_str + reso_str + ".cx");
        double l_cy = pt.get<double>(left_str + reso_str + ".cy");
        double l_fx = pt.get<double>(left_str + reso_str + ".fx");
        double l_fy = pt.get<double>(left_str + reso_str + ".fy");
        double l_k1 = pt.get<double>(left_str + reso_str + ".k1");
        double l_k2 = pt.get<double>(left_str + reso_str + ".k2");
        // right value
        double r_cx = pt.get<double>(right_str + reso_str + ".cx");
        double r_cy = pt.get<double>(right_str + reso_str + ".cy");
        double r_fx = pt.get<double>(right_str + reso_str + ".fx");
        double r_fy = pt.get<double>(right_str + reso_str + ".fy");
        double r_k1 = pt.get<double>(right_str + reso_str + ".k1");
        double r_k2 = pt.get<double>(right_str + reso_str + ".k2");

        // get baseline and convert mm to m
        boost::optional<double> baselineCheck;
        double baseline = 0.0;
        // some config files have "Baseline" instead of "BaseLine", check accordingly...
        if (baselineCheck = pt.get_optional<double>("STEREO.BaseLine"))
        {
          baseline = pt.get<double>("STEREO.BaseLine") * 0.001;
        }
        else if (baselineCheck = pt.get_optional<double>("STEREO.Baseline"))
        {
          baseline = pt.get<double>("STEREO.Baseline") * 0.001;
        }
        else
        {
          throw std::runtime_error("baseline parameter not found");
        }

        // get Rx and Rz
        double rx = pt.get<double>("STEREO.RX_" + reso_str);
        double rz = pt.get<double>("STEREO.RZ_" + reso_str);
        double ry = pt.get<double>("STEREO.CV_" + reso_str);

        // assume zeros, maybe not right
        double p1 = 0, p2 = 0, k3 = 0;

        left_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        right_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        // TODO(dizeng) verify loading default zed config is still working

        // distortion parameters
        // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
        left_info.d.resize(5);
        left_info.d[0] = l_k1;
        left_info.d[1] = l_k2;
        left_info.d[2] = k3;
        left_info.d[3] = p1;
        left_info.d[4] = p2;

        right_info.d.resize(5);
        right_info.d[0] = r_k1;
        right_info.d[1] = r_k2;
        right_info.d[2] = k3;
        right_info.d[3] = p1;
        right_info.d[4] = p2;

        // Intrinsic camera matrix
        // 	[fx  0 cx]
        // K =  [ 0 fy cy]
        //	[ 0  0  1]
        left_info.k.fill(0.0);
        left_info.k[0] = l_fx;
        left_info.k[2] = l_cx;
        left_info.k[4] = l_fy;
        left_info.k[5] = l_cy;
        left_info.k[8] = 1.0;

        right_info.k.fill(0.0);
        right_info.k[0] = r_fx;
        right_info.k[2] = r_cx;
        right_info.k[4] = r_fy;
        right_info.k[5] = r_cy;
        right_info.k[8] = 1.0;

        // rectification matrix
        // Rl = R_rect, R_r = R * R_rect
        // since R is identity, Rl = Rr;
        left_info.r.fill(0.0);
        right_info.r.fill(0.0);
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
        cv::Mat rmat(3, 3, CV_64F);
        cv::Rodrigues(rvec, rmat);
        int id = 0;
        cv::MatIterator_<double> it, end;
        for (it = rmat.begin<double>(); it != rmat.end<double>(); ++it, id++)
        {
          left_info.r[id] = *it;
          right_info.r[id] = *it;
        }

        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        left_info.p.fill(0.0);
        left_info.p[0] = l_fx;
        left_info.p[2] = l_cx;
        left_info.p[5] = l_fy;
        left_info.p[6] = l_cy;
        left_info.p[10] = 1.0;

        right_info.p.fill(0.0);
        right_info.p[0] = r_fx;
        right_info.p[2] = r_cx;
        right_info.p[3] = (-1 * l_fx * baseline);
        right_info.p[5] = r_fy;
        right_info.p[6] = r_cy;
        right_info.p[10] = 1.0;

        left_info.width = right_info.width = width_;
        left_info.height = right_info.height = height_;

        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;
      }
      
      /**
      * @brief      { publish image }
      *
      * @param[in]  img           The image
      * @param      img_pub       The image pub
      * @param[in]  img_frame_id  The image frame identifier
      * @param[in]  t             { parameter_description }
      * @param[in]  encoding      image_transport encoding
      */
      void publishImage(const cv::Mat& img, image_transport::Publisher& img_pub, const std::string& img_frame_id,rclcpp::Time t)
      {
        cv_bridge::CvImage cv_image;
        // TODO(dizeng) maybe we can save a copy here?
        // or it seems like CV mat is passing by reference?
        cv_image.image = img;
        // TODO(dizeng)
        // by default the cv::mat from zed is bgr8, here just chaing encoding seems
        // doesn't work, need to implement conversion function specificly
        cv_image.encoding = encoding_;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        img_pub.publish(cv_image.toImageMsg());
      }

      /**
      * @brief      { publish camera info }
      *
      * @param[in]  pub_cam_info  The pub camera information
      * @param[in]  cam_info_msg  The camera information message
      * @param[in]  now           The now
      */
      void publishCamInfo(const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_cam_info, sensor_msgs::msg::CameraInfo& cam_info_msg, rclcpp::Time now)
      {
        cam_info_msg.header.stamp = now;
        pub_cam_info->publish(cam_info_msg);
      }

  };

}

int main(int argc , char **argv)
{   
  try
  {
    rclcpp::init(argc,argv);
    arti::ZedCameraROS zed_ros2;

    return EXIT_SUCCESS;
  }
  catch(const std::runtime_error& e)
  {
    rclcpp::shutdown();

    return EXIT_FAILURE;
  }
  
}