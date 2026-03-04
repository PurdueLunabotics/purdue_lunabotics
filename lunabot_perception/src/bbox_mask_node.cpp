#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>

class BBoxMaskNode : public rclcpp::Node
{
public:
    BBoxMaskNode()
        : Node("bbox_mask_node"),
        tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        image_sub_ = image_transport::create_subscription(
            this,
            "image_raw",
            std::bind(&BBoxMaskNode::imageCallback, this, std::placeholders::_1),
            "raw");
        
        rgb_image_sub_ = image_transport::create_subscription(
            this,
            "image_raw",
            std::bind(&BBoxMaskNode::rgbImageCallback, this, std::placeholders::_1),
            "raw");

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            10,
            std::bind(&BBoxMaskNode::cameraInfoCallback, this, std::placeholders::_1));

        image_pub_ = image_transport::create_publisher(
            this,
            "image_masked");

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/bbox_marker", 10);

        map_frame_ = "map";
        camera_frame_ = "d455_front_depth_link";
        
            RCLCPP_INFO(this->get_logger(), "Bounding Box Mask Node Started");

    }

private:

    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::string map_frame_;
    std::string camera_frame_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    image_geometry::PinholeCameraModel camera_model_;
    bool camera_info_received_ = false;

    // Example 3D bounding box in camera frame (meters)
    double xmin_ = -3.5;
    double xmax_ =  3.5;
    double ymin_ = -2.5;
    double ymax_ =  2.5;
    double zmin_ =  -0.5;
    double zmax_ =  1.5;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_model_.fromCameraInfo(*msg);
        camera_info_received_ = true;
    }

    void publishBoxMarker(const builtin_interfaces::msg::Time & stamp)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";
        marker.header.stamp = stamp;

        marker.ns = "bbox";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.02;  // line width

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        // 8 corners in map frame
        std::vector<geometry_msgs::msg::Point> corners(8);

        corners[0].x = xmin_; corners[0].y = ymin_; corners[0].z = zmin_;
        corners[1].x = xmax_; corners[1].y = ymin_; corners[1].z = zmin_;
        corners[2].x = xmax_; corners[2].y = ymax_; corners[2].z = zmin_;
        corners[3].x = xmin_; corners[3].y = ymax_; corners[3].z = zmin_;

        corners[4].x = xmin_; corners[4].y = ymin_; corners[4].z = zmax_;
        corners[5].x = xmax_; corners[5].y = ymin_; corners[5].z = zmax_;
        corners[6].x = xmax_; corners[6].y = ymax_; corners[6].z = zmax_;
        corners[7].x = xmin_; corners[7].y = ymax_; corners[7].z = zmax_;

        auto add_line = [&](int a, int b)
        {
            marker.points.push_back(corners[a]);
            marker.points.push_back(corners[b]);
        };

        // Bottom square
        add_line(0,1);
        add_line(1,2);
        add_line(2,3);
        add_line(3,0);

        // Top square
        add_line(4,5);
        add_line(5,6);
        add_line(6,7);
        add_line(7,4);

        // Vertical edges
        add_line(0,4);
        add_line(1,5);
        add_line(2,6);
        add_line(3,7);

        marker_pub_->publish(marker);
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (!camera_info_received_)
            return;

        if (msg->encoding != "32FC1")
        {
            RCLCPP_ERROR(this->get_logger(),
                "Depth image must be 32FC1 (float meters)");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_ptr->image;

        geometry_msgs::msg::TransformStamped cam_to_map_tf;

        try
        {
            cam_to_map_tf = tf_buffer_.lookupTransform(
                map_frame_,
                camera_frame_,
                msg->header.stamp,
                tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
            return;
        }

        for (int v = 0; v < depth_image.rows; ++v)
        {
            for (int u = 0; u < depth_image.cols; ++u)
            {
                float depth = depth_image.at<float>(v, u);

                if (!std::isfinite(depth) || depth <= 0.0)
                    continue;

                // Back-project pixel → 3D camera coords
                cv::Point2d uv(u, v);
                cv::Point3d ray = camera_model_.projectPixelTo3dRay(uv);

                cv::Point3d pt_cam = ray * depth;

                geometry_msgs::msg::PointStamped pt_camera;
                pt_camera.header.frame_id = camera_frame_;
                pt_camera.header.stamp = msg->header.stamp;
                pt_camera.point.x = pt_cam.x;
                pt_camera.point.y = pt_cam.y;
                pt_camera.point.z = pt_cam.z;

                geometry_msgs::msg::PointStamped pt_map;

                try
                {
                    tf2::doTransform(pt_camera, pt_map, cam_to_map_tf);
                }
                catch (tf2::TransformException &ex)
                {
                    continue;
                }

                // Check if inside 3D box (defined in MAP frame)
                bool inside =
                    pt_map.point.x >= xmin_ && pt_map.point.x <= xmax_ &&
                    pt_map.point.y >= ymin_ && pt_map.point.y <= ymax_ &&
                    pt_map.point.z >= zmin_ && pt_map.point.z <= zmax_;
                if (!inside)
                {
                    depth_image.at<float>(v, u) =
                        std::numeric_limits<float>::infinity();
                }
            }
        }

        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = "32FC1";
        out_msg.image = depth_image;

        image_pub_.publish(out_msg.toImageMsg());
        publishBoxMarker(msg->header.stamp);
    }

    void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBoxMaskNode>());
    rclcpp::shutdown();
    return 0;
}