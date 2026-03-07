#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rtabmap_msgs/msg/detail/rgbd_image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>

using rtabmap_msgs::msg::RGBDImage;

class BBoxMaskNode : public rclcpp::Node
{
public:
    BBoxMaskNode()
        : Node("bbox_mask_node"),
        tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        image_sub_ = create_subscription<RGBDImage>("image_raw", 10, [this] (RGBDImage image) {
            this->imageCallback(image);
        });

        image_pub_ = create_publisher<RGBDImage>("image_masked", 10);
        depth_pub_ = image_transport::create_publisher(this, "image_masked_depth");
        rgb_pub_ = image_transport::create_publisher(this, "image_masked_rgb");

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "bbox_marker", 10);

        declare_parameter("map_frame", "map");
        get_parameter("map_frame", map_frame_);
        
        RCLCPP_INFO(this->get_logger(), "Bounding Box Mask Node Started");

    }

private:

    rclcpp::Subscription<RGBDImage>::SharedPtr image_sub_;
    rclcpp::Publisher<RGBDImage>::SharedPtr image_pub_;
    image_transport::Publisher depth_pub_;
    image_transport::Publisher rgb_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::string map_frame_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    image_geometry::PinholeCameraModel camera_model_;
    bool camera_info_received_ = false;

    // Example 3D bounding box in camera frame (meters)
    double xmin_ = -3.3;
    double xmax_ =  3.3;
    double ymin_ = -2.3;
    double ymax_ =  2.3;
    double zmin_ = -0.3;
    double zmax_ =  1.3;

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

    void imageCallback(RGBDImage msg)
    {
        if (!camera_info_received_) {
            camera_model_.fromCameraInfo(msg.depth_camera_info);
            camera_info_received_ = true;
        }
        if (image_pub_->get_subscription_count() == 0)
            return;

        if (msg.depth.encoding != "32FC1")
        {
            RCLCPP_ERROR(this->get_logger(),
                "Depth image must be 32FC1 (float meters)");
            return;
        }

        cv_bridge::CvImagePtr depth_ptr;
        cv_bridge::CvImagePtr rgb_ptr;

        try
        {
            depth_ptr = cv_bridge::toCvCopy(msg.depth, "");
            rgb_ptr = cv_bridge::toCvCopy(msg.rgb, "rgb8");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = depth_ptr->image;
        cv::Mat rgb_image = rgb_ptr->image;

        geometry_msgs::msg::TransformStamped cam_to_map_tf;

        try
        {
            cam_to_map_tf = tf_buffer_.lookupTransform(
                map_frame_,
                msg.header.frame_id,
                rclcpp::Time(),
                tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            depth_pub_.publish(msg.depth);
            rgb_pub_.publish(msg.rgb);
            image_pub_->publish(msg);
            RCLCPP_WARN(this->get_logger(), "TF failed (still published): %s", ex.what());
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
                pt_camera.header.frame_id = msg.header.frame_id;
                pt_camera.header.stamp = msg.header.stamp;
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
                    rgb_image.at<cv::Vec3b>(v, u) = { 0, 0, 0 };
                }
            }
        }

        cv_bridge::CvImage out_depth;
        out_depth.header = msg.header;
        out_depth.encoding = "32FC1";
        out_depth.image = depth_image;

        cv_bridge::CvImage out_rgb;
        out_rgb.header = msg.header;
        out_rgb.encoding = "rgb8";
        out_rgb.image = rgb_image;

        sensor_msgs::msg::Image::SharedPtr depth_msg = out_depth.toImageMsg();
        sensor_msgs::msg::Image::SharedPtr rgb_msg = out_rgb.toImageMsg();

        depth_pub_.publish(depth_msg);
        rgb_pub_.publish(rgb_msg);

        RGBDImage out_image;
        out_image.header = msg.header;
        out_image.depth = *depth_msg;
        out_image.rgb = *rgb_msg;
        out_image.rgb_camera_info = msg.rgb_camera_info;
        out_image.depth_camera_info = msg.depth_camera_info;

        image_pub_->publish(out_image);
        publishBoxMarker(msg.header.stamp);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBoxMaskNode>());
    rclcpp::shutdown();
    return 0;
}
