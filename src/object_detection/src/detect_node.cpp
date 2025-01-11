#include "object_detection/constants.h"
#include "object_detection/yolov5.h"
#include <cv_bridge/cv_bridge.h>
#include <interfaces/msg/object_command.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

class ObjectDetectionNode : public rclcpp::Node {
 public:
   ObjectDetectionNode() : Node("object_detection_node"), cap_(0) {
      if (!cap_.isOpened()) {
         RCLCPP_ERROR(this->get_logger(), "Error opening video stream");
         return;
      }

      image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/objects_box", 10);
      objects_publisher_ = this->create_publisher<interfaces::msg::ObjectCommand>("/objects", 10);

      Config config = {0.4f, 0.4f, 0.4f, 640, 640, "src/object_detection/models/yolov5m.onnx"};
      yolomodel_ = std::make_shared<YOLOV5>(config);

      timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&ObjectDetectionNode::timer_callback, this));

      RCLCPP_INFO(this->get_logger(), "Object Detection Node Initialized");
   }

 private:
   void timer_callback() {
      cv::Mat frame;
      cap_.read(frame);

      if (frame.empty()) {
         RCLCPP_ERROR(this->get_logger(), "Captured empty frame");
         return;
      }

      yolomodel_->detect(frame);

      // publish with bounding box
      sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      image_publisher_->publish(*out_msg);

      // publish detected objects
      std::vector<Detection> detections = yolomodel_->get_detections();
      for (const auto &det : detections) {
         interfaces::msg::ObjectCommand obj_msg;
         obj_msg.class_id = det.class_id;
         obj_msg.object_name = coconame[det.class_id];
         obj_msg.confidence = det.confidence;
         obj_msg.x = det.box.x;
         obj_msg.y = det.box.y;
         obj_msg.width = det.box.width;
         obj_msg.height = det.box.height;
         objects_publisher_->publish(obj_msg);
      }
   }

   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
   rclcpp::Publisher<interfaces::msg::ObjectCommand>::SharedPtr objects_publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
   std::shared_ptr<YOLOV5> yolomodel_;
   cv::VideoCapture cap_;
};

int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<ObjectDetectionNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}
