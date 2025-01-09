#pragma once

#include <string>
#include <vector>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <openvino/openvino.hpp>

struct Config {
    float confThreshold;
    float nmsThreshold;
    float scoreThreshold;
    int inpWidth;
    int inpHeight;
    std::string onnx_path;
};

struct Resize {
    cv::Mat resized_image;
    int dw;
    int dh;
};

struct Detection {
    int class_id;
    float confidence;
    cv::Rect box;
    std::string object_name;
};

class YOLOV5 {
public:
    YOLOV5(const Config& config);
    ~YOLOV5();
    void detect(cv::Mat& frame);
    std::vector<Detection> get_detections() const;

private:
    float confThreshold_;
    float nmsThreshold_;
    float scoreThreshold_;
    int inpWidth_;
    int inpHeight_;
    float rx_;   // Width ratio of original image and resized image
    float ry_;   // Height ratio of original image and resized image
    std::string onnx_path_;
    Resize resize_;
    ov::Tensor input_tensor_;
    ov::InferRequest infer_request_;
    ov::CompiledModel compiled_model_;
    std::vector<Detection> detections_;

    void initialize_model();
    void preprocess_img(cv::Mat& frame);
    void postprocess_img(cv::Mat& frame, float* detections, const ov::Shape& output_shape);
};
