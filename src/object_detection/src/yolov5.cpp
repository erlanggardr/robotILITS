#include "object_detection/yolov5.h"
#include "object_detection/constants.h"
#include <iostream>
#include <opencv2/imgcodecs.hpp>

YOLOV5::YOLOV5(const Config& config)
    : confThreshold_(config.confThreshold),
      nmsThreshold_(config.nmsThreshold),
      scoreThreshold_(config.scoreThreshold),
      inpWidth_(config.inpWidth),
      inpHeight_(config.inpHeight),
      rx_(1.0f),
      ry_(1.0f),
      onnx_path_(config.onnx_path) {
    initialize_model();
}

YOLOV5::~YOLOV5() {}

void YOLOV5::initialize_model() {
    try {
        ov::Core core;
        std::shared_ptr<ov::Model> model = core.read_model(this->onnx_path_);

        // Preprocessing pipeline
        ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
        ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::RGB);
        ppp.input().preprocess().convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255.0f, 255.0f, 255.0f});
        ppp.input().model().set_layout("NCHW");
        ppp.output().tensor().set_element_type(ov::element::f32);
        model = ppp.build();

        // Compile model untuk CPU
        this->compiled_model_ = core.compile_model(model, "CPU");
        this->infer_request_ = compiled_model_.create_infer_request();
    }
    catch (const std::exception& e) {
        std::cerr << "Error initializing model: " << e.what() << std::endl;
        throw;
    }
}

void YOLOV5::preprocess_img(cv::Mat& frame) {
    float width = static_cast<float>(frame.cols);
    float height = static_cast<float>(frame.rows);
    cv::Size new_shape = cv::Size(inpWidth_, inpHeight_);
    float r = static_cast<float>(new_shape.width) / std::max(width, height);
    int new_unpadW = static_cast<int>(round(width * r));
    int new_unpadH = static_cast<int>(round(height * r));

    cv::resize(frame, resize_.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);
    resize_.dw = new_shape.width - new_unpadW;
    resize_.dh = new_shape.height - new_unpadH;

    // padding jaga rasio
    cv::Scalar color = cv::Scalar(100, 100, 100);
    cv::copyMakeBorder(resize_.resized_image, resize_.resized_image, 0, resize_.dh, 0, resize_.dw, cv::BORDER_CONSTANT, color);

    // Hitung rasio untuk mengembalikan koordinat asli
    this->rx_ = static_cast<float>(frame.cols) / static_cast<float>(resize_.resized_image.cols - resize_.dw);
    this->ry_ = static_cast<float>(frame.rows) / static_cast<float>(resize_.resized_image.rows - resize_.dh);

    // Persiapkan tensor input
    float* input_data = reinterpret_cast<float*>(resize_.resized_image.data);
    input_tensor_ = ov::Tensor(compiled_model_.input().get_element_type(), compiled_model_.input().get_shape(), input_data);
    infer_request_.set_input_tensor(input_tensor_);
}

void YOLOV5::detect(cv::Mat& frame) {
    try {
        preprocess_img(frame);
        infer_request_.infer();
        const ov::Tensor& output_tensor = infer_request_.get_output_tensor();
        ov::Shape output_shape = output_tensor.get_shape();
        float* detections = output_tensor.data<float>();
        postprocess_img(frame, detections, output_shape);
    }
    catch (const std::exception& e) {
        std::cerr << "Detection error: " << e.what() << std::endl;
    }
}

void YOLOV5::postprocess_img(cv::Mat& frame, float* detections, const ov::Shape& output_shape) {
    std::vector<cv::Rect> boxes;
    std::vector<int> class_ids;
    std::vector<float> confidences;

    // Parsing hasil deteksi
    for (size_t i = 0; i < static_cast<size_t>(output_shape[1]); i++) {
        float* detection = &detections[i * output_shape[2]];

        float confidence = detection[4];
        if (confidence >= this->confThreshold_) {
            float* classes_scores = &detection[5];
            cv::Mat scores(1, output_shape[2] - 5, CV_32FC1, classes_scores);
            cv::Point class_id_point;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
            if (max_class_score > this->scoreThreshold_) {
                confidences.push_back(confidence);
                class_ids.push_back(class_id_point.x);
                float x = detection[0];
                float y = detection[1];
                float w = detection[2];
                float h = detection[3];
                float xmin = x - (w / 2);
                float ymin = y - (h / 2);

                boxes.emplace_back(cv::Rect(static_cast<int>(xmin), static_cast<int>(ymin), static_cast<int>(w), static_cast<int>(h)));
            }
        }
    }

    // Non-Maximum Suppression untuk mengurangi overlapping bounding box
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold_, this->nmsThreshold_, nms_result);

    // Menyimpan hasil deteksi
    detections_.clear();
    for (const auto& idx : nms_result) {
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        detections_.push_back(result);
    }

    // Menggambar bounding box pada frame
    for (const auto& detection : detections_) {
        if (detection.class_id < 0 || detection.class_id >= static_cast<int>(coconame_size)) {
            continue; 
        }

        // Mengembalikan koordinat ke ukuran asli
        cv::Rect box;
        box.x = static_cast<int>(detection.box.x * rx_);
        box.y = static_cast<int>(detection.box.y * ry_);
        box.width = static_cast<int>(detection.box.width * rx_);
        box.height = static_cast<int>(detection.box.height * ry_);

        // Menggambar box
        cv::Scalar color = cv::Scalar(color_list[detection.class_id][0] * 255,
                                     color_list[detection.class_id][1] * 255,
                                     color_list[detection.class_id][2] * 255);
        cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), color, 2);

        // teks label
        char text[256];
        snprintf(text, sizeof(text), "%s %.1f%%", coconame[detection.class_id], detection.confidence * 100);

        // Menghitung ukuran teks
        int baseline = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
        cv::rectangle(frame, cv::Point(box.x, box.y),
                      cv::Point(box.x + label_size.width, box.y + label_size.height + baseline),
                      cv::Scalar(color_list[detection.class_id][0] * 255 * 0.7,
                                 color_list[detection.class_id][1] * 255 * 0.7,
                                 color_list[detection.class_id][2] * 255 * 0.7),
                      cv::FILLED);

        // menggambar teks
        cv::putText(frame, text, cv::Point(box.x, box.y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    (cv::mean(color)[0] > 128) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255),
                    1);
    }
}

std::vector<Detection> YOLOV5::get_detections() const {
    return detections_;
}
