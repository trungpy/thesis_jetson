#include "Detect.h"

#include <NvOnnxParser.h>

#include <fstream>
#include <iostream>

#include "config.h"
#include "cuda_utils.h"
#include "logging.h"
#include "macros.h"
#include "preprocess.h"
static Logger logger;
#define isFP16 true
#define warmup true

Detect::Detect(string model_path, nvinfer1::ILogger &logger) {
    // Deserialize an engine
    if (model_path.find(".onnx") == std::string::npos) {
        init(model_path, logger);
    }
    // Build an engine from an ONNX model
    else {
        build(model_path, logger);
        saveEngine(model_path);
    }

    std::cout << "Input dimensions: " << inputHeight << "x" << inputWeight << std::endl;
    std::cout << "TensorRT version: " << NV_TENSORRT_MAJOR << "." << NV_TENSORRT_MINOR << std::endl;
}

void Detect::init(std::string engine_path, nvinfer1::ILogger &logger) {
    // Read the engine file
    ifstream engineStream(engine_path, ios::binary);
    engineStream.seekg(0, ios::end);
    const size_t modelSize = engineStream.tellg();
    engineStream.seekg(0, ios::beg);
    unique_ptr<char[]> engineData(new char[modelSize]);
    engineStream.read(engineData.get(), modelSize);
    engineStream.close();

    // Deserialize the TensorRT engine
    runtime = createInferRuntime(logger);
    engine = runtime->deserializeCudaEngine(engineData.get(), modelSize);
    context = engine->createExecutionContext();

#if NV_TENSORRT_MAJOR < 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR < 5)
    inputHeight = engine->getBindingDimensions(0).d[2];
    inputWeight = engine->getBindingDimensions(0).d[3];
    detection_attribute_size = engine->getBindingDimensions(1).d[1];
    num_detections = engine->getBindingDimensions(1).d[2];
#else
    auto input_name = engine->getIOTensorName(0);
    auto output_name = engine->getIOTensorName(1);

    auto input_dims = engine->getTensorShape(input_name);
    auto output_dims = engine->getTensorShape(output_name);

    inputHeight = input_dims.d[2];
    inputWeight = input_dims.d[3];
    detection_attribute_size = output_dims.d[1];
    num_detections = output_dims.d[2];
#endif
    num_classes = detection_attribute_size - 4;

    // Initialize input buffers
    cpu_output_buffer = new float[detection_attribute_size * num_detections];
    CUDA_CHECK(cudaMalloc(&gpu_buffers[0], 3 * inputWeight * inputHeight * sizeof(float)));
    CUDA_CHECK(
        cudaMalloc(&gpu_buffers[1], detection_attribute_size * num_detections * sizeof(float)));

    cuda_preprocess_init(MAX_IMAGE_SIZE);
    CUDA_CHECK(cudaStreamCreate(&stream));

    if (warmup) {
        for (int i = 0; i < 10; i++) {
            this->infer();
        }
        printf("model warmup 10 times\n");
    }
}

Detect::~Detect() {
    // Release stream and buffers
    CUDA_CHECK(cudaStreamSynchronize(stream));
    CUDA_CHECK(cudaStreamDestroy(stream));
    for (int i = 0; i < 2; i++) CUDA_CHECK(cudaFree(gpu_buffers[i]));
    delete[] cpu_output_buffer;

    // Destroy the engine
    cuda_preprocess_destroy();
#if NV_TENSORRT_MAJOR < 8
    context->destroy();
    engine->destroy();
    runtime->destroy();
#else
    delete context;
    delete engine;
    delete runtime;
#endif
}

void Detect::preprocess(Mat &image) {
    // Preprocessing data on gpu
    cuda_preprocess(image.ptr(), image.cols, image.rows, gpu_buffers[0], inputWeight, inputHeight,
                    stream);
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void Detect::infer() {
    // Register the input and output buffers
#if NV_TENSORRT_MAJOR < 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR < 5)
    // For TensorRT < 8.5, do not use setTensorAddress, just use enqueueV2
#else
    const char *input_name = engine->getIOTensorName(0);
    const char *output_name = engine->getIOTensorName(1);

    // Set the input tensor address
    context->setTensorAddress(input_name, gpu_buffers[0]);
    context->setTensorAddress(output_name, gpu_buffers[1]);
#endif

#if NV_TENSORRT_MAJOR < 10
    context->enqueueV2((void **)gpu_buffers, stream, nullptr);
#else
    this->context->enqueueV3(this->stream);
#endif
}

void Detect::postprocess(Mat &image, vector<Detection> &output) {
    // Memcpy from device output buffer to host output buffer
    CUDA_CHECK(cudaMemcpyAsync(cpu_output_buffer, gpu_buffers[1],
                               num_detections * detection_attribute_size * sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));

    vector<Rect> boxes;
    vector<int> classIds;
    vector<float> confidences;

    const Mat det_output(detection_attribute_size, num_detections, CV_32F, cpu_output_buffer);

    for (int i = 0; i < det_output.cols; ++i) {
        const Mat classes_scores = det_output.col(i).rowRange(4, 4 + num_classes);
        Point class_id_point;
        double score;
        minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);

        if (score > conf_threshold) {
            const float cx = det_output.at<float>(0, i);
            const float cy = det_output.at<float>(1, i);
            const float ow = det_output.at<float>(2, i);
            const float oh = det_output.at<float>(3, i);
            Rect box;
            box.x = static_cast<int>((cx - 0.5 * ow));
            box.y = static_cast<int>((cy - 0.5 * oh));
            box.width = static_cast<int>(ow);
            box.height = static_cast<int>(oh);

            boxes.push_back(box);
            classIds.push_back(class_id_point.y);
            confidences.push_back(score);
        }
    }

    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, nms_result);

    for (int i = 0; i < nms_result.size(); i++) {
        Detection result;
        int idx = nms_result[i];
        result.classId = classIds[idx];
        result.conf = confidences[idx];
        result.bbox =
            scaleBoxToOriginal(boxes[idx], cv::Size(inputWeight, inputHeight), image.size());
        output.push_back(result);
    }
}

void Detect::build(std::string onnxPath, nvinfer1::ILogger &logger) {
    auto builder = createInferBuilder(logger);
    const auto explicitBatch =
        1U << 0;  // kEXPLICIT_BATCH is 0, use 0 directly to avoid deprecation warning
    INetworkDefinition *network = builder->createNetworkV2(explicitBatch);
    IBuilderConfig *config = builder->createBuilderConfig();

    if (isFP16) {
        config->setFlag(BuilderFlag::kFP16);
    }

    nvonnxparser::IParser *parser = nvonnxparser::createParser(*network, logger);
    bool parsed = parser->parseFromFile(onnxPath.c_str(),
                                        static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
#if NV_TENSORRT_MAJOR < 8
    ICudaEngine *tmp_engine = builder->buildCudaEngine(*network);
    IHostMemory *plan = nullptr;
    if (tmp_engine) {
        plan = tmp_engine->serialize();
        tmp_engine->destroy();
    }
#else
    IHostMemory *plan{builder->buildSerializedNetwork(*network, *config)};
#endif

    runtime = createInferRuntime(logger);

    engine = runtime->deserializeCudaEngine(plan->data(), plan->size());

    context = engine->createExecutionContext();

#if NV_TENSORRT_MAJOR < 8
    network->destroy();
    config->destroy();
    parser->destroy();
    plan->destroy();
#else
    delete network;
    delete config;
    delete parser;
    delete plan;
#endif
}

bool Detect::saveEngine(const std::string &onnxPath) {
    // Create an engine path from onnx path
    std::string engine_path;
    size_t dotIndex = onnxPath.find_last_of(".");
    if (dotIndex != std::string::npos) {
        engine_path = onnxPath.substr(0, dotIndex) + ".engine";
    } else {
        return false;
    }

    // Save the engine to the path
    if (engine) {
        nvinfer1::IHostMemory *data = engine->serialize();
        std::ofstream file;
        file.open(engine_path, std::ios::binary | std::ios::out);
        if (!file.is_open()) {
            std::cout << "Create engine file" << engine_path << " failed" << std::endl;
            return 0;
        }
        file.write((const char *)data->data(), data->size());
        file.close();

#if NV_TENSORRT_MAJOR < 8
        data->destroy();
#else
        delete data;
#endif
    }
    return true;
}

void Detect::draw(cv::Mat &image, const std::vector<STrack> &output) {
    const float ratio_h = static_cast<float>(inputHeight) / image.rows;
    const float ratio_w = static_cast<float>(inputWeight) / image.cols;

    for (const auto &detection : output) {
        const auto &tlwh = detection.tlwh;
        cv::Rect box(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);

        int classId = detection.classId;
        float conf = detection.score;

        cv::Scalar color(Config::colors[classId][0], Config::colors[classId][1],
                         Config::colors[classId][2]);

        // Draw rectangle
        cv::rectangle(image, box, color, 2);

        // Estimate and draw distance if class is relevant
        if (classId == 2 || classId == 4 || classId == 5) {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << detection.estimatedDistance << "m";

            // Smaller font for distance text
            cv::putText(image, ss.str(), cv::Point(box.x + 2, box.y - 20), cv::FONT_HERSHEY_SIMPLEX,
                        0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }

        // Prepare and draw label with smaller font
        std::ostringstream label_ss;
        label_ss << detection.track_id << ". " << Config::classNames[classId] << " " << std::fixed
                 << std::setprecision(2) << conf;

        std::string label = label_ss.str();

        // Smaller font for main label
        cv::putText(image, label, cv::Point(box.x + 2, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
}

cv::Rect Detect::scaleBoxToOriginal(const cv::Rect &input_box, const cv::Size &model_input_size,
                                    const cv::Size &original_size) {
    float ratio_h = static_cast<float>(model_input_size.height) / original_size.height;
    float ratio_w = static_cast<float>(model_input_size.width) / original_size.width;

    cv::Rect box = input_box;

    if (ratio_h > ratio_w) {
        float pad = (model_input_size.height - ratio_w * original_size.height) / 2;
        box.x = box.x / ratio_w;
        box.y = (box.y - pad) / ratio_w;
        box.width = static_cast<int>(box.width / ratio_w);
        box.height = static_cast<int>(box.height / ratio_w);
    } else {
        float pad = (model_input_size.width - ratio_h * original_size.width) / 2;
        box.x = (box.x - pad) / ratio_h;
        box.y = box.y / ratio_h;
        box.width = static_cast<int>(box.width / ratio_h);
        box.height = static_cast<int>(box.height / ratio_h);
    }

    // Clamp to valid image region
    box.x = std::max(0, box.x);
    box.y = std::max(0, box.y);
    box.width = std::min(box.width, original_size.width - box.x);
    box.height = std::min(box.height, original_size.height - box.y);

    return box;
}
