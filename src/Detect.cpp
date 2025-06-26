#include "Detect.h"
#include "tensorrt/logging.h"
#include "common/cuda_utils.h"
#include "common/macros.h"
#include "preprocess.h"
#include <NvOnnxParser.h>
#include "common/common.h"
#include <fstream>
#include <iostream>
#include <followdist/FrontDistanceEstimator.h>
static Logger logger;
#define isFP16 true
#define warmup true

Detect::Detect(string model_path, nvinfer1::ILogger &logger)
{
    // Deserialize an engine
    if (model_path.find(".onnx") == std::string::npos)
    {
        init(model_path, logger);
    }
    // Build an engine from an ONNX model
    else
    {
        build(model_path, logger);
        saveEngine(model_path);
    }

#if NV_TENSORRT_MAJOR < 8 || (NV_TENSORRT_MAJOR == 8 && NV_TENSORRT_MINOR < 5)
    // // For TensorRT < 8.5, use getBindingDimensions
    auto input_dims = engine->getBindingDimensions(0);
    input_h = input_dims.d[2];
    input_w = input_dims.d[3];
    cout << "Input dimensions: " << input_h << "x" << input_w << std::endl;
    std::cout << "TensorRT version is lower than 8.5, using getBindingDimensions." << std::endl;
#else
    // For TensorRT >= 8.5, use getIOTensorName and getTensorShape
    auto input_dims = engine->getTensorShape(engine->getIOTensorName(0));
    input_h = input_dims.d[2];
    input_w = input_dims.d[3];
    cout << "Input dimensions: " << input_h << "x" << input_w << std::endl;
    std::cout << "TensorRT version is 8.5 or higher, using getIOTensorName and getTensorShape." << std::endl;
#endif
}

void Detect::init(std::string engine_path, nvinfer1::ILogger &logger)
{
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
    input_h = engine->getBindingDimensions(0).d[2];
    input_w = engine->getBindingDimensions(0).d[3];
    detection_attribute_size = engine->getBindingDimensions(1).d[1];
    num_detections = engine->getBindingDimensions(1).d[2];
#else
    auto input_name = engine->getIOTensorName(0);
    auto output_name = engine->getIOTensorName(1);

    auto input_dims = engine->getTensorShape(input_name);
    auto output_dims = engine->getTensorShape(output_name);

    input_h = input_dims.d[2];
    input_w = input_dims.d[3];
    detection_attribute_size = output_dims.d[1];
    num_detections = output_dims.d[2];
#endif
    num_classes = detection_attribute_size - 4;

    // Initialize input buffers
    cpu_output_buffer = new float[detection_attribute_size * num_detections];
    CUDA_CHECK(cudaMalloc(&gpu_buffers[0], 3 * input_w * input_h * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&gpu_buffers[1], detection_attribute_size * num_detections * sizeof(float)));

    cuda_preprocess_init(MAX_IMAGE_SIZE);
    CUDA_CHECK(cudaStreamCreate(&stream));

    if (warmup)
    {
        for (int i = 0; i < 10; i++)
        {
            this->infer();
        }
        printf("model warmup 10 times\n");
    }
}

Detect::~Detect()
{
    // Release stream and buffers
    CUDA_CHECK(cudaStreamSynchronize(stream));
    CUDA_CHECK(cudaStreamDestroy(stream));
    for (int i = 0; i < 2; i++)
        CUDA_CHECK(cudaFree(gpu_buffers[i]));
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

void Detect::preprocess(Mat &image)
{
    // Preprocessing data on gpu
    cuda_preprocess(image.ptr(), image.cols, image.rows, gpu_buffers[0], input_w, input_h, stream);
    CUDA_CHECK(cudaStreamSynchronize(stream));
}

void Detect::infer()
{
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

void Detect::postprocess(vector<Detection> &output)
{
    // Memcpy from device output buffer to host output buffer
    CUDA_CHECK(cudaMemcpyAsync(cpu_output_buffer, gpu_buffers[1], num_detections * detection_attribute_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));

    vector<Rect> boxes;
    vector<int> class_ids;
    vector<float> confidences;

    const Mat det_output(detection_attribute_size, num_detections, CV_32F, cpu_output_buffer);

    for (int i = 0; i < det_output.cols; ++i)
    {
        const Mat classes_scores = det_output.col(i).rowRange(4, 4 + num_classes);
        Point class_id_point;
        double score;
        minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);

        if (score > conf_threshold)
        {
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
            class_ids.push_back(class_id_point.y);
            confidences.push_back(score);
        }
    }

    vector<int> nms_result;
    dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, nms_result);

    for (int i = 0; i < nms_result.size(); i++)
    {
        Detection result;
        int idx = nms_result[i];
        result.class_id = class_ids[idx];
        result.conf = confidences[idx];
        result.bbox = boxes[idx];
        output.push_back(result);
    }
}

void Detect::build(std::string onnxPath, nvinfer1::ILogger &logger)
{
    auto builder = createInferBuilder(logger);
    const auto explicitBatch = 1U << 0; // kEXPLICIT_BATCH is 0, use 0 directly to avoid deprecation warning
    INetworkDefinition *network = builder->createNetworkV2(explicitBatch);
    IBuilderConfig *config = builder->createBuilderConfig();

    if (isFP16)
    {
        config->setFlag(BuilderFlag::kFP16);
    }

    nvonnxparser::IParser *parser = nvonnxparser::createParser(*network, logger);
    bool parsed = parser->parseFromFile(onnxPath.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kINFO));
#if NV_TENSORRT_MAJOR < 8
    ICudaEngine *tmp_engine = builder->buildCudaEngine(*network);
    IHostMemory *plan = nullptr;
    if (tmp_engine)
    {
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

bool Detect::saveEngine(const std::string &onnxpath)
{
    // Create an engine path from onnx path
    std::string engine_path;
    size_t dotIndex = onnxpath.find_last_of(".");
    if (dotIndex != std::string::npos)
    {
        engine_path = onnxpath.substr(0, dotIndex) + ".engine";
    }
    else
    {
        return false;
    }

    // Save the engine to the path
    if (engine)
    {
        nvinfer1::IHostMemory *data = engine->serialize();
        std::ofstream file;
        file.open(engine_path, std::ios::binary | std::ios::out);
        if (!file.is_open())
        {
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

void Detect::draw(Mat &image, const vector<Detection> &output)
{
    const float ratio_h = input_h / (float)image.rows;
    const float ratio_w = input_w / (float)image.cols;
    for (const auto &detection : output)
    {
        auto box = detection.bbox;
        auto class_id = detection.class_id;
        auto conf = detection.conf;
        cv::Scalar color = cv::Scalar(COLORS[class_id][0], COLORS[class_id][1], COLORS[class_id][2]);

        // Adjust box back to original image size
        if (ratio_h > ratio_w)
        {
            box.x = box.x / ratio_w;
            box.y = (box.y - (input_h - ratio_w * image.rows) / 2) / ratio_w;
            box.width = static_cast<int>(box.width / ratio_w);
            box.height = static_cast<int>(box.height / ratio_w);
        }
        else
        {
            box.x = (box.x - (input_w - ratio_h * image.cols) / 2) / ratio_h;
            box.y = box.y / ratio_h;
            box.width = static_cast<int>(box.width / ratio_h);
            box.height = static_cast<int>(box.height / ratio_h);
        }

        // Clamp box coordinates to image size
        box.x = std::max(0.0f, static_cast<float>(box.x));
        box.y = std::max(0.0f, static_cast<float>(box.y));
        box.width = std::min(static_cast<float>(box.width), static_cast<float>(image.cols - box.x));
        box.height = std::min(static_cast<float>(box.height), static_cast<float>(image.rows - box.y));

        // Draw rectangle
        rectangle(image, cv::Rect(box.x, box.y, box.width, box.height), color, 2);

        // // Draw center point
        // cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
        // cv::circle(image, center, 3, color, -1);
        // Draw distance estimation
        if (class_id == 2 || class_id == 4 || class_id == 5)
        {
            FrontDistanceEstimator distance_estimator;
            double focal_length = 1500.0;   // Example focal length, adjust as needed
            double real_object_width = 1.5; // Default width

            if (class_id == 2)
            {
                real_object_width = 1.6;
            }
            else if (class_id == 4 || class_id == 5)
            {
                real_object_width = 2.5;
            }

            double pixel_distance = box.width;
            double distance = distance_estimator.estimate(pixel_distance, focal_length, real_object_width);

            std::ostringstream distance_ss;
            distance_ss << std::fixed << std::setprecision(2) << distance << " m";
            std::string distance_text = distance_ss.str();

            // Draw distance text (white text with black outline)
            cv::putText(image, distance_text, cv::Point(box.x + 2, box.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }

        // Prepare label text
        std::ostringstream label_ss;
        label_ss << CLASS_NAMES[class_id] << " " << std::fixed << std::setprecision(2) << conf;
        std::string label = label_ss.str();

        // Text and background size
        int baseLine = 0;
        cv::Size textSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseLine);
        int top = std::max((int)box.y, textSize.height);

        // Draw label text (yellow text with black outline)
        putText(image, label, cv::Point(box.x + 2, box.y - 5),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2); // Yellow text
    }
}

int Detect::getInputH()
{
    return input_h;
}

int Detect::getInputW()
{
    return input_w;
}