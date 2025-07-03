#pragma once

#include <STrack.h>

#include <opencv2/opencv.hpp>  // OpenCV for image processing

#include "NvInfer.h"       // TensorRT library for high-performance inference
using namespace nvinfer1;  // Namespace for TensorRT
using namespace std;       // Use standard library namespace
using namespace cv;        // Use OpenCV namespace

// Struct to store detection results
struct Detection {
    float conf;   // Confidence score of the detection
    int classId;  // Class ID of the detected object (e.g., person, car, etc.)
    Rect bbox;    // Bounding box coordinates around the detected object
};

// Main class for the YOLOv12 model
class Detect {
   private:
    // Private members for the Detect class
    cv::Rect scaleBoxToOriginal(const cv::Rect& input_box, const cv::Size& model_input_size,
                                const cv::Size& original_size);

   public:
    // Constructor: Loads the TensorRT engine and initializes the model
    Detect(string model_path, nvinfer1::ILogger& logger);

    // Destructor: Cleans up resources used by the model
    ~Detect();

    // Preprocess the input image to match the model's input format
    void preprocess(Mat& image);

    // Run inference on the preprocessed image
    void infer();

    // Postprocess the model's output to extract detection results
    void postprocess(Mat& image, vector<Detection>& output);

    // Draw bounding boxes and labels on the original image
    void draw(Mat& image, const vector<STrack>& output);

   private:
    // Initialize the TensorRT engine from a serialized model file
    void init(std::string engine_path, nvinfer1::ILogger& logger);

    // Device (GPU) buffers for input and output
    float* gpu_buffers[2];  //!< Input and output buffers allocated on the GPU

    // Host (CPU) buffer for storing inference output
    float* cpu_output_buffer;

    // CUDA stream for asynchronous execution
    cudaStream_t stream;

    // TensorRT runtime for deserializing the engine from file
    IRuntime* runtime;

    // TensorRT engine used to execute the network
    ICudaEngine* engine;

    // Execution context for running inference with the engine
    IExecutionContext* context;

    // Model parameters
    int inputWeight;               // Input image width expected by the model
    int inputHeight;               // Input image height expected by the model
    int num_detections;            // Number of detections output by the model
    int detection_attribute_size;  // Attributes (e.g., bbox, class) per detection
    int num_classes = 21;          // Number of classes (e.g., COCO dataset has 80 classes)

    // Maximum supported image size (used for memory allocation checks)
    const int MAX_IMAGE_SIZE = 4096 * 4096;

    // Build TensorRT engine from an ONNX model file (if applicable)
    void build(std::string onnxPath, nvinfer1::ILogger& logger);

    // Save the built TensorRT engine to a file
    bool saveEngine(const std::string& filename);
};