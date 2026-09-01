#include "yolov9mit/yolov9mit_tensorrt.hpp"

#include <fstream>

#include "cuda_runtime_api.h"

namespace yolov9mit
{

static inline void print_dims(nvinfer1::Dims dims)
{
    std::cout << "[ ";
    for (int32_t i = 0; i < dims.nbDims; ++i)
    {
        std::cout << dims.d[i];
        if (i < dims.nbDims - 1) std::cout << ", ";
    }
    std::cout << " ]";
}

static inline void cuda_check(cudaError_t status)
{
    if (status != 0)
    {
        // HH_260811 - The error object used to be constructed and discarded, so every
        // CUDA failure passed silently.
        throw std::runtime_error(cudaGetErrorString(status));
    }
}

YOLOV9MIT_TensorRT::YOLOV9MIT_TensorRT(const std::string& engine_path, const int32_t device,
                                       const float min_iou, const float min_confidence,
                                       const size_t num_classes)
    : AbcYOLOV9MIT(min_iou, min_confidence, num_classes), device_(device)
{
    cuda_check(cudaSetDevice(this->device_));

    this->runtime_ = std::unique_ptr<IRuntime>(createInferRuntime(this->trt_logger_));
    if (this->runtime_ == nullptr)
    {
        throw std::runtime_error("TensorRT createInferRuntime returned null");
    }

    std::ifstream file(engine_path, std::ios::binary);
    if (file.good())
    {
        file.seekg(0, file.end);
        const auto end_position = file.tellg();
        if (end_position <= 0)
        {
            throw std::runtime_error("TensorRT engine file is empty: " + engine_path);
        }
        const size_t size = static_cast<size_t>(end_position);
        file.seekg(0, file.beg);
        std::vector<char> trt_model_stream(size);
        file.read(trt_model_stream.data(), static_cast<std::streamsize>(size));
        if (!file)
        {
            throw std::runtime_error("failed to read TensorRT engine: " + engine_path);
        }
        file.close();

        this->engine_ = std::unique_ptr<ICudaEngine>(
            this->runtime_->deserializeCudaEngine(trt_model_stream.data(), size));
        if (this->engine_ == nullptr)
        {
            throw std::runtime_error(
                "TensorRT could not deserialize engine (runtime/GPU mismatch): " + engine_path);
        }
    }
    else
    {
        throw std::runtime_error("TensorRT engine path does not exist: " + engine_path);
    }

    this->context_ = std::unique_ptr<IExecutionContext>(this->engine_->createExecutionContext());
    if (this->context_ == nullptr)
    {
        throw std::runtime_error("TensorRT createExecutionContext returned null");
    }

    const auto input_name = this->engine_->getIOTensorName(this->input_index_);
    const auto output0_name = this->engine_->getIOTensorName(this->output0_index_);
    const auto output1_name = this->engine_->getIOTensorName(this->output1_index_);

    assert(this->engine_->getTensorDataType(input_name) == nvinfer1::DataType::kFLOAT);
    assert(this->engine_->getTensorDataType(output0_name) == nvinfer1::DataType::kFLOAT);
    assert(this->engine_->getTensorDataType(output1_name) == nvinfer1::DataType::kFLOAT);

    const auto input_dims = this->engine_->getTensorShape(input_name);
    this->input_channel_ = input_dims.d[1];
    this->input_h_ = input_dims.d[2];
    this->input_w_ = input_dims.d[3];

    std::cout << "MODEL Input:" << std::endl;
    std::cout << "  name:  " << input_name << std::endl;
    std::cout << "  shape: ";
    print_dims(input_dims);
    std::cout << std::endl;

    {
        auto output0_dims = this->engine_->getTensorShape(output0_name);
        this->output0_size_ = 1;
        for (int32_t j = 0; j < output0_dims.nbDims; ++j)
        {
            this->output0_size_ *= output0_dims.d[j];
        }
        std::cout << "MODEL Output1:" << std::endl;
        std::cout << "  name:  " << output0_name << std::endl;
        std::cout << "  shape: ";
        print_dims(output0_dims);
        std::cout << std::endl;
    }

    {
        auto output1_dims = this->engine_->getTensorShape(output1_name);
        this->output1_size_ = 1;
        for (int32_t j = 0; j < output1_dims.nbDims; ++j)
        {
            this->output1_size_ *= output1_dims.d[j];
        }
        std::cout << "MODEL Output2:" << std::endl;
        std::cout << "  name:  " << output1_name << std::endl;
        std::cout << "  shape: ";
        print_dims(output1_dims);
        std::cout << std::endl;
    }
    std::cout << std::endl;

    cuda_check(cudaMalloc(&inference_buffers_[this->input_index_],
                          this->input_channel_ * this->input_h_ * this->input_w_ * sizeof(float)));
    cuda_check(
        cudaMalloc(&inference_buffers_[this->output0_index_], this->output0_size_ * sizeof(float)));
    cuda_check(
        cudaMalloc(&inference_buffers_[this->output1_index_], this->output1_size_ * sizeof(float)));

    output_blob_classes_.resize(output0_size_);
    output_blob_bbox_.resize(output1_size_);

    cuda_check(cudaStreamCreate(&stream_));

    // HH_260811 - These calls used to sit inside assert(), so the canonical Release
    // build (-DNDEBUG) stripped them along with their side effects. enqueueV3() then
    // ran with unbound tensor addresses and every frame failed inference.
    if (!context_->setInputShape(input_name, input_dims))
    {
        std::cerr << "setInputShape() rejected " << input_name
                  << "; falling back to the engine's static input shape" << std::endl;
    }
    if (!context_->allInputDimensionsSpecified())
    {
        throw std::runtime_error("input dimensions are not fully specified");
    }

    if (!context_->setTensorAddress(input_name, inference_buffers_[this->input_index_]))
    {
        throw std::runtime_error(std::string("setTensorAddress failed for ") + input_name);
    }
    if (!context_->setTensorAddress(output0_name, inference_buffers_[this->output0_index_]))
    {
        throw std::runtime_error(std::string("setTensorAddress failed for ") + output0_name);
    }
    if (!context_->setTensorAddress(output1_name, inference_buffers_[this->output1_index_]))
    {
        throw std::runtime_error(std::string("setTensorAddress failed for ") + output1_name);
    }
}

YOLOV9MIT_TensorRT::~YOLOV9MIT_TensorRT()
{
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
    for (void *buffer : inference_buffers_)
    {
        if (buffer != nullptr) cudaFree(buffer);
    }
}

std::vector<Object> YOLOV9MIT_TensorRT::inference(const cv::Mat& frame)
{
    const auto pr_img = preprocess(frame);

    // HWC -> NCHW
    blobFromImage(pr_img);

    this->doInference(output_blob_classes_, output_blob_bbox_);

    const auto objects =
        decode_outputs(output_blob_classes_, output_blob_bbox_, frame.cols, frame.rows);

    return objects;
}

void YOLOV9MIT_TensorRT::doInference(PinnedVector<float>& output0, PinnedVector<float>& output1)
{
    cuda_check(cudaMemcpyAsync(inference_buffers_[this->input_index_], blob_data_.data(),
                               3 * this->input_h_ * this->input_w_ * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));

    bool status = context_->enqueueV3(stream_);
    if (!status) throw std::runtime_error("failed inference");

    cuda_check(cudaMemcpyAsync(output0.data(), inference_buffers_[this->output0_index_],
                               this->output0_size_ * sizeof(float), cudaMemcpyDeviceToHost,
                               stream_));
    cuda_check(cudaMemcpyAsync(output1.data(), inference_buffers_[this->output1_index_],
                               this->output1_size_ * sizeof(float), cudaMemcpyDeviceToHost,
                               stream_));

    cuda_check(cudaStreamSynchronize(stream_));
}

} // namespace yolov9mit
