// Copyright 2026 hwanhonglee
//
// Licensed under the MIT License.

#include <cuda_runtime_api.h>

#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvOnnxParser.h"
#include "yolov9mit/tensorrt_logger.h"

namespace
{

struct Options
{
    std::filesystem::path onnx_path;
    std::filesystem::path engine_path;
    std::filesystem::path validate_engine_path;
    std::size_t workspace_mib{2048U};
    int device{0};
    bool fp16{false};
    bool print_runtime_key{false};
};

const char *usage_text()
{
    return
        "usage:\n"
        "  yolov9mit_build_engine --onnx MODEL.onnx --engine MODEL.engine "
        "[--workspace-mib 2048] [--device 0] [--fp16]\n"
        "  yolov9mit_build_engine --validate-engine MODEL.engine [--device 0]\n"
        "  yolov9mit_build_engine --print-runtime-key [--device 0]\n";
}

[[noreturn]] void usage_error(const std::string &message)
{
    throw std::invalid_argument(
        message + "\n" + usage_text());
}

std::size_t parse_size(const std::string &value, const char *name)
{
    std::size_t consumed = 0U;
    unsigned long long parsed = 0U;
    try
    {
        parsed = std::stoull(value, &consumed, 10);
    }
    catch (const std::exception &)
    {
        usage_error(std::string("invalid ") + name + ": " + value);
    }
    if (consumed != value.size() || parsed == 0U ||
        parsed > std::numeric_limits<std::size_t>::max())
    {
        usage_error(std::string("invalid ") + name + ": " + value);
    }
    return static_cast<std::size_t>(parsed);
}

int parse_device(const std::string &value)
{
    std::size_t consumed = 0U;
    long parsed = 0;
    try
    {
        parsed = std::stol(value, &consumed, 10);
    }
    catch (const std::exception &)
    {
        usage_error("invalid device: " + value);
    }
    if (consumed != value.size() || parsed < 0 ||
        parsed > std::numeric_limits<int>::max())
    {
        usage_error("invalid device: " + value);
    }
    return static_cast<int>(parsed);
}

Options parse_options(int argc, char **argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument(argv[index]);
        if (argument == "--fp16")
        {
            options.fp16 = true;
            continue;
        }
        if (argument == "--print-runtime-key")
        {
            options.print_runtime_key = true;
            continue;
        }
        if (argument == "--help" || argument == "-h")
        {
            std::cout << usage_text();
            std::exit(0);
        }
        if (index + 1 >= argc)
        {
            usage_error("missing value for " + argument);
        }
        const std::string value(argv[++index]);
        if (argument == "--onnx")
        {
            options.onnx_path = value;
        }
        else if (argument == "--engine")
        {
            options.engine_path = value;
        }
        else if (argument == "--validate-engine")
        {
            options.validate_engine_path = value;
        }
        else if (argument == "--workspace-mib")
        {
            options.workspace_mib = parse_size(value, "workspace-mib");
        }
        else if (argument == "--device")
        {
            options.device = parse_device(value);
        }
        else
        {
            usage_error("unknown argument: " + argument);
        }
    }
    const bool build_requested =
        !options.onnx_path.empty() || !options.engine_path.empty();
    const int mode_count = static_cast<int>(build_requested) +
                           static_cast<int>(options.print_runtime_key) +
                           static_cast<int>(!options.validate_engine_path.empty());
    if (mode_count != 1)
    {
        usage_error("select exactly one build, validation, or runtime-key mode");
    }
    if (build_requested && options.onnx_path.empty())
    {
        usage_error("--onnx is required");
    }
    if (build_requested && options.engine_path.empty())
    {
        usage_error("--engine is required");
    }
    if (!build_requested && options.fp16)
    {
        usage_error("--fp16 is only valid in build mode");
    }
    return options;
}

void cuda_check(cudaError_t status, const char *operation)
{
    if (status != cudaSuccess)
    {
        throw std::runtime_error(
            std::string(operation) + " failed: " + cudaGetErrorString(status));
    }
}

cudaDeviceProp select_device(int device)
{
    cuda_check(cudaSetDevice(device), "cudaSetDevice");
    cudaDeviceProp properties{};
    cuda_check(cudaGetDeviceProperties(&properties, device), "cudaGetDeviceProperties");
    return properties;
}

std::string runtime_key(const cudaDeviceProp &properties)
{
    std::string gpu_name;
    bool separator_pending = false;
    for (const char character : std::string(properties.name))
    {
        const auto byte = static_cast<unsigned char>(character);
        if (std::isalnum(byte))
        {
            if (separator_pending && !gpu_name.empty())
            {
                gpu_name += '-';
            }
            gpu_name += static_cast<char>(std::tolower(byte));
            separator_pending = false;
        }
        else
        {
            separator_pending = true;
        }
    }
    return "trt" + std::to_string(NV_TENSORRT_MAJOR) + "." +
           std::to_string(NV_TENSORRT_MINOR) + "." +
           std::to_string(NV_TENSORRT_PATCH) + "." +
           std::to_string(NV_TENSORRT_BUILD) + "-sm" +
           std::to_string(properties.major) + std::to_string(properties.minor) +
           "-" + gpu_name;
}

void initialize_plugins(MyTRTLogger &logger)
{
    if (!initLibNvInferPlugins(&logger, ""))
    {
        throw std::runtime_error("initLibNvInferPlugins failed");
    }
}

int validate_engine(const Options &options)
{
    if (!std::filesystem::is_regular_file(options.validate_engine_path))
    {
        throw std::runtime_error(
            "engine input is not a regular file: " +
            options.validate_engine_path.string());
    }
    const auto file_size = std::filesystem::file_size(options.validate_engine_path);
    if (file_size == 0U ||
        file_size > static_cast<std::uintmax_t>(
                        std::numeric_limits<std::size_t>::max()) ||
        file_size > static_cast<std::uintmax_t>(
                        std::numeric_limits<std::streamsize>::max()))
    {
        throw std::runtime_error("engine input has an invalid size");
    }
    std::vector<char> bytes(static_cast<std::size_t>(file_size));
    std::ifstream stream(options.validate_engine_path, std::ios::binary);
    if (!stream || !stream.read(bytes.data(), static_cast<std::streamsize>(bytes.size())))
    {
        throw std::runtime_error(
            "could not read engine input: " + options.validate_engine_path.string());
    }

    const auto properties = select_device(options.device);
    MyTRTLogger logger(nvinfer1::ILogger::Severity::kWARNING);
    initialize_plugins(logger);
    std::unique_ptr<nvinfer1::IRuntime> runtime(nvinfer1::createInferRuntime(logger));
    if (!runtime)
    {
        throw std::runtime_error("createInferRuntime returned null");
    }
    std::unique_ptr<nvinfer1::ICudaEngine> engine(
        runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine)
    {
        throw std::runtime_error("deserializeCudaEngine returned null");
    }
#if NV_TENSORRT_MAJOR >= 10
    const auto io_count = engine->getNbIOTensors();
#else
    const auto io_count = engine->getNbBindings();
#endif
    std::cout << "validated engine=" << options.validate_engine_path
              << " bytes=" << bytes.size()
              << " io_tensors=" << io_count
              << " runtime_key=" << runtime_key(properties) << '\n';
    return 0;
}

std::string dimensions_to_string(const nvinfer1::Dims &dimensions)
{
    std::string result("[");
    for (int32_t index = 0; index < dimensions.nbDims; ++index)
    {
        if (index != 0)
        {
            result += ",";
        }
        result += std::to_string(dimensions.d[index]);
    }
    result += "]";
    return result;
}

void require_static_inputs(const nvinfer1::INetworkDefinition &network)
{
    for (int32_t input_index = 0; input_index < network.getNbInputs(); ++input_index)
    {
        const auto *tensor = network.getInput(input_index);
        if (tensor == nullptr)
        {
            throw std::runtime_error("TensorRT returned a null network input");
        }
        const auto dimensions = tensor->getDimensions();
        for (int32_t axis = 0; axis < dimensions.nbDims; ++axis)
        {
            if (dimensions.d[axis] < 0)
            {
                throw std::runtime_error(
                    std::string("dynamic ONNX input is not supported by this safe builder: ") +
                    tensor->getName() + " " + dimensions_to_string(dimensions));
            }
        }
        std::cout << "input " << tensor->getName() << " "
                  << dimensions_to_string(dimensions) << '\n';
    }
}

class TemporaryFile
{
public:
    explicit TemporaryFile(std::filesystem::path path) : path_(std::move(path)) {}

    ~TemporaryFile()
    {
        if (!committed_)
        {
            std::error_code error;
            std::filesystem::remove(path_, error);
        }
    }

    const std::filesystem::path &path() const { return path_; }
    void commit() { committed_ = true; }

private:
    std::filesystem::path path_;
    bool committed_{false};
};

void write_engine_atomically(
    const nvinfer1::IHostMemory &engine, const std::filesystem::path &output_path)
{
    if (std::filesystem::exists(output_path))
    {
        throw std::runtime_error("refusing to overwrite existing engine: " + output_path.string());
    }
    const auto parent = output_path.parent_path().empty()
                            ? std::filesystem::current_path()
                            : output_path.parent_path();
    if (!std::filesystem::is_directory(parent))
    {
        throw std::runtime_error("engine output directory does not exist: " + parent.string());
    }

    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    TemporaryFile temporary(output_path.string() + ".tmp." + std::to_string(nonce));
    std::ofstream stream(temporary.path(), std::ios::binary | std::ios::trunc);
    if (!stream)
    {
        throw std::runtime_error("could not open temporary engine output: " +
                                 temporary.path().string());
    }
    stream.write(
        static_cast<const char *>(engine.data()),
        static_cast<std::streamsize>(engine.size()));
    stream.flush();
    if (!stream)
    {
        throw std::runtime_error("failed while writing engine: " + temporary.path().string());
    }
    stream.close();
    std::filesystem::rename(temporary.path(), output_path);
    temporary.commit();
}

int build_engine(const Options &options)
{
    if (!std::filesystem::is_regular_file(options.onnx_path))
    {
        throw std::runtime_error("ONNX input is not a regular file: " +
                                 options.onnx_path.string());
    }
    if (options.workspace_mib >
        std::numeric_limits<std::size_t>::max() / (1024U * 1024U))
    {
        throw std::runtime_error("workspace-mib is too large");
    }

    const auto device_properties = select_device(options.device);
    std::cout << "GPU " << options.device << ": " << device_properties.name
              << " compute=" << device_properties.major << '.'
              << device_properties.minor << '\n';
    std::cout << "TensorRT " << NV_TENSORRT_MAJOR << '.' << NV_TENSORRT_MINOR
              << '.' << NV_TENSORRT_PATCH << '.' << NV_TENSORRT_BUILD << '\n';

    MyTRTLogger logger(nvinfer1::ILogger::Severity::kINFO);
    initialize_plugins(logger);

    std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
    if (!builder)
    {
        throw std::runtime_error("createInferBuilder returned null");
    }
#if NV_TENSORRT_MAJOR >= 10
    constexpr uint32_t network_flags = 0U;
#else
    constexpr uint32_t network_flags =
        1U << static_cast<uint32_t>(
                  nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
#endif
    std::unique_ptr<nvinfer1::INetworkDefinition> network(
        builder->createNetworkV2(network_flags));
    if (!network)
    {
        throw std::runtime_error("createNetworkV2 returned null");
    }
    std::unique_ptr<nvonnxparser::IParser> parser(
        nvonnxparser::createParser(*network, logger));
    if (!parser)
    {
        throw std::runtime_error("nvonnxparser::createParser returned null");
    }
    if (!parser->parseFromFile(
            options.onnx_path.c_str(),
            static_cast<int>(nvinfer1::ILogger::Severity::kINFO)))
    {
        std::string message("ONNX parsing failed");
        for (int32_t index = 0; index < parser->getNbErrors(); ++index)
        {
            const auto *error = parser->getError(index);
            if (error != nullptr)
            {
                message += "\n  - ";
                message += error->desc();
            }
        }
        throw std::runtime_error(message);
    }
    require_static_inputs(*network);
    std::cout << "network outputs=" << network->getNbOutputs() << '\n';

    std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
    if (!config)
    {
        throw std::runtime_error("createBuilderConfig returned null");
    }
    config->setMemoryPoolLimit(
        nvinfer1::MemoryPoolType::kWORKSPACE,
        options.workspace_mib * 1024U * 1024U);
    if (options.fp16)
    {
        if (!builder->platformHasFastFp16())
        {
            throw std::runtime_error("--fp16 requested but this GPU has no fast FP16 support");
        }
        config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }

    std::cout << "building engine workspace_mib=" << options.workspace_mib
              << " fp16=" << (options.fp16 ? "true" : "false") << std::endl;
    std::unique_ptr<nvinfer1::IHostMemory> serialized(
        builder->buildSerializedNetwork(*network, *config));
    if (!serialized)
    {
        throw std::runtime_error("buildSerializedNetwork returned null");
    }
    write_engine_atomically(*serialized, options.engine_path);
    std::cout << "engine bytes=" << serialized->size()
              << " path=" << options.engine_path << '\n';
    return 0;
}

}  // namespace

int main(int argc, char **argv)
{
    try
    {
        const auto options = parse_options(argc, argv);
        if (options.print_runtime_key)
        {
            std::cout << runtime_key(select_device(options.device)) << '\n';
            return 0;
        }
        if (!options.validate_engine_path.empty())
        {
            return validate_engine(options);
        }
        return build_engine(options);
    }
    catch (const std::exception &error)
    {
        std::cerr << "yolov9mit_build_engine: " << error.what() << '\n';
        return 1;
    }
}
