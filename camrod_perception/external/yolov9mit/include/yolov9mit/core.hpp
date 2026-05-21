#ifndef _YOLOV9MIT_CORE_HPP_
#define _YOLOV9MIT_CORE_HPP_

#include <cuda_runtime_api.h>
#include <opencv2/opencv.hpp>

namespace yolov9mit
{

// CUDA page-locked (pinned) allocator for std::vector.
// On Jetson unified memory, pinned alloc enables true async DMA transfers.
template <typename T>
struct PinnedAllocator
{
    using value_type = T;
    T * allocate(std::size_t n)
    {
        T * ptr = nullptr;
        cudaMallocHost(reinterpret_cast<void **>(&ptr), n * sizeof(T));
        return ptr;
    }
    void deallocate(T * ptr, std::size_t) { cudaFreeHost(ptr); }
    template <typename U>
    bool operator==(const PinnedAllocator<U> &) const noexcept { return true; }
    template <typename U>
    bool operator!=(const PinnedAllocator<U> &) const noexcept { return false; }
};

template <typename T>
using PinnedVector = std::vector<T, PinnedAllocator<T>>;



struct Object
{
    cv::Rect2f rect;
    int32_t class_id;
    float confidence;

    Object(const cv::Rect2f &rect_, const int32_t class_id_, const float confidence_)
        : rect(rect_), class_id(class_id_), confidence(confidence_)
    {
    }
};

class AbcYOLOV9MIT
{
public:
    AbcYOLOV9MIT() {}
    AbcYOLOV9MIT(const float min_iou = 0.45f, const float min_confidence = 0.6f,
                 const size_t num_classes = 80)
        : min_iou_(min_iou), min_confidence_(min_confidence), num_classes_(num_classes)
    {
    }
    virtual std::vector<Object> inference(const cv::Mat &frame) = 0;

private:
    inline float sigmoid(const float x) { return 1.0f / (1.0f + std::exp(-x)); }

protected:
    int32_t input_w_;
    int32_t input_h_;
    int32_t input_channel_;
    float min_iou_;
    float min_confidence_;
    size_t num_classes_;
    const float blob_scale_ = 1.0f / 255.0f;
    PinnedVector<float> blob_data_;
    // Reused across frames to avoid per-frame heap allocation in blobFromImage
    std::vector<cv::Mat> split_channels_;

    cv::Mat preprocess(const cv::Mat &img)
    {
        cv::Mat output(input_h_, input_w_, CV_8UC3);
        cv::resize(img, output, output.size());
        cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
        return output;
    }

    // HWC -> NCHW, writes into pinned blob_data_
    void blobFromImage(const cv::Mat &img)
    {
        const int32_t channel_size = input_h_ * input_w_;
        const int32_t input_size = input_channel_ * channel_size;
        if ((int32_t)blob_data_.size() != input_size)
        {
            blob_data_.resize(input_size);
        }

        // dst views point directly into pinned blob_data_ — no extra copy
        split_channels_.resize(input_channel_);
        for (int32_t c = 0; c < input_channel_; ++c)
        {
            split_channels_[c] = cv::Mat(input_h_, input_w_, CV_32FC1,
                                         blob_data_.data() + c * channel_size);
        }

        // split into temporary u8 planes, then convert in-place into pinned dst
        std::vector<cv::Mat> tmp(input_channel_);
        cv::split(img, tmp);
        for (int32_t c = 0; c < input_channel_; ++c)
        {
            tmp[c].convertTo(split_channels_[c], CV_32FC1, blob_scale_);
        }
    }

    template <typename Alloc1, typename Alloc2>
    std::vector<Object> outputs_to_objects(const std::vector<float, Alloc1> &prob_classes,
                                           const std::vector<float, Alloc2> &prob_bboxes,
                                           const int32_t org_img_w, const int32_t org_img_h)
    {
        std::vector<Object> objects;

        const float w_scale = (float)org_img_w / (float)input_w_;
        const float h_scale = (float)org_img_h / (float)input_h_;
        const float x_max = org_img_w - 1;
        const float y_max = org_img_h - 1;

        const size_t length = prob_classes.size() / this->num_classes_;

        for (size_t i = 0; i < length; ++i)
        {
            const size_t idx = i * num_classes_;
            int32_t class_id = -1;
            float max_raw = -std::numeric_limits<float>::infinity();

            // sigmoid is monotonically increasing, so argmax(sigmoid(x)) == argmax(x)
            for (size_t class_idx = 0; class_idx < num_classes_; ++class_idx)
            {
                const float raw = prob_classes[idx + class_idx];
                if (raw > max_raw)
                {
                    class_id = class_idx;
                    max_raw = raw;
                }
            }
            const float max_confidence = sigmoid(max_raw);
            if (max_confidence > this->min_confidence_)
            {
                const size_t bbox_idx = i * 4;
                float x0 = prob_bboxes[bbox_idx + 0] * w_scale;
                float y0 = prob_bboxes[bbox_idx + 1] * h_scale;
                float x1 = prob_bboxes[bbox_idx + 2] * w_scale;
                float y1 = prob_bboxes[bbox_idx + 3] * h_scale;

                // clip
                // clamp to image bounds
                x0 = std::max(std::min(x0, x_max), 0.f);
                y0 = std::max(std::min(y0, y_max), 0.f);
                x1 = std::max(std::min(x1, x_max), 0.f);
                y1 = std::max(std::min(y1, y_max), 0.f);

                objects.emplace_back(cv::Rect2f(x0, y0, x1 - x0, y1 - y0), class_id,
                                     max_confidence);
            }
        }
        return objects;
    }

    std::vector<Object> nms(const std::vector<Object> &objects)
    {
        std::vector<Object> results;
        results.push_back(objects[0]);
        for (size_t i = 1; i < objects.size(); ++i)
        {
            const auto obj = objects[i];
            const auto obj_area = obj.rect.area();
            bool keep = true;
            for (const auto &result : results)
            {
                if (obj.class_id != result.class_id) continue;
                const auto intersect_area = (obj.rect & result.rect).area();
                const auto union_area = obj_area + result.rect.area() - intersect_area;
                const auto iou = intersect_area / union_area;
                if (iou > this->min_iou_)
                {
                    keep = false;
                    break;
                }
            }
            if (keep) results.push_back(obj);
        }
        return results;
    }

    template <typename Alloc1, typename Alloc2>
    std::vector<Object> decode_outputs(const std::vector<float, Alloc1> &prob_classes,
                                       const std::vector<float, Alloc2> &prob_bboxes,
                                       const int32_t org_img_w, const int32_t org_img_h)
    {
        auto objects = outputs_to_objects(prob_classes, prob_bboxes, org_img_w, org_img_h);
        if (objects.size() == 0)
        {
            return {};
        }

        std::sort(objects.begin(), objects.end(),
                  [](const Object &a, const Object &b) { return a.confidence > b.confidence; });

        const auto results = nms(objects);
        return results;
    }
};
} // namespace yolov9mit
#endif
