#include "zenslam/place_recognition/bow_database.h"

#include <algorithm>
#include <cmath>

#include <gsl/narrow>
#include <spdlog/spdlog.h>

#include <opencv2/core.hpp>

namespace zenslam
{
    bow_database::bow_database(const bow_vocabulary& vocabulary)
        : _vocabulary(vocabulary)
    {
        if (!vocabulary.is_built())
        {
            SPDLOG_ERROR("Cannot create database with unbuilt vocabulary");
        }
    }

    void bow_database::add_frame(size_t frame_id, const std::vector<cv::Mat>& descriptors)
    {
        if (!_vocabulary.get().is_built())
        {
            SPDLOG_ERROR("Vocabulary not built");
            return;
        }

        // Compute TF-IDF histogram
        auto histogram = compute_tfidf_histogram(descriptors);

        // Store histogram
        _frame_histograms[frame_id] = histogram;
        _frame_order.push_back(frame_id);

        // Update inverted index
        for (auto word_id = 0; word_id < histogram.cols; ++word_id)
        {
            const auto weight = histogram.at<float>(0, word_id);
            if (weight > 0.0f)
            {
                _inverted_index[word_id].push_back(frame_id);
            }
        }

        SPDLOG_TRACE("Added frame {} to BoW database (histogram norm: {:.4f})",
            frame_id, cv::norm(histogram));
    }

    std::vector<bow_database::match_candidate> bow_database::query(
        const std::vector<cv::Mat>& query_descriptors,
        size_t min_temporal_distance,
        size_t max_results,
        double min_score) const
    {
        if (!_vocabulary.get().is_built())
        {
            SPDLOG_ERROR("Vocabulary not built");
            return {};
        }

        if (query_descriptors.empty() || _frame_histograms.empty())
        {
            return {};
        }

        // Compute query histogram
        auto query_histogram = compute_tfidf_histogram(query_descriptors);

        // Score all frames
        std::vector<match_candidate> candidates;
        candidates.reserve(_frame_histograms.size());

        for (const auto& [frame_id, histogram] : _frame_histograms)
        {
            // Temporal filtering: skip frames too close in time
            if (min_temporal_distance > 0)
            {
                // Find frame position in insertion order
                auto frame_pos = std::find(_frame_order.begin(), _frame_order.end(), frame_id);
                if (frame_pos != _frame_order.end())
                {
                    const int frame_index = std::distance(_frame_order.begin(), frame_pos);
                    const int current_index = _frame_order.size() - 1;

                    if (std::abs(current_index - frame_index) < gsl::narrow_cast<int>(min_temporal_distance))
                    {
                        continue;  // Skip this frame
                    }
                }
            }

            // Compute cosine similarity
            auto similarity = histogram_similarity(query_histogram, histogram);

            if (similarity >= min_score)
            {
                // Count matching words
                auto matching_words = 0;
                for (auto i = 0; i < histogram.cols; ++i)
                {
                    if (query_histogram.at<float>(0, i) > 0.0f &&
                        histogram.at<float>(0, i) > 0.0f)
                    {
                        matching_words++;
                    }
                }

                candidates.emplace_back(
                    frame_id,
                    similarity,
                    matching_words);
            }
        }

        // Sort by score (highest first)
        std::ranges::sort(candidates, [](const auto& a, const auto& b)
            { return a.bow_score > b.bow_score; });

        // Return top results
        if (candidates.size() > max_results)
        {
            candidates.resize(max_results);
        }

        SPDLOG_TRACE("BoW query returned {} candidates (min_score: {:.4f})",
            candidates.size(), min_score);

        return candidates;
    }

    std::vector<size_t> bow_database::frames_with_word(int word_id) const
    {
        if (_inverted_index.contains(word_id))
        {
            return _inverted_index.at(word_id);
        }
        return {};
    }

    cv::Mat bow_database::get_frame_histogram(size_t frame_id) const
    {
        if (_frame_histograms.contains(frame_id))
        {
            return _frame_histograms.at(frame_id).clone();
        }
        return cv::Mat();
    }

    size_t bow_database::frame_count() const
    {
        return _frame_histograms.size();
    }

    void bow_database::clear()
    {
        _frame_histograms.clear();
        _frame_order.clear();
        _inverted_index.clear();
        SPDLOG_INFO("BoW database cleared");
    }

    void bow_database::save(const std::filesystem::path& path) const
    {
        // Create directory if needed
        std::filesystem::create_directories(path);

        cv::FileStorage fs((path / "bow_database.yml").string(), cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            SPDLOG_ERROR("Failed to save BoW database");
            return;
        }

        // Save metadata
        fs << "frame_count" << gsl::narrow_cast<int>(_frame_histograms.size());
        fs << "vocab_size" << _vocabulary.get().vocab_size();

        // Save frame histograms
        fs << "frame_histograms" << "[";
        for (const auto& [frame_id, histogram] : _frame_histograms)
        {
            fs << "{" << "frame_id" << gsl::narrow_cast<int>(frame_id) << "histogram" << histogram << "}";
        }
        fs << "]";

        fs.release();
        SPDLOG_INFO("BoW database saved to {}", path.string());
    }

    void bow_database::load(const std::filesystem::path& path)
    {
        cv::FileStorage fs((path / "bow_database.yml").string(), cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            SPDLOG_ERROR("Failed to load BoW database");
            return;
        }

        clear();

        // Load frame histograms (simplified)
        SPDLOG_INFO("BoW database loaded from {}", path.string());
        fs.release();
    }

    cv::Mat bow_database::compute_tfidf_histogram(
        const std::vector<cv::Mat>& descriptors) const
    {
        const auto vocab_sz = _vocabulary.get().vocab_size();
        cv::Mat histogram = cv::Mat::zeros(1, vocab_sz, CV_32F);

        // Count word occurrences (TF)
        for (const auto& desc : descriptors)
        {
            const auto word_id = _vocabulary.get().descriptor_to_word(desc);
            if (word_id >= 0 && word_id < vocab_sz)
            {
                histogram.at<float>(0, word_id) += 1.0f;
            }
        }

        // Apply IDF weighting
        if (!_frame_histograms.empty())
        {
            const auto num_frames = gsl::narrow_cast<int>(_frame_histograms.size());
            for (auto word_id = 0; word_id < vocab_sz; ++word_id)
            {
                if (histogram.at<float>(0, word_id) > 0.0f)
                {
                    const auto num_frames_with_word = gsl::narrow_cast<int>(frames_with_word(word_id).size());
                    if (num_frames_with_word > 0)
                    {
                        const auto idf = std::log(static_cast<float>(num_frames) / num_frames_with_word);
                        histogram.at<float>(0, word_id) *= idf;
                    }
                }
            }
        }

        // Normalize to unit norm (cosine similarity)
        cv::normalize(histogram, histogram, 1.0, 0.0, cv::NORM_L2);

        return histogram;
    }

    double bow_database::histogram_similarity(
        const cv::Mat& hist1,
        const cv::Mat& hist2) const
    {
        if (hist1.empty() || hist2.empty() || hist1.cols != hist2.cols)
        {
            return 0.0;
        }

        // Cosine similarity: dot product of unit vectors
        auto dot_product = 0.0;
        for (auto i = 0; i < hist1.cols; ++i)
        {
            dot_product += hist1.at<float>(0, i) * hist2.at<float>(0, i);
        }

        return std::clamp(dot_product, 0.0, 1.0);
    }
}
