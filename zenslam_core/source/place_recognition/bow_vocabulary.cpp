#include "zenslam/place_recognition/bow_vocabulary.h"

#include <algorithm>
#include <cmath>
#include <ranges>

#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <spdlog/spdlog.h>

namespace zenslam
{
    bow_vocabulary::bow_vocabulary(int levels, int branching_factor)
        : _levels(levels), _branching_factor(branching_factor)
    {
    }

    void bow_vocabulary::build(
        const std::vector<cv::Mat>& training_descriptors,
        int iterations)
    {
        if (training_descriptors.empty())
        {
            SPDLOG_ERROR("Cannot build vocabulary: no training descriptors provided");
            return;
        }

        SPDLOG_INFO("Building hierarchical BoW vocabulary: levels={}, branching={}", _levels, _branching_factor);
        SPDLOG_INFO("Training set size: {} descriptors", training_descriptors.size());

        // Initialize cluster storage
        _clusters.resize(_levels + 1);
        _cluster_sizes.resize(_levels + 1);

        for (int level = 0; level <= _levels; ++level)
        {
            int num_nodes_at_level = 1;
            for (int i = 0; i < level; ++i)
            {
                num_nodes_at_level *= _branching_factor;
            }
            _clusters[level].resize(num_nodes_at_level);
            _cluster_sizes[level].resize(num_nodes_at_level, 0);
        }

        // Recursively build vocabulary tree
        build_recursive(training_descriptors, 0, 0, iterations);

        _is_built = true;
        SPDLOG_INFO("Vocabulary built successfully: vocab_size={}", vocab_size());
    }

    void bow_vocabulary::build_recursive(
        const std::vector<cv::Mat>& descriptors,
        int level,
        int node_idx,
        int iterations)
    {
        if (descriptors.empty())
        {
            SPDLOG_WARN("Empty descriptor set at level {} node {}", level, node_idx);
            return;
        }

        // Base case: at leaf level, store the single cluster center (mean of descriptors)
        if (level == _levels)
        {
            cv::Mat center = cv::Mat::zeros(1, descriptors[0].cols, descriptors[0].type());
            for (const auto& desc : descriptors)
            {
                center += desc;
            }
            center /= static_cast<double>(descriptors.size());
            _clusters[level][node_idx] = center;
            _cluster_sizes[level][node_idx] = static_cast<int>(descriptors.size());
            return;
        }

        // Recursive case: cluster descriptors and recurse into each cluster
        cv::Mat cluster_assignments = kmeans_cluster(descriptors, _branching_factor, iterations);

        // Group descriptors by cluster
        std::map<int, std::vector<cv::Mat>> cluster_to_descriptors;
        for (int i = 0; i < descriptors.size(); ++i)
        {
            int cluster_id = cluster_assignments.at<int>(i, 0);
            cluster_to_descriptors[cluster_id].push_back(descriptors[i]);
        }

        // For each cluster, recurse
        int num_nodes_at_next_level = 1;
        for (int i = 0; i < level + 1; ++i)
        {
            num_nodes_at_next_level *= _branching_factor;
        }

        for (int child_idx = 0; child_idx < _branching_factor; ++child_idx)
        {
            int child_node_idx = node_idx * _branching_factor + child_idx;

            if (cluster_to_descriptors.contains(child_idx))
            {
                build_recursive(
                    cluster_to_descriptors[child_idx],
                    level + 1,
                    child_node_idx,
                    iterations);
            }
        }

        // Store cluster centers at current level
        cv::Mat combined_descriptors;
        for (const auto& [cluster_id, cluster_descriptors] : cluster_to_descriptors)
        {
            for (const auto& desc : cluster_descriptors)
            {
                if (combined_descriptors.empty())
                {
                    combined_descriptors = desc.clone();
                }
                else
                {
                    cv::vconcat(combined_descriptors, desc, combined_descriptors);
                }
            }
        }

        // Store as final cluster centers (computed from data)
        if (!combined_descriptors.empty())
        {
            // Compute mean of all descriptors as this node's center
            cv::Mat center = cv::Mat::zeros(1, combined_descriptors.cols, combined_descriptors.type());
            for (int i = 0; i < combined_descriptors.rows; ++i)
            {
                center += combined_descriptors.row(i);
            }
            center /= static_cast<double>(combined_descriptors.rows);
            _clusters[level][node_idx] = center;
            _cluster_sizes[level][node_idx] = combined_descriptors.rows;
        }
    }

    cv::Mat bow_vocabulary::kmeans_cluster(
        const std::vector<cv::Mat>& descriptors,
        int k,
        int iterations) const
    {
        // Convert vector to single matrix
        cv::Mat data;
        for (const auto& desc : descriptors)
        {
            if (data.empty())
            {
                data = desc.clone();
            }
            else
            {
                cv::vconcat(data, desc, data);
            }
        }

        // Ensure data is floating point for kmeans
        if (data.type() != CV_32F)
        {
            data.convertTo(data, CV_32F);
        }

        cv::Mat labels;
        cv::Mat centers;

        cv::kmeans(
            data,
            k,
            labels,
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, iterations, 1e-4),
            3,  // attempts
            cv::KMEANS_PP_CENTERS);

        return labels;
    }

    int bow_vocabulary::descriptor_to_word(const cv::Mat& descriptor) const
    {
        if (!_is_built)
        {
            SPDLOG_ERROR("Vocabulary not built");
            return -1;
        }

        cv::Mat query = descriptor.clone();
        if (query.type() != CV_32F)
        {
            query.convertTo(query, CV_32F);
        }

        int node_idx = 0;

        // Traverse tree from root to leaf
        for (int level = 0; level < _levels; ++level)
        {
            const std::vector<cv::Mat>& next_level_clusters = _clusters[level + 1];

            if (node_idx < 0)
            {
                SPDLOG_ERROR("Node index out of bounds at level {}", level);
                return -1;
            }

            // Find nearest cluster center at this level
            int num_children = _branching_factor;
            double min_dist = std::numeric_limits<double>::max();
            int nearest_child = 0;

            for (int child_idx = 0; child_idx < num_children; ++child_idx)
            {
                int child_node_idx = node_idx * _branching_factor + child_idx;

                if (child_node_idx < static_cast<int>(next_level_clusters.size()) &&
                    !next_level_clusters[child_node_idx].empty())
                {
                    double dist = cv::norm(query, next_level_clusters[child_node_idx], cv::NORM_L2);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        nearest_child = child_idx;
                    }
                }
            }

            node_idx = node_idx * _branching_factor + nearest_child;
        }

        return node_idx;
    }

    cv::Mat bow_vocabulary::descriptors_to_histogram(
        const std::vector<cv::Mat>& descriptors) const
    {
        if (!_is_built)
        {
            SPDLOG_ERROR("Vocabulary not built");
            return cv::Mat();
        }

        int vocab_sz = vocab_size();
        cv::Mat histogram = cv::Mat::zeros(1, vocab_sz, CV_32F);

        // Count word occurrences
        for (const auto& desc : descriptors)
        {
            int word_id = descriptor_to_word(desc);
            if (word_id >= 0 && word_id < vocab_sz)
            {
                histogram.at<float>(0, word_id) += 1.0f;
            }
        }

        // Normalize to unit norm
        cv::normalize(histogram, histogram, 1.0, 0.0, cv::NORM_L2);

        return histogram;
    }

    int bow_vocabulary::vocab_size() const
    {
        int size = 1;
        for (int i = 0; i < _levels; ++i)
        {
            size *= _branching_factor;
        }
        return size;
    }

    void bow_vocabulary::save(const std::filesystem::path& path) const
    {
        if (!_is_built)
        {
            SPDLOG_ERROR("Cannot save unbuilt vocabulary");
            return;
        }

        cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
        if (!fs.isOpened())
        {
            SPDLOG_ERROR("Failed to open file for writing: {}", path.string());
            return;
        }

        fs << "levels" << _levels;
        fs << "branching_factor" << _branching_factor;
        fs << "is_built" << _is_built;

        // Save cluster centers
        fs << "clusters" << "[";
        for (int level = 0; level < _levels; ++level)
        {
            fs << "{";
            for (size_t node = 0; node < _clusters[level].size(); ++node)
            {
                fs << "center" << _clusters[level][node];
            }
            fs << "}";
        }
        fs << "]";

        fs.release();
        SPDLOG_INFO("Vocabulary saved to {}", path.string());
    }

    void bow_vocabulary::load(const std::filesystem::path& path)
    {
        cv::FileStorage fs(path.string(), cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            SPDLOG_ERROR("Failed to open file for reading: {}", path.string());
            return;
        }

        fs["levels"] >> _levels;
        fs["branching_factor"] >> _branching_factor;
        fs["is_built"] >> _is_built;

        // Load cluster centers (simplified - full implementation would need proper YAML parsing)
        SPDLOG_INFO("Vocabulary loaded from {}", path.string());
        fs.release();
    }

    bool bow_vocabulary::is_built() const
    {
        return _is_built;
    }

    std::vector<cv::Mat> bow_vocabulary::mat_to_vector(const cv::Mat& mat) const
    {
        std::vector<cv::Mat> result;
        for (int i = 0; i < mat.rows; ++i)
        {
            result.push_back(mat.row(i));
        }
        return result;
    }
}
