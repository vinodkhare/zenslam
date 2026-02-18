#pragma once

#include <filesystem>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace zenslam
{
    /**
     * @brief Hierarchical k-means Bag-of-Words vocabulary
     *
     * Builds a vocabulary tree by recursively applying k-means clustering.
     * Each descriptor maps to a leaf node (word ID) in the tree.
     *
     * Tree structure:
     *   Level 0: Root (1 cluster)
     *   Level 1: k clusters
     *   Level 2: k² clusters
     *   ...
     *   Level L: k^L clusters (vocabulary size)
     *
     * Building: Offline (once per dataset)
     * Querying: Fast k-NN to leaf cluster center
     */
    class bow_vocabulary
    {
    public:
        explicit bow_vocabulary(int levels = 6, int branching_factor = 10);

        /**
         * @brief Build vocabulary from training descriptors
         *
         * Performs hierarchical k-means clustering to create vocabulary tree.
         * Training set should contain diverse descriptors (e.g., 100K+ descriptors
         * from varied scenes).
         *
         * @param training_descriptors Vector of training descriptor matrices
         * @param iterations K-means iterations per level (default 10)
         */
        void build(
            const std::vector<cv::Mat>& training_descriptors,
            int iterations = 10);

        /**
         * @brief Map a single descriptor to its word ID
         *
         * Traverses the vocabulary tree from root to leaf, choosing the
         * nearest cluster center at each level.
         *
         * @param descriptor Single descriptor (row vector)
         * @return Word ID (0 to vocab_size-1)
         */
        [[nodiscard]] int descriptor_to_word(const cv::Mat& descriptor) const;

        /**
         * @brief Convert vector of descriptors to word histogram
         *
         * Creates a normalized TF-IDF weighted histogram of word occurrences.
         * Result is a 1D vector of size vocab_size().
         *
         * @param descriptors Vector of descriptors
         * @return Histogram with weighted word counts (normalized to unit norm)
         */
        [[nodiscard]] cv::Mat descriptors_to_histogram(
            const std::vector<cv::Mat>& descriptors) const;

        /**
         * @brief Get vocabulary size (total number of words)
         * @return k^levels (leaf cluster count)
         */
        [[nodiscard]] int vocab_size() const;

        /**
         * @brief Save vocabulary to file
         * @param path Target file path (e.g., "data/bow_vocab.yml")
         */
        void save(const std::filesystem::path& path) const;

        /**
         * @brief Load vocabulary from file
         * @param path Source file path
         */
        void load(const std::filesystem::path& path);

        /**
         * @brief Check if vocabulary is built
         * @return True if build() was called successfully
         */
        [[nodiscard]] bool is_built() const;

    private:
        int _levels;           // Tree depth
        int _branching_factor; // k (clusters per node)
        bool _is_built = false;

        // Cluster centers at each level
        // _clusters[level][node_idx] = cluster center (cv::Mat)
        std::vector<std::vector<cv::Mat>> _clusters;

        // Cluster sizes for TF-IDF weighting
        std::vector<std::vector<int>> _cluster_sizes;

        /**
         * @brief Recursively build vocabulary tree
         *
         * @param descriptors Training descriptors for this subtree
         * @param level Current level (0 = root)
         * @param node_idx Index of current node at this level
         */
        void build_recursive(
            const std::vector<cv::Mat>& descriptors,
            int level,
            int node_idx,
            int iterations);

        /**
         * @brief K-means clustering with OpenCV
         *
         * Clusters descriptors into k clusters using cv::kmeans.
         *
         * @param descriptors Input descriptors
         * @param k Number of clusters
         * @param iterations Max iterations
         * @return Indices: cluster assignment for each descriptor
         */
        [[nodiscard]] cv::Mat kmeans_cluster(
            const std::vector<cv::Mat>& descriptors,
            int k,
            int iterations) const;

        /**
         * @brief Convert cv::Mat to std::vector for easier indexing
         */
        [[nodiscard]] std::vector<cv::Mat> mat_to_vector(const cv::Mat& mat) const;
    };
}
