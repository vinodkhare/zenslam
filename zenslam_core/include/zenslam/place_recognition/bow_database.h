#pragma once

#include <filesystem>
#include <map>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "bow_vocabulary.h"

namespace zenslam
{
    /**
     * @brief Bag-of-Words database for place recognition
     *
     * Maintains inverted index of words to frames for fast similarity search.
     * Supports TF-IDF weighted queries for place recognition and loop closure detection.
     */
    class bow_database
    {
    public:
        /**
         * @brief Result of a similarity query
         */
        struct match_candidate
        {
            size_t frame_id;
            double bow_score;        // TF-IDF cosine similarity [0, 1]
            int matching_words;      // Number of matching words
        };

        /**
         * @brief Initialize database with pre-built vocabulary
         *
         * @param vocabulary Pre-built BoW vocabulary (must be already built)
         */
        explicit bow_database(const bow_vocabulary& vocabulary);

        /**
         * @brief Add a new frame to the database
         *
         * Computes BoW histogram and updates inverted index.
         *
         * @param frame_id Unique frame identifier
         * @param descriptors Vector of descriptors in this frame
         */
        void add_frame(size_t frame_id, const std::vector<cv::Mat>& descriptors);

        /**
         * @brief Query for similar frames (place recognition)
         *
         * Returns top N most similar frames based on TF-IDF cosine similarity.
         * Implements temporal consistency filtering: skips frames too close
         * in time (within min_temporal_distance).
         *
         * @param query_descriptors Descriptors from query frame
         * @param min_temporal_distance Skip frames within this distance (keyframes)
         * @param max_results Maximum number of results to return
         * @param min_score Minimum similarity score threshold [0, 1]
         * @return Sorted list of matches (highest score first)
         */
        [[nodiscard]] std::vector<match_candidate> query(
            const std::vector<cv::Mat>& query_descriptors,
            size_t min_temporal_distance = 10,
            size_t max_results = 5,
            double min_score = 0.01) const;

        /**
         * @brief Get frames matching a specific word
         *
         * Inverted index query: which frames contain this word?
         *
         * @param word_id Word ID from vocabulary
         * @return Vector of frame IDs containing this word
         */
        [[nodiscard]] std::vector<size_t> frames_with_word(int word_id) const;

        /**
         * @brief Get cached BoW histogram for a frame
         *
         * @param frame_id Frame ID
         * @return BoW histogram (vocab_size × 1), or empty if not found
         */
        [[nodiscard]] cv::Mat get_frame_histogram(size_t frame_id) const;

        /**
         * @brief Get total number of frames in database
         */
        [[nodiscard]] size_t frame_count() const;

        /**
         * @brief Clear all data
         */
        void clear();

        /**
         * @brief Save database to file
         *
         * Saves frame histograms and metadata (not vocabulary).
         * Vocabulary should be saved separately via bow_vocabulary::save().
         *
         * @param path Target directory (will create bow_database.yml, frame_histograms.yml)
         */
        void save(const std::filesystem::path& path) const;

        /**
         * @brief Load database from file
         *
         * @param path Source directory
         */
        void load(const std::filesystem::path& path);

    private:
        std::reference_wrapper<const bow_vocabulary> _vocabulary;

        // Frame histogram storage
        // frame_id → BoW histogram (vocab_size × 1, type CV_32F)
        std::map<size_t, cv::Mat> _frame_histograms;

        // Insertion order tracking (for temporal consistency)
        std::vector<size_t> _frame_order;

        // Inverted index: word_id → list of frame_ids containing this word
        // (for faster candidate filtering)
        std::map<int, std::vector<size_t>> _inverted_index;

        /**
         * @brief Compute TF-IDF weighted histogram from descriptors
         *
         * TF = term frequency (word count)
         * IDF = inverse document frequency (log(num_frames / frames_with_word))
         * Result normalized to unit L2 norm.
         *
         * @return Normalized TF-IDF histogram
         */
        [[nodiscard]] cv::Mat compute_tfidf_histogram(
            const std::vector<cv::Mat>& descriptors) const;

        /**
         * @brief Compute cosine similarity between two histograms
         *
         * @return Similarity score in [0, 1]
         */
        [[nodiscard]] double histogram_similarity(
            const cv::Mat& hist1,
            const cv::Mat& hist2) const;
    };
}
