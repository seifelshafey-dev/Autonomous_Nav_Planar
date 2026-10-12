/**
 * @file search_marker_observer.hpp
 * @author Seif ElShafey
 * @brief SearchObserver that records expansions for incremental publishing without blocking search.
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026
 *
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/planner/search_observer.hpp"

/**
 * @brief Records search expansions into a fixed buffer for a separate reader thread to drain.
 *
 * Construct with the maximum number of expansions the search may report (the planner's expansion
 * budget). onExpand() runs on the search thread and only appends; a reader (a publishing timer)
 * calls writeCount() and at(i) to read the [last_published, writeCount()) slice. Single-writer,
 * single-reader; one instance per plan() call.
 */
class SearchMarkerObserver : public SearchObserver {
public:
    /**
     * @brief Create an observer that records up to `capacity` expansions.
     * @param capacity Maximum number of expansions kept; later expansions are not recorded.
     */
    explicit SearchMarkerObserver(std::size_t capacity)
        : buffer_(capacity) {}

    void onExpand(const SearchExpansion & e) override {
        const std::size_t i = write_index_.load(std::memory_order_relaxed);
        if (i >= buffer_.size()) return;          // budget bound; never reallocate
        buffer_[i] = e;
        best_f_.store(e.f, std::memory_order_relaxed);
        write_index_.store(i + 1, std::memory_order_release);
    }

    /// @brief Number of expansions recorded so far.
    /// @return Count in `[0, capacity]`.
    std::size_t writeCount() const { return write_index_.load(std::memory_order_acquire); }

    /// @brief Access expansion at index @p i; caller must ensure i < writeCount().
    /// @param i Index in recording order.
    /// @return The i-th recorded expansion.
    const SearchExpansion & at(std::size_t i) const { return buffer_[i]; }

    /// @brief f-value of the most recent expansion recorded.
    /// @return The f-value, or infinity if nothing has been recorded.
    double bestF() const { return best_f_.load(std::memory_order_relaxed); }

private:
    std::vector<SearchExpansion> buffer_;
    std::atomic<std::size_t>     write_index_{0};
    std::atomic<double>          best_f_{std::numeric_limits<double>::infinity()};
};
