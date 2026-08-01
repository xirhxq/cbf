#ifndef CBF_DIAGNOSTICS_EVIDENCE_STREAM_HPP
#define CBF_DIAGNOSTICS_EVIDENCE_STREAM_HPP

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace cbf2026::diagnostics {

template<typename Witness>
class ExactResetWitnessStore {
public:
    void record(
        std::uint64_t frameIndex,
        int nodeId,
        Witness witness
    ) {
        const auto key = std::make_pair(frameIndex, nodeId);
        if (!witnesses_.emplace(key, std::move(witness)).second) {
            throw std::logic_error("reset QP witness is duplicated");
        }
    }

    const Witness& at(std::uint64_t frameIndex, int nodeId) const {
        return witnesses_.at(std::make_pair(frameIndex, nodeId));
    }

    void eraseFrame(std::uint64_t frameIndex) {
        auto iterator = witnesses_.lower_bound(
            std::make_pair(frameIndex, std::numeric_limits<int>::min())
        );
        while (iterator != witnesses_.end()
               && iterator->first.first == frameIndex) {
            iterator = witnesses_.erase(iterator);
        }
    }

private:
    std::map<std::pair<std::uint64_t, int>, Witness> witnesses_;
};

class EvidenceStream {
public:
    explicit EvidenceStream(std::ostream& output) : output_(output) {}

    void write(const nlohmann::json& row) {
        if (!row.is_object() || !finite(row)) {
            throw std::invalid_argument(
                "evidence row must be a finite JSON object"
            );
        }
        output_ << row.dump() << '\n';
        output_.flush();
        if (!output_) {
            throw std::runtime_error("evidence stream write failed");
        }
    }

    void flush() {
        output_.flush();
        if (!output_) {
            throw std::runtime_error("evidence stream flush failed");
        }
    }

private:
    static bool finite(const nlohmann::json& value) {
        if (value.is_discarded()) {
            return false;
        }
        if (value.is_number_float()) {
            return std::isfinite(value.get<double>());
        }
        if (value.is_array() || value.is_object()) {
            for (const auto& item : value) {
                if (!finite(item)) {
                    return false;
                }
            }
        }
        return true;
    }

    std::ostream& output_;
};

}

#endif
