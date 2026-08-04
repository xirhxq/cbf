#ifndef CBF_DIAGNOSTICS_EVIDENCE_STREAM_HPP
#define CBF_DIAGNOSTICS_EVIDENCE_STREAM_HPP

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace cbf2026::diagnostics {

inline void validateHardInteriorSelectionEvidence(
    const std::optional<std::string>& controllerSchemaVersion,
    const nlohmann::json& hardInteriorSelection
) {
    constexpr const char* malformed =
        "controller evidence interior selection is malformed";
    const std::set<std::string> publicFields = {
        "mode",
        "fraction",
        "cap_mps",
        "feasibility_tolerance_mps",
        "planar_chebyshev_radius_mps",
        "enforced_floor_mps"
    };
    const auto hasAllPublicFields = [&]() {
        return std::all_of(
            publicFields.begin(), publicFields.end(),
            [&](const std::string& field) {
                return hardInteriorSelection.contains(field);
            }
        );
    };
    if (!controllerSchemaVersion.has_value()) {
        if (!hardInteriorSelection.is_null()) {
            throw std::runtime_error(malformed);
        }
        return;
    }
    const std::string& schema = *controllerSchemaVersion;
    if (schema == "hard-interior-v2") {
        if (!hardInteriorSelection.is_object()
            || hardInteriorSelection.size() != publicFields.size()
            || hardInteriorSelection.contains("schema_version")
            || !hasAllPublicFields()) {
            throw std::runtime_error(malformed);
        }
        return;
    }
    if (schema == "hard-interior-v3") {
        const auto schemaIt = hardInteriorSelection.find("schema_version");
        if (!hardInteriorSelection.is_object()
            || hardInteriorSelection.size() != publicFields.size() + 1U
            || schemaIt == hardInteriorSelection.end()
            || !schemaIt->is_string()
            || schemaIt->get<std::string>() != "hard-interior-v3"
            || !hasAllPublicFields()) {
            throw std::runtime_error(malformed);
        }
        return;
    }
    throw std::runtime_error(malformed);
}

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
