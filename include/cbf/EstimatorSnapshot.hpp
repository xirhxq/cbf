#ifndef CBF_TVT_ESTIMATOR_SNAPSHOT_HPP
#define CBF_TVT_ESTIMATOR_SNAPSHOT_HPP
#include "nlohmann/json.hpp"
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace tvt2026 {
struct EstimatedNode { int id; double x; double y; double epsilon; };
// Validate the entire packet before the caller applies any node. Array order
// is immaterial; the returned vector has exactly one entry per ID in ID order.
inline std::vector<EstimatedNode> parseEstimatorSnapshot(
    const nlohmann::json& packet, std::uint64_t frame, int n) {
    auto reject=[](const char* message) { throw std::runtime_error(message); };
    if (n <= 0 || !packet.is_object() || !packet.contains("frame_index")
        || !packet.at("frame_index").is_number_integer()
        || packet.at("frame_index") < 0
        || packet.at("frame_index").get<std::uint64_t>() != frame)
        reject("estimator snapshot frame missing, invalid, or mismatched");
    if (!packet.contains("robots") || !packet.at("robots").is_array()
        || packet.at("robots").size() != static_cast<std::size_t>(n))
        reject("estimator snapshot must contain all node IDs exactly once");
    std::vector<EstimatedNode> result(n);
    std::vector<bool> seen(n, false);
    for (const auto& entry : packet.at("robots")) {
        if (!entry.is_object() || !entry.contains("id")
            || !entry.at("id").is_number_integer()
            || entry.at("id") < 1 || entry.at("id") > n)
            reject("estimator snapshot ID must be an integer in 1..N");
        const int id=entry.at("id").get<int>();
        if (seen[id-1]) reject("estimator snapshot duplicate ID");
        if (!entry.contains("estimate") || !entry.at("estimate").is_array()
            || entry.at("estimate").size()!=2 || !entry.at("estimate")[0].is_number()
            || !entry.at("estimate")[1].is_number()
            || !entry.contains("epsilon") || !entry.at("epsilon").is_number())
            reject("estimator snapshot position/epsilon schema invalid");
        const double x=entry.at("estimate")[0].get<double>();
        const double y=entry.at("estimate")[1].get<double>();
        const double epsilon=entry.at("epsilon").get<double>();
        if (!std::isfinite(x) || !std::isfinite(y)
            || !std::isfinite(epsilon) || epsilon<=0)
            reject("estimator snapshot position/epsilon must be finite with positive epsilon");
        result[id-1]={id,x,y,epsilon}; seen[id-1]=true;
    }
    for (bool present : seen) if (!present) reject("estimator snapshot missing ID");
    return result;
}
inline void validateLocalSymmetricExecution(const nlohmann::json& config) {
    const auto e=config.value("tvt-experiment",nlohmann::json::object());
    if (e.value("row-mode", "legacy")!="symmetric-l1") return;
    const auto route=config.at("cbfs").value("route1",nlohmann::json::object());
    if (route.value("joint-resolve",false))
        throw std::invalid_argument("symmetric L1 local controller forbids joint fallback");
    if (config.at("execute").value("execution-mode", "distributed")!="distributed"
        || !route.value("on",false))
        throw std::invalid_argument("symmetric L1 requires per-node local execution");
    const std::string identity=e.value("controller-identity", "prototype-symmetric-l1");
    if (identity!="symmetric-l1-main-v1") return;
    if (e.value("information-contract", "")!="estimated-task-v1"
        || e.value("estimator-validation", "")!="complete-finite-common-snapshot-v2"
        || config.at("model")!="SingleIntegrate2D"
        || !e.value("refresh-before-solve",false)
        || !config.value("estimator-in-loop",nlohmann::json::object()).value("on",false))
        throw std::invalid_argument("main controller requires its complete common estimated snapshot interface");
    const auto hard=config.at("cbfs").at("without-slack");
    if (!hard.at("safety").at("on").get<bool>() || !hard.at("comm-fixed").at("on").get<bool>()
        || hard.at("energy").at("on").get<bool>() || hard.at("comm-auto").at("on").get<bool>()
        || hard.at("safety").value("consider-uncertainty",false)
        || hard.at("comm-fixed").value("consider-uncertainty",false)
        || config.at("cbfs").at("with-slack").at("target-yaw").at("on").get<bool>()
        || config.at("cbfs").at("uncertainty-rate").value("mode","off")!="off")
        throw std::invalid_argument("main controller requires both physical hard edge families and its audited task architecture");
}
} // namespace tvt2026
#endif
