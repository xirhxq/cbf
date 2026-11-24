#pragma once

#include "grand_finale/Types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace gf {

struct RouteBLeg {
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end = Eigen::Vector2d::Zero();
    std::pair<NodeId, NodeId> parents{0, 0};
    std::optional<NodeId> auxiliary;
    std::optional<NodeId> prerequest_next_parent;
    std::string kind;
};

struct RouteBMove {
    NodeId node = 0;
    std::vector<RouteBLeg> legs;
};

struct RouteBTriangulation {
    std::vector<NodeId> beacon_ids;
    std::map<NodeId, Eigen::Vector2d> beacons;
    std::vector<NodeId> mobile_ids;
    std::map<NodeId, Eigen::Vector2d> positions;
    std::map<NodeId, std::pair<NodeId, NodeId>> parents;
    std::vector<std::array<NodeId, 3>> faces;
    std::vector<std::array<NodeId, 3>> root_faces;
    std::map<NodeId, std::array<NodeId, 3>> support_face;
    double h_b_m = 100.0;
    double h_c_m = 140.0;
    double berth_spacing_m = 60.0;

    bool isBeacon(NodeId id) const { return beacons.count(id) != 0; }

    Eigen::Vector2d coord(NodeId id) const {
        const auto bit = beacons.find(id);
        if (bit != beacons.end()) return bit->second;
        return positions.at(id);
    }

    std::vector<DirectedEdge> initialTopologyEdges() const {
        std::vector<DirectedEdge> edges;
        for (NodeId nid : mobile_ids) {
            const auto& ps = parents.at(nid);
            edges.push_back({ps.first, nid});
            edges.push_back({ps.second, nid});
        }
        return edges;
    }
};

inline Eigen::Vector2d circumcenter2d(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c) {
    const double d =
        2.0 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
    if (std::abs(d) <= 1.0e-12) {
        throw std::invalid_argument("collinear triangle for circumcenter");
    }
    const double a2 = a.squaredNorm();
    const double b2 = b.squaredNorm();
    const double c2 = c.squaredNorm();
    const double ux =
        (a2 * (b.y() - c.y()) + b2 * (c.y() - a.y()) + c2 * (a.y() - b.y())) / d;
    const double uy =
        (a2 * (c.x() - b.x()) + b2 * (a.x() - c.x()) + c2 * (b.x() - a.x())) / d;
    return {ux, uy};
}

namespace route_b_planner_detail {

inline std::pair<std::pair<NodeId, NodeId>, NodeId> closestTwoAndThird(
    const Eigen::Vector2d& pt,
    const std::map<NodeId, Eigen::Vector2d>& beacons,
    const std::vector<NodeId>& beacon_ids) {
    if (beacon_ids.size() < 2) {
        throw std::invalid_argument("need at least 2 beacons");
    }
    std::vector<std::pair<double, NodeId>> ordered;
    for (NodeId bid : beacon_ids) {
        ordered.emplace_back((pt - beacons.at(bid)).norm(), bid);
    }
    std::sort(ordered.begin(), ordered.end(), [&](const auto& lhs, const auto& rhs) {
        if (std::abs(lhs.first - rhs.first) > 1.0e-9) return lhs.first < rhs.first;
        return beacons.at(lhs.second).x() < beacons.at(rhs.second).x();
    });
    const NodeId aux = ordered.size() >= 3 ? ordered[2].second : ordered[0].second;
    return {{ordered[0].second, ordered[1].second}, aux};
}

inline std::vector<NodeId> topologicalOrder(
    const std::map<NodeId, std::pair<NodeId, NodeId>>& parents,
    const std::map<NodeId, Eigen::Vector2d>& beacons) {
    std::map<NodeId, std::vector<NodeId>> children;
    std::map<NodeId, int> indeg;
    for (const auto& [u, ps] : parents) {
        children[u];
        int deg = 0;
        for (NodeId p : {ps.first, ps.second}) {
            if (beacons.count(p) == 0) {
                ++deg;
                children[p].push_back(u);
            }
        }
        indeg[u] = deg;
    }
    std::set<NodeId> ready;
    for (const auto& [u, d] : indeg) {
        if (d == 0) ready.insert(u);
    }
    std::vector<NodeId> order;
    while (!ready.empty()) {
        const NodeId u = *ready.begin();
        ready.erase(ready.begin());
        order.push_back(u);
        for (NodeId ch : children[u]) {
            if (--indeg[ch] == 0) ready.insert(ch);
        }
    }
    if (order.size() != parents.size()) {
        throw std::invalid_argument("parent DAG has a cycle");
    }
    return order;
}

inline std::set<NodeId> descendantsOf(
    const std::map<NodeId, std::pair<NodeId, NodeId>>& parents,
    NodeId root) {
    std::map<NodeId, std::vector<NodeId>> children;
    for (const auto& [u, ps] : parents) {
        children[ps.first].push_back(u);
        children[ps.second].push_back(u);
    }
    std::set<NodeId> desc;
    std::deque<NodeId> q;
    q.push_back(root);
    while (!q.empty()) {
        const NodeId cur = q.front();
        q.pop_front();
        const auto it = children.find(cur);
        if (it == children.end()) continue;
        for (NodeId ch : it->second) {
            if (desc.insert(ch).second) q.push_back(ch);
        }
    }
    return desc;
}

inline Eigen::Vector2d faceCircumcenter(
    const RouteBTriangulation& tri,
    const std::array<NodeId, 3>& f) {
    return circumcenter2d(tri.coord(f[0]), tri.coord(f[1]), tri.coord(f[2]));
}

inline std::vector<std::array<NodeId, 3>> dualFacePath(
    const RouteBTriangulation& tri,
    NodeId node) {
    const auto start_face = tri.support_face.at(node);
    const auto is_root = [&](const std::array<NodeId, 3>& f) {
        return std::find(tri.root_faces.begin(), tri.root_faces.end(), f) != tri.root_faces.end();
    };
    if (is_root(start_face)) return {start_face};

    std::set<NodeId> forbidden = descendantsOf(tri.parents, node);
    forbidden.insert(node);

    const std::size_t nf = tri.faces.size();
    std::vector<std::set<NodeId>> face_sets(nf);
    std::vector<Eigen::Vector2d> face_centers(nf);
    std::size_t start_idx = nf;
    for (std::size_t i = 0; i < nf; ++i) {
        face_sets[i] = {tri.faces[i][0], tri.faces[i][1], tri.faces[i][2]};
        face_centers[i] = faceCircumcenter(tri, tri.faces[i]);
        if (tri.faces[i] == start_face) start_idx = i;
    }
    if (start_idx == nf) throw std::invalid_argument("missing support face");

    std::vector<std::vector<std::size_t>> adj(nf);
    for (std::size_t a = 0; a < nf; ++a) {
        for (std::size_t b = a + 1; b < nf; ++b) {
            int shared = 0;
            for (NodeId v : face_sets[a]) {
                if (face_sets[b].count(v) != 0) ++shared;
            }
            if (shared == 2) {
                adj[a].push_back(b);
                adj[b].push_back(a);
            }
        }
    }
    for (auto& nb : adj) {
        std::sort(nb.begin(), nb.end(), [&](std::size_t lhs, std::size_t rhs) {
            const auto& cl = face_centers[lhs];
            const auto& cr = face_centers[rhs];
            if (std::abs(cl.y() - cr.y()) > 1.0e-9) return cl.y() < cr.y();
            return cl.x() < cr.x();
        });
    }

    std::deque<std::pair<std::size_t, std::vector<std::size_t>>> q;
    std::set<std::size_t> visited{start_idx};
    q.push_back({start_idx, {start_idx}});
    while (!q.empty()) {
        auto [cur, path] = std::move(q.front());
        q.pop_front();
        for (std::size_t nxt : adj[cur]) {
            if (visited.count(nxt) != 0) continue;
            bool touches_forbidden = false;
            for (NodeId v : face_sets[nxt]) {
                if (forbidden.count(v) != 0) {
                    touches_forbidden = true;
                    break;
                }
            }
            if (touches_forbidden) continue;
            auto new_path = path;
            new_path.push_back(nxt);
            if (is_root(tri.faces[nxt])) {
                std::vector<std::array<NodeId, 3>> out;
                for (std::size_t idx : new_path) out.push_back(tri.faces[idx]);
                return out;
            }
            visited.insert(nxt);
            q.push_back({nxt, std::move(new_path)});
        }
    }
    throw std::runtime_error("no non-descendant dual path to shore");
}

inline std::vector<double> corridorCuts(
    double x_start,
    double x_end,
    const std::vector<double>& beacon_xs) {
    const double lo = std::min(x_start, x_end);
    const double hi = std::max(x_start, x_end);
    std::vector<double> cuts;
    for (std::size_t i = 0; i + 2 < beacon_xs.size(); ++i) {
        const double mid_skip = 0.5 * (beacon_xs[i] + beacon_xs[i + 2]);
        if (lo + 1.0e-9 < mid_skip && mid_skip < hi - 1.0e-9) {
            cuts.push_back(mid_skip);
        }
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) {
        return std::abs(a - b) < 1.0e-9;
    }), cuts.end());
    if (x_start > x_end) std::reverse(cuts.begin(), cuts.end());
    cuts.push_back(x_end);
    return cuts;
}

inline std::vector<RouteBLeg> attachSymmetricPrerequests(std::vector<RouteBLeg> legs) {
    for (std::size_t i = 0; i + 1 < legs.size(); ++i) {
        const auto& cur_p = legs[i].parents;
        const auto& nxt_p = legs[i + 1].parents;
        std::vector<NodeId> incoming;
        for (NodeId np : {nxt_p.first, nxt_p.second}) {
            if (np != cur_p.first && np != cur_p.second) incoming.push_back(np);
        }
        if (incoming.size() > 1) {
            throw std::invalid_argument("handover changes more than one parent");
        }
        if (!incoming.empty()) {
            legs[i].prerequest_next_parent = incoming.front();
        }
    }
    return legs;
}

struct DualSpineResult {
    std::vector<RouteBLeg> legs;
    Eigen::Vector2d port;
    std::pair<NodeId, NodeId> root_beacons;
};

inline DualSpineResult dualSpineToPortLegs(
    const RouteBTriangulation& tri,
    NodeId node,
    double h_c) {
    const auto face_path = dualFacePath(tri, node);
    Eigen::Vector2d here = tri.positions.at(node);
    std::vector<RouteBLeg> legs;

    auto append_leg = [&](const Eigen::Vector2d& end,
                          const std::pair<NodeId, NodeId>& parents,
                          std::optional<NodeId> aux,
                          const std::string& kind) {
        if ((here - end).norm() > 1.0e-9) {
            legs.push_back({here, end, parents, aux, std::nullopt, kind});
            here = end;
        }
    };

    if (face_path.size() >= 2) {
        const auto& f_top = face_path[0];
        const auto& f_next = face_path[1];
        NodeId aux_top = 0;
        for (NodeId v : f_next) {
            if (v != f_top[0] && v != f_top[1] && v != f_top[2]) {
                aux_top = v;
                break;
            }
        }
        append_leg(faceCircumcenter(tri, f_top), tri.parents.at(node), aux_top, "tip");

        for (std::size_t idx = 0; idx + 1 < face_path.size(); ++idx) {
            const auto& f_cur = face_path[idx];
            const auto& f_nxt = face_path[idx + 1];
            std::vector<NodeId> shared;
            for (NodeId v : f_cur) {
                if (v == f_nxt[0] || v == f_nxt[1] || v == f_nxt[2]) shared.push_back(v);
            }
            NodeId aux = 0;
            for (NodeId v : f_nxt) {
                if (v != shared[0] && v != shared[1]) {
                    aux = v;
                    break;
                }
            }
            Eigen::Vector2d target_pt = faceCircumcenter(tri, f_nxt);
            if (idx == face_path.size() - 2 && target_pt.y() < h_c) {
                const double y_a = here.y();
                const double y_b = target_pt.y();
                if (y_a <= h_c) {
                    throw std::runtime_error("expected entry into root face above corridor");
                }
                const double frac = (h_c - y_a) / (y_b - y_a);
                target_pt = {here.x() + frac * (target_pt.x() - here.x()), h_c};
            }
            append_leg(target_pt, {shared[0], shared[1]}, aux, "bisector");
        }
    }

    const auto& root_face = face_path.back();
    std::vector<NodeId> root_beacons;
    for (NodeId v : root_face) {
        if (tri.isBeacon(v)) root_beacons.push_back(v);
    }
    std::sort(root_beacons.begin(), root_beacons.end(), [&](NodeId a, NodeId b) {
        return tri.beacons.at(a).x() < tri.beacons.at(b).x();
    });
    if (root_beacons.size() != 2) {
        throw std::runtime_error("root face must have exactly 2 coastal beacons");
    }
    const NodeId b_left = root_beacons[0];
    const NodeId b_right = root_beacons[1];
    const Eigen::Vector2d port(
        0.5 * (tri.beacons.at(b_left).x() + tri.beacons.at(b_right).x()), h_c);
    append_leg(port, {b_left, b_right}, std::nullopt, "root");
    return {legs, port, {b_left, b_right}};
}

inline std::vector<RouteBLeg> coastLegs(
    double x_start,
    double x_end,
    double h_c,
    const RouteBTriangulation& tri) {
    std::vector<double> beacon_xs;
    for (NodeId bid : tri.beacon_ids) beacon_xs.push_back(tri.beacons.at(bid).x());
    std::vector<RouteBLeg> legs;
    double cur_x = x_start;
    for (double nxt_x : corridorCuts(x_start, x_end, beacon_xs)) {
        if (std::abs(nxt_x - cur_x) <= 1.0e-9) continue;
        const double mid_x = 0.5 * (cur_x + nxt_x);
        const auto [parents, aux] = closestTwoAndThird(
            {mid_x, h_c}, tri.beacons, tri.beacon_ids);
        legs.push_back({{cur_x, h_c}, {nxt_x, h_c}, parents, aux, std::nullopt, "coast"});
        cur_x = nxt_x;
    }
    return legs;
}

inline RouteBMove dualWithdrawalMove(
    const RouteBTriangulation& tri,
    NodeId node,
    double h_c,
    const Eigen::Vector2d& berth_pos,
    const std::pair<NodeId, NodeId>& berth_parents) {
    auto spine = dualSpineToPortLegs(tri, node, h_c);
    auto legs = std::move(spine.legs);
    auto c_legs = coastLegs(spine.port.x(), berth_pos.x(), h_c, tri);
    legs.insert(legs.end(), c_legs.begin(), c_legs.end());
    const auto [_, dock_aux] = closestTwoAndThird(
        {berth_pos.x(), 0.5 * (h_c + berth_pos.y())}, tri.beacons, tri.beacon_ids);
    (void)_;
    const Eigen::Vector2d above_berth(berth_pos.x(), h_c);
    if ((above_berth - berth_pos).norm() > 1.0e-9) {
        legs.push_back({above_berth, berth_pos, berth_parents, dock_aux, std::nullopt, "dock"});
    }
    return {node, attachSymmetricPrerequests(std::move(legs))};
}

inline RouteBMove invertWithdrawalToInstallation(const RouteBMove& wd) {
    std::vector<RouteBLeg> rev;
    rev.reserve(wd.legs.size());
    for (auto it = wd.legs.rbegin(); it != wd.legs.rend(); ++it) {
        rev.push_back({it->end, it->start, it->parents, it->auxiliary, std::nullopt, it->kind});
    }
    return {wd.node, attachSymmetricPrerequests(std::move(rev))};
}

inline std::vector<std::size_t> hungarianMinAssignment(
    const std::vector<std::vector<double>>& cost) {
    const std::size_t n = cost.size();
    std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
    std::vector<std::size_t> p(n + 1, 0), way(n + 1, 0);
    for (std::size_t i = 1; i <= n; ++i) {
        p[0] = i;
        std::size_t j0 = 0;
        std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const std::size_t i0 = p[j0];
            double delta = std::numeric_limits<double>::infinity();
            std::size_t j1 = 0;
            for (std::size_t j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (std::size_t j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            const std::size_t j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }
    std::vector<std::size_t> ans(n, 0);
    for (std::size_t j = 1; j <= n; ++j) {
        ans[p[j] - 1] = j - 1;
    }
    return ans;
}

inline RouteBTriangulation relabelTriangulation(
    const RouteBTriangulation& tri,
    const std::map<NodeId, NodeId>& old_to_new) {
    auto map_v = [&](NodeId v) -> NodeId {
        if (tri.isBeacon(v)) return v;
        return old_to_new.at(v);
    };
    RouteBTriangulation out;
    out.beacon_ids = tri.beacon_ids;
    out.beacons = tri.beacons;
    out.h_b_m = tri.h_b_m;
    out.h_c_m = tri.h_c_m;
    out.berth_spacing_m = tri.berth_spacing_m;
    for (const auto& [old_id, pt] : tri.positions) {
        const NodeId nid = map_v(old_id);
        out.mobile_ids.push_back(nid);
        out.positions[nid] = pt;
        const auto& ps = tri.parents.at(old_id);
        out.parents[nid] = {map_v(ps.first), map_v(ps.second)};
        const auto& sf = tri.support_face.at(old_id);
        out.support_face[nid] = {map_v(sf[0]), map_v(sf[1]), map_v(sf[2])};
    }
    std::sort(out.mobile_ids.begin(), out.mobile_ids.end());
    for (const auto& f : tri.faces) {
        out.faces.push_back({map_v(f[0]), map_v(f[1]), map_v(f[2])});
    }
    for (const auto& f : tri.root_faces) {
        out.root_faces.push_back({map_v(f[0]), map_v(f[1]), map_v(f[2])});
    }
    return out;
}

}  // namespace route_b_planner_detail

inline RouteBTriangulation buildStaggeredTriangulation(
    const std::vector<double>& beacon_xs,
    const std::vector<std::vector<NodeId>>& units,
    double h1,
    double dy,
    double h_b_m = 100.0,
    double h_c_m = 140.0,
    double berth_spacing_m = 60.0) {
    if (beacon_xs.size() < 2 || units.size() + 1 != beacon_xs.size()) {
        throw std::invalid_argument("invalid staggered triangulation beacon/unit count");
    }
    RouteBTriangulation tri;
    tri.h_b_m = h_b_m;
    tri.h_c_m = h_c_m;
    tri.berth_spacing_m = berth_spacing_m;
    for (std::size_t i = 0; i < beacon_xs.size(); ++i) {
        const NodeId bid = static_cast<NodeId>(100 + i);
        tri.beacon_ids.push_back(bid);
        tri.beacons[bid] = {beacon_xs[i], 0.0};
    }
    for (std::size_t r = 0; r < units.size(); ++r) {
        const double span = beacon_xs[r + 1] - beacon_xs[r];
        if (!(span > 0.0 && h1 > 0.5 * span && dy > 0.0 && h1 * dy < 0.25 * span * span)) {
            throw std::invalid_argument("staggered triangulation violates acute conditions");
        }
        std::vector<NodeId> chain = {tri.beacon_ids[r], tri.beacon_ids[r + 1]};
        for (NodeId nid : units[r]) chain.push_back(nid);
        for (std::size_t j = 1; j <= units[r].size(); ++j) {
            const NodeId nid = units[r][j - 1];
            const double x = beacon_xs[r] + ((j % 2 == 1) ? 0.5 * span : span);
            const double y = h1 + static_cast<double>(j - 1) * dy;
            tri.mobile_ids.push_back(nid);
            tri.positions[nid] = {x, y};
            const std::pair<NodeId, NodeId> ps{chain[j - 1], chain[j]};
            tri.parents[nid] = ps;
            const std::array<NodeId, 3> face{chain[j - 1], chain[j], nid};
            tri.faces.push_back(face);
            tri.support_face[nid] = face;
            if (j == 1) tri.root_faces.push_back(face);
        }
    }
    std::sort(tri.mobile_ids.begin(), tri.mobile_ids.end());
    return tri;
}

inline RouteBTriangulation buildPinballTriangulation(
    int order_w,
    double span_s = 400.0,
    double h_b_m = 100.0,
    double h_c_m = 140.0,
    double x_offset = 0.0,
    double berth_spacing_m = 60.0) {
    if (order_w < 2) throw std::invalid_argument("Pinball order must be >= 2");
    RouteBTriangulation tri;
    tri.h_b_m = h_b_m;
    tri.h_c_m = h_c_m;
    tri.berth_spacing_m = berth_spacing_m;
    std::map<std::pair<int, int>, NodeId> grid_ids;
    for (int c = 0; c <= order_w; ++c) {
        const NodeId bid = static_cast<NodeId>(100 + c);
        tri.beacon_ids.push_back(bid);
        tri.beacons[bid] = {x_offset + static_cast<double>(c) * span_s, 0.0};
        grid_ids[{0, c}] = bid;
    }
    const double h_eq = 0.5 * std::sqrt(3.0) * span_s;
    NodeId next_id = 1;
    for (int r = 1; r <= order_w; ++r) {
        for (int c = 0; c <= order_w - r; ++c) {
            const NodeId nid = next_id++;
            grid_ids[{r, c}] = nid;
            tri.mobile_ids.push_back(nid);
            tri.positions[nid] = {
                x_offset + (static_cast<double>(c) + 0.5 * static_cast<double>(r)) * span_s,
                static_cast<double>(r) * h_eq};
            const NodeId p1 = grid_ids[{r - 1, c}];
            const NodeId p2 = grid_ids[{r - 1, c + 1}];
            tri.parents[nid] = {p1, p2};
            const std::array<NodeId, 3> face{p1, p2, nid};
            tri.faces.push_back(face);
            tri.support_face[nid] = face;
            if (r == 1) tri.root_faces.push_back(face);
        }
    }
    for (int r = 1; r < order_w; ++r) {
        for (int c = 0; c < order_w - r; ++c) {
            tri.faces.push_back({grid_ids[{r, c}], grid_ids[{r - 1, c + 1}], grid_ids[{r, c + 1}]});
        }
    }
    return tri;
}

inline std::set<NodeId> maximalSharedPrefix(
    const RouteBTriangulation& src,
    const RouteBTriangulation& dst) {
    const auto topo = route_b_planner_detail::topologicalOrder(src.parents, src.beacons);
    std::set<NodeId> prefix;
    for (NodeId u : topo) {
        if (dst.positions.count(u) == 0) continue;
        if ((src.positions.at(u) - dst.positions.at(u)).norm() > 1.0e-6) continue;
        const auto sp = src.parents.at(u);
        const auto dp = dst.parents.at(u);
        const std::set<NodeId> ss{sp.first, sp.second};
        const std::set<NodeId> ds{dp.first, dp.second};
        if (ss != ds) continue;
        bool ok = true;
        for (NodeId p : ss) {
            if (!src.isBeacon(p) && prefix.count(p) == 0) {
                ok = false;
                break;
            }
        }
        if (ok) prefix.insert(u);
    }
    return prefix;
}

inline std::vector<RouteBMove> planLabelledTransfer(
    const RouteBTriangulation& source_tri,
    const RouteBTriangulation& target_tri) {
    const auto prefix = maximalSharedPrefix(source_tri, target_tri);
    std::vector<NodeId> moving_ids;
    for (NodeId u : source_tri.mobile_ids) {
        if (prefix.count(u) == 0) moving_ids.push_back(u);
    }
    if (moving_ids.empty()) return {};

    std::vector<double> beacon_xs;
    for (NodeId bid : source_tri.beacon_ids) beacon_xs.push_back(source_tri.beacons.at(bid).x());
    double x_left = 0.5 * (beacon_xs[0] + beacon_xs[1]);
    double x_right = 0.5 * (beacon_xs[beacon_xs.size() - 2] + beacon_xs.back());
    double width = x_right - x_left;
    const double needed = static_cast<double>(moving_ids.size() - 1) * source_tri.berth_spacing_m;
    if (needed > width + 1.0e-9) {
        x_left = beacon_xs.front();
        x_right = beacon_xs.back();
        width = x_right - x_left;
    }
    const double x_start = 0.5 * (x_left + x_right) - 0.5 * needed;
    std::map<NodeId, Eigen::Vector2d> berth_pos;
    std::map<NodeId, std::pair<NodeId, NodeId>> berth_par;
    for (std::size_t idx = 0; idx < moving_ids.size(); ++idx) {
        const NodeId nid = moving_ids[idx];
        const double bx = x_start + static_cast<double>(idx) * source_tri.berth_spacing_m;
        berth_pos[nid] = {bx, source_tri.h_b_m};
        berth_par[nid] = route_b_planner_detail::closestTwoAndThird(
            berth_pos[nid], source_tri.beacons, source_tri.beacon_ids).first;
    }

    const auto src_topo = route_b_planner_detail::topologicalOrder(source_tri.parents, source_tri.beacons);
    const auto dst_topo = route_b_planner_detail::topologicalOrder(target_tri.parents, target_tri.beacons);
    std::vector<RouteBMove> moves;
    for (auto it = src_topo.rbegin(); it != src_topo.rend(); ++it) {
        if (prefix.count(*it) != 0) continue;
        moves.push_back(route_b_planner_detail::dualWithdrawalMove(
            source_tri, *it, source_tri.h_c_m, berth_pos.at(*it), berth_par.at(*it)));
    }
    for (NodeId nid : dst_topo) {
        if (prefix.count(nid) != 0) continue;
        moves.push_back(route_b_planner_detail::invertWithdrawalToInstallation(
            route_b_planner_detail::dualWithdrawalMove(
                target_tri, nid, target_tri.h_c_m, berth_pos.at(nid), berth_par.at(nid))));
    }
    return moves;
}

inline std::pair<std::vector<RouteBMove>, RouteBTriangulation> planDirectMatchedTransfer(
    const RouteBTriangulation& source_tri,
    const RouteBTriangulation& target_tri) {
    const auto src_topo = route_b_planner_detail::topologicalOrder(source_tri.parents, source_tri.beacons);
    const auto dst_topo = route_b_planner_detail::topologicalOrder(target_tri.parents, target_tri.beacons);

    std::map<NodeId, NodeId> src_to_dst;
    std::set<NodeId> used_dst;
    for (NodeId s_id : src_topo) {
        const auto& sp = source_tri.positions.at(s_id);
        NodeId matched_d = 0;
        for (NodeId d_id : dst_topo) {
            if (used_dst.count(d_id) != 0) continue;
            if ((sp - target_tri.positions.at(d_id)).norm() <= 1.0e-5) {
                matched_d = d_id;
                break;
            }
        }
        if (matched_d == 0) continue;
        const auto s_ps = source_tri.parents.at(s_id);
        const auto d_ps = target_tri.parents.at(matched_d);
        bool all_mapped = true;
        std::set<NodeId> mapped_s;
        for (NodeId p : {s_ps.first, s_ps.second}) {
            if (source_tri.isBeacon(p)) {
                mapped_s.insert(p);
            } else if (src_to_dst.count(p) != 0) {
                mapped_s.insert(src_to_dst.at(p));
            } else {
                all_mapped = false;
                break;
            }
        }
        const std::set<NodeId> d_set{d_ps.first, d_ps.second};
        if (all_mapped && mapped_s == d_set) {
            src_to_dst[s_id] = matched_d;
            used_dst.insert(matched_d);
        }
    }

    std::vector<NodeId> rem_src;
    for (auto it = src_topo.rbegin(); it != src_topo.rend(); ++it) {
        if (src_to_dst.count(*it) == 0) rem_src.push_back(*it);
    }
    std::vector<NodeId> rem_dst;
    for (NodeId d : dst_topo) {
        if (used_dst.count(d) == 0) rem_dst.push_back(d);
    }

    if (rem_src.empty()) {
        std::map<NodeId, NodeId> d2s;
        for (const auto& [s, d] : src_to_dst) d2s[d] = s;
        return {{}, route_b_planner_detail::relabelTriangulation(target_tri, d2s)};
    }

    std::map<NodeId, route_b_planner_detail::DualSpineResult> src_spines;
    std::map<NodeId, route_b_planner_detail::DualSpineResult> dst_spines;
    std::set<std::pair<NodeId, NodeId>> src_intervals, dst_intervals;
    for (NodeId u : rem_src) {
        src_spines[u] = route_b_planner_detail::dualSpineToPortLegs(source_tri, u, source_tri.h_c_m);
        src_intervals.insert(src_spines[u].root_beacons);
    }
    for (NodeId v : rem_dst) {
        dst_spines[v] = route_b_planner_detail::dualSpineToPortLegs(target_tri, v, target_tri.h_c_m);
        dst_intervals.insert(dst_spines[v].root_beacons);
    }

    std::stable_sort(rem_src.begin(), rem_src.end(), [&](NodeId a, NodeId b) {
        return source_tri.beacons.at(src_spines.at(a).root_beacons.first).x() <
               source_tri.beacons.at(src_spines.at(b).root_beacons.first).x();
    });

    auto legs_len = [](const std::vector<RouteBLeg>& ls) {
        double s = 0.0;
        for (const auto& l : ls) s += (l.end - l.start).norm();
        return s;
    };

    const std::size_t m = rem_src.size();
    std::vector<std::vector<double>> cost(m, std::vector<double>(m, 0.0));
    for (std::size_t i = 0; i < m; ++i) {
        const auto& sp_u = src_spines.at(rem_src[i]);
        const double lu = legs_len(sp_u.legs);
        for (std::size_t j = 0; j < m; ++j) {
            const auto& sp_v = dst_spines.at(rem_dst[j]);
            const double lv = legs_len(sp_v.legs);
            const double diff = static_cast<double>(i) - static_cast<double>(j);
            cost[i][j] = lu + std::abs(sp_u.port.x() - sp_v.port.x()) + lv + 1.0e-4 * diff * diff;
        }
    }
    const auto assign = route_b_planner_detail::hungarianMinAssignment(cost);
    std::map<NodeId, NodeId> dst_to_src;
    for (const auto& [s, d] : src_to_dst) dst_to_src[d] = s;
    for (std::size_t i = 0; i < m; ++i) {
        dst_to_src[rem_dst[assign[i]]] = rem_src[i];
    }
    const auto relabeled_dst = route_b_planner_detail::relabelTriangulation(target_tri, dst_to_src);

    bool disjoint = true;
    for (const auto& iv : src_intervals) {
        if (dst_intervals.count(iv) != 0) {
            disjoint = false;
            break;
        }
    }
    if (!disjoint) {
        return {planLabelledTransfer(source_tri, relabeled_dst), relabeled_dst};
    }

    std::vector<RouteBMove> moves;
    for (NodeId u : rem_src) {
        const auto& src_sp = src_spines.at(u);
        const auto dst_sp = route_b_planner_detail::dualSpineToPortLegs(
            relabeled_dst, u, source_tri.h_c_m);
        auto full_legs = src_sp.legs;
        const auto coast = route_b_planner_detail::coastLegs(
            src_sp.port.x(), dst_sp.port.x(), source_tri.h_c_m, source_tri);
        full_legs.insert(full_legs.end(), coast.begin(), coast.end());
        for (auto it = dst_sp.legs.rbegin(); it != dst_sp.legs.rend(); ++it) {
            full_legs.push_back({it->end, it->start, it->parents, it->auxiliary, std::nullopt, it->kind});
        }
        moves.push_back({u, route_b_planner_detail::attachSymmetricPrerequests(std::move(full_legs))});
    }
    return {moves, relabeled_dst};
}

}  // namespace gf
