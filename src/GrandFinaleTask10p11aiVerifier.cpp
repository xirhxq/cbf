#include "grand_finale/Task10p11aiTerminalCertificate.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>

namespace {

using json=nlohmann::json;

// Numeric fields encode non-finite sentinels as JSON null.
double numberField(const json& object,const std::string& key) {
    if (!object.contains(key)||object.at(key).is_null())
        return std::numeric_limits<double>::quiet_NaN();
    return object.at(key).get<double>();
}

std::unique_ptr<gf::Task10p11rFixedBaselineFixture> makeP3(
    const json& manifest) {
    const auto initialization=gf::task10p11acInitializationFromManifest(
        manifest,"P3");
    auto scenario=gf::task10p11rFixedBaselineScenario();
    scenario.mobile_positions=initialization.positions;
    auto settings=gf::task10p11acSwarmSettings(
        scenario,initialization.velocities);
    auto fixture=gf::makeTask10p11rFixedBaselineFixture(
        std::move(scenario),std::move(settings),
        gf::GammaFeedbackSelectionMode::LeastIntervention,22.0);
    if (!fixture->adapter.initializeStageZero().initialized)
        throw std::runtime_error("P3 stage-zero initialization failed");
    return fixture;
}

// Step 1 of the verifier contract: the R0-family containment table must be
// present, complete over the frozen P constraint classes, and every entry
// must relax or drop in the superset direction.  The bound is only examined
// after this table passes.
bool verifyContainmentTable(const json& result,std::string& reason) {
    if (!result.contains("containment_table")) {
        reason="containment_table_absent";
        return false;
    }
    const auto& table=result.at("containment_table");
    const std::vector<std::string> required{
        "x0_full_rows_with_D(S0)_externals",
        "x1_same_frame_externals_D(S1(u0))",
        "x2_full_rows",
        "terminal_owner_local_gamma",
        "terminal_signed_transfer_interval",
        "terminal_native_D(S2)_current_rows",
        "terminal_native_successor_after_D(S2)",
        "component_control_box"};
    for (const auto& key:required) {
        if (!table.contains(key)) {
            reason="containment_table_missing:"+key;
            return false;
        }
        const auto& entry=table.at(key);
        if (!entry.contains("treatment")||!entry.contains("direction")) {
            reason="containment_entry_incomplete:"+key;
            return false;
        }
        const std::string treatment=entry.at("treatment").get<std::string>();
        const std::string direction=entry.at("direction").get<std::string>();
        const bool kept=treatment=="kept_exact";
        const bool relaxed=treatment=="relaxed_superset"||
            treatment=="dropped";
        if (!kept&&!relaxed) {
            reason="containment_treatment_unknown:"+key;
            return false;
        }
        if (kept&&direction!="equal") {
            reason="kept_entry_direction_not_equal:"+key;
            return false;
        }
        if (relaxed&&direction!="superset") {
            reason="relaxed_entry_direction_not_superset:"+key;
            return false;
        }
    }
    if (!table.contains("relative_domain_argument")||
        table.at("relative_domain_argument").value(
            "domain_covers_box_differences",false)!=true) {
        reason="relative_domain_argument_invalid";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc,char** argv) {
    if (argc!=5) {
        std::cerr<<"usage: GrandFinaleTask10p11aiVerifier PACKED_157P8 "
            "INIT_MANIFEST RESULT_JSON OUTPUT_JSON\n";
        return 2;
    }
    try {
        const auto protocol=gf::task10p11aiProtocol();
        const auto packed=gf::readTask10p11vJson(argv[1]);
        const auto manifest=gf::readTask10p11vJson(argv[2]);
        const auto result=gf::readTask10p11vJson(argv[3]);
        json verification={{"protocol","task10p11ai-verifier-v1"},
            {"result_valid",false},{"checks",json::array()}};
        auto check=[&](const std::string& name,bool passed,
                       const std::string& detail) {
            verification["checks"].push_back({{"check",name},
                {"passed",passed},{"detail",detail}});
            std::cout<<name<<" "<<(passed?"PASS":"FAIL")<<" "<<detail<<'\n';
            return passed;
        };
        auto fixture=makeP3(manifest);
        gf::restoreTask10p11vRestartState(*fixture,
            packed.at("restart_checkpoint"));
        if (!fixture->topologyFrozen())
            throw std::runtime_error("fixed topology identity failed");
        // Gate 1 (first by contract): containment table.
        std::string table_reason;
        const bool table_ok=verifyContainmentTable(result,table_reason);
        check("containment_table",table_ok,table_ok?"complete and "
            "superset-direction valid":table_reason);
        if (!table_ok) {
            gf::writeTask10p11vJson(argv[4],verification);
            return 4;
        }
        // Gate 2: anchor and limiting-row reproduction.
        gf::Task10p11ahComponentPlan frozen_plan;
        frozen_plan.owner2_u0=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u0=Eigen::Vector2d(4,-4);
        frozen_plan.owner2_u1=Eigen::Vector2d(-4,4);
        frozen_plan.owner9_u1=Eigen::Vector2d(4,-4);
        const auto anchor=gf::evaluateTask10p11ahTerminalRecoveryPlan(
            *fixture,std::nullopt,frozen_plan);
        bool anchor_ok=anchor.valid&&
            std::abs(anchor.terminal_native_successor_residual_mps2-
                protocol.anchor_successor_residual_mps2)<=
                protocol.anchor_tolerance_mps2;
        check("anchor_reproduction",anchor_ok,anchor_ok?"matches frozen "
            "10.11ah result":"anchor mismatch");
        const auto limiting=gf::task10p11aiIdentifyLimitingRow(*fixture,
            task10p11ahPlanVector(frozen_plan));
        const bool limiting_ok=
            result.contains("limiting_row")&&
            limiting.row_id==result.at("limiting_row").value("row_id",
                std::string());
        check("limiting_row_reproduction",limiting_ok,limiting.row_id);
        // Gate 3: per-chain certificate checks.
        bool chains_ok=true;
        bool any_strict=false;
        if (!result.contains("stage_a1_chains")||
            result.at("stage_a1_chains").empty()) {
            chains_ok=false;
            check("stage_a1_chains",false,"absent");
        }
        for (const auto& chain:result.value("stage_a1_chains",json::array())) {
            const std::string owner_a=std::to_string(
                chain.value("pair",json::object()).value("owner_a",0u));
            const std::string owner_b=std::to_string(
                chain.value("pair",json::object()).value("owner_b",0u));
            const bool collision=chain.value("pair",json::object()).value(
                "collision",true);
            const bool mobile_pair=chain.value("pair",json::object()).value(
                "mobile_pair",true);
            const bool solved=chain.value("solved",false);
            const bool global_optimal=chain.value("global_optimal",false);
            const double bound=numberField(chain,"global_bound_mps2");
            const double incumbent=numberField(chain,"incumbent_mps2");
            const double formula=numberField(chain,
                "independent_formula_mps2");
            const double canonical_value=numberField(chain,
                "canonical_builder_mps2");
            const bool strict=chain.value("strict",false);
            const std::string name=(collision?"collision:":"reference:")+
                owner_a+(mobile_pair?"--":"->")+owner_b;
            if (!solved||!global_optimal) {
                if (strict) {
                    chains_ok=false;
                    check("chain_"+name,false,"strict without global optimal");
                }
                continue;
            }
            const bool bound_agrees=std::abs(incumbent-formula)<=
                protocol.independent_agreement_mps2;
            chains_ok=chains_ok&&bound_agrees;
            check("chain_"+name+"_formula_agreement",bound_agrees,
                "incumbent vs closed-form");
            if (strict) {
                const bool negative=bound<protocol.strict_negative_bound_mps2;
                const bool canonical_ok=std::isfinite(canonical_value)&&
                    std::abs(canonical_value-formula)<=
                        protocol.independent_agreement_mps2;
                chains_ok=chains_ok&&negative&&canonical_ok;
                check("chain_"+name+"_strict_conditions",negative&&
                    canonical_ok,negative&&canonical_ok?"bound< -1e-8 and "
                    "canonical builder agrees":"strict conditions unmet");
                any_strict=any_strict||(negative&&canonical_ok);
            }
        }
        check("stage_a1_chains",chains_ok,any_strict?"strict certificate "
            "present":"no strict certificate (consistent with undetermined)");
        // Gate 4: classification consistency.
        const std::string classification=result.value("classification",
            std::string());
        bool classification_ok;
        if (classification=="strictly_infeasible")
            classification_ok=any_strict;
        else if (classification=="undetermined")
            classification_ok=!any_strict;
        else classification_ok=false;
        check("classification_consistency",classification_ok,classification);
        bool a3_ok=true;
        if (result.contains("stage_a3_interval_audit")) {
            const auto& audit=result.at("stage_a3_interval_audit");
            if (audit.contains("layers"))
                for (const auto& layer:audit.at("layers"))
                    if (classification=="strictly_infeasible")
                        a3_ok=false;
        }
        check("interval_audit_never_upgrades",a3_ok,"evidence-only role");
        const bool terminality=result.value("terminality",json::object()).
            value("no_followup_subtasks",false);
        check("terminality_recorded",terminality,"157.8 s problem closed");
        const bool all_ok=table_ok&&anchor_ok&&limiting_ok&&chains_ok&&
            classification_ok&&a3_ok&&terminality;
        verification["result_valid"]=all_ok;
        gf::writeTask10p11vJson(argv[4],verification);
        std::cout<<verification.dump(2)<<'\n';
        return all_ok?0:4;
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ai verifier failed: "<<error.what()<<'\n';
        return 4;
    }
}
