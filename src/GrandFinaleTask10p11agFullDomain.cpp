#include "grand_finale/Task10p11agFullDomainPredecessor.hpp"

#include <iostream>

int main(int argc,char** argv) {
    if (argc!=3 && argc!=4) {
        std::cerr<<"usage: GrandFinaleTask10p11agFullDomain "
            "INPUT_PACKED OUTPUT_JSON [TIME_LIMIT_S]\n";
        return 2;
    }
    try {
        const auto snapshot=gf::readTask10p11vJson(argv[1]);
        const auto request=gf::task10p11s_capture_detail::requestFromJson(
            snapshot.at("canonical_request"));
        const auto distributed=gf::task10p11afAppliedControlsFromCheckpoint(
            snapshot);
        const double limit=argc==4?std::stod(argv[3]):12.0*60.0*60.0;
        auto result=gf::solveTask10p11agFullDomainPredecessor(
            snapshot,distributed,limit);
        auto output=gf::task10p11agFullDomainJson(
            result,request.mobile_ids);
        if (result.witness.solved) {
            const auto context=gf::task10p11ag_detail::makeContext(
                snapshot,gf::task10p11sOrderedControls(
                    request.mobile_ids,distributed));
            output["witness"]["current_all_1113_residuals"]=
                gf::task10p11ag_detail::residualsJson(
                    context.current_problem,result.witness.u0);
        }
        gf::writeTask10p11vJson(argv[2],output);
        std::cout<<output.dump(2)<<'\n';
        return result.witness_found?0:(result.valid?5:4);
    } catch (const std::exception& error) {
        std::cerr<<"Task 10.11ag full-domain oracle failed: "
                 <<error.what()<<'\n';
        return 4;
    }
}
