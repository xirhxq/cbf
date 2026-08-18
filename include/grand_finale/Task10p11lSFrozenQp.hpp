#pragma once

#include "grand_finale/CanonicalHocbfQpController.hpp"
#include "optimisers/ConstraintRowScaling.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace gf {

struct FrozenCanonicalQp {
    double source_runtime_s = 0.0;
    CanonicalQpRequest request;
    double current_gamma_mps2 = 0.0;
    Eigen::Matrix2d raw_hessian = Eigen::Matrix2d::Zero();
    Eigen::Vector2d raw_gradient = Eigen::Vector2d::Zero();
    Eigen::MatrixXd raw_constraint_matrix;
    Eigen::VectorXd raw_lower_bound;
    Eigen::VectorXd raw_upper_bound;
    Eigen::VectorXd positive_row_scaling;
    Eigen::MatrixXd scaled_constraint_matrix;
    Eigen::VectorXd scaled_lower_bound;
    Eigen::VectorXd scaled_upper_bound;
    std::string snapshot_digest;
};

namespace task10p11l_s_detail {

inline CanonicalHardRow row(
    const char* id, CanonicalHardRowKind kind, NodeId peer,
    double nx, double ny, double cx, double cy, double constant,
    double responsibility, bool gamma, double h, double psi1, double hdot,
    double coefficient_reserve, double position_reserve,
    double velocity_reserve) {
    CanonicalHardRow result{
        id,kind,11,peer==0?std::optional<NodeId>{}:std::optional<NodeId>{peer},
        {nx,ny},{cx,cy},constant,responsibility,gamma,h,psi1,hdot,
        coefficient_reserve};
    if (position_reserve!=0.0 || velocity_reserve!=0.0) {
        result.tube_provenance=SnapshotTubeProvenance::CovarianceSigmaDevelopment;
    }
    result.position_uncertainty_reserve_m=position_reserve;
    result.velocity_uncertainty_reserve_mps=velocity_reserve;
    return result;
}

inline std::vector<CanonicalHardRow> frozenRows() {
    const double inf=std::numeric_limits<double>::infinity();
    return {
        row("collision:1--11:owner:11",CanonicalHardRowKind::Collision,1,
            0.67719495649423278,0.73580363610053889,
            0.67719495649423278,0.73580363610053889,
            117.74709173572879,0.5,true,225.57864289519307,
            230.5422967569001,4.9636538617070434,0.0058835735747985859,
            0.24527539957253719,0.056018454441270314),
        row("collision:10--11:owner:11",CanonicalHardRowKind::Collision,10,
            0.99999323540205132,0.0036781993064010786,
            0.99999323540205132,0.0036781993064010786,
            -4.0044164971685632,0.5,true,26.203312334987096,
            9.1346489429817801,-17.068663392005316,0.037409272656794883,
            0.2410081662584202,0.055161596818912001),
        row("collision:11--100:owner:11",CanonicalHardRowKind::Collision,100,
            0.52665760734769151,0.85007750506809965,
            0.52665760734769151,0.85007750506809965,
            284.93880340791327,1.0,true,268.38814283794318,
            276.66469827468131,8.2765554367381178,0.0024503035061410041,
            0.12063790153213227,0.027610073314444674),
        row("collision:11--101:owner:11",CanonicalHardRowKind::Collision,101,
            -0.014026814633374794,0.99990161939624889,
            -0.014026814633374794,0.99990161939624889,
            258.85290513660829,1.0,true,226.65670586707398,
            242.75624658365501,16.099540716581018,0.0028821636277109419,
            0.12063790153213227,0.027610073314444674),
        row("collision:11--102:owner:11",CanonicalHardRowKind::Collision,102,
            -0.54356970011743588,0.83936403372686919,
            -0.54356970011743588,0.83936403372686919,
            309.63784405612682,1.0,true,271.94297211444319,
            290.79161779652975,18.848645682086559,0.0024194224894783876,
            0.12063790153213227,0.027610073314444674),
        row("collision:11--12:owner:11",CanonicalHardRowKind::Collision,12,
            -0.99999566388433403,-0.0029448620561779329,
            -0.99999566388433403,-0.0029448620561779329,
            36.242507014638811,0.5,true,66.483304930792571,
            69.502128829486779,3.0188238986942126,0.017969349451688942,
            0.24372788571846482,0.055707307784374868),
        row("collision:11--13:owner:11",CanonicalHardRowKind::Collision,13,
            0.8357976935244209,-0.54903753560139978,
            0.8357976935244209,-0.54903753560139978,
            27.088443118743875,0.5,true,81.235609253000263,
            67.721359801391188,-13.514249451609071,0.015112056147183606,
            0.24438486714501087,0.055846300184721594),
        row("collision:11--14:owner:11",CanonicalHardRowKind::Collision,14,
            -0.65503426814184817,-0.75559917122762477,
            -0.65503426814184817,-0.75559917122762477,
            39.912934570083003,0.5,true,56.227004433418244,
            68.047259605682086,11.820255172263845,0.020822818889962683,
            0.24468113629399688,0.055915555043210122),
        row("collision:2--11:owner:11",CanonicalHardRowKind::Collision,2,
            0.33470637237870365,0.94232247361987953,
            0.33470637237870365,0.94232247361987953,
            98.463697135004352,0.5,true,168.41476032907892,
            182.67874253491539,14.263982205836479,0.0076652353715947084,
            0.24208622051094811,0.055373158101293028),
        row("collision:3--11:owner:11",CanonicalHardRowKind::Collision,3,
            0.018865788227752093,0.99982202517975449,
            0.018865788227752093,0.99982202517975449,
            101.12477830544067,0.5,true,156.16676957630239,
            179.21633779532914,23.04956821902676,0.0081747017372740746,
            0.24047450618341229,0.05505345379260479),
        row("collision:4--11:owner:11",CanonicalHardRowKind::Collision,4,
            -0.40653582006316008,0.91363484336225598,
            -0.40653582006316008,0.91363484336225598,
            87.577159421329924,0.5,true,152.94261802828609,
            164.0569289265226,11.114310898236498,0.0084604910496196683,
            0.24406481459087875,0.055765618446793355),
        row("collision:5--11:owner:11",CanonicalHardRowKind::Collision,5,
            0.75337447449539696,0.65759174354517613,
            0.75337447449539696,0.65759174354517613,
            85.64061577178056,0.5,true,175.1874743001583,
            173.24177338704501,-1.9457009131132912,0.0074204651853028549,
            0.24324152867393697,0.055601577093749195),
        row("collision:6--11:owner:11",CanonicalHardRowKind::Collision,6,
            0.33891910996515301,0.94081551693221366,
            0.33891910996515301,0.94081551693221366,
            65.562336817771069,0.5,true,109.31545342664222,
            120.2314546582612,10.916001231618974,0.011391127169013153,
            0.24074850609612536,0.055096692704544904),
        row("collision:7--11:owner:11",CanonicalHardRowKind::Collision,7,
            -0.15148361546905431,0.98845976865243412,
            -0.15148361546905431,0.98845976865243412,
            69.100549996809391,0.5,true,100.45764792897903,
            119.34164097259128,18.883993043612254,0.012267011292382191,
            0.24005022140480603,0.054957098196238709),
        row("collision:8--11:owner:11",CanonicalHardRowKind::Collision,8,
            -0.52168901643499577,0.85313572784820513,
            -0.52168901643499577,0.85313572784820513,
            48.76790439589751,0.5,true,78.720036573539403,
            88.143225066977635,9.4231884934382268,0.015302384310416701,
            0.24064776553261824,0.055077096717392929),
        row("collision:9--11:owner:11",CanonicalHardRowKind::Collision,9,
            0.99884952686168094,0.047954381324295275,
            0.99884952686168094,0.047954381324295275,
            44.126775772649339,0.5,true,126.01185696940249,
            107.14282056702885,-18.869036402373641,0.010116309678270076,
            0.24366944750185443,0.055691706533567101),
        row("input:11:ax:lower",CanonicalHardRowKind::InputBox,0,
            1,0,1,0,4,1,false,inf,inf,inf,0,0,0),
        row("input:11:ax:upper",CanonicalHardRowKind::InputBox,0,
            -1,0,-1,0,4,1,false,inf,inf,inf,0,0,0),
        row("input:11:ay:lower",CanonicalHardRowKind::InputBox,0,
            0,1,0,1,4,1,false,inf,inf,inf,0,0,0),
        row("input:11:ay:upper",CanonicalHardRowKind::InputBox,0,
            0,-1,0,-1,4,1,false,inf,inf,inf,0,0,0),
        row("reference:101->11:owner:11",CanonicalHardRowKind::ReferenceDistance,101,
            -0.014026814633374794,0.99990161939624889,
            0.014026814633374794,-0.99990161939624889,
            579.21571006450063,1,true,613.08201832986174,
            596.90641711347712,-16.175601216384567,0.0033599842507881176,
            0.14063790153213226,0.027610073314444674),
        row("reference:102->11:owner:11",CanonicalHardRowKind::ReferenceDistance,102,
            -0.54356970011743588,0.83936403372686919,
            0.54356970011743588,-0.83936403372686919,
            528.68130230826171,1,true,567.79575208249253,
            548.87439188996609,-18.921360192526468,0.0028205273824563823,
            0.14063790153213226,0.027610073314444674),
        row("speed:11",CanonicalHardRowKind::SpeedLimit,0,
            20.07681220458905,-31.995080628797286,
            20.07681220458905,-31.995080628797286,
            541.95307001918422,1,true,542.2654423402995,
            542.2654423402995,inf,0.31237232111522506,
            0,0.027610073314444674),
    };
}

inline FrozenCanonicalQp buildFrozenQp() {
    FrozenCanonicalQp frozen;
    frozen.source_runtime_s=4.3;
    frozen.request={SolverProfile::OpenSource,11,44,1,SupervisorMode::Search,
        {4.0,1.2080790599424951},4.0,frozenRows(),1.0e-7};
    frozen.current_gamma_mps2=0.01026924166524612;
    frozen.raw_hessian=2.0*Eigen::Matrix2d::Identity();
    frozen.raw_gradient=-2.0*frozen.request.nominal;
    const Eigen::Index count=static_cast<Eigen::Index>(frozen.request.rows.size());
    frozen.raw_constraint_matrix.resize(count,2);
    frozen.raw_lower_bound.resize(count);
    frozen.raw_upper_bound=Eigen::VectorXd::Constant(
        count,std::numeric_limits<double>::infinity());
    frozen.positive_row_scaling.resize(count);
    for (Eigen::Index index=0;index<count;++index) {
        const auto& hard=frozen.request.rows[static_cast<std::size_t>(index)];
        frozen.raw_constraint_matrix.row(index)=hard.control_coefficient.transpose();
        frozen.raw_lower_bound(index)=-hard.constant;
        const double scale=exactEquivalentConstraintRowScale(
            hard.control_coefficient,-hard.constant);
        frozen.positive_row_scaling(index)=scale;
    }
    frozen.scaled_constraint_matrix=
        frozen.positive_row_scaling.asDiagonal()*frozen.raw_constraint_matrix;
    frozen.scaled_lower_bound=
        frozen.positive_row_scaling.asDiagonal()*frozen.raw_lower_bound;
    frozen.scaled_upper_bound=frozen.raw_upper_bound;
    frozen.snapshot_digest="literal-owner11-t4p3-formal-10m-v1";
    return frozen;
}

}  // namespace task10p11l_s_detail

inline const FrozenCanonicalQp& task10p11lSOwner11FrozenQp() {
    static const FrozenCanonicalQp frozen=task10p11l_s_detail::buildFrozenQp();
    return frozen;
}

}  // namespace gf
