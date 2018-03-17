#include <UT/UT_DSOVersion.h>

#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_AutoLockInputs.h>
#include <SYS/SYS_Math.h>
#include <ICP.h>
#include "SOP_PCAllign.hpp"


#include <unordered_map>
#include <memory>


using namespace pcallign;
typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "pcallign",
        "Point Cloud Allign",
        SOP_PCAllign::myConstructor,
        SOP_PCAllign::myTemplateList,
        2,
        2,
        0));
}

static PRM_Name names[] = {
    PRM_Name("model",   "Model"),
};

PRM_Template
SOP_PCAllign::myTemplateList[] = {
    PRM_Template(),
};


OP_Node *
SOP_PCAllign::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_PCAllign(net, name, op);
}

SOP_PCAllign::SOP_PCAllign(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    mySopFlags.setManagesDataIDs(true);  
}

SOP_PCAllign::~SOP_PCAllign() {}

OP_ERROR
SOP_PCAllign::cookMySop(OP_Context &context)
{
    flags().timeDep = 1;
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();

    fpreal t = context.getTime();
    duplicatePointSource(0, context);

    // Get rest and deform geometry:
    const GU_Detail *source_gdp = inputGeo(1);

    // Points count in control rig should match:
    if (source_gdp->getNumPoints() == 0 || gdp->getNumPoints() == 0) {
        addError(SOP_MESSAGE, "Needs two points clouds to allign.");
        return error();
    }

    // Eigen::MatrixXd target(3, gdp->getNumPoints());
    // Eigen::MatrixXd source(3, source_gdp->getNumPoints());
    Vertices target, source;
    target.resize(Eigen::NoChange, gdp->getNumPoints());
    source.resize(Eigen::NoChange, source_gdp->getNumPoints());

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff){
        const UT_Vector3 pos = gdp->getPos3(ptoff);
        const GA_Index   idx = gdp->pointIndex(ptoff);
        target(0, idx) = pos.x();
        target(1, idx) = pos.y();
        target(2, idx) = pos.z();
    }

    {
        GA_FOR_ALL_PTOFF(source_gdp, ptoff){
            const UT_Vector3 pos = source_gdp->getPos3(ptoff);
            const GA_Index   idx = source_gdp->pointIndex(ptoff);
            source(0, idx) = pos.x();
            source(1, idx) = pos.y();
            source(2, idx) = pos.z();
        }
    }

    SICP::Parameters pars;
    pars.p = .5;
    pars.max_icp = 15;
    pars.print_icpn = true;
    SICP::point_to_point(source, target, pars);

    ptoff = gdp->appendPointBlock(source_gdp->getNumPoints());
    
    for(size_t i=0; i<source_gdp->getNumPoints(); ++i, ++ptoff) {
        const UT_Vector3 pos(source(0, i), source(1, i), source(2, i));
        gdp->setPos3(ptoff, pos);
    }
    

    gdp->getP()->bumpDataId();
    return error();
}
