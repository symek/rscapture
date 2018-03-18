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
    PRM_Name("usepenalty",   "Use penalty"),
    PRM_Name("p",            "P norm"),
    PRM_Name("mu",           "Penelty weight"),
    PRM_Name("alpha",        "Penalty Factor"),
    PRM_Name("maxmu",        "Max penalty"),
    PRM_Name("maxicp",       "Max ICP iteration"),
    PRM_Name("maxouter",     "Max outer iteration"),
    PRM_Name("maxinner",     "Max inner iteration"),
    PRM_Name("stop",         "Stopping criteria"),
};


static PRM_Default alphaDefault(1.2);
static PRM_Default maxmuDefault(1e5);
static PRM_Default maxicpDefault(100);
static PRM_Default stopDefault(1e-5);


PRM_Template
SOP_PCAllign::myTemplateList[] = {
    PRM_Template(PRM_TOGGLE,1, &names[0], PRMzeroDefaults),
    PRM_Template(PRM_FLT_J, 1, &names[1], PRMoneDefaults),
    PRM_Template(PRM_FLT_J, 1, &names[2], PRMtenDefaults),
    PRM_Template(PRM_FLT_J, 1, &names[3], &alphaDefault),
    PRM_Template(PRM_FLT_J, 1, &names[4], &maxmuDefault),
    PRM_Template(PRM_INT_J, 1, &names[5], &maxicpDefault),
    PRM_Template(PRM_INT_J, 1, &names[6], &maxicpDefault),
    PRM_Template(PRM_INT_J, 1, &names[7], PRMoneDefaults),
    PRM_Template(PRM_FLT_LOG, 1, &names[8], &stopDefault),
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

    // Get second geometry:
    const GU_Detail *source_gdp = inputGeo(1);

    // any points?:
    if (source_gdp->getNumPoints() == 0 || gdp->getNumPoints() == 0) {
        addError(SOP_MESSAGE, "Needs two points clouds to align.");
        return error();
    }

    const int   penalty = USE_PENALTY(t);
    const float p_norm  = P_NORM(t);   
    const float mu      = MU(t);         
    const float alpha   = ALPHA(t);         
    const float max_mu  = MAX_MU(t);         
    const int   max_icp = MAX_ICP(t);       
    const int   max_o   = MAX_OUTER(t);   
    const int   max_i   = MAX_INNER(t);   
    const float stop    = STOP(t);      
   
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

    SICP::Parameters parms;
    parms.use_penalty = static_cast<bool>(penalty);
    parms.p = p_norm;
    parms.mu = mu;
    parms.alpha = alpha;
    parms.max_mu = max_mu;
    parms.max_icp = max_icp;
    parms.max_outer = max_o;
    parms.max_inner = max_i;
    parms.stop = stop;
    SICP::point_to_point(source, target, parms);

    ptoff = gdp->appendPointBlock(source_gdp->getNumPoints());
    
    for(size_t i=0; i<source_gdp->getNumPoints(); ++i, ++ptoff) {
        const UT_Vector3 pos(source(0, i), source(1, i), source(2, i));
        gdp->setPos3(ptoff, pos);
    }
    

    gdp->getP()->bumpDataId();
    return error();
}
