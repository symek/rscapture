#include <UT/UT_DSOVersion.h>

#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>

#include <librealsense2/rs.hpp>

#include "SOP_IntelRSCapture.hpp"

#include <unordered_map>
#include <memory>


using namespace rscapture;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "rscapture",
        "Realsense Capture",
        SOP_RSCapture::myConstructor,
        SOP_RSCapture::myTemplateList,
        0,
        0,
        0,
        OP_FLAG_GENERATOR));
}



static PRM_Name names[] = {
    // PRM_Name("model",   "Model"),
    // PRM_Name("term",    "RBF Term"),
    // PRM_Name("qcoef",   "Q (Smoothness)"),
    // PRM_Name("zcoef",   "Z (Deviation)"),
    // PRM_Name("radius",  "Radius"),
    // PRM_Name("layers",  "Layers"),
    // PRM_Name("lambda",  "Lambda"),
    // PRM_Name("tangent", "Tangent space"),
    // PRM_Name("morphspace","Blendshapes subspace"),
    // PRM_Name("maxedges",      "Max edges"),
    // PRM_Name("doclampweight", "Clamp weights"),
    // PRM_Name("weightrange", "Range"),
    // PRM_Name("dofalloff",   "Falloff"),
    // PRM_Name("falloffradius", "Falloff radius"),
    // PRM_Name("falloffrate", "Falloff rate (exponent)"),
};

PRM_Template
SOP_RSCapture::myTemplateList[] = {
    PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu, 0, 0, \
        SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),
    // PRM_Template(PRM_ORD,   1, &names[0], 0, &modelMenu, 0, 0, 0, 0, model_help),
    // PRM_Template(PRM_ORD,   1, &names[1], 0, &termMenu, 0, 0, 0, 0, term_help),
    // PRM_Template(PRM_FLT_J, 1, &names[2], PRMoneDefaults),
    // PRM_Template(PRM_FLT_J, 1, &names[3], PRMfiveDefaults),
    // PRM_Template(PRM_FLT_LOG,1,&names[4], PRMoneDefaults, 0, &radiusRange, 0, 0, 0, radius_help), // radius
    // PRM_Template(PRM_INT_J, 1, &names[9], PRMfourDefaults, 0, 0, 0, 0, 0, maxedges_help), // maxedges
    // PRM_Template(PRM_INT_J, 1, &names[5], PRMfourDefaults), // layers
    // PRM_Template(PRM_FLT_J, 1, &names[6], PRMpointOneDefaults), // lambda
    // PRM_Template(PRM_TOGGLE,1, &names[7], PRMzeroDefaults, 0, 0, 0, 0, 0, tangent_help), // tangent
    // PRM_Template(PRM_TOGGLE,1, &names[8], PRMzeroDefaults, 0, 0, 0, 0, 0, morphspace_help), // morphspace
    // PRM_Template(PRM_TOGGLE,1, &names[10], PRMzeroDefaults, 0, 0, 0, 0, 0, weightrange_help),
    // PRM_Template(PRM_FLT_J, 2, &names[11], ZeroOneDefaults, 0, 0, 0, 0, 0, weightrange_help),
    // PRM_Template(PRM_TOGGLE,1, &names[12], PRMzeroDefaults, 0, 0, 0, 0, 0),
    // PRM_Template(PRM_FLT_LOG,1,&names[13], PRMoneDefaults, 0, &radiusRange, 0, 0, 0, radius_help), // falloffradius
    // PRM_Template(PRM_FLT_J, 1, &names[14], PRMoneDefaults, 0, &falloffRange, 0, 0, 0, falloff_help), // falloff rate
    PRM_Template(),
};


OP_Node *
SOP_RSCapture::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_RSCapture(net, name, op);
}

SOP_RSCapture::SOP_RSCapture(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op)
{
    pipe.start();
    mySopFlags.setManagesDataIDs(true);
}

SOP_RSCapture::~SOP_RSCapture() {}

OP_ERROR
SOP_RSCapture::cookMySop(OP_Context &context)
{
 
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // auto color = frames.get_color_frame();

        const rs2::vertex * vertices = points.get_vertices();
        const size_t nverts = points.size();
        GA_Offset ptoff = gdp->appendPointBlock(nverts);
        
        GA_FOR_ALL_PTOFF(gdp, ptoff) {
            const GA_Index index  = gdp->pointIndex(ptoff);
            const rs2::vertex & v = vertices[index];
            const UT_Vector3 pos(v.x, v.y, v.z);
            gdp->setPos3(ptoff, pos);
        }



    

    gdp->getP()->bumpDataId();

    return error();
}
