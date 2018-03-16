#include <UT/UT_DSOVersion.h>

#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_AutoLockInputs.h>
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
    PRM_Name("model",   "Model"),
};

PRM_Template
SOP_RSCapture::myTemplateList[] = {
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
    mySopFlags.setManagesDataIDs(true);
    pipe.start();
    // Wait for the next set of frames from the camera
    for (int i=0; i<30; ++i) {
        rs2::frameset tmp = pipe.wait_for_frames();
    }
}

SOP_RSCapture::~SOP_RSCapture() {
    pipe.stop();
}

OP_ERROR
SOP_RSCapture::cookMySop(OP_Context &context)
{
    flags().timeDep = 1;
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();
    gdp->clearAndDestroy();


    // Flag the SOP as being time dependent (i.e. cook on time changes)
    // rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    // rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    // rs2::pipeline pipe;
    // pipe.start();
 
    

    rs2::frameset frames = pipe.wait_for_frames();
    const bool result = (frames.size() != 0);
    // const bool result = pipe.poll_for_frames(&frames);

    if (!result) {
        addWarning(SOP_MESSAGE, "No capture.");
        // pipe.stop();
        return error();
    }

    rs2::depth_frame depth = frames.get_depth_frame();
    if (!depth) {
        addWarning(SOP_MESSAGE, "No depth frame");
        pipe.stop();
        return error();
    }
    // Generate the pointcloud and texture mappings
    // points.reset();
    // rs2::pointcloud pc;
    points = pc.calculate(depth);
    rs2::video_frame color = frames.get_color_frame();
    if (!color) {
        addWarning(SOP_MESSAGE, "No depth frame");
        // pipe.stop();
        return error();
    }
    // pc.map_to(color);
    // const int height = color.get_height();
    // const int width  = color.get_width();
    // const uint8_t * rgb_buff = static_cast<const uint8_t*>(color.get_data());

    const rs2::vertex * vertices        = points.get_vertices();
    // const rs2::texture_coordinate * uvs = points.get_texture_coordinates();
    const size_t nverts = points.size();

    if (nverts == 0)
    {
        addWarning(SOP_MESSAGE, "No points.");
        // pipe.stop();
        return error();
    }
    GA_Offset ptoff = gdp->appendPointBlock(nverts);

    // GA_RWHandleV3 colorh(gdp->addDiffuseAttribute(GA_ATTRIB_POINT));

    GA_FOR_ALL_PTOFF(gdp, ptoff) {
        const GA_Index index  = gdp->pointIndex(ptoff);
        const rs2::vertex & v = vertices[index];
        // const uint8_t r       = rgb_buff[3*index+0];
        // const uint8_t g       = rgb_buff[3*index+1];
        // const uint8_t b       = rgb_buff[3*index+2];
        // const UT_Vector3 cd(r*1.0f/255.0f, g*1.0f/255.0f, b*1.0f/255.0f);
        const UT_Vector3 pos(v.x, v.y, v.z);
        gdp->setPos3(ptoff, pos);
        // colorh.set(ptoff, cd);
    }

    gdp->getP()->bumpDataId();
    

    return error();
}
