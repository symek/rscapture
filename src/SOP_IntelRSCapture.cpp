#include <UT/UT_DSOVersion.h>

#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_AutoLockInputs.h>
#include <SYS/SYS_Math.h>
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
        PRM_Name("distance",   "Distance threshold"),
};

PRM_Template
SOP_RSCapture::myTemplateList[] = {
        PRM_Template(PRM_FLT_J, 1, &names[1], PRMoneDefaults),
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
//    rs2::config config;
//    config.enable_all_streams();
//    pipe.start(config);
    pipe.start();

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
    flags().setTimeDep(1);

    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT) {
        return error();
    }


    const fpreal t = context.getTime();
    const float distance = evalFloat("distance", 0, t);

    while(frames.size() == 0 || refresh_frames) {
        frames = pipe.wait_for_frames();
        if(frames.size() != 0);
            refresh_frames = false;
    }


    rs2::depth_frame depth = frames.get_depth_frame();
    if (!depth) {
        addWarning(SOP_MESSAGE, "No depth frame");
        return error();
    }
    // Generate the pointcloud and texture mappings
    points = pointcloud.calculate(depth);
    if ( points.size() == 0) {
        addWarning(SOP_MESSAGE, "No points.");
        return error();
    }
    
    rs2::video_frame color = frames.get_color_frame();
    if (!color) {
        addWarning(SOP_MESSAGE, "No color frame");
        return error();
    }

    pointcloud.map_to(color);
    const int height = color.get_height();
    const int width  = color.get_width();
    auto* rgb_buff   = static_cast<const uint8_t*>(color.get_data());

    const rs2::vertex * vertices        = points.get_vertices();
    const rs2::texture_coordinate * uvs = points.get_texture_coordinates();

    gdp->clearAndDestroy();
    gdp->appendPointBlock(points.size());

    GA_RWHandleV3 cdh(gdp->addDiffuseAttribute(GA_ATTRIB_POINT));
    GA_RWHandleV3 uvh(gdp->addFloatTuple(GA_ATTRIB_POINT, "uv", 3));

    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
        const GA_Index index  = gdp->pointIndex(ptoff);
        const rs2::vertex & v = vertices[index];
        const rs2::texture_coordinate & uv = uvs[index];
        const int   x = static_cast<int>(SYSfloor(uv.u * width)*3);
        const int   y = static_cast<int>(SYSfloor(uv.v * height)*3);
        const auto buffer_index = y * width + x;
        if (buffer_index >= gdp->getNumPoints())
            continue;
        const float r = SYSpow((float)rgb_buff[buffer_index+0]/255.0f, 2.2f);
        const float g = SYSpow((float)rgb_buff[buffer_index+1]/255.0f, 2.2f);
        const float b = SYSpow((float)rgb_buff[buffer_index+2]/255.0f, 2.2f);
        const UT_Vector3 cdv(r, g, b);
        const UT_Vector3 uvv(uv.u, 1.f - uv.v, 0);
        const UT_Vector3 pos(v.x, v.y, v.z);
        gdp->setPos3(ptoff, pos);
        cdh.set(ptoff, cdv);
        uvh.set(ptoff, uvv);
    }

    {
        GA_FOR_ALL_PTOFF(gdp, ptoff) {
            const UT_Vector3 pos = gdp->getPos3(ptoff);
            if (pos.z() >= distance)
                gdp->destroyPointOffset(ptoff);
            }
    }

    gdp->getP()->bumpDataId();
    refresh_frames = true;
    

    return error();
}
