#pragma once 
#include <SOP/SOP_Node.h>

namespace rscapture {

class SOP_RSCapture : public SOP_Node
{
public:
         SOP_RSCapture(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_RSCapture();
    static PRM_Template      myTemplateList[];
    static OP_Node      *myConstructor(OP_Network*, const char *,
                                OP_Operator *);
protected:
    /// Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);
private:
    rs2::pointcloud pointcloud;
    rs2::points points;
    rs2::pipeline pipe;
};

} // End rscapture namespace

