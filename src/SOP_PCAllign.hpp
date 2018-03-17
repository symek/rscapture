#pragma once 
#include <SOP/SOP_Node.h>

namespace pcallign {

class SOP_PCAllign : public SOP_Node
{
public:
         SOP_PCAllign(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_PCAllign();
    static PRM_Template      myTemplateList[];
    static OP_Node      *myConstructor(OP_Network*, const char *,
                                OP_Operator *);
protected:
    /// Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);
private:
};

} // End pcallign namespace

