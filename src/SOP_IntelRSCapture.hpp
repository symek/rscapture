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
    // void    getGroups(UT_String &str){ evalString(str, "group", 0, 0); }
    // void    MODEL(UT_String &str)    { evalString(str, "model", 0, 0); }
    // void    TERM(UT_String &str)     { evalString(str, "term", 0, 0); }
    // fpreal  QCOEF(fpreal t)          { return evalFloat("qcoef", 0, t); }
    // fpreal  ZCOEF(fpreal t)           { return evalFloat("zcoef", 0, t); }
    // fpreal  RADIUS(fpreal t)    { return evalFloat("radius", 0, t); }
    // int     LAYERS(fpreal t)    { return evalInt("layers", 0, t); }
    // fpreal  LAMBDA(fpreal t)    { return evalFloat("lambda", 0, t); }
    // int     TANGENT(fpreal t)   { return evalInt("tangent", 0, t); }
    // int     MAXEDGES(fpreal t)  { return evalInt("maxedges", 0, t); }
    // int     MORPHSPACE(fpreal t){ return evalInt("morphspace", 0, t); }
    // int     DOCLAMPWEIGHT(fpreal t){  return evalInt("doclampweight", 0, t); }
    // void    WEIGHTRANGE(fpreal t, UT_Vector2 & range) const {
    //     evalFloats("weightrange", range.data(), t);
    // }
    // int     DOFALLOFF(fpreal t)    { return evalInt("dofalloff", 0, t); }
    // fpreal  FALLOFFRADIUS(fpreal t)    { return evalFloat("falloffradius", 0, t); }
    // fpreal  FALLOFFRATE(fpreal t)  { return evalFloat("falloffrate", 0, t); }

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    // const GA_PointGroup *myGroup;

    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
};

} // End rscapture namespace

