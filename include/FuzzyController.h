#ifndef __FuzzyController__
#define __FuzzyController__

#define FL_CPP11

#include<fl/Headers.h>

using namespace fl;
class FuzzyController
{
    
private:
    Engine* engine;
    InputVariable* center;
    InputVariable* left;
    InputVariable* right;
    InputVariable* up;
    InputVariable* down;
    OutputVariable* pitch;
    OutputVariable* yaw;
    
public:
    FuzzyController();
    void evaluate(float center,float left,float right,float up,float down, float& pitch,float& yaw);
    

    
};

#endif