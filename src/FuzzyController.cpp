#include<FuzzyController.h>
#include<iostream>
#include <string>
#include <fstream>

using namespace std;
FuzzyController::FuzzyController(){
    std::string path ="ObstacleAvoidance_final.fis";
    
    stringstream ss;
    
    ifstream source(path.c_str());
    if (source.is_open()) {
        string line;
        while (source.good()) {
            std::getline(source, line);
            ss << line << "\n";
        }
        source.close();
    }
    
    
    
     engine = FisImporter().fromString(ss.str());
     std::string status;
     if (not engine->isReady(&status))
         throw Exception("Engine not ready. "
         "The following errors were encountered:\n" + status, FL_AT);
    center = engine->getInputVariable("C");
    left = engine->getInputVariable("L");
     right = engine->getInputVariable("R");
     up = engine->getInputVariable("U");
      down = engine->getInputVariable("D");
     pitch = engine->getOutputVariable("Pitch");
      yaw = engine->getOutputVariable("Yaw");
}


void FuzzyController::evaluate(float center, float left, float right, float up, float down, float& pitch, float& yaw)
{
    this->center->setInputValue(center);
    this->left->setInputValue(left);
    this->right->setInputValue(right);
    this->up->setInputValue(up);
    this->down->setInputValue(down);
    this->engine->process();
    pitch =(float) this->pitch->getOutputValue();
    yaw =(float) this->yaw->getOutputValue();

}