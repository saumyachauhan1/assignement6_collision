#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "Assignment5.h" //GLView subclass instantiated to drive this simulation

int main(int argc, char* argv[])
{
    std::vector<std::string> args{ argv, argv + argc };
    int simStatus = 0;

    do
    {
        std::unique_ptr<Aftr::GLViewNewModule> glView(Aftr::GLViewNewModule::New(args));
        simStatus = glView->startWorldSimulationLoop();
    } while (simStatus != 0);

    std::cout << "Exited AftrBurner Engine Normally..." << std::endl;
    return 0;
}

