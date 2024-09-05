#pragma once

#include <string>
#include "Vector.h"  // Make sure this path is correct
#include "Mat4.h"    // Make sure this path is correct

class NetMsgBlock
{
public:
    NetMsgBlock();
    NetMsgBlock(const std::string& action, const Aftr::Vector& position, const Aftr::Mat4& displayMat);  // Corrected types
    virtual ~NetMsgBlock();

    std::string serialize() const;
    void deserialize(const std::string& data);

    std::string action;
    Aftr::Vector position;   // Corrected types
    Aftr::Mat4 displayMat;   // Corrected types
};
