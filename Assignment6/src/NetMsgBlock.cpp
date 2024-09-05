#include "NetMsgBlock.h"
#include "Vector.h"  // Include the header file where Vector is defined
#include "Mat4.h"    // Include the header file where Mat4 is defined
#include <sstream>

// Helper function to create an identity matrix
Aftr::Mat4 createIdentityMatrix()
{
    Aftr::Mat4 identity;
    identity[0] = 1; identity[1] = 0; identity[2] = 0; identity[3] = 0;
    identity[4] = 0; identity[5] = 1; identity[6] = 0; identity[7] = 0;
    identity[8] = 0; identity[9] = 0; identity[10] = 1; identity[11] = 0;
    identity[12] = 0; identity[13] = 0; identity[14] = 0; identity[15] = 1;
    return identity;
}

NetMsgBlock::NetMsgBlock()
    : action(""), position(Aftr::Vector(0, 0, 0)), displayMat(createIdentityMatrix()) {}

NetMsgBlock::NetMsgBlock(const std::string& action, const Aftr::Vector& position, const Aftr::Mat4& displayMat)
    : action(action), position(position), displayMat(displayMat) {}

NetMsgBlock::~NetMsgBlock() {}

std::string NetMsgBlock::serialize() const
{
    std::ostringstream oss;
    oss << action << " "
        << position.x << " " << position.y << " " << position.z << " "
        << displayMat[0] << " " << displayMat[1] << " " << displayMat[2] << " " << displayMat[3] << " "
        << displayMat[4] << " " << displayMat[5] << " " << displayMat[6] << " " << displayMat[7] << " "
        << displayMat[8] << " " << displayMat[9] << " " << displayMat[10] << " " << displayMat[11] << " "
        << displayMat[12] << " " << displayMat[13] << " " << displayMat[14] << " " << displayMat[15];
    return oss.str();
}

void NetMsgBlock::deserialize(const std::string& data)
{
    std::istringstream iss(data);
    iss >> action
        >> position.x >> position.y >> position.z
        >> displayMat[0] >> displayMat[1] >> displayMat[2] >> displayMat[3]
        >> displayMat[4] >> displayMat[5] >> displayMat[6] >> displayMat[7]
        >> displayMat[8] >> displayMat[9] >> displayMat[10] >> displayMat[11]
        >> displayMat[12] >> displayMat[13] >> displayMat[14] >> displayMat[15];
}
