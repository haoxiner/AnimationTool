#include "FbxHelper.h"
#include <iostream>

int main(int argc, char* argv[])
{
    FbxHelper fbx;
    if (!fbx.Startup()) {
        return 1;
    }
    if (!fbx.LoadFBX("D:\\Development\\test.FBX")) {
        std::cerr << "Fail To Open" << std::endl;
        return 1;
    }
    //fbx.ExportKeyFrames("D:\\Development\\anim.bin");
    fbx.ExportVertexSkinning("D:\\Development", "skinning");
    std::cerr << "Everything is ok" << std::endl;
    return 0;
}