#include "FbxHelper.h"
#include <iostream>

int main(int argc, char* argv[])
{
    FbxHelper fbx;
    if (!fbx.Startup()) {
        return 1;
    }
    if (!fbx.LoadFBX("../../Resources/Model/prototype.FBX")) {
        std::cerr << "Fail To Open" << std::endl;
        return 1;
    }
    fbx.ExportKeyFrames("../../Resources/Model", "prototype");
    fbx.ExportVertexSkinning("../../Resources/Model/output/", "prototype");
    //fbx.ExportVertexSkinningAsTextureForFaceUnity("D:\\Development", "skinning");
    std::cerr << "Everything is ok" << std::endl;
    return 0;
}