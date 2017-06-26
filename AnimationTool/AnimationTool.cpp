#include "FbxHelper.h"
#include <iostream>

int main(int argc, char* argv[])
{
    FbxHelper fbx;
    if (!fbx.Startup()) {
        return 1;
    }
    if (!fbx.LoadFBX("G:/wyx/xiaohai_bs_01_wb.fbx")) {
        std::cerr << "Fail To Open" << std::endl;
        return 1;
    }

    fbx.ExportBlendshapeToObj("G:/", "test");

    //fbx.ExportKeyFrames("../../Resources/Model", "run");
    //fbx.ExportVertexSkinning("../../Resources/Model", "prototype");
    //fbx.ExportVertexSkinningAsTextureForFaceUnity("D:\\Development", "skinning");
    //male.ExportHierarchy("G:/", "testAnim");

    //FbxHelper female;
    //if (!female.Startup()) {
    //    return 1;
    //}
    //if (!female.LoadFBX("G:/wyh/female.fbx")) {
    //    std::cerr << "Fail To Open" << std::endl;
    //    return 1;
    //}
    //female.ExportHierarchy("G:/", "female");
    //std::cerr << "Everything is ok" << std::endl;
    return 0;
}