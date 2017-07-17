#include "FbxHelper.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: animation.exe <fbx_file_path> <output_path>" << std::endl;
        return 0;
    }
    std::string inputFilePath(argv[1]);
    FbxHelper fbx;
    if (!fbx.Startup()) {
        return 1;
    }
    if (!fbx.LoadFBX(inputFilePath)) {
        std::cerr << "Fail To Open" << std::endl;
        return 1;
    }

    //fbx.ExportAllFramesInBoneSpace(argv[2], "");
    //fbx.ExportVertexSkinningAsTextureForFaceUnity(argv[2], "");

    //fbx.ExportAllFrames(argv[2], "anim");
    //fbx.ExportHierarchy(argv[2], "skeleton");
    //fbx.ExportHierarchyAnimation(argv[2], "animation");

    fbx.ExportAllFramesAsTexture(argv[2], "");
    fbx.ExportBlendshapeToObj(argv[2], "");
    return 0;
}