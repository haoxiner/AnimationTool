#include "FbxHelper.h"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: animation.exe 0|1 <fbx_file_path> <output_path>" << std::endl;
        return 0;
    }
    std::string inputFilePath(argv[2]);
    FbxHelper fbx;
    if (!fbx.Startup()) {
        return 1;
    }
    if (!fbx.LoadFBX(inputFilePath)) {
        std::cerr << "Fail To Open" << std::endl;
        return 1;
    }

    //fbx.ExportAllFramesInBoneSpace(argv[2], "");
    

    //fbx.ExportAllFrames(argv[2], "anim");
    //fbx.ExportHierarchy(argv[2], "skeleton");
    //fbx.ExportHierarchyAnimation(argv[2], "animation");

    //======================== FACEUNITY BEGIN ============================= //
    fbx.ExportVertexSkinningForFaceUnity(argv[3], "");
    // export frames and blendshape
    //fbx.ExportAllFramesAsTexture(argv[3], "", argv[1][0] == '1');
    fbx.ExportBlendshapeToObj(argv[3], "", false);
    //======================== FACEUNITY END ============================= //
    return 0;
}