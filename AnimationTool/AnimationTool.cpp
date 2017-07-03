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
    //fbx.ExportVertexSkinningAsTextureForFaceUnity(argv[2], "");
    fbx.ExportAllFramesAsTexture(argv[2], "");
    fbx.ExportBlendshapeToObj(argv[2] + std::string("/expression.json"), "", true);
    return 0;
}