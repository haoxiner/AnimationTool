#pragma once
#include <fbxsdk.h>
#include <memory>
#include <vector>
#include <map>
#include <string>
class FbxHelper
{
public:
    bool Startup();
    ~FbxHelper();
    bool LoadFBX(const std::string& filename);
    bool ExportAllFrames(const std::string& directory, const std::string& fileID);
    bool ExportAllFramesAsTexture(const std::string& directory, const std::string& fileID);
    bool ExportVertexSkinning(const std::string& directory, const std::string& fileID);
    bool ExportVertexSkinningAsTextureForFaceUnity(const std::string& directory, const std::string& fileID);
    
    bool ExportHierarchy(const std::string& directory, const std::string& fileID);
    bool ExportHierarchyAnimation(const std::string& directory, const std::string& fileID);

    bool ExportBlendshapeToObj(const std::string& directory, const std::string& fileID, bool noBlendshape = true);
private:
    void TraverseHierarchy(FbxNode* node, FbxNode* parent);
    void ConstructBoneMap();
    int GetNumOfMesh(FbxNode* node);
    void LogError(const std::string& errorMessage);
    void LogInfo(const std::string& message);
private:
    std::string hierarchyString_;
    std::string errorMessage_;
    FbxManager* fbxManager_ = nullptr;
    FbxScene* fbxScene_ = nullptr;
    // animation
    FbxTime durationPerFrame_;
    FbxTime startTime_;
    FbxTime stopTime_;
    FbxArray<FbxString*> animStackNameArray_;

    // collect bones
    std::map<std::string, int> boneMap_;
    std::map<int, std::vector<float>> boneAnimMap_;
};
struct Float3
{
    float x;
    float y;
    float z;
    Float3() {}
    Float3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
};