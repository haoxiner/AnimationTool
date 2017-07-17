#include "FbxHelper.h"
#include "fbxsdk\core\math\fbxvector4.h"
#include "fbxsdk\scene\geometry\fbxblendshapechannel.h"
#include <iostream>
#include <algorithm>
#include <fstream>
#include <tuple>

void ComputeShapeDeformation(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer* pAnimLayer, FbxVector4* pVertexArray);
void ComputeBlenshapeAnimationWeight(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer* pAnimLayer, std::vector<double>& weights);

std::string FilterInvalidFileNameChar(const std::string& filename)
{
    std::string str = filename;
    std::string t;
    t.resize(9);
    t[0] = 0x5C;
    t[1] = 0x2F;
    t[2] = 0x3A;
    t[3] = 0x2A;
    t[4] = 0x3F;
    t[5] = 0x22;
    t[6] = 0x3C;
    t[7] = 0x3E;
    t[8] = 0x7C;
    int length = str.length();
    for (int i = 0; i< length; ++i) {
        if (str[i] <= 0x1F || str[i] == 0x7F || t.find(str[i]) != std::string::npos) {
            str[i] = 0x5F;
        }
    }
    return str;
}
/****************************************************************************************

Copyright (C) 2015 Autodesk, Inc.
All rights reserved.

Use of this software is subject to the terms of the Autodesk license agreement
provided at the time of installation or download, or which otherwise accompanies
this software in either electronic or hard copy form.

****************************************************************************************/
#ifndef _GET_POSITION_H
#define _GET_POSITION_H

FbxAMatrix GetGlobalPosition(FbxNode* pNode,
                             const FbxTime& pTime,
                             FbxPose* pPose = NULL,
                             FbxAMatrix* pParentGlobalPosition = NULL);
FbxAMatrix GetPoseMatrix(FbxPose* pPose,
                         int pNodeIndex);
FbxAMatrix GetGeometry(FbxNode* pNode);
//Compute the transform matrix that the cluster will transform the vertex.
void ComputeClusterDeformation(FbxAMatrix& pGlobalPosition,
                                      FbxMesh* pMesh,
                                      FbxCluster* pCluster,
                                      FbxAMatrix& pVertexTransformMatrix,
                                      FbxTime pTime,
                                      FbxPose* pPose)
{
    FbxCluster::ELinkMode lClusterMode = pCluster->GetLinkMode();

    FbxAMatrix lReferenceGlobalInitPosition;
    FbxAMatrix lReferenceGlobalCurrentPosition;
    FbxAMatrix lAssociateGlobalInitPosition;
    FbxAMatrix lAssociateGlobalCurrentPosition;
    FbxAMatrix lClusterGlobalInitPosition;
    FbxAMatrix lClusterGlobalCurrentPosition;

    FbxAMatrix lReferenceGeometry;
    FbxAMatrix lAssociateGeometry;
    FbxAMatrix lClusterGeometry;

    FbxAMatrix lClusterRelativeInitPosition;
    FbxAMatrix lClusterRelativeCurrentPositionInverse;

    if (lClusterMode == FbxCluster::eAdditive && pCluster->GetAssociateModel()) {
        std::cerr << "========================== 00000000000 ===========================" << std::endl;
        pCluster->GetTransformAssociateModelMatrix(lAssociateGlobalInitPosition);
        // Geometric transform of the model
        lAssociateGeometry = GetGeometry(pCluster->GetAssociateModel());
        lAssociateGlobalInitPosition *= lAssociateGeometry;
        lAssociateGlobalCurrentPosition = GetGlobalPosition(pCluster->GetAssociateModel(), pTime, pPose);

        pCluster->GetTransformMatrix(lReferenceGlobalInitPosition);
        // Multiply lReferenceGlobalInitPosition by Geometric Transformation
        lReferenceGeometry = GetGeometry(pMesh->GetNode());
        lReferenceGlobalInitPosition *= lReferenceGeometry;
        lReferenceGlobalCurrentPosition = pGlobalPosition;

        // Get the link initial global position and the link current global position.
        pCluster->GetTransformLinkMatrix(lClusterGlobalInitPosition);
        // Multiply lClusterGlobalInitPosition by Geometric Transformation
        lClusterGeometry = GetGeometry(pCluster->GetLink());
        lClusterGlobalInitPosition *= lClusterGeometry;
        lClusterGlobalCurrentPosition = GetGlobalPosition(pCluster->GetLink(), pTime, pPose);

        // Compute the shift of the link relative to the reference.
        //ModelM-1 * AssoM * AssoGX-1 * LinkGX * LinkM-1*ModelM
        pVertexTransformMatrix = lReferenceGlobalInitPosition.Inverse() * lAssociateGlobalInitPosition * lAssociateGlobalCurrentPosition.Inverse() *
            lClusterGlobalCurrentPosition * lClusterGlobalInitPosition.Inverse() * lReferenceGlobalInitPosition;
    } else {
        
        pCluster->GetTransformMatrix(lReferenceGlobalInitPosition);
        lReferenceGlobalCurrentPosition = pGlobalPosition;
        // Multiply lReferenceGlobalInitPosition by Geometric Transformation
        lReferenceGeometry = GetGeometry(pMesh->GetNode());
        lReferenceGlobalInitPosition *= lReferenceGeometry;

        // Get the link initial global position and the link current global position.
        pCluster->GetTransformLinkMatrix(lClusterGlobalInitPosition);
        lClusterGlobalCurrentPosition = GetGlobalPosition(pCluster->GetLink(), pTime, pPose);

        // Compute the initial position of the link relative to the reference.
        lClusterRelativeInitPosition = lClusterGlobalInitPosition.Inverse() * lReferenceGlobalInitPosition;

        // Compute the current position of the link relative to the reference.
        lClusterRelativeCurrentPositionInverse = lReferenceGlobalCurrentPosition.Inverse() * lClusterGlobalCurrentPosition;

        // Compute the shift of the link relative to the reference.
        pVertexTransformMatrix = lClusterRelativeCurrentPositionInverse * lClusterRelativeInitPosition;
    }
}
void ComputeClusterDeformation(FbxAMatrix& pGlobalPosition,
                               FbxMesh* pMesh,
                               FbxCluster* pCluster,
                               FbxAMatrix& pVertexTransformMatrix,
                               FbxTime pTime)
{
    FbxCluster::ELinkMode lClusterMode = pCluster->GetLinkMode();

    FbxAMatrix lReferenceGlobalInitPosition;
    FbxAMatrix lReferenceGlobalCurrentPosition;
    FbxAMatrix lAssociateGlobalInitPosition;
    FbxAMatrix lAssociateGlobalCurrentPosition;
    FbxAMatrix lClusterGlobalInitPosition;
    FbxAMatrix lClusterGlobalCurrentPosition;

    FbxAMatrix lReferenceGeometry;
    FbxAMatrix lAssociateGeometry;
    FbxAMatrix lClusterGeometry;

    FbxAMatrix lClusterRelativeInitPosition;
    FbxAMatrix lClusterRelativeCurrentPositionInverse;

    pCluster->GetTransformMatrix(lReferenceGlobalInitPosition);
    lReferenceGlobalCurrentPosition = pGlobalPosition;
    // Multiply lReferenceGlobalInitPosition by Geometric Transformation
    lReferenceGeometry = GetGeometry(pMesh->GetNode());
    lReferenceGlobalInitPosition *= lReferenceGeometry;

    // Get the link initial global position and the link current global position.
    pCluster->GetTransformLinkMatrix(lClusterGlobalInitPosition);
    //lClusterGlobalCurrentPosition = GetGlobalPosition(pCluster->GetLink(), pTime, pPose);

    // Compute the initial position of the link relative to the reference.
    lClusterRelativeInitPosition = lClusterGlobalInitPosition.Inverse() * lReferenceGlobalInitPosition;

    // Compute the current position of the link relative to the reference.
    lClusterRelativeCurrentPositionInverse = lReferenceGlobalCurrentPosition.Inverse() * lClusterGlobalCurrentPosition;

    // Compute the shift of the link relative to the reference.
    pVertexTransformMatrix = lClusterRelativeCurrentPositionInverse * lClusterRelativeInitPosition;
}
#endif // #ifndef _GET_POSITION_H

bool FbxHelper::Startup()
{
    //The first thing to do is to create the FBX Manager which is the object allocator for almost all the classes in the SDK
    fbxManager_ = FbxManager::Create();
    if (!fbxManager_) {
        LogError("Error: Unable to create FBX Manager!\n");
        return false;
    } else {
        LogInfo(std::string("Autodesk FBX SDK version ") + fbxManager_->GetVersion());
    }

    //Create an IOSettings object. This object holds all import/export settings.
    FbxIOSettings* ios = FbxIOSettings::Create(fbxManager_, IOSROOT);
    fbxManager_->SetIOSettings(ios);

    //Load plugins from the executable directory (optional)
    FbxString lPath = FbxGetApplicationDirectory();
    fbxManager_->LoadPluginsDirectory(lPath.Buffer());

    //Create an FBX scene. This object holds most objects imported/exported from/to files.
    fbxScene_ = FbxScene::Create(fbxManager_, "My Scene");
    if (!fbxScene_) {
        LogError("Error: Unable to create FBX scene!\n");
        return false;
    }
    return true;
}

FbxHelper::~FbxHelper()
{
    if (fbxManager_) {
        fbxManager_->Destroy();
    }
}

bool FbxHelper::LoadFBX(const std::string& filename)
{
    // Create the importer.
    int lFileFormat = -1;
    FbxImporter *lImporter = FbxImporter::Create(fbxManager_, "");
    if (!fbxManager_->GetIOPluginRegistry()->DetectReaderFileFormat(filename.c_str(), lFileFormat)) {
        // Unrecognizable file format. Try to fall back to FbxImporter::eFBX_BINARY
        lFileFormat = fbxManager_->GetIOPluginRegistry()->FindReaderIDByDescription("FBX binary (*.fbx)");;
    }
    // Initialize the importer by providing a filename.
    bool success = lImporter->Initialize(filename.c_str(), lFileFormat);
    success = success && lImporter->Import(fbxScene_);
    // Destroy the importer to release the file.
    lImporter->Destroy();
    lImporter = NULL;
    if (!success) {
        return false;
    }
    FbxAxisSystem SceneAxisSystem = fbxScene_->GetGlobalSettings().GetAxisSystem();
    FbxAxisSystem OurAxisSystem(FbxAxisSystem::eYAxis, FbxAxisSystem::eParityOdd, FbxAxisSystem::eRightHanded);
    if (SceneAxisSystem != OurAxisSystem) {
        OurAxisSystem.ConvertScene(fbxScene_);
    }
    // Convert Unit System to what is used in this example, if needed
    FbxSystemUnit SceneSystemUnit = fbxScene_->GetGlobalSettings().GetSystemUnit();
    if (SceneSystemUnit.GetScaleFactor() != 1.0) {
        //The unit in this example is centimeter.
        FbxSystemUnit::cm.ConvertScene(fbxScene_);
    }
    // Get the list of all the animation stack.
    fbxScene_->FillAnimStackNameArray(animStackNameArray_);
    // Convert mesh, NURBS and patch into triangle mesh
    FbxGeometryConverter lGeomConverter(fbxManager_);
    lGeomConverter.Triangulate(fbxScene_, true);
    // Initialize the frame period.
    durationPerFrame_.SetTime(0, 0, 0, 1, 0, fbxScene_->GetGlobalSettings().GetTimeMode());

    ConstructBoneMap();
    return true;
}

bool FbxHelper::ExportAllFrames(const std::string& directory, const std::string& fileID)
{
    boneAnimMap_.clear();
    const int numOfAnimStack = animStackNameArray_.GetCount();
    if (numOfAnimStack > 1) {
        LogError("Multiple anim stack is not supported");
        return false;
    }
    const int animIndex = 0;
    FbxAnimStack* animStack = fbxScene_->FindMember<FbxAnimStack>(animStackNameArray_[animIndex]->Buffer());
    if (!animStack) {
        LogError("Invalid anim stack");
        return false;
    }
    FbxAnimLayer* animLayer = animStack->GetMember<FbxAnimLayer>();
    fbxScene_->SetCurrentAnimationStack(animStack);
    FbxTakeInfo* takeInfo = fbxScene_->GetTakeInfo(*(animStackNameArray_[animIndex]));

    if (takeInfo) {
        startTime_ = takeInfo->mLocalTimeSpan.GetStart();
        stopTime_ = takeInfo->mLocalTimeSpan.GetStop();
        LogInfo("Load animation time info from TakeInfo");
    } else {
        // Take the time line value
        FbxTimeSpan lTimeLineTimeSpan;
        fbxScene_->GetGlobalSettings().GetTimelineDefaultTimeSpan(lTimeLineTimeSpan);
        startTime_ = lTimeLineTimeSpan.GetStart();
        stopTime_ = lTimeLineTimeSpan.GetStop();
        LogInfo("TakeInfo not found. Use DefaultTimeSpan");
    }
    LogInfo("StartTime:" + std::to_string(startTime_.GetMilliSeconds()) +
            "\nStopTime: " + std::to_string(stopTime_.GetMilliSeconds()) +
            "\nDuration per frame: " + std::to_string(durationPerFrame_.GetMilliSeconds()));

    int numOfFrame = 0;
    for (auto time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
        numOfFrame++;
    }
    LogInfo("Number of frame: " + std::to_string(numOfFrame));
    
    
    int numOfMesh = 0;
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            std::cout << "mesh: " << node->GetName() << std::endl;
            numOfMesh++;
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxAMatrix geometryOffset = GetGeometry(node);
            for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                float transformation[16] = { 0.0f };
                for (int i = 0; i < 16; i++) {
                    transformation[i] = (float)(((double*)&globalOffPosition)[i]);
                }
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            const bool hasVertexCache = mesh->GetDeformerCount(FbxDeformer::eVertexCache) &&
                (static_cast<FbxVertexCacheDeformer*>(mesh->GetDeformer(0, FbxDeformer::eVertexCache)))->Active.Get();
            if (hasVertexCache) {
                LogError("Vertex cache is not supported.");
                return false;
            }
            //const bool hasShape = mesh->GetShapeCount() > 0;
            //if (hasShape) {
            //    LogError("Blendshape is not supported.");
            //    return false;
            //}
            const bool hasSkin = mesh->GetDeformerCount(FbxDeformer::eSkin) > 0;
            if (!hasSkin) {
                LogInfo("Mesh is not skinning.");
                continue;
            }
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBone; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                        FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                        FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                        FbxAMatrix vertexTransformation;
                        ComputeClusterDeformation(globalOffPosition, mesh, bone, vertexTransformation, time, nullptr);
                        float animFrame[16] = { 0.0f };
                        for (int i = 0; i < 16; i++) {
                            animFrame[i] = (float)(((double*)&vertexTransformation)[i]);
                        }
                        // transpose
                        for (int i = 0; i < 4; i++) {
                            for (int j = i + 1; j < 4; j++) {
                                std::swap(animFrame[4 * i + j], animFrame[4 * j + i]);
                            }
                        }
                        auto iter = boneMap_.find(bone->GetLink()->GetName());
                        if (iter == boneMap_.end()) {
                            LogError("Unknown bone: " + iter->first);
                            return false;
                        }
                        if (boneAnimMap_[iter->second].size() < numOfFrame * 12) {
                            for (int i = 0; i < 12; i++) {
                                boneAnimMap_[iter->second].push_back(animFrame[i]);
                            }
                        }
                        //output.write(reinterpret_cast<char*>(animFrame), sizeof(float) * 12);
                    }
                }
            }
        }
    }
    //std::ofstream json(directory + "/" + FilterInvalidFileNameChar(fileID + ".frame.json"));
    //json << "{\n";
    //for (int i = 0; i < numOfFrame; i++) {
    //    if (i > 0) {
    //        json << ",\n";
    //    }
    //    json << "\"frame_" << i << "\": {\n";
    //    bool first = true;
    //    for (const auto& pair : boneMap_) {
    //        if (first) {
    //            first = false;
    //        } else {
    //            json << ",\n";
    //        }
    //        json << "    \"" << pair.first << "\": [";
    //        for (int j = 0; j < 12; j++) {
    //            if (j > 0) {
    //                json << ", ";
    //            }
    //            json << boneAnimMap_[pair.second][i * 12 + j];
    //        }
    //        json << "]";
    //    }
    //    json << "}";
    //}
    //json << "}";
    //json.close();

    std::ofstream output(directory + "/anim.bin", std::ios::binary);
    int count = 0;
    for (int i = 0; i < numOfFrame; i++) {
        for (const auto& pair : boneAnimMap_) {
            output.write((char*)&(pair.second[i * 12]), sizeof(float) * 12);
        }
    }
    output.close();
    LogInfo("Number of mesh: " + std::to_string(numOfMesh));
    return true;
}
static int InterpolationFlagToIndex(int flags)
{
    if ((flags & FbxAnimCurveDef::eInterpolationConstant) == FbxAnimCurveDef::eInterpolationConstant) return 1;
    if ((flags & FbxAnimCurveDef::eInterpolationLinear) == FbxAnimCurveDef::eInterpolationLinear) return 2;
    if ((flags & FbxAnimCurveDef::eInterpolationCubic) == FbxAnimCurveDef::eInterpolationCubic) return 3;
    return 0;
}

static int ConstantmodeFlagToIndex(int flags)
{
    if ((flags & FbxAnimCurveDef::eConstantStandard) == FbxAnimCurveDef::eConstantStandard) return 1;
    if ((flags & FbxAnimCurveDef::eConstantNext) == FbxAnimCurveDef::eConstantNext) return 2;
    return 0;
}

static int TangentmodeFlagToIndex(int flags)
{
    if ((flags & FbxAnimCurveDef::eTangentAuto) == FbxAnimCurveDef::eTangentAuto) return 1;
    if ((flags & FbxAnimCurveDef::eTangentAutoBreak) == FbxAnimCurveDef::eTangentAutoBreak) return 2;
    if ((flags & FbxAnimCurveDef::eTangentTCB) == FbxAnimCurveDef::eTangentTCB) return 3;
    if ((flags & FbxAnimCurveDef::eTangentUser) == FbxAnimCurveDef::eTangentUser) return 4;
    if ((flags & FbxAnimCurveDef::eTangentGenericBreak) == FbxAnimCurveDef::eTangentGenericBreak) return 5;
    if ((flags & FbxAnimCurveDef::eTangentBreak) == FbxAnimCurveDef::eTangentBreak) return 6;
    return 0;
}

static int TangentweightFlagToIndex(int flags)
{
    if ((flags & FbxAnimCurveDef::eWeightedNone) == FbxAnimCurveDef::eWeightedNone) return 1;
    if ((flags & FbxAnimCurveDef::eWeightedRight) == FbxAnimCurveDef::eWeightedRight) return 2;
    if ((flags & FbxAnimCurveDef::eWeightedNextLeft) == FbxAnimCurveDef::eWeightedNextLeft) return 3;
    return 0;
}

static int TangentVelocityFlagToIndex(int flags)
{
    if ((flags & FbxAnimCurveDef::eVelocityNone) == FbxAnimCurveDef::eVelocityNone) return 1;
    if ((flags & FbxAnimCurveDef::eVelocityRight) == FbxAnimCurveDef::eVelocityRight) return 2;
    if ((flags & FbxAnimCurveDef::eVelocityNextLeft) == FbxAnimCurveDef::eVelocityNextLeft) return 3;
    return 0;
}
static void DisplayCurveKeys(FbxAnimCurve* pCurve)
{
    static const char* interpolation[] = { "?", "constant", "linear", "cubic" };
    static const char* constantMode[] = { "?", "Standard", "Next" };
    static const char* cubicMode[] = { "?", "Auto", "Auto break", "Tcb", "User", "Break", "User break" };
    static const char* tangentWVMode[] = { "?", "None", "Right", "Next left" };

    FbxTime   lKeyTime;
    float   lKeyValue;
    char    lTimeString[256];
    FbxString lOutputString;
    int     lCount;

    int lKeyCount = pCurve->KeyGetCount();

    for (lCount = 0; lCount < lKeyCount; lCount++) {
        lKeyValue = static_cast<float>(pCurve->KeyGetValue(lCount));
        lKeyTime = pCurve->KeyGetTime(lCount);

        lOutputString = "            Key Time: ";
        lOutputString += lKeyTime.GetTimeString(lTimeString, FbxUShort(256));
        lOutputString += ".... Key Value: ";
        lOutputString += lKeyValue;
        lOutputString += " [ ";
        lOutputString += interpolation[InterpolationFlagToIndex(pCurve->KeyGetInterpolation(lCount))];
        if ((pCurve->KeyGetInterpolation(lCount)&FbxAnimCurveDef::eInterpolationConstant) == FbxAnimCurveDef::eInterpolationConstant) {
            lOutputString += " | ";
            lOutputString += constantMode[ConstantmodeFlagToIndex(pCurve->KeyGetConstantMode(lCount))];
        } else if ((pCurve->KeyGetInterpolation(lCount)&FbxAnimCurveDef::eInterpolationCubic) == FbxAnimCurveDef::eInterpolationCubic) {
            lOutputString += " | ";
            lOutputString += cubicMode[TangentmodeFlagToIndex(pCurve->KeyGetTangentMode(lCount))];
            lOutputString += " | ";
            lOutputString += tangentWVMode[TangentweightFlagToIndex(pCurve->KeyGet(lCount).GetTangentWeightMode())];
            lOutputString += " | ";
            lOutputString += tangentWVMode[TangentVelocityFlagToIndex(pCurve->KeyGet(lCount).GetTangentVelocityMode())];
        }
        lOutputString += " ]";
        lOutputString += "\n";
        FBXSDK_printf(lOutputString);
    }
}
bool FbxHelper::ExportAllFramesInBoneSpace(const std::string& directory, const std::string& fileID)
{
    const int numOfAnimStack = animStackNameArray_.GetCount();
    if (numOfAnimStack > 1) {
        LogError("Multiple anim stack is not supported");
        return false;
    }
    const int animIndex = 0;
    FbxAnimStack* animStack = fbxScene_->FindMember<FbxAnimStack>(animStackNameArray_[animIndex]->Buffer());
    if (!animStack) {
        LogError("Invalid anim stack");
        return false;
    }
    FbxAnimLayer* animLayer = animStack->GetMember<FbxAnimLayer>();
    fbxScene_->SetCurrentAnimationStack(animStack);

    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxAMatrix geometryOffset = GetGeometry(node);
            for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                float transformation[16] = { 0.0f };
                for (int i = 0; i < 16; i++) {
                    transformation[i] = (float)(((double*)&globalOffPosition)[i]);
                }
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            const bool hasVertexCache = mesh->GetDeformerCount(FbxDeformer::eVertexCache) &&
                (static_cast<FbxVertexCacheDeformer*>(mesh->GetDeformer(0, FbxDeformer::eVertexCache)))->Active.Get();
            if (hasVertexCache) {
                LogInfo("Vertex cache is not supported.");
                continue;
            }
            const bool hasSkin = mesh->GetDeformerCount(FbxDeformer::eSkin) > 0;
            if (!hasSkin) {
                LogInfo("Mesh is not skinning.");
                continue;
            }

            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBone; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    auto boneNode = bone->GetLink();
                    auto curve = boneNode->LclTranslation.GetCurve(animLayer);
                    if (curve) {
                        std::cerr << curve->KeyGetCount() << std::endl;
                    }
                }
            }
        }
    }
    return true;
}

bool FbxHelper::ExportAllFramesAsTexture(const std::string& directory, const std::string& fileID)
{
    const int numOfAnimStack = animStackNameArray_.GetCount();
    if (numOfAnimStack > 1) {
        LogError("Multiple anim stack is not supported");
        return false;
    }
    const int animIndex = 0;
    FbxAnimStack* animStack = fbxScene_->FindMember<FbxAnimStack>(animStackNameArray_[animIndex]->Buffer());
    if (!animStack) {
        LogError("Invalid anim stack");
        return false;
    }
    FbxAnimLayer* animLayer = animStack->GetMember<FbxAnimLayer>();
    fbxScene_->SetCurrentAnimationStack(animStack);
    FbxTakeInfo* takeInfo = fbxScene_->GetTakeInfo(*(animStackNameArray_[animIndex]));

    if (takeInfo) {
        startTime_ = takeInfo->mLocalTimeSpan.GetStart();
        stopTime_ = takeInfo->mLocalTimeSpan.GetStop();
        LogInfo("Load animation time info from TakeInfo");
    } else {
        // Take the time line value
        FbxTimeSpan lTimeLineTimeSpan;
        fbxScene_->GetGlobalSettings().GetTimelineDefaultTimeSpan(lTimeLineTimeSpan);
        startTime_ = lTimeLineTimeSpan.GetStart();
        stopTime_ = lTimeLineTimeSpan.GetStop();
        LogInfo("TakeInfo not found. Use DefaultTimeSpan");
    }
    LogInfo("StartTime:" + std::to_string(startTime_.GetMilliSeconds()) +
            "\nStopTime: " + std::to_string(stopTime_.GetMilliSeconds()) +
            "\nDuration per frame: " + std::to_string(durationPerFrame_.GetMilliSeconds()));

    int numOfFrame = 0;
    for (auto time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
        numOfFrame++;
    }
    LogInfo("Number of frame: " + std::to_string(numOfFrame));


    int numOfMesh = 0;
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            std::cout << "mesh: " << node->GetName() << std::endl;
            numOfMesh++;
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxAMatrix geometryOffset = GetGeometry(node);
            for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                float transformation[16] = { 0.0f };
                for (int i = 0; i < 16; i++) {
                    transformation[i] = (float)(((double*)&globalOffPosition)[i]);
                }
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            const bool hasVertexCache = mesh->GetDeformerCount(FbxDeformer::eVertexCache) &&
                (static_cast<FbxVertexCacheDeformer*>(mesh->GetDeformer(0, FbxDeformer::eVertexCache)))->Active.Get();
            if (hasVertexCache) {
                LogInfo("Vertex cache is not supported.");
                continue;
            }
            const bool hasSkin = mesh->GetDeformerCount(FbxDeformer::eSkin) > 0;
            if (!hasSkin) {
                LogInfo("Mesh is not skinning.");
                continue;
            }
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBone; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                        FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                        FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                        FbxAMatrix vertexTransformation;
                        ComputeClusterDeformation(globalOffPosition, mesh, bone, vertexTransformation, time, nullptr);
                        float animFrame[16] = { 0.0f };
                        for (int i = 0; i < 16; i++) {
                            animFrame[i] = (float)(((double*)&vertexTransformation)[i]);
                        }
                        // transpose
                        for (int i = 0; i < 4; i++) {
                            for (int j = i + 1; j < 4; j++) {
                                //std::swap(animFrame[4 * i + j], animFrame[4 * j + i]);
                                float tmp = animFrame[4 * i + j];
                                animFrame[4 * i + j] = animFrame[4 * j + i];
                                animFrame[4 * j + i] = tmp;
                            }
                        }
                        auto iter = boneMap_.find(bone->GetLink()->GetName());
                        if (iter == boneMap_.end()) {
                            LogError("Unknown bone: " + iter->first);
                            return false;
                        }
                        if (boneAnimMap_[iter->second].size() < numOfFrame * 12) {
                            for (int i = 0; i < 12; i++) {
                                boneAnimMap_[iter->second].push_back(animFrame[i]);
                            }
                        }
                        //output.write(reinterpret_cast<char*>(animFrame), sizeof(float) * 12);
                    }
                }
            }
        }
    }
    std::ofstream output(directory + "/anim.bin", std::ios::binary);
    /*for (const auto& pair : boneAnimMap_) {
        output.write(reinterpret_cast<const char*>(pair.second.data()), sizeof(float) * pair.second.size());
    }*/
    for (int i = 0; i < numOfFrame; i++) {
        for (const auto& pair : boneAnimMap_) {
            output.write((char*)&(pair.second[i * 12]), sizeof(float) * 12);
        }
    }
    output.close();
    std::ofstream outputJson(directory + "/anim.json");
    outputJson << "{\"frame_num\":" << numOfFrame << ",\"cluster_num\":" << boneAnimMap_.size() << ",\"anim_head\":1}";
    outputJson.close();
    return true;
}

bool FbxHelper::ExportVertexSkinning(const std::string& directory, const std::string& fileID)
{
    const int MAX_BONE_PER_VERTEX = 4;
    int meshID = 0;
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            std::vector<int> perVertexBoneIDList(numOfVertex * MAX_BONE_PER_VERTEX, 0);
            std::vector<float> perVertexBoneWeightList(numOfVertex * MAX_BONE_PER_VERTEX, 0);
            std::vector<std::vector<std::pair<int, float>>> vertexBoneIDAndWeightList(numOfVertex);
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBoneOfDeformer = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBoneOfDeformer; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    //std::cerr << bone->GetLink()->GetName() << std::endl;
                    int numOfRelatedVertex = bone->GetControlPointIndicesCount();
                    for (int i = 0; i < numOfRelatedVertex; i++) {
                        int indexOfVertex = bone->GetControlPointIndices()[i];
                        if (indexOfVertex >= numOfVertex) {
                            continue;
                        }
                        float boneWeight = static_cast<float>(bone->GetControlPointWeights()[i]);
                        if (boneWeight == 0.0f) {
                            continue;
                        }
                        auto iter = boneMap_.find(bone->GetLink()->GetName());
                        if (iter == boneMap_.end()) {
                            LogError("Unknown bone: " + iter->first);
                            return false;
                        }
                        vertexBoneIDAndWeightList[indexOfVertex].emplace_back(std::make_pair(iter->second, boneWeight));
                    }
                }
            }
            int maxNumOfBonePerVertex = 0;
            for (const auto& perVertexBoneIDAndWeight : vertexBoneIDAndWeightList) {
                if (perVertexBoneIDAndWeight.size() > maxNumOfBonePerVertex) {
                    maxNumOfBonePerVertex = perVertexBoneIDAndWeight.size();
                }
            }
            std::cerr << "max num of bone per vertex: " << maxNumOfBonePerVertex << std::endl;
            for (int i = 0; i < numOfVertex; i++) {
                std::sort(vertexBoneIDAndWeightList[i].begin(), vertexBoneIDAndWeightList[i].end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                    if (b.second < a.second) {
                        return true;
                    }
                    return false;
                });
            }
            for (int i = 0; i < numOfVertex; i++) {
                if (vertexBoneIDAndWeightList[i].size() < MAX_BONE_PER_VERTEX) {
                    for (int j = 0; j < vertexBoneIDAndWeightList[i].size(); j++) {
                        perVertexBoneIDList[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].first;
                        perVertexBoneWeightList[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].second;
                    }
                } else {
                    float totalWeight = 0.0f;
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        perVertexBoneIDList[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].first;
                        perVertexBoneWeightList[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].second;
                        totalWeight += perVertexBoneIDList[i * MAX_BONE_PER_VERTEX + j];
                    }
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        perVertexBoneIDList[i * MAX_BONE_PER_VERTEX + j] /= totalWeight;
                    }
                }
            }
            std::string filename = directory + "\\" + FilterInvalidFileNameChar(fileID + "_" + std::to_string(meshID) + "_" + node->GetName() + ".skin");
            std::ofstream output(filename, std::ios::binary);
            std::cerr << "output: " << filename << std::endl;
            std::cerr << "vertices: " << numOfVertex << std::endl;
            //output.write(reinterpret_cast<const char*>(&numOfVertex), sizeof(int));
            output.write(reinterpret_cast<char*>(perVertexBoneIDList.data()), sizeof(int) * perVertexBoneIDList.size());
            output.write(reinterpret_cast<char*>(perVertexBoneWeightList.data()), sizeof(float) * perVertexBoneWeightList.size());
            output.close();
            meshID++;
        }
    }
    
    return true;
}

bool FbxHelper::ExportVertexSkinningAsTextureForFaceUnity(const std::string& directory, const std::string& fileID)
{
    const int MAX_BONE_PER_VERTEX = 8;
    int meshID = 0;
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            std::vector<std::vector<std::pair<int, float>>> vertexBoneIDAndWeightList(numOfVertex);
            int totalBonesOfMesh = 0;
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
                totalBonesOfMesh += numOfBone;
                for (int boneIndex = 0; boneIndex < numOfBone; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    //std::cerr << bone->GetLink()->GetName() << std::endl;
                    int numOfRelatedVertex = bone->GetControlPointIndicesCount();
                    for (int i = 0; i < numOfRelatedVertex; i++) {
                        int indexOfVertex = bone->GetControlPointIndices()[i];
                        if (indexOfVertex >= numOfVertex) {
                            continue;
                        }
                        float boneWeight = static_cast<float>(bone->GetControlPointWeights()[i]);
                        if (boneWeight == 0.0f) {
                            continue;
                        }
                        vertexBoneIDAndWeightList[indexOfVertex].emplace_back(std::make_pair(boneIndex, boneWeight));
                    }
                }
            }
            int maxNumOfBonePerVertex = 0;
            for (const auto& perVertexBoneIDAndWeight : vertexBoneIDAndWeightList) {
                if (perVertexBoneIDAndWeight.size() > maxNumOfBonePerVertex) {
                    maxNumOfBonePerVertex = perVertexBoneIDAndWeight.size();
                }
            }
            std::cerr << "max num of bone per vertex: " << maxNumOfBonePerVertex << std::endl;
            for (int i = 0; i < numOfVertex; i++) {
                std::sort(vertexBoneIDAndWeightList[i].begin(), vertexBoneIDAndWeightList[i].end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                    if (b.second < a.second) {
                        return true;
                    }
                    return false;
                });
            }
            std::vector<float> perVertexBoneIDList0(numOfVertex * 4, 0);
            std::vector<float> perVertexBoneWeightList0(numOfVertex * 4, 0);
            std::vector<float> perVertexBoneIDList1(numOfVertex * 4, 0);
            std::vector<float> perVertexBoneWeightList1(numOfVertex * 4, 0);
            for (int i = 0; i < numOfVertex; i++) {
                if (vertexBoneIDAndWeightList[i].size() < MAX_BONE_PER_VERTEX) {
                    for (int j = 0; j < vertexBoneIDAndWeightList[i].size(); j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * 4 + j] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList0[i * 4 + j] = vertexBoneIDAndWeightList[i][j].second;
                        } else {
                            perVertexBoneIDList1[i * 4 + j - 4] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList1[i * 4 + j - 4] = vertexBoneIDAndWeightList[i][j].second;
                        }
                    }
                } else {
                    float totalWeight = 0.0f;
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * 4 + j] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList0[i * 4 + j] = vertexBoneIDAndWeightList[i][j].second;
                        } else {
                            perVertexBoneIDList1[i * 4 + j - 4] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList1[i * 4 + j - 4] = vertexBoneIDAndWeightList[i][j].second;
                        }
                        totalWeight += vertexBoneIDAndWeightList[i][j].second;
                    }
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * 4 + j] /= totalWeight;
                        } else {
                            perVertexBoneIDList1[i * 4 + j - 4] /= totalWeight;
                        }
                    }
                }
            }
            for (int i = 0; i < numOfVertex * 4; i++) {
                perVertexBoneIDList0[i] += 0.5f;
                perVertexBoneIDList0[i] /= totalBonesOfMesh;
                perVertexBoneIDList1[i] += 0.5f;
                perVertexBoneIDList1[i] /= totalBonesOfMesh;
            }
            std::cerr << "TOTAL BONE: " << totalBonesOfMesh << std::endl;
            std::string filename = directory + "\\" + FilterInvalidFileNameChar(std::to_string(meshID) + "_" + node->GetName());
            std::ofstream output(filename, std::ios::binary);
            std::cerr << "output: " << filename << std::endl;
            output.write(reinterpret_cast<const char*>(&numOfVertex), sizeof(int));
            output.write(reinterpret_cast<char*>(perVertexBoneIDList0.data()), sizeof(float) * perVertexBoneIDList0.size());
            output.write(reinterpret_cast<char*>(perVertexBoneIDList1.data()), sizeof(float) * perVertexBoneIDList1.size());
            output.write(reinterpret_cast<char*>(perVertexBoneWeightList0.data()), sizeof(float) * perVertexBoneWeightList0.size());
            output.write(reinterpret_cast<char*>(perVertexBoneWeightList1.data()), sizeof(float) * perVertexBoneWeightList1.size());
            output.close();
            meshID++;
        }
    }

    return true;
}

bool FbxHelper::ExportHierarchy(const std::string& directory, const std::string& fileID)
{
    std::ofstream file(directory + "/" + fileID + ".json");
    file << hierarchyString_;
    file.close();

    std::ofstream output(directory + "/" + "skeleton_tree", std::ios::binary);
    output.write(reinterpret_cast<char*>(hierarchyArray_.data()), sizeof(float) * hierarchyArray_.size());
    return true;
}

bool FbxHelper::ExportHierarchyAnimation(const std::string& directory, const std::string& fileID)
{
    boneAnimMap_.clear();
    // animation
    FbxAnimStack * lCurrentAnimationStack = fbxScene_->FindMember<FbxAnimStack>(animStackNameArray_[0]->Buffer());
    auto animLayer = lCurrentAnimationStack->GetMember<FbxAnimLayer>();
    fbxScene_->SetCurrentAnimationStack(lCurrentAnimationStack);

    FbxTakeInfo* lCurrentTakeInfo = fbxScene_->GetTakeInfo(*(animStackNameArray_[0]));
    if (lCurrentTakeInfo) {
        startTime_ = lCurrentTakeInfo->mLocalTimeSpan.GetStart();
        stopTime_ = lCurrentTakeInfo->mLocalTimeSpan.GetStop();
    } else {
        // Take the time line value
        FbxTimeSpan lTimeLineTimeSpan;
        fbxScene_->GetGlobalSettings().GetTimelineDefaultTimeSpan(lTimeLineTimeSpan);

        startTime_ = lTimeLineTimeSpan.GetStart();
        stopTime_ = lTimeLineTimeSpan.GetStop();
    }

    int numOfFrame = 0;
    for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
        numOfFrame++;
    }
    std::cerr << "Num of frames: " << numOfFrame << std::endl;
    
    for (int nodeIdx = 0; nodeIdx < fbxScene_->GetNodeCount(); nodeIdx++) {
        auto node = fbxScene_->GetNode(nodeIdx);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            std::cout << "mesh: " << node->GetName() << std::endl;
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxAMatrix geometryOffset = GetGeometry(node);
            for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                FbxAMatrix globalPosition = GetGlobalPosition(node, time, nullptr, nullptr);
                FbxAMatrix globalOffPosition = globalPosition * geometryOffset;
                float transformation[16] = { 0.0f };
                for (int i = 0; i < 16; i++) {
                    transformation[i] = (float)(((double*)&globalOffPosition)[i]);
                }
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            const bool hasVertexCache = mesh->GetDeformerCount(FbxDeformer::eVertexCache) &&
                (static_cast<FbxVertexCacheDeformer*>(mesh->GetDeformer(0, FbxDeformer::eVertexCache)))->Active.Get();
            if (hasVertexCache) {
                LogError("Vertex cache is not supported.");
                return false;
            }
            const bool hasSkin = mesh->GetDeformerCount(FbxDeformer::eSkin) > 0;
            if (!hasSkin) {
                LogInfo("Mesh is not skinning.");
                continue;
            }

            FbxAMatrix matrixGeo;
            matrixGeo.SetIdentity();
            if (node->GetNodeAttribute()) {
                const FbxVector4 lT = node->GetGeometricTranslation(FbxNode::eSourcePivot);
                const FbxVector4 lR = node->GetGeometricRotation(FbxNode::eSourcePivot);
                const FbxVector4 lS = node->GetGeometricScaling(FbxNode::eSourcePivot);
                matrixGeo.SetT(lT);
                matrixGeo.SetR(lR);
                matrixGeo.SetS(lS);
            }
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBone; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                        
                        //auto rotation = bone->GetLink()->EvaluateLocalRotation(time);
                        //auto scale = bone->GetLink()->EvaluateLocalScaling(time);
                        //auto translation = bone->GetLink()->EvaluateLocalTranslation(time);
                        auto transform = bone->GetLink()->EvaluateLocalTransform(time);
                        //FbxNode* pParentNode = bone->GetLink()->GetParent();
                        //FbxAMatrix parentMatrix = pParentNode->EvaluateLocalTransform(time);
                        //while ((pParentNode = pParentNode->GetParent()) != NULL) {
                        //    parentMatrix = pParentNode->EvaluateLocalTransform(time) * parentMatrix;
                        //}

                        //FbxAMatrix linkMatrix;
                        //bone->GetTransformLinkMatrix(linkMatrix);
                        //transform = transform;
                        //memset(&transform, 0, sizeof(double) * 16);
                        //for (int i = 0; i < 4; i++) {
                        //    transform[i] = translation[i];
                        //}
                        
                        float animFrame[16] = { 0.0f };
                        for (int i = 0; i < 16; i++) {
                            animFrame[i] = (float)(((double*)&transform)[i]);
                        }
                        // transpose
                        for (int i = 0; i < 4; i++) {
                            for (int j = i + 1; j < 4; j++) {
                                std::swap(animFrame[4 * i + j], animFrame[4 * j + i]);
                            }
                        }
                        auto iter = boneMap_.find(bone->GetLink()->GetName());
                        if (iter == boneMap_.end()) {
                            LogError("Unknown bone: " + iter->first);
                            return false;
                        }
                        if (boneAnimMap_[iter->second].size() < numOfFrame * 16) {
                            for (int i = 0; i < 16; i++) {
                                boneAnimMap_[iter->second].push_back(animFrame[i]);
                            }
                        }
                        //output.write(reinterpret_cast<char*>(animFrame), sizeof(float) * 12);
                    }
                }
            }
        }
    }
    
    std::cerr << "EXPORT HIRARCHY" << std::endl;
    std::ofstream output(directory + "/" + fileID, std::ios::binary);
    for (int frameID = 0; frameID < numOfFrame; frameID++) {
        for (const auto& bone : boneMap_) {
            auto& boneAnim = boneAnimMap_[bone.second];
            output.write(reinterpret_cast<char*>(&boneAnim[frameID * 16]), sizeof(float) * 16);
        }
    }

    std::ofstream json(directory + "/" + FilterInvalidFileNameChar(fileID + ".frame.json"));
    json << "{\n";
    for (int i = 0; i < numOfFrame; i++) {
        if (i > 0) {
            json << ",\n";
        }
        json << "\"frame_" << i << "\": {\n";
        bool first = true;
        for (const auto& pair : boneMap_) {
            if (first) {
                first = false;
            } else {
                json << ",\n";
            }
            json << "    \"" << pair.first << "\": [";
            for (int j = 0; j < 12; j++) {
                if (j > 0) {
                    json << ", ";
                }
                json << boneAnimMap_[pair.second][i * 16 + j];
            }
            json << "]";
        }
        json << "}";
    }
    json << "}";
    json.close();
    return false;
}

bool FbxHelper::ExportBlendshapeToObj(const std::string& directory, const std::string& fileID, bool noBlendshape)
{
    // animation
    FbxAnimStack * lCurrentAnimationStack = fbxScene_->FindMember<FbxAnimStack>(animStackNameArray_[0]->Buffer());
    auto animLayer = lCurrentAnimationStack->GetMember<FbxAnimLayer>();
    fbxScene_->SetCurrentAnimationStack(lCurrentAnimationStack);

    FbxTakeInfo* lCurrentTakeInfo = fbxScene_->GetTakeInfo(*(animStackNameArray_[0]));
    if (lCurrentTakeInfo) {
        startTime_ = lCurrentTakeInfo->mLocalTimeSpan.GetStart();
        stopTime_ = lCurrentTakeInfo->mLocalTimeSpan.GetStop();
    } else {
        // Take the time line value
        FbxTimeSpan lTimeLineTimeSpan;
        fbxScene_->GetGlobalSettings().GetTimelineDefaultTimeSpan(lTimeLineTimeSpan);

        startTime_ = lTimeLineTimeSpan.GetStart();
        stopTime_ = lTimeLineTimeSpan.GetStop();
    }
    int lStart = (int)startTime_.GetMilliSeconds();
    int lStop = (int)stopTime_.GetMilliSeconds();
    int lTime = (int)durationPerFrame_.GetMilliSeconds();
    
    int frameCount = 0;
    for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
        frameCount++;
    }
    std::cerr << "Num of frames: " << frameCount << std::endl;

    if (!noBlendshape) {

        // output blendshape
        int meshCount = -1;
        for (int nodeIdx = 0; nodeIdx < fbxScene_->GetNodeCount(); nodeIdx++) {
            auto node = fbxScene_->GetNode(nodeIdx);
            FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
            if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
                if (node->GetChildCount() > 0) {
                    LogInfo("Warning: Mesh node contains child node.");
                }
                int outputIndex = 0;
                FbxMesh* mesh = node->GetMesh();

                auto faceCount = mesh->GetPolygonCount();
                int vertexCount = mesh->GetControlPointsCount();
                std::vector<int> indices;
                std::vector<std::tuple<float, float, float>> normals(mesh->GetControlPointsCount());
                std::vector<std::tuple<float, float>> texCoords(mesh->GetControlPointsCount());

                FbxStringList uvNames;
                mesh->GetUVSetNames(uvNames);
                std::cerr << uvNames[0] << std::endl;
                for (int faceIdx = 0; faceIdx < faceCount; faceIdx++) {
                    for (int vIdx = 0; vIdx < 3; vIdx++) {
                        int index = mesh->GetPolygonVertex(faceIdx, vIdx);
                        indices.push_back(index);
                        FbxVector4 normal;
                        mesh->GetPolygonVertexNormal(faceIdx, vIdx, normal);
                        normals[index] = std::make_tuple(normal[0], normal[1], normal[2]);
                        bool unmapped;
                        FbxVector2 texCoord;
                        mesh->GetPolygonVertexUV(faceIdx, vIdx, uvNames[0], texCoord, unmapped);
                        texCoords[index] = std::make_tuple(texCoord[0], texCoord[1]);
                    }
                }
                {
                    meshCount++;
                    std::ofstream output(directory + "/" + FilterInvalidFileNameChar(node->GetName()) + std::to_string(outputIndex++)/* + "_" + FilterInvalidFileNameChar(shape->GetName())*/ + ".obj");
                    int vSize = vertexCount;

                    for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                        auto& vertex = mesh->GetControlPoints()[vertexIdx];
                        output << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\r\n";
                    }
                    for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                        auto& normal = normals[vertexIdx];
                        output << "vn " << std::get<0>(normal) << " " << std::get<1>(normal) << " " << std::get<2>(normal) << "\r\n";
                    }
                    for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                        auto& texCoord = texCoords[vertexIdx];
                        output << "vt " << std::get<0>(texCoord) << " " << std::get<1>(texCoord) << "\r\n";
                    }
                    for (int indexIdx = 0; indexIdx < indices.size(); indexIdx += 3) {
                        output << "f "
                            << indices[indexIdx] + 1 << "/" << indices[indexIdx] + 1 << "/" << indices[indexIdx] + 1 << " "
                            << indices[indexIdx + 1] + 1 << "/" << indices[indexIdx + 1] + 1 << "/" << indices[indexIdx + 1] + 1 << " "
                            << indices[indexIdx + 2] + 1 << "/" << indices[indexIdx + 2] + 1 << "/" << indices[indexIdx + 2] + 1 << "\r\n";
                    }
                }
                auto deformerCount = mesh->GetDeformerCount(FbxDeformer::eBlendShape);
                for (int deformerIdx = 0; deformerIdx < deformerCount; deformerIdx++) {
                    FbxBlendShape* lBlendShape = (FbxBlendShape*)mesh->GetDeformer(deformerIdx, FbxDeformer::eBlendShape);

                    int blendShapeChannelCount = lBlendShape->GetBlendShapeChannelCount();
                    std::cerr << "Channel Count: " << blendShapeChannelCount << std::endl;
                    for (int channelIdx = 0; channelIdx < blendShapeChannelCount; ++channelIdx) {
                        FbxBlendShapeChannel* channel = lBlendShape->GetBlendShapeChannel(channelIdx);
                        if (channel) {
                            // export blendshape
                            auto targetShapeCount = channel->GetTargetShapeCount();
                            for (int targetShapeIdx = 0; targetShapeIdx < targetShapeCount; targetShapeIdx++) {
                                auto shape = channel->GetTargetShape(targetShapeIdx);
                                std::ofstream output(directory + "/" + FilterInvalidFileNameChar(node->GetName()) + std::to_string(outputIndex++)/* + "_" + FilterInvalidFileNameChar(shape->GetName())*/ + ".obj");
                                int vSize = shape->GetControlPointsCount();
                                for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                                    auto& vertex = shape->GetControlPoints()[vertexIdx];
                                    output << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\r\n";
                                }
                                for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                                    auto& normal = normals[vertexIdx];
                                    output << "vn " << std::get<0>(normal) << " " << std::get<1>(normal) << " " << std::get<2>(normal) << "\r\n";
                                }
                                for (int vertexIdx = 0; vertexIdx < vSize; vertexIdx++) {
                                    auto& texCoord = texCoords[vertexIdx];
                                    output << "vt " << std::get<0>(texCoord) << " " << std::get<1>(texCoord) << "\r\n";
                                }
                                for (int indexIdx = 0; indexIdx < indices.size(); indexIdx += 3) {
                                    output << "f "
                                        << indices[indexIdx] + 1 << "/" << indices[indexIdx] + 1 << "/" << indices[indexIdx] + 1 << " "
                                        << indices[indexIdx + 1] + 1 << "/" << indices[indexIdx + 1] + 1 << "/" << indices[indexIdx + 1] + 1 << " "
                                        << indices[indexIdx + 2] + 1 << "/" << indices[indexIdx + 2] + 1 << "/" << indices[indexIdx + 2] + 1 << "\r\n";
                                }
                                output.close();
                            }
                        }
                    }
                }
            }
        }
    }
    // output expression animation
    std::vector<std::vector<double>> weightsTable;
    for (int nodeIdx = 0; nodeIdx < fbxScene_->GetNodeCount(); nodeIdx++) {
        auto node = fbxScene_->GetNode(nodeIdx);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxMesh* mesh = node->GetMesh();
            for (FbxTime time = startTime_; time < stopTime_ + durationPerFrame_; time += durationPerFrame_) {
                //animOutput << frameIdx++ << ": " << weight / fullWeight << std::endl;
                std::vector<double> weights;
                ComputeBlenshapeAnimationWeight(mesh, time, animLayer, weights);
                if (weights.empty()) {
                    break;
                }
                weightsTable.emplace_back(weights);
            }
        }
    }

    //std::ofstream animOutput(directory + std::string("/expression.json"));
    //animOutput << "{\n\"expressions\":[";
    //bool objBegin = true;
    //for (const auto& row : weightsTable) {
    //    if (objBegin) {
    //        objBegin = false;
    //    } else {
    //        animOutput << ",";
    //    }
    //    animOutput << "\n[";
    //    bool rowBegin = true;
    //    for (const auto& v : row) {
    //        if (rowBegin) {
    //            rowBegin = false;
    //        } else {
    //            animOutput << ", ";
    //        }
    //        animOutput << v;
    //    }
    //    animOutput << "]";
    //}
    //animOutput << "\n]}";

    std::ofstream expressionOutput(directory + std::string("/expression.bin"), std::ios::binary);
    int numOfExpression = weightsTable.size();
    int numOfBlendshape = numOfExpression > 0 ? weightsTable[0].size() : 0;
    expressionOutput.write((char*)&numOfExpression, sizeof(int));
    expressionOutput.write((char*)&numOfBlendshape, sizeof(int));
    for (const auto& row : weightsTable) {
        for (const auto& v : row) {
            float expr = v;
            expressionOutput.write((char*)&expr, sizeof(expr));
        }
    }
    return true;
}

void FbxHelper::TraverseHierarchy(FbxNode* node, FbxNode* parent)
{
    FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
    if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eSkeleton) {
        //hierarchyString_ += "\"" + std::string(node->GetName()) + "\": {\n\"parent\": ";
        hierarchyString_ += "\"" + std::to_string(boneMap_[node->GetName()]) + "_" + std::string(node->GetName()) + "\":";
        if (parent) {
            hierarchyString_ += "\"";
            hierarchyString_ += std::to_string(boneMap_[parent->GetName()]) + "_" + parent->GetName();
            hierarchyArray_[boneMap_[node->GetName()]] = boneMap_[parent->GetName()];
            hierarchyString_ += "\"";
        } else {
            hierarchyArray_[boneMap_[node->GetName()]] = -1;
            hierarchyString_ += "null";
        }
        hierarchyString_ += ",\n";

        for (int i = 0; i < node->GetChildCount(); i++) {
            TraverseHierarchy(node->GetChild(i), node);
        }
    } else {
        for (int i = 0; i < node->GetChildCount(); i++) {
            TraverseHierarchy(node->GetChild(i), nullptr);
        }
    }
}

void FbxHelper::ConstructBoneMap()
{
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxMesh* mesh = node->GetMesh();
            const int numOfVertex = mesh->GetControlPointsCount();
            if (numOfVertex == 0) {
                LogInfo("Warning: Mesh has no vertex.");
                continue;
            }
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBoneOfDeformer = deformer->GetClusterCount();
                for (int boneIndex = 0; boneIndex < numOfBoneOfDeformer; boneIndex++) {
                    FbxCluster* bone = deformer->GetCluster(boneIndex);
                    if (!bone->GetLink()) {
                        continue;
                    }
                    auto boneName = bone->GetLink()->GetName();
                    if (boneMap_.find(boneName) == boneMap_.end()) {
                        boneMap_.insert({ boneName, boneMap_.size() });
                    }
                }
            }
        }
    }
    int boneID = 0;
    for (auto&& pair : boneMap_) {
        pair.second = boneID;
        boneID++;
        //std::cerr << pair.first << ": " << pair.second << std::endl;
    }
    hierarchyArray_.resize(boneMap_.size());

    auto root = fbxScene_->GetRootNode();
    hierarchyString_ += "{\n";
    TraverseHierarchy(root, nullptr);
    auto lastCommaIdx = hierarchyString_.find_last_of(',');
    if (lastCommaIdx != std::string::npos) {
        hierarchyString_[lastCommaIdx] = '\n';
        hierarchyString_[lastCommaIdx + 1] = '}';
    } else {
        hierarchyString_ += "}";
    }
}

int FbxHelper::GetNumOfMesh(FbxNode* node)
{
    int numOfMesh = 0;
    FbxNodeAttribute* lNodeAttribute = node->GetNodeAttribute();
    if (lNodeAttribute && lNodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh)
        numOfMesh++;

    const int lChildCount = node->GetChildCount();
    for (int lChildIndex = 0; lChildIndex < lChildCount; ++lChildIndex)
        numOfMesh += GetNumOfMesh(node->GetChild(lChildIndex));
    return numOfMesh;
}

void FbxHelper::LogError(const std::string& errorMessage)
{
    if (errorMessage_.length() > 0) {
        errorMessage_ += "\n";
    }
    errorMessage_ += errorMessage;
}

void FbxHelper::LogInfo(const std::string& message)
{
    std::cerr << message << std::endl;
}

// Get the global position of the node for the current pose.
// If the specified node is not part of the pose or no pose is specified, get its
// global position at the current time.
FbxAMatrix GetGlobalPosition(FbxNode* pNode, const FbxTime& pTime, FbxPose* pPose, FbxAMatrix* pParentGlobalPosition)
{
    FbxAMatrix lGlobalPosition;
    bool        lPositionFound = false;

    if (pPose) {
        int lNodeIndex = pPose->Find(pNode);

        if (lNodeIndex > -1) {
            // The bind pose is always a global matrix.
            // If we have a rest pose, we need to check if it is
            // stored in global or local space.
            if (pPose->IsBindPose() || !pPose->IsLocalMatrix(lNodeIndex)) {
                lGlobalPosition = GetPoseMatrix(pPose, lNodeIndex);
            } else {
                // We have a local matrix, we need to convert it to
                // a global space matrix.
                FbxAMatrix lParentGlobalPosition;

                if (pParentGlobalPosition) {
                    lParentGlobalPosition = *pParentGlobalPosition;
                } else {
                    if (pNode->GetParent()) {
                        lParentGlobalPosition = GetGlobalPosition(pNode->GetParent(), pTime, pPose);
                    }
                }

                FbxAMatrix lLocalPosition = GetPoseMatrix(pPose, lNodeIndex);
                lGlobalPosition = lParentGlobalPosition * lLocalPosition;
            }

            lPositionFound = true;
        }
    }

    if (!lPositionFound) {
        // There is no pose entry for that node, get the current global position instead.

        // Ideally this would use parent global position and local position to compute the global position.
        // Unfortunately the equation 
        //    lGlobalPosition = pParentGlobalPosition * lLocalPosition
        // does not hold when inheritance type is other than "Parent" (RSrs).
        // To compute the parent rotation and scaling is tricky in the RrSs and Rrs cases.
        lGlobalPosition = pNode->EvaluateGlobalTransform(pTime);
    }

    return lGlobalPosition;
}

// Get the matrix of the given pose
FbxAMatrix GetPoseMatrix(FbxPose* pPose, int pNodeIndex)
{
    FbxAMatrix lPoseMatrix;
    FbxMatrix lMatrix = pPose->GetMatrix(pNodeIndex);

    memcpy((double*)lPoseMatrix, (double*)lMatrix, sizeof(lMatrix.mData));

    return lPoseMatrix;
}

// Get the geometry offset to a node. It is never inherited by the children.
FbxAMatrix GetGeometry(FbxNode* pNode)
{
    const FbxVector4 lT = pNode->GetGeometricTranslation(FbxNode::eSourcePivot);
    const FbxVector4 lR = pNode->GetGeometricRotation(FbxNode::eSourcePivot);
    const FbxVector4 lS = pNode->GetGeometricScaling(FbxNode::eSourcePivot);

    return FbxAMatrix(lT, lR, lS);
}

void ComputeBlenshapeAnimationWeight(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer* pAnimLayer, std::vector<double>& weights)
{
    int lBlendShapeDeformerCount = pMesh->GetDeformerCount(FbxDeformer::eBlendShape);
    for (int lBlendShapeIndex = 0; lBlendShapeIndex<lBlendShapeDeformerCount; ++lBlendShapeIndex) {
        FbxBlendShape* lBlendShape = (FbxBlendShape*)pMesh->GetDeformer(lBlendShapeIndex, FbxDeformer::eBlendShape);
        int lBlendShapeChannelCount = lBlendShape->GetBlendShapeChannelCount();
        for (int lChannelIndex = 0; lChannelIndex<lBlendShapeChannelCount; ++lChannelIndex) {
            FbxBlendShapeChannel* lChannel = lBlendShape->GetBlendShapeChannel(lChannelIndex);
            if (lChannel) {
                // Get the percentage of influence on this channel.
                FbxAnimCurve* lFCurve = pMesh->GetShapeChannel(lBlendShapeIndex, lChannelIndex, pAnimLayer);
                if (!lFCurve) continue;
                double lWeight = lFCurve->Evaluate(pTime);

                // Find the two shape indices for influence calculation according to the weight.
                // Consider index of base geometry as -1.

                int lShapeCount = lChannel->GetTargetShapeCount();
                double* lFullWeights = lChannel->GetTargetShapeFullWeights();

                // Find out which scope the lWeight falls in.
                int lStartIndex = -1;
                int lEndIndex = -1;
                for (int lShapeIndex = 0; lShapeIndex<lShapeCount; ++lShapeIndex) {
                    if (lWeight > 0 && lWeight <= lFullWeights[0]) {
                        lEndIndex = 0;
                        break;
                    }
                    if (lWeight > lFullWeights[lShapeIndex] && lWeight < lFullWeights[lShapeIndex + 1]) {
                        lStartIndex = lShapeIndex;
                        lEndIndex = lShapeIndex + 1;
                        break;
                    }
                }

                FbxShape* lStartShape = NULL;
                FbxShape* lEndShape = NULL;
                if (lStartIndex > -1) {
                    lStartShape = lChannel->GetTargetShape(lStartIndex);
                }
                if (lEndIndex > -1) {
                    lEndShape = lChannel->GetTargetShape(lEndIndex);
                }

                //The weight percentage falls between base geometry and the first target shape.
                if (lStartIndex == -1 && lEndShape) {
                    double lEndWeight = lFullWeights[0];
                    // Calculate the real weight.
                    lWeight = (lWeight / lEndWeight);
                }
                //The weight percentage falls between two target shapes.
                else if (lStartShape && lEndShape) {
                    double lStartWeight = lFullWeights[lStartIndex];
                    double lEndWeight = lFullWeights[lEndIndex];
                    // Calculate the real weight.
                    lWeight = ((lWeight - lStartWeight) / (lEndWeight - lStartWeight));
                }
                weights.push_back(lWeight);
            }//If lChannel is valid
        }//For each blend shape channel
    }//For each blend shape deformer
}

// Deform the vertex array with the shapes contained in the mesh.
void ComputeShapeDeformation(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer* pAnimLayer, FbxVector4* pVertexArray)
{
    int lVertexCount = pMesh->GetControlPointsCount();

    FbxVector4* lSrcVertexArray = pVertexArray;
    FbxVector4* lDstVertexArray = new FbxVector4[lVertexCount];
    memcpy(lDstVertexArray, pVertexArray, lVertexCount * sizeof(FbxVector4));

    int lBlendShapeDeformerCount = pMesh->GetDeformerCount(FbxDeformer::eBlendShape);
    for (int lBlendShapeIndex = 0; lBlendShapeIndex<lBlendShapeDeformerCount; ++lBlendShapeIndex) {
        FbxBlendShape* lBlendShape = (FbxBlendShape*)pMesh->GetDeformer(lBlendShapeIndex, FbxDeformer::eBlendShape);

        int lBlendShapeChannelCount = lBlendShape->GetBlendShapeChannelCount();
        for (int lChannelIndex = 0; lChannelIndex<lBlendShapeChannelCount; ++lChannelIndex) {
            FbxBlendShapeChannel* lChannel = lBlendShape->GetBlendShapeChannel(lChannelIndex);
            if (lChannel) {
                // Get the percentage of influence on this channel.
                FbxAnimCurve* lFCurve = pMesh->GetShapeChannel(lBlendShapeIndex, lChannelIndex, pAnimLayer);
                if (!lFCurve) continue;
                double lWeight = lFCurve->Evaluate(pTime);

                /*
                If there is only one targetShape on this channel, the influence is easy to calculate:
                influence = (targetShape - baseGeometry) * weight * 0.01
                dstGeometry = baseGeometry + influence

                But if there are more than one targetShapes on this channel, this is an in-between
                blendshape, also called progressive morph. The calculation of influence is different.

                For example, given two in-between targets, the full weight percentage of first target
                is 50, and the full weight percentage of the second target is 100.
                When the weight percentage reach 50, the base geometry is already be fully morphed
                to the first target shape. When the weight go over 50, it begin to morph from the
                first target shape to the second target shape.

                To calculate influence when the weight percentage is 25:
                1. 25 falls in the scope of 0 and 50, the morphing is from base geometry to the first target.
                2. And since 25 is already half way between 0 and 50, so the real weight percentage change to
                the first target is 50.
                influence = (firstTargetShape - baseGeometry) * (25-0)/(50-0) * 100
                dstGeometry = baseGeometry + influence

                To calculate influence when the weight percentage is 75:
                1. 75 falls in the scope of 50 and 100, the morphing is from the first target to the second.
                2. And since 75 is already half way between 50 and 100, so the real weight percentage change
                to the second target is 50.
                influence = (secondTargetShape - firstTargetShape) * (75-50)/(100-50) * 100
                dstGeometry = firstTargetShape + influence
                */

                // Find the two shape indices for influence calculation according to the weight.
                // Consider index of base geometry as -1.

                int lShapeCount = lChannel->GetTargetShapeCount();
                double* lFullWeights = lChannel->GetTargetShapeFullWeights();

                // Find out which scope the lWeight falls in.
                int lStartIndex = -1;
                int lEndIndex = -1;
                for (int lShapeIndex = 0; lShapeIndex<lShapeCount; ++lShapeIndex) {
                    if (lWeight > 0 && lWeight <= lFullWeights[0]) {
                        lEndIndex = 0;
                        break;
                    }
                    if (lWeight > lFullWeights[lShapeIndex] && lWeight < lFullWeights[lShapeIndex + 1]) {
                        lStartIndex = lShapeIndex;
                        lEndIndex = lShapeIndex + 1;
                        break;
                    }
                }

                FbxShape* lStartShape = NULL;
                FbxShape* lEndShape = NULL;
                if (lStartIndex > -1) {
                    lStartShape = lChannel->GetTargetShape(lStartIndex);
                }
                if (lEndIndex > -1) {
                    lEndShape = lChannel->GetTargetShape(lEndIndex);
                }

                //The weight percentage falls between base geometry and the first target shape.
                if (lStartIndex == -1 && lEndShape) {
                    double lEndWeight = lFullWeights[0];
                    // Calculate the real weight.
                    lWeight = (lWeight / lEndWeight) * 100;
                    // Initialize the lDstVertexArray with vertex of base geometry.
                    memcpy(lDstVertexArray, lSrcVertexArray, lVertexCount * sizeof(FbxVector4));
                    for (int j = 0; j < lVertexCount; j++) {
                        // Add the influence of the shape vertex to the mesh vertex.
                        FbxVector4 lInfluence = (lEndShape->GetControlPoints()[j] - lSrcVertexArray[j]) * lWeight * 0.01;
                        lDstVertexArray[j] += lInfluence;
                    }
                }
                //The weight percentage falls between two target shapes.
                else if (lStartShape && lEndShape) {
                    double lStartWeight = lFullWeights[lStartIndex];
                    double lEndWeight = lFullWeights[lEndIndex];
                    // Calculate the real weight.
                    lWeight = ((lWeight - lStartWeight) / (lEndWeight - lStartWeight)) * 100;
                    // Initialize the lDstVertexArray with vertex of the previous target shape geometry.
                    memcpy(lDstVertexArray, lStartShape->GetControlPoints(), lVertexCount * sizeof(FbxVector4));
                    for (int j = 0; j < lVertexCount; j++) {
                        // Add the influence of the shape vertex to the previous shape vertex.
                        FbxVector4 lInfluence = (lEndShape->GetControlPoints()[j] - lStartShape->GetControlPoints()[j]) * lWeight * 0.01;
                        lDstVertexArray[j] += lInfluence;
                    }
                }
            }//If lChannel is valid
        }//For each blend shape channel
    }//For each blend shape deformer

    memcpy(pVertexArray, lDstVertexArray, lVertexCount * sizeof(FbxVector4));

    delete[] lDstVertexArray;
}