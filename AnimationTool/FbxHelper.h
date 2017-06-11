#pragma once
#include <fbxsdk.h>
#include <string>
#include <memory>
#include <vector>
class FbxHelper
{
public:
    bool Startup();
    ~FbxHelper();
    bool LoadFBX(const std::string& filename);
    bool ExportKeyFrames(const std::string& filename);
    bool ExportVertexSkinning(const std::string& directory, const std::string& fileID);
    bool ExportVertexSkinningAsTexture(const std::string& directory, const std::string& fileID);
private:
    int GetNumOfMesh(FbxNode* node);
    void LogError(const std::string& errorMessage);
    void LogInfo(const std::string& message);
private:
    std::string errorMessage_;
    FbxManager* fbxManager_ = nullptr;
    FbxScene* fbxScene_ = nullptr;
    // animation
    FbxTime durationPerFrame_;
    FbxTime startTime_;
    FbxTime stopTime_;
    FbxArray<FbxString*> animStackNameArray_;
};
inline std::string FilterInvalidFileNameChar(const std::string& filename)
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

#include <fbxsdk.h>

FbxAMatrix GetGlobalPosition(FbxNode* pNode,
                             const FbxTime& pTime,
                             FbxPose* pPose = NULL,
                             FbxAMatrix* pParentGlobalPosition = NULL);
FbxAMatrix GetPoseMatrix(FbxPose* pPose,
                         int pNodeIndex);
FbxAMatrix GetGeometry(FbxNode* pNode);
//Compute the transform matrix that the cluster will transform the vertex.
inline void ComputeClusterDeformation(FbxAMatrix& pGlobalPosition,
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
#endif // #ifndef _GET_POSITION_H