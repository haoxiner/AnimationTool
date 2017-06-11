#include "FbxHelper.h"
#include <iostream>
#include <algorithm>
#include <fstream>

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
    return true;
}

bool FbxHelper::ExportKeyFrames(const std::string& filename)
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

    FbxTime lCurTime;
    int numOfFrame = 0;
    for (auto time = startTime_; lCurTime < stopTime_ + durationPerFrame_; lCurTime += durationPerFrame_) {
        numOfFrame++;
    }
    LogInfo("Number of frame: " + std::to_string(numOfFrame));
    std::ofstream output(filename, std::ios::binary);
    int numOfMesh = 0;
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
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
            const bool hasShape = mesh->GetShapeCount() > 0;
            if (hasShape) {
                LogError("Blendshape is not supported.");
                return false;
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
                            for (int j = 1; j < 4; j++) {
                                std::swap(animFrame[4 * i + j], animFrame[4 * j + i]);
                            }
                        }
                        output.write(reinterpret_cast<char*>(animFrame), sizeof(float) * 12);
                    }
                }
            }
        }
    }
    output.close();
    LogInfo("Number of mesh: " + std::to_string(numOfMesh));
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
                int numOfBone = deformer->GetClusterCount();
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
            std::string filename = directory + "\\" + FilterInvalidFileNameChar(fileID + "_" + std::to_string(meshID) + "_" + node->GetName());
            std::ofstream output(filename, std::ios::binary);
            std::cerr << "output: " << filename << std::endl;
            output.write(reinterpret_cast<const char*>(&numOfVertex), sizeof(int));
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
            for (int deformerIndex = 0; deformerIndex < mesh->GetDeformerCount(FbxDeformer::eSkin); deformerIndex++) {
                FbxSkin* deformer = reinterpret_cast<FbxSkin*>(mesh->GetDeformer(deformerIndex, FbxDeformer::eSkin));
                int numOfBone = deformer->GetClusterCount();
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
            std::vector<int> perVertexBoneIDList0(numOfVertex * 4, 0);
            std::vector<float> perVertexBoneWeightList0(numOfVertex * 4, 0);
            std::vector<int> perVertexBoneIDList1(numOfVertex * 4, 0);
            std::vector<float> perVertexBoneWeightList1(numOfVertex * 4, 0);
            for (int i = 0; i < numOfVertex; i++) {
                if (vertexBoneIDAndWeightList[i].size() < MAX_BONE_PER_VERTEX) {
                    for (int j = 0; j < vertexBoneIDAndWeightList[i].size(); j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList0[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].second;
                        } else {
                            perVertexBoneIDList1[i * MAX_BONE_PER_VERTEX + j - 4] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList1[i * MAX_BONE_PER_VERTEX + j - 4] = vertexBoneIDAndWeightList[i][j].second;
                        }
                    }
                } else {
                    float totalWeight = 0.0f;
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList0[i * MAX_BONE_PER_VERTEX + j] = vertexBoneIDAndWeightList[i][j].second;
                        } else {
                            perVertexBoneIDList1[i * MAX_BONE_PER_VERTEX + j - 4] = vertexBoneIDAndWeightList[i][j].first;
                            perVertexBoneWeightList1[i * MAX_BONE_PER_VERTEX + j - 4] = vertexBoneIDAndWeightList[i][j].second;
                        }
                        totalWeight += vertexBoneIDAndWeightList[i][j].second;
                    }
                    for (int j = 0; j < MAX_BONE_PER_VERTEX; j++) {
                        if (j < 4) {
                            perVertexBoneIDList0[i * MAX_BONE_PER_VERTEX + j] /= totalWeight;
                        } else {
                            perVertexBoneIDList1[i * MAX_BONE_PER_VERTEX + j - 4] /= totalWeight;
                        }
                    }
                }
            }
            std::string filename = directory + "\\" + FilterInvalidFileNameChar(fileID + "_" + std::to_string(meshID) + "_" + node->GetName());
            std::ofstream output(filename, std::ios::binary);
            std::cerr << "output: " << filename << std::endl;
            output.write(reinterpret_cast<const char*>(&numOfVertex), sizeof(int));
            output.write(reinterpret_cast<char*>(perVertexBoneIDList0.data()), sizeof(int) * perVertexBoneIDList0.size());
            output.write(reinterpret_cast<char*>(perVertexBoneIDList1.data()), sizeof(int) * perVertexBoneIDList1.size());
            output.write(reinterpret_cast<char*>(perVertexBoneWeightList0.data()), sizeof(float) * perVertexBoneWeightList0.size());
            output.write(reinterpret_cast<char*>(perVertexBoneWeightList1.data()), sizeof(float) * perVertexBoneWeightList1.size());
            output.close();
            meshID++;
        }
    }

    return true;
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