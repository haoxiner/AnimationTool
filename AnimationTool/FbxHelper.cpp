#include "FbxHelper.h"
#include "fbxsdk\core\math\fbxvector4.h"
#include "fbxsdk\scene\geometry\fbxblendshapechannel.h"
#include <iostream>
#include <algorithm>
#include <fstream>

void ComputeShapeDeformation(FbxMesh* pMesh, FbxTime& pTime, FbxAnimLayer* pAnimLayer, FbxVector4* pVertexArray);

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

bool FbxHelper::ExportKeyFrames(const std::string& directory, const std::string& fileID)
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
    std::ofstream output(directory + "/" + FilterInvalidFileNameChar(fileID + ".anim"), std::ios::binary);
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
            std::string filename = directory + "\\" + FilterInvalidFileNameChar(fileID + "_" + std::to_string(meshID) + "_" + node->GetName());
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
    auto root = fbxScene_->GetRootNode();
    hierarchyString_ += "[\n";
    TraverseHierarchy(root, nullptr);
    auto lastCommaIdx = hierarchyString_.find_last_of(',');
    if (lastCommaIdx != std::string::npos) {
        hierarchyString_[lastCommaIdx] = '\n';
        hierarchyString_[lastCommaIdx + 1] = ']';
    } else {
        hierarchyString_ += "]";
    }
    std::cerr << hierarchyString_ << std::endl;
    std::ofstream file(directory + "/" + fileID + ".json");
    file << hierarchyString_;
    file.close();
    return true;
}

bool FbxHelper::ExportBlendshapeToObj(const std::string& directory, const std::string& fileID)
{
    for (int i = 0; i < fbxScene_->GetNodeCount(); i++) {
        auto node = fbxScene_->GetNode(i);
        FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
        if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eMesh) {
            if (node->GetChildCount() > 0) {
                LogInfo("Warning: Mesh node contains child node.");
            }
            FbxMesh* lMesh = node->GetMesh();
            auto deformerCount = lMesh->GetDeformerCount(FbxDeformer::eBlendShape);
            for (int deformerIdx = 0; deformerIdx < deformerCount; deformerIdx++) {
                FbxBlendShape* lBlendShape = (FbxBlendShape*)lMesh->GetDeformer(deformerIdx, FbxDeformer::eBlendShape);

                int lBlendShapeChannelCount = lBlendShape->GetBlendShapeChannelCount();
                for (int lChannelIndex = 0; lChannelIndex < lBlendShapeChannelCount; ++lChannelIndex) {
                    FbxBlendShapeChannel* lChannel = lBlendShape->GetBlendShapeChannel(lChannelIndex);
                    if (lChannel) {
                        auto targetShapeCount = lChannel->GetTargetShapeCount();
                        for (int targetShapeIdx = 0; targetShapeIdx < targetShapeCount; targetShapeIdx++) {
                            auto shape = lChannel->GetTargetShape(targetShapeIdx);
                            std::cerr << shape->GetName() << std::endl;
                        }
                    }
                }
            }
            // ====================================================== //

            //const int lVertexCount = lMesh->GetControlPointsCount();
            //// If it has some defomer connection, update the vertices position
            //const bool lHasVertexCache = lMesh->GetDeformerCount(FbxDeformer::eVertexCache) &&
            //    (static_cast<FbxVertexCacheDeformer*>(lMesh->GetDeformer(0, FbxDeformer::eVertexCache)))->Active.Get();
            //const bool lHasShape = lMesh->GetShapeCount() > 0;
            //const bool lHasSkin = lMesh->GetDeformerCount(FbxDeformer::eSkin) > 0;
            //const bool lHasDeformation = lHasVertexCache || lHasShape || lHasSkin;

            //FbxVector4* lVertexArray = NULL;
            //if (lHasDeformation) {
            //    lVertexArray = new FbxVector4[lVertexCount];
            //    memcpy(lVertexArray, lMesh->GetControlPoints(), lVertexCount * sizeof(FbxVector4));
            //}

            //if (lHasDeformation) {
            //    if (lHasShape) {
            //        // Deform the vertex array with the shapes.
            //        ComputeShapeDeformation(lMesh, pTime, pAnimLayer, lVertexArray);
            //    }
            //}
            // ====================================================== //
        }
    }
    return false;
}

void FbxHelper::TraverseHierarchy(FbxNode* node, FbxNode* parent)
{
    FbxNodeAttribute* nodeAttribute = node->GetNodeAttribute();
    if (nodeAttribute && nodeAttribute->GetAttributeType() == FbxNodeAttribute::eSkeleton) {
        hierarchyString_ += "{\n\"parent\": ";
        if (parent) {
            hierarchyString_ += "\"";
            hierarchyString_ += parent->GetName();
            hierarchyString_ += "\"";
        } else {
            hierarchyString_ += "null";
        }
        hierarchyString_ += ",\n";

        hierarchyString_ += "\"name\": \"";
        hierarchyString_ += node->GetName();
        hierarchyString_ += "\",\n";

        const FbxDouble3 lT = node->LclTranslation;
        const FbxDouble3 lR = node->LclRotation;
        const FbxDouble3 lS = node->LclScaling;

        hierarchyString_ += "\"rotation\": [";
        for (int i = 0; i < 3; i++) {
            hierarchyString_ += std::to_string(lR[i]) + ((i < 2) ? ", " : "],\n");
        }
        hierarchyString_ += "\"translation\": [";
        for (int i = 0; i < 3; i++) {
            hierarchyString_ += std::to_string(lT[i]) + ((i < 2) ? ", " : "]\n");
        }
        
        hierarchyString_ += "},\n";

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
    for (const auto& pair : boneMap_) {
        //std::cerr << pair.first << ": " << pair.second << std::endl;
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