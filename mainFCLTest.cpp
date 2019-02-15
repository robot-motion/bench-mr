#define DEBUG  // TODO activate DEBUG in PlannerSettings.h

#include "base/PlannerSettings.h"

#include <omplapp/geometry/detail/FCLStateValidityChecker.h>

#include "metrics/PathLengthMetric.h"

#include "base/Primitives.h"
#include "planners/OMPLPlanner.hpp"

namespace og = ompl::geometric;

typedef std::array<Point, 3> Triangle;

aiScene *createRobot(const std::vector<Triangle> &triangles) {
  auto *robot = new aiScene;
  robot->mRootNode = new aiNode();

  robot->mMaterials = new aiMaterial *[1];
  robot->mMaterials[0] = nullptr;
  robot->mNumMaterials = 1;

  robot->mMaterials[0] = new aiMaterial();

  robot->mMeshes = new aiMesh *[1];
  robot->mMeshes[0] = nullptr;
  robot->mNumMeshes = 1;

  robot->mMeshes[0] = new aiMesh();
  robot->mMeshes[0]->mMaterialIndex = 0;

  robot->mRootNode->mMeshes = new unsigned int[1];
  robot->mRootNode->mMeshes[0] = 0;
  robot->mRootNode->mNumMeshes = 1;

  auto pMesh = robot->mMeshes[0];

  pMesh->mVertices = new aiVector3D[triangles.size() * 3];
  //    pMesh->mNormals = new aiVector3D[ triangles.size() * 3 ];
  pMesh->mNumVertices = triangles.size() * 3;

  pMesh->mTextureCoords[0] = new aiVector3D[triangles.size() * 3];
  pMesh->mNumUVComponents[0] = triangles.size() * 3;

  int j = 0;
  for (const auto &tri : triangles) {
    pMesh->mVertices[j++] = aiVector3D(static_cast<ai_real>(tri[0].x),
                                       static_cast<ai_real>(tri[0].y), 0);
    pMesh->mVertices[j++] = aiVector3D(static_cast<ai_real>(tri[1].x),
                                       static_cast<ai_real>(tri[1].y), 0);
    pMesh->mVertices[j++] = aiVector3D(static_cast<ai_real>(tri[2].x),
                                       static_cast<ai_real>(tri[2].y), 0);
    //        pMesh->mNormals[ itr - triangles.begin() ] = aiVector3D(
    //        normals[j].x, normals[j].y, normals[j].z );
  }

  pMesh->mFaces = new aiFace[triangles.size()];
  pMesh->mNumFaces = (unsigned int)(triangles.size());

  int k = 0;
  for (int i = 0; i < triangles.size(); i++) {
    aiFace &face = pMesh->mFaces[i];
    face.mIndices = new unsigned int[3];
    face.mNumIndices = 3;

    face.mIndices[0] = static_cast<unsigned int>(k);
    face.mIndices[1] = static_cast<unsigned int>(k + 1);
    face.mIndices[2] = static_cast<unsigned int>(k + 2);
    k = k + 3;
  }

  return robot;
}

aiScene *createObstacles() {
  auto *map = new aiScene;
  map->mRootNode = new aiNode();

  map->mMaterials = new aiMaterial *[1];
  map->mMaterials[0] = nullptr;
  map->mNumMaterials = 1;

  map->mMaterials[0] = new aiMaterial();

  map->mMeshes = new aiMesh *[1];
  map->mMeshes[0] = nullptr;
  map->mNumMeshes = 1;

  map->mMeshes[0] = new aiMesh();
  map->mMeshes[0]->mMaterialIndex = 0;

  map->mRootNode->mMeshes = new unsigned int[1];
  map->mRootNode->mMeshes[0] = 0;
  map->mRootNode->mNumMeshes = 1;

  auto pMesh = map->mMeshes[0];

  std::vector<Point> vertices;
  for (unsigned int y = 0; y < PlannerSettings::environment->height(); ++y) {
    for (unsigned int x = 0; x < PlannerSettings::environment->width(); ++x) {
      if (PlannerSettings::environment->collides(x, y)) {
        // triangle 1
        vertices.emplace_back(Point(x, y));
        vertices.emplace_back(Point(x + 1, y));
        vertices.emplace_back(Point(x + 1, y + 1));

        // triangle 2
        vertices.emplace_back(Point(x, y));
        vertices.emplace_back(Point(x + 1, y + 1));
        vertices.emplace_back(Point(x, y + 1));
      }
    }
  }

  pMesh->mVertices = new aiVector3D[vertices.size()];
  //    pMesh->mNormals = new aiVector3D[ vertices.size() ];
  pMesh->mNumVertices = vertices.size();

  pMesh->mTextureCoords[0] = new aiVector3D[vertices.size()];
  pMesh->mNumUVComponents[0] = vertices.size();

  int j = 0;
  for (auto itr = vertices.begin(); itr != vertices.end(); ++itr) {
    pMesh->mVertices[itr - vertices.begin()] =
        aiVector3D(static_cast<ai_real>(vertices[j].x),
                   static_cast<ai_real>(vertices[j].y), 0);
    //        pMesh->mNormals[ itr - vertices.begin() ] = aiVector3D(
    //        normals[j].x, normals[j].y, normals[j].z );
    j++;
  }

  pMesh->mFaces = new aiFace[vertices.size() / 4];
  pMesh->mNumFaces = (unsigned int)(vertices.size() / 4);

  int k = 0;
  for (int i = 0; i < (vertices.size() / 3); i++) {
    aiFace &face = pMesh->mFaces[i];
    face.mIndices = new unsigned int[3];
    face.mNumIndices = 3;

    face.mIndices[0] = static_cast<unsigned int>(k);
    face.mIndices[1] = static_cast<unsigned int>(k + 1);
    face.mIndices[2] = static_cast<unsigned int>(k + 2);
    k = k + 3;
  }

  return map;
}

int main(int argc, char **argv) {
  PlannerSettings::environment =
      Environment::createRandomCorridor(50, 50, 3, 30, 123);

  PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
  PlannerSettings::initializeSteering();

  auto ss = new og::SimpleSetup(PlannerSettings::stateSpace);
  ompl::app::GeometrySpecification geomSpec;
  ompl::app::GeometricStateExtractor geomStateExtractor =
      [](const ompl::base::State *s,
         unsigned int) -> const ompl::base::State * { return s; };

  // create robot with 4 vertices

  auto *planner = new RRTPlanner;

  std::vector<Triangle> robot = {
      {Point(-0.5, -0.5), Point(0.5, -0.5), Point(0.5, 0.5)},
      {Point(-0.5, -0.5), Point(0.5, 0.5), Point(-0.5, 0.5)}};
  geomSpec.robot.emplace_back(createRobot(robot));
  geomSpec.robotShift.emplace_back(aiVector3D(0, 0, 0));
  geomSpec.obstacles.emplace_back(createObstacles());
  geomSpec.obstaclesShift.emplace_back(aiVector3D(0, 0, 0));
  ss->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
      new ompl::app::FCLStateValidityChecker<ompl::app::Motion_2D>(
          PlannerSettings::spaceInfo, geomSpec, geomStateExtractor, false)));

  if (!planner->run()) return EXIT_FAILURE;

  return EXIT_SUCCESS;
}