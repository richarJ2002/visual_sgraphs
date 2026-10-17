# Pinhole object sources.
set(VS_GRAPHS_CAMERAMODELS_PINHOLE_SOURCES
  core/src/camera_models/pinhole/methods/computeProjectionJacobian.cc
  core/src/camera_models/pinhole/methods/epipolarConstrain.cc
  core/src/camera_models/pinhole/methods/isEqual.cc
  core/src/camera_models/pinhole/methods/matchAndTriangulate.cc
  core/src/camera_models/pinhole/methods/project.cc
  core/src/camera_models/pinhole/methods/projectMat.cc
  core/src/camera_models/pinhole/methods/readCalibration.cc
  core/src/camera_models/pinhole/methods/reconstructWithTwoViews.cc
  core/src/camera_models/pinhole/methods/serialize.cc
  core/src/camera_models/pinhole/methods/toK.cc
  core/src/camera_models/pinhole/methods/toK_.cc
  core/src/camera_models/pinhole/methods/uncertainty2.cc
  core/src/camera_models/pinhole/methods/unproject.cc
  core/src/camera_models/pinhole/methods/unprojectEig.cc
  core/src/camera_models/pinhole/methods/writeCalibration.cc
)
