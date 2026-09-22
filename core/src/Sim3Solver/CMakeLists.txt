# Sim3Solver object sources: exactly this object's translation units.
set(VS_GRAPHS_SIM3SOLVER_SOURCES
  core/src/Sim3Solver/methods/setRansacParameters.cc
  core/src/Sim3Solver/methods/iterate/fixedIterations.cc
  core/src/Sim3Solver/methods/iterate/withConvergence.cc
  core/src/Sim3Solver/methods/find.cc
  core/src/Sim3Solver/methods/computeCentroid.cc
  core/src/Sim3Solver/methods/computeSim3.cc
  core/src/Sim3Solver/methods/checkInliers.cc
  core/src/Sim3Solver/methods/getEstimatedTransformation.cc
  core/src/Sim3Solver/methods/getEstimatedRotation.cc
  core/src/Sim3Solver/methods/getEstimatedTranslation.cc
  core/src/Sim3Solver/methods/getEstimatedScale.cc
  core/src/Sim3Solver/methods/project.cc
  core/src/Sim3Solver/methods/fromCameraToImage.cc
)
