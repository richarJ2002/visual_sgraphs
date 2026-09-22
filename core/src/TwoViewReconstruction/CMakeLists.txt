# TwoViewReconstruction object sources: exactly this object's translation
# units.
set(VS_GRAPHS_TWOVIEWRECONSTRUCTION_SOURCES
  core/src/TwoViewReconstruction/methods/reconstruct.cc
  core/src/TwoViewReconstruction/methods/findHomography.cc
  core/src/TwoViewReconstruction/methods/findFundamental.cc
  core/src/TwoViewReconstruction/methods/computeH21.cc
  core/src/TwoViewReconstruction/methods/computeF21.cc
  core/src/TwoViewReconstruction/methods/checkHomography.cc
  core/src/TwoViewReconstruction/methods/checkFundamental.cc
  core/src/TwoViewReconstruction/methods/reconstructF.cc
  core/src/TwoViewReconstruction/methods/reconstructH.cc
  core/src/TwoViewReconstruction/methods/normalize.cc
  core/src/TwoViewReconstruction/methods/checkRT.cc
  core/src/TwoViewReconstruction/methods/decomposeE.cc
)
