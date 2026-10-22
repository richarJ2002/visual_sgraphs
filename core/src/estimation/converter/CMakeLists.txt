# converter module sources.
set(VS_GRAPHS_UTILS_CONVERTER_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/methods/isRotationMatrix.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toCvMat.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toCvSE3.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toCvSkewMatrix.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toDescriptorVector.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toEuler.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toMatrix3d.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toMatrix3f.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toMatrix4d.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toMatrix4f.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toQuaternion.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toSE3Quat.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toSophus.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toVector3d.cc
  ${CMAKE_CURRENT_LIST_DIR}/methods/toVector3f.cc
)
