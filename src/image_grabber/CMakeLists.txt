set(VS_GRAPHS_FRONTEND_RGBD_GRABBER_SOURCES
  methods/rgbdGrabRGBD.cc
  methods/rgbdGrabVoxbloxSkeletonGraph.cc
  methods/rgbdLogRgbdObservabilitySummary.cc
  methods/rgbdProcessRgbdPackets.cc
  methods/rgbdPublishRgbdFrontendHealth.cc
  methods/rgbdRequestStop.cc
)
list(TRANSFORM VS_GRAPHS_FRONTEND_RGBD_GRABBER_SOURCES
  PREPEND "${CMAKE_CURRENT_LIST_DIR}/")

set(VS_GRAPHS_FRONTEND_INERTIAL_GRABBER_SOURCES
  methods/getImage.cc
  methods/grabPointCloud.cc
  methods/grabRgbd.cc
  methods/grabVoxbloxSkeletonGraph.cc
  methods/syncWithImu.cc
)
list(TRANSFORM VS_GRAPHS_FRONTEND_INERTIAL_GRABBER_SOURCES
  PREPEND "${CMAKE_CURRENT_LIST_DIR}/")
