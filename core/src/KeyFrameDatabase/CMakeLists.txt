# KeyFrameDatabase module sources: single KeyFrameDatabase object (§4.5)
# plus the file-local ordering helper (§5.4). No source file is listed
# twice.
set(VS_GRAPHS_KEYFRAMEDATABASE_SOURCES
  core/src/KeyFrameDatabase/methods/add.cc
  core/src/KeyFrameDatabase/methods/erase.cc
  core/src/KeyFrameDatabase/methods/clear.cc
  core/src/KeyFrameDatabase/methods/clearMap.cc
  core/src/KeyFrameDatabase/methods/detectLoopCandidates.cc
  core/src/KeyFrameDatabase/methods/detectCandidates.cc
  core/src/KeyFrameDatabase/methods/detectBestCandidates.cc
  core/src/KeyFrameDatabase/methods/detectNBestCandidates.cc
  core/src/KeyFrameDatabase/methods/detectRelocalizationCandidates.cc
  core/src/KeyFrameDatabase/methods/setORBVocabulary.cc
  core/src/KeyFrameDatabase/compFirst.cc
)
