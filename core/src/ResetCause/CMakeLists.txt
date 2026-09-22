# ResetCause module sources: single ResetCauseRetention object (§4.5)
# plus module free functions (§5.4). The owner registry lives in
# retainResetCause.cc (see its note). No source file is listed twice.
set(VS_GRAPHS_RESETCAUSE_SOURCES
  core/src/ResetCause/methods/ResetCauseRetention/retain.cc
  core/src/ResetCause/methods/ResetCauseRetention/consume.cc
  core/src/ResetCause/methods/ResetCauseRetention/hasRetainedCause.cc
  core/src/ResetCause/resetCauseToString.cc
  core/src/ResetCause/retainResetCause.cc
  core/src/ResetCause/consumeResetCause.cc
  core/src/ResetCause/clearResetCause.cc
  core/src/ResetCause/resetActionToString.cc
  core/src/ResetCause/formatResetAttribution.cc
  core/src/ResetCause/reportResetAttribution.cc
)
