# pcl_custom module sources.
set(VS_GRAPHS_PCLCUSTOM_SOURCES
  core/src/pcl_custom/methods/explicit_instantiations.cc
)

# The explicit instantiations emit an out-of-line copy of every member and of
# the PCL base members their vtables reach. Compiled on its own, the file does
# not change the inlining context of a unity batch, and its object (linked
# after the unity batches) does not displace their COMDAT copies.
set_source_files_properties(
  core/src/pcl_custom/methods/explicit_instantiations.cc
  PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
