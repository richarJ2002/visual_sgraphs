std_msgs/Header header

int32 id
bool passable
geometry_msgs/Vector3 centroid
float32 width
float32 height

# Room whose walls this passage's associated walls belong to; -1 if this
# passage is not currently associated with any mapped room's walls.
int32 known_room_id

# The far-side room this passage has been confirmed to open onto, per
# Passage::getProspectiveRoomId(); -1 if not yet known.
int32 prospective_room_id

# Wall plane IDs associated with this passage (from Passage::getAssociateWalls()).
# These are the wall planes that form the supporting structure for this passage.
int32[] associated_wall_ids
