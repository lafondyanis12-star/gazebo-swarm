int32 sender_id
int32 claimed_leader
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 timestamp
bool[3] peer_connected  # peer_connected[i]: am I currently CONNECTED to drone_i (message-timeout
                         # based, same belief the election uses) -- not the same as being within
                         # radio range: a peer can be in range but still show as disconnected if
                         # its updates stopped arriving. Own index is unused/false.
float64 ref_vx
float64 ref_vy          # Only meaningful when sender believes itself leader: the *analytical*
                         # scripted-path velocity (loiter/line reference), not vx/vy's noisy
                         # telemetry-derived one. Followers feedforward off this instead -- it's
                         # exactly what the leader is trying to fly, without its own tracking lag
                         # or MAVSDK telemetry noise riding along. 0 when sender is a follower.
