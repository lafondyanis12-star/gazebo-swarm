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
bool ready               # True once this drone has actually reached Offboard and is flying real
                         # setpoints -- false while still connecting/arming/taking off, even if it
                         # already believes itself leader. A drone that fails to ever connect to
                         # its own PX4 instance still elects itself leader over the radio link (the
                         # election only needs message traffic, not flight) and would otherwise
                         # broadcast x/y/z frozen at 0 plus a fully-formed ref_vx/ref_vy from the
                         # scripted path it is not actually flying -- a live test traced a follower
                         # drifting 15m+ chasing exactly this "ghost leader" feedforward with no
                         # real leader motion to correct against. Followers must not trust a leader
                         # claim until this is true.
