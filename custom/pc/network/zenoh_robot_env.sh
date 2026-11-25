# Join the robot's zenoh network. SOURCE this (don't execute) in a container shell
# before launching anything that must talk to the robot -- the leader bringup,
# teleop, rviz, ros2 CLI:
#
#   source /root/ros2_ws/src/ai_worker/custom/pc/network/zenoh_robot_env.sh
#   ffw_lg2_leader_ai
#
# Why: since ai_worker 2.0.0 the RMW is rmw_zenoh_cpp. Every host runs its own
# router on :7447 and routers on different hosts are NOT linked, so a leader on
# this PC publishes into the PC's router while the follower listens on the
# robot's -- the topics never meet. This points our nodes at the ROBOT's router
# as zenoh CLIENTS, so both ends share one router.
#
# Client (not peer) is deliberate: the robot's nodes only listen on their own
# loopback, so a peer has no route to them; a client relays through the router.
#
#   ROBOT_IP=<ip>  override (the container has no mDNS for *.local)

_zre_ip="${ROBOT_IP:-}"
if [ -z "$_zre_ip" ]; then
    _zre_ip="$(getent hosts "${ROBOT_HOST:-ffw-SNPR48A1016.local}" 2>/dev/null | awk '{print $1; exit}')"
fi

if [ -z "$_zre_ip" ]; then
    echo "zenoh_robot_env: cannot resolve the robot; re-run as ROBOT_IP=<ip> source $BASH_SOURCE" >&2
elif ! timeout 5 bash -c "cat < /dev/null > /dev/tcp/$_zre_ip/7447" 2>/dev/null; then
    echo "zenoh_robot_env: no zenoh router at $_zre_ip:7447 -- run 'zenohd' on the robot first." >&2
else
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/$_zre_ip:7447\"]"
    echo "zenoh_robot_env: joined robot router $_zre_ip:7447 (domain $ROS_DOMAIN_ID)"
    # The ros2 CLI daemon caches a graph from whatever RMW it started under.
    ros2 daemon stop >/dev/null 2>&1 || true
fi
unset _zre_ip
