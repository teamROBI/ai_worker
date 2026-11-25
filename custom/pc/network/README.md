# Robot <-> PC ROS 2 over the direct gigabit cable

## Topology
- PC (lunar): `enp68s0 = 192.168.50.1` (gigabit cable), `enp71s0 = 192.168.1.x` (LAN/default route)
- Robot (ffw-SNPR48A1016): `eth1 = 192.168.50.2` (cable), `eth0 = 192.168.6.x` (default route + lidars)
- All ROS 2 runs in Docker (host networking), `ROS_DOMAIN_ID=30`.

> **Outdated since ai_worker 2.0.0.** The RMW is now `rmw_zenoh_cpp`, not FastDDS, so the
> DDS-specific findings below no longer describe how the PC reaches the robot. The ufw
> lesson still holds (zenoh's TCP 7447 must get through), but routers on different hosts
> are not linked automatically — see `custom/pc/control/README.md` → "Zenoh networking".

## The problem we hit
ROS 2 discovery worked over the slow `192.168.6.x` path but NOT over the direct `192.168.50.x` cable.
`ssh` and `ping` worked over the cable, so it looked like a DDS/multicast problem.

## The actual root cause
**`ufw` on the PC was dropping all new inbound packets on the cable.**
- PC -> robot worked (robot isn't firewalled); robot -> PC was silently dropped.
- ping worked only because ufw lets the *replies* back through connection tracking.
- DDS discovery needs the robot's announcements to reach the PC -> blocked -> no topics.

Everything that looked like a multicast/route/RMW problem was just packets being discarded on arrival.

## The fix (one command on the PC)
    sudo ufw allow in on enp68s0      # or: sudo ufw allow from 192.168.50.0/24
ufw rules persist across reboots automatically.

## Verify
    ros2 daemon stop && ros2 topic list      # robot topics appear
Heavy data (camera) then rides the cable: confirmed ~32 MB/5s on enp68s0 vs ~2 KB on enp71s0.

## NOT needed
No FastDDS XML profiles, no interface whitelist, no discovery server, no `ROS_DISCOVERY_SERVER`.
Stock config just works once the firewall allows the cable subnet.
