#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Self-collision census for the Mirobot SRDF (what MoveIt Setup Assistant does).

Samples N random joint states within the URDF limits (plus home) and asks the
running move_group (/check_state_validity) which link pairs collide.
  - pairs colliding in (almost) every sample, incl. home -> mesh overlap at a
    joint; must be disabled ("Default")
  - pairs never colliding -> may be disabled ("Never") for speed
Prints SRDF lines; does not edit files.

    python3 tools/collision_census.py 2000   (with move_group.launch.py running)
"""
import random
import sys
import xml.etree.ElementTree as ET
from collections import Counter

import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

N = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
rclpy.init()
n = Node('collision_census')
cli = n.create_client(GetStateValidity, '/check_state_validity')
pc = n.create_client(GetParameters, '/move_group/get_parameters')
cli.wait_for_service(timeout_sec=10)
pc.wait_for_service(timeout_sec=10)


def call(c, req):
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    return f.result()


urdf = call(pc, GetParameters.Request(names=['robot_description'])).values[0].string_value
root = ET.fromstring(urdf)
limits = {}
for j in root.findall('joint'):
    if j.get('type') == 'revolute':
        lim = j.find('limit')
        limits[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
links = [l.get('name') for l in root.findall('link') if l.find('collision') is not None]

pairs = Counter()
home_pairs = set()
for k in range(N + 1):
    req = GetStateValidity.Request()
    req.group_name = 'mirobot_arm'
    req.robot_state.joint_state.name = list(limits)
    req.robot_state.joint_state.position = [0.0] * len(limits) if k == 0 else \
        [random.uniform(lo, hi) for lo, hi in limits.values()]
    res = call(cli, req)
    seen = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}
    if k == 0:
        home_pairs = seen
    pairs.update(seen)

print(f'# {N} random states + home; links with collision geometry: {links}')
print('# colliding at home:', sorted(home_pairs))
for (a, b), c in pairs.most_common():
    print(f'#   {a:10s} {b:10s} {c / (N + 1) * 100:5.1f} %')
print('# --- SRDF suggestions ---')
for (a, b), c in pairs.items():
    if (a, b) in home_pairs and c >= 0.95 * (N + 1):
        print(f'  <disable_collisions link1="{a}" link2="{b}" reason="Default"/>')
for i, a in enumerate(links):
    for b in links[i + 1:]:
        p = tuple(sorted((a, b)))
        if p not in pairs:
            print(f'  <disable_collisions link1="{p[0]}" link2="{p[1]}" reason="Never"/>')
