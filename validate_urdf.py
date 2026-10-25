#!/usr/bin/env python3
"""Validate the four_dof_arm URDF/Xacro model.

Usage:
    ros2 run four_dof_arm_description validate_urdf
    ros2 run four_dof_arm_description validate_urdf /path/to/model.urdf.xacro
    python3 validate_urdf.py [model.urdf | model.urdf.xacro]

Exit code 0 = all checks passed, 1 = at least one check failed.
"""
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import deque

EXPECTED_LINKS = ['base_link', 'shoulder_link', 'upper_arm_link',
                  'forearm_link', 'wrist_link', 'tool_link']

# name: (type, parent, child, axis)
EXPECTED_JOINTS = {
    'joint_1': ('revolute', 'base_link', 'shoulder_link', (0, 0, 1)),
    'joint_2': ('revolute', 'shoulder_link', 'upper_arm_link', (0, 1, 0)),
    'joint_3': ('revolute', 'upper_arm_link', 'forearm_link', (0, 1, 0)),
    'joint_4': ('revolute', 'forearm_link', 'wrist_link', (1, 0, 0)),
    'tool_fixed_joint': ('fixed', 'wrist_link', 'tool_link', None),
}
ALLOWED_GEOMETRY = ('box', 'cylinder')


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, ok, message, detail=''):
        tag = 'PASS' if ok else 'FAIL'
        print(f'  [{tag}] {message}' + (f'  ({detail})' if detail else ''))
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        return ok


def floats(text, n):
    parts = [float(v) for v in (text or '').split()]
    if len(parts) != n:
        raise ValueError(f'expected {n} numbers, got "{text}"')
    return parts


def default_model_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('four_dof_arm_description')
        path = os.path.join(share, 'urdf', 'four_dof_arm.urdf.xacro')
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, '..', 'urdf', 'four_dof_arm.urdf.xacro')


def load_urdf_text(path):
    """Return URDF XML text; expands .xacro files with the xacro tool."""
    if not path.endswith('.xacro'):
        with open(path, encoding='utf-8') as f:
            return f.read()
    try:
        import xacro
        return xacro.process_file(path).toxml()
    except ImportError:
        pass
    exe = shutil.which('xacro')
    if exe is None:
        raise RuntimeError('xacro not found. Source ROS 2 first: '
                           'source /opt/ros/humble/setup.bash')
    return subprocess.run([exe, path], check=True, capture_output=True,
                          text=True).stdout


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = os.path.abspath(argv[0] if argv else default_model_path())
    r = Report()
    print(f'Validating: {path}\n')

    # ---- 1. XML validity -------------------------------------------------
    print('1. XML validity')
    try:
        root = ET.fromstring(load_urdf_text(path))
    except Exception as exc:  # noqa: BLE001
        r.check(False, 'Model loads and parses as XML', str(exc))
        print(f'\nRESULT: {r.passed} passed, {r.failed} failed')
        return 1
    r.check(True, 'Model loads and parses as well-formed XML')
    r.check(root.tag == 'robot', 'Root element is <robot>')

    links = {l.get('name'): l for l in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}
    materials = {}
    for m in root.findall('material'):
        c = m.find('color')
        if c is not None:
            materials[m.get('name')] = tuple(floats(c.get('rgba'), 4))

    # ---- 2. Required links / joints -------------------------------------
    print('\n2. Required links and joints')
    r.check(len(links) == len(root.findall('link')), 'Link names are unique')
    r.check(len(joints) == len(root.findall('joint')), 'Joint names are unique')
    for name in EXPECTED_LINKS:
        r.check(name in links, f'Link present: {name}')
    r.check(set(links) == set(EXPECTED_LINKS),
            'No unexpected links', f'found {sorted(links)}')
    for name in EXPECTED_JOINTS:
        r.check(name in joints, f'Joint present: {name}')
    r.check(set(joints) == set(EXPECTED_JOINTS),
            'No unexpected joints', f'found {sorted(joints)}')

    # ---- 3/4. Parent-child, types, counts --------------------------------
    print('\n3. Joint types and parent-child relationships')
    types = [j.get('type') for j in joints.values()]
    r.check(types.count('revolute') == 4, 'Exactly 4 revolute joints',
            f'found {types.count("revolute")}')
    r.check(types.count('fixed') == 1, 'Exactly 1 fixed joint',
            f'found {types.count("fixed")}')
    r.check(len(types) == 5, 'Exactly 5 joints in total')
    for name, (jtype, parent, child, _axis) in EXPECTED_JOINTS.items():
        j = joints.get(name)
        if j is None:
            continue
        p = j.find('parent').get('link') if j.find('parent') is not None else None
        c = j.find('child').get('link') if j.find('child') is not None else None
        r.check(j.get('type') == jtype, f'{name} type is {jtype}', j.get('type'))
        r.check((p, c) == (parent, child),
                f'{name}: {parent} -> {child}', f'found {p} -> {c}')

    # ---- 5. Axes, limits, origins ---------------------------------------
    print('\n4. Joint axes, limits and origins')
    for name, (jtype, _p, _c, axis) in EXPECTED_JOINTS.items():
        j = joints.get(name)
        if j is None or jtype != 'revolute':
            continue
        ax = j.find('axis')
        try:
            vec = floats(ax.get('xyz'), 3)
            norm = math.sqrt(sum(v * v for v in vec))
            unit = tuple(round(v / norm, 6) for v in vec) if norm else None
            r.check(unit == tuple(float(a) for a in axis),
                    f'{name} axis is {axis}', f'found {vec}')
        except Exception as exc:  # noqa: BLE001
            r.check(False, f'{name} has a valid <axis>', str(exc))
        lim = j.find('limit')
        if lim is None:
            r.check(False, f'{name} has <limit>')
            continue
        try:
            lo, hi = float(lim.get('lower')), float(lim.get('upper'))
            eff, vel = float(lim.get('effort')), float(lim.get('velocity'))
            r.check(lo < hi, f'{name} lower < upper', f'[{lo:.3f}, {hi:.3f}]')
            r.check(lo <= 0.0 <= hi, f'{name} zero position inside limits')
            r.check(eff > 0 and vel > 0, f'{name} effort/velocity positive',
                    f'effort={eff}, velocity={vel}')
        except (TypeError, ValueError) as exc:
            r.check(False, f'{name} limit attributes are complete numbers', str(exc))
    for name, j in joints.items():
        o = j.find('origin')
        try:
            floats(o.get('xyz'), 3)
            floats(o.get('rpy', '0 0 0'), 3)
            r.check(True, f'{name} has a valid <origin>')
        except Exception as exc:  # noqa: BLE001
            r.check(False, f'{name} has a valid <origin>', str(exc))

    # ---- 6. Geometry, materials, inertia --------------------------------
    print('\n5. Visual, collision and inertial properties')
    colours = {}
    for name in EXPECTED_LINKS:
        link = links.get(name)
        if link is None:
            continue
        for tag in ('visual', 'collision'):
            el = link.find(tag)
            geo = el.find('geometry') if el is not None else None
            shape = geo[0] if geo is not None and len(geo) else None
            ok = shape is not None and shape.tag in ALLOWED_GEOMETRY
            if ok:
                if shape.tag == 'box':
                    ok = all(v > 0 for v in floats(shape.get('size'), 3))
                else:
                    ok = float(shape.get('radius')) > 0 and float(shape.get('length')) > 0
            r.check(ok, f'{name} has valid {tag} geometry (box/cylinder)',
                    shape.tag if shape is not None else 'missing')
        mat = link.find('visual/material')
        mname = mat.get('name') if mat is not None else None
        col = materials.get(mname)
        if col is None and mat is not None and mat.find('color') is not None:
            col = tuple(floats(mat.find('color').get('rgba'), 4))
        r.check(col is not None, f'{name} has a coloured material', str(mname))
        colours[name] = col
        inert = link.find('inertial')
        try:
            mass = float(inert.find('mass').get('value'))
            i = inert.find('inertia')
            ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
            tri = ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx
            r.check(mass > 0, f'{name} mass > 0', f'{mass} kg')
            r.check(ixx > 0 and iyy > 0 and izz > 0 and tri,
                    f'{name} inertia positive and physically valid',
                    f'{ixx:.2e}, {iyy:.2e}, {izz:.2e}')
        except Exception as exc:  # noqa: BLE001
            r.check(False, f'{name} has valid <inertial>', str(exc))
    known = [c for c in colours.values() if c is not None]
    r.check(len(known) == len(set(known)) == len(EXPECTED_LINKS),
            'Every link has a different colour')

    # ---- 7. Connectivity -------------------------------------------------
    print('\n6. Link connectivity')
    children = {}
    parent_of = {}
    multi_parent = False
    for j in joints.values():
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        children.setdefault(p, []).append(c)
        if c in parent_of:
            multi_parent = True
        parent_of[c] = p
    roots = [l for l in links if l not in parent_of]
    r.check(roots == ['base_link'], 'base_link is the only root', f'roots={roots}')
    r.check(not multi_parent, 'Every link has at most one parent')
    seen, queue = set(), deque(['base_link'])
    while queue:
        cur = queue.popleft()
        if cur in seen:
            continue
        seen.add(cur)
        queue.extend(children.get(cur, []))
    r.check(seen == set(links), 'All links reachable from base_link (no orphans)')
    r.check(len(joints) == len(links) - 1, 'Joint count = link count - 1 (a tree)')
    chain, cur = ['base_link'], 'base_link'
    while children.get(cur):
        cur = children[cur][0]
        chain.append(cur)
    r.check(chain == EXPECTED_LINKS, 'Kinematic chain order',
            ' -> '.join(chain))

    # ---- zero-pose tool position (informational) -------------------------
    try:
        z = sum(floats(j.find('origin').get('xyz'), 3)[2] for j in joints.values())
        print(f'\nInfo: at zero pose, tool_link origin is {z:.3f} m above base_link '
              '(expected from tf2_echo when all joints = 0).')
    except Exception:  # noqa: BLE001
        pass

    print(f'\nRESULT: {r.passed} passed, {r.failed} failed')
    return 0 if r.failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
