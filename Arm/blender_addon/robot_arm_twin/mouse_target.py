"""
mouse_target.py -- follow the mouse across a target plane, through the
existing IK solver.

Adds nothing to the maths: the mouse is converted to a world-space XYZ point
and handed to `inverse_kinematics.solve()`, the same solver the XYZ panel
uses.  There is exactly one IK implementation in this project.

The pipeline
------------
    mouse pixel  ->  viewport ray  ->  intersection with the target plane
                 ->  world XYZ     ->  existing IK  ->  joint angles
                 ->  Blender model ->  (only if Physical Sync is on) Arduino

Scene objects
-------------
Two objects are created on demand, in their own collection so they are easy
to find and delete:

    ArmTargetPlane   the surface the mouse is projected onto
    ArmTargetMarker  an empty showing exactly where the arm is aiming

Neither is referenced by the arm config, so forward kinematics, the rest-pose
capture and the save handlers all ignore them completely.
"""

from __future__ import annotations

import math

try:
    import bpy
except ImportError:          # plain CPython: the pure geometry still imports,
    bpy = None               # which is what lets selftest.py exercise it

COLLECTION_NAME = "RobotArmTargets"
PLANE_NAME = "ArmTargetPlane"
MARKER_NAME = "ArmTargetMarker"

PLANE_COLOUR = (0.05, 0.40, 0.90, 1.0)
MARKER_SIZE = 0.18

# A ray nearly parallel to the plane produces a wildly distant, useless hit.
MIN_RAY_COS = 1.0e-4


# --------------------------------------------------------------------------
# geometry -- pure python, no bpy, so it can be tested headlessly
# --------------------------------------------------------------------------

def ray_plane_intersect(origin, direction, plane_point, plane_normal,
                        max_distance=1.0e6):
    """
    Where a ray meets an infinite plane.

    Returns (x, y, z) or None.  None means the ray is parallel to the plane,
    points away from it, or the hit is absurdly far away -- all of which must
    be treated as "no target", never as a position to drive an arm to.
    """
    ox, oy, oz = (float(c) for c in origin)
    dx, dy, dz = (float(c) for c in direction)
    px, py, pz = (float(c) for c in plane_point)
    nx, ny, nz = (float(c) for c in plane_normal)

    n_len = math.sqrt(nx * nx + ny * ny + nz * nz)
    if n_len < 1e-12:
        return None
    nx, ny, nz = nx / n_len, ny / n_len, nz / n_len

    d_len = math.sqrt(dx * dx + dy * dy + dz * dz)
    if d_len < 1e-12:
        return None
    dx, dy, dz = dx / d_len, dy / d_len, dz / d_len

    denom = dx * nx + dy * ny + dz * nz
    if abs(denom) < MIN_RAY_COS:
        return None                      # parallel: grazing the plane edge-on

    t = ((px - ox) * nx + (py - oy) * ny + (pz - oz) * nz) / denom
    if t < 0.0 or t > max_distance:
        return None                      # behind the viewer, or effectively at infinity

    return (ox + dx * t, oy + dy * t, oz + dz * t)


def plane_frame(plane_obj):
    """
    (point_on_plane, unit_normal) in world space.

    The plane's local +Z is its normal, so moving, rotating or scaling the
    plane object in Blender changes the target surface with no code changes.
    """
    m = plane_obj.matrix_world
    point = (m[0][3], m[1][3], m[2][3])
    normal = (m[0][2], m[1][2], m[2][2])
    return point, normal


# --------------------------------------------------------------------------
# scene objects
# --------------------------------------------------------------------------

def _collection(scene):
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION_NAME)
    if coll.name not in scene.collection.children:
        try:
            scene.collection.children.link(coll)
        except RuntimeError:
            pass
    return coll


def _make_plane_material():
    mat = bpy.data.materials.get(PLANE_NAME + "Mat")
    if mat is not None:
        return mat
    mat = bpy.data.materials.new(PLANE_NAME + "Mat")
    mat.diffuse_color = PLANE_COLOUR          # what Solid shading shows
    try:
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf is not None and "Base Color" in bsdf.inputs:
            bsdf.inputs["Base Color"].default_value = PLANE_COLOUR
    except Exception:
        pass                                   # cosmetic only, never fatal
    return mat


def create_plane(scene, centre, half_size=2.0):
    """A square target plane, horizontal, centred on `centre`."""
    obj = bpy.data.objects.get(PLANE_NAME)
    if obj is None:
        s = float(half_size)
        mesh = bpy.data.meshes.new(PLANE_NAME)
        mesh.from_pydata([(-s, -s, 0.0), (s, -s, 0.0), (s, s, 0.0), (-s, s, 0.0)],
                         [], [(0, 1, 2, 3)])
        mesh.update()
        obj = bpy.data.objects.new(PLANE_NAME, mesh)
        obj.data.materials.append(_make_plane_material())
        obj.show_wire = True                   # visible even in Solid shading
        obj.color = PLANE_COLOUR
    obj.location = tuple(float(c) for c in centre)

    coll = _collection(scene)
    if obj.name not in coll.objects:
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        coll.objects.link(obj)
    return obj


def create_marker(scene, location):
    """A small empty sphere showing where the arm is aiming."""
    obj = bpy.data.objects.get(MARKER_NAME)
    if obj is None:
        obj = bpy.data.objects.new(MARKER_NAME, None)
        obj.empty_display_type = 'SPHERE'
        obj.empty_display_size = MARKER_SIZE
        obj.show_in_front = True
        obj.color = (1.0, 0.35, 0.05, 1.0)
    obj.location = tuple(float(c) for c in location)

    coll = _collection(scene)
    if obj.name not in coll.objects:
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        coll.objects.link(obj)
    return obj


def ensure_targets(scene, geom, half_size=2.0):
    """
    Make sure a plane and marker exist, sized to the real workspace.

    Placed at the rest tip so the default plane sits where the arm can
    actually reach, rather than somewhere arbitrary.
    """
    tip = geom.tip((0.0, 0.0, 0.0))
    plane = create_plane(scene, tip, half_size)
    marker = create_marker(scene, tip)
    return plane, marker


def remove_targets(scene):
    """Delete both helper objects and their collection."""
    removed = []
    for name in (PLANE_NAME, MARKER_NAME):
        obj = bpy.data.objects.get(name)
        if obj is not None:
            bpy.data.objects.remove(obj, do_unlink=True)
            removed.append(name)
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is not None and not coll.objects:
        bpy.data.collections.remove(coll)
    return removed


def move_marker(scene, location, visible=True):
    obj = bpy.data.objects.get(MARKER_NAME)
    if obj is None:
        return None
    obj.location = tuple(float(c) for c in location)
    obj.hide_viewport = not visible
    return obj


# --------------------------------------------------------------------------
# viewport -> ray
# --------------------------------------------------------------------------

def region_under_mouse(window, mouse_x, mouse_y):
    """
    Find the 3D viewport the cursor is over.

    The modal handler receives window-space coordinates, and the user may
    have several viewports; picking the one under the cursor is what makes
    the mode behave the way a mouse is expected to.
    """
    screen = getattr(window, "screen", None)
    if screen is None:
        return None, None
    for area in screen.areas:
        if area.type != 'VIEW_3D':
            continue
        for region in area.regions:
            if region.type != 'WINDOW':
                continue
            if (region.x <= mouse_x < region.x + region.width and
                    region.y <= mouse_y < region.y + region.height):
                space = area.spaces.active
                rv3d = getattr(space, "region_3d", None)
                if rv3d is not None:
                    return region, rv3d
    return None, None


def mouse_to_plane(region, rv3d, mouse_x, mouse_y, plane_obj):
    """
    Project a window-space mouse position onto the target plane.

    Returns (x, y, z) in world space, or None if the ray misses.
    """
    from bpy_extras import view3d_utils

    coord = (mouse_x - region.x, mouse_y - region.y)
    try:
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
        direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    except Exception:
        return None
    if origin is None or direction is None:
        return None

    point, normal = plane_frame(plane_obj)
    return ray_plane_intersect(origin, direction, point, normal)
