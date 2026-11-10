"""Build the GodView demo scene in Blender from a D4 scene package.

Run with Blender 5.2 (headless) or the `bpy` module:

    blender -b --python build_scene.py -- --package DIR --outdir OUT \
        [--res 1920x1080] [--samples 64] [--device GPU] [--stills 130,160,190] \
        [--animation] [--save scene.blend]

What is drawn, all from the package:
* every object in vehicle 1's GodView ego view within --radius metres: solid
  (vehicle 1 itself contributes) or ghost (known only through other agents);
* a halo under each ghost, the 2-sigma ellipse of its position covariance;
* arcs from each contributing agent to the focus objects while they are ghosts
  (one arc per agent that reported the object in that frame);
* the four agents from their poses (two vehicles, two roadside units);
* the static lidar scan of the street as glowing points.
Nothing is added that GodView did not output.
"""

import argparse
import math
import sys
from pathlib import Path

import bpy  # noqa: I001  (bpy must load before bmesh when used as a module)
import bmesh
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scene_data import VEHICLE_SIZE, Scene  # noqa: E402

CYAN = (0.20, 0.85, 1.00)
AMBER = (1.00, 0.55, 0.15)
VIOLET = (0.60, 0.35, 1.00)


# ----------------------------------------------------------------------------
# arguments


def parse_args():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else argv[1:]
    p = argparse.ArgumentParser()
    p.add_argument("--package", required=True)
    p.add_argument("--outdir", default="renders")
    p.add_argument("--res", default="960x540")
    p.add_argument("--samples", type=int, default=32)
    p.add_argument("--device", default="CPU", choices=["CPU", "GPU"])
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--radius", type=float, default=60.0)
    p.add_argument("--points", type=int, default=200000)
    p.add_argument("--focus", default="58,30,46")
    p.add_argument("--stills", default="")
    p.add_argument("--animation", action="store_true")
    p.add_argument("--save", default="")
    p.add_argument("--shot", default="hero", choices=["hero", "top", "egotop", "orbit"])
    p.add_argument("--freeze", type=int, default=0,
                   help="dataset frame to pause time at; the camera still moves")
    p.add_argument("--orbit-frames", type=int, default=150)
    return p.parse_args(argv)


# ----------------------------------------------------------------------------
# helpers


def new_material(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    return m, nt, out


def mat_principled(name, color, rough=0.35, coat=0.6, metallic=0.0):
    m, nt, out = new_material(name)
    b = nt.nodes.new("ShaderNodeBsdfPrincipled")
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metallic
    if "Coat Weight" in b.inputs:
        b.inputs["Coat Weight"].default_value = coat
    nt.links.new(b.outputs[0], out.inputs["Surface"])
    return m


def mat_emission(name, color, strength):
    m, nt, out = new_material(name)
    e = nt.nodes.new("ShaderNodeEmission")
    e.inputs["Color"].default_value = (*color, 1)
    e.inputs["Strength"].default_value = strength
    nt.links.new(e.outputs[0], out.inputs["Surface"])
    return m


def mat_ghost(name, color, strength=3.0):
    """Translucent hologram: bright at grazing angles, see-through face-on."""
    m, nt, out = new_material(name)
    lw = nt.nodes.new("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.3
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.03, 0.03, 0.03, 1)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (1, 1, 1, 1)
    knee = ramp.color_ramp.elements.new(0.75)
    knee.color = (0.22, 0.22, 0.22, 1)
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*color, 1)
    em.inputs["Strength"].default_value = strength
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(lw.outputs["Facing"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], mix.inputs["Fac"])
    nt.links.new(tr.outputs[0], mix.inputs[1])
    nt.links.new(em.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs["Surface"])
    return m


def mat_radial(name, color, strength=2.0, fill=0.12):
    """Ellipse marker: a bright rim with a faint fill, for halos and agent rings."""
    m, nt, out = new_material(name)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = "SPHERICAL"
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = 0.0, (0, 0, 0, 1)
    els[1].position, els[1].color = 1.0, (fill * 0.6, fill * 0.6, fill * 0.6, 1)
    rim = els.new(0.025)
    rim.color = (1, 1, 1, 1)
    inner = els.new(0.10)
    inner.color = (fill, fill, fill, 1)
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*color, 1)
    em.inputs["Strength"].default_value = strength
    tr = nt.nodes.new("ShaderNodeBsdfTransparent")
    mix = nt.nodes.new("ShaderNodeMixShader")
    nt.links.new(tc.outputs["Object"], grad.inputs["Vector"])
    nt.links.new(grad.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], mix.inputs["Fac"])
    nt.links.new(tr.outputs[0], mix.inputs[1])
    nt.links.new(em.outputs[0], mix.inputs[2])
    nt.links.new(mix.outputs[0], out.inputs["Surface"])
    return m


def mat_points(name, z0, z1):
    """Lidar points coloured by height, deep blue at street level to pale at roof level."""
    m, nt, out = new_material(name)
    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    mr = nt.nodes.new("ShaderNodeMapRange")
    mr.inputs["From Min"].default_value = z0
    mr.inputs["From Max"].default_value = z1
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    els = ramp.color_ramp.elements
    els[0].position, els[0].color = 0.0, (0.03, 0.10, 0.45, 1)
    els[1].position, els[1].color = 1.0, (0.75, 0.80, 1.00, 1)
    mid = els.new(0.35)
    mid.color = (0.25, 0.20, 0.85, 1)
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Strength"].default_value = 1.2
    nt.links.new(tc.outputs["Object"], sep.inputs[0])
    nt.links.new(sep.outputs["Z"], mr.inputs["Value"])
    nt.links.new(mr.outputs["Result"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], em.inputs["Color"])
    nt.links.new(em.outputs[0], out.inputs["Surface"])
    return m


def add_box(bm, centre, size, taper_top=None):
    geom = bmesh.ops.create_cube(bm, size=1.0)
    verts = geom["verts"]
    bmesh.ops.scale(bm, vec=Vector(size), verts=verts)
    if taper_top:
        top = [v for v in verts if v.co.z > 0]
        bmesh.ops.scale(bm, vec=Vector((taper_top[0], taper_top[1], 1.0)), verts=top)
        if len(taper_top) > 2:
            bmesh.ops.translate(bm, vec=Vector((taper_top[2], 0, 0)), verts=top)
    bmesh.ops.translate(bm, vec=Vector(centre), verts=verts)
    return verts


def add_wheel(bm, x, y, r, width):
    geom = bmesh.ops.create_cone(bm, cap_ends=True, segments=28, radius1=r, radius2=r,
                                 depth=width,
                                 matrix=Matrix.Translation((x, y, r)) @ Matrix.Rotation(math.pi / 2, 4, "X"))
    return geom["verts"]


def finish_mesh(bm, name, bevel=0.06, segments=3):
    edges = [e for e in bm.edges]
    bmesh.ops.bevel(bm, geom=edges, offset=bevel, segments=segments, profile=0.5,
                    affect="EDGES", clamp_overlap=True)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    for poly in me.polygons:
        poly.use_smooth = True
    return me


def extrude_profile(bm, pts, width):
    """Side profile (x forward, z up) extruded across y, centred on y = 0."""
    verts = [bm.verts.new((x, -width / 2.0, z)) for x, z in pts]
    face = bm.faces.new(verts)
    res = bmesh.ops.extrude_face_region(bm, geom=[face])
    new = [g for g in res["geom"] if isinstance(g, bmesh.types.BMVert)]
    bmesh.ops.translate(bm, vec=Vector((0, width, 0)), verts=new)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])


def mesh_car():
    """Generic sedan, 4.5 x 2.0 x 1.5 m, origin at the ground centre, facing +x."""
    bm = bmesh.new()
    extrude_profile(bm, [(2.25, 0.30), (2.27, 0.64), (2.02, 0.85), (0.80, 0.97),
                         (-1.92, 1.00), (-2.25, 0.86), (-2.25, 0.30)], 1.96)
    body = finish_mesh(bm, "car_body", bevel=0.14, segments=5)
    bm = bmesh.new()
    extrude_profile(bm, [(0.86, 0.94), (0.12, 1.45), (-1.06, 1.48), (-1.84, 0.98)], 1.64)
    glass = finish_mesh(bm, "car_glass", bevel=0.10, segments=4)
    bm = bmesh.new()
    for x in (1.42, -1.42):
        for y in (0.86, -0.86):
            add_wheel(bm, x, y, 0.35, 0.24)
    wheels = finish_mesh(bm, "car_wheels", bevel=0.04, segments=2)
    return body, wheels, glass


def mesh_truck():
    """Generic box truck, 7.0 x 2.6 x 3.0 m."""
    bm = bmesh.new()
    extrude_profile(bm, [(3.50, 0.55), (3.50, 1.45), (3.20, 2.45), (2.00, 2.60), (1.95, 0.55)], 2.40)
    add_box(bm, (-0.85, 0.0, 1.90), (5.30, 2.60, 2.25))
    add_box(bm, (0.0, 0.0, 0.62), (6.9, 2.2, 0.32))
    body = finish_mesh(bm, "truck_body", bevel=0.10, segments=4)
    bm = bmesh.new()
    extrude_profile(bm, [(3.44, 1.55), (3.18, 2.38), (2.45, 2.48), (2.45, 1.55)], 2.30)
    glass = finish_mesh(bm, "truck_glass", bevel=0.06, segments=3)
    bm = bmesh.new()
    for x in (2.6, -1.7, -2.8):
        for y in (1.05, -1.05):
            add_wheel(bm, x, y, 0.47, 0.32)
    wheels = finish_mesh(bm, "truck_wheels", bevel=0.04, segments=2)
    return body, wheels, glass


def mesh_person():
    """Generic standing figure, 1.75 m, facing +x, built from ellipsoids."""
    bm = bmesh.new()

    def blob(centre, radii):
        g = bmesh.ops.create_uvsphere(bm, u_segments=18, v_segments=12, radius=1.0)
        bmesh.ops.scale(bm, vec=Vector(radii), verts=g["verts"])
        bmesh.ops.translate(bm, vec=Vector(centre), verts=g["verts"])
    blob((0.0, 0.10, 0.46), (0.085, 0.085, 0.46))
    blob((0.0, -0.10, 0.46), (0.085, 0.085, 0.46))
    blob((0.0, 0.0, 1.18), (0.14, 0.22, 0.34))
    blob((0.0, 0.29, 1.12), (0.055, 0.055, 0.32))
    blob((0.0, -0.29, 1.12), (0.055, 0.055, 0.32))
    blob((0.02, 0.0, 1.63), (0.11, 0.10, 0.125))
    me = bpy.data.meshes.new("person")
    bm.to_mesh(me)
    bm.free()
    for poly in me.polygons:
        poly.use_smooth = True
    return me, None, None


CANON = {1: (4.5, 2.0, 1.5), 2: (0.5, 0.5, 1.75), 3: (7.0, 2.6, 3.0)}


def set_mat(obj, mat):
    """Assign a material to this object only (meshes are shared between objects)."""
    if len(obj.data.materials) == 0:
        obj.data.materials.append(None)
    slot = obj.material_slots[0]
    slot.link = "OBJECT"
    slot.material = mat


def make_object(name, meshes, mats, collection):
    """Root empty with body, wheels and glass parts; mats is (body, wheels, glass)."""
    root = bpy.data.objects.new(name, None)
    collection.objects.link(root)
    parts = []
    for suffix, me, mat in zip(("body", "wheels", "glass"), meshes, mats):
        if me is None:
            continue
        o = bpy.data.objects.new(f"{name}_{suffix}", me)
        collection.objects.link(o)
        set_mat(o, mat)
        o.parent = root
        parts.append(o)
    return root, parts


def key_visible(parts, frame, visible):
    for p in parts:
        p.hide_render = not visible
        p.hide_viewport = not visible
        p.keyframe_insert("hide_render", frame=frame)
        p.keyframe_insert("hide_viewport", frame=frame)


# ----------------------------------------------------------------------------
# scene


def build(args):
    sc = Scene(args.package)
    focus_labels = [g for g in args.focus.split(",") if g]
    focus_ids = set(sc.focus_tracks(focus_labels))
    f0 = sc.frame_ids[0]
    step = args.fps // 10

    freeze = args.freeze

    def bf(frame):
        return 1 if freeze else 1 + (frame - f0) * step

    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_end = args.orbit_frames if freeze else bf(sc.frame_ids[-1])
    scene.render.fps = args.fps
    prefs = bpy.context.preferences.edit
    prefs.keyframe_new_interpolation_type = "LINEAR"

    col = bpy.data.collections.new("godview")
    scene.collection.children.link(col)
    gz = sc.ground_z
    ego0 = sc.ego_state(freeze or f0)
    ex, ey = ego0["centre"][0], ego0["centre"][1]

    # materials
    m_solid = mat_principled("solid", (0.55, 0.57, 0.60), rough=0.28, coat=0.9)
    m_ego = mat_principled("ego", (0.90, 0.88, 0.84), rough=0.18, coat=1.0)
    m_glass = mat_principled("glass", (0.015, 0.018, 0.025), rough=0.08, coat=1.0)
    m_agent = mat_principled("agent", (0.55, 0.52, 0.66), rough=0.25, coat=1.0)
    m_wheel = mat_principled("wheel", (0.02, 0.02, 0.025), rough=0.6, coat=0.0)
    m_ghost = mat_ghost("ghost", CYAN, 2.2)
    m_ghost_wheel = mat_ghost("ghost_wheel", CYAN, 1.0)
    m_halo = mat_radial("halo", CYAN, 1.1, fill=0.05)
    m_ring_ego = mat_radial("ring_ego", AMBER, 3.0, fill=0.25)
    m_ring_agent = mat_radial("ring_agent", VIOLET, 2.5, fill=0.2)
    m_arc = mat_emission("arc", CYAN, 3.0)
    m_pole = mat_principled("pole", (0.08, 0.08, 0.1), rough=0.4, coat=0.2)
    m_head = mat_emission("sensor", VIOLET, 8.0)
    m_ground = mat_principled("ground", (0.007, 0.008, 0.012), rough=0.92, coat=0.0)

    # world: dusk
    world = bpy.data.worlds.new("dusk")
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.020, 0.028, 0.055, 1)
    bg.inputs["Strength"].default_value = 1.0

    # dusk lighting: a low warm sun, a cool fill from the opposite side, sky ambient
    sun = bpy.data.lights.new("sun", "SUN")
    sun.energy = 2.2
    sun.color = (1.0, 0.82, 0.62)
    sun.angle = math.radians(14)
    sun_obj = bpy.data.objects.new("sun", sun)
    sun_obj.rotation_euler = (math.radians(62), 0, math.radians(35))
    scene.collection.objects.link(sun_obj)
    fill = bpy.data.lights.new("fill", "SUN")
    fill.energy = 0.8
    fill.color = (0.55, 0.65, 1.0)
    fill_obj = bpy.data.objects.new("fill", fill)
    fill_obj.rotation_euler = (math.radians(50), 0, math.radians(215))
    scene.collection.objects.link(fill_obj)

    # ground
    bpy.ops.mesh.primitive_plane_add(size=600, location=(ex, ey, gz))
    ground = bpy.context.active_object
    ground.active_material = m_ground

    # static lidar scan as points
    pts = sc.static_points(args.points)
    pts = pts[np.hypot(pts[:, 0] - ex, pts[:, 1] - ey) < args.radius * 1.6]
    me = bpy.data.meshes.new("scan")
    me.vertices.add(len(pts))
    me.vertices.foreach_set("co", pts.astype(np.float32).ravel())
    scan = bpy.data.objects.new("scan", me)
    col.objects.link(scan)
    ng = bpy.data.node_groups.new("to_points", "GeometryNodeTree")
    ng.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    gi = ng.nodes.new("NodeGroupInput")
    go = ng.nodes.new("NodeGroupOutput")
    m2p = ng.nodes.new("GeometryNodeMeshToPoints")
    m2p.inputs["Radius"].default_value = 0.035
    sm = ng.nodes.new("GeometryNodeSetMaterial")
    sm.inputs["Material"].default_value = mat_points("scan", gz, gz + 14.0)
    ng.links.new(gi.outputs[0], m2p.inputs["Mesh"])
    ng.links.new(m2p.outputs["Points"], sm.inputs["Geometry"])
    ng.links.new(sm.outputs["Geometry"], go.inputs[0])
    mod = scan.modifiers.new("points", "NODES")
    mod.node_group = ng

    meshes = {1: mesh_car(), 2: mesh_person(), 3: mesh_truck()}

    # disc mesh for halos and rings (radius 1)
    bm = bmesh.new()
    bmesh.ops.create_circle(bm, cap_ends=True, segments=48, radius=1.0)
    disc = bpy.data.meshes.new("disc")
    bm.to_mesh(disc)
    bm.free()

    # agents
    for idx, at in sc.agent_tracks.items():
        s0 = at["states"][0]
        if at["kind"] == "vehicle":
            mats = (m_ego if idx == sc.ego_index else m_agent, m_wheel, m_glass)
            root, parts = make_object(f"agent{idx}", meshes[1], mats, col)
            root.scale = (VEHICLE_SIZE[0] / 4.5, VEHICLE_SIZE[1] / 2.0, 1.0)
            ring = bpy.data.objects.new(f"ring{idx}", disc)
            col.objects.link(ring)
            set_mat(ring, m_ring_ego if idx == sc.ego_index else m_ring_agent)
            ring.scale = (3.4, 2.2, 1)
            for s in at["states"]:
                f = bf(s["frame"])
                root.location = (s["centre"][0], s["centre"][1], gz)
                root.rotation_euler = (0, 0, math.radians(s["yaw_deg"]))
                root.keyframe_insert("location", frame=f)
                root.keyframe_insert("rotation_euler", frame=f)
                ring.location = (s["centre"][0], s["centre"][1], gz + 0.02)
                ring.rotation_euler = (0, 0, math.radians(s["yaw_deg"]))
                ring.keyframe_insert("location", frame=f)
                ring.keyframe_insert("rotation_euler", frame=f)
        else:
            sx, sy, sz = s0["sensor"]
            h = max(sz - gz, 1.0)
            bpy.ops.mesh.primitive_cylinder_add(radius=0.12, depth=h, location=(sx, sy, gz + h / 2))
            bpy.context.active_object.active_material = m_pole
            bpy.ops.mesh.primitive_uv_sphere_add(radius=0.28, location=(sx, sy, sz))
            bpy.context.active_object.active_material = m_head
            ring = bpy.data.objects.new(f"ring{idx}", disc)
            col.objects.link(ring)
            set_mat(ring, m_ring_agent)
            ring.scale = (2.0, 2.0, 1)
            ring.location = (sx, sy, gz + 0.02)

    # GodView objects
    frames_all = [freeze] if freeze else sc.frame_ids
    # In paused time, tell the story of the instant: first only what the car
    # sees itself, then the reports arrive (arcs grow), then the ghosts appear.
    arc_start, arc_end = int(scene.frame_end * 0.28), int(scene.frame_end * 0.48)
    ghost_on = int(scene.frame_end * 0.46)
    late_parts = []
    for tid, t in sc.tracks.items():
        states = [s for s in t["states"]
                  if math.hypot(s["pos"][0] - ex, s["pos"][1] - ey) < args.radius]
        if not states:
            continue
        cls = t["class_id"]
        by_frame = {s["frame"]: s for s in states}
        L0, W0, H0 = CANON[cls]
        solid, solid_parts = make_object(f"{tid}_solid", meshes[cls], (m_solid, m_wheel, m_glass), col)
        ghost, ghost_parts = make_object(f"{tid}_ghost", meshes[cls], (m_ghost, m_ghost_wheel, m_ghost), col)
        halo = bpy.data.objects.new(f"{tid}_halo", disc)
        col.objects.link(halo)
        set_mat(halo, m_halo)
        for fr in frames_all:
            f = bf(fr)
            s = by_frame.get(fr)
            if s is None:
                key_visible(solid_parts + ghost_parts + [halo], f, False)
                continue
            l, w, h = s["size"]
            if cls == 2:
                scale = (1.0, 1.0, h / H0)
            else:
                scale = (l / L0, w / W0, h / H0)
            loc = (s["pos"][0], s["pos"][1], s["pos"][2] - h / 2.0)
            for obj in (solid, ghost):
                obj.location = loc
                obj.rotation_euler = (0, 0, math.radians(s["yaw_deg"]))
                obj.scale = scale
                obj.keyframe_insert("location", frame=f)
                obj.keyframe_insert("rotation_euler", frame=f)
                obj.keyframe_insert("scale", frame=f)
            is_ghost = s["status"] == "ghost"
            key_visible(solid_parts, f, not is_ghost)
            key_visible(ghost_parts, f, is_ghost)
            # 1-sigma ellipse of the position covariance
            evals, evecs = np.linalg.eigh(s["cov_xy"])
            radii = np.sqrt(np.maximum(evals, 1e-4))
            ang = math.atan2(evecs[1, 1], evecs[0, 1])
            halo.location = (s["pos"][0], s["pos"][1], gz + 0.03)
            halo.rotation_euler = (0, 0, ang)
            halo.scale = (max(radii[1], 0.25), max(radii[0], 0.25), 1)
            halo.keyframe_insert("location", frame=f)
            halo.keyframe_insert("rotation_euler", frame=f)
            halo.keyframe_insert("scale", frame=f)
            key_visible([halo], f, is_ghost)
            if freeze and is_ghost:
                late_parts.extend(ghost_parts + [halo])

        # arcs from contributing agents, only for focus objects while ghost
        if tid not in focus_ids:
            continue
        agents_seen = sorted({a for s in states if s["status"] == "ghost" for a in s["agents"]})
        for a in agents_seen:
            if a not in sc.agent_tracks:
                continue
            a0 = sc.agent_tracks[a]["states"][0]["sensor"]
            if math.hypot(a0[0] - states[0]["pos"][0], a0[1] - states[0]["pos"][1]) > 45:
                continue
            cu = bpy.data.curves.new(f"arc_{tid}_{a}", "CURVE")
            cu.dimensions = "3D"
            cu.bevel_depth = 0.014
            cu.bevel_resolution = 3
            sp = cu.splines.new("BEZIER")
            sp.bezier_points.add(2)
            for bp in sp.bezier_points:
                bp.handle_left_type = bp.handle_right_type = "AUTO"
            arc = bpy.data.objects.new(f"arc_{tid}_{a}", cu)
            arc.active_material = m_arc
            col.objects.link(arc)
            asts = {s["frame"]: s for s in sc.agent_tracks[a]["states"]}
            if freeze:
                cu.bevel_factor_end = 0.0
                cu.keyframe_insert("bevel_factor_end", frame=arc_start)
                cu.bevel_factor_end = 1.0
                cu.keyframe_insert("bevel_factor_end", frame=arc_end)
            for fr in frames_all:
                f = bf(fr)
                s = by_frame.get(fr)
                on = s is not None and s["status"] == "ghost" and a in s["agents"] and fr in asts
                arc.hide_render = not on
                arc.keyframe_insert("hide_render", frame=f)
                if not on:
                    continue
                src = Vector(asts[fr]["sensor"]) + Vector((0, 0, 0.3))
                dst = Vector((s["pos"][0], s["pos"][1], s["pos"][2] + s["size"][2] / 2 + 0.15))
                d = (dst - src).length
                mid = (src + dst) / 2 + Vector((0, 0, 1.2 + 0.10 * d))
                for bp, co in zip(sp.bezier_points, (src, mid, dst)):
                    bp.co = co
                    bp.keyframe_insert("co", frame=f)

    for p_ in late_parts:
        key_visible([p_], 1, False)
        key_visible([p_], ghost_on, True)

    # camera
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = 26
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    target = bpy.data.objects.new("target", None)
    scene.collection.objects.link(target)
    tc = cam.constraints.new("TRACK_TO")
    tc.target = target
    tc.track_axis = "TRACK_NEGATIVE_Z"
    tc.up_axis = "UP_Y"
    prefs.keyframe_new_interpolation_type = "BEZIER"
    yaw = math.radians(ego0["yaw_deg"])
    fwd = Vector((math.cos(yaw), math.sin(yaw), 0))
    left = Vector((-math.sin(yaw), math.cos(yaw), 0))
    ego = Vector((ex, ey, gz))
    focus_pts = [Vector(s["pos"][:2] + (gz,)) for tid in focus_ids
                 for s in sc.tracks[tid]["states"] if s["status"] == "ghost"]
    focus = sum(focus_pts, Vector()) / len(focus_pts) if focus_pts else ego + 15 * fwd
    up = Vector((0, 0, 1))
    end = scene.frame_end
    if args.shot == "orbit":
        pivot = ego + 0.55 * (focus - ego)
        base = math.atan2(-fwd.y, -fwd.x)  # behind the ego car
        shots = []
        for k, (deg, radius, height) in enumerate([(-25, 19, 8.5), (15, 18, 8.0), (45, 16.5, 7.5),
                                                    (72, 15, 7.0), (92, 14, 6.5)]):
            ang = base + math.radians(deg) * (1 if left.dot(Vector((math.cos(base + 1), math.sin(base + 1), 0))) > 0 else -1)
            pos = pivot + Vector((radius * math.cos(ang), radius * math.sin(ang), 0)) + height * up
            f = 1 + int(round(k * (end - 1) / 4))
            shots.append((f, pos, pivot + 1.0 * up))
    elif args.shot == "egotop":
        shots = [(1, ego + 26 * up - 2 * fwd, ego + 4 * fwd)]
    elif args.shot == "top":
        shots = [(1, ego + 0.5 * (focus - ego) + 38 * up - 4 * fwd, ego + 0.5 * (focus - ego))]
    else:
        side = focus - ego
        side.z = 0
        shots = [
            (1, ego - 17 * fwd + 3 * left + 9 * up, ego + 7 * fwd),
            (int(end * 0.35), ego - 7 * fwd + 11 * left + 8 * up, ego + 0.55 * side),
            (int(end * 0.68), ego + 1.25 * side + 7 * left + 6 * up, ego + 0.6 * side),
            (end, ego - 2.5 * fwd + 0.6 * left + 3.4 * up, focus + 1.0 * up),
        ]
    for f, cpos, tpos in shots:
        cam.location = cpos
        target.location = tpos
        cam.keyframe_insert("location", frame=f)
        target.keyframe_insert("location", frame=f)

    # render settings
    r = scene.render
    r.engine = "CYCLES"
    w, h = (int(v) for v in args.res.split("x"))
    r.resolution_x, r.resolution_y = w, h
    r.use_motion_blur = True
    r.motion_blur_shutter = 0.5
    scene.cycles.samples = args.samples
    scene.cycles.use_denoising = True
    scene.cycles.max_bounces = 6
    scene.cycles.transparent_max_bounces = 16
    if args.device == "GPU":
        cp = bpy.context.preferences.addons["cycles"].preferences
        for dev_type in ("OPTIX", "CUDA"):
            try:
                cp.compute_device_type = dev_type
                cp.get_devices()
                if any(d.type == dev_type for d in cp.devices):
                    for d in cp.devices:
                        d.use = d.type == dev_type
                    break
            except TypeError:
                continue
        scene.cycles.device = "GPU"
    try:
        scene.view_settings.view_transform = "AgX"
        scene.view_settings.look = "AgX - Medium High Contrast"
    except TypeError:
        pass
    scene.view_settings.exposure = -0.4
    return scene, bf


def main():
    args = parse_args()
    scene, bf = build(args)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    if args.save:
        bpy.ops.wm.save_as_mainfile(filepath=str(Path(args.save).resolve()))
    for fr in [int(x) for x in args.stills.split(",") if x]:
        scene.frame_set(fr if args.freeze else bf(fr))
        scene.render.filepath = str((out / f"{args.shot}_{fr:06d}.png").resolve())
        bpy.ops.render.render(write_still=True)
        print("wrote", scene.render.filepath)
    if args.animation:
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = str((out / f"{args.shot}_").resolve())
        bpy.ops.render.render(animation=True)


if __name__ == "__main__":
    main()
