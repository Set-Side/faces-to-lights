# Set-Side Faces to Lights
# Turns selected mesh faces into area lights fitted to each face (or group of
# similar faces), with matching glowing faces. Lights remember their faces,
# so they can be re-fitted after the mesh changes.

import math
import re
import uuid

import bpy
import bmesh
from mathutils import Matrix, Vector



COLLECTION_NAME = "Set-Side Lights"
MANY_LIGHTS = 100  # warn above this many area lights
FACE_ATTR = "setside_light_id"  # invisible face label linking faces to their light


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def area_vector(points):
    """Newell's method: a vector along the polygon's normal whose length is
    twice its area. Works on world-space points, so object scale is handled."""
    total = Vector((0.0, 0.0, 0.0))
    count = len(points)
    for i in range(count):
        a = points[i]
        b = points[(i + 1) % count]
        total.x += (a.y - b.y) * (a.z + b.z)
        total.y += (a.z - b.z) * (a.x + b.x)
        total.z += (a.x - b.x) * (a.y + b.y)
    return total


def frame_for_faces(faces, matrix_world, fit='AREA', shape='RECTANGLE'):
    """Work out where one area light should sit to cover a group of faces.

    fit='AREA' sizes the light to the faces' real surface area (less overlap
    between neighbouring lights); fit='BOUNDS' covers the faces' whole outline.

    Returns (centre, rotation matrix, width, height) in world space,
    or None if the faces are too small or broken to use.
    """
    total = Vector((0.0, 0.0, 0.0))
    points = {}
    for face in faces:
        world = [matrix_world @ v.co for v in face.verts]
        total += area_vector(world)
        for v, p in zip(face.verts, world):
            points[v.index] = p
    if total.length < 1e-12:
        return None
    normal = total.normalized()
    pts = list(points.values())
    centre = sum(pts, Vector()) / len(pts)

    if len(faces) == 1:
        # One face: line the light up with its longest edge
        longest = None
        for edge in faces[0].edges:
            a = matrix_world @ edge.verts[0].co
            b = matrix_world @ edge.verts[1].co
            if longest is None or (b - a).length > longest.length:
                longest = b - a
        x_axis = longest - normal * longest.dot(normal)
    else:
        # Several faces: line the light up with the group's long direction
        u = normal.orthogonal().normalized()
        w = normal.cross(u).normalized()
        sxx = syy = sxy = 0.0
        for p in pts:
            a = (p - centre).dot(u)
            b = (p - centre).dot(w)
            sxx += a * a
            syy += b * b
            sxy += a * b
        angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        x_axis = u * math.cos(angle) + w * math.sin(angle)

    if x_axis.length < 1e-9:
        return None
    x_axis.normalize()

    # An area light shines along its -Z axis, so point -Z along the normal
    z_axis = -normal
    y_axis = z_axis.cross(x_axis).normalized()

    # Size the light to the footprint, then re-centre on it
    xs = [(p - centre).dot(x_axis) for p in pts]
    ys = [(p - centre).dot(y_axis) for p in pts]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    if width < 1e-6 or height < 1e-6:
        return None
    centre = centre + x_axis * (max(xs) + min(xs)) / 2 + y_axis * (max(ys) + min(ys)) / 2

    # Lift the light to the front of curved patches, so no part of it sits
    # behind the surface
    lift = max((p - centre).dot(normal) for p in pts)
    if lift > 0.0:
        centre = centre + normal * lift

    # Shrink the light to the faces' real (projected) area, keeping its
    # proportions, so neighbouring lights overlap far less
    if fit == 'AREA':
        surface_area = total.length / 2.0
        light_area_now = width * height
        if shape in {'ELLIPSE', 'DISK'}:
            light_area_now *= math.pi / 4.0
        if light_area_now > surface_area > 1e-12:
            scale = math.sqrt(surface_area / light_area_now)
            width *= scale
            height *= scale

    rotation = Matrix((x_axis, y_axis, z_axis)).transposed()
    return centre, rotation, width, height


def group_faces(faces, merge, max_angle):
    """Split the selected faces into groups. With merging on, neighbouring
    faces that point within max_angle of each other share one light."""
    if not merge:
        return [[f] for f in faces]
    selected = {f.index: f for f in faces}
    seen = set()
    groups = []
    for seed in faces:
        if seed.index in seen:
            continue
        seen.add(seed.index)
        group = [seed]
        stack = [seed]
        while stack:
            current = stack.pop()
            for edge in current.edges:
                for neighbour in edge.link_faces:
                    i = neighbour.index
                    if i in selected and i not in seen and \
                            neighbour.normal.angle(seed.normal, 0.0) <= max_angle:
                        seen.add(i)
                        group.append(neighbour)
                        stack.append(neighbour)
        groups.append(group)
    return groups


def next_light_id():
    """A number no existing Set-Side light uses yet."""
    used = [o.setside_light_id for o in bpy.data.objects if o.setside_light_id > 0]
    return max(used, default=0) + 1


def tag_faces(bm, faces, light_id):
    """Label faces with their light's id, stored on the mesh itself. The label
    survives most edits, unlike face numbers, which change when faces are
    added or deleted."""
    layer = bm.faces.layers.int.get(FACE_ATTR) or bm.faces.layers.int.new(FACE_ATTR)
    for face in faces:
        face[layer] = light_id


def faces_for_light(bm, target):
    """Find the faces a light belongs to: by label first, then by stored
    face numbers for lights made before labels existed."""
    layer = bm.faces.layers.int.get(FACE_ATTR)
    if layer is not None and target.setside_light_id > 0:
        found = [f for f in bm.faces if f[layer] == target.setside_light_id]
        if found:
            return found
        return None  # labelled faces were deleted or re-labelled
    indices = text_to_indices(target.setside_faces) or [target.setside_face]
    if any(i < 0 or i >= len(bm.faces) for i in indices):
        return None
    bm.faces.ensure_lookup_table()
    return [bm.faces[i] for i in indices]


def light_area(light):
    """Area of an area light in square metres (before object scale)."""
    area = light.size * light.size_y
    if light.shape in {'ELLIPSE', 'DISK'}:
        area *= math.pi / 4.0
    return area


def area_m2(light, scene):
    """A light's area in real square metres, allowing for the scene's Unit
    Scale (a scene in centimetres has Unit Scale 0.01)."""
    scale = scene.unit_settings.scale_length or 1.0
    return light_area(light) * scale * scale


def faces_to_text(faces):
    return ",".join(str(f.index) for f in faces)


def text_to_indices(text):
    return [int(x) for x in text.split(",") if x.strip().isdigit()]


# ---------------------------------------------------------------------------
# Light group helpers
# ---------------------------------------------------------------------------

def lightgroup_name(source):
    """A valid Cycles light group name for a mesh's lights."""
    return "SetSide_" + re.sub(r"[^A-Za-z0-9_]", "_", source.name)


def ensure_lightgroup(context, name):
    """Make sure the view layer has a light group of this name, so it gets
    its own render pass in Cycles."""
    groups = context.view_layer.lightgroups
    if name not in [g.name for g in groups]:
        groups.add(name=name)


# ---------------------------------------------------------------------------
# Scene helpers
# ---------------------------------------------------------------------------

def get_lights_collection(context):
    """Find or create the collection the new lights go into."""
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is None:
        coll = bpy.data.collections.new(COLLECTION_NAME)
    if coll.name not in context.scene.collection.children:
        context.scene.collection.children.link(coll)
    return coll


def fit_light(light_obj, frame, flip, offset, purpose='EMIT', scale=1.0):
    """Place and size an area light object to match a face frame.

    purpose='EMIT': the light sits on the faces and shines out of them.
    purpose='ONTO': the light sits `offset` in front of the faces and shines
    back at them, like a softbox aimed at the surface.
    """
    centre, rotation, width, height = frame
    normal = -(rotation.col[2])  # the way the faces point
    if purpose == 'ONTO':
        rotation = rotation @ Matrix.Rotation(math.pi, 3, 'X')  # face the surface
        location = centre + normal * offset
    else:
        if flip:
            rotation = rotation @ Matrix.Rotation(math.pi, 3, 'X')
        direction = -(rotation.col[2])  # the way the light shines
        location = centre + direction * offset
    target = Matrix.Translation(location) @ rotation.to_4x4()

    light_obj.data.size = width * scale
    light_obj.data.size_y = height * scale

    if light_obj.parent is not None:
        # Cancel out the parent's current transform (including any squash or
        # stretch), so the light keeps its exact size and angle.
        light_obj.matrix_parent_inverse = light_obj.parent.matrix_world.inverted_safe()
        light_obj.matrix_basis = target
    else:
        light_obj.matrix_world = target


def make_emission_material(name, color, strength, casts_light):
    """An emission material. casts_light=False makes it glow without acting
    as a light source in Cycles (just for looks)."""
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
    try:
        mat.use_nodes = True  # always on in Blender 5, needed in Blender 4
    except (AttributeError, TypeError):
        pass
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    if mat.node_tree.animation_data is not None:
        mat.node_tree.animation_data_clear()  # old drivers would point at removed nodes
    nodes.clear()
    emission = nodes.new("ShaderNodeEmission")
    emission.name = emission.label = GLOW_EMISSION
    output = nodes.new("ShaderNodeOutputMaterial")
    emission.location = (-200, 0)
    links.new(emission.outputs["Emission"], output.inputs["Surface"])
    # The colour lives in its own node, so Variation can tint it per face
    base = nodes.new("ShaderNodeRGB")
    base.name = base.label = GLOW_BASE
    base.location = (-800, 100)
    base.outputs[0].default_value = (*color, 1.0)
    links.new(base.outputs[0], emission.inputs["Color"])
    emission.inputs["Color"].default_value = (*color, 1.0)
    emission.inputs["Strength"].default_value = strength
    mat.diffuse_color = (*color, 1.0)  # colour in Solid view

    setting = 'AUTO' if casts_light else 'NONE'
    for owner in (mat, getattr(mat, "cycles", None)):
        try:
            owner.emission_sampling = setting
            break
        except (AttributeError, TypeError):
            continue
    return mat


def _tanner_rgb(kelvin):
    """Rough colour for very low temperatures (below 1667 K)."""
    k = kelvin / 100.0
    r = 255.0
    g = 99.4708025861 * math.log(k) - 161.1195681661
    b = 0.0
    srgb = [max(0.0, min(255.0, c)) / 255.0 for c in (r, g, b)]
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in srgb]


def kelvin_to_rgb(kelvin):
    """Colour of light at a temperature in kelvin, as linear RGB in Blender's
    default colour space, scaled so the brightest channel is 1.

    Uses the Planckian locus (Kim et al. approximation of the CIE 1931
    chromaticity of a black body), converted from CIE XYZ to linear Rec.709.
    """
    t = max(1000.0, min(20000.0, kelvin))
    if t < 1667.0:
        rgb = _tanner_rgb(t)
    else:
        if t <= 4000.0:
            x = -0.2661239e9 / t**3 - 0.2343589e6 / t**2 + 0.8776956e3 / t + 0.179910
        else:
            x = -3.0258469e9 / t**3 + 2.1070379e6 / t**2 + 0.2226347e3 / t + 0.240390
        if t <= 2222.0:
            y = -1.1063814 * x**3 - 1.34811020 * x**2 + 2.18555832 * x - 0.20219683
        elif t <= 4000.0:
            y = -0.9549476 * x**3 - 1.37418593 * x**2 + 2.09137015 * x - 0.16748867
        else:
            y = 3.0817580 * x**3 - 5.87338670 * x**2 + 3.75112997 * x - 0.37001483
        X = x / y
        Y = 1.0
        Z = (1.0 - x - y) / y
        rgb = [
            3.2404542 * X - 1.5371385 * Y - 0.4985314 * Z,
            -0.9692660 * X + 1.8760108 * Y + 0.0415560 * Z,
            0.0556434 * X - 0.2040259 * Y + 1.0572252 * Z,
        ]
    rgb = [max(0.0, c) for c in rgb]
    top = max(rgb) or 1.0
    return tuple(c / top for c in rgb)


def source_uid(source):
    """A permanent ID for a mesh, so its materials are found even after the
    mesh is renamed."""
    uid = source.get("setside_uid")
    if not uid:
        uid = source["setside_uid"] = uuid.uuid4().hex[:12]
    return uid


def ensure_unique_uid(source):
    """A duplicated mesh copies its ID: give the copy its own, unless it
    already has lights of its own."""
    uid = source.get("setside_uid")
    if not uid:
        return source_uid(source)
    shared = any(o is not source and o.get("setside_uid") == uid for o in bpy.data.objects)
    owns_lights = any(o.setside_source == source for o in bpy.data.objects)
    if shared and not owns_lights:
        source["setside_uid"] = uuid.uuid4().hex[:12]
    return source["setside_uid"]


def own_material(mat, source, emitter=False):
    if mat is not None:
        mat["setside_owner"] = source_uid(source)
        if emitter:
            mat["setside_emitter"] = True
    return mat


def emitter_material(source):
    """The Exact Shape material belonging to a mesh, if any."""
    uid = source.get("setside_uid")
    for mat in bpy.data.materials:
        if mat.get("setside_emitter") and uid and mat.get("setside_owner") == uid:
            return mat
    return bpy.data.materials.get(emitter_material_name(source))


def glow_material_name(source):
    return f"Set-Side Glow {source.name}"


def emitter_material_name(source):
    return f"Set-Side Emitter {source.name}"


GLOW_EMISSION = "SetSide Emission"
GLOW_BASE = "SetSide Base"
GLOW_TEXTURE = "SetSide Texture Mix"


def set_emission(mat, color=None, strength=None):
    """Change a Set-Side emission material's colour and/or strength."""
    node = emission_node(mat)
    if node is None:
        return
    if color is not None:
        node.inputs["Color"].default_value = (*color, 1.0)
        base = mat.node_tree.nodes.get(GLOW_BASE)
        if base is not None:
            base.outputs[0].default_value = (*color, 1.0)
        if not mat.get("setside_keep"):
            mat.diffuse_color = (*color, 1.0)
    if strength is not None:
        node.inputs["Strength"].default_value = strength


def glow_strength_for(power, per_area, lights, brightness):
    """How bright the glowing faces should look, following the lights' power,
    so turning the lights up makes the faces glow brighter too."""
    if per_area:
        density = power
    else:
        areas = [light_area(o.data) for o in lights if o.type == 'LIGHT']
        mean_area = sum(areas) / len(areas) if areas else 1.0
        density = power / max(mean_area, 1e-6)
    return brightness * density / 10.0


# ---------------------------------------------------------------------------
# Brightness and power
# ---------------------------------------------------------------------------

# Blender lights are in radiant watts. 683 lumens per watt is the definition
# of the lumen, and the same figure Blender's own glTF exporter uses.
LUMENS_PER_WATT = 683.0

def any_solo(scene):
    return False


def mix_level(source, scene, solo=None):
    return 1.0


def write_energy(obj, watts, scene, solo=None):
    """Set a light's power. The mixer level is applied on top, and the value
    is also stored as the base that animated effects work from."""
    obj.setside_watts = watts
    value = watts * mix_level(obj.setside_source, scene, solo) * obj.setside_var
    obj.data["setside_base"] = value
    obj.data.energy = value


def write_color(obj, color):
    """Set a light's colour, keeping any Variation tint on top."""
    obj["setside_color"] = list(color[:3])
    tint = obj.get("setside_tint", (1.0, 1.0, 1.0))
    obj.data.color = tuple(color[i] * tint[i] for i in range(3))


def emission_node(mat):
    """The Set-Side emission node in a material (older versions: any)."""
    if mat is None or mat.node_tree is None:
        return None
    node = mat.node_tree.nodes.get(GLOW_EMISSION)
    if node is not None:
        return node
    if mat.get("setside_keep"):
        return None
    for node in mat.node_tree.nodes:
        if node.type == 'EMISSION':
            return node
    return None


def write_strength(mat, strength, source, scene, solo=None):
    """Set a glow or Exact Shape material's brightness (mixer level on top)."""
    if mat is None:
        return
    mat["setside_strength"] = strength
    value = strength * mix_level(source, scene, solo)
    mat["setside_base"] = value
    set_emission(mat, strength=value)


def source_materials(source):
    """Every Set-Side material that belongs to a mesh."""
    mats = list(glow_materials(source))
    emitter = emitter_material(source)
    if emitter is not None:
        mats.append(emitter)
    return mats


def scene_sources(scene):
    """Meshes in the scene that have Set-Side lights, in name order."""
    found = {o.setside_source for o in scene.objects if o.setside_source is not None}
    return sorted((s for s in found if s.name in scene.objects), key=lambda o: o.name)


def refresh_mix(scene):
    """Re-apply every mesh's mixer level (after a level, mute or solo change)."""
    solo = any_solo(scene)
    for obj in scene.objects:
        if obj.setside_source is None or obj.type != 'LIGHT':
            continue
        if getattr(obj.data.cycles, "is_portal", False):
            continue
        watts = obj.setside_watts or obj.data.get("setside_base", obj.data.energy)
        write_energy(obj, watts, scene, solo)
    for source in scene_sources(scene):
        for mat in source_materials(source):
            strength = mat.get("setside_strength")
            if strength is None:
                node = emission_node(mat)
                strength = node.inputs["Strength"].default_value if node else 0.0
            write_strength(mat, strength, source, scene, solo)


ORIG_MAT_ATTR = "setside_orig_mat"  # original material slot + 1 (0 = not recorded)


def assign_material_to_faces(obj, faces, mat, orig_layer=None):
    """Put a material on just these faces, leaving the rest of the mesh alone.
    With orig_layer, each face's first material is remembered for Remove."""
    materials = obj.data.materials
    if len(materials) == 0:
        # Keep an empty first slot, so unselected faces don't change
        materials.append(None)
    index = None
    for i, slot_mat in enumerate(materials):
        if slot_mat == mat:
            index = i
            break
    if index is None:
        materials.append(mat)
        index = len(materials) - 1
    for face in faces:
        if orig_layer is not None and face[orig_layer] == 0:
            face[orig_layer] = face.material_index + 1
        face.material_index = index


def set_portal(light, on):
    """Make an area light a Cycles light portal (helps sky light through windows)."""
    try:
        light.cycles.is_portal = on
    except AttributeError:
        pass  # Cycles not available


def set_visibility(obj, camera, reflections):
    """Whether a light shows up directly in the camera and in reflections.
    It still lights the scene either way."""
    obj.visible_camera = camera
    obj.visible_glossy = reflections


def get_exclude_collection(source_obj):
    """A hidden collection holding the source mesh, used to stop its lights
    from lighting it (Blender light linking)."""
    name = f"Set-Side Exclude {source_obj.name}"
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
    if source_obj.name not in coll.objects:
        coll.objects.link(source_obj)
    # collection_objects lists the same objects, in the same order, as
    # coll.objects, so pair them up to find the source mesh's entry
    for linked_obj, item in zip(coll.objects, coll.collection_objects):
        if linked_obj == source_obj:
            item.light_linking.link_state = 'EXCLUDE'
    return coll


def get_only_collection(source_obj):
    """A hidden collection holding just the source mesh, so lights linked to it
    light that mesh and nothing else (Blender light linking)."""
    name = f"Set-Side Only {source_obj.name}"
    coll = bpy.data.collections.get(name)
    if coll is None:
        coll = bpy.data.collections.new(name)
    if source_obj.name not in coll.objects:
        coll.objects.link(source_obj)
    for linked_obj, item in zip(coll.objects, coll.collection_objects):
        if linked_obj == source_obj:
            item.light_linking.link_state = 'INCLUDE'
    return coll


def read_faces(obj):
    """Return (bmesh, needs_free) for an object's current mesh."""
    if obj.mode == 'EDIT':
        return bmesh.from_edit_mesh(obj.data), False
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    return bm, True


def remember_source(target, source, faces, kind, flip, offset, light_id=0):
    """Store where a light came from, for Update Lights."""
    target.setside_light_id = light_id
    target.setside_source = source
    target.setside_faces = faces_to_text(faces)
    target.setside_face = faces[0].index
    target.setside_kind = kind
    target.setside_flip = flip
    target.setside_offset = offset


# ---------------------------------------------------------------------------
# Operators
# ---------------------------------------------------------------------------

class SETSIDE_OT_faces_to_lights(bpy.types.Operator):
    """Turn the selected faces into lights"""
    bl_idname = "setside.faces_to_lights"
    bl_label = "Faces to Lights"
    bl_options = {'REGISTER', 'UNDO'}

    merge_faces: bpy.props.BoolProperty(
        name="Merge Similar Faces",
        description="Neighbouring faces pointing roughly the same way share one light. "
        "Far fewer lights on detailed or curved shapes",
        default=True,
    )
    merge_angle: bpy.props.FloatProperty(
        name="Merge Angle",
        description="How far apart neighbouring faces can point and still share a light",
        default=math.radians(15.0), min=0.0, max=math.radians(90.0), subtype='ANGLE',
    )
    power: bpy.props.FloatProperty(
        name="Power", description="Power of each area light in watts, "
        "or per square metre if Power per m\u00b2 is on",
        default=10.0, min=0.0, soft_max=1000.0, unit='POWER',
    )
    power_per_area: bpy.props.BoolProperty(
        name="Power per m\u00b2",
        description="Scale each light's power by its size, so big faces are brighter "
        "than small ones, like real light panels",
        default=True,
    )
    fit: bpy.props.EnumProperty(
        name="Light Size",
        items=(
            ('AREA', "Match Surface", "Size each light to its faces' real area. "
             "Less overlap between neighbouring lights"),
            ('BOUNDS', "Cover Outline", "Size each light to cover its faces' whole outline. "
             "Best for flat faces such as windows"),
        ),
        default='AREA',
    )
    color: bpy.props.FloatVectorProperty(
        name="Colour", subtype='COLOR', size=3,
        default=(1.0, 1.0, 1.0), min=0.0, max=1.0,
    )
    shape: bpy.props.EnumProperty(
        name="Shape",
        items=(
            ('RECTANGLE', "Rectangle", "Fit a rectangle to each face or group"),
            ('ELLIPSE', "Ellipse", "Fit an ellipse to each face or group"),
        ),
        default='RECTANGLE',
    )
    spread: bpy.props.FloatProperty(
        name="Spread", description="How wide the light spreads. Lower values "
        "give a narrower beam and less spill onto nearby surfaces",
        default=math.radians(180.0), min=math.radians(1.0), max=math.radians(180.0),
        subtype='ANGLE',
    )
    offset: bpy.props.FloatProperty(
        name="Offset", description="Distance to move the lights off the faces",
        default=0.001, min=0.0, soft_max=1.0, unit='LENGTH',
    )
    flip: bpy.props.BoolProperty(
        name="Flip Direction", description="Shine into the mesh instead of out of it",
        default=False,
    )
    parent_to_mesh: bpy.props.BoolProperty(
        name="Parent to Mesh", description="Make the lights follow the mesh when it moves",
        default=True,
    )
    exclude_source: bpy.props.BoolProperty(
        name="Don't Light Source Mesh",
        description="Stop these lights from lighting the mesh they were made from "
        "(uses Blender's light linking)",
        default=False,
    )
    portal: bpy.props.BoolProperty(
        name="Make Portals",
        description="Turn the faces into Cycles light portals, which help light from "
        "the sky or an HDRI come in through windows. Point them into the room "
        "(use Flip Direction if needed). Cycles only",
        default=False,
    )
    visible_camera: bpy.props.BoolProperty(
        name="Visible to Camera",
        description="Show the light panels directly in renders. They still light the scene when off",
        default=False,
    )
    visible_reflections: bpy.props.BoolProperty(
        name="Visible in Reflections",
        description="Show the light panels in reflections. They still light the scene when off",
        default=False,
    )
    make_glow: bpy.props.BoolProperty(
        name="Make Faces Glow",
        description="Give the selected faces a glowing material in the light's colour, "
        "so the faces themselves look lit",
        default=True, options={'SKIP_SAVE'},
    )
    glow_strength: bpy.props.FloatProperty(
        name="Glow Brightness",
        description="How bright the glowing faces look compared with the lights' power. "
        "1 is a good match",
        default=1.0, min=0.0, soft_max=10.0,
    )
    use_temperature: bpy.props.BoolProperty(
        name="Use Temperature",
        description="Set the colour from a colour temperature in kelvin, like a real bulb",
        default=False,
    )
    temperature: bpy.props.FloatProperty(
        name="Temperature (K)",
        description="Colour temperature in kelvin. 1900 K is candlelight, 2700 K warm "
        "household light, 5600 K daylight, 7000 K and up cool blue sky",
        default=5600.0, min=1000.0, max=20000.0, step=10000, precision=0,
    )

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        exact = False
        self.draw_colour(layout)
        if not exact:
            layout.prop(self, "portal")
            if not self.portal:
                layout.prop(self, "power", text="Power per m\u00b2" if self.power_per_area else "Power")
                layout.prop(self, "power_per_area")
            layout.prop(self, "shape")
            layout.prop(self, "fit")
            if not self.portal:
                layout.prop(self, "spread")
            layout.prop(self, "merge_faces")
            row = layout.row()
            row.active = self.merge_faces
            row.prop(self, "merge_angle")
            if not self.portal:
                layout.prop(self, "make_glow")
                if self.make_glow:
                    layout.prop(self, "glow_strength")
        layout.prop(self, "offset")
        layout.prop(self, "flip")
        layout.prop(self, "parent_to_mesh")
        if exact or not self.portal:
            layout.prop(self, "exclude_source")
            layout.separator()
            layout.prop(self, "visible_camera")
            layout.prop(self, "visible_reflections")

    def draw_colour(self, layout):
        layout.prop(self, "use_temperature")
        layout.prop(self, "temperature" if self.use_temperature else "color")


    def light_color(self):
        return kelvin_to_rgb(self.temperature) if self.use_temperature else tuple(self.color)

    def execute(self, context):
        coll = get_lights_collection(context)
        made = 0
        color = self.light_color()

        for obj in context.objects_in_mode:
            if obj.type != 'MESH':
                continue
            bm = bmesh.from_edit_mesh(obj.data)
            # Add data layers before reading faces: adding one later would
            # invalidate the faces already picked up
            if bm.faces.layers.int.get(FACE_ATTR) is None:
                bm.faces.layers.int.new(FACE_ATTR)
            if self.make_glow and bm.faces.layers.int.get(ORIG_MAT_ATTR) is None:
                bm.faces.layers.int.new(ORIG_MAT_ATTR)
            bm.faces.index_update()
            bm.verts.index_update()
            selected = [f for f in bm.faces if f.select]
            if not selected:
                continue
            ensure_unique_uid(obj)
            # Running again on faces that already have lights replaces them,
            # instead of stacking a second set on top
            label = bm.faces.layers.int[FACE_ATTR]
            old_ids = {f[label] for f in selected if f[label]}
            refit_later = []
            if old_ids:
                left = {}
                for f in bm.faces:
                    if f[label] in old_ids and not f.select:
                        left[f[label]] = left.get(f[label], 0) + 1
                for old in [o for o in bpy.data.objects
                            if o.setside_source == obj and o.setside_light_id in old_ids]:
                    if left.get(old.setside_light_id):
                        refit_later.append(old)  # still lights other faces
                    else:
                        delete_setside_object(old)
            portal = self.portal
            exclude_coll = get_exclude_collection(obj) if self.exclude_source else None

            lit_faces = []
            new_lights = []
            glow_groups = {}  # material colour -> faces, for Colour from Material
            for i, group in enumerate(group_faces(selected, self.merge_faces, self.merge_angle)):
                frame = frame_for_faces(group, obj.matrix_world, self.fit, self.shape)
                if frame is None:
                    continue
                group_color = color
                group_key = ""
                light_id = next_light_id()
                light = bpy.data.lights.new(f"{obj.name}_light_{light_id}", 'AREA')
                light.color = group_color
                light.shape = self.shape
                light.spread = self.spread
                set_portal(light, portal)

                light_obj = bpy.data.objects.new(light.name, light)
                coll.objects.link(light_obj)
                light_obj["setside_color"] = list(group_color)
                if not portal:
                    set_visibility(light_obj, self.visible_camera, self.visible_reflections)
                if self.parent_to_mesh:
                    light_obj.parent = obj
                remember_source(light_obj, obj, group, 'AREA', self.flip, self.offset, light_id)
                light_obj.setside_fit = self.fit
                tag_faces(bm, group, light_id)
                if exclude_coll is not None and not portal:
                    light_obj.light_linking.receiver_collection = exclude_coll
                fit_light(light_obj, frame, self.flip, self.offset)
                if self.power_per_area and not portal:
                    light_obj.setside_power_density = self.power
                    write_energy(light_obj, self.power * area_m2(light, context.scene),
                                 context.scene)
                else:
                    write_energy(light_obj, self.power, context.scene)
                lit_faces.extend(group)
                glow_groups.setdefault(group_key, (group_color, []))[1].extend(group)
                new_lights.append(light_obj)
                made += 1

            do_glow = self.make_glow
            if do_glow and lit_faces and not portal:
                strength = glow_strength_for(self.power, self.power_per_area,
                                             new_lights, self.glow_strength)
                orig_layer = bm.faces.layers.int.get(ORIG_MAT_ATTR)
                for key, (group_color, faces) in glow_groups.items():
                    name = glow_material_name(obj) + (f" {key}" if key else "")
                    by_material = {}
                    for f in faces:
                        original = None
                        by_material.setdefault(original, []).append(f)
                    for original, mat_faces in by_material.items():
                        if original is None:
                            glow = make_emission_material(name, group_color, strength,
                                                          casts_light=False)
                        own_material(glow, obj)
                        write_strength(glow, strength, obj, context.scene)
                        assign_material_to_faces(obj, mat_faces, glow, orig_layer)
                obj.setside_glow_brightness = self.glow_strength
            for old in refit_later:
                refit_object(context, old, bm, obj, {})
            bmesh.update_edit_mesh(obj.data)  # save face labels (and glow)

        if made == 0:
            self.report({'WARNING'}, "No usable faces selected")
            return {'CANCELLED'}
        sync_edit_settings(context, self)
        if made > MANY_LIGHTS:
            tip = "Try Merge Similar Faces with a larger angle"
            self.report({'WARNING'}, f"Created {made} lights, which can slow Blender down. " + tip)
        else:
            portals = self.portal
            thing = "portal" if portals else "light"
            self.report({'INFO'}, f"Created {made} {thing}{'s' if made != 1 else ''} "
                        f"in '{COLLECTION_NAME}'")
        return {'FINISHED'}

def delete_setside_object(obj):
    """Delete a Set-Side light or glowing object, and its data if unused."""
    data = obj.data
    bpy.data.objects.remove(obj)
    if data is not None and data.users == 0:
        if isinstance(data, bpy.types.Light):
            bpy.data.lights.remove(data)
        elif isinstance(data, bpy.types.Mesh):
            bpy.data.meshes.remove(data)


def refit_object(context, target, bm, src, trees, faces=None):
    """Re-fit one Set-Side light (or Exact Shape object) to its faces.
    Returns False when its faces no longer exist."""
    if faces is None:
        faces = faces_for_light(bm, target)
    if not faces:
        return False
    if target.type != 'LIGHT':
        return False
    frame = frame_for_faces(faces, src.matrix_world, target.setside_fit or 'BOUNDS',
                            target.data.shape)
    if frame is None:
        return False
    if target.setside_kind == 'ONTO':
        fit_light(target, frame, False, target.setside_offset, 'ONTO', target.setside_scale)
    else:
        fit_light(target, frame, target.setside_flip, target.setside_offset)
    if target.setside_power_density > 0:
        write_energy(target, target.setside_power_density
                     * area_m2(target.data, context.scene), context.scene)
    return True


class SETSIDE_OT_update_lights(bpy.types.Operator):
    """Re-fit every Set-Side light to its faces' current shape, size and position"""
    bl_idname = "setside.update_face_lights"
    bl_label = "Update Lights"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        targets = [o for o in context.scene.objects if o.setside_source is not None]
        if not targets:
            self.report({'WARNING'}, "No Set-Side lights found in this scene")
            return {'CANCELLED'}

        meshes = {}
        trees = {}
        updated = skipped = 0
        for target in targets:
            src = target.setside_source
            if src.type != 'MESH':
                skipped += 1
                continue
            if src not in meshes:
                bm, needs_free = read_faces(src)
                bm.faces.ensure_lookup_table()
                bm.faces.index_update()
                bm.verts.index_update()
                # Index faces by label once, instead of searching per light
                by_label = {}
                layer = bm.faces.layers.int.get(FACE_ATTR)
                if layer is not None:
                    for f in bm.faces:
                        if f[layer]:
                            by_label.setdefault(f[layer], []).append(f)
                meshes[src] = (bm, needs_free, by_label, layer)
            bm, _free, by_label, layer = meshes[src]
            faces = None
            if layer is not None and target.setside_light_id > 0:
                faces = by_label.get(target.setside_light_id) or []
            if refit_object(context, target, bm, src, trees, faces):
                updated += 1
            else:
                skipped += 1

        for bm, needs_free, _by_label, _layer in meshes.values():
            if needs_free:
                bm.free()

        message = f"Updated {updated} light{'s' if updated != 1 else ''}"
        if skipped:
            message += f", skipped {skipped} (faces no longer exist)"
            self.report({'WARNING'}, message)
        else:
            self.report({'INFO'}, message)
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Edit existing lights (live settings in the sidebar)
# ---------------------------------------------------------------------------

def edit_targets(context):
    """Set-Side lights belonging to the active mesh. If a Set-Side light is
    active, use the mesh it came from."""
    active = context.active_object
    if active is None:
        return None, []
    source = active.setside_source if active.setside_source is not None else active
    targets = [o for o in context.scene.objects if o.setside_source == source]
    return source, targets


def _edit_lights(context):
    _source, targets = edit_targets(context)
    for obj in targets:
        yield obj


def _is_portal(obj):
    return obj.type == 'LIGHT' and getattr(obj.data.cycles, "is_portal", False)


_syncing = False  # True while the panel is being filled in, so nothing re-applies


def sync_edit_settings(context, op):
    """Copy the options just used into the Edit Lights box, so it shows them."""
    global _syncing
    s = context.scene.setside_edit
    _syncing = True
    try:
        s.power = op.power
        s.power_per_area = op.power_per_area
        s.use_temperature = op.use_temperature
        s.temperature = op.temperature
        s.color = op.color
        s.spread = op.spread
        s.glow_brightness = op.glow_strength
        s.visible_camera = op.visible_camera
        s.visible_reflections = op.visible_reflections
    finally:
        _syncing = False
    for obj in context.objects_in_mode:
        if obj.type == 'MESH' and any(o.setside_source == obj for o in context.scene.objects):
            save_edit_settings(obj, s)


def _settings_color(s):
    return kelvin_to_rgb(s.temperature) if s.use_temperature else tuple(s.color)


def glow_materials(source):
    """All glow materials belonging to a mesh (one per colour)."""
    uid = source.get("setside_uid")
    prefix = glow_material_name(source)
    return [m for m in bpy.data.materials if not m.get("setside_emitter") and (
        (uid and m.get("setside_owner") == uid)
        or m.name == prefix or m.name.startswith(prefix + " "))]


def _refresh_glow(s, context):
    """Keep the glowing faces' brightness in step with the lights."""
    source, targets = edit_targets(context)
    if source is None:
        return
    lights = [o for o in targets if o.type == 'LIGHT' and not _is_portal(o)]
    strength = glow_strength_for(s.power, s.power_per_area, lights, s.glow_brightness)
    for mat in glow_materials(source):
        write_strength(mat, strength, source, context.scene)


def update_power(self, context):
    """Only the setting that changed is applied, so other per-light values stay."""
    global _syncing
    if _syncing:
        return
    solo = any_solo(context.scene)
    for obj in _edit_lights(context):
        if obj.type != 'LIGHT' or _is_portal(obj):
            continue
        if self.power_per_area:
            obj.setside_power_density = self.power
            write_energy(obj, self.power * area_m2(obj.data, context.scene), context.scene,
                         solo)
        else:
            obj.setside_power_density = 0.0
            write_energy(obj, self.power, context.scene, solo)
    _refresh_glow(self, context)


def update_color(self, context):
    """Colour (or temperature) changes the lights, the glowing faces and any
    Exact Shape lights together."""
    if _syncing:
        return
    color = _settings_color(self)
    for obj in _edit_lights(context):
        if obj.type == 'LIGHT' and not _is_portal(obj) and not obj.setside_screen:
            write_color(obj, color)
    source, _targets = edit_targets(context)
    if source is not None:
        for mat in glow_materials(source):
            set_emission(mat, color=color)
        set_emission(emitter_material(source), color=color)


def update_spread(self, context):
    if _syncing:
        return
    for obj in _edit_lights(context):
        if obj.type == 'LIGHT' and not _is_portal(obj):
            obj.data.spread = self.spread


def update_glow_brightness(self, context):
    if _syncing:
        return
    _refresh_glow(self, context)


def update_emission(self, context):
    if _syncing:
        return
    source, _targets = edit_targets(context)
    if source is not None:
        write_strength(emitter_material(source), self.emission_strength, source,
                       context.scene)


def update_visibility(self, context):
    if _syncing:
        return
    for obj in _edit_lights(context):
        if not _is_portal(obj):
            set_visibility(obj, self.visible_camera, self.visible_reflections)


EDIT_SKIP = {"rna_type", "name", "fixture"}


def save_edit_settings(source, s):
    """Remember a mesh's Edit Lights values, to show them when it's selected again."""
    if source is None:
        return
    data = {}
    for prop in s.bl_rna.properties:
        key = prop.identifier
        if key in EDIT_SKIP:
            continue
        value = getattr(s, key)
        data[key] = list(value) if getattr(prop, "is_array", False) else value
    source["setside_edit"] = data


def load_edit_settings(context):
    """Show the selected mesh's own values in Edit Lights."""
    global _syncing
    source, targets = edit_targets(context)
    if source is None or not targets:
        return
    s = context.scene.setside_edit
    data = source.get("setside_edit")
    if data is None:
        # Older lights: read what we can from the first light
        light = next((o for o in targets if o.type == 'LIGHT' and not _is_portal(o)), None)
        if light is None:
            return
        density = light.setside_power_density
        data = {"power": density if density > 0 else light.setside_watts or light.data.energy,
                "power_per_area": density > 0,
                "color": list(light.get("setside_color", light.data.color[:3])),
                "spread": light.data.spread}
    _syncing = True
    try:
        for key, value in dict(data).items():
            if key in s.bl_rna.properties and key not in EDIT_SKIP:
                try:
                    setattr(s, key, value)
                except (TypeError, ValueError, AttributeError):
                    pass
    finally:
        _syncing = False


def _remembered(fn):
    """Wrap an Edit Lights update so the mesh remembers the new values."""
    def wrapper(self, context):
        fn(self, context)
        if not _syncing:
            save_edit_settings(edit_targets(context)[0], self)
    wrapper.__name__ = fn.__name__
    wrapper.__doc__ = fn.__doc__
    return wrapper


update_power = _remembered(update_power)
update_color = _remembered(update_color)
update_spread = _remembered(update_spread)
update_glow_brightness = _remembered(update_glow_brightness)
update_emission = _remembered(update_emission)
update_visibility = _remembered(update_visibility)


_msgbus_owner = object()


def _on_active_change(*_args):
    try:
        load_edit_settings(bpy.context)
    except (AttributeError, RuntimeError, ReferenceError):
        pass


def _subscribe():
    bpy.msgbus.clear_by_owner(_msgbus_owner)
    bpy.msgbus.subscribe_rna(key=(bpy.types.LayerObjects, "active"), owner=_msgbus_owner,
                             args=(), notify=_on_active_change)


@bpy.app.handlers.persistent
def _on_load(*_args):
    _subscribe()


class SETSIDE_EditSettings(bpy.types.PropertyGroup):
    power: bpy.props.FloatProperty(
        name="Power", default=10.0, min=0.0, soft_max=1000.0, unit='POWER', update=update_power)
    power_per_area: bpy.props.BoolProperty(
        name="Power per m\u00b2", default=True, update=update_power)
    color: bpy.props.FloatVectorProperty(
        name="Colour", subtype='COLOR', size=3, default=(1.0, 1.0, 1.0),
        min=0.0, max=1.0, update=update_color)
    spread: bpy.props.FloatProperty(
        name="Spread", default=math.radians(180.0), min=math.radians(1.0),
        max=math.radians(180.0), subtype='ANGLE', update=update_spread)
    use_temperature: bpy.props.BoolProperty(
        name="Use Temperature", default=False, update=update_color)
    temperature: bpy.props.FloatProperty(
        name="Temperature (K)", default=5600.0, min=1000.0, max=20000.0,
        step=10000, precision=0, update=update_color)
    glow_brightness: bpy.props.FloatProperty(
        name="Glow Brightness", default=1.0, min=0.0, soft_max=10.0,
        update=update_glow_brightness)
    emission_strength: bpy.props.FloatProperty(
        name="Emission Strength", default=5.0, min=0.0, soft_max=100.0,
        update=update_emission)
    visible_camera: bpy.props.BoolProperty(
        name="Visible to Camera", default=False, update=update_visibility)
    visible_reflections: bpy.props.BoolProperty(
        name="Visible in Reflections", default=False, update=update_visibility)


# ---------------------------------------------------------------------------
# Interface
# ---------------------------------------------------------------------------

SOURCE_PROPS = (
    "setside_level", "setside_solo", "setside_mute", "setside_effect", "setside_effect_speed",
    "setside_effect_amount", "setside_var_share", "setside_var_brightness",
    "setside_var_temperature", "setside_var_seed", "setside_var_dusk",
    "setside_var_dusk_start", "setside_var_dusk_end", "setside_screen_image",
    "setside_screen_step", "setside_glow_brightness",
)


def remove_lights(source, scene):
    """Delete everything Set-Side made for a mesh and give its faces their own
    materials back. Returns how many lights and glowing objects went."""
    removed = 0
    owned_mats = list(glow_materials(source))
    emitter = emitter_material(source)
    if emitter is not None:
        owned_mats.append(emitter)
    linking, groups = set(), {lightgroup_name(source)}
    for obj in [o for o in bpy.data.objects if o.setside_source == source]:
        coll = obj.light_linking.receiver_collection
        if coll is not None and coll.name.startswith("Set-Side "):
            linking.add(coll.name)
        if obj.lightgroup:
            groups.add(obj.lightgroup)
        delete_setside_object(obj)
        removed += 1

    # Faces back to their own materials
    mesh = source.data
    mats = list(mesh.materials)

    def ours(m):
        return m is not None and (m.get("setside_keep") or m.name.startswith("Set-Side "))

    if any(ours(m) for m in mats):
        n = len(mesh.polygons)
        index = [0] * n
        mesh.polygons.foreach_get("material_index", index)
        orig = [0] * n
        attr = mesh.attributes.get(ORIG_MAT_ATTR)
        if attr is not None and attr.domain == 'FACE':
            attr.data.foreach_get("value", orig)
        for i in range(n):
            current = mats[index[i]] if index[i] < len(mats) else None
            if not ours(current):
                continue
            if orig[i] > 0 and orig[i] - 1 < len(mats) and not ours(mats[orig[i] - 1]):
                index[i] = orig[i] - 1
            elif current.get("setside_keep") and current.get("setside_original") in bpy.data.materials:
                original = bpy.data.materials[current["setside_original"]]
                if original not in mats:
                    mats.append(original)
                index[i] = mats.index(original)
            else:
                index[i] = 0
        # Drop Set-Side slots and renumber the rest
        keep = [i for i, m in enumerate(mats) if not ours(m)]
        remap = {old: new for new, old in enumerate(keep)}
        new_index = [remap.get(i, 0) for i in index]
        mesh.materials.clear()
        for i in keep:
            mesh.materials.append(mats[i])
        if len(mesh.polygons):
            mesh.polygons.foreach_set("material_index", new_index)

    for name in (FACE_ATTR, "setside_glow", "setside_glow_on", ORIG_MAT_ATTR):
        attr = mesh.attributes.get(name)
        if attr is not None:
            mesh.attributes.remove(attr)
    mesh.update()

    for mat in owned_mats:
        if mat.name in bpy.data.materials and mat.users == 0:
            bpy.data.materials.remove(mat)
    linking |= {f"Set-Side Exclude {source.name}", f"Set-Side Only {source.name}"}
    for name in linking:
        coll = bpy.data.collections.get(name)
        if coll is not None and not any(
                o.light_linking.receiver_collection == coll for o in bpy.data.objects):
            bpy.data.collections.remove(coll)
    if source.lightgroup in groups:
        source.lightgroup = ""
    for group in groups:
        if group == "SetSide_Rest" or any(o.lightgroup == group for o in bpy.data.objects):
            continue
        for view_layer in scene.view_layers:
            groups_api = view_layer.lightgroups
            lg = groups_api.get(group) if hasattr(groups_api, "get") else None
            if lg is not None and hasattr(groups_api, "remove"):
                groups_api.remove(lg)
    for name in SOURCE_PROPS:
        if name in source.bl_rna.properties:
            source.property_unset(name)
    for key in ("setside_edit",):
        if key in source:
            del source[key]
    coll = bpy.data.collections.get(COLLECTION_NAME)
    if coll is not None and not coll.objects and not coll.children:
        bpy.data.collections.remove(coll)
    refresh_mix(scene)
    return removed


def orphan_objects(scene):
    """Set-Side lights whose mesh has been deleted."""
    return [o for o in scene.objects if o.setside_source is None
            and (o.setside_light_id > 0 or o.setside_faces)]


class SETSIDE_OT_remove_orphans(bpy.types.Operator):
    """Delete Set-Side lights left behind when their mesh was deleted"""
    bl_idname = "setside.remove_orphan_lights"
    bl_label = "Delete Leftover Lights"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        orphans = orphan_objects(context.scene)
        for obj in orphans:
            delete_setside_object(obj)
        coll = bpy.data.collections.get(COLLECTION_NAME)
        if coll is not None and not coll.objects and not coll.children:
            bpy.data.collections.remove(coll)
        self.report({'INFO'}, f"Deleted {len(orphans)} leftover light"
                    f"{'s' if len(orphans) != 1 else ''}")
        return {'FINISHED'}


class SETSIDE_OT_remove_lights(bpy.types.Operator):
    """Delete this mesh's Set-Side lights and glow, and give the faces their own materials back"""
    bl_idname = "setside.remove_face_lights"
    bl_label = "Remove Lights"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return edit_targets(context)[0] is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        source, targets = edit_targets(context)
        if source is None or not targets:
            self.report({'WARNING'}, "Select a mesh with Set-Side lights, or one of its lights")
            return {'CANCELLED'}
        was_editing = source.mode == 'EDIT'
        if context.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        removed = remove_lights(source, context.scene)
        if was_editing and context.view_layer.objects.active == source:
            bpy.ops.object.mode_set(mode='EDIT')
        self.report({'INFO'}, f"Removed {removed} from {source.name}. Openings cut with Cut "
                    "Openings stay: use Undo for those")
        return {'FINISHED'}


class SETSIDE_PT_faces_to_lights(bpy.types.Panel):
    bl_label = "Faces to Lights"
    bl_idname = "SETSIDE_PT_faces_to_lights"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Set-Side"  # every Set-Side tool shares one sidebar tab

    def draw(self, context):
        layout = self.layout

        # One big button: hide light outlines but keep their light. This uses
        # the viewport's Extras overlay, so camera and empty outlines hide too.
        overlay = context.space_data.overlay
        row = layout.row()
        row.scale_y = 1.4
        row.prop(overlay, "show_extras", toggle=True,
                 text="Light Shapes: Shown" if overlay.show_extras else "Light Shapes: Hidden",
                 icon='HIDE_OFF' if overlay.show_extras else 'HIDE_ON')

        layout.separator()
        col = layout.column(align=True)
        if context.mode == 'EDIT_MESH':
            col.operator(SETSIDE_OT_faces_to_lights.bl_idname, icon='LIGHT_AREA')
        else:
            col.label(text="Edit Mode: select faces to create", icon='INFO')
        row = col.row(align=True)
        row.operator(SETSIDE_OT_update_lights.bl_idname, icon='FILE_REFRESH')
        row.operator(SETSIDE_OT_remove_lights.bl_idname, text="Remove", icon='TRASH')

        orphans = orphan_objects(context.scene)
        if orphans:
            row = layout.row()
            row.alert = True
            row.operator(SETSIDE_OT_remove_orphans.bl_idname,
                         text=f"Delete {len(orphans)} Leftover Light{'s' if len(orphans) != 1 else ''}",
                         icon='ERROR')

        layout.label(text="After creating, adjust options in", icon='BLANK1')
        layout.label(text="the panel at the bottom left", icon='BLANK1')

        # Edit lights that already exist
        box = layout.box()
        source, targets = edit_targets(context)
        if not targets:
            box.label(text="Edit Lights: select a mesh with", icon='LIGHT')
            box.label(text="Set-Side lights, or one of its lights", icon='BLANK1')
            return
        box.label(text=f"Edit {len(targets)} light{'s' if len(targets) != 1 else ''} "
                  f"from {source.name}", icon='LIGHT')
        settings = context.scene.setside_edit
        col = box.column()
        col.use_property_split = True
        col.prop(settings, "power", text="Power per m\u00b2" if settings.power_per_area
                 else "Power")
        col.prop(settings, "power_per_area")
        col.prop(settings, "use_temperature")
        col.prop(settings, "temperature" if settings.use_temperature else "color")
        col.prop(settings, "spread")
        if glow_materials(source):
            col.prop(settings, "glow_brightness")
        if any(o.type == 'MESH' for o in targets):
            col.prop(settings, "emission_strength")
        col.prop(settings, "visible_camera")
        col.prop(settings, "visible_reflections")
        box.label(text="Changes apply as you edit", icon='INFO')


def menu_entry(self, context):
    self.layout.separator()
    self.layout.operator(SETSIDE_OT_faces_to_lights.bl_idname, icon='LIGHT_AREA')
    self.layout.operator(SETSIDE_OT_update_lights.bl_idname, icon='FILE_REFRESH')
    self.layout.operator(SETSIDE_OT_remove_lights.bl_idname, icon='TRASH')


classes = (
    SETSIDE_EditSettings,
    SETSIDE_OT_faces_to_lights,
    SETSIDE_OT_update_lights,
    SETSIDE_OT_remove_lights,
    SETSIDE_OT_remove_orphans,
    SETSIDE_PT_faces_to_lights,
)


def register():
    if hasattr(bpy.types.Scene, "setside_edit"):
        raise RuntimeError("Another copy of Set-Side Faces to Lights is already enabled. "
                           "Disable it in Preferences > Add-ons first")
    # Data stored on each light, so it can be re-fitted later
    bpy.types.Object.setside_source = bpy.props.PointerProperty(
        type=bpy.types.Object, name="Source Mesh")
    bpy.types.Object.setside_face = bpy.props.IntProperty(name="Source Face", default=-1)
    bpy.types.Object.setside_faces = bpy.props.StringProperty(name="Source Faces", default="")
    bpy.types.Object.setside_kind = bpy.props.StringProperty(name="Light Kind", default="AREA")
    bpy.types.Object.setside_flip = bpy.props.BoolProperty(name="Flipped", default=False)
    bpy.types.Object.setside_offset = bpy.props.FloatProperty(name="Offset", default=0.001)
    bpy.types.Object.setside_light_id = bpy.props.IntProperty(name="Light ID", default=0)
    bpy.types.Object.setside_fit = bpy.props.StringProperty(name="Light Size", default="")
    bpy.types.Object.setside_scale = bpy.props.FloatProperty(name="Light Scale", default=1.0)
    bpy.types.Object.setside_glow_brightness = bpy.props.FloatProperty(
        name="Glow Brightness", default=1.0)
    bpy.types.Object.setside_follow = bpy.props.BoolProperty(
        name="Follow Modifiers", default=False)
    bpy.types.Object.setside_power_density = bpy.props.FloatProperty(
        name="Power per m\u00b2", default=0.0)
    bpy.types.Object.setside_watts = bpy.props.FloatProperty(name="Power Before Mixer", default=0.0)
    bpy.types.Object.setside_var = bpy.props.FloatProperty(name="Variation Factor", default=1.0)
    bpy.types.Object.setside_screen = bpy.props.BoolProperty(name="Screen Light", default=False)

    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.setside_edit = bpy.props.PointerProperty(type=SETSIDE_EditSettings)
    bpy.types.VIEW3D_MT_edit_mesh_faces.append(menu_entry)
    # Edit Lights shows the selected mesh's own values
    _subscribe()
    if _on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load)


def unregister():
    bpy.types.VIEW3D_MT_edit_mesh_faces.remove(menu_entry)
    del bpy.types.Scene.setside_edit
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    bpy.msgbus.clear_by_owner(_msgbus_owner)
    if _on_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load)
    for name in ("setside_screen_step", "setside_screen_image", "setside_var_dusk_end",
                 "setside_var_dusk_start", "setside_var_dusk", "setside_var_seed",
                 "setside_var_temperature", "setside_var_brightness", "setside_var_share",
                 "setside_screen", "setside_var",
                 "setside_effect_amount", "setside_effect_speed", "setside_effect",
                 "setside_mute", "setside_solo", "setside_level", "setside_watts"):
        if hasattr(bpy.types.Object, name):
            delattr(bpy.types.Object, name)
    del bpy.types.Object.setside_follow
    del bpy.types.Object.setside_glow_brightness
    del bpy.types.Object.setside_scale
    del bpy.types.Object.setside_fit
    del bpy.types.Object.setside_power_density
    del bpy.types.Object.setside_light_id
    del bpy.types.Object.setside_offset
    del bpy.types.Object.setside_flip
    del bpy.types.Object.setside_kind
    del bpy.types.Object.setside_faces
    del bpy.types.Object.setside_face
    del bpy.types.Object.setside_source
