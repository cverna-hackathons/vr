# Adds viewport-only scale helpers and a glTF export setup to a model .blend, in place. Safe to run again (it replaces its own helpers).
#
#   blender --background data/objects/crate.blend --python tools/blender_helpers.py [-- --collection Name]
#
# What it does to the open .blend, then saves it and writes <name>.glb next to it:
#   * puts the model objects in a collection (default: the .blend's file name, "crate" -> "Crate"; a copied .blend's one exporting
#     collection is renamed to match) that has a glTF collection
#     exporter writing //<name>.glb (Collection properties > Exporters; File > Export All Collections re-exports it),
#   * adds a collection "Helpers (not exported)", render-disabled and outside the exporter's collection, holding: the 1 m map
#     cell (outline + 1 m cube), a 5 x 5 m grid of cells, a 1.75 m person, a ruler to 3.5 m (the demo tree is 3.2 m) and an
#     arrow for the model's front. Delete or hide them freely; running the script again rebuilds them.
# Axes: explore's +Z (south) is Blender's -Y, so a model's front should face -Y in Blender; explore's +X (east) is Blender's +X.
import bpy, bmesh, os, sys, math

HELPERS = "Helpers (not exported)"


def _viewport_material(name, rgba):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color = rgba   # solid-mode viewport colour only; helpers are never rendered or exported
    return m


def _add_object(name, data, coll, mat=None, location=(0, 0, 0), rotation=(0, 0, 0)):
    o = bpy.data.objects.new(name, data)
    o.location, o.rotation_euler = location, rotation
    coll.objects.link(o)
    if mat is not None:
        data.materials.append(mat)
        o.color = mat.diffuse_color
    return o


def _wire(name, verts, edges, coll, mat, **kw):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, edges, [])
    o = _add_object(name, me, coll, mat, **kw)
    o.display_type = 'WIRE'
    return o


def _label(name, text, coll, mat, location, size=0.12, rotation=(math.radians(90), 0, 0)):
    cu = bpy.data.curves.new(name, 'FONT')
    cu.body, cu.size, cu.align_x = text, size, 'CENTER'
    return _add_object(name, cu, coll, mat, location=location, rotation=rotation)


def _remove_old_helpers():
    old = bpy.data.collections.get(HELPERS)
    if not old:
        return
    for o in list(old.objects):
        data = o.data
        bpy.data.objects.remove(o)
        if data and data.users == 0:
            (bpy.data.meshes if isinstance(data, bpy.types.Mesh) else bpy.data.curves).remove(data)
    bpy.data.collections.remove(old)


def _build_helpers(scene):
    _remove_old_helpers()
    coll = bpy.data.collections.new(HELPERS)
    scene.collection.children.link(coll)
    coll.hide_render = True
    grey = _viewport_material("helper_grey", (0.55, 0.55, 0.55, 1))
    orange = _viewport_material("helper_orange", (1.0, 0.55, 0.05, 1))
    blue = _viewport_material("helper_blue", (0.2, 0.45, 0.95, 1))
    green = _viewport_material("helper_green", (0.2, 0.8, 0.35, 1))
    white = _viewport_material("helper_white", (0.95, 0.95, 0.95, 1))

    # 5 x 5 m grid of 1 m map cells, centred so the model's cell is the middle one (lines at 0.5, 1.5, 2.5 ...).
    gv, ge = [], []
    for i in range(6):
        p = -2.5 + i
        gv += [(p, -2.5, 0), (p, 2.5, 0), (-2.5, p, 0), (2.5, p, 0)]
        ge += [(4 * i, 4 * i + 1), (4 * i + 2, 4 * i + 3)]
    _wire("Helper_CellGrid", gv, ge, coll, grey)

    # The map cell the model must fit in: a 1 x 1 m outline on the ground and a 1 m cube outline.
    sq = [(-.5, -.5, 0.003), (.5, -.5, 0.003), (.5, .5, 0.003), (-.5, .5, 0.003)]
    _wire("Helper_Cell", sq, [(0, 1), (1, 2), (2, 3), (3, 0)], coll, orange)
    cv = [(x, y, z) for z in (0, 1) for (x, y) in ((-.5, -.5), (.5, -.5), (.5, .5), (-.5, .5))]
    ce = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    _wire("Helper_1m_Cube", cv, ce, coll, orange)
    _label("Helper_Label_Cell", "1 map cell = 1 x 1 m", coll, orange, (0, 0.5, 1.12), size=0.09)

    # A 1.75 m person (the explore default eye height is 1.63 m).
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=12, radius1=0.2, radius2=0.2, depth=1.45)
    bmesh.ops.translate(bm, vec=(0, 0, 0.725), verts=bm.verts)
    head = bmesh.ops.create_uvsphere(bm, u_segments=12, v_segments=8, radius=0.125)
    bmesh.ops.translate(bm, vec=(0, 0, 1.625), verts=head["verts"])
    pm = bpy.data.meshes.new("Helper_Person")
    bm.to_mesh(pm)
    bm.free()
    _add_object("Helper_Person", pm, coll, blue, location=(1.5, 0, 0))
    _label("Helper_Label_Person", "person 1.75 m", coll, blue, (1.5, 0, 1.9), size=0.1)

    # Ruler to 3.5 m with a tick every 0.5 m (longer every 1 m).
    rv, re = [(0, 0, 0), (0, 0, 3.5)], [(0, 1)]
    for i in range(1, 8):
        z, w = 0.5 * i, (0.18 if i % 2 == 0 else 0.1)
        rv += [(0, 0, z), (w, 0, z)]
        re += [(len(rv) - 2, len(rv) - 1)]
    _wire("Helper_Ruler", rv, re, coll, white, location=(-1.5, 0, 0))
    for m in (1, 2, 3):
        _label("Helper_Label_%dm" % m, "%d m" % m, coll, white, (-1.2, 0, m), size=0.09)

    # Front arrow on the ground: explore's +Z (south) = Blender -Y. The model's front should face this way.
    fv = [(0, -0.7, 0.003), (0, -1.6, 0.003), (-0.12, -1.38, 0.003), (0.12, -1.38, 0.003)]
    _wire("Helper_FrontArrow", fv, [(0, 1), (1, 2), (1, 3)], coll, green)
    _label("Helper_Label_Front", "FRONT (explore: south, +Z)", coll, green, (0, -1.85, 0.003), size=0.09, rotation=(0, 0, 0))
    return coll


def add_helpers(collection_name=None):
    scene = bpy.context.scene
    if not bpy.data.filepath:
        raise SystemExit("Save the .blend first (the glTF path is relative to it).")
    stem = os.path.splitext(os.path.basename(bpy.data.filepath))[0]
    name = collection_name or stem.capitalize()
    helpers = _build_helpers(scene)

    # The model collection: reuse it if it exists, otherwise gather every other object into a new one.
    model = bpy.data.collections.get(name)
    if model is None:   # a copied .blend: adopt its single exporting collection (e.g. "Crate") under the new name
        copied = [c for c in scene.collection.children if c.name != HELPERS and len(c.exporters)]
        if len(copied) == 1:
            model = copied[0]
            model.name = name
    if model is None:
        model = bpy.data.collections.new(name)
        scene.collection.children.link(model)
        for o in list(scene.collection.objects):
            scene.collection.objects.unlink(o)
            model.objects.link(o)
        for c in list(scene.collection.children):
            if c not in (model, helpers):
                scene.collection.children.unlink(c)
                for o in list(c.all_objects):
                    model.objects.link(o)
    if not model.objects:
        raise SystemExit("Collection '%s' has no objects to export." % name)

    # glTF collection exporter stored in the .blend (File > Export All Collections).
    if len(model.exporters) == 0:
        bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection.children[name]
        bpy.ops.collection.exporter_add(name="IO_FH_gltf2")
    props = model.exporters[0].export_properties
    props.filepath = "//%s.glb" % stem
    props.export_format = 'GLB'
    props.export_yup = True
    props.export_apply = True

    for area in (bpy.context.screen.areas if bpy.context.screen else []):
        if area.type == 'VIEW_3D':
            area.spaces[0].shading.type = 'SOLID'
            area.spaces[0].shading.color_type = 'MATERIAL'

    bpy.ops.wm.save_mainfile()
    glb = os.path.join(os.path.dirname(bpy.data.filepath), stem + ".glb")
    bpy.ops.collection.export_all()
    print("helpers added; wrote", bpy.data.filepath, "and", glb, "(exists: %s)" % os.path.exists(glb))


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    add_helpers(argv[argv.index("--collection") + 1] if "--collection" in argv else None)
