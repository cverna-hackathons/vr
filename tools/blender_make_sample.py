# One-off generator for the sample model: data/objects/crate.blend (texture packed inside) + crate.glb, a 1 m wooden crate with its
# origin at the middle of its base. It REFUSES to overwrite an existing crate.blend (you may have edited it in Blender);
# pass `-- --force` to regenerate from scratch.
#
#   blender --background --python tools/blender_make_sample.py [-- --force]
#
# Scale helpers and the glTF export setup come from tools/blender_helpers.py, which you can run on any model .blend:
#   blender --background data/objects/mything.blend --python tools/blender_helpers.py
# Use this file as a template for your own models: Y-up glTF export, origin at the foot, about 1 m wide to fit one map cell.
import bpy, bmesh, os, sys, math

here = os.path.dirname(os.path.abspath(__file__))
blend = os.path.normpath(os.path.join(here, "..", "data", "objects", "crate.blend"))
argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if os.path.exists(blend) and "--force" not in argv:
    raise SystemExit("%s already exists; not overwriting it (it may hold your edits). Pass `-- --force` to regenerate." % blend)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'

# Procedural texture: planks with a dark frame (256 x 256).
N = 256
img = bpy.data.images.new("crate_tex", N, N, alpha=False)
px = []
for y in range(N):
    for x in range(N):
        edge = min(x, y, N - 1 - x, N - 1 - y)
        plank = (y // 32) % 2
        base = (0.55, 0.36, 0.18) if plank else (0.62, 0.42, 0.22)
        grain = 0.06 * math.sin(x * 0.35 + (y // 32) * 3.0)
        c = tuple(max(0.0, v + grain) for v in base)
        if edge < 14 or abs(x - y) < 7 or abs(x - (N - 1 - y)) < 7: c = (0.30, 0.18, 0.08)
        if y % 32 < 2: c = tuple(v * 0.6 for v in c)
        px += [c[0], c[1], c[2], 1.0]
img.pixels = px
img.pack()

bm = bmesh.new()
bmesh.ops.create_cube(bm, size=1.0)
bmesh.ops.translate(bm, vec=(0, 0, 0.5), verts=bm.verts)     # Blender is Z-up; the exporter converts to Y-up
uv = bm.loops.layers.uv.new("UVMap")
for f in bm.faces:                                           # each face covers the whole texture
    for loop, co in zip(f.loops, ((0, 0), (1, 0), (1, 1), (0, 1))):
        loop[uv].uv = co
me = bpy.data.meshes.new("Crate")
bm.to_mesh(me)
bm.free()
mat = bpy.data.materials.new("CrateMat")
mat.use_nodes = True
tex = mat.node_tree.nodes.new("ShaderNodeTexImage")
tex.image = img
bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
mat.node_tree.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
me.materials.append(mat)
scene.collection.objects.link(bpy.data.objects.new("Crate", me))

bpy.ops.wm.save_as_mainfile(filepath=blend, compress=True)
sys.path.insert(0, here)
import blender_helpers
blender_helpers.add_helpers()
