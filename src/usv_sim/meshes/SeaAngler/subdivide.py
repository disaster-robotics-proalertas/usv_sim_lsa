import bpy, bmesh
from bpy import context as C

bpy.ops.object.mode_set(mode='EDIT')

bm = bmesh.from_edit_mesh(C.object.data)

edges = []

for i in range(-10, 10, 2):
        ret = bmesh.ops.bisect_plane(bm, geom=bm.verts[:]+bm.edges[:]+bm.faces[:], plane_co=(i,0,0), plane_no=(-1,0,0))
        bmesh.ops.split_edges(bm, edges=[e for e in ret['geom_cut'] if isinstance(e, bmesh.types.BMEdge)])

for i in range(-10, 10, 2):
        ret = bmesh.ops.bisect_plane(bm, geom=bm.verts[:]+bm.edges[:]+bm.faces[:], plane_co=(0,i,0), plane_no=(0,1,0))
        bmesh.ops.split_edges(bm, edges=[e for e in ret['geom_cut'] if isinstance(e, bmesh.types.BMEdge)])


bmesh.update_edit_mesh(C.object.data)

bpy.ops.mesh.separate(type='LOOSE')
bpy.ops.object.mode_set(mode='OBJECT')
