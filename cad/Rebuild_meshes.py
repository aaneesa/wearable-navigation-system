"""Generate both printable parts using Python, NumPy and manifold3d 3.5.5. Units are millimetres."""
from itertools import product
from math import cos, sin, radians

import numpy as np
from manifold3d import Manifold as Solid, OpType, Error

RX, RY, RZ = 90, 55, 65
FACE_W, FACE_H = 50, 29
HOLE = 16.9
UNION, SUBTRACT = OpType.Add, OpType.Subtract


def union(*solids):
    return Solid.batch_boolean(solids, UNION)


def cut(base, *tools):
    return Solid.batch_boolean([base, *tools], SUBTRACT)


def box(x0, x1, y0, y1, z0, z1):
    return Solid.cube((x1-x0, y1-y0, z1-z0)).translate((x0,y0,z0))


def ellipse(rx, ry, z0, z1):
    return Solid.cylinder(z1-z0, 1, circular_segments=192).scale((rx,ry,1)).translate((0,0,z0))


def frame(center, normal, tangent):
    n, u = np.array(normal), np.array(tangent)
    return np.column_stack((u, np.cross(n, u), n, center))


def seat(yaw, pitch):
    a, b = radians(yaw), radians(pitch)
    n = np.array((sin(a)*cos(b),sin(b),cos(a)*cos(b)))
    u = np.array((cos(a),0,-sin(a)))
    pos = n / np.linalg.norm(n / [RX,RY,RZ])
    if yaw:
        # Shift along the seating plane to keep the leading corner within z=65.
        shift = (pos[2] + FACE_W/2*abs(u[2]) - 64.7) / abs(u[2])
        pos += np.sign(yaw)*shift*u
    return frame(pos,n,u)


seats = [seat(*angles) for angles in [(0,0),(-45,0),(45,0),(0,45),(0,-45)]]


def loft(xy_sections):
    return Solid.hull_points(np.array([(sx*w/2,sy*h/2,z)
         for w,h,z in xy_sections for sx,sy in product((-1,1), repeat=2)]))


def skin_envelope(inward):
    outer = Solid.sphere(1, 88).scale((RX-inward, RY-inward, RZ-inward))
    pad_outer = loft([(FACE_W, FACE_H,-inward),(63,43,-12-inward),(53,33,-21-inward)])
    if inward:
        # Carry the cavity up to the back of the entire transducer region.
        pad_outer = loft([(46,25,-inward),(53,36,-12-inward),(45,24,-21-inward)])
    outer = union(outer, *(pad_outer.transform(m) for m in seats))
    for m in seats:
        normal = m[:,2]
        outer = outer.trim_by_plane(tuple(-normal), inward - float(normal @ m[:,3]))
    return outer.trim_by_plane((0,0,1),0)


outer = skin_envelope(0)
inner = skin_envelope(4)
# The shell is nominally 3 mm at the seating faces. At the rear the
# 4 mm side wall also gives a recessed cover a continuous stop ledge.
inner_faces = []
for matrix in seats:
    relief = loft([(46,25,-3),(46,25,-5),(46,25,-11)])
    inner_faces.append(relief.transform(matrix))
inner = union(inner,*inner_faces)
cover_recess = ellipse(87.15,52.15,-1,3.1)
housing = cut(outer,inner,cover_recess)

belt_blocks = [box(72,94,-21,21,4.1,13.4),box(-94,-72,-21,21,4.1,13.4)]
belt_tool = box(-100,100,-18,18,6.1,11.1)
housing = cut(union(housing, *(b ^ outer for b in belt_blocks)),belt_tool)

# Headed cantilevers in the cover and cored receiver blocks in the housing.
def latch_frame(a):
    a = radians(a)
    out = np.array([cos(a)/86, sin(a)/51, 0])
    out /= np.linalg.norm(out)
    origin = np.array([86*cos(a),51*sin(a),0]) - 3.2*out
    tangent = np.array([out[1],-out[0],0])
    return np.column_stack((tangent, out, np.array([0,0,1]), origin))


clip_frames = [latch_frame(a) for a in (52,128,232,308)]
clip = union(box(-4,4,-.85,.85,2.85,12.4),
    Solid.hull_points(np.array([(x,y,z) for x in [-4,4] for y,z in
          [(.65,9.5),(1.85,9.5),(.85,13.1),(.65,13.1)]])))
receiver = box(-5.3,5.3,.95,8.3,6.15,14)
slot_ramp = box(-4.3,4.3,-2.3,2.1,9.28,13.35)
for m in clip_frames:
    housing = cut(union(housing,receiver.transform(m) ^ outer),slot_ramp.transform(m))

# The cover's 3 mm alignment rim fits just inside the rear opening.
rim = cut(ellipse(85.35,50.35,2.8,5.8),ellipse(82.8,47.8,2.7,6.1))
rim = cut(rim,box(70,94,-22,22,3.93,6.2),box(-94,-70,-22,22,3.93,6.2))
cover = union(ellipse(86.8,51.8,0,3),rim)
for side in (-1,1):
    notch = Solid.cylinder(6.8,9,circular_segments=80).translate((0,side*53,-.5))
    cover = cut(cover,notch)

bores = []
boards = []
for m in seats:
    for x in (-13,13):
        bores.append(Solid.cylinder(9,HOLE/2,circular_segments=96).translate((x,0,-6)).transform(m))
    boards.append(box(-22.5,22.5,-10,10,-14.9,-13.3).transform(m))
housing = cut(housing,*bores)

# Flat MPU cradle. Two narrow legs link the low L-bracket to the shell.
imu = union(box(17,46,-39,-36.5,14,37),box(17,46,-36.7,-28.5,14,16.2),
    box(17,19.4,-36.7,-34.4,16,35.5),box(43.6,46,-36.7,-34.4,16,35.5),
    box(17,38,-36.7,-34.4,34.2,37)).translate((7,6,-7))
feet = [box(27,31,-54,-32.8,13.5,18),box(45,49,-54,-32.8,13.5,19)]
print("Bracket direct shell overlap before feet:", (housing ^ imu).volume())
housing = union(housing,*(foot ^ outer for foot in feet))
# Rear-facing ESP32-S3 tray, with space beside the connector and side pin rows.
esp_tray = union(box(-62,-32,-26,33,20.8,23),
    box(-62,-59.4,-26,33,17.8,21),box(-34.6,-32,-26,33,17.8,21),
    box(-62,-32,30.5,33,17.8,21),box(-62,-56.8,-26,-23.6,17.8,21),
    box(-38,-32,-26,-23.6,17.8,21)).translate((4,0,-3.5))
esp_feet = [box(-90,-57.8,-16,-11,17,20),box(-90,-57.8,16,21,17,20)]
housing = union(housing,*(foot ^ outer for foot in esp_feet))

# Short ledges support the sensor PCBs independently of their transducer cans.
supports = []
for i,m in enumerate(seats):
    brackets = []
    for x in (-19.5,19.5):
        for y in (-11.3,11.3):
            local = Solid.hull_points(np.array([(x+dx,y+dy,z) for dx,dy in product((-2.2,2.2),(-.9,.9)) for z in (-13.4,-2.6)]))
            brackets.append(local.transform(m) ^ outer)
    supports.append(union(*brackets))
housing_with_sensor_supports = union(housing,*supports)

clips = union(*(clip.transform(m) for m in clip_frames))
assert len(housing_with_sensor_supports.decompose()) == 1
assert len(union(cover,clips).decompose()) == 1
print("bounds",housing.bounding_box())
assert all(abs(a-b)<.01 for a,b in zip(housing.bounding_box(),(-90,-55,0,90,55,65)))
assert all((b ^ housing_with_sensor_supports).volume() < .1 for b in bores)
for name,obj in [('cover',cover),('clip',clips),('imu',imu),('esp',esp_tray)]:
    print(name, 'housing overlap mm³', round((obj ^ housing_with_sensor_supports).volume(),2))
print('Seated cover assembly overlap:', (housing_with_sensor_supports ^ union(cover,clips)).volume())
print('Cover vs rim vs housing:', (housing ^ ellipse(86.8,51.8,0,3)).volume(),(housing ^ rim).volume())
print('Bracket joins:',len(union(housing_with_sensor_supports,imu,esp_tray).decompose()))
for i,board in enumerate(boards):
    print('sensor',i,'shell',round((board ^ housing_with_sensor_supports).volume(),2),'imu',round((board ^ imu).volume(),2),'esp',round((board ^ esp_tray).volume(),2),
          'other PCB', [round((board ^ other).volume(),2) for other in boards[:i]])
imu_board = box(27.5,49.5,-30.5,-25.9,12.2,27.5)
esp_board = box(-54.8,-31.2,-22,29,13.9,17.3)
for name,obj in [('IMU electronics',imu_board),('ESP electronics',esp_board)]:
    print(name,'housing',round((obj ^ housing_with_sensor_supports).volume(),2),'PCB clashes',
          [round((obj ^ b).volume(),2) for b in boards])
for i,m in enumerate(seats):
    cans = union(*(Solid.cylinder(13,8,circular_segments=96).translate((x,0,-13.3)).transform(m) for x in (-13,13)))
    connector = box(-5,5,-15,-9,-19,-13.3).transform(m)
    for label,part in [('cans',cans),('header',connector)]:
        print(i,label,[round((part ^ s).volume(),2) for s in (housing_with_sensor_supports, imu,esp_tray,imu_board,esp_board)])
print('Belt slot obstruction mm³:',(belt_tool ^ union(housing_with_sensor_supports,cover,clips,imu,esp_tray)).volume())



from pathlib import Path
from struct import pack
root=Path(__file__).resolve().parent
for name,part in [('Housing',union(housing_with_sensor_supports,imu,esp_tray)),('RearCover',union(cover,clips))]:
    assert len(part.decompose())==1 and part.status()==Error.NoError
    mesh=part.to_mesh()
    triangles=mesh.vert_properties[mesh.tri_verts,:3]
    with (root/(name+'.stl')).open('wb') as f:
        f.write(bytes(80)+pack('<I',len(triangles)))
        for tri in triangles:
            n=np.cross(tri[1]-tri[0],tri[2]-tri[0])
            n/=np.linalg.norm(n)
            f.write(pack('<12fH',*n,*tri.flatten(),0))
