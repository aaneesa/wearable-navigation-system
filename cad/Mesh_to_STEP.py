from pathlib import Path
import FreeCAD as A,Mesh,Part,Import
p=Path(__file__).resolve().parent
d=A.newDocument('BeltStick')
objs=[]
for name in ['Housing','RearCover']:
 m=Mesh.Mesh(str(p/(name+'.stl')))
 print(name,m.isSolid(),m.countComponents(),flush=True)
 s=Part.Shape()
 s.makeShapeFromMesh(m.Topology,.000001)
 sh=Part.makeSolid(s.Shells[0]).removeSplitter()
 print(name,sh.isValid(),len(sh.Faces),flush=True)
 obj=d.addObject('Part::Feature',name)
 obj.Shape=sh;objs.append(obj)
 Part.export([obj],str(p/(name+'.step')))
d.recompute()
Import.export(objs,str(p/'BeltStick_Assembly.step'))
d.saveAs(str(p/'BeltStick.FCStd'))
