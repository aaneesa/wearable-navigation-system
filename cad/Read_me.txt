BeltStick | Mechanical prototype | Dimensions in mm

FUSION
Unzip the package first. Upload BeltStick_Assembly.step to Fusion and open the converted design. The housing and rear cover share their assembled coordinates. Hide the cover or use Section Analysis to see the board supports and belt path. Individual STEP files let you work on either body separately. STEP imports as solid geometry. The STEP solids preserve the faceted surfaces of the approved Three.js geometry. For dimensional changes, edit Rebuild_meshes.py (Python 3, NumPy, manifold3d 3.5.5), regenerate the STLs, then use Mesh_to_STEP.py in a FreeCAD Python environment to regenerate the solids. Both scripts operate in their own directory. The FCStd file contains the two solid bodies.

PRINTING
Housing.stl and RearCover.stl are the two enclosure parts. Import in millimetres at 100%. Check that the housing measures 180 x 110 x 65 mm. The sensor supports, ESP32 tray and MPU cradle are integral to the housing. The four hooks are integral to the cover. The optional Bore_fit_coupon.stl has openings of 16.4, 16.6, 16.8, 16.9 and 17.0 mm in order from the notched end. Test your actual sensor and print settings on the coupon first.

Place the cover flat with its clips pointing up. The housing exports with its open rear rim on the XY plane. Inspect support access underneath the internal trays and around the angled bores before slicing. Keep removable support out of latch pockets and belt tunnels where possible.

FEATURES
Five pairs of 16.9 mm bores on 26 mm centers. Center axis is forward (+Z). Peripheral axes point 45 degrees left, right, up and down. Model +Y is up when worn. The thin MPU shelf is horizontal in this orientation. The cover has four snap clips and finger cutouts. Two vertical 36 x 5 mm openings carry the strap through the structural housing.

FIT TO CHECK ON YOUR HARDWARE
The preview used a 34 mm belt, a 22 x 15.3 mm MPU board and a 23.6 x 51 mm ESP32-S3 board. Measure your board outlines, headers, USB plug and cable bend space. Check that the HC-SR04 PCBs can be inserted, supported and retained with the actual wiring attached. The supplied solids are the enclosure only. Battery, switch and external USB openings were outside the approved layout. Physical snap force, retention, belt load and repeated release cycles need prototype testing.
