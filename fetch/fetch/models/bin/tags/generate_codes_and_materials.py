#!/usr/bin/env python
import os
for i in xrange(0,12):
    os.system("convert ~/fetch_ws/src/fetch/models/bin/tags/MarkerData_%d.png -bordercolor white -border 100x100 " +
              "~/fetch_ws/src/fetch/models/bin/tags/MarkerData_%d.png" % i) #<2>
    with open("product_%d.material" % i, 'w') as f: #<3>
      f.write("""
material product_%d {
  receive_shadows on
  technique {
    pass {
      ambient 1.0 1.0 1.0 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.5 0.5 0.5 1.0
      lighting on
      shading gouraud
      texture_unit { texture MarkerData_%d.png }
    }
  }
}
""" % (i, i))
