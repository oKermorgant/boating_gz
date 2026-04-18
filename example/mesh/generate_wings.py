#!/usr/bin/env python3

import pylab as pl
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import sys
import pyvista as pv
from scipy.spatial import ConvexHull
import numpy as np


# overall shape from length
def shape(l, h, t, axis):
    p = []
    for xl in list(pl.linspace(0,0.1,100)) + list(pl.linspace(0.1, 1, 100)):
        x = l*(axis-xl)

        # NACA airfoil https://en.wikipedia.org/wiki/NACA_airfoil
        y = 5*t*(.2969*np.sqrt(xl) - 0.126*xl-0.3516*xl**2+0.2843*xl**3-0.1015*xl**4)
        p.append([x,y, h])
        p.append([x,-y, h])
    return pl.array(p)


class Spec:
    def __init__(self, name, h, lh, l0, t):
        self.name = name
        self.h = h
        self.lh = lh
        self.l0 = l0
        self.t = t
        self.axis = 0.3 if name == 'wing' else 0.05

    def flipped(self):
        return self.h < 0

    def points(self):

        pt = shape(self.lh, self.h, self.t, self.axis)
        pb = shape(self.l0, 0, self.t, self.axis)

        if self.h > 0:
            return pl.vstack((pt,pb))
        return pl.vstack((pb,pt))


specs = [Spec('wing', 2.5, 1., 1.3, .15),
         Spec('rudder', -0.5, .2, .3, .05)]

for spec in specs:

    nodes = spec.points()

    hull = ConvexHull(nodes)
    faces = hull.simplices

    faces = pl.column_stack((3*pl.ones((len(hull.simplices), 1), dtype=np.int32), hull.simplices)).flatten()
    poly = pv.PolyData(hull.points, faces)
    poly.compute_normals(inplace=True,consistent_normals=True,flip_normals=spec.flipped())

    print(f'Writing {spec.name}.stl')
    if '-p' in sys.argv:
        pv.plot(poly)
    pv.save_meshio(spec.name+'.stl',poly)
