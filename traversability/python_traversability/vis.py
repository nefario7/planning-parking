import pyvista as pv


def show_labeled_cloud(points, labels):
    plotter = pv.Plotter()
    plotter.add_mesh(pv.PolyData(points), scalars=labels)
    plotter.show()
