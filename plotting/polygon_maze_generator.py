import numpy as np


class PolygonMazeGenerator:
    @staticmethod
    def get_limits(obstacles):
        pmin = np.min([np.min(points, axis=0) for points in obstacles], axis=0)
        pmax = np.max([np.max(points, axis=0) for points in obstacles], axis=0)
        return (pmin, pmax)

    @staticmethod
    def save_svg(obstacles, svg_filename):
        pmin, pmax = PolygonMazeGenerator.get_limits(obstacles)
        width = pmax[0] - pmin[0]
        height = pmax[1] - pmin[1]
        with open(svg_filename, "w") as file:
            file.write(
                f'<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n<svg version="1.1" viewBox="{pmin[0]} {pmin[1]} {width} {height}">\n\t<g id="maze">\n')
            for i_ob, ob in enumerate(obstacles):
                file.write(
                    f'\t\t<path id="path_{i_ob}" style="fill:#c7c7c7;fill-opacity:1;stroke:#000000;stroke-width:0.26458332px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1" \n\t\t\td="M ')
                for i in range(ob.shape[0]):
                    file.write('%.5f,%.5f ' % (ob[i, 0], -ob[i, 1]))  # flip y to match Inkscape
                file.write('z" />\n')
            file.write('\t</g>\n</svg>')

    @staticmethod
    def plot(obstacles):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Polygon
        from matplotlib.collections import PatchCollection        
        polygons = [Polygon(points, True) for points in obstacles]
        ax = plt.gca()
        ax.add_collection(PatchCollection(polygons, color='lightgray', edgecolor='gray', alpha=0.8))
        pmin, pmax = PolygonMazeGenerator.get_limits(obstacles)
        ax.set_xlim([pmin[0], pmax[0]])
        ax.set_ylim([pmin[1], pmax[1]])
        plt.axis('equal')
        plt.grid()

    @staticmethod
    def create_convex(min_width=0.5, min_height=0.5, num_points=10, std_width=2., std_height=2.):
        from scipy.spatial import ConvexHull
        points = np.random.randn(num_points, 2)
        points[:, 0] *= std_width
        points[:, 1] *= std_height
        points[:, 0] += np.sign(points[:, 0]) * min_width
        points[:, 1] += np.sign(points[:, 1]) * min_height
        hull = ConvexHull(points)
        return points[hull.vertices]
