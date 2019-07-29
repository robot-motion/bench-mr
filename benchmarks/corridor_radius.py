from mpb import MPB, MultipleMPB
import sys

CORRIDOR_RADII = [3, 4, 5, 6, 7, 8]


def main():
    pool = MultipleMPB()
    for r in CORRIDOR_RADII:
        m = MPB()
        m.set_corridor_grid_env(100, 100, 80, radius=r)
        m.set_planners(['rrt', 'rrt_star', 'informed_rrt_star', 'bfmt', 'bit_star', 'cforest', 'est', 'sbpl_adstar', 'theta_star'])
        m.set_steer_functions(['posq', 'reeds_shepp'])
        pool.benchmarks.append(m)
    pool.run_parallel("corridor_radius", 5)


if __name__ == "__main__":
    main()
