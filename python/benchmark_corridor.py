import matplotlib
matplotlib.use("Agg")
from mpb import MPB, MultipleMPB
import matplotlib as mpl
import matplotlib.pyplot as plt
from definitions import stat_names
import datetime

mpb = MPB()
mpb.set_corridor_grid_env(radius = 3)
mpb.set_planners(['rrt', 'rrt_star', 'informed_rrt_star', 'sorrt_star', 'prm_star', 'cforest', 'bfmt', 'spars2', 'sbpl_adstar', 'sbpl_mha'])
mpb.set_steer_functions(['reeds_shepp'])
mpb.run(id='test_run', runs=12)
mpb.plot_planner_stats(metrics=", ".join(stat_names.keys()), max_plots_per_line=4, headless=True)
plt.savefig("corridor_stats.png", dpi=300)
mpb.visualize_trajectories(metrics=", ".join(stat_names.keys()), max_plots_per_line=4, headless=True)
plt.tight_layout()
plt.savefig("corridor_trajectories.png", dpi=300)

with open("corridor.md", "w") as f:
	f.write('''---
layout: default
title:  "Corridor Results"
date:   ''')
	f.write(datetime.datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S %z"))
	f.write('''
parent: "Results"
nav_order: 1
---

# Results from Procedurally Generated Corridors

These results have been automatically generated from our Continuous Integration (CI) system.
{: .fs-6 .fw-300 }
''')
	f.write(f'''
### Time stamp: <b>{datetime.datetime.now().astimezone().strftime("%Y-%m-%dT%H:%M:%S %z")}</b>''')
	f.write('''
{: .no_toc }

## Trajectories

<img src="{{ site.baseurl }}/assets/results/corridor_trajectories.png" style="width:100%;max-height:none"/>


## Planning Statistics

<img src="{{ site.baseurl }}/assets/results/corridor_stats.png" style="width:100%;max-height:none"/>

''')
