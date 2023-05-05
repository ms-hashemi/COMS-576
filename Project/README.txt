Install the dubins package by doing

$ pip install dubins

Put the following files in the same folder or in your PYTHONPATH
* project.py
* geometry.py
* edge.py
* graph.py
* obstacle.py
* planning.py
* queue.py
* draw_cspace.py (This can be obtained from the course github.)

For RRT, run

$ python project.py --alg rrt

For k-RRT*, run

$ python project.py --alg rrt_optimal

For PRM, run

$ python project.py --alg prm

For k-PRM*, run

$ python project.py --alg prm_optimal

It will take a few seconds before showing the plot of the configuration space, the roadmap, and the path.
