# Visibility-based Probabilistic Roadmaps
<p align="center">
  <img src="https://github.com/AngeloEspinoza/visibility-based-probabilistic-roadmaps/assets/40195016/5df506f5-7856-4cd2-8fb2-5872b8665178" alt="drawing" width="450"/>
</p>

## Description
A 2D simulation in the framework Pygame of the paper [Visibility-based Probabilistic Roadmaps for Motion Planning](https://www.cimat.mx/~fory/robots/advrob00.pdf).
The environment has 2 non-convex obstacles. Key asspects of the algorithm such as the the number of failures before the insertion of a new guard node can be
modified, as well as the total volume of $\mathcal{C}_{free}$ covered by the visibility domains of the nodes.

## Usage 
```
usage: visibility_prm.py [-h] [-o | --obstacles | --no-obstacles] [-init  [...]] [-goal  [...]] [-srn | --show_random_nodes | --no-show_random_nodes] [-srjn | --show_rejected_nodes | --no-show_rejected_nodes]
                         [-sen | --show_enumerated_nodes | --no-show_enumerated_nodes] [-sve | --show_volume_estimation | --no-show_volume_estimation] [-M] [-kr | --keep_roadmap | --no-keep_roadmap] [-r]

Implements the Visibility PRM algorithm for path planning.

options:
  -h, --help            show this help message and exit
  -o, --obstacles, --no-obstacles
                        Obstacles on the map
  -init  [ ...], --x_init  [ ...]
                        Initial node position in X and Y respectively
  -goal  [ ...], --x_goal  [ ...]
                        Goal node position in X and Y respectively
  -srn, --show_random_nodes, --no-show_random_nodes
                        Show random nodes on screen
  -srjn, --show_rejected_nodes, --no-show_rejected_nodes
                        Show rejected nodes on screen
  -sen, --show_enumerated_nodes, --no-show_enumerated_nodes
                        Show the node enumerated in the order it was sampled
  -sve, --show_volume_estimation, --no-show_volume_estimation
                        Show the volume covered by visibility domains, and the volume not covered yet
  -M , --M              Maximum number of failures before allowed before inserting a new guard node into the roadmap
  -kr, --keep_roadmap, --no-keep_roadmap
                        Keeps the tree while the robot is moving towards the goal
  -r , --radius         Set the robot radius
```

## Examples
Generate obstacles in the map, and enumerate the nodes as they are sampled; all with a maximum number of attempts of $M = 30$

```python3 visibility_prm.py --obstacles --show_enumerated_nodes --M 30```

Generate obstacles, initial configuration $\mathbf{x}_{init} = (300, 300)$, show the rejected nodes, and enumerate nodes; all with a maximum number of attempts of $M = 15$

```python3 visibility_prm.py --obstacles --x_init 300 300 --show_rejected_nodes --show_enumerated_nodes --M 15```

 ## License 
 MIT License

Copyright (c) [2024] [Angelo Espinoza]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
