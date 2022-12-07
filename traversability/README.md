## Traversability Assessment for Pointclouds in Python
This approach provides a baseline method for computing whether a UGV can drive through the world given a pointcloud representation. This is fairly fast because significant parts of the codebase rely on `PDAL`.

## Install
Begin by installing the system dependencies, namely `PDAL`, using conda.   
```
conda create -n traversability pdal  poetry python=3.8 -c conda-forge
conda activate traversability
```
Now install the rest of the python dependencies 
```poetry install```

Now you can run the example with
```
python python_traversability/traversability.py
```

## Use
The expermental settings and logging are handled by [sacred](https://sacred.readthedocs.io/en/stable/) which exposes a CLI. You can set the parameters in the config block as described [here](https://sacred.readthedocs.io/en/stable/command_line.html). An example of changing the input filename and the DEM resolution is the following.
```
python python_traversability/traversability.py with input_file="my/new/file/name.txt" resolution=0.1
```


## Attribution
This work was initially developed in MATLAB by Chinmay Garg [@nefario7](https://github.com/nefario7).

If you use this work in an academic context, please include the following citation
```
@inproceedings{
russell2022uav,
title={{UAV} Mapping with Semantic and Traversability Metrics for Forest Fire Mitigation},
author={David Jacob Russell and Tito Arevalo-Ramirez and Chinmay Garg and Winnie Kuang and Francisco Yandun and David Wettergreen and George Kantor},
booktitle={ICRA 2022 Workshop in Innovation in Forestry Robotics: Research and Industry Adoption},
year={2022},
url={https://openreview.net/forum?id=Bbx8xClhG9}
}
```
