import imageio
from sacred import Experiment
from sacred.observers import MongoObserver
# from python_traversability.load_segment_and_filter 
import load_and_segment
from python_traversability.grid_cloud import grid_cloud
from python_traversability.vis import show_labeled_cloud
from python_traversability.compute_traversability import compute_traversability
import matplotlib.pyplot as plt
from python_traversability.config import (
    DEM_FILE,
    HAG_THRESHOLD,
    INPUT_FILE,
    RESOLUTION,
    FILTER_TYPE,
    VIS,
    OUTPUT_FILE,
)

ex = Experiment("traversability")
ex.observers.append(MongoObserver(url="localhost:27017", db_name="mmseg"))


@ex.config
def config():
    input_file = INPUT_FILE  # The file of xyz points
    DEM_file = DEM_FILE  # Where to save the intermediate DEM file
    resolution = RESOLUTION  # DEM/grid resolution
    HAG_threshold = HAG_THRESHOLD  # height above ground threshold
    filter_type = FILTER_TYPE  # The filter for computing the ground
    vis = VIS  # whether to visualize
    output_file = OUTPUT_FILE


@ex.automain
def main(
    input_file, DEM_file, resolution, HAG_threshold, filter_type, output_file, vis
):
    # These results are currently unused, instead we read from the DEM which is written in this step
    xyz, is_ground = load_segment_and_filter(
        input_file,
        dem_resolution=resolution,
        dem_file=DEM_file,
        HAG_limit=HAG_threshold,
        filter_type=filter_type,
        vis=vis,
    )
    traversability = compute_traversability(DEM_file, resolution=resolution, vis=vis)
    imageio.imwrite(output_file, traversability)
