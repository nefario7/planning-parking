from bson import re
import matplotlib.pyplot as plt
import numpy as np
from sacred import Experiment
from sacred.observers import MongoObserver
from skimage import io
from tqdm import tqdm
from fuzzy_inference import FuzzyInference

ex = Experiment("traversability")
ex.observers.append(MongoObserver(url="localhost:27017", db_name="mmseg"))


@ex.config
def config():
    DEM_filename = "data/outputs/dtm.tif"
    vis = True
    resolution = 0.3


def vis_DEM(image):
    names = ("min", "max", "mean", "inverse distance weighting", "count", "stdev")
    for i in range(image.shape[0]):
        disp_image = image[i]
        disp_image[disp_image == -9999.0] = np.nan
        plt.imshow(disp_image)
        plt.title(names[i])
        plt.colorbar()
        plt.show()


def run_fuzzy_logic(roughness, slope):
    # Set antecedents values, perform Sugeno inference and print output values.
    fi = FuzzyInference()
    return fi.infer(slope, roughness)


@ex.automain
def compute_traversability(DEM_filename, resolution, vis):
    image = io.imread(DEM_filename)
    # if vis:
    #    vis_DEM(image)

    roughness = image[5]
    height = image[3]
    roughness[roughness == -9999.0] = np.nan
    height[height == -9999.0] = np.nan

    slope_xy = np.gradient(height)
    slope_mag = np.linalg.norm(slope_xy, axis=0) / resolution
    slope_rad = np.arctan2(slope_mag, resolution)

    traversability = run_fuzzy_logic(roughness, slope_rad)
    if vis:
        fig, axs = plt.subplots(1, 3)
        cb1 = axs[0].imshow(slope_rad)
        cb2 = axs[1].imshow(roughness)
        cb3 = axs[2].imshow(traversability, vmin=0, vmax=1)

        axs[0].set_title("Slope")
        axs[1].set_title("Roughness")
        axs[2].set_title("Traversability index")

        fig.colorbar(cb1, ax=axs[0])
        fig.colorbar(cb2, ax=axs[1])
        fig.colorbar(cb3, ax=axs[2])

        plt.show()
    return traversability
