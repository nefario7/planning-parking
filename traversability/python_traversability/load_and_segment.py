import pdal
import numpy as np
import pyvista as pv

# from ubelt.timerit import Timer, Timerit


def load_segment_and_filter(
    input_file,
    filter_type="smrf",
    HAG_limit=3,
    dem_resolution=0.3,
    dem_file="data/outputs/dtm.tif",
    vis=False,
    screenshot_name=None,
):
    """ Loads a pointcloud and segments the groundplane
    Arguments:
        input_file : str
            The path to the file, local or absolute
        filter_type: str
            ("smrf", "csf") 
        HAG_limit: float
            Height above ground limit for retaining points
        dem_resolution: float
            The size of grid squares in the digital elevation model
        vis: bool
        screenshot_name: str
            where to save a screenshot of vis false

    Returns:
        xyz: 
        is_ground:
    """

    # Create the format string
    json = f"""
    [
        "{input_file}",
        {{
            "type": "filters.{filter_type}"
        }},
        {{
            "type": "filters.hag_delaunay"
        }},
        {{
            "type":"filters.range",
            "limits":"HeightAboveGround[0:{HAG_limit}]"
        }},
        {{
            "filename":"{dem_file}",
            "gdaldriver":"GTiff",
            "output_type":"all",
            "resolution":"{dem_resolution}"
        }}
    ]
    """
    # Create the PDAL pipeline
    pipeline = pdal.Pipeline(json)
    # Run the pipeline
    pipeline.execute()
    # Extract the information
    output = pipeline.arrays[0]
    # Format the quantaties of interest
    xyz = np.vstack([output[x] for x in "XYZ"]).T
    is_ground = (output["Classification"] - 1).astype(bool)

    if vis:
        plotter = pv.Plotter()
    elif screenshot_name is not None:
        plotter = pv.Plotter(off_screen=True)

    if vis or screenshot_name is not None:
        plotter.add_mesh(pv.PolyData(xyz), scalars=is_ground)
        plotter.show(screenshot=screenshot_name)
    return xyz, is_ground
