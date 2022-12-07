from more_itertools import run_length
import simpful as sf
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator


class FuzzyInference:
    def __init__(self, slope_UoD=(0, np.pi / 2), roughness_UoD=(0, 5), grid_steps=20):

        self.slope_UoD = slope_UoD
        self.roughness_UoD = roughness_UoD

        self.grid_steps = 20

        self.interpolator = None

    def create_FS(
        self,
        smooth_params=(0, 1, 1, 0.5),  # (0.00842, -0.0345, 0.1261, 0.1652),
        rough_params=(2, 3, 0.5, 0.5),  # (0.138, 0.483, 0.1359, 0.5178),
        rocky_params=(4, 6, 0.5, 0.5),  # (0.1538, 0.895, 0.1, 0.995),
        flat_params=(0, 0.3, 0.1, 0.1),  # (0.1571, 0, 0.07854, 0.07854),
        sloped_params=(0.45, 0.75, 0.1, 0.15),  # (0.0785, 0.2948, 0.173, 0.525),
        steep_params=(0.9, 2, 0.15, 0.15),  # (0.123, 0.8405, 0.157, 1.57),
        slope_names=("flat", "sloped", "steep"),
        roughness_names=("smooth", "rough", "rocky"),
        vis=False,
    ):
        FS = sf.FuzzySystem()

        # Define a linguistic variable.

        roughness_params = (smooth_params, rough_params, rocky_params)
        sloped_params = (flat_params, sloped_params, steep_params)

        roughness_lv = [
            sf.fuzzy_sets.DoubleGaussianFuzzySet(p[0], p[2], p[1], p[3], name)
            for i, (p, name) in enumerate(zip(roughness_params, roughness_names))
        ]
        roughness_lv = sf.LinguisticVariable(
            roughness_lv, universe_of_discourse=self.roughness_UoD
        )

        slope_lv = [
            sf.fuzzy_sets.DoubleGaussianFuzzySet(p[0], p[2], p[1], p[3], name)
            for i, (p, name) in enumerate(zip(sloped_params, slope_names))
        ]
        slope_lv = sf.LinguisticVariable(slope_lv, universe_of_discourse=self.slope_UoD)

        if vis:
            roughness_lv.plot()
            slope_lv.plot()

        FS.add_linguistic_variable("roughness", roughness_lv)
        FS.add_linguistic_variable("slope", slope_lv)

        # Define consequents.
        # This isn't quite the same as Chinmay's, which has a fuzzy output
        FS.set_crisp_output_value("low", 0)
        FS.set_crisp_output_value("medium", 0.5)
        FS.set_crisp_output_value("high", 1)

        # Define fuzzy rules.
        RULE1 = "IF ((slope IS flat) AND (roughness IS smooth)) THEN (traversability IS high)"

        RULE2 = "IF ((slope IS flat) AND (roughness IS rough)) THEN (traversability IS medium)"
        RULE3 = "IF ((slope IS sloped) AND (roughness IS smooth)) THEN (traversability IS medium)"
        RULE4 = "IF ((slope IS sloped) AND (roughness IS rough)) THEN (traversability IS medium)"

        RULE5 = (
            "IF ((slope IS steep) OR (roughness IS rocky)) THEN (traversability IS low)"
        )
        FS.add_rules([RULE1, RULE2, RULE3, RULE4, RULE5])
        return FS

    def cache_fuzzy_inference(self):
        self.FS = self.create_FS()
        slope_samples = np.linspace(
            self.slope_UoD[0], self.slope_UoD[1], self.grid_steps
        )
        roughness_samples = np.linspace(
            self.roughness_UoD[0], self.roughness_UoD[1], self.grid_steps
        )

        slope_samples_grid, roughness_samples_grid = np.meshgrid(
            slope_samples, roughness_samples
        )
        initial_shape = slope_samples_grid.shape

        infered_values = []

        for s, r in zip(slope_samples_grid.flatten(), roughness_samples_grid.flatten()):
            self.FS.set_variable("slope", s)
            self.FS.set_variable("roughness", r)
            infered_values.append(
                self.FS.Sugeno_inference(["traversability"])["traversability"]
            )
        infered_values = np.reshape(infered_values, initial_shape)

        self.interpolator = RegularGridInterpolator(
            (slope_samples, roughness_samples), infered_values
        )

    def infer(self, slope, roughness):
        if self.interpolator is None:
            self.cache_fuzzy_inference()
        initial_shape = slope.shape

        inputs = np.vstack((slope.flatten(), roughness.flatten())).T
        valid = np.logical_and(np.isfinite(inputs[:, 0]), np.isfinite(inputs[:, 1]))

        valid_outputs = self.interpolator(inputs[valid])

        all_outputs = np.full(inputs.shape[0], np.nan)
        all_outputs[valid] = valid_outputs

        all_outputs = np.reshape(all_outputs, initial_shape)

        return all_outputs
