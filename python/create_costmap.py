import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    file_path = "..\costmap.csv"

    df = pd.read_csv(file_path, header=None)

    costmap = df.to_numpy()

    print(costmap.shape)
    # rgba = [costmap.shape[0], costmap.shape[1], 3]
    # rgba_mask = np.where(costmap == 0, 0, 1)
    # rgba[rgba_mask] = 

    plt.imshow(costmap)
    plt.show()

