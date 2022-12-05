import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.cbook as cbook
import matplotlib.patches as patches
import matplotlib as mpl
import sys

import pandas as pd


# FILE_NAME = "b_level"
FILE_NAME = "mit_base"

def create_csv():

    file_name = f"../maps/{FILE_NAME}_map.csv"
    df = pd.read_csv(file_name)

    df_array = df.to_numpy()
    
    plt.imshow(df_array)
    plt.savefig(f"../maps/{FILE_NAME}_map.png")
    # plt.show()

    # print(base_map.shape)


if __name__ == '__main__':
    create_csv()