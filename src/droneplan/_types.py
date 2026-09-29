import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
IntArray = npt.NDArray[np.int64]
BoolArray = npt.NDArray[np.bool_]


def as_xy(a: npt.ArrayLike) -> FloatArray:
    return np.asarray(a, dtype=np.float64).reshape(-1, 2)
