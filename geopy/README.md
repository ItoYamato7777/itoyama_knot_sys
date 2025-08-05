# geo.py

## How to use
```
from geo import VECTOR, MATRIX, FRAME
```

## VECTOR

## MATRIX

## FRAME
```
t_A2B = FRAME(xyzabc = [x1, y1, z1, a, b, c])
t_C2B = FRAME(xyzrpy = [x2, y2, z2, roll, pitch, yaw])
t_B2C = -t_C2B
t_A2C = t_A2B * t_B2C
```

### Check Values
```
t_A2B_xyzrpy = t_A2B.xyzrpy()
t_C2B_xyzabc = t_C2B.xyzabc()
```

### Convert from/to numpy
```
import numpy as np
## np.array -> FRAME
np_matrix = np.array([[1, 0, 0, 1],
                      [0, 1, 0, 2],
                      [0, 0, 1, 3],
                      [0, 0, 0, 1]])

frm = FRAME(frm = np_matrix)
## FRAME -> np.array
np.matrix2 = frm.toarray()
```

### Get MATRIX and VECTOR
```
mat = frm.mat
vec = frm.vec
```
