import numpy as np
def depth_search(x, y, depth_image):
    depth = 0
    max_i = 5
    for i in range(max_i):
        new_x_max = min(x+i, depth_image.shape[0])
        new_x_min = max(x-i, 0)
        new_y_max = min(y+i, depth_image.shape[1])
        new_y_min = max(y-i, 0)
        print(new_x_min,new_x_max, new_y_min,new_y_max)
        arr = depth_image[new_x_min:new_x_max, new_y_min:new_y_max]
        arr[arr == 0] = np.nan
        if arr.any():
            if not np.isnan(arr).all():
                depth = np.nanmean(arr)

        if depth != 0:
            break
    
    return depth

arrp = np.zeros((5,5))
arrp[2, 3:4] =8
print(depth_search(0, 0, arrp))