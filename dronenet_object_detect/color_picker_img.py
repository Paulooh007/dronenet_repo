import numpy as np
import cv2
import os


# Silver
# Min:  102 46 103
# Max:  134 122 128

# Blue
# Min:  134 105 87
# Max:  168 136 105

# White
# Min:  131 128 128
# Max:  158 155 158

def distance(pointA, pointB):
    """Get Euclidean distance between 2 points"""
    return (
        ((pointA[0] - pointB[0]) ** 2) +
        ((pointA[1] - pointB[1]) ** 2) 
        # ((pointA[2] - pointB[2]) ** 2)
        ) ** 0.5  


def findCarOfInterest(img):
    lower_white = np.array([131, 128, 128])
    high_white = np.array([158, 155, 158])

    low_silver = np.array([102, 46, 103])
    high_silver = np.array([134, 122, 128])

    low_blue = np.array([134, 105, 87])
    high_blue = np.array([168, 136, 105])


    data = np.reshape(img, (-1,3))
    data = np.float32(data)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_RANDOM_CENTERS
    compactness,labels,centers = cv2.kmeans(data,1,None,criteria,10,flags)
    arr = centers[0].astype(np.int32)

    if arr[0] in range(lower_white[0], high_white[0]) and arr[1] in range(lower_white[1], high_white[1]) and arr[2] in range(lower_white[2], high_white[2]):
        return "white"
    elif arr[0] in range(low_blue[0], high_blue[0]) and arr[1] in range(low_blue[1], high_blue[1]) and arr[2] in range(low_blue[2], high_blue[2]):
        return "blue"
    elif arr[0] in range(low_silver[0], high_silver[0]) and arr[1] in range(low_silver[1], high_silver[1]) and arr[2] in range(low_silver[2], high_silver[2]):
        return "silver"
    else:
        return 'none'


color = "blue"

path_list = os.listdir(f"test_img/{color}/")
path_list = os.listdir(f"img_unknown/")

r_min, g_min, b_min = 300, 300, 300
r_max, g_max, b_max = 0, 0, 0

# for path in path_list:
    
#     img = cv2.imread(f'img_unknown/'+path)
#     # print(path)
    
#     arr = findCarOfInterest(img)
#     # r_min, g_min, b_min = min(r_min, arr[0]), min(g_min, arr[1]), min(b_min, arr[2])
#     # r_max, g_max, b_max = max(r_max, arr[0]), max(g_max, arr[1]), max(b_max, arr[2])
#     print(arr)

for path in path_list:
    
    img = cv2.imread(f'img_unknown/'+path)
    print(path)
    
    arr = findCarOfInterest(img)
    # r_min, g_min, b_min = min(r_min, arr[0]), min(g_min, arr[1]), min(b_min, arr[2])
    # r_max, g_max, b_max = max(r_max, arr[0]), max(g_max, arr[1]), max(b_max, arr[2])
    print(arr)


print('Min: ', r_min, g_min, b_min)
print('Max: ', r_max, g_max, b_max)
# cv2.imshow("Car", img)
# cv2.waitKey(0)

# Silver
# Min:  102 46 103
# Max:  134 122 128

# Blue
# Min:  134 105 87
# Max:  168 136 105

# White
# Min:  131 128 128
# Max:  158 155 158
