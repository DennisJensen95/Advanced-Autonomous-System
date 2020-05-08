import re
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

def get_object_from_point_o(obj_id, point, orientation):
    R = np.array([[np.cos(orientation), -np.sin(orientation)],
                  [np.sin(orientation), np.cos(orientation)]])

    point = np.array(point)

    if obj_id == 1:
        L = 0.40
        b = 0.15

        point1 = np.array([point[0] + 1 / 2 * L, point[1] + 1 / 2 * b]).transpose()
        point2 = np.array([point[0] - 1 / 2 * L, point[1] + 1 / 2 * b]).transpose()
        point3 = np.array([point[0] - 1 / 2 * L, point[1] - 1 / 2 * b]).transpose()
        point4 = np.array([point[0] + 1 / 2 * L, point[1] - 1 / 2 * b]).transpose()

        points = np.array([point1, point2, point3, point4])

        str_to_write = f'{point1[0]}\t{point1[1]}\t{point2[0]}\t{point2[1]}\n' \
                       f'{point2[0]}\t{point2[1]}\t{point3[0]}\t{point3[1]}\n' \
                       f'{point3[0]}\t{point3[1]}\t{point4[0]}\t{point4[1]}\n'  \
                       f'{point4[0]}\t{point4[1]}\t{point1[0]}\t{point1[1]}\n'
    elif obj_id == 2:
        L = 0.30
        b = 0.20

        point1 = np.array([point[0] + 1 / 2 * L, point[1] + 1 / 2 * b]).transpose()
        point2 = np.array([point[0] - 1 / 2 * L, point[1] + 1 / 2 * b]).transpose()
        point3 = np.array([point[0] - 1 / 2 * L, point[1] - 1 / 2 * b]).transpose()
        point4 = np.array([point[0] + 1 / 2 * L, point[1], - 1 / 2 * b]).transpose()

        points = np.array([point1, point2, point3, point4])

        str_to_write = f'{point1[0]}\t{point1[1]}\t{point2[0]}\t{point2[1]}\n' \
                       f'{point2[0]}\t{point2[1]}\t{point3[0]}\t{point3[1]}\n' \
                       f'{point3[0]}\t{point3[1]}\t{point4[0]}\t{point4[1]}\n' \
                       f'{point4[0]}\t{point4[1]}\t{point1[0]}\t{point1[1]}\n'
    elif obj_id == 3:
        L = 0.40
        b = 0.10
    
        point1 = np.array([point[0]+L, point[1]]).transpose()
        point2 = np.array([point[0], point[1]+b]).transpose()
    
        points = np.array([point, point1, point2])

        str_to_write = f'{point1[0]}\t{point1[1]}\t{point2[0]}\t{point2[1]}\n' \
                       f'{point2[0]}\t{point2[1]}\t{point[0]}\t{point[1]}\n' \
                       f'{point[0]}\t{point[1]}\t{point1[0]}\t{point1[1]}\n'
    elif obj_id == 4:
        L = 0.30
        b = 0.15
    
        point1 = np.array([point[0]+L, point[1]]).transpose()
        point2 = np.array([point[0], point[1]+b]).transpose()
    
        points = np.array([point, point1, point2])



    s = points - point
    print(s)
    print(R)
    so = np.dot(s, R)
    vo = so + point

    if obj_id > 2:
        point, point1, point2 = vo[0], vo[1], vo[2]
        str_to_write = f'{point1[0]}\t{point1[1]}\t{point2[0]}\t{point2[1]}\n' \
                       f'{point2[0]}\t{point2[1]}\t{point[0]}\t{point[1]}\n' \
                       f'{point[0]}\t{point[1]}\t{point1[0]}\t{point1[1]}\n'
    else:
        point1, point2, point3, point4 = vo[0], vo[1], vo[2], vo[3]
        str_to_write = f'{point1[0]}\t{point1[1]}\t{point2[0]}\t{point2[1]}\n' \
                       f'{point2[0]}\t{point2[1]}\t{point3[0]}\t{point3[1]}\n' \
                       f'{point3[0]}\t{point3[1]}\t{point4[0]}\t{point4[1]}\n' \
                       f'{point4[0]}\t{point4[1]}\t{point1[0]}\t{point1[1]}\n'

    return str_to_write

map = f'0.0     0.0     1.8     0.0     bottom left\n' \
      f'2.2     0.0     4.0     0.0     bottom right\n' \
      f'0.0     5.0     1.8     5.0     top left\n' \
      f'2.2     5.0     4.0     5.0     top right\n' \
      f'0.0     0.0     0.0     1.8     left down\n' \
      f'0.0     3.2     0.0     5.0     left up\n' \
      f'4.0     0.0     4.0     1.8     right down\n' \
      f'4.0     3.2     4.0     5.0     right up\n' \
      f'0.9     3.1     1.7     3.1     maze bottom left\n' \
      f'2.3     3.1     3.1     3.1     maze bottom right\n' \
      f'1.7     2.5     1.7     3.1     maze left down\n' \
      f'0.9     3.1     0.9     4.3     maze left up\n' \
      f'2.3     2.5     2.3     3.1     maze right down\n' \
      f'3.1     3.1     3.1     4.3     maze right up\n' \
      f'1.5     3.7     2.5     3.7     maze middle horizontal\n' \
      f'2.0     3.7     2.0     4.3     maze middle vertical\n' \
      f'0.9     4.3     3.1     4.3     maze top\n'

os.chdir('./../../../test/')

map_environ = map
if os.path.exists('388auto'):
    os.remove('388auto')

with open('./388auto', 'w+') as file:
    file.write(map_environ)

files_py = sorted(glob.glob('results_2/res*pyt*.txt'))

for j in range(len(files_py)):
    with open(files_py[j], 'r') as file:
        ground_truths = file.readlines()
        ground_truths.pop(0)

    for i, ground_truth in enumerate(ground_truths):
        truth = np.around(np.array(re.findall('-?\d+\.?\d*', ground_truth)).astype(float), 2)

        object_truth, point_o_x_truth, point_o_y_truth, orientation_truth = truth[0], truth[1], truth[2], truth[3]

        obj = get_object_from_point_o(object_truth, [point_o_x_truth, point_o_y_truth], orientation_truth)

        with open('./388auto', 'a+') as file:
            file.write(obj)

