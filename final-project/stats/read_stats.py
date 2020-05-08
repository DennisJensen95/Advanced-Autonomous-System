import re
import numpy as np
import glob
from math import isclose
import matplotlib.pyplot as plt

def angleDifference(first_angle, second_angle):
    diff = second_angle - first_angle
    while (diff < -np.pi/2):
        diff += np.pi

    while (diff > np.pi/2):
        diff -= np.pi
    return diff

files_py = sorted(glob.glob('results_2/res*pyt*.txt'))
files_cpp = sorted(glob.glob('results_2/res*cpp*.txt'))
print(files_py)
print(files_cpp)
experiments = 0
# Get number of experiments
for file in files_py:
    with open(file, 'r') as file:
        lines = file.readlines()
        experiments += len(lines)

results = np.zeros((experiments-1, 4))
obj_results = np.zeros((2, 4))
res_idx = 0
threshold = 0.03
for j in range(len(files_py)):
    with open(files_py[j], 'r') as file:
        ground_truths = file.readlines()
        ground_truths.pop(0)

    with open(files_cpp[j], 'r') as file:
        predictions = file.readlines()
        predictions.pop(0)

    for i, ground_truth in enumerate(ground_truths):
        preds = np.around(np.array(re.findall('-?\d+\.?\d*', predictions[i])).astype(float), 2)
        truth = np.around(np.array(re.findall('-?\d+\.?\d*', ground_truth)).astype(float), 2)
        print(preds)
        print(truth)
        object_truth, point_o_x_truth, point_o_y_truth, orientation_truth = truth[0], truth[1], truth[2], truth[3]
        object_pred, point_o_x, point_o_y, orientation, SSD = preds[0], preds[1], preds[2], preds[3], preds[4]

        if isclose(point_o_x, point_o_x_truth, rel_tol=threshold):
            results[res_idx, 1] = 1

        if isclose(point_o_y, point_o_y_truth, rel_tol=threshold):
            results[res_idx, 2] = 1
        if isclose(orientation, orientation_truth, rel_tol=threshold):
            results[res_idx, 3] = 1

        if abs(angleDifference(orientation, orientation_truth)) < threshold * np.pi:
            results[res_idx, 3] = 1
        else:
            print(f'{abs(angleDifference(orientation, orientation_truth))} < {threshold * np.pi}')
            print(f'{orientation_truth} ?= {orientation}')
            print(f'Line number {i} in files: {files_py[j]} and {files_cpp[j]}')

        obj_results[1, int(object_truth)-1] += 1
        if object_pred == object_truth:
            obj_results[0, int(object_truth)-1] += 1
            results[res_idx, 0] = 1

        res_idx += 1

print(obj_results)
print(np.sum(obj_results[1,:]))

sum_res = np.sum(results, axis=0)
object_correct, point_o_x_correct, point_o_y_correct, orientation_correct = sum_res[0], sum_res[1], \
                                                                            sum_res[2], sum_res[3]
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

# Each object accuracy
plt.rc('font', **font)
fig, ax = plt.subplots()
# plt.title('Total Results')
plt.rc('font', **font)
labels = ['Object 1', 'Object 2', 'Object 3', 'Object 4']
bar_results = obj_results[0, :]/obj_results[1, :] * 100
bar1 = ax.bar(labels[0], bar_results[0])
bar2 = ax.bar(labels[1], bar_results[1])
bar3 = ax.bar(labels[2], bar_results[2])
bar4 = ax.bar(labels[3], bar_results[3])
bars = [bar1, bar2, bar3, bar4]
plt.ylabel('Accuracy prediction [%]', family=font['family'], weight=font['weight'], size=font['size'])
# plt.xlabel('Objects to predict', family=font['family'], weight=font['weight'], size=font['size'])
# plt.legend(labels)
plt.xlim([-1, 4])

for container in bars:
    for bar in container:
        height = round(bar.get_height(), 2)
        ax.annotate(f'{height} %',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    ha='center')

# Each prediction
fig, ax = plt.subplots()
plt.rc('font', **font)
# plt.title('Total Results')
labels = ['Object', 'Point o x', 'Point o y', 'Orientation']
bar_results = sum_res/experiments*100
bar1 = ax.bar(labels[0], bar_results[0])
bar2 = ax.bar(labels[1], bar_results[1])
bar3 = ax.bar(labels[2], bar_results[2])
bar4 = ax.bar(labels[3], bar_results[3])
bars = [bar1, bar2, bar3, bar4]
plt.ylabel('Accuracy prediction [%]', family=font['family'], weight=font['weight'], size=font['size'])
# plt.xlabel('Measure to predict', family=font['family'], weight=font['weight'], size=font['size'])
# plt.legend(labels)
plt.xlim([-1, 4])

for container in bars:
    for bar in container:
        height = round(bar.get_height(), 2)
        ax.annotate(f'{height} %',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    ha='center')



plt.show()
