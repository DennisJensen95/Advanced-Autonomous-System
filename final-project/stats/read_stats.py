import re
import numpy as np
cpp_res_file = open('results_cpp_donefullwait22.txt', 'r+')
python_res_file = open('results_python_donefullwait22.txt', 'r+')

results_cpp = cpp_res_file.readlines()
results_python = python_res_file.readlines()

print(len(results_cpp)-1)
print(len(results_python)-1)

matches = 0
for i, _object in enumerate(results_python):
    _object = _object.replace(',', '').replace('[', '').replace(']', '')
    numbers = re.findall('\d+\.?\d*', _object)
    _object = ' '.join(np.around(np.asarray(numbers).astype(float), decimals=2).astype(str))
    print(_object)
    if any(_object in s for s in results_cpp):
        print(_object)
        matches += 1

    if i > len(results_cpp):
        break

print(matches)
