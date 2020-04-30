import re
import numpy as np


cpp_res_file = open('results_cpp_donefullwait22.txt', 'r+')
python_res_file = open('results_python_donefullwait22.txt', 'r+')

results_cpp = cpp_res_file.readlines()[1:]
results_python = python_res_file.readlines()[1:]
print(len(results_cpp))
print(len(results_python))

matches = 0
for i, _object in enumerate(results_cpp):
    _object = _object.replace(',', '').replace('[', '').replace(']', '')
    numbers = re.findall('\d+\.?\d*', _object)
    _object = np.around(np.asarray(numbers).astype(float), decimals=2)
    for guess_object in results_python[i:10]:
        numbers = re.findall('\d+\.?\d*', guess_object)
        guess_object = np.around(np.asarray(numbers).astype(float), decimals=2)
        # direction

        if guess_object[0] == _object[0] and guess_object[1] == _object[1] and \
                guess_object[2] == _object[2]:
            print(guess_object)
            print(_object)
            matches += 1
            break
    if i > len(results_cpp):
        break

print(matches)
