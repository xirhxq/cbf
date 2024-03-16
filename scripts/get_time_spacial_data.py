import json, os, re, math, numpy as np, time


ptn = re.compile('.*.json')
src = '../data'
files = os.listdir(src)

json_files = []
for file in files:
    if re.match(ptn, file):
        json_files.append(f'{src}/{file}')

json_files.sort(reverse=True)
newest_json = json_files[0]
print('find {}'.format(newest_json))

filename = newest_json
filename = filename[:-9]

with open(filename + 'data.json') as f:
    data_dict = json.load(f)


total_length = len(data_dict["state"])
robot_num = data_dict["para"]["number"]

position_offset = {'suav_1': [0.2, -1.5, 0],
                   'suav_2': [0, 0, 0],
                   'suav_3': [0.2, 1.5, 0]}

time_spacial_data = {}
for i in range(robot_num):
    uav_name = 'suav_' + str(i + 1)
    time_spacial_data[uav_name] = [
        [data_dict["state"][l]["robot"][i]["x"] - position_offset[uav_name][0],
         data_dict["state"][l]["robot"][i]["y"] - position_offset[uav_name][1],
         1.0 + 0.5 * i,
         data_dict["state"][l]["runtime"]]
        for l in range(total_length)]

skip_num = 50
for n in list(time_spacial_data.keys()):
    with open('tra_' + n + '.txt', 'w') as f:
        for i in range(len(time_spacial_data[n]) // skip_num):
            f.write(" ".join('{:f}'.format(x) for x in time_spacial_data[n][i * skip_num]) + os.linesep)
