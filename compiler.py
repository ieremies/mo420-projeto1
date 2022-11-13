
import os
import subprocess

directory = 'out'

results = os.listdir(directory)

# print("inst", "nnodes", "simpl_it", "time", "work","obj","bound","gap", sep='\t')

for i in sorted(results, key=lambda x: int(x.split("_")[1])):
    explored = ''
    best = ''
    type = i.split()[1]

    if type != "e":
        continue

    with open("out/"+i, "r") as fd:
        print(" | ", end='')
        print(i.split()[0][5:], end=' | ')
        for line in fd:
            if "Explored" in line:
                explored = line
            if "Best objective" in line:
                best = line
        explored = explored.split()
        nnodes = explored[1]
        simplex_it = explored[3][1:]
        time = explored[7]
        work = explored[-3][1:]

        best = best.split()
        obj = best[2][:-1]
        obj = float(obj) if obj != '-' else obj
        bound = best[-3][:-1]
        bound = float(bound) if bound != '-' else bound
        gap = best[-1]

        print(nnodes, time, obj, bound, gap, sep=" | ")
        print(" | ", end='')
