import os
import subprocess
import sys

program = sys.argv[1]
instances_dir = sys.argv[2]
param = sys.argv[3].split("=")[1].split(",")

# TODO use argparse
print(program, instances_dir, param)

# TODO make before run
instances = os.listdir(instances_dir)

print(instances)

qtd = 0
total = len(instances) * len(param)
for i in sorted(instances):
    for p in param:
        # TODO tranformar em barra de progresso
        print(f"executando instância {i} \t({qtd} de {total})")
        qtd += 1
        with open(f"out/{i}-{p}.out", "w") as fd:
            # print([program, i, p])
            process = subprocess.Popen(
                [program, f"./{instances_dir}/{i}", p], stdout=subprocess.PIPE
            )
            output, error = process.communicate()
            fd.write(output.decode())
