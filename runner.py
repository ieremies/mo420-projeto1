import os
import subprocess

cmd = "./bin/pli-projeto1.e ./bin/../instances/{instance} {type}"
directory = 'instances'

instances = os.listdir(directory)

qtd = 0
total = len(instances)
for i in sorted(instances, key=lambda x: int(x.split("_")[1])):
    type = " l"
    print(f"executando instância {i} \t({qtd} de {total})")
    qtd += 1
    with open("out/"+i+type, "w") as fd:
        process = subprocess.Popen(cmd.format(instance=i, type=type).split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        fd.write(output.decode())
