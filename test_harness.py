#!/usr/bin/python3

from subprocess import Popen, PIPE

TYPENAME = ['Serial', 'CUDA', 'TBB']
WIDTH = [256, 512, 1024, 2048, 4096, 8192]
HEIGHT = WIDTH
EXECNAME = './parametric_rendering_'
F = open('output', 'w')

for t in TYPENAME:
    execname = EXECNAME + t
    for w in WIDTH:
        print([execname, str(w)])
        p = Popen([execname, str(w)], stdout=PIPE, stderr=PIPE)
        p.wait()
        out, err = p.communicate()
        print(out.decode("utf-8"))
        F.writelines(out.decode("utf-8"))
        print(['mv', 'reg3D.pnm', 'reg3D_' + t + '_' + str(w) + '.pnm'])
        p = Popen(['mv', 'reg3D.pnm', 'reg3D_' + t + '_' + str(w) + '.pnm'], stdout=PIPE, stderr=PIPE)

