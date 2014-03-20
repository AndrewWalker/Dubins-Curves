import os

env = Environment(ENV = os.environ)
env.Append(CPPPATH = ['include'])
env.Append(CCFLAGS = '-g -Wall -O0'.split())
env.Append(LIBS    = ['m'])
env.Append(RPATH   = '.')
env.SharedLibrary('dubins', ['src/dubins.c'])

appenv = env.Clone()
appenv.Append(LIBS    = ['dubins'])
appenv.Append(LIBPATH = ['.'])
app = appenv.Program('test_dubins', ['ctests/main.c'])

Depends(app, 'libdubins.so')
