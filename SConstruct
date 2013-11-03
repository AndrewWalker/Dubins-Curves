import os

env = Environment(
	ENV = os.environ,
    CPPPATH = 'include'.split(),
	CCFLAGS = '-g -Wall -O0'.split()
	)

env.SharedLibrary('dubinspaths', ['src/dubins.c'])

env.Program('dubinstest', ['ctests/main.c'],
            LIBS=['dubinspaths','m'],
            LIBPATH='.',
            RPATH='.')
