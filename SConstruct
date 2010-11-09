import os

env = Environment( 
	ENV = os.environ,
    CPPPATH = 'include'.split(),
	CCFLAGS = '-g -Wall -O0'.split()
	)

env.SharedLibrary('dubinspaths', ['src/dubins.cpp'])

env.Program('dubinsmain', ['ctests/main.cpp'], 
            LIBS=['dubinspaths'], 
            LIBPATH='.')
