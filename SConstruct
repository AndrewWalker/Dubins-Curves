import os

env = Environment( 
	ENV = os.environ,
	CCFLAGS = '-g -Wall -O0'.split()
	)
env.SharedLibrary('dubinspaths', ['dubins.cpp'])
env.Program('dubinsmain', ['main.c'], LIBS=['dubinspaths'], LIBPATH='.')
