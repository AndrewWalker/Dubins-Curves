import os

env = Environment( 
	ENV = os.environ,
	CCFLAGS = '-g -Wall -O0'.split()
	)
srcs = Glob('*.cpp') + Glob('*.c')
#env.Program('dubinsmain', srcs)
env.SharedLibrary('dubinspaths', srcs )
