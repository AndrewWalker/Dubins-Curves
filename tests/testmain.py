#!/usr/bin/python
from ctypes import *

lib = CDLL('./libdubinspaths.so')
inf = float('inf')

class DubinsPathStruct(Structure):
    _fields_ = [('qi',    c_double * 3),
                ('params', c_double * 3),
                ('type',  c_int)]

class DubinsPath(object):
    def __init__(self, pathStruct):
        self.type   = pathStruct.type
        self.qi     = path.qi[:]
        self.params = path.params[:]

#print path.type
#print path.qi[:]
#print path.params[:]

def createDubinsFunction( name ):
    func = getattr(lib, name)
    def res( alpha, beta, d ):
        output = (c_double * 3)(inf, inf, inf)
        func( c_double(alpha), c_double(beta), c_double(d), output )
        return output[:]
    return res

LRL = createDubinsFunction( 'dubins_LRL' )
RLR = createDubinsFunction( 'dubins_RLR' )
LSL = createDubinsFunction( 'dubins_LSL' )
LSR = createDubinsFunction( 'dubins_LSR' )
RSL = createDubinsFunction( 'dubins_RSL' )
RSR = createDubinsFunction( 'dubins_RSR' )

def dubinsInit( q0, q1, r=1. ):
    configuration = c_double * 3
    q0p = configuration(*tuple(q0))
    q1p = configuration(*tuple(q1))
    path = DubinsPathStruct()
    lib.dubins_init( q0p, q1p, c_double(r), pointer(path) )
    return path

def dubinsInitNormalised( alpha, beta, d ):
    path = DubinsPathStruct()
    lib.dubins_init_normalised( c_double(alpha), c_double(beta), c_double(d), pointer(path) );
    return path

def dubinsNormalised2( alpha, beta, d ):
    funcs = [ LRL, RLR, LSL, LSR, RSL, RSR ]
    outs  = [ fun(alpha,beta,d) for fun in funcs ]
    lens  = [ sum(o) for o in outs ]
    for f, o, l in zip(funcs,outs,lens):
        print f, o, l    

#dubinsNormalised2( 1., 1., 5. )
#path = dubinsInitNormalised( 1., 1., 5. )
#print path.type
#print path.qi[:]
#print path.params[:]

import unittest

class DubinsTests(unittest.TestCase):
    def testLRL(self):
        self.assertAlmostEquals( sum(LSL( 0, 0, 3.2 )) , 3.2 )

    def testNormalisation(self):
        q0 = [ 0.,0.,0.]
        q1 = [ 1.,1.,1.]
        dubinsInit( q0, q1 )
        #self.

unittest.main()

