#!/usr/bin/python
import dubins
import unittest
import numpy
import math

class DubinsTests(unittest.TestCase):
    def testLRL(self):
        self.assertAlmostEquals( sum(dubins.LSLpath( 0, 0, 3.2 )) , 3.2 )

    def testNormalisation(self):
        q0 = numpy.array([ 0.,0.,0.])
        q1 = numpy.array([ 1.,1.,1.])
        offset = numpy.array([ 3.,3.,0.])
        path0 = dubins.init( q0, q1 )
        path1 = dubins.init( q0+offset, q1+offset )
        for i in xrange(3):
            self.assertAlmostEquals( path0.params[i], path1.params[i] )

    def testSampling(self):
        def normaliseConfiguration( q ):
            qc = numpy.array(q).copy()
            qc[2] = qc[2] % (2. * math.pi)
            return qc

        def impl( q0, q1 ):
            q0 = normaliseConfiguration( q0 )
            q1 = normaliseConfiguration( q1 )
            path = dubins.init( q0, q1 )
            length = dubins.pathLength(path)
            qr = dubins.pathSample(path,length-1e-10)[:]
            qr = numpy.array(qr)
            for i in xrange(3):
                print qr, q1, path.type
                self.assertAlmostEquals( qr[i], q1[i] )

        q0 = [0,0,0.]
        for _ in xrange(10000):
            q1 = (numpy.random.random(3) * numpy.array([10.,10., 2*math.pi]) ) - numpy.array([5.,5.,0.])
            impl(q0,q1)
if __name__ == "__main__":
    unittest.main()


##from ctypes import *
##import numpy
#
#lib = CDLL('./libdubinspaths.so')
#inf = float('inf')
#
#
#class DubinsPathStruct(Structure):
#    _fields_ = [('qi',    c_double * 3),
#                ('params', c_double * 3),
#                ('type',  c_int)]
#
#    def __str__(self):
#        return 'q = %s, p = %s, type = %d' % (self.qi[:], self.params[:],self.type)
#
#def createDubinsFunction( name ):
#    func = getattr(lib, name)
#    def res( alpha, beta, d ):
#        output = (c_double * 3)(inf, inf, inf)
#        func( c_double(alpha), c_double(beta), c_double(d), output )
#        return output[:]
#    return res
#
#LRLpath = createDubinsFunction( 'dubins_LRL' )
#RLRpath = createDubinsFunction( 'dubins_RLR' )
#LSLpath = createDubinsFunction( 'dubins_LSL' )
#LSRpath = createDubinsFunction( 'dubins_LSR' )
#RSLpath = createDubinsFunction( 'dubins_RSL' )
#RSRpath = createDubinsFunction( 'dubins_RSR' )
#
#def dubinsInit( q0, q1, r=1. ):
#    configuration = c_double * 3
#    q0p = configuration(*tuple(q0))
#    q1p = configuration(*tuple(q1))
#    path = DubinsPathStruct()
#    lib.dubins_init( q0p, q1p, c_double(r), pointer(path) )
#    return path
#
#def dubinsInitNormalised( alpha, beta, d ):
#    path = DubinsPathStruct()
#    lib.dubins_init_normalised( c_double(alpha), c_double(beta), c_double(d), pointer(path) );
#    return path
#
#if __name__ == "__main__":
#    import unittest
#

