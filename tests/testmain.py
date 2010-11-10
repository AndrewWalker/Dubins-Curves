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
            #print qr, q1, path.type
            self.assertAlmostEquals( qr[0], q1[0] )
            self.assertAlmostEquals( qr[1], q1[1] )
            self.assertAlmostEquals( qr[2], q1[2] )
        q0 = [0,0,0.]
        for _ in xrange(1):#(100000):
            q1 = (numpy.random.random(3) * numpy.array([10.,10., 2*math.pi]) ) - numpy.array([5.,5.,0.])
            impl(q0,q1)

    def testSameConfigurations(self):
        q = [0,0,0]
        path = dubins.init( q, q )
        self.assertAlmostEquals(dubins.pathLength(path), 0. )

if __name__ == "__main__":
    unittest.main()


