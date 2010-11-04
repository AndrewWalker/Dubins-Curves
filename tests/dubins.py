import ctypes

_lib = ctypes.CDLL('../libdubinspaths.so')
_inf = float('inf')

class _DubinsPathStruct(ctypes.Structure):
    _fields_ = [('qi',     ctypes.c_double * 3),
                ('params', ctypes.c_double * 3),
                ('type',   ctypes.c_int)]

    def __str__(self):
        return 'q = %s, p = %s, type = %d' % (self.qi[:], self.params[:],self.type)
    
_Configuration = ctypes.c_double * 3

def _createDubinsFunction( name ):
    func = getattr(_lib, name)
    def res( alpha, beta, d ):
        output = (ctypes.c_double * 3)(_inf, _inf, _inf)
        a = ctypes.c_double(alpha)
        b = ctypes.c_double(beta)
        d = ctypes.c_double(d)
        func( a, b, d, output )
        return output[:]
    return res

LRLpath = _createDubinsFunction( 'dubins_LRL' )
RLRpath = _createDubinsFunction( 'dubins_RLR' )
LSLpath = _createDubinsFunction( 'dubins_LSL' )
LSRpath = _createDubinsFunction( 'dubins_LSR' )
RSLpath = _createDubinsFunction( 'dubins_RSL' )
RSRpath = _createDubinsFunction( 'dubins_RSR' )

def init( q0, q1, r=1. ):
    q0p = _Configuration(*tuple(q0))
    q1p = _Configuration(*tuple(q1))
    path = _DubinsPathStruct()
    _lib.dubins_init( q0p, q1p, ctypes.c_double(r), ctypes.pointer(path) )
    return path

def initNormalised( alpha, beta, d ):
    path = _DubinsPathStruct()
    a = ctypes.c_double(alpha)
    b = ctypes.c_double(beta)
    d = ctypes.c_double(d)
    _lib.dubins_init_normalised( a, b, d, pointer(path) );
    return path

def pathLength( path ):
    fun = _lib.dubins_path_length
    fun.restype = ctypes.c_double
    return fun(ctypes.pointer(path))

def pathSample( path, t ):
    q = _Configuration()
    _lib.dubins_path_sample(ctypes.pointer(path), ctypes.c_double(t), q)
    return q[:]

# types codes
(LSL, LSR, RSL, RSR, RLR, LRL) = map( ctypes.c_int, xrange(6) ) 
