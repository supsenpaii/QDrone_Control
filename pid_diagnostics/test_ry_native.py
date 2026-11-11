"""Tests the compiled C source that the MATLAB wrapper links into QUARC.

Native pure-math tests only: no target/model/QGC connections.
"""
import ctypes as ct
import math
import pathlib
import random
import unittest

Double = ct.c_double
Ptr = ct.POINTER(Double)
CFG = [.02, .01, .10, .015, .01, .003, 1, 1]


class Core:
    def __init__(self):
        self.lib = ct.CDLL(str(pathlib.Path(__file__).with_name('native') / 'qdrone_yaw_only_core.dll'))
        self.lib.qdrone_ry_core_step.argtypes = [Ptr]*5 + [Double]*3 + [Ptr]*2 + [Double]*2 + [Ptr]*2
        self.lib.qdrone_ry_core_step.restype = None
        self.lib.qdrone_ry_core_aw.argtypes = [Ptr]*5
        self.lib.qdrone_ry_core_aw.restype = None
        self.state = (Double*4)()

    def step(self, error=(1, 0, -1), angles=(0,0,0), gyro=(0,0,0), residual=(0,0,0),
             arm=1, throttle=10, mode=0, enable=(1,1), cfg=CFG, reset=0, dt=.001):
        def arr(v): return (Double*len(v))(*v)
        out, diag = (Double*3)(), (Double*10)()
        self.lib.qdrone_ry_core_step(self.state, arr(error), arr(angles), arr(gyro), arr(residual),
                                  arm, throttle, mode, arr(enable), arr(cfg), reset, dt, out, diag)
        return list(out), list(diag)

    def aw(self, motors, raw, limited, inverse):
        def arr(v): return (Double*len(v))(*v)
        out = (Double*3)()
        self.lib.qdrone_ry_core_aw(arr(motors), arr(raw), arr(limited), arr(inverse), out)
        return list(out)


class ControllerTests(unittest.TestCase):
    def test_sign_caps_slew_and_pitch_untouched(self):
        c=Core(); previous=[0,0,0]
        for _ in range(10000):
            out,_=c.step()
            self.assertEqual(out[1],0)
            self.assertEqual(out[0],0)
            self.assertTrue(-.015<=out[2]<=0)
            self.assertLessEqual(abs(out[0]-previous[0]),.01*.001+1e-12)
            self.assertLessEqual(abs(out[2]-previous[2]),.003*.001+1e-12)
            previous=out
        self.assertEqual(out[0],0)
        self.assertAlmostEqual(out[2],-.015)

    def test_transient_pauses_without_erasing_compensation(self):
        c=Core()
        for _ in range(500): baseline,_=c.step()
        for _ in range(100):
            out,diag=c.step(gyro=(0,math.radians(70),0))
            self.assertEqual(out,baseline)
            self.assertEqual(diag[:2],[0,2])
        out,diag=c.step()
        self.assertEqual(out[0],0)
        self.assertLess(out[2],baseline[2])
        self.assertEqual(diag[2:4],[0,0])

    def test_sustained_pause_sheds_integral_and_severe_envelope_resets(self):
        c=Core()
        for _ in range(1000):c.step()
        for _ in range(1300): out,diag=c.step(angles=(math.radians(20),0,0))
        self.assertEqual(out,[0,0,0]);self.assertEqual(diag[:2],[0,4])
        c.step()
        out,diag=c.step(gyro=(0,math.radians(310),0))
        self.assertEqual(out,[0,0,0]);self.assertEqual(diag[:2],[0,5])

    def test_hard_reset_gates(self):
        cases=[{'arm':0},{'throttle':0},{'mode':1},{'reset':1},{'dt':0},{'dt':.1},
               {'error':(1,0,float('nan'))},{'residual':(float('nan'),0,0)},
               {'cfg':[CFG[0],.1]+CFG[2:]}]
        for case in cases:
            c=Core()
            for _ in range(100):c.step()
            out,_=c.step(**case)
            self.assertEqual(out,[0,0,0],case)
            self.assertEqual(list(c.state),[0,0,0,0],case)

    def test_roll_is_removed_even_when_enabled_or_preloaded(self):
        c=Core()
        c.state[0]=.08;c.state[2]=.1
        for _ in range(100):out,_=c.step(enable=(1,0))
        self.assertEqual(out,[0,0,0])
        self.assertEqual(c.state[0],0);self.assertEqual(c.state[2],0)
        out,_=c.step(enable=(0,1))
        self.assertEqual(out[0],0);self.assertLess(out[2],0)

    def test_directional_antiwindup_and_unwind(self):
        c=Core()
        for _ in range(500):out,_=c.step()
        before=out
        # Negative yaw unavailable: yaw must unwind; roll output remains zero.
        out,diag=c.step(residual=(-.01,0,.01))
        self.assertEqual(diag[4:6],[0,1])
        self.assertEqual(out[0],0);self.assertGreater(out[2],before[2])
        # Opposite error (unwinding) is allowed while the actuator remains clipped.
        out2,diag=c.step(error=(-1,0,1),residual=(-.01,0,.01))
        self.assertEqual(diag[4:6],[0,0])
        self.assertEqual(out2[0],0);self.assertGreater(out2[2],out[2])

    def test_cross_axis_feedback_does_not_freeze_unaffected_axis(self):
        c=Core()
        out,diag=c.step(residual=(0,-1,0))
        self.assertEqual(out[0],0);self.assertLess(out[2],0)
        self.assertEqual(diag[4:6],[0,0])

    def test_allocator_residual_uses_column_major_inverse(self):
        c=Core()
        # Independent reference matrix-vector multiplication.
        inv=[float((r+1)*10+j) for j in range(4) for r in range(4)]
        m=[-.2,.4,1.3,.5]; raw=[.1,.2,.3];lim=[.08,.25,.2]
        dm=[min(1,max(0,x))-x for x in m]
        expected=[lim[r]-raw[r]+sum(inv[r+1+4*j]*dm[j] for j in range(4)) for r in range(3)]
        for actual,want in zip(c.aw(m,raw,lim,inv),expected):self.assertAlmostEqual(actual,want)
        identity=[float(r==j) for j in range(4) for r in range(4)]
        self.assertEqual(c.aw([.5]*4,raw,raw,identity),[0,0,0])

    def test_random_inputs_remain_bounded(self):
        c=Core();rng=random.Random(42)
        for _ in range(20000):
            out,_=c.step(error=[rng.uniform(-5,5) for _ in range(3)],
                         residual=[rng.uniform(-.5,.5) for _ in range(3)],
                         arm=1 if rng.random()>.01 else 0)
            self.assertTrue(all(math.isfinite(x) for x in out))
            self.assertEqual(out[0],0);self.assertLessEqual(abs(out[2]),.015)
            self.assertEqual(out[1],0)

    def test_synthetic_rate_plant_disturbance_rejection(self):
        # A simple mathematical test plant, NOT an identified QDrone model.
        c=Core(); rate=0.;disturbance=.008;p=.0395
        # Allow 100 seconds: deliberately small trial Ki has a slow response.
        for _ in range(100000):
            out,_=c.step(error=(5,0,-rate))
            self.assertEqual(out[0],0)
            rate+=.001*((-p*rate+out[2]+disturbance)/.03-.2*rate)
        self.assertLess(abs(rate),.004)


if __name__=='__main__':unittest.main(verbosity=2)
