"""Shared SB selector C: no joystick/drone access, no actuator commands."""
import ctypes as C
import pathlib
import unittest

D=C.c_double;P=C.POINTER(D)
lib=C.CDLL(str(pathlib.Path(__file__).resolve().parent/'native/qdrone_sb_mode_core.dll'))
lib.qdrone_sb_mode.argtypes=[P,D,D,D,D,D,P,P];lib.qdrone_sb_mode.restype=None
class Sim:
    def __init__(self):self.s=(D*6)();self.t=0.;self.mode=0.;self.diag=[]
    def run(self,x,n=300,fresh=1,base=0,en=1):
        for _ in range(n):
            self.t=round(self.t+.001,9);o=D();d=(D*6)()
            lib.qdrone_sb_mode(self.s,x,fresh,self.t,base,en,C.byref(o),d)
            self.mode=o.value;self.diag=list(d)
        return self.mode
class Tests(unittest.TestCase):
    def test_up_middle_down(self):
        s=Sim();self.assertEqual(s.run(-1),0);self.assertEqual(s.run(0),0);self.assertEqual(s.run(1),2)
    def test_start_down_does_not_activate(self):
        s=Sim();self.assertEqual(s.run(1,2000),0);s.run(0);self.assertEqual(s.run(1),0)
    def test_up_must_be_held(self):
        s=Sim();s.run(-1,100);self.assertEqual(s.run(1),0)
    def test_down_debounce(self):
        s=Sim();s.run(-1);self.assertEqual(s.run(1,100),0);self.assertEqual(s.run(1,50),2)
    def test_exit_immediately(self):
        for x in (0,-1):
            s=Sim();s.run(-1);s.run(1);self.assertEqual(s.run(x,1),0)
    def test_dropout_returns_angle_requires_up_again(self):
        s=Sim();s.run(-1);s.run(1);self.assertEqual(s.run(1,260,fresh=0),0)
        self.assertEqual(s.run(1),0);s.run(-1);self.assertEqual(s.run(1),2)
    def test_disabled_and_torque_never_FA(self):
        s=Sim();s.run(-1);self.assertEqual(s.run(1,en=0),0)
        s.run(-1);self.assertEqual(s.run(1,base=1),1)
    def test_invalid_axis_never_triggers(self):
        for x in (2,-2,.5,-.5,float('nan'),float('inf')):
            s=Sim();s.run(-1);self.assertEqual(s.run(x),0);self.assertEqual(s.run(1),0)
    def test_clock_fault_resets(self):
        s=Sim();s.run(-1);s.run(1);s.t=0;self.assertEqual(s.run(1,1),0);self.assertEqual(s.diag[0],6)
    def test_idle_connection_qualification(self):
        # Adapter truth table: valid = connected AND (new_event OR seen).
        # No event after the SB transition, but a connected sample stays valid.
        s=Sim();seen=False
        def tick(axis,connected,event):
            nonlocal seen
            seen=bool(connected and (event or seen))
            return s.run(axis,1,fresh=int(seen))
        for _ in range(500):tick(-1,True,False)
        self.assertEqual(s.diag[0],5)
        tick(-1,True,True)
        for _ in range(300):tick(-1,True,False)
        tick(1,True,True)
        for _ in range(5000):tick(1,True,False)
        self.assertEqual(s.mode,2)
        for _ in range(260):tick(1,False,False)
        self.assertEqual(s.mode,0)
        for _ in range(300):tick(1,True,False)
        self.assertEqual(s.diag[0],5)
        tick(1,True,True)
        for _ in range(300):tick(1,True,False)
        self.assertEqual(s.mode,0)  # reconnect still requires UP qualification
        tick(-1,True,True)
        for _ in range(300):tick(-1,True,False)
        tick(1,True,True)
        for _ in range(300):tick(1,True,False)
        self.assertEqual(s.mode,2)
if __name__=='__main__':unittest.main(verbosity=2)
