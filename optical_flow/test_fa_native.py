"""Exercise the SAME C core used by MATLAB MEX and the QUARC target.
Synthetic optical fields verify algebra/gates, not drone flight stability.
"""
import ctypes as C
import math
import pathlib
import random
import unittest

ROOT=pathlib.Path(__file__).resolve().parent
DLL=C.CDLL(str(ROOT/'native/qdrone_fa_core.dll'))
P=C.POINTER(C.c_double);D=C.c_double
DLL.qdrone_fa_step.argtypes=[P,P,P,D,P,P,D,D,D,D,P,P,D,P,P]
DLL.qdrone_fa_step.restype=None
DLL.qdrone_fa_step_timed.argtypes=DLL.qdrone_fa_step.argtypes
DLL.qdrone_fa_step_timed.restype=None
def vec(values):return (D*len(values))(*values)

class Sim:
    def __init__(self,timed=False):
        self.s=vec([0]*(1045 if timed else 16));self.t=0.;self.ft=0.;self.cmd=[0.,0.,.2]
        self.step=DLL.qdrone_fa_step_timed if timed else DLL.qdrone_fa_step
        self.cfg=[1,400,400,1,.03,math.radians(3),math.radians(4),.1,.35,.1,.35,math.radians(1),math.radians(12)]
        self.R=[0,-1,0,-1,0,0,0,0,-1] # column-major
        self.g=[0.,0.,0.];self.a=[0.,0.,0.];self.arm=1.;self.throttle=10.;self.mode=0.;self.req=1.
        self.f=[0.,0.,.8,1.,.033,0.,30.,28.];self.out=[0]*3;self.diag=[0]*16
    def field(self,v=(0.,0.,0.)):
        # v is v_CG/h_ref in body coordinates. Camera scene translation
        # and angular rotation both contribute; lever correction is included.
        wc=[sum(self.R[k+3*i]*self.g[k] for k in range(3)) for i in range(3)]
        lever=[-.03*self.g[1],.1*self.g[2]+.03*self.g[0],-.1*self.g[1]]
        lc=[sum(self.R[k+3*i]*lever[k] for k in range(3)) for i in range(3)]
        vc=[sum(self.R[k+3*i]*v[k] for k in range(3)) for i in range(3)]
        self.f[0]=self.cfg[1]*.033*(-vc[0]-wc[1]-lc[0]/self.cfg[3])
        self.f[1]=self.cfg[2]*.033*(-vc[1]+wc[0]-lc[1]/self.cfg[3])
    def tick(self,frame=True):
        self.t=round(self.t+.001,9)
        if frame and round(self.t*1000)%33==0:self.ft=self.t
        out=vec([0]*3);diag=vec([0]*16)
        self.step(self.s,vec(self.cmd),vec(self.f),self.ft,vec(self.g),vec(self.a),self.arm,self.throttle,
            self.mode,self.req,vec(self.cfg),vec(self.R),self.t,out,diag)
        self.out=list(out);self.diag=list(diag)
        return self.out
    def run(self,n=1200,frame=True):
        for _ in range(n):self.tick(frame)
        return self.out

class Tests(unittest.TestCase):
    def bounded_sim(self):
        s=Sim(timed=True);s.cfg[0]=4;s.cfg[4]=.01
        s.cfg[5]=math.radians(.5);s.cfg[6]=math.radians(1);s.cfg[12]=math.radians(8)
        return s

    def test_bounded_active_trial_sign_cap_slew_and_yaw_passthrough(self):
        for vx,vy in ((1.2,0),(-1.2,0),(0,1.2),(0,-1.2)):
            s=self.bounded_sim();s.field((vx,vy,0));last=[0,0]
            for _ in range(2500):
                s.tick()
                for axis in range(2):
                    self.assertLessEqual(abs(s.out[axis]-s.cmd[axis]),s.cfg[5]+1e-12)
                    self.assertLessEqual(abs(s.out[axis]-last[axis]),s.cfg[6]*.001+1e-12)
                self.assertEqual(s.out[2],s.cmd[2]);last=s.out[:2]
            self.assertEqual(s.diag[0],5)
            self.assertAlmostEqual(s.out[0],math.copysign(s.cfg[5],vy) if vy else 0)
            self.assertAlmostEqual(s.out[1],-math.copysign(s.cfg[5],vx) if vx else 0)

    def test_bounded_active_trial_immediate_pilot_and_disarm_priority(self):
        for fault in ('roll','pitch','disarm','throttle','request','mode','tilt','gyro'):
            s=self.bounded_sim();s.field((.4,.4,0));s.run();self.assertEqual(s.diag[0],5)
            if fault=='roll':s.cmd[0]=.04
            if fault=='pitch':s.cmd[1]=-.04
            if fault=='disarm':s.arm=0
            if fault=='throttle':s.throttle=3
            if fault=='request':s.req=0
            if fault=='mode':s.mode=1
            if fault=='tilt':s.a[0]=math.radians(9)
            if fault=='gyro':s.g[2]=.9
            s.tick();self.assertEqual(s.out,s.cmd)
            self.assertEqual(s.diag[0],0 if fault=='request' else 2)

    def test_bounded_active_trial_rejects_unsafe_limits_and_legacy_entrypoint(self):
        for index,value in ((4,.011),(5,math.radians(.6)),(6,math.radians(1.1)),(12,math.radians(9))):
            s=self.bounded_sim();s.cfg[index]=value;s.field((.4,0,0));s.run()
            self.assertEqual(s.diag[0],1);self.assertEqual(s.out,s.cmd)
        s=self.bounded_sim();s.step=DLL.qdrone_fa_step;s.field((.4,0,0));s.run()
        self.assertEqual(s.diag[0],1);self.assertEqual(s.out,s.cmd)

    def test_bounded_active_trial_stale_invalid_flow_clock_and_recovery(self):
        for fault in ('stale','invalid','quality','clock'):
            s=self.bounded_sim();s.field((.4,0,0));s.run();self.assertEqual(s.diag[0],5)
            if fault=='stale':s.run(150,False)
            if fault=='invalid':s.f[3]=0;s.run(40)
            if fault=='quality':s.f[2]=.1;s.run(40)
            if fault=='clock':s.t=0;s.tick(False)
            self.assertEqual(s.out,s.cmd)
            self.assertEqual(s.diag[0],8 if fault=='clock' else 4)
            s.ft=0 if fault=='clock' else s.ft;s.f[3]=1;s.f[2]=.8
            s.run(600);self.assertEqual(s.diag[0],5)

    def test_timed_trial_cancels_varying_rotation_with_delayed_delivery(self):
        aligned=Sim(timed=True);aligned.cfg[0]=3;aligned.arm=0;aligned.throttle=0
        legacy=Sim();legacy.cfg[0]=2;legacy.arm=0;legacy.throttle=0
        errors=[[],[]]
        frequency=2*math.pi*2
        for k in range(1,5001):
            t=round(k*.001,9)
            gyro=[.18*math.sin(frequency*t),.07*math.cos(frequency*t),.12*math.sin(frequency*t)]
            for s in (aligned,legacy):s.g=gyro
            if k%33==0 and t>.1:
                frame_time=round(t-.033,9)
                end=frame_time-.034;begin=end-.033
                mean_sin=(math.cos(frequency*begin)-math.cos(frequency*end))/(frequency*.033)
                mean_cos=(math.sin(frequency*end)-math.sin(frequency*begin))/(frequency*.033)
                aligned.g=[.18*mean_sin,.07*mean_cos,.12*mean_sin]
                aligned.field();aligned.g=gyro
                legacy.f=aligned.f[:]
                for s in (aligned,legacy):s.ft=frame_time
            for n,s in enumerate((aligned,legacy)):
                s.tick(frame=False)
                if k>1000 and k%33==0:
                    self.assertEqual(s.diag[0],9)
                    errors[n].extend(s.diag[6:8])
                self.assertEqual(s.out,s.cmd)
        rms=[math.sqrt(sum(v*v for v in row)/len(row)) for row in errors]
        self.assertLess(rms[0],.0001)
        self.assertGreater(rms[1],.02)

    def test_timed_wrapper_preserves_legacy_stages(self):
        for stage in (1,2):
            old=Sim();new=Sim(timed=True)
            old.cfg[0]=new.cfg[0]=stage
            old.arm=new.arm=1 if stage==1 else 0
            for k in range(2500):
                for s in (old,new):
                    s.g=[.05*math.sin(k*.01),0,0];s.field((.2,.1,0));s.tick()
                self.assertEqual(old.out,new.out)
                self.assertEqual(old.diag,new.diag)

    def test_timed_trial_raw_gyro_gate_clock_reset_staleness_and_recovery(self):
        s=Sim(timed=True);s.cfg[0]=3;s.arm=0;s.field();s.run()
        self.assertEqual(s.diag[0],9)
        s.g[2]=.9;s.tick();self.assertEqual(s.diag[0],2)
        s.g[2]=0;s.run(600);self.assertEqual(s.diag[0],9)
        s.arm=1;s.tick();self.assertEqual(s.diag[0],2);self.assertEqual(s.out,s.cmd)
        s.arm=0;s.run(600)
        s.run(150,False);self.assertEqual(s.diag[0],4)
        s.run(600);self.assertEqual(s.diag[0],9)
        s.t=0;s.tick(False);self.assertEqual(s.diag[0],8)
        self.assertTrue(all(v==0 for v in s.s))
        s.ft=0;s.run(600);self.assertEqual(s.diag[0],9)

    def test_timed_trial_rejects_missing_history_and_corrupt_state(self):
        s=Sim(timed=True);s.cfg[0]=3;s.arm=0;s.field()
        s.run(50);self.assertEqual(s.diag[0],4)
        s.s[16]=float('nan');s.tick();self.assertEqual(s.out,s.cmd)
        s.run(600);self.assertEqual(s.diag[0],9)
        s.s[16]=256.5;s.tick();self.assertEqual(s.out,s.cmd)

    def test_camera_poll_without_new_frame_holds_packet_and_still_times_out(self):
        s=Sim();s.cfg[0]=2;s.arm=0;s.throttle=0;s.field()
        # Model a 33 ms camera poll that occasionally has no new image.
        # The publisher holds BOTH the old flow and its original timestamp.
        polls=0
        for k in range(1,10001):
            if k%33==0:
                polls+=1
                if polls%101!=0:
                    s.ft=round(k*.001,9)
                    s.f[4]=.066 if polls%101==1 and polls>1 else .033
            s.tick(frame=False)
            if k>600:self.assertEqual(s.diag[0],9)
        s.run(150,frame=False)
        self.assertEqual(s.diag[0],4)
        self.assertEqual(s.out,s.cmd)

    def test_disarmed_shadow_does_not_change_commands(self):
        s=Sim();s.cfg[0]=2;s.arm=0;s.throttle=0;s.field((.4,.4,0));s.run()
        self.assertEqual(s.diag[0],9);self.assertEqual(s.out,s.cmd)
        self.assertGreater(s.diag[10],0);self.assertLess(s.diag[11],0)
        s.arm=1;s.run();self.assertEqual(s.out,s.cmd);self.assertEqual(s.diag[0],2)
    def test_forward_motion_brakes_with_negative_pitch(self):
        s=Sim();s.field((.4,0,0));s.run();self.assertEqual(s.diag[0],5);self.assertLess(s.out[1],-.01);self.assertAlmostEqual(s.out[0],0)
    def test_left_motion_brakes_with_positive_roll(self):
        s=Sim();s.field((0,.4,0));s.run();self.assertGreater(s.out[0],.01);self.assertAlmostEqual(s.out[1],0)
    def test_all_translation_signs_and_rotation_mount(self):
        for R in ([0,-1,0,-1,0,0,0,0,-1],[1,0,0,0,-1,0,0,0,-1],[-1,0,0,0,1,0,0,0,-1]):
            for vx,vy in ((.3,0),(-.3,0),(0,.3),(0,-.3)):
                s=Sim();s.R=list(R);s.field((vx,vy,0));s.run()
                self.assertAlmostEqual(s.out[0],.03*vy,places=4);self.assertAlmostEqual(s.out[1],-.03*vx,places=4)
    def test_rotation_and_lever_arm_cancel(self):
        for g in ((.1,.12,.2),(-.1,-.12,-.2),(0,0,.5)):
            s=Sim();s.g=list(g);s.field();s.run();self.assertEqual(s.diag[0],5)
            self.assertLess(max(abs(x) for x in s.out[:2]),1e-12)
    def test_calibration_lock_and_bad_geometry(self):
        for kind in ('unconfirmed','fx','height','reflection','not_down'):
            s=Sim();s.field((.4,0,0))
            if kind=='unconfirmed':s.cfg[0]=0
            if kind=='fx':s.cfg[1]=0
            if kind=='height':s.cfg[3]=0
            if kind=='reflection':s.R[8]=1
            if kind=='not_down':s.R=[1,0,0,0,1,0,0,0,1]
            s.run();self.assertEqual(s.out,s.cmd);self.assertEqual(s.diag[0],1)
    def test_manual_priority_and_gates(self):
        for kind in ('roll','pitch','disarm','throttle','mode','request','tilt','gyro'):
            s=Sim();s.field((.4,.4,0));s.run();self.assertEqual(s.diag[0],5)
            if kind=='roll':s.cmd[0]=.04
            if kind=='pitch':s.cmd[1]=-.04
            if kind=='disarm':s.arm=0
            if kind=='throttle':s.throttle=3
            if kind=='mode':s.mode=1
            if kind=='request':s.req=0
            if kind=='tilt':s.a[0]=.3
            if kind=='gyro':s.g[2]=1
            s.tick();self.assertEqual(s.out,s.cmd);self.assertEqual(s.s[9],0);self.assertEqual(s.s[10],0)
    def test_stale_camera_reverts_and_requalifies(self):
        s=Sim();s.field((.4,.4,0));s.run();s.run(110,False);self.assertEqual(s.out,s.cmd);self.assertEqual(s.diag[0],4)
        s.run(120);self.assertNotEqual(s.diag[0],5);s.run(500);self.assertEqual(s.diag[0],5)
    def test_bad_flow_quality_timestamp_and_search_boundary(self):
        for kind in ('invalid','quality','age','dt','nan','outlier','inliers','future'):
            s=Sim();s.field((.4,.4,0));s.run()
            if kind=='invalid':s.f[3]=0
            if kind=='quality':s.f[2]=.1
            if kind=='age':s.f[5]=.04
            if kind=='dt':s.f[4]=.08
            if kind=='nan':s.f[0]=float('nan')
            if kind=='outlier':s.f[0]=24
            if kind=='inliers':s.f[7]=4
            if kind=='future':s.ft=s.t+.2
            s.run(40,kind!='future');self.assertEqual(s.out,s.cmd)
    def test_cap_slew_and_no_integrator(self):
        s=Sim();s.cfg[4]=.1;s.field((1,1,0));last=[0,0]
        for _ in range(2500):
            s.tick()
            for j in range(2):
                self.assertLessEqual(abs(s.out[j]),math.radians(3)+1e-10)
                self.assertLessEqual(abs(s.out[j]-last[j]),s.cfg[6]*.001+1e-10)
            self.assertEqual(s.out[2],s.cmd[2]);last=s.out[:2]
        s.field();s.run(2000);self.assertLess(max(abs(x) for x in s.out[:2]),1e-7)
    def test_duplicate_timestamp_is_not_new_camera_data(self):
        s=Sim();s.field((.4,0,0));s.run(300,False);self.assertNotEqual(s.diag[0],5)
    def test_clock_rewind_clears(self):
        s=Sim();s.field((.4,0,0));s.run();s.t=0;s.tick(False);self.assertEqual(s.out,s.cmd);self.assertEqual(s.diag[0],8)
    def test_yaw_passthrough_and_random_bounded_outputs(self):
        rng=random.Random(41);s=Sim()
        for n in range(15000):
            if n%33==0:
                s.cmd=[0,0,rng.uniform(-.6,.6)];s.field((rng.uniform(-1,1),rng.uniform(-1,1),0))
            s.tick();self.assertTrue(all(math.isfinite(x) for x in s.out));self.assertEqual(s.out[2],s.cmd[2])
            self.assertLessEqual(max(abs(x) for x in s.out[:2]),s.cfg[5]+1e-12)

if __name__=='__main__':unittest.main(verbosity=2)
