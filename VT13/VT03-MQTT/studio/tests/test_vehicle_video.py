import copy
from pathlib import Path
import struct
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vehicle import DualInput
from video import Reassembler

def snapshot():
    hand={'tracked':True,'pose':{'position':[0,1,-.3],'orientation':[0,0,0,1]},'trigger':0}
    return {'fresh':True,'frame':{'visibility':'visible','head':{'position':[0,1.6,0],'orientation':[0,0,0,1]},'hands':{'left':copy.deepcopy(hand),'right':copy.deepcopy(hand)}}}

class VehicleTests(unittest.TestCase):
    def setUp(self):self.input=DualInput();self.s=snapshot();self.input.arm(self.s);self.input.update(self.s)
    def test_right_hand_four_directions(self):
        h=self.s['frame']['hands']['right'];h['trigger']=1
        for p,expected in (([0,1,-.45],(100,0)),([0,1,-.15],(-100,0)),([.15,1,-.3],(0,100)),([-.15,1,-.3],(0,-100))):
            h['pose']['position']=p;c=self.input.update(self.s)
            self.assertEqual((c['forward'],c['right']),expected);self.assertEqual(c['rotation'],0)
    def test_left_trigger_horizontal_controls_rotation_only(self):
        h=self.s['frame']['hands']['left'];h['trigger']=1
        h['pose']['position'][0]=-.15;c=self.input.update(self.s)
        self.assertEqual((c['forward'],c['right'],c['rotation']),(0,0,-100))
        h['pose']['position'][0]=.15;c=self.input.update(self.s);self.assertEqual(c['rotation'],100)
        h['trigger']=0;c=self.input.update(self.s);self.assertFalse(c['enabled']);self.assertEqual(c['rotation'],0)
    def test_simultaneous_hands_and_independent_release(self):
        l,r=[self.s['frame']['hands'][side] for side in ('left','right')]
        l['trigger']=r['trigger']=1;l['pose']['position'][0]=-.15;r['pose']['position'][2]=-.45
        c=self.input.update(self.s);self.assertEqual((c['forward'],c['rotation']),(100,-100))
        l['trigger']=0;c=self.input.update(self.s);self.assertEqual((c['forward'],c['rotation']),(100,0))
    def test_tracking_loss_requires_release_before_reentry(self):
        h=self.s['frame']['hands']['right'];h['trigger']=1;h['pose']['position'][2]=-.45
        self.assertEqual(self.input.update(self.s)['forward'],100)
        self.s['fresh']=False;self.assertFalse(self.input.update(self.s)['enabled'])
        self.s['fresh']=True;self.assertFalse(self.input.update(self.s)['enabled'])
        h['trigger']=0;self.input.update(self.s);h['trigger']=1;h['pose']['position'][2]=-.60
        self.assertEqual(self.input.update(self.s)['forward'],100)
    def test_arm_rejects_held_trigger_and_missing_head(self):
        self.s['frame']['hands']['left']['trigger']=1
        with self.assertRaises(ValueError):self.input.arm(self.s)
        self.s['frame']['hands']['left']['trigger']=0;self.s['frame']['head']=None
        with self.assertRaises(ValueError):self.input.arm(self.s)
    def test_control_expires_before_display_lease(self):
        self.s['frame']['hands']['right']['trigger']=1
        self.s['frame']['hands']['right']['pose']['position'][2]=-.45
        self.assertEqual(self.input.update(self.s)['forward'],100)
        self.s['age_ms']=121
        self.assertFalse(self.input.update(self.s)['enabled'])
    def test_disabled_and_body_heading_rotation(self):
        self.input.disarm();self.assertFalse(self.input.update(self.s)['enabled'])
        self.s['frame']['head']['orientation']=[0,2**-.5,0,2**-.5]
        self.input.arm(self.s);self.input.update(self.s)
        h=self.s['frame']['hands']['right'];h['trigger']=1;h['pose']['position'][0]=-.15
        self.assertEqual(self.input.update(self.s)['forward'],100)

class VideoTests(unittest.TestCase):
    def test_official_big_endian_reordered_duplicate_fragments(self):
        a=Reassembler();data=b'x'*3000
        packets=[struct.pack('>HHI',65535,i,len(data))+data[i*1392:(i+1)*1392] for i in range(3)]
        self.assertIsNone(a.feed(packets[2],0));self.assertIsNone(a.feed(packets[2],.01));self.assertIsNone(a.feed(packets[0],.02))
        self.assertEqual(a.feed(packets[1],.03),data)
    def test_loss_expires_and_malformed_lengths_rejected(self):
        a=Reassembler();p=struct.pack('>HHI',1,0,2000)+b'x'*1392
        self.assertIsNone(a.feed(p,0));self.assertIsNone(a.feed(struct.pack('>HHI',1,1,2000)+b'x'*608,.2))
        for p in (b'',struct.pack('>HHI',2,0,0)+b'x',struct.pack('>HHI',2,0,3000000)+b'x',struct.pack('>HHI',2,8,20)+b'x'*20):self.assertIsNone(a.feed(p,.21))
    def test_header_total_changes_discard_frame(self):
        a=Reassembler();a.feed(struct.pack('>HHI',1,0,2000)+b'x'*1392,0)
        self.assertIsNone(a.feed(struct.pack('>HHI',1,0,2100)+b'x'*1392,.01));self.assertNotIn(1,a.frames)

if __name__=='__main__':unittest.main(verbosity=2)
