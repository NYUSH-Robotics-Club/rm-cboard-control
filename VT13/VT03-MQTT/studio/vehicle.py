"""Independent hand clutches. RC ticks, not metres/second or radians/second."""
import math

class DualInput:
    def __init__(self):
        self.armed=False
        self.basis=None
        self.reason='未启用车辆控制'
        self.reset_clutches()

    def reset_clutches(self):
        self.hands={side:{'released':False,'active':False,'origin':None} for side in ('left','right')}

    def disarm(self):
        self.armed=False
        self.reset_clutches()
        self.reason='已停止车辆控制'

    def arm(self, snapshot, basis=None):
        frame = ready_frame(snapshot)
        if basis is not None:
            self.basis = basis
            self.reset_clutches()
            self.armed = True
            self.reason = '已启用，按住对应扳机后移动手柄'
            return
        x,y,z,w=frame['head']['orientation']
        # Rotate WebXR forward (0,0,-1), then project onto the floor.
        fx,fz=-2*(x*z+w*y),-(1-2*(x*x+y*y))
        length=math.hypot(fx,fz)
        if length<.3: raise ValueError('请平视前方后启用')
        forward=(fx/length,fz/length)
        right=(-forward[1],forward[0])
        self.basis={'forward':forward,'right':right}
        self.reset_clutches();self.armed=True
        self.reason='已启用；两手先松开扳机，再分别按住移动'

    def _hand(self, side, hand):
        clutch=self.hands[side]
        if not hand.get('tracked'):
            clutch.update(released=False,active=False,origin=None)
            return False,0,0
        p=hand['pose']['position']
        point=(sum(p[i]*a for i,a in zip((0,2),self.basis['right'])),sum(p[i]*a for i,a in zip((0,2),self.basis['forward'])))
        trigger=hand['trigger']
        if trigger<=.2:
            clutch.update(released=True,active=False,origin=point)
            return False,0,0
        if trigger>=.65 and clutch['released'] and clutch['origin'] is not None:
            clutch['active']=True
        if not clutch['active']:return False,0,0
        dx,dy=(v-o for v,o in zip(point,clutch['origin']))
        return True,dx,dy

    def update(self, snapshot):
        if not self.armed:return {'enabled':False,'forward':0,'right':0,'rotation':0,'reason':self.reason}
        if not snapshot['fresh'] or (snapshot.get('age_ms') or 0)>120 or snapshot['frame']['visibility']!='visible':
            self.reset_clutches()
            return {'enabled':False,'forward':0,'right':0,'rotation':0,'reason':'追踪失效；恢复后需松开扳机重新接管'}
        frame=snapshot['frame']
        ra,rx,ry=self._hand('right',frame['hands']['right'])
        la,lx,_=self._hand('left',frame['hands']['left'])
        forward=right=rotation=0
        magnitude=math.hypot(rx,ry)
        if ra and magnitude>.03:
            scale=100*min(1,(magnitude-.03)/.12)/magnitude
            forward,right=round(ry*scale),round(rx*scale)
        # +RC dial maps to +wz (counterclockwise in the chassis convention).
        # Left hand left => negative right displacement => clockwise.
        if la and abs(lx)>.03:
            rotation=round(math.copysign(100*min(1,(abs(lx)-.03)/.12),lx))
        return {'enabled':ra or la,'forward':forward,'right':right,'rotation':rotation,'reason':'右手平移 / 左手旋转；各自松扳机归零'}


def ready_frame(snapshot):
    if not snapshot['fresh'] or (snapshot.get('age_ms') or 0) > 120:
        raise ValueError('进入 VR 并保持两只手柄追踪有效')
    frame = snapshot['frame']
    if frame['visibility'] != 'visible' or not frame.get('head') or not all(
        hand['tracked'] for hand in frame['hands'].values()
    ):
        raise ValueError('请保持头显和两只手柄追踪有效')
    if any(hand['trigger'] > .2 for hand in frame['hands'].values()):
        raise ValueError('请松开两只扳机，再记录或启用')
    return frame


class DirectionCalibration:
    prompts = {
        'idle': '点击重新校准，开始记录方向',
        'center': '双手放在舒适中立位，松开扳机，记录中立',
        'forward': '右手从中立向正前方平移 15–25 cm，保持位置，记录前方',
        'right': '右手回到中立，再向身体右侧平移 15–25 cm，记录右方',
        'neutral': '双手回到中立并松开扳机，完成校准',
        'repository': '正在原版页面校准，底盘已归零',
        'ready': '校准完成。可以启用控车',
    }

    def __init__(self):
        self.stage = 'idle'
        self.center = None
        self.forward = None
        self.basis = None
        self.last_seq = -1

    def start(self):
        self.__init__()
        self.stage = 'center'

    def status(self):
        return {'stage': self.stage, 'prompt': self.prompts[self.stage], 'basis': self.basis}

    def capture(self, snapshot):
        if self.stage not in ('center', 'forward', 'right', 'neutral'):
            raise ValueError('请先开始重新校准')
        frame = ready_frame(snapshot)
        if frame['seq'] <= self.last_seq:
            raise ValueError('请等待新的手柄帧')
        positions = {side: tuple(hand['pose']['position'][i] for i in (0, 2))
                     for side, hand in frame['hands'].items()}
        if self.stage == 'center':
            self.center = positions
            self.stage = 'forward'
        elif self.stage == 'neutral':
            if any(math.dist(positions[side], self.center[side]) > .10 for side in positions):
                raise ValueError('两只手柄请回到最初的中立位置')
            self.stage = 'ready'
        else:
            delta = tuple(value - center for value, center in zip(positions['right'], self.center['right']))
            length = math.hypot(*delta)
            if not .12 <= length <= .50:
                raise ValueError('从中立位置水平移动约 15–25 cm 后重试')
            direction = tuple(value / length for value in delta)
            if self.stage == 'forward':
                self.forward = direction
                self.stage = 'right'
            else:
                fx, fz = self.forward
                expected_right = (-fz, fx)
                if sum(a * b for a, b in zip(direction, expected_right)) < .82:
                    raise ValueError('右向与前向应接近直角，请回中立再向正右平移')
                self.basis = {'forward': self.forward, 'right': expected_right}
                self.stage = 'neutral'
        self.last_seq = frame['seq']

