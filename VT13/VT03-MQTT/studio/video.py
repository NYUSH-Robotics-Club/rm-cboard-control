"""Official local UDP/3334 HEVC -> latest JPEG; no simulated image fallback."""
import io
import math
import socket
import struct
import threading
import time

class Reassembler:
    MAX_BYTES=2*1024*1024
    def __init__(self):self.frames={}
    def feed(self, packet, now):
        self.frames={n:f for n,f in self.frames.items() if now-f['time']<.18}
        if not 9<=len(packet)<=1400:return None
        frame,index,total=struct.unpack('>HHI',packet[:8])
        if not 0<total<=self.MAX_BYTES:return None
        count=math.ceil(total/1392)
        if index>=count:return None
        payload=packet[8:]
        expected=1392 if index<count-1 else total-1392*(count-1)
        if len(payload)!=expected:return None
        entry=self.frames.get(frame)
        if entry is None:
            if len(self.frames)>=4:self.frames.pop(next(iter(self.frames)))
            entry=self.frames[frame]={'time':now,'total':total,'parts':{}}
        if entry['total']!=total:
            self.frames.pop(frame,None);return None
        entry['parts'][index]=payload
        if len(entry['parts'])!=count:return None
        self.frames.pop(frame,None)
        return b''.join(entry['parts'][i] for i in range(count))

class VideoReceiver:
    def __init__(self):
        self.lock=threading.RLock();self.stop=threading.Event();self.jpeg=None
        self.last=0;self.last_packet=0;self.sequence=0;self.packets=0;self.assembled=0;self.decoded=0;self.errors=0;self.error='尚未启动视频接收'
        self.width=self.height=0;self.thread=None
    def start(self):
        self.thread=threading.Thread(target=self.run,name='official-udp-video',daemon=True);self.thread.start()
    def snapshot(self):
        with self.lock:
            age=time.monotonic()-self.last if self.last else None
            return {'fresh':age is not None and age<.5,'age_ms':round(age*1000) if age is not None else None,'sequence':self.sequence,'packets':self.packets,'assembled':self.assembled,'decoded':self.decoded,'errors':self.errors,'error':self.error,'width':self.width,'height':self.height,'source':'official client UDP 127.0.0.1:3334 / HEVC'}
    def latest(self):
        with self.lock:return self.jpeg if self.last and time.monotonic()-self.last<.5 else None
    def run(self):
        sock=None
        try:
            import av
            decoder=av.CodecContext.create('hevc','r');decoder.thread_count=2
            assembler=Reassembler()
            sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
            # Deliberately no SO_REUSEADDR: do not steal an existing receiver.
            sock.bind(('127.0.0.1',3334));sock.settimeout(.2)
            with self.lock:self.error='等待官方选手端视频流'
            last_jpeg=0
            while not self.stop.is_set():
                try:packet,peer=sock.recvfrom(2048)
                except socket.timeout:continue
                if peer[0]!='127.0.0.1':continue
                now=time.monotonic()
                with self.lock:self.packets+=1;self.last_packet=now
                payload=assembler.feed(packet,now)
                if payload is None:continue
                with self.lock:self.assembled+=1
                try:frames=decoder.decode(av.Packet(payload))
                except av.error.FFmpegError as exc:
                    with self.lock:self.errors+=1;self.error='等待完整关键帧：'+str(exc)[:100]
                    continue
                for frame in frames:
                    with self.lock:self.decoded+=1
                    if now-last_jpeg<1/24:continue
                    image=frame.to_image();image.thumbnail((960,540))
                    output=io.BytesIO();image.save(output,format='JPEG',quality=75)
                    with self.lock:
                        self.jpeg=output.getvalue();self.last=time.monotonic();self.sequence+=1
                        self.width,self.height=image.size;self.error=''
                    last_jpeg=now
        except Exception as exc:
            with self.lock:self.error=type(exc).__name__+': '+str(exc)
        finally:
            if sock:sock.close()
            with self.lock:self.last=0;self.jpeg=None

