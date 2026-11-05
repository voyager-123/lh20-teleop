"""Side-specific L20-10 factory/model conversion; no device access."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from hand_recording import SLOTS
from measured_mapping import interpolate

ROOT = Path(__file__).resolve().parents[1]
PROFILE_SHA256 = '5bb3f16ed735dfda8f5d52e01429f25044e200d7e858839bd15838843bf9d123'
RIGHT_PROFILE_SHA256 = 'cbbd4c983cf73ba51682fba088cbf65fd32343c566cb8fafc99d02a747898825'
MODEL_ID = 'models/l20_10_left/linkerhand_l20_10_left.urdf'
ACTIVE = tuple(range(11)) + tuple(range(15, 20))
BRIDGE_PATH_POLICY = 'bridge_path_v1'


class FactoryMapping:
    def __init__(self, thumb_axes="native", hand="left"):
        if thumb_axes not in ('native','swapped'):
            raise ValueError('Unknown thumb axis adapter')
        if hand not in ('left','right'):raise ValueError('Unknown hand side')
        self.hand=hand
        self.thumb_axes=thumb_axes
        self.model_id=f'models/l20_10_{hand}/linkerhand_l20_10_{hand}.urdf'
        path = ROOT / f'config/l20_10_{hand}_factory.json'
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        if self.sha256 != (PROFILE_SHA256 if hand=='left' else RIGHT_PROFILE_SHA256):
            raise ValueError(f'{hand} factory mapping differs from the audited profile')
        self.profile = json.loads(path.read_text())
        if self.profile.get('hand')!=hand or self.profile.get('model')!=self.model_id:
            raise ValueError('Factory side/model mismatch')
        self.model_path = ROOT / self.model_id
        self.model_hash = hashlib.sha256(self.model_path.read_bytes()).hexdigest()
        if self.model_hash != self.profile['urdf_sha256']:
            raise ValueError(f'{hand} factory URDF differs from the audited model')
        self.joints = json.loads(json.dumps(self.profile['joints']))
        tree = ET.parse(self.model_path).getroot()
        self.urdf_limits = {j.get('name'): (float(j.find('limit').get('lower')), float(j.find('limit').get('upper')))
                            for j in tree.findall('joint') if j.find('limit') is not None}
        self.velocities = {j.get('name'): float(j.find('limit').get('velocity'))
                           for j in tree.findall('joint') if j.get('name') in SLOTS}
        self.limits = {n: (max(self.urdf_limits[n][0], e['anchors'][0][0]),
                           min(self.urdf_limits[n][1], e['anchors'][-1][0])) for n,e in self.joints.items()}
        for j in tree.findall('joint'):
            m = j.find('mimic')
            if m is not None:
                parent, mult, offset = m.get('joint'), float(m.get('multiplier', 1)), float(m.get('offset', 0))
                if mult <= 0 or parent not in self.limits:
                    raise ValueError('Unsupported factory mimic relation')
                lo, hi = self.limits[parent]
                child_lo, child_hi = self.urdf_limits[j.get('name')]
                self.limits[parent] = max(lo, (child_lo-offset)/mult), min(hi, (child_hi-offset)/mult)
                self.velocities[parent] = min(self.velocities[parent], float(j.find('limit').get('velocity'))/mult)
        self.raw_limits = {}
        for name, e in self.joints.items():
            if e['slot'] != SLOTS[name][0]:
                raise ValueError('Factory joint order mismatch')
            lo, hi = self.limits[name]
            a,b = sorted(interpolate(q,e['anchors']) for q in (lo,hi))
            self.raw_limits[name] = math.ceil(a-1e-9), math.floor(b+1e-9)
        self.factory_sha256=self.sha256
        if thumb_axes=='swapped':
            # Operator A/B adapter only; keep manufacturer evidence untouched.
            a,b='thumb_cmc_roll','thumb_cmc_yaw'
            self.joints[a]['slot'],self.joints[b]['slot']=self.joints[b]['slot'],self.joints[a]['slot']
            self.sha256=hashlib.sha256((self.factory_sha256+':thumb-axes=swapped').encode()).hexdigest()

    def decode(self, raw):
        if len(raw) != 20 or any(type(v) not in (int,float) or not math.isfinite(v) or v!=round(v) or not 0<=v<=255 for v in raw):
            raise ValueError('Expected 20 finite integer protocol counts')
        return {n: interpolate(raw[e['slot']], sorted((u,q) for q,u in e['anchors'])) for n,e in self.joints.items()}

    def encode(self, named):
        if set(named) != set(self.joints):
            raise ValueError('Expected all 16 named factory joints')
        target = [255]*20
        for n,value in named.items():
            lo,hi = self.limits[n]
            if type(value) not in (int,float) or not math.isfinite(value) or not lo-1e-6<=value<=hi+1e-6:
                raise ValueError('Angle outside factory/model limits: '+n)
            raw_lo,raw_hi = self.raw_limits[n]
            target[self.joints[n]['slot']] = min(raw_hi,max(raw_lo,round(interpolate(min(hi,max(lo,value)),self.joints[n]['anchors']))))
        return target

    def step_interval(self, before, after):
        a,b = self.decode(before),self.decode(after)
        return max([1/30]+[abs(b[n]-a[n])/self.velocities[n] for n in self.joints])
