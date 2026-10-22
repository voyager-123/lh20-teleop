"""CAD checks for actual commands, including recovery from existing overlap."""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
from factory_mapping import FactoryMapping
from collision_recovery import RecoveryEnvelope
from collision_check import CollisionChecker


class FactoryAudit:
    def __init__(self,thumb_axes="native"):
        self.mapping=FactoryMapping(thumb_axes)
        self.checker=CollisionChecker(self.mapping.model_path)
        self.cache=OrderedDict()

    def snapshot(self,q,detailed=False):
        key=(detailed,tuple(np.round(q,12)))
        if key in self.cache:
            self.cache.move_to_end(key);return self.cache[key]
        limits=self.checker.violations(q)
        for request in self.checker.gdata.collisionRequests:
            request.enable_contact=True
            request.num_max_contacts=4096 if detailed else 1
        hits=self.checker.check(q)
        depths={};complete=True
        if detailed:
            for pair,result in zip(self.checker.geometry.collisionPairs,self.checker.gdata.collisionResults):
                if not result.isCollision():continue
                contacts=result.getContacts()
                if not contacts or result.numContacts()>=4096:
                    complete=False;break
                names=(self.checker.geometry.geometryObjects[pair.first].name,
                       self.checker.geometry.geometryObjects[pair.second].name)
                depth=max(float(c.penetration_depth) for c in contacts)
                if not np.isfinite(depth) or depth<0:complete=False;break
                depths[names]=depth
        value={'limits':limits,'collisions':hits,'depths':depths,'complete':complete}
        self.cache[key]=value
        if len(self.cache)>128:self.cache.popitem(last=False)
        return value

    def check(self, before, after):
        a=self.checker.expand(self.mapping.decode(before))
        b=self.checker.expand(self.mapping.decode(after))
        count=max(1,int(np.ceil(np.max(np.abs(b-a))/.01)))
        initial=self.snapshot(a)
        if initial['limits']:
            return {'ok':False,'limits':initial['limits'],'collisions':initial['collisions'],'sample':0,'samples':count+1}
        recovery=None
        if initial['collisions']:
            initial=self.snapshot(a,True)
            if not initial['complete']:
                return {'ok':False,'reason':'incomplete_contact_metrics','collisions':initial['collisions'],'sample':0,'samples':count+1}
            recovery=RecoveryEnvelope(initial['depths'])
        for i in range(1,count+1):
            q=a+(b-a)*i/count
            checked=self.snapshot(q,recovery is not None)
            if checked['limits']:
                return {'ok':False,'limits':checked['limits'],'collisions':checked['collisions'],'sample':i,'samples':count+1}
            if recovery is None:
                ok,reason=not checked['collisions'],'new_collision_from_clear_pose'
            elif not checked['complete']:
                ok,reason=False,'incomplete_contact_metrics'
            else:ok,reason=recovery.advance(checked['depths'])
            if not ok:
                return {'ok':False,'reason':reason,'limits':[],'collisions':checked['collisions'],'sample':i,'samples':count+1}
        return {'ok':True,'samples':count+1,'recovery_from_existing_collision':recovery is not None,
                'initial_collision_pairs':initial['collisions'] if recovery is not None else []}


def worker(thumb_axes="native"):
    audit=FactoryAudit(thumb_axes)
    print(json.dumps({'ready':True,'profile_sha256':audit.mapping.sha256,'model_sha256':audit.mapping.model_hash}),flush=True)
    for line in sys.stdin:
        try:
            row=json.loads(line)
            result=audit.check(row['before'],row['after'])
        except Exception as error:
            result={'ok':False,'error':str(error)}
        print(json.dumps(result,allow_nan=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--worker',action='store_true');p.add_argument('--thumb-axes',choices=('native','swapped'),default='native');a=p.parse_args()
    try:
        if a.worker:worker(a.thumb_axes)
    except KeyboardInterrupt:
        pass
