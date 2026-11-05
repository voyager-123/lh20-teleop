"""CAD checks for actual commands, including recovery from existing overlap."""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import sys
import numpy as np
import pinocchio as pin
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'bridge'))
from factory_mapping import FactoryMapping
from collision_recovery import RecoveryEnvelope
from collision_check import CollisionChecker


class FactoryAudit:
    def __init__(self,thumb_axes="native",hand="left",use_broadphase=True):
        self.mapping=FactoryMapping(thumb_axes,hand=hand)
        self.checker=CollisionChecker(self.mapping.model_path)
        self.cache=OrderedDict()
        self.pair_cache=OrderedDict()
        self.use_broadphase=use_broadphase
        # Bounds come from the loaded collision geometry (including mesh scale),
        # not the visual mesh. A transformed local box encloses every triangle.
        lower=[];upper=[]
        for obj in self.checker.geometry.geometryObjects:
            obj.geometry.computeLocalAABB()
            lower.append(np.array(obj.geometry.aabb_local.min_,copy=True))
            upper.append(np.array(obj.geometry.aabb_local.max_,copy=True))
        lower,upper=np.array(lower),np.array(upper)
        self.box_centers=lower/2+upper/2
        self.box_extents=upper/2-lower/2
        self.boxes_valid=bool(np.isfinite(lower).all() and np.isfinite(upper).all() and (upper>=lower).all())
        self.pair_first=np.array([p.first for p in self.checker.geometry.collisionPairs])
        self.pair_second=np.array([p.second for p in self.checker.geometry.collisionPairs])

    def overlapping_pairs(self):
        """Conservative broadphase only; every surviving pair uses exact meshes."""
        c=self.checker
        # A positive query margin could collide even with disjoint mesh bounds.
        # Preserve arbitrary request semantics by falling back to all pairs.
        if not self.use_broadphase or not self.boxes_valid or any(r.security_margin!=0 for r in c.gdata.collisionRequests):
            return range(len(c.geometry.collisionPairs))
        rotations=np.array([p.rotation for p in c.gdata.oMg])
        translations=np.array([p.translation for p in c.gdata.oMg])
        centers=np.einsum('nij,nj->ni',rotations,self.box_centers)+translations
        extents=np.einsum('nij,nj->ni',np.abs(rotations),self.box_extents)+1e-9
        lower,upper=centers-extents,centers+extents
        a,b=self.pair_first,self.pair_second
        overlaps=np.all(lower[a]<=upper[b],axis=1)&np.all(lower[b]<=upper[a],axis=1)
        return np.flatnonzero(overlaps).tolist()

    def pair_result(self,index,detailed):
        c=self.checker
        request=c.gdata.collisionRequests[index]
        request.enable_contact=True
        cap=4096 if detailed else 1
        while True:
            request.num_max_contacts=cap
            collision=pin.computeCollision(c.geometry,c.gdata,index)
            if not collision or not detailed:return collision,None,True
            result=c.gdata.collisionResults[index]
            # A saturated mesh query is truncated, even when its current
            # maximum depth looks valid. Retry before caching any metrics.
            if result.numContacts()>=cap and cap<65536:
                cap*=2
                continue
            depths=[float(contact.penetration_depth) for contact in result.getContacts()]
            known=bool(depths) and result.numContacts()<cap and all(np.isfinite(d) and d>=0 for d in depths)
            return collision,max(depths) if known else None,known

    def snapshot(self,q,detailed=False):
        key=(detailed,tuple(q))
        if key in self.cache:
            self.cache.move_to_end(key);return self.cache[key]
        limits=self.checker.violations(q)
        c=self.checker
        pin.updateGeometryPlacements(c.model,c.data,c.geometry,c.gdata,q)
        placements=[pose.homogeneous.tobytes() for pose in c.gdata.oMg]
        hits=[];depths={};complete=True
        for index in self.overlapping_pairs():
            pair,kind=c.geometry.collisionPairs[index],c.kinds[index]
            # Unchanged geometry pairs have identical contacts. Reuse their exact
            # result, so holding three fingers does not redo expensive mesh work.
            pair_key=(detailed,index,placements[pair.first],placements[pair.second])
            if pair_key in self.pair_cache:
                self.pair_cache.move_to_end(pair_key)
                collision,depth,known=self.pair_cache[pair_key]
            else:
                collision,depth,known=self.pair_result(index,detailed)
                self.pair_cache[pair_key]=(collision,depth,known)
                if len(self.pair_cache)>4096:self.pair_cache.popitem(last=False)
            if collision:
                names=(c.geometry.geometryObjects[pair.first].name,
                       c.geometry.geometryObjects[pair.second].name)
                hits.append({'a':names[0],'b':names[1],'kind':kind})
                if detailed:
                    complete=complete and known
                    if known:depths[names]=depth
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

    def select_target(self,before,hold,requested):
        """Hold implicated fingers near readback, then audit the whole path."""
        checked=self.check(before,requested)
        if checked['ok']:
            return {**checked,'target':list(requested),'held_fingers':[]}
        if checked.get('reason') not in ('existing_collision_deepened','new_collision_pair','new_collision_from_clear_pose'):
            return checked
        fingers={self.checker.finger(hit[side]) for hit in checked.get('collisions',[]) for side in ('a','b')}
        fingers.discard(None)
        if not fingers:return checked
        partial=list(requested)
        for name,entry in self.mapping.joints.items():
            if name.split('_',1)[0] in fingers:
                slot=entry['slot']
                # Quantized readback can differ from the last target by one or
                # two counts. Returning to that old target can deepen contact.
                # Hold the observed position only within that existing tracking
                # tolerance; never rebase a moving/lagging axis by a large step.
                partial[slot]=before[slot] if abs(before[slot]-hold[slot])<=2 else hold[slot]
        if partial==list(requested) or partial==list(hold):return checked
        if self.mapping.step_interval(hold,partial)>self.mapping.step_interval(hold,requested)+1e-9:
            return {'ok':False,'reason':'partial_target_exceeds_slew','blocked_check':checked}
        # Never assume independent fingers are safe: recheck all pairs, limits,
        # sampled path and existing-contact depths for the combined command.
        selected=self.check(before,partial)
        if not selected['ok']:return selected
        return {**selected,'target':partial,'held_fingers':sorted(fingers),'blocked_check':checked,
                'feedback_hold_slots':[i for i in range(20) if partial[i]!=requested[i] and partial[i]!=hold[i]]}


def worker(thumb_axes="native",hand="left"):
    audit=FactoryAudit(thumb_axes,hand)
    print(json.dumps({'ready':True,'profile_sha256':audit.mapping.sha256,'model_sha256':audit.mapping.model_hash}),flush=True)
    for line in sys.stdin:
        try:
            row=json.loads(line)
            result=(audit.select_target(row['before'],row['hold'],row['after']) if 'hold' in row
                    else audit.check(row['before'],row['after']))
        except Exception as error:
            result={'ok':False,'error':str(error)}
        print(json.dumps(result,allow_nan=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--worker',action='store_true');p.add_argument('--hand',choices=('left','right'),default='left');p.add_argument('--thumb-axes',choices=('native','swapped'),default='native');a=p.parse_args()
    try:
        if a.worker:worker(a.thumb_axes,a.hand)
    except KeyboardInterrupt:
        pass
