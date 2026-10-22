"""Feedback pacing for slow/asynchronous hardware; no ROS or device access."""
from factory_mapping import ACTIVE


class FeedbackGovernor:
    def __init__(self, mapping, max_lead=4, stall_seconds=2.5):
        self.mapping=mapping
        self.max_lead=max_lead
        self.stall_seconds=stall_seconds
        self.names={e['slot']:n for n,e in mapping.joints.items()}
        self.progress={}
        self.hard_since={}

    def reset(self):
        self.progress.clear();self.hard_since.clear()

    def constrain(self, previous, candidate, feedback):
        """Hold lagging axes, without jumping a target back toward the sensor."""
        target=candidate.copy();held=[]
        for i in ACTIVE:
            old_error=abs(previous[i]-feedback[i]);new_error=abs(candidate[i]-feedback[i])
            limit=getattr(self,'lead_limits',{}).get(i,self.max_lead)
            if new_error>limit and new_error>=old_error:
                target[i]=previous[i]
                held.append({'slot':i,'joint':self.names[i],'target':previous[i],
                             'feedback':feedback[i],'error_counts':previous[i]-feedback[i]})
        return target,held

    def observe(self, feedback, target, now):
        """Keep the original hard error bound; stop sustained per-axis stalls too."""
        for i in ACTIVE:
            error=target[i]-feedback[i]
            if abs(error)>8:
                self.hard_since.setdefault(i,now)
                if now-self.hard_since[i]>.75:
                    raise RuntimeError(f'关节 {self.names[i]} (slot {i}) 持续偏差超过 8 计数：目标 {target[i]}，反馈 {feedback[i]}')
            else:self.hard_since.pop(i,None)
            if abs(error)<=3:
                self.progress.pop(i,None)
                continue
            direction=1 if error>0 else -1
            old=self.progress.get(i)
            if old is None or old['direction']!=direction:
                self.progress[i]={'direction':direction,'best_feedback':feedback[i],'at':now}
                continue
            if (feedback[i]-old['best_feedback'])*direction>=1:
                old.update(best_feedback=feedback[i],at=now)
            elif now-old['at']>=self.stall_seconds:
                raise RuntimeError(f'关节 {self.names[i]} (slot {i}) 等待 {self.stall_seconds:g} 秒仍未追上且无位置进展：'
                                   f'目标 {target[i]}，反馈 {feedback[i]}；请查看该关节是否受阻或未响应')


class LatencyAwareGovernor(FeedbackGovernor):
    """Bound outstanding travel in seconds of motion, and compare delayed readback
    with recent issued commands. Progressing motors are not treated as stalled.
    """
    def __init__(self,mapping,latency_seconds=.25,stall_seconds=2.5,isolate_stalls=False):
        from collections import deque
        import math
        super().__init__(mapping,stall_seconds=stall_seconds)
        self.isolate_stalls=isolate_stalls
        self.isolated={}
        self.events=[]
        self.latency_seconds=latency_seconds
        self.history=deque()
        self.lead_limits={}
        for name,e in mapping.joints.items():
            points=e['anchors']
            quantum=abs((points[-1][0]-points[0][0])/(points[-1][1]-points[0][1]))
            self.lead_limits[e['slot']]=math.ceil(mapping.velocities[name]*(latency_seconds+.05)/quantum)+3

    def reset(self):
        super().reset();self.history.clear()

    def observe(self,feedback,target,now):
        self.history.append((now,target.copy()))
        cutoff=now-self.latency_seconds
        while len(self.history)>1 and self.history[1][0]<=cutoff:self.history.popleft()
        for i in ACTIVE:
            if i in self.isolated:continue
            error=target[i]-feedback[i]
            progressed=False
            if abs(error)<=3:
                self.progress.pop(i,None)
                self.hard_since.pop(i,None)
                continue
            direction=1 if error>0 else -1
            old=self.progress.get(i)
            if old is None or old['direction']!=direction:
                old={'direction':direction,'best_feedback':feedback[i],'at':now}
                self.progress[i]=old
            elif (feedback[i]-old['best_feedback'])*direction>=1:
                old.update(best_feedback=feedback[i],at=now);progressed=True
            if now-old['at']>=self.stall_seconds:
                if self.isolate_stalls:
                    self.hold_joint(i,feedback[i],target[i],'no_progress')
                    continue
                raise RuntimeError(f'关节 {self.names[i]} (slot {i}) 等待 {self.stall_seconds:g} 秒仍无位置进展：目标 {target[i]}，反馈 {feedback[i]}')
            lo=min(q[i] for _,q in self.history);hi=max(q[i] for _,q in self.history)
            distance=max(lo-feedback[i],feedback[i]-hi,0)
            if distance>8:
                self.hard_since.setdefault(i,now)
                # This is an obstruction check, not a comparison against a
                # newer target the motor has not had time to reach.
                if progressed:self.hard_since[i]=now
                if now-max(self.hard_since[i],old['at'])>.75:
                    if self.isolate_stalls:
                        self.hold_joint(i,feedback[i],target[i],'off_recent_path')
                        continue
                    raise RuntimeError(f'关节 {self.names[i]} (slot {i}) 偏离近期命令轨迹且无进展：目标 {target[i]}，反馈 {feedback[i]}')
            else:self.hard_since.pop(i,None)


    def hold_joint(self,slot,feedback,target,reason):
        self.isolated[slot]={'hold_raw':feedback,'direction':1 if target>feedback else -1}
        self.progress.pop(slot,None);self.hard_since.pop(slot,None)
        self.events.append({'event':'held','slot':slot,'joint':self.names[slot],
                            'hold_raw':feedback,'previous_target':target,'reason':reason})

    def adapt_desired(self,desired):
        adjusted=desired.copy()
        for i,item in list(self.isolated.items()):
            # Release only when the operator requests motion away from the
            # blocked direction. Never automatically retry the same push.
            if (desired[i]-item['hold_raw'])*item['direction']<=-3:
                self.events.append({'event':'released','slot':i,'joint':self.names[i]})
                del self.isolated[i]
            else:
                adjusted[i]=item['hold_raw']
        return adjusted

    def drain_events(self):
        events,self.events=self.events,[]
        return events
