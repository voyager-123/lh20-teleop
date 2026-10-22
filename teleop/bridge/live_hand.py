#!/usr/bin/env python3
from runtime_paths import vision_python as resolve_vision_python
"""LEFT L20-10 native factory full-hand following with quantized CAD checks."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time

from factory_mapping import FactoryMapping, ACTIVE, MODEL_ID
from tracking_governor import LatencyAwareGovernor
from live_index import SOURCE_COMMIT, VisionReceiver, default_socket
from probe_index_joint import validate_positions, feedback_errors, topic_names, vision_subprocess_env

ROOT=Path(__file__).resolve().parents[1]


class HandController:
    def __init__(self, baseline, mapping=None):
        self.mapping=mapping or FactoryMapping()
        self.last_command=validate_positions(baseline)
        for i in (11,12,13,14):self.last_command[i]=255
        self.mapping.encode(self.mapping.decode(self.last_command))  # Reject out-of-model starting poses.
        self.desired=self.last_command.copy()
        self.active=False
        self.state='WAITING_FOR_LEFT_HAND'
        self.session=None
        self.seq=-1
        self.epoch=None
        self.last_input=-math.inf
        self.last_step=-math.inf
        self.samples=deque(maxlen=3)

    def pause(self,reason):
        self.active=False;self.samples.clear();self.state=reason

    def accept(self,p,now):
        try:
            if not isinstance(p,dict) or p.get('schema')!='lh20.retarget.v1' or p.get('hand')!='left' or \
                    p.get('scope')!='hand' or p.get('model')!=MODEL_ID or \
                    p.get('model_sha256')!=self.mapping.model_hash or p.get('source_commit')!=SOURCE_COMMIT or p.get('thumb_axes','native')!=self.mapping.thumb_axes:
                raise ValueError('Wrong side/model/source/scope')
            session,seq,epoch=p.get('stream_session'),p.get('seq'),p.get('calibration_epoch')
            if not isinstance(session,str) or len(session)!=32 or type(seq) is not int or seq<0 or type(epoch) is not int or epoch<0:
                raise ValueError('Invalid session/sequence')
            stamp=p.get('captured_monotonic_ns')
            if type(stamp) is not int or not -.02<=now-stamp/1e9<=.25:
                raise ValueError('Stale/future vision frame')
            if session!=self.session:
                self.pause('NEW_LEFT_VISION_SESSION');self.session=session;self.seq=-1;self.epoch=epoch
            if seq<=self.seq:raise ValueError('Duplicate/out-of-order vision frame')
            self.seq=seq
            if self.epoch!=epoch:
                self.pause('VISION_RESTARTED');self.epoch=epoch
            if type(p.get('valid')) is not bool or type(p.get('operator_enabled')) is not bool:
                raise ValueError('Invalid enable/valid flags')
            if not p['valid'] or not p['operator_enabled'] or p.get('auto_follow') is not True or p.get('cad_collision_hits'):
                self.pause('WAITING_FOR_VALID_LEFT_HAND');return
            names,angles=p.get('joint_names'),p.get('position_rad')
            if not isinstance(names,list) or not isinstance(angles,list) or len(names)!=16 or len(angles)!=16 or \
                    any(not isinstance(n,str) for n in names) or len(set(names))!=16 or set(names)!=set(self.mapping.joints):
                raise ValueError('Expected 16 named factory joints')
            desired=self.mapping.encode(dict(zip(names,angles)))
            if p.get('factory_profile_sha256')!=self.mapping.sha256 or p.get('motor_position_raw')!=desired:
                raise ValueError('Factory target/profile mismatch')
            self.last_input=stamp/1e9
            self.samples.append(stamp/1e9)
            if len(self.samples)<3 or self.samples[-1]-self.samples[0]<.06:
                self.active=False;self.state='DETECTING_LEFT_HAND';return
            self.desired=desired;self.active=True;self.state='AUTO_FOLLOWING_FACTORY_LEFT_HAND'
        except (ValueError,KeyError,TypeError) as e:
            self.pause('REJECTED_FRAME: '+str(e))

    def candidate(self,now,desired_override=None):
        if now-self.last_input>.25:self.pause('WAITING_FOR_LEFT_HAND')
        if not self.active:return None
        elapsed=1/30 if not math.isfinite(self.last_step) else now-self.last_step
        if elapsed<1/30:return None
        # Velocity in URDF radians/s, rather than one raw count for every axis.
        # Cap elapsed after a pause so reacquisition cannot cause a large jump.
        dt=min(.05,elapsed)
        target=self.last_command.copy()
        for name,e in self.mapping.joints.items():
            i=e['slot'];anchors=e['anchors']
            quantum=abs((anchors[-1][0]-anchors[0][0])/(anchors[-1][1]-anchors[0][1]))
            budget=math.floor(self.mapping.velocities[name]*dt/quantum+1e-9)
            desired=self.desired if desired_override is None else desired_override
            target[i]+=max(-budget,min(budget,desired[i]-target[i]))
        return target

    def commit(self,target,now):
        elapsed=1/30 if not math.isfinite(self.last_step) else now-self.last_step
        if self.mapping.step_interval(self.last_command,target)>min(.05,elapsed)+1e-9:
            raise ValueError('Factory command exceeds model velocity limit')
        self.last_command=target.copy();self.last_step=now


class CadWorker:
    def __init__(self,thumb_axes='native'):
        python=resolve_vision_python()
        self.process=subprocess.Popen([str(python),'-E','-s','-u',str(ROOT/'vision/factory_hand_audit.py'),'--worker','--thumb-axes',thumb_axes],
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1,
                                      env=vision_subprocess_env(python))
        try:
            ready=self.read(15)
            m=FactoryMapping(thumb_axes)
            if ready.get('ready') is not True or ready.get('profile_sha256')!=m.sha256 or ready.get('model_sha256')!=m.model_hash:
                raise RuntimeError('Factory CAD worker/model mismatch')
        except BaseException:
            self.close();raise

    def read(self,timeout,pump=None):
        deadline=time.monotonic()+timeout
        while not select.select([self.process.stdout],[],[],min(.01,max(0,deadline-time.monotonic())))[0]:
            if pump is not None:pump()
            if time.monotonic()>=deadline:raise RuntimeError('Factory CAD worker response timeout')
        line=self.process.stdout.readline()
        if not line:raise RuntimeError('Factory CAD worker exited')
        return json.loads(line)

    def check(self,before,after,pump=None):
        self.process.stdin.write(json.dumps({'before':before,'after':after},allow_nan=False)+'\n')
        self.process.stdin.flush()
        return self.read(1.,pump)

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)
        for handle in (self.process.stdin,self.process.stdout):
            if handle:handle.close()


def run(args):
    if args.hand!='left' or not args.auto_follow:raise ValueError('Use the automatic left full-hand entry')
    if not 1<=args.duration<=3600:raise ValueError('Invalid duration')
    mapping=FactoryMapping(getattr(args,'thumb_axes','native'))
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    rclpy.init();node=rclpy.create_node('lh20_left_factory_hand')
    topics=topic_names('left')
    latest={'q':None,'state_at':0.,'info':None,'info_at':0.}
    recent=deque(maxlen=5)
    publisher=receiver=cad=None
    path=ROOT/'data'/f'live_hand_{time.time_ns()}.jsonl';path.parent.mkdir(exist_ok=True)
    log=path.open('x');last_fault=None
    def record(kind,**values):
        log.write(json.dumps({'kind':kind,'monotonic':time.monotonic(),**values},allow_nan=False)+'\n');log.flush()
    def state(msg):
        latest['state_at']=time.monotonic()
        try:latest['q']=validate_positions(msg.position);recent.append(latest['q'])
        except ValueError:latest['q']=None;recent.clear()
    def info(msg):
        nonlocal last_fault
        latest['info_at']=time.monotonic()
        try:
            latest['info']=json.loads(msg.data)
            fault=latest['info'].get('fault') if isinstance(latest['info'],dict) else None
            if fault!=last_fault:record('raw_fault_change',fault=fault);last_fault=fault
        except ValueError:latest['info']=None
    def guard():
        errors=feedback_errors(latest,time.monotonic(),ignore_128=args.fault_policy in ('ignore-128','ignore-64-128'),ignore_64=args.fault_policy=='ignore-64-128')
        if errors:raise RuntimeError('; '.join(errors))
        if latest['info'].get('version')!=mapping.profile['firmware_version']:
            raise RuntimeError('Firmware differs from the factory-control configuration')
        if any(node.count_publishers(t)!=c for t,c in ((topics['state'],1),(topics['info'],1),
                (topics['full'],int(publisher is not None)),(topics['index'],0),('/cb_hand_setting_cmd',0))):
            raise RuntimeError('Duplicate/missing SDK or competing controller')
        if node.count_subscribers(topics['full'])!=1 or (publisher is not None and publisher.get_subscription_count()!=1):
            raise RuntimeError('Expected exactly one SDK full-hand subscriber')
    subs=[node.create_subscription(JointState,topics['state'],state,1),node.create_subscription(String,topics['info'],info,1)]
    try:
        print('(lh20_ros2) 等待左手稳定反馈；准备厂家整手映射与 CAD 检查。',flush=True)
        end=time.monotonic()+10
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if len(recent)==5 and all(max(q[i] for q in recent)-min(q[i] for q in recent)<=2 for i in ACTIVE) and not feedback_errors(latest,time.monotonic(),ignore_128=args.fault_policy in ('ignore-128','ignore-64-128'),ignore_64=args.fault_policy=='ignore-64-128'):break
        else:raise RuntimeError('No stable fresh left-hand feedback')
        guard()
        engine=HandController(latest['q'],mapping)
        governor=LatencyAwareGovernor(mapping,isolate_stalls=True)
        cad=CadWorker(mapping.thumb_axes)
        initial=cad.check(engine.last_command,engine.last_command)
        if not initial['ok']:raise RuntimeError('Starting pose failed factory model check: '+json.dumps(initial))
        if initial.get('recovery_from_existing_collision'):
            print('(lh20_ros2) 模型起始姿态存在接触：允许保持或分离，阻止加深及新增碰撞。',flush=True)
        # CAD-worker startup can take seconds. Refresh before authorizing output.
        recent.clear();end=time.monotonic()+2
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.03)
            if len(recent)==5:break
        guard()
        if len(recent)!=5 or any(abs(q[i]-engine.last_command[i])>2 for q in recent for i in ACTIVE):
            raise RuntimeError('Starting pose moved during factory CAD initialization')
        engine=HandController(latest['q'],mapping)
        if not cad.check(engine.last_command,engine.last_command)['ok']:raise RuntimeError('Refreshed starting pose failed CAD check')
        if args.execute:
            publisher=node.create_publisher(JointState,topics['full'],1)
            end=time.monotonic()+3
            while publisher.get_subscription_count()==0 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        guard()
        receiver=VisionReceiver(args.socket or default_socket('left'))
        record('start',execute=args.execute,baseline=engine.last_command,profile_sha256=mapping.sha256,
               model_sha256=mapping.model_hash,source=mapping.profile['source'],fault_policy=args.fault_policy,
               thumb_axes=mapping.thumb_axes,tracking_stall_action='hold_affected_joint',
               mapping='manufacturer_left_g20_native_model',physical_angle_calibration=False,
               feedback_pacing={'mode':'latency_aware','max_lead_by_slot':governor.lead_limits,
                                'history_seconds':governor.latency_seconds,'stall_seconds':governor.stall_seconds},
               ignored_fault_codes=[64,128] if args.fault_policy=='ignore-64-128' else [128] if args.fault_policy=='ignore-128' else [],
               slew_velocity_rad_s=mapping.velocities)
        print('(lh20_ros2) '+('整手控制已接通' if args.execute else '仅控制桥预览')+'：16 个独立关节；按模型角速度限速，至多 30 Hz；只跟随最新目标；Q 退出。',flush=True)
        end=time.monotonic()+args.duration;last_state=None;next_print=0.;next_wait_print=0.;cad_waiting=False;next_block_log=0.;last_block_key=None
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01);guard()
            for _ in range(32):
                try:p=receiver.receive()
                except (ValueError,UnicodeError):engine.pause('REJECTED_MALFORMED_JSON');break
                if p is None:break
                engine.accept(p,time.monotonic())
            now=time.monotonic()
            if engine.active and publisher is not None:
                governor.observe(latest['q'],engine.last_command,now)
            else:
                governor.reset()
            adjusted=governor.adapt_desired(engine.desired)
            target=engine.candidate(now,adjusted)
            for event in governor.drain_events():
                record('joint_isolation',**event)
                print('(lh20_ros2) 单关节'+('暂停' if event['event']=='held' else '恢复')+'：'+event['joint']+'；其他关节继续。',flush=True)
            if target is not None:
                if publisher is not None:
                    target,held=governor.constrain(engine.last_command,target,latest['q'])
                    if held and now>=next_wait_print:
                        record('feedback_wait',joints=held)
                        print('(lh20_ros2) 等待关节反馈追上，保持滞后轴目标：'+', '.join(item['joint'] for item in held),flush=True)
                        next_wait_print=now+1
                observed=latest['q'].copy()
                cad_began=time.monotonic()
                checked=cad.check(observed,target,pump=lambda: rclpy.spin_once(node,timeout_sec=0.))
                cad_ms=(time.monotonic()-cad_began)*1000
                if not checked['ok']:
                    cad_waiting=True
                    block_key=json.dumps(checked.get('collisions',[]),sort_keys=True)+str(checked.get('reason'))
                    if now>=next_block_log or block_key!=last_block_key:
                        record('cad_blocked',check=checked)
                        next_block_log=now+1;last_block_key=block_key
                else:
                    cad_waiting=False
                    rclpy.spin_once(node,timeout_sec=0.);guard()
                    now=time.monotonic()
                    if now-engine.last_input>.25:engine.pause('WAITING_FOR_LEFT_HAND')
                    elif any(abs(latest['q'][i]-observed[i])>2 for i in ACTIVE):
                        engine.pause('FEEDBACK_MOVED_DURING_CAD_CHECK')
                    else:
                        if publisher is not None:
                            msg=JointState();msg.header.stamp=node.get_clock().now().to_msg()
                            msg.name=[next(n for n,e in mapping.joints.items() if e['slot']==i) if i in ACTIVE else 'reserved' for i in range(20)]
                            msg.position=[float(v) for v in target];publisher.publish(msg)
                        engine.commit(target,now)
                        record('command' if publisher is not None else 'preview',target=target,desired=engine.desired,feedback=latest['q'],cad_samples=checked['samples'],
                               vision_age_ms=(now-engine.last_input)*1000,cad_ms=cad_ms,
                               cad_recovery=checked.get('recovery_from_existing_collision',False))
                        if now>=next_print:print('(lh20_ros2) root='+str(target[:5])+' tip='+str(target[15:]),flush=True);next_print=now+1
            display_state='CAD_WAITING_FOR_SEPARATING_MOTION' if cad_waiting and engine.active else engine.state
            if display_state!=last_state:
                last_state=display_state;record('state',state=last_state);print('(lh20_ros2) '+last_state,flush=True)
    except Exception as error:
        record('error',error=str(error),last_feedback=latest['q'])
        raise
    finally:
        record('stop',last_feedback=latest['q'],auto_return=False)
        if receiver is not None:receiver.close()
        if cad is not None:cad.close()
        log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        print('(lh20_ros2) 停止整手目标，不自动回原位。Log: '+str(path),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hand',choices=('left',),required=True);p.add_argument('--auto-follow',action='store_true')
    p.add_argument('--execute',action='store_true');p.add_argument('--duration',type=float,default=3600)
    p.add_argument('--thumb-axes',choices=('native','swapped'),default='native')
    p.add_argument('--socket',type=Path);p.add_argument('--fault-policy',choices=('stop','ignore-128','ignore-64-128'),default='stop')
    args=p.parse_args()
    try:run(args)
    except KeyboardInterrupt:return 130
    except (RuntimeError,ValueError,OSError,subprocess.SubprocessError) as error:
        print('(lh20_ros2) STOPPED: '+str(error),flush=True);return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
