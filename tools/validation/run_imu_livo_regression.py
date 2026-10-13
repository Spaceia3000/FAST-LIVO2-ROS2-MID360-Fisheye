#!/usr/bin/env python3
"""Bounded full UGV10 native LIVO regression; publication/finite-state, not accuracy."""
import argparse, csv, hashlib, importlib.util, json, math, os, pathlib, shutil
import signal, subprocess, time, traceback
import numpy as np

def load(path, name):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--project-source',type=pathlib.Path,required=True)
    ap.add_argument('--output',type=pathlib.Path,required=True)
    ap.add_argument('--domain',type=int,required=True)
    args=ap.parse_args()
    if str(args.domain)!=os.environ.get('ROS_DOMAIN_ID'): raise RuntimeError('ROS domain env mismatch')
    if args.output.exists(): raise RuntimeError('Refuse existing output')
    source=pathlib.Path('/home/breidy/Documents/ham_slam_ws/validation_sources/FAST-LIVO2-imu-fix')
    overlay=pathlib.Path('/home/breidy/Documents/fast_livo2_worktrees/imu_validation_20261013/mapping_overlay/install/fast_livo')
    from ament_index_python.packages import get_package_prefix
    if pathlib.Path(get_package_prefix('fast_livo')).resolve()!=overlay.resolve(): raise RuntimeError('Wrong FAST overlay')
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()!='b8075486fa8fef16254a30b1671df9cf146cce32': raise RuntimeError('Wrong FAST source commit')
    # Refuse another process in our selected domain. Reading /proc does not stop any process.
    occupied=[]
    for proc in pathlib.Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) in (os.getpid(),os.getppid()): continue
        try:
            env=(proc/'environ').read_bytes().split(b'\0')
            if ('ROS_DOMAIN_ID='+str(args.domain)).encode() in env: occupied.append(int(proc.name))
        except (FileNotFoundError,ProcessLookupError,PermissionError): pass
    if occupied: raise RuntimeError('Domain occupied: '+str(occupied))
    harness=args.project_source/'tools/validation/validate_split_launches_host.py'
    base=load(harness,'split_validation')
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosbag2_interfaces.srv import Resume
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, PointCloud2
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry, Path
    from rtabmap_msgs.msg import MapGraph
    from rclpy.serialization import serialize_message
    case=base.Case(args.output,dict(os.environ))
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    clock_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    counts={};first={};last={};bad_odom=0;bad_metric_clouds=0;metric_points=0;rgb_nonempty=0;rgb_valid=0;image_nonempty=0
    rgb_colors=set();latest_graph=None;latest_dense=None;odom_seen=set();odom_sequence=[];frames={};duplicates=0;backwards=0;prev=None
    rows=(args.output/'odom.csv').open('w');writer=csv.writer(rows)
    writer.writerow(['stamp_ns','x','y','z','qx','qy','qz','qw'])
    def stamp(msg): return int(msg.header.stamp.sec)*1000000000+int(msg.header.stamp.nanosec)
    def seen(topic,msg):
        counts[topic]=counts.get(topic,0)+1
        t=stamp(msg) if hasattr(msg,'header') else int(msg.clock.sec)*1000000000+int(msg.clock.nanosec)
        first.setdefault(topic,t);last[topic]=t
        if hasattr(msg,'header'):frames.setdefault(topic,set()).add(msg.header.frame_id)
    def input_cb(topic): return lambda msg:seen(topic,msg)
    def finite_pose(p):
        return all(math.isfinite(x) for x in (p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w))
    def odom(msg):
        nonlocal bad_odom,duplicates,backwards,prev
        seen('/aft_mapped_to_init',msg);t=stamp(msg);p=msg.pose.pose
        bad_odom+=int(not finite_pose(p) or not all(math.isfinite(x) for x in msg.pose.covariance))
        duplicates+=int(t in odom_seen);backwards+=int(prev is not None and t<prev);prev=t;odom_seen.add(t);odom_sequence.append(t)
        writer.writerow([t,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    def rgb(msg):
        nonlocal rgb_nonempty,rgb_valid
        seen('/cloud_registered',msg);fields={f.name:f for f in msg.fields}
        valid=msg.width*msg.height>0 and bool(msg.data)
        rgb_nonempty+=int(valid)
        color=fields.get('rgb',fields.get('rgba'))
        valid=valid and {'x','y','z'}.issubset(fields) and color is not None and color.datatype in (6,7)
        rgb_valid+=int(valid)
        if valid and len(rgb_colors)<256:
            data=memoryview(msg.data)
            for i in range(min(8,msg.width)):
                at=i*msg.point_step+color.offset
                if at+4<=len(data): rgb_colors.add(int.from_bytes(data[at:at+4],'big' if msg.is_bigendian else 'little') & 0xffffff)
    def metric(msg):
        nonlocal bad_metric_clouds,metric_points
        seen('/cloud_registered_metric',msg)
        fields={f.name:f for f in msg.fields}
        try:
            endian='>' if msg.is_bigendian else '<'
            kinds={7:'f4',8:'f8'}
            dtype=np.dtype({'names':['x','y','z'], 'formats':[endian+kinds[fields[k].datatype] for k in ['x','y','z']],
                            'offsets':[fields[k].offset for k in ['x','y','z']], 'itemsize':msg.point_step})
            data=np.ndarray((msg.height,msg.width),buffer=msg.data,dtype=dtype,strides=(msg.row_step,msg.point_step))
            good=msg.width*msg.height>0 and all(np.isfinite(data[k]).all() for k in ['x','y','z'])
            metric_points+=msg.width*msg.height
        except (KeyError,ValueError,TypeError):good=False
        bad_metric_clouds+=int(not good)
    def image(msg):
        nonlocal image_nonempty
        seen('/rgb_img',msg);image_nonempty+=int(msg.width*msg.height>0 and bool(msg.data))
    def dense(msg):
        nonlocal latest_dense
        seen('/fast_livo2_rtabmap/trajectory_dense',msg);latest_dense=msg
    def graph(msg):
        nonlocal latest_graph
        seen('/rtabmap/mapGraph',msg);latest_graph=msg
    inputs={'/livox/lidar':(CustomMsg,1899),'/livox/imu':(Imu,37984),'/camera/camera/color/image_raw':(Image,4502)}
    subscriptions=[case.node.create_subscription(typ,t,input_cb(t),qos) for t,(typ,_) in inputs.items()]
    subscriptions += [
        case.node.create_subscription(Odometry,'/aft_mapped_to_init',odom,qos),
        case.node.create_subscription(PointCloud2,'/cloud_registered',rgb,qos),
        case.node.create_subscription(PointCloud2,'/cloud_registered_metric',metric,qos),
        case.node.create_subscription(Image,'/rgb_img',image,qos),
        case.node.create_subscription(Path,'/fast_livo2_rtabmap/trajectory_dense',dense,qos),
        case.node.create_subscription(MapGraph,'/rtabmap/mapGraph',graph,qos),
        case.node.create_subscription(Clock,'/clock',input_cb('/clock'),clock_qos)]
    outcome={'passed':False,'scope':'full UGV10 LIVO publication/finite-state regression; no GT or causal closure-accuracy claim',
             'source_commit':'b8075486fa8fef16254a30b1671df9cf146cce32','frontend_prefix':get_package_prefix('fast_livo'),'backend_prefix':get_package_prefix('fast_livo2_rtabmap_core'),'profile_sha256':hashlib.sha256((source/'config/ugv_v1_livo.yaml').read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),'harness_sha256':hashlib.sha256(harness.read_bytes()).hexdigest(),
             'input_camera_ends_before_lidar_s':39.700727824,'input_capture_qos':'RELIABLE depth100; exact receipt counts are a gate; clock BEST_EFFORT depth10'}
    started=time.monotonic()
    try:
        case.start('frontend',['ros2','launch','fast_livo','s1_fast_golden.launch.py','mode:=livo','use_rviz:=false','use_sim_time:=true'])
        case.start('backend',['ros2','launch','fast_livo2_rtabmap_bringup','backend.launch.py','launch_rviz:=false','use_sim_time:=true','database_path:='+str(args.output/'rtabmap.db')])
        case.check_nodes()
        case.audit_maps()
        expected_imu=overlay.parent.parent/'build/fast_livo/libimu_proc.so'
        actual_imus={path for paths in case.result['loaded_paths'].values() for path in paths if pathlib.Path(path).name=='libimu_proc.so'}
        if actual_imus!={str(expected_imu)}:raise RuntimeError('Wrong loaded IMU library: '+str(actual_imus))
        outcome['loaded_imu_library']={'path':str(expected_imu),'sha256':hashlib.sha256(expected_imu.read_bytes()).hexdigest()}
        player=case.start('player',['ros2','bag','play','/home/breidy/Documents/datasets/ugv_fast_livo2/ugv_custom_fastlivo2_10',
            '--rate','0.5','--clock','40','--start-paused','--disable-keyboard-controls','--topics',*inputs])
        end=time.monotonic()+45;ready=False;endpoints={}
        while time.monotonic()<end:
            case.spin(.2)
            endpoints={t:{'publishers':[x.node_name for x in case.node.get_publishers_info_by_topic(t)],
                          'subscribers':[x.node_name for x in case.node.get_subscriptions_info_by_topic(t)]} for t in inputs}
            ready=all('rosbag2_player' in e['publishers'] and 'laserMapping' in e['subscribers'] and 'modular_supervisor_probe' in e['subscribers'] for e in endpoints.values())
            if ready:break
        outcome['input_endpoints_before_resume']=endpoints
        observed_topics=[*inputs,'/aft_mapped_to_init','/cloud_registered','/cloud_registered_metric','/rgb_img','/rtabmap/mapGraph','/fast_livo2_rtabmap/trajectory_dense']
        offered={t:[{'name':x.node_name,'reliability':int(x.qos_profile.reliability),'durability':int(x.qos_profile.durability),'depth':x.qos_profile.depth} for x in case.node.get_publishers_info_by_topic(t)] for t in observed_topics}
        outcome['offered_qos_before_resume']=offered
        if any(not records or any(x['reliability']!=int(ReliabilityPolicy.RELIABLE) for x in records) for records in offered.values()):raise RuntimeError('Reliable probe requires reliable offered publishers')
        if not ready:raise RuntimeError('Native sensor endpoint matching timeout')
        clock=case.node.get_publishers_info_by_topic('/clock');outcome['clock_publishers_before_resume']=[x.node_name for x in clock]
        if len(clock)!=1 or clock[0].node_name!='rosbag2_player':raise RuntimeError('Clock source must be unique player')
        resume=case.node.create_client(Resume,'/rosbag2_player/resume')
        if not resume.wait_for_service(timeout_sec=10):raise RuntimeError('Resume unavailable')
        fut=resume.call_async(Resume.Request());end=time.monotonic()+10
        while not fut.done() and time.monotonic()<end:case.spin(.1)
        if not fut.done() or fut.exception():raise RuntimeError('Resume failed')
        case.node.destroy_client(resume)
        deadline=time.monotonic()+450; peak_rss=0
        while player.poll() is None:
            case.spin(.1)
            if time.monotonic()>deadline:raise RuntimeError('Playback wall timeout450s')
            if shutil.disk_usage(args.output).free<5*1024**3:raise RuntimeError('Disk safety reserve below5GiB')
            rss=0
            for name,p,_ in case.processes:
                if name!='player' and p.poll() is not None:raise RuntimeError(name+' exited during replay')
            if int(time.monotonic())%5==0:
                groups={p.pid for _,p,_ in case.processes}
                for proc in pathlib.Path('/proc').iterdir():
                    if not proc.name.isdigit():continue
                    try:
                        if os.getpgid(int(proc.name)) in groups:
                            for line in (proc/'status').read_text().splitlines():
                                if line.startswith('VmRSS:'):rss+=int(line.split()[1])*1024
                    except (FileNotFoundError,ProcessLookupError,PermissionError):pass
                peak_rss=max(peak_rss,rss)
                if rss>32*1024**3:raise RuntimeError('Owned RSS above32GiB')
        if player.returncode!=0:raise RuntimeError('Player exit '+str(player.returncode))
        case.spin(15)
        case.audit_maps()
        outcome['peak_owned_rss_bytes']=peak_rss
        missing=[t for t,(_,expected) in inputs.items() if counts.get(t)!=expected]
        if missing:raise RuntimeError('Incomplete raw input observer counts: '+str(missing))
        required=['/aft_mapped_to_init','/cloud_registered_metric','/cloud_registered','/rgb_img','/fast_livo2_rtabmap/trajectory_dense','/rtabmap/mapGraph']
        if any(counts.get(t,0)==0 for t in required):raise RuntimeError('Required output absent')
        if bad_odom or duplicates or backwards:raise RuntimeError('Invalid/nonmonotonic/duplicate odometry')
        if bad_metric_clouds:raise RuntimeError('Malformed/nonfinite/empty metric cloud')
        if not(rgb_nonempty and rgb_valid and image_nonempty and len(rgb_colors)>1):raise RuntimeError('RGB/image publication gate failed')
        if latest_dense is None or not latest_dense.poses or not all(finite_pose(p.pose) for p in latest_dense.poses):raise RuntimeError('Invalid final dense trajectory')
        if last['/aft_mapped_to_init']<first['/camera/camera/color/image_raw']+145000000000:raise RuntimeError('LIVO failed to reach camera horizon')
        expected_frames={'/aft_mapped_to_init':'camera_init','/cloud_registered_metric':'camera_init','/cloud_registered':'camera_init','/fast_livo2_rtabmap/trajectory_dense':'map'}
        for topic,frame in expected_frames.items():
            if frames.get(topic)!={frame}:raise RuntimeError('Unexpected frame '+topic+': '+str(frames.get(topic)))
        dense_stamps=[stamp(p) for p in latest_dense.poses]
        if dense_stamps!=odom_sequence:raise RuntimeError('Final dense timestamps do not equal entire observed odometry sequence')
        outcome['final_dense_exact_odom_sequence']=True
        unique_links={(min(x.from_id,x.to_id),max(x.from_id,x.to_id),x.type) for x in latest_graph.links}
        if sum(x[2]==2 for x in unique_links)<1:raise RuntimeError('No native type2 closure in UGV10 integration regression')
        capture_script=args.project_source/'tools/metric_parity/capture_graph_snapshot.py'
        capture=case.start('graph_capture',['python3',str(capture_script),'--output-cdr',str(args.output/'optimized_graph.cdr'),'--output-json',str(args.output/'optimized_graph.json'),'--timeout','30'])
        end=time.monotonic()+35
        while capture.poll() is None and time.monotonic()<end:case.spin(.1)
        if capture.poll()!=0:raise RuntimeError('Graph-only capture failed or timed out')
        graph_snapshot=json.loads((args.output/'optimized_graph.json').read_text())
        outcome['capture_script_sha256']=hashlib.sha256(capture_script.read_bytes()).hexdigest()
        node_stamps={n['id']:round(n['stamp']*1e9) for n in graph_snapshot['nodes']}
        dense_by_stamp={stamp(p):p.pose for p in latest_dense.poses}
        errors=[];unmatched=[]
        for item in graph_snapshot['optimized_poses']:
            identifier=item['id'];t=node_stamps.get(identifier)
            if t not in dense_by_stamp:unmatched.append(identifier);continue
            a=item['pose'];b=dense_by_stamp[t]
            translation=float(np.linalg.norm(np.array(a['translation_xyz'])-[b.position.x,b.position.y,b.position.z]))
            qa=np.array(a['quaternion_xyzw']);qb=np.array([b.orientation.x,b.orientation.y,b.orientation.z,b.orientation.w])
            rotation=math.degrees(2*math.acos(float(np.clip(abs(np.dot(qa,qb)/(np.linalg.norm(qa)*np.linalg.norm(qb))),0,1))))
            errors.append({'id':identifier,'stamp_ns':t,'translation_m':translation,'rotation_deg':rotation})
        maximum=max((e['translation_m'] for e in errors),default=float('inf'))
        maximum_rotation=max((e['rotation_deg'] for e in errors),default=float('inf'))
        anchors={'graph_poses':len(graph_snapshot['optimized_poses']),'matched':len(errors),'unmatched_ids':unmatched,'max_translation_m':maximum,'max_rotation_deg':maximum_rotation,'anchors':errors,'scope':'Internal optimized graph/dense consistency, not GT accuracy; GetMap2 has no remote epoch.'}
        (args.output/'anchor_consistency.json').write_text(json.dumps(anchors,indent=2)+'\n')
        outcome['anchor_consistency']={k:v for k,v in anchors.items() if k!='anchors'}
        if unmatched or not errors or any(not math.isfinite(e[k]) for e in errors for k in ['translation_m','rotation_deg']) or maximum>0.001 or maximum_rotation>0.01:raise RuntimeError('Graph/dense internal numerical consistency gate failed')
        if [stamp(p) for p in latest_dense.poses]!=odom_sequence:raise RuntimeError('Final dense changed to incomplete odometry sequence during graph capture')
        outcome['passed']=True
    except Exception:outcome['error']=traceback.format_exc()
    finally:
        rows.close()
        try:
            if latest_dense is not None:
                (args.output/'final_dense.cdr').write_bytes(serialize_message(latest_dense))
                outcome['final_dense_pose_count']=len(latest_dense.poses)
            if latest_graph is not None:
                (args.output/'final_mapgraph.cdr').write_bytes(serialize_message(latest_graph))
                unique={(min(x.from_id,x.to_id),max(x.from_id,x.to_id),x.type) for x in latest_graph.links}
                outcome['final_graph_pose_count']=len(latest_graph.poses)
                outcome['final_graph_link_counts']={str(t):sum(x[2]==t for x in unique) for t in sorted({x[2] for x in unique})}
            outcome.update({'counts':counts,'frames':{k:sorted(v) for k,v in frames.items()},'first_stamp_ns':first,'last_stamp_ns':last,'bad_odom':bad_odom,
                            'duplicate_odom_stamps':duplicates,'backward_odom_stamps':backwards,
                            'nonempty_rgb_clouds':rgb_nonempty,'xyz_rgb_clouds':rgb_valid,'sampled_unique_rgb':len(rgb_colors),
                            'nonempty_images':image_nonempty,'bad_metric_clouds':bad_metric_clouds,'metric_points_checked':metric_points,'wall_seconds':time.monotonic()-started})
        except Exception:
            outcome['passed']=False;outcome['evidence_error']=traceback.format_exc()
        clean=case.close();outcome['cleanup_passed']=clean;outcome['passed'] &= clean
        (args.output/'regression.json').write_text(json.dumps(outcome,indent=2)+'\n')
        print(json.dumps(outcome,indent=2),flush=True)
    return 0 if outcome['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
