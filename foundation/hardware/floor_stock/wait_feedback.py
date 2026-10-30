"""Bounded passive preparation gate; no publishes, loads or controller switches."""
import argparse,importlib.util,json,os,time
from pathlib import Path

def main():
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True);p.add_argument('--seconds',type=float,default=30.)
    p.add_argument('--default-inactive',action='store_true');args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='168' or not 0<args.seconds<=30:p.error('requires domain168 and bound<=30s')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters,DescribeParameters
    from std_msgs.msg import String
    spec=importlib.util.spec_from_file_location('probe',args.candidate/'console_init_stop_probe.py')
    probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
    spec=importlib.util.spec_from_file_location('status',Path(os.environ['PUPPER_STAGED_GEMINI_DIR'])/'robot/stop_status.py')
    status_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(status_module)
    tracker=status_module.StopStatus();context=Context();rclpy.init(context=context)
    node=rclpy.create_node('supervised_preparation_readonly',context=context);executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    clients=[node.create_client(ListControllers,'/controller_manager/list_controllers'),
             node.create_client(GetParameters,'/estop_controller/get_parameters'),
             node.create_client(DescribeParameters,'/estop_controller/describe_parameters')]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(String,status_module.TOPIC,lambda msg:tracker.accept(msg.data,time.monotonic()),qos)
    report={'passed':False,'scope':'Read-only controller/authority gate; probe separately validates sensors, policy, gains and mapped libraries'}
    names=['stop_arbitration_protocol','stop_arbitration_instance_id'];pending=[None]*3;sent=[0.]*3;values=[None]*3;at=[0.]*3
    started=time.monotonic()
    try:
        while time.monotonic()-started<args.seconds:
            executor.spin_once(timeout_sec=.02);now=time.monotonic()
            for i,client in enumerate(clients):
                if pending[i] is not None:
                    if pending[i].done():values[i]=pending[i].result();at[i]=now;pending[i]=None
                    elif now-sent[i]>1.:
                        client.remove_pending_request(pending[i]);pending[i]=None;values[i]=None
                if pending[i] is None and client.service_is_ready() and (values[i] is None or i==0 and now-at[i]>.1):
                    request=(ListControllers.Request() if i==0 else GetParameters.Request() if i==1 else DescribeParameters.Request())
                    if i:request.names=names
                    pending[i]=client.call_async(request);sent[i]=now
            if any(v is None for v in values):continue
            inventory={c.name:{'type':c.type,'state':c.state} for c in values[0].controller}
            expected={n:{'type':t,'state':'active'} for n,t in probe.BROADCASTERS.items()}
            if args.default_inactive:expected[probe.DEFAULT]={'type':'neural_controller/NeuralController','state':'inactive'}
            params=values[1].values;descriptors=values[2].descriptors
            if len(params)!=2 or len(descriptors)!=2:raise RuntimeError('authority descriptors/values missing')
            if not all(d.read_only for d in descriptors) or any(v.type!=4 for v in params):raise RuntimeError('authority markers not immutable strings')
            if params[0].string_value!='pupper_stop_v1':raise RuntimeError('authority protocol mismatch')
            state=tracker.state
            if tracker.permanent_error:raise RuntimeError(tracker.permanent_error)
            fresh=(tracker.received is not None and 0<=now-tracker.received<=.5 and tracker.error is None and now-at[0]<=.5)
            if (inventory==expected and fresh and state and state['instance_id']==params[1].string_value
                    and state['latched'] and not any(state[k] for k in ('fault','unknown_remote_submission','shutting_down','stop_required','in_flight'))):
                report.update(passed=True,instance_id=state['instance_id'],status=state,controllers=inventory,elapsed_s=now-started);break
        if not report['passed']:report['error']='bounded preparation gate did not become ready'
    except Exception as error:report['error']=repr(error)
    finally:
        executor.shutdown(timeout_sec=.5);node.destroy_node();context.shutdown()
        args.report.write_text(json.dumps(report,indent=2)+'\n')
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
