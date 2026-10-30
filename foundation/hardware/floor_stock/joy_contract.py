"""One exact input contract for native no-motor preflight and the disabled floor path."""
import hashlib,math
from pathlib import Path

NODE_NAME='floor_physical_joy'
NAMESPACE='/'
TOPIC='/joy'
MESSAGE_TYPE='sensor_msgs/msg/Joy'
DEVICE_NAME='DualSense Wireless Controller'
SUBSCRIBER_DEPTH=10
PARAMETERS={'device_name':DEVICE_NAME,'autorepeat_rate':50.0,'coalesce_interval':0.02}

def command(executable):
    # Explicit namespace removes an ambient/default namespace assumption.
    return [str(executable),'--ros-args','-r','__node:='+NODE_NAME,'-r','__ns:=/',
            '-p','device_name:='+DEVICE_NAME,'-p','autorepeat_rate:=50.0','-p','coalesce_interval:=0.02']

def executable_identity(executable):
    p=Path(executable).resolve(strict=True)
    return {'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}

def enum_name(value):
    return getattr(value,'name',str(value)).upper()

def endpoint(info):
    qos=info.qos_profile
    return {'name':info.node_name,'namespace':info.node_namespace,'topic_type':info.topic_type,
            'gid':bytes(info.endpoint_gid).hex(),
            'reliability':enum_name(qos.reliability),'durability':enum_name(qos.durability),
            'history':enum_name(qos.history),'depth':qos.depth}

def graph_diagnostic(rows, expected_gid=None):
    # Discovery may be incomplete before input-only readiness; it never authorizes motion.
    if not rows:return {'ready':False,'reason':'no_publisher_discovered','publishers':rows}
    if len(rows)!=1:return {'ready':False,'reason':'multiple_publishers','publishers':rows}
    row=rows[0]
    for key,want,reason in [('name',NODE_NAME,'unexpected_node_name'),('namespace',NAMESPACE,'unexpected_namespace'),('topic_type',MESSAGE_TYPE,'unexpected_message_type')]:
        if row.get(key)!=want:return {'ready':False,'reason':reason,'publishers':rows}
    if not row.get('gid') or set(row['gid'])=={'0'}:return {'ready':False,'reason':'endpoint_identity_unavailable','publishers':rows}
    if expected_gid is not None and row['gid']!=expected_gid:return {'ready':False,'reason':'endpoint_identity_changed','publishers':rows}
    # A RELIABLE subscription cannot consume a BEST_EFFORT publisher. Unknown
    # RMW policy is insufficient evidence of compatibility; preserve details.
    if row.get('reliability')!='RELIABLE':return {'ready':False,'reason':'reliability_not_proven_compatible','publishers':rows}
    if row.get('durability') not in ('VOLATILE','TRANSIENT_LOCAL'):return {'ready':False,'reason':'durability_unknown','publishers':rows}
    return {'ready':True,'reason':'exact_single_source','publishers':rows,'gid':row['gid']}

def valid_joy(buttons,axes):
    return len(buttons)==13 and len(axes)==8 and all(type(x) is int and x in (0,1) for x in buttons) and all(math.isfinite(x) for x in axes)


def source_matches(diagnostic,gid):
    return diagnostic.get('ready') is True and isinstance(gid,str) and gid==diagnostic.get('gid')
