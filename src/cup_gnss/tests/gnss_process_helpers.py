"""TEST_ONLY ownership helpers for the isolated GNSS PTY drill.

Extracted without behavior changes from cup_nav_bridge.bringup at old project
commit 65308aa68ddf640cb3f0012393e6d02b3cae1947. This test utility is not a
production launcher or a field process manager. The caller must enforce its
network-none, no-device container and independent-master test boundary.
"""
import os
from pathlib import Path
import signal
import time
import xmlrpc.client


def safe_path(value, must_exist=False):
    path=Path(value).expanduser()
    if '..' in path.parts or any(x.casefold().startswith('backup') for x in path.parts):
        raise ValueError('Excluded backup or parent traversal path')
    path=path.absolute()
    for parent in [*reversed(path.parents),path]:
        if parent.is_symlink(): raise ValueError('Symlink paths are excluded: '+str(parent))
    if must_exist and not path.exists(): raise ValueError('Missing path: '+str(path))
    return path


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        conn=super().make_connection(host);conn.timeout=1.5;return conn


def rpc(uri): return xmlrpc.client.ServerProxy(uri,transport=TimeoutTransport(),allow_none=True)


def graph(master):
    try:
        code,message,state=rpc(master).getSystemState('/cup_nav_cli')
        if code!=1: raise RuntimeError(message)
        code,_,types=rpc(master).getTopicTypes('/cup_nav_cli')
        return {'reachable':True,'state':state,'types':types if code==1 else [],'time':time.time()}
    except (OSError,xmlrpc.client.Error,ValueError) as exc:
        return {'reachable':False,'error':str(exc),'state':[[],[],[]],'types':[],'time':time.time()}


def identity(pid):
    try:
        stat=Path('/proc/{}/stat'.format(pid)).read_text().rsplit(')',1)[1].split()
        raw=Path('/proc/{}/environ'.format(pid)).read_bytes().split(b'\0')
        token=next((v.split(b'=',1)[1].decode() for v in raw if v.startswith(b'NAV_RUN_ID=')),None)
        return {'birth':stat[19],'pgid':int(stat[2]),'ppid':int(stat[1]),'state':stat[0],'token':token}
    except (OSError,ValueError,IndexError,UnicodeError): return None


def capture_process(process,run_id,command):
    ident=identity(process.pid)
    if not ident: raise RuntimeError('Process exited before ownership capture')
    return {'pid':process.pid,'birth':ident['birth'],'run_id':run_id,'command':command,'started_at':time.time()}


def group_members(state):
    pid=state.get('pid',0)
    if not isinstance(pid,int) or pid<=1: return []
    result=[]
    for entry in Path('/proc').iterdir():
        if entry.name.isdecimal():
            ident=identity(int(entry.name))
            if ident and ident['pgid']==pid and ident['state']!='Z': result.append((int(entry.name),ident))
    return result


def stop_process(state,timeout=8):
    ident=identity(state.get('pid',0))
    if ident and ident['birth']!=state.get('birth'): return False
    members=group_members(state)
    if not members: return False
    if any(i['token']!=state.get('run_id') for _,i in members):
        raise RuntimeError('Refuse process group whose ownership changed')
    for sig,wait in [(signal.SIGINT,timeout),(signal.SIGTERM,2),(signal.SIGKILL,1)]:
        members=group_members(state)
        if not members: return True
        if any(i['token']!=state.get('run_id') for _,i in members): raise RuntimeError('Process group ownership changed during stop')
        os.killpg(state['pid'],sig)
        end=time.monotonic()+wait
        while group_members(state) and time.monotonic()<end: time.sleep(.05)
    if group_members(state): raise RuntimeError('Owned process group did not exit')
    return True
