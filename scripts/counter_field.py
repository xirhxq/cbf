"""Versioned, index-addressed latent field for new experiments, not old-field continuation."""
import hashlib,math
ALGORITHM='sha256-box-muller-edge-v1'
def latent(frame_index,formation,seed):
 if not isinstance(seed,int) or seed<0 or not isinstance(frame_index,int) or frame_index<0:raise ValueError('nonnegative integer seed and frame required')
 entries=[]
 for robot in formation:
  i=int(robot['id'])
  for kind,j in [('base',int(x)) for x in robot['baseIds']]+[('uav',int(x)) for x in robot['anchorIds']]:
   key=f'{ALGORITHM}|{seed}|{frame_index}|{i}|{kind}|{j}'.encode('ascii');b=hashlib.sha256(key).digest();u=[((int.from_bytes(b[k:k+8],'big')>>11)+.5)/2**53 for k in (0,8,16)];normal=math.sqrt(-2*math.log(u[1]))*math.cos(2*math.pi*u[2]);entries.append({'node':i,'kind':kind,'reference':j,'availability_uniform':u[0],'standard_normal':normal})
 return {'frame_index':frame_index,'edges':entries}
