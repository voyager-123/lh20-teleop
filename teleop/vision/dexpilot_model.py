"""Add fixed, mesh-derived fingertip references without changing the CAN model."""
import hashlib
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

FINGERS=('thumb','index','middle','ring','pinky')


def palm_center(robot,wrist_link):
    """Neutral MCP center relative to the manufacturer's wrist/base frame."""
    robot.compute_forward_kinematics(np.zeros(robot.dof))
    wrist=robot.get_link_pose(robot.get_link_index(wrist_link))
    centers=np.array([robot.get_link_pose(robot.get_link_index(finger+'_proximal'))[:3,3]
                      for finger in FINGERS[1:]])
    return wrist[:3,:3].T@(centers.mean(axis=0)-wrist[:3,3])


def finger_chain_lengths(robot):
    """Four neutral MCP-to-tip polyline lengths, in model meters."""
    robot.compute_forward_kinematics(np.zeros(robot.dof))
    lengths=[]
    for finger in FINGERS[1:]:
        names=(finger+'_proximal',finger+'_middle',finger+'_distal','teleop_'+finger+'_tip')
        positions=np.array([robot.get_link_pose(robot.get_link_index(name))[:3,3] for name in names])
        lengths.append(float(np.linalg.norm(np.diff(positions,axis=0),axis=1).sum()))
    return np.array(lengths)


def reference_vectors(points,retargeting):
    """Align wrist vectors at MCP center; keep inter-tip vectors intact.

    The manufacturer's base can lie below an anatomical wrist. Matching it
    directly to MediaPipe 0 shortens targets and bends otherwise open fingers.
    Adapt wrist-origin vectors for palm offset and four-finger bone lengths;
    preserve the original inter-tip vectors, projection and hysteresis.
    """
    points=np.asarray(points,dtype=float)
    if points.shape!=(21,3) or not np.isfinite(points).all():
        raise ValueError('Expected 21 finite hand landmarks')
    indices=retargeting.optimizer.target_link_human_indices
    result=points[indices[1]]-points[indices[0]]
    if retargeting.lh20_algorithm!='dexpilot':return result
    if not np.all(indices[0,-5:]==0):raise ValueError('Unexpected DexPilot wrist vector order')
    scale=retargeting.optimizer.scaling
    if not np.isfinite(scale) or scale<=0:raise ValueError('Invalid DexPilot scale')
    human_center=points[[5,9,13,17]].mean(axis=0)-points[0]
    result[-5:]+=retargeting.lh20_palm_center/scale-human_center
    for index,mcp in enumerate((5,9,13,17)):
        human_length=float(np.linalg.norm(np.diff(points[mcp:mcp+4],axis=0),axis=1).sum())
        if not .02<=human_length<=.20:raise ValueError('Invalid tracked finger bone length')
        ratio=retargeting.lh20_finger_lengths[index]/(scale*human_length)
        result[-4+index]+=(ratio-1)*(points[mcp+3]-points[mcp])
    return result


def prepare_model(source):
    source=Path(source).resolve()
    tree=ET.parse(source);root=tree.getroot();frames={}
    for finger in FINGERS:
        parent=finger+'_distal'
        link=root.find(f"link[@name='{parent}']")
        collision=link.find('collision') if link is not None else None
        mesh=collision.find('geometry/mesh') if collision is not None else None
        if mesh is None:raise ValueError('Missing distal mesh: '+parent)
        origin=collision.find('origin')
        if origin is not None and any(float(v)!=0 for field in ('xyz','rpy') for v in origin.get(field,'0 0 0').split()):
            raise ValueError('Unsupported distal mesh transform: '+parent)
        path=(source.parent/mesh.get('filename')).resolve()
        geometry=trimesh.load(path,force='mesh',process=False)
        vertices=np.unique(np.asarray(geometry.vertices,dtype=float),axis=0)
        scale=np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
        vertices=vertices*scale
        if not np.isfinite(vertices).all() or vertices.shape[0]<4:
            raise ValueError('Invalid fingertip mesh: '+parent)
        # These manufacturer distal links point along +Z. Use the end-cap
        # surface (last 1 mm), not the DIP joint origin, as the tip reference.
        z=float(vertices[:,2].max())
        if not .015<z<.06:raise ValueError('Unexpected distal length: '+parent)
        cap=vertices[vertices[:,2]>=z-.001]
        xyz=[float(cap[:,0].mean()),float(cap[:,1].mean()),z]
        name='teleop_'+finger+'_tip'
        ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=name+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=' '.join(format(v,'.12g') for v in xyz),rpy='0 0 0')
        frames[finger]={'link':name,'parent':parent,'xyz_m':xyz,
                        'mesh_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    # The upstream loader writes another temporary URDF, so every mesh must
    # still resolve after either relocation. Source and CAD URDF stay untouched.
    for mesh in root.iter('mesh'):
        mesh.set('filename',str((source.parent/mesh.get('filename')).resolve()))
    temp=tempfile.TemporaryDirectory(prefix='lh20-dexpilot-model-')
    path=Path(temp.name)/source.name
    tree.write(path,encoding='utf-8',xml_declaration=True)
    metadata={'method':'distal_mesh_terminal_1mm_cap','physical_calibration':False,
              'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'frames':frames}
    return temp,path,metadata
