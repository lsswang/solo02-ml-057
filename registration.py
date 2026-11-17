"""Known-pair rigid registration and ordinary nearest-feature diagnostics."""
import numpy as np


def fit_known(nominal, observed):
    p,q=np.asarray(nominal,dtype=float),np.asarray(observed,dtype=float)
    if p.shape!=q.shape or p.ndim!=2 or p.shape[1]!=3 or len(p)<3:
        raise ValueError('at least three aligned known point pairs required')
    pc,qc=p.mean(axis=0),q.mean(axis=0)
    u,s,vt=np.linalg.svd((p-pc).T@(q-qc))
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:
        vt[-1]*=-1
        rotation=vt.T@u.T
    translation=qc-rotation@pc
    residual=q-(p@rotation.T+translation)
    return {'rotation':rotation.tolist(),'translation':translation.tolist(),
            'squared_residual':float(np.sum(residual**2))}


def nearest_matches(nominal,observed,rotation=None,translation=None):
    p=np.asarray(nominal,dtype=float);q=np.asarray(observed,dtype=float)
    r=np.eye(3) if rotation is None else np.asarray(rotation,dtype=float)
    t=np.zeros(3) if translation is None else np.asarray(translation,dtype=float)
    distances=np.sum((q[:,None,:]-(p@r.T+t)[None,:,:])**2,axis=2)
    return np.argmin(distances,axis=1).tolist()


def transform_contacts(contacts,pose):
    r=np.asarray(pose['rotation'],dtype=float);t=np.asarray(pose['translation'],dtype=float)
    result=[]
    for contact in contacts:
        row=dict(contact)
        row['point']=(r@np.asarray(contact['point'])+t).tolist()
        for key in ('normal','u','v'):
            row[key]=(r@np.asarray(contact[key])).tolist()
        result.append(row)
    return result
