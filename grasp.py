"""Ordinary fixed-contact bilateral normal-force balance, without cap or choice search."""
import numpy as np


def plan(contacts,request):
    by_id={row['id']:row for row in contacts}
    selected=[by_id[key] for key in request['fixed_assignment']]
    columns=[]
    for row in selected:
        n=np.asarray(row['normal'],dtype=float);p=np.asarray(row['point'],dtype=float)
        columns.append(np.concatenate([n,np.cross(p,n)]))
    matrix=np.column_stack(columns)
    certificates=[]
    total=0.0
    for load,weight in zip(request['loads'],request['weights']):
        force=np.linalg.lstsq(matrix,-np.asarray(load,dtype=float),rcond=None)[0]
        residual=matrix@force+np.asarray(load,dtype=float)
        certificates.append({'normal_forces':force.tolist(),'wrench_residual':residual.tolist()})
        total+=weight*float(force.sum())
    return {'assignment':list(request['fixed_assignment']),'certificates':certificates,'cost':total}
