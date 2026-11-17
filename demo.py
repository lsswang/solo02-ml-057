import json
from pathlib import Path
import registration
import grasp


def run(data):
    pose=registration.fit_known(data['nominal_points'],data['observed_points'])
    geometry=registration.transform_contacts(data['contacts'],pose)
    return {'pose':pose,'geometry':geometry,'grasp':grasp.plan(geometry,data['request'])}


if __name__=='__main__':
    data=json.loads((Path(__file__).parent/'fixtures/cases.json').read_text())
    print(json.dumps(run(data),sort_keys=True))
