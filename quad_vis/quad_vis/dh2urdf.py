import json
import os
from string import Template
from math import pi

from ament_index_python.packages import get_package_share_directory
from transformations import (translation_matrix, rotation_matrix,
                             concatenate_matrices, euler_from_matrix,
                             translation_from_matrix)

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def generate_leg(leg: str, params: list):
    front_signs = {'front': '', 'rear': '-'}
    side_signs = {'left': '', 'right': '-'}
    lower_limits = {'left': '0', 'right': str(-pi)}
    upper_limits = {'left': str(pi), 'right': '0'}
    with open('../urdf/leg.urdf.template', 'r') as file:
        leg_template = Template(file.read())

    mappings = {'l_width': 0.02, 'leg': leg}
    front, side = leg.split('_')
    mappings['front_sign'] = front_signs[front]
    mappings['side_sign'] = side_signs[side]
    mappings['lower_limit'] = lower_limits[side]
    mappings['upper_limit'] = upper_limits[side]

    for param in params:
        try:
            d = float(param['d'])
        except ValueError:
            d = 0.0
        try:
            th = float(param['th'])
        except ValueError:
            th = 0.0
        try:
            a = float(param['a'])
        except ValueError:
            a = 0.0
        try:
            al = float(param['al'])
        except ValueError:
            al = 0.0

        tz = translation_matrix((0, 0, d))
        rz = rotation_matrix(th, zaxis)
        tx = translation_matrix((a, 0, 0))
        rx = rotation_matrix(al, xaxis)

        matrix = concatenate_matrices(tz, rz, tx, rx)

        rpy = euler_from_matrix(matrix)
        xyz = translation_from_matrix(matrix)

        name = param['joint']

        mappings[f'{name}_j_xyz'] = '{} {} {}'.format(*xyz)
        mappings[f'{name}_j_rpy'] = '{} {} {}'.format(*rpy)
        mappings[f'{name}_l_xyz'] = '{} 0 0'.format(xyz[0] / 2.)
        mappings[f'{name}_l_rpy'] = '0 0 0'
        mappings[f'{name}_l_len'] = str(a)

    return leg_template.substitute(mappings)


def generate_robot(legs_description: list):
    with open('../urdf/robot.urdf.template', 'r') as file:
        robot_template = Template(file.read())

    mapping = {'legs_description': '\n'.join(legs_description)}

    return robot_template.substitute(mapping)


def raw_to_legs(raw_params: list):
    legs = {}
    for param in raw_params:
        key = param['leg']
        if key not in legs:
            legs[key] = []
        legs[key].append(param)

    return legs


if __name__ == '__main__':
    params_path = os.path.join(
        get_package_share_directory('quad_params'),
        'quad_params.json'
    )
    with open(params_path, 'r') as file:
        legs_params = json.load(file)
    legs_params.pop('general', None)

    legs_description = [
        generate_leg(leg, params)
        for leg, params in legs_params.items()
    ]

    result = generate_robot(legs_description)

    with open('../urdf/robot.urdf', 'w') as file:
        file.write(result)
