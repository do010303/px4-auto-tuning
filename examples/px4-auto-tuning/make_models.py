#!/usr/bin/env python3
"""Generate physical variants of the x500 into ./models.

Each variant is a copy of PX4's x500 (and, when the airframe itself changes,
of x500_base) with a few numbers edited, plus a `px4.params` file with the
PX4 parameters that have to follow the geometry. run_sitl.sh picks both up:

    PX4_SIM_MODEL=gz_x500_heavy ./run_sitl.sh

    python3 make_models.py          # regenerate all variants
"""

import os
import re
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, '..', '..', 'Tools', 'simulation', 'gz', 'models')
OUT = os.path.join(HERE, 'models')

# x500: rotors at (+-0.174, +-0.174) m in the model, 2.0 kg base link
VARIANTS = {
    # 30 % heavier airframe, inertia scaled with it; hover needs ~76 % thrust
    'x500_heavy': {'mass': 1.3, 'inertia': 1.3},
    # arms 40 % longer: more torque authority, more inertia
    'x500_long': {'arm': 1.4, 'inertia': 1.6},
    # 25 % less thrust per motor and three times slower motor response
    'x500_weak': {'thrust': 0.75, 'motor_lag': 3.0},
    # Gazebo's own NavSat noise: white Gaussian, 0.5 m per axis, declared in the SDF
    'x500_gznoise': {'navsat_noise': 0.5},
}


def scale_tag(text, tag, factor, count=0):
    def repl(match):
        return f'<{tag}>{float(match.group(1)) * factor:.10g}</{tag}>'

    return re.sub(rf'<{tag}>([-+0-9.eE]+)</{tag}>', repl, text, count=count)


def build(name, spec):
    model_dir = os.path.join(OUT, name)
    shutil.rmtree(model_dir, ignore_errors=True)
    os.makedirs(model_dir)
    base_name = 'x500_base'
    px4_params = {}

    if any(key in spec for key in ('mass', 'inertia', 'arm', 'navsat_noise')):
        base_name = f'{name}_base'
        base_dir = os.path.join(OUT, base_name)
        shutil.rmtree(base_dir, ignore_errors=True)
        os.makedirs(base_dir)
        sdf = open(os.path.join(SOURCE, 'x500_base', 'model.sdf')).read()
        sdf = sdf.replace("<model name='x500_base'>", f"<model name='{base_name}'>")

        # The base link is the first link: its mass and inertia come first in the file
        if 'mass' in spec:
            sdf = scale_tag(sdf, 'mass', spec['mass'], count=1)

        if 'inertia' in spec:
            for tag in ('ixx', 'iyy', 'izz'):
                sdf = scale_tag(sdf, tag, spec['inertia'], count=1)

        if 'arm' in spec:
            # Move everything that sits at the end of an arm: motor mounts and rotor links
            def move(match):
                x, y = float(match.group(1)), float(match.group(2))
                return f'<pose>{x * spec["arm"]:.6g} {y * spec["arm"]:.6g} {match.group(3)}</pose>'

            sdf = re.sub(r'<pose>(-?0\.174) (-?0\.174) ([^<]*)</pose>', move, sdf)

            # Control allocation has to know the new rotor positions (airframe 4001 values scaled)
            for i, (px, py) in enumerate(((0.13, 0.22), (-0.13, -0.20), (0.13, -0.22), (-0.13, 0.20))):
                px4_params[f'CA_ROTOR{i}_PX'] = round(px * spec['arm'], 4)
                px4_params[f'CA_ROTOR{i}_PY'] = round(py * spec['arm'], 4)

        if 'navsat_noise' in spec:
            sigma = spec['navsat_noise']
            noise = '<noise type="gaussian"><mean>0</mean><stddev>{:.4g}</stddev></noise>'
            # gz-sensors adds the horizontal noise to latitude and longitude in
            # degrees, the vertical noise to altitude in metres
            block = (f'<navsat><position_sensing><horizontal>{noise.format(sigma / 111320.0)}</horizontal>'
                     f'<vertical>{noise.format(sigma)}</vertical></position_sensing></navsat>')
            sdf = re.sub(r'(<sensor name="navsat_sensor" type="navsat">)', r'\1\n        ' + block, sdf)

        open(os.path.join(base_dir, 'model.sdf'), 'w').write(sdf)
        config = open(os.path.join(SOURCE, 'x500_base', 'model.config')).read()
        open(os.path.join(base_dir, 'model.config'), 'w').write(config.replace('x500_base', base_name))

    sdf = open(os.path.join(SOURCE, 'x500', 'model.sdf')).read()
    sdf = sdf.replace("<model name='x500'>", f"<model name='{name}'>")
    sdf = sdf.replace('model://x500_base</uri>', f'model://{base_name}</uri>')

    if 'thrust' in spec:
        sdf = scale_tag(sdf, 'motorConstant', spec['thrust'])

    if 'motor_lag' in spec:
        sdf = scale_tag(sdf, 'timeConstantUp', spec['motor_lag'])
        sdf = scale_tag(sdf, 'timeConstantDown', spec['motor_lag'])

    open(os.path.join(model_dir, 'model.sdf'), 'w').write(sdf)
    config = open(os.path.join(SOURCE, 'x500', 'model.config')).read()
    open(os.path.join(model_dir, 'model.config'), 'w').write(config.replace('<name>x500</name>', f'<name>{name}</name>'))

    with open(os.path.join(model_dir, 'px4.params'), 'w') as f:
        f.write(f'# PX4 parameters that go with {name}; applied at boot by run_sitl.sh\n')

        for key, value in px4_params.items():
            f.write(f'{key} {value}\n')


if __name__ == '__main__':
    for variant, changes in VARIANTS.items():
        build(variant, changes)
        print(variant, changes)
