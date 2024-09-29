import os
import re
import jinja2
import symforce

from pathlib import Path

symforce.set_epsilon_to_symbol()

import groups.generator as group_generator
import splines.generator as spline_generator
import cost_functions.generator as cost_function_generator

CURRENT_DIR = Path(__file__).parent
PROJECT_DIR = CURRENT_DIR.parent


def generate_header(include_dir: Path, relative_path: str, extension: str = '.hpp',
                    regex=re.compile('(.*h$)|(.*hpp$)'), usr_excluded=None):
    if usr_excluded is None:
        usr_excluded = []

    includes: set[str] = set()
    full_path = include_dir / relative_path
    usr_excluded.append(full_path.name + extension)
    for root, dirs, files in os.walk(full_path):
        for file in files:
            if file not in usr_excluded and regex.match(file):
                includes.add(file)
        for dir in dirs:
            includes.add(dir + '/' + dir + extension)
        break

    loader = jinja2.FileSystemLoader(CURRENT_DIR / 'templates')
    environment = jinja2.Environment(loader=loader)
    template = environment.get_template('AGGREGATE_HEADER.hpp.jinja')
    output = template.render(relative_path=relative_path, includes=sorted(includes))
    file = open(full_path / (full_path.name + extension), "w")
    file.write(output)
    file.close()


def generate():
    # Generate optimized code.
    include_dir = PROJECT_DIR / 'include'
    hyperion_root_dir = PROJECT_DIR / 'include/hyperion'

    # Generate Ceres code.
    group_generator.generate_ceres(hyperion_root_dir / 'ceres/groups/sym')
    cost_function_generator.generate_ceres(hyperion_root_dir / 'ceres/cost_functions')

    # Generate Ceres headers.
    generate_header(include_dir, 'hyperion/ceres/cost_functions/sym')
    generate_header(include_dir, 'hyperion/ceres/cost_functions')
    generate_header(include_dir, 'hyperion/ceres/groups/sym')
    generate_header(include_dir, 'hyperion/ceres/groups')
    generate_header(include_dir, 'hyperion/ceres/manifolds')

    # Generate Hyperion code.
    group_generator.generate_hyperion(hyperion_root_dir / 'groups/sym')
    spline_generator.generate(hyperion_root_dir / 'splines/sym')
    cost_function_generator.generate_hyperion(hyperion_root_dir / 'cost_functions')

    # Generate Hyperion headers.
    generate_header(include_dir, 'hyperion/cost_functions/sym')
    generate_header(include_dir, 'hyperion/cost_functions')
    generate_header(include_dir, 'hyperion/loss_functions')
    generate_header(include_dir, 'hyperion/factors')
    generate_header(include_dir, 'hyperion/groups/sym')
    generate_header(include_dir, 'hyperion/groups')
    generate_header(include_dir, 'hyperion/messages')
    generate_header(include_dir, 'hyperion/nodes')
    generate_header(include_dir, 'hyperion/splines/sym')
    generate_header(include_dir, 'hyperion/splines')
    generate_header(include_dir, 'hyperion/utils')


if __name__ == "__main__":
    generate()
