"""ROS-free construction of joystick deactivation coverage.

Face buttons use only the first len(switch_button_indices) controller names.
Trailing names extend emergency-stop/switch-away coverage without adding buttons.
The two explicit defaults below match the reviewed course joy_utils source; they
are materialized in the generated YAML so readiness never guesses runtime values.
"""
from copy import deepcopy
import re

NEURAL_TYPE = 'neural_controller/NeuralController'
ESTOP_DEFAULTS = {'estop_index': 12, 'estop_release_index': 9}
ESTOP_KEYS = ('controller_names', 'switch_button_indices', 'estop_index', 'estop_release_index')


def _names(values, label):
    if not isinstance(values, list) or not values:
        raise ValueError(f'{label} must be a nonempty list')
    if any(not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', value)
           for value in values):
        raise ValueError(f'{label} contains an invalid controller name')
    if len(set(values)) != len(values):
        raise ValueError(f'{label} contains duplicate controller names')
    return values


def estop_parameters(config):
    """Extract the explicit behavior contract; no fallback values are permitted."""
    try:
        params = config['estop_controller']['ros__parameters']
        result = {key: deepcopy(params[key]) for key in ESTOP_KEYS}
    except (KeyError, TypeError) as exc:
        raise ValueError('estop configuration must explicitly supply all four parameters') from exc
    names = _names(result['controller_names'], 'estop controller_names')
    buttons = result['switch_button_indices']
    if (not isinstance(buttons, list) or not buttons or len(buttons) > len(names)
            or any(type(value) is not int or value < 0 for value in buttons)
            or len(set(buttons)) != len(buttons)):
        raise ValueError('switch_button_indices must be unique nonnegative integers with mapped controllers')
    stops = [result['estop_index'], result['estop_release_index']]
    if any(type(value) is not int or value < 0 for value in stops):
        raise ValueError('estop and release indices must be nonnegative integers')
    if stops[0] == stops[1] or set(stops).intersection(buttons):
        raise ValueError('estop, release and face-button indices must not overlap')
    return result


def combined_course_config(course, added_controllers):
    """Copy the course config, preserving its mappings and all control settings.

    Only the estop section changes: extend coverage and explicitly retain the
    source's stop/release defaults when the course YAML omits them. The course
    launch always spawns default_alt when using the supplied course config.
    """
    result = deepcopy(course)
    try:
        params = result['estop_controller']['ros__parameters']
        manager = result['controller_manager']['ros__parameters']
        if not isinstance(params, dict) or not isinstance(manager, dict):
            raise TypeError('expected parameter mappings')
        for key, default in ESTOP_DEFAULTS.items():
            params.setdefault(key, default)
    except (KeyError, TypeError) as exc:
        raise ValueError('course config lacks estop/controller_manager parameters') from exc
    original = estop_parameters(result)['controller_names']
    additions = _names(list(added_controllers), 'Gemini controllers')
    for name in original:
        if not isinstance(manager.get(name), dict) or manager[name].get('type') != NEURAL_TYPE:
            raise ValueError(f'estop controller {name} is not a declared course neural controller')
    if set(additions).intersection(manager) or set(additions).intersection(original):
        raise ValueError('Gemini controller names collide with course controllers')
    names = list(original)
    alternate = 'neural_controller_default_alt'
    if alternate in manager:
        if not isinstance(manager[alternate], dict) or manager[alternate].get('type') != NEURAL_TYPE:
            raise ValueError('default_alt must be a neural controller')
        if alternate not in names:
            names.append(alternate)
    params['controller_names'] = names + additions
    estop_parameters(result)
    return result
