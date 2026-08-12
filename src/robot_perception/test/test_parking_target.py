import json

import pytest

from robot_perception.parking_target_node import parse_parking_spaces
from robot_perception.parking_target_node import select_parking_space


def make_spaces_json():
    return json.dumps([{
        'id': 'space_2',
        'center_x': 0.8015,
        'center_y': 0.0,
        'entry_yaw': 0.0,
        'final_yaw': 3.141592653589793,
        'length': 0.297,
        'width': 0.210,
        'approach_distance': 0.4,
    }])


def test_configured_parking_space_can_be_selected_by_id():
    spaces = parse_parking_spaces(make_spaces_json())

    selected = select_parking_space(spaces, 'space_2')

    assert selected['center_x'] == 0.8015
    assert selected['final_yaw'] > 3.14


def test_unknown_parking_space_is_rejected():
    spaces = parse_parking_spaces(make_spaces_json())

    with pytest.raises(ValueError, match='unknown selected_space_id'):
        select_parking_space(spaces, 'space_3')
