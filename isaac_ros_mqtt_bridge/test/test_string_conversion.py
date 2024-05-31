# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from isaac_ros_mqtt_bridge.MqttBridgeUtils import convert_camel_to_snake, \
    convert_dict_keys, convert_snake_to_camel


def test_camel_to_snake():
    assert convert_camel_to_snake('CamelToSnakeCase') == 'camel_to_snake_case'
    assert convert_camel_to_snake(
        'dromedaryToSnakeCase') == 'dromedary_to_snake_case'
    assert convert_camel_to_snake('allowedDeviationXY') == 'allowed_deviation_x_y'
    assert convert_snake_to_camel('') == ''


def test_snake_to_camel():
    # Testing typical CamelCase
    assert convert_snake_to_camel('snake_to_camel_case') == 'SnakeToCamelCase'
    assert convert_snake_to_camel('Snake') == 'Snake'
    assert convert_snake_to_camel('') == ''

    # Testing dromedaryCase
    assert convert_snake_to_camel(
        'snake_to_camel_case', dromedary=True) == 'snakeToCamelCase'
    assert convert_snake_to_camel('Snake', dromedary=True) == 'snake'
    assert convert_snake_to_camel('', dromedary=True) == ''


def test_convert_dict_keys():
    # Convert from snake to dromedary case
    snake_dict = {'snake_case': 0, 'snakey_case': 1,
                  'another_dict': {'snakey_snakey_case': 2},
                  'an_array': [{'yet_another_dict': 5}]}
    dromedary_dict = convert_dict_keys(snake_dict, 'snake_to_dromedary')

    assert 'snakeCase' in dromedary_dict
    assert 'snake_case' not in dromedary_dict
    assert 'snakeyCase' in dromedary_dict
    assert 'snakey_case' not in dromedary_dict
    assert 'anotherDict' in dromedary_dict
    assert 'another_dict' not in dromedary_dict

    inner_dict = dromedary_dict['anotherDict']
    assert 'snakeySnakeyCase' in inner_dict
    assert 'snakey_snakey_case' not in inner_dict

    assert 'yetAnotherDict' in dromedary_dict['anArray'][0]
    assert 'yet_another_dict' not in dromedary_dict['anArray'][0]

    # Convert from dromedary to snake case
    snake_dict = {'dromedaryCase': 0, 'dromedaryDromedaryCase': 1,
                  'anotherDict': {'notTrueCamelCase': 2},
                  'anArray': [{'yetAnotherDict': 5}]}
    dromedary_dict = convert_dict_keys(snake_dict, 'camel_to_snake')

    assert 'dromedary_case' in dromedary_dict
    assert 'dromedaryCase' not in dromedary_dict
    assert 'dromedary_dromedary_case' in dromedary_dict
    assert 'dromedaryDromedaryCase' not in dromedary_dict
    assert 'another_dict' in dromedary_dict
    assert 'anotherDict' not in dromedary_dict

    inner_dict = dromedary_dict['another_dict']
    assert 'not_true_camel_case' in inner_dict
    assert 'notTrueCamelCase' not in inner_dict

    assert 'yet_another_dict' in dromedary_dict['an_array'][0]
    assert 'yetAnotherDict' not in dromedary_dict['an_array'][0]
