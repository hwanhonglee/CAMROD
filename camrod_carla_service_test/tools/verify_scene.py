#!/usr/bin/env python3
"""HH_261103 - Rehash a caller-supplied local scene receipt; do not fabricate asset or gate acceptance."""
import hashlib
import json
from pathlib import Path
import shlex
import sys


def require(condition, detail):
    if not condition:
        raise ValueError(detail)


def verify(path):
    value = json.loads(Path(path).read_text())
    require(value['schema'] == 'camrod.service_charging_geometry.stage.v1', 'wrong scene schema')
    require(value['status'] == 'APPLIED' and value['errors'] == [], 'scene was not applied cleanly')
    for flag in ('source_disk_reload_contract_unchanged', 'non_tag_actor_contracts_unchanged',
                 'target_disk_reload_verified', 'native_normal_station_bytes_unchanged', 'earlier_scene_bytes_unchanged'):
        require(value[flag] is True, flag)
    expected = '/Game/map_package/Maps/Woraksan_v27_charge_anchor_20261103/Woraksan_v27_charge_anchor_20261103'
    require(value['target_map_package'] == expected, 'wrong scene URI')
    require(value['charging_geometry']['charging_behind_station_m'] == .60, 'wrong charging anchor')
    change = value['map_change']
    require(change['only_tag_actor_locations_modified'] is True, 'nontranslation scene edit')
    require(change['tag_actor_count_before'] == change['tag_actor_count_after'] == 46, 'tag inventory drift')
    require(change['actor_count_before'] == change['actor_count_after'], 'actor inventory drift')
    for group in ('source_files', 'target_files_after', 'protected_files_after', 'charging_geometry_protected_files'):
        require(bool(value[group]), 'empty scene bindings: ' + group)
        for record in value[group].values():
            file = Path(record['path'])
            require(file.is_file() and hashlib.sha256(file.read_bytes()).hexdigest() == record['sha256'], str(file))
    return expected


if __name__ == '__main__':
    uri = verify(sys.argv[1])
    print('export CARLA_UE_MAP=' + shlex.quote(uri))
    print('export CARLA_TOWN=' + shlex.quote(uri.removeprefix('/Game/')))
    print('export CAMROD_CARLA_MAP_PROFILE=""')
