"""HH_261007 - Read-only UE mesh export for the repeated forward-loop contact.

No level load, package save, collision mutation or runtime control. Export only
the named nearby mesh candidates into a new explicit evidence directory.
"""
import hashlib
import json
import os
from pathlib import Path
import unreal


PIPELINE = Path('/home/hong/Downloads/ranger-carla-4ws-pipeline')
PROJECT = Path('/home/hong/carla/Unreal/CarlaUE4')
STAGE = PIPELINE / '.work/evidence/v224_validation/map_dropzone_stage.json'
SUFFIXES = (
    'Roads_Road_Layer0Node', 'Roads_Marking_Layer0Node',
    'WoodenFenceSupport01_PropNode1278', 'Prop_Prop_3Node',
    'Bush_Sm03_FoliageNode1588', 'Bush_Sm06_FoliageNode1581',
    'Prop_Prop_4Node', 'Roads_Bridge_Layer0Node', 'Terrain_Ground_Layer0Node',
    'Bush_Sm06_FoliageNode1583', 'Bush_Sm06_FoliageNode1591',
    'Bush_Sm03_FoliageNode1586', 'Bush_Sm03_FoliageNode1567',
    'Terrain_Marking_Layer0Node', 'B11_Wheel_Sweep_Clearance_Support_Patch_v13',
)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def prop(obj, name):
    try:
        return str(obj.get_editor_property(name))
    except Exception as error:
        return {'unavailable': str(error)}


def main():
    output = Path(os.environ['CAMROD_CONTACT_EXPORT_ROOT'])
    if not output.is_absolute() or output.exists():
        raise RuntimeError('require a new absolute export directory')
    if Path(str(unreal.Paths.get_project_file_path())).resolve() != PROJECT / 'CarlaUE4.uproject':
        raise RuntimeError('wrong project')
    stage = json.loads(STAGE.read_text())
    actors = stage['source_level_contract']['all_actor_contracts']
    selected = []
    for suffix in SUFFIXES:
        matches = [a for a in actors if a['label'].endswith(suffix)]
        if len(matches) != 1:
            raise RuntimeError('ambiguous/missing actor ' + suffix)
        selected.append(matches[0])
    # HH_261007 - Use the stage's full path/hash inventory, not guessed filenames.
    protected = {}
    for a in selected:
        package = a['component']['static_mesh'].split('.')[0]
        if not package.startswith('/Game/'):
            raise RuntimeError('mesh is outside project content')
        path = PROJECT / 'Content' / (package[len('/Game/'):] + '.uasset')
        protected[str(path)] = digest(path)
    map_path = Path(stage['target_files_after']['map']['path'])
    protected[str(map_path)] = digest(map_path)
    output.mkdir(parents=True)
    report = {'read_only': True, 'source_stage': str(STAGE), 'meshes': [],
              'protected_before': protected, 'status': 'EXPORTING'}
    try:
        for index, a in enumerate(selected):
            mesh = unreal.load_asset(a['component']['static_mesh'])
            if not isinstance(mesh, unreal.StaticMesh):
                raise RuntimeError('not a StaticMesh: ' + a['label'])
            target = output / ('%02d_%s.fbx' % (index, a['label']))
            task = unreal.AssetExportTask()
            for key, value in {'object': mesh, 'filename': str(target),
                               'automated': True, 'prompt': False,
                               'replace_identical': False,
                               'write_empty_files': False}.items():
                task.set_editor_property(key, value)
            task.set_editor_property('exporter', unreal.StaticMeshExporterFBX())
            options = unreal.FbxExportOption()
            options.set_editor_property('collision', True)
            options.set_editor_property('level_of_detail', False)
            task.set_editor_property('options', options)
            if not unreal.Exporter.run_asset_export_task(task) or not target.is_file():
                raise RuntimeError('mesh export failed: ' + a['label'])
            body = mesh.get_editor_property('body_setup')
            report['meshes'].append({'actor': a, 'export': str(target),
                'sha256': digest(target), 'collision_trace_flag': prop(body, 'collision_trace_flag'),
                'double_sided_geometry': prop(body, 'double_sided_geometry'),
                'simple_collision': prop(body, 'agg_geom'),
                'bounds': str(mesh.get_bounds())})
        report['status'] = 'EXPORTED_READ_ONLY'
    finally:
        report['protected_after'] = {p: digest(Path(p)) for p in protected}
        report['protected_unchanged'] = report['protected_after'] == protected
        with (output / 'manifest.json').open('x') as stream:
            json.dump(report, stream, indent=2)
    if not report['protected_unchanged']:
        raise RuntimeError('protected assets changed unexpectedly')
    print('CONTACT_EXPORT_READ_ONLY_OK ' + str(output))


main()
