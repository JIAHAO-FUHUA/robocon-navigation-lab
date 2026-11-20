"""Generate a dimensioned training world, not an unverified competition replica."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def generate(spec):
    if spec.get('mode') == 'competition' and not spec['official_layout_verified']:
        raise ValueError('Competition layout requires verified Appendix coordinates')
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='robocon_training')
    ET.SubElement(world, 'gravity').text = f"0 0 -{spec['gravity_mps2']}"
    physics = ET.SubElement(world, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(light, 'direction').text = '-0.3 -0.2 -1'

    def box(name, size, xyz, color):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, xyz)) + ' 0 0 0'
        link = ET.SubElement(model, 'link', name='link')
        for kind in ['collision', 'visual']:
            elem = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(elem, 'geometry')
            ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
            if kind == 'visual':
                material = ET.SubElement(elem, 'material')
                ET.SubElement(material, 'ambient').text = color
                ET.SubElement(material, 'diffuse').text = color

    sx, sy = spec['field_size_m']
    thickness = spec['floor_thickness_m']
    box('floor', [sx, sy, thickness], [0, 0, -thickness/2], '0.65 0.65 0.65 1')
    previous_top = 0.0
    for platform in sorted(spec['platforms'], key=lambda item: item['top_height_m']):
        top = platform['top_height_m']
        height = top - previous_top
        if height <= 0:
            raise ValueError('Platform top heights must increase')
        box(platform['name'], [*platform['size_m'], height],
            [*platform['center_xy_m'], previous_top+height/2], '0.35 0.48 0.65 1')
        previous_top = top
    # Unequal reference boxes break symmetry for the first localisation exercise.
    # They are training fixtures and must be removed/replaced for an official layout.
    for name, size, xyz in [('training_ref_A', [.4, .7, .8], [-3.5, -1, .4]),
                            ('training_ref_B', [.8, .4, 1.1], [1, -3.5, .55])]:
        box(name, size, xyz, '0.65 0.35 0.25 1')
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode', xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, default=ROOT/'config/field_spec.json')
    parser.add_argument('--output', type=Path, default=ROOT/'generated/robocon_training.sdf')
    args = parser.parse_args()
    spec = json.loads(args.spec.read_text(encoding='utf-8'))
    content = generate(spec)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content+'\n', encoding='utf-8', newline='\n')
    print(f'{args.output}: {spec["geometry_status"]}; no robot or official ramps added')


if __name__ == '__main__':
    main()
