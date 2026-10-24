"""RViz layout derived from the real fleet, with three isolated TF trees."""
import copy
import os
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from fleet_gazebo import robot_model
from warehouse_queue import COLORS, TRAY_Z


def robot_description(name, rgb):
    """Single rigid visual root; simulated odometry already publishes its TF."""
    sdf=ET.fromstring(robot_model(name,rgb))
    root=ET.Element('robot',name=name)
    link=ET.SubElement(root,'link',name='base_footprint')
    for v in sdf.findall('./link/visual'):
        vis=ET.SubElement(link,'visual',name=v.get('name'))
        p=v.findtext('pose').split()
        ET.SubElement(vis,'origin',xyz=' '.join(p[:3]),rpy=' '.join(p[3:]))
        g=ET.SubElement(vis,'geometry')
        cyl=v.find('geometry/cylinder')
        if cyl is not None:
            ET.SubElement(g,'cylinder',radius=cyl.findtext('radius'),length=cyl.findtext('length'))
        else:ET.SubElement(g,'box',size=v.findtext('geometry/box/size'))
        material=ET.SubElement(vis,'material',name=v.get('name')+'_color')
        ET.SubElement(material,'color',rgba=v.findtext('material/diffuse'))
    # Match the load platform and four posts shown in Gazebo.
    def shape(label,z,geometry,xyz='0 0'):
        v=ET.SubElement(link,'visual',name=label)
        ET.SubElement(v,'origin',xyz=xyz+' '+str(z))
        g=ET.SubElement(v,'geometry');g.append(geometry)
        mat=ET.SubElement(v,'material',name=label+'_color')
        ET.SubElement(mat,'color',rgba=rgb+' 1')
    shape('tray',TRAY_Z-.006,ET.Element('box',size='.33 .33 .012'))
    for a in (-1,1):
        for b in (-1,1):
            shape(f'post_{a}_{b}',.109,ET.Element('cylinder',radius='.006',length='.078'),
                  f'{a*.065} {b*.065}')
    return ET.tostring(root,encoding='unicode')


def prepare_rviz(out, template):
    out=Path(out)
    cfg=yaml.safe_load(Path(template).read_text())
    vm=cfg['Visualization Manager']
    base=next(d for d in vm['Displays'] if d.get('Name')=='Xe 1 (agv1)')
    map_display=copy.deepcopy(vm['Displays'][0]);map_display['Alpha']=.7
    markers=copy.deepcopy(vm['Displays'][1]);markers['Name']='Sa ban + ba xe'
    displays=[map_display,markers]
    for i,(name,rgb) in enumerate(COLORS.items(),1):
        group=copy.deepcopy(base);group['Name']=f'Xe {i} ({name})'
        group['Displays']=[d for d in group['Displays'] if d['Name']!='Hat AMCL']
        color='; '.join(str(round(float(x)*255)) for x in rgb.split())
        for d in group['Displays']:
            for key in ('Topic','Update Topic','Description Topic'):
                if isinstance(d.get(key),dict):
                    d[key]['Value']=d[key]['Value'].replace('agv1',name)
                    if key!='Description Topic' and d[key]['Value'].startswith('/'+name+'/'):
                        d[key]['Value']='/fleet_viz'+d[key]['Value']
            if 'Color' in d:d['Color']=color
            if d['Name']=='Mo hinh xe':
                path=out/(name+'.urdf');path.write_text(robot_description(name,rgb))
                d.update({'Description Source':'File','Description File':str(path),'TF Prefix':name})
            if d['Name'] in ('Local Costmap','Global Costmap'):
                d['Enabled']=False;d['Value']=False
        displays.append(group)
    displays.append(copy.deepcopy(vm['Displays'][-1]))
    vm['Displays']=displays
    vm['Views']['Current'].update(Scale=185,X=0.,Y=0.)
    cfg['Window Geometry'].update(Width=1250,Height=850)
    path=out/'warehouse.rviz';path.write_text(yaml.safe_dump(cfg,sort_keys=False))
    return path


def rviz_environment(out):
    """Apply this laptop's Mesa sampler fix to a private resource copy only."""
    from ament_index_python.packages import get_package_share_directory
    env=dict(os.environ)
    source=Path(get_package_share_directory('rviz_rendering'))
    relative=Path('ogre_media/materials/glsl120/indexed_8bit_image.frag')
    shader=(source/relative).read_text()
    if 'layout(binding' in shader:return env
    overlay=Path(out)/'rviz_resources'
    target=overlay/'share/rviz_rendering'
    shutil.copytree(source,target,dirs_exist_ok=True)
    shader=shader.replace('#version 120','#version 130\n#extension GL_ARB_shading_language_420pack : require')
    shader=shader.replace('uniform sampler2D eight_bit_image;',
                          'layout(binding = 0) uniform sampler2D eight_bit_image;')
    shader=shader.replace('uniform sampler1D palette;',
                          'layout(binding = 1) uniform sampler1D palette;')
    (target/relative).write_text(shader)
    marker=overlay/'share/ament_index/resource_index/packages/rviz_rendering'
    marker.parent.mkdir(parents=True,exist_ok=True);marker.touch()
    env['AMENT_PREFIX_PATH']=str(overlay)+os.pathsep+env.get('AMENT_PREFIX_PATH','')
    return env
