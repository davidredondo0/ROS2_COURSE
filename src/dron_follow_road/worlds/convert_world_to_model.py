import xml.etree.ElementTree as ET
import sys
import os

def fix_world_poses(input_file, output_file):
    if not os.path.exists(input_file):
        print(f"Error: No encuentro el archivo {input_file}")
        return

    tree = ET.parse(input_file)
    root = tree.getroot()
    world = root.find('world')

    if world is None:
        print("Error: El archivo no contiene una etiqueta <world>")
        return

    # 1. Leer las poses guardadas en <state> (la sección del final)
    state = world.find('state')
    pose_map = {}
    
    if state is not None:
        print("Leyendo estados guardados...")
        for model in state.findall('model'):
            name = model.get('name')
            pose = model.find('pose')
            if pose is not None:
                pose_map[name] = pose.text
        
        # Borramos la sección state para evitar conflictos futuros
        world.remove(state)
    else:
        print("Advertencia: No se encontró sección <state>. Se usaran las posiciones originales.")

    # 2. Aplicar esas poses a los modelos reales
    count = 0
    for model in world.findall('model'):
        name = model.get('name')
        if name in pose_map:
            # Buscar la etiqueta pose o crearla si no existe
            p = model.find('pose')
            if p is None:
                p = ET.SubElement(model, 'pose')
            
            # Sobrescribir con el valor del state
            p.text = pose_map[name]
            count += 1
    
    print(f"Se actualizaron las posiciones de {count} modelos.")

    # 3. Limpieza: Borrar física antigua (ODE) que da problemas en Ignition
    physics_removed = False
    for physics in world.findall('physics'):
        if physics.get('type') == 'ode':
            world.remove(physics)
            physics_removed = True
    
    if physics_removed:
        print("Se eliminó la configuración de física antigua (ODE).")

    # 4. Guardar
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"¡Éxito! Mundo listo guardado en: {output_file}")

if __name__ == "__main__":
    # Usamos nombres fijos para facilitarte la vida
    input_f = "gas_station_2.world"     # Tu archivo original
    output_f = "gas_station_fixed.sdf" # El archivo que vas a ejecutar
    
    fix_world_poses(input_f, output_f)