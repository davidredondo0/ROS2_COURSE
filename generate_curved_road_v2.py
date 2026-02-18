import math

def generate_sdf():
    R = 10.0
    W = 5.0
    H = 0.1
    N = 9  # 9 segments for 90 degrees
    
    xml = []
    xml.append('<?xml version="1.0" ?>')
    xml.append('<sdf version="1.6">')
    xml.append('  <model name="road_curved">')
    xml.append('    <static>true</static>')
    xml.append('    <link name="link">')
    
    delta_deg = 90.0 / N
    delta_rad = math.radians(delta_deg)
    
    # Calculate length based on OUTER radius to ensure overlap and no gaps
    R_outer_edge = R + W/2.0
    # Chord length for the outer edge
    L_segment = 2 * R_outer_edge * math.sin(delta_rad / 2.0)
    # Add a small overlap
    L_segment += 0.1
    
    # Line dimensions
    line_width = 0.15
    line_height = 0.001
    line_z = H/2 + 0.001
    
    # Side offset for lines
    side_offset = 2.4
    
    for i in range(N):
        # Angle of the center of the segment
        alpha_deg = i * delta_deg + delta_deg / 2.0
        alpha_rad = math.radians(alpha_deg)
        
        x = R * math.sin(alpha_rad)
        y = R * (1 - math.cos(alpha_rad))
        yaw = alpha_rad
        
        # Segment Visual
        xml.append(f'      <visual name="visual_segment_{i}">')
        xml.append(f'        <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>')
        xml.append('        <geometry>')
        xml.append('          <box>')
        xml.append(f'            <size>{L_segment:.4f} {W} {H}</size>')
        xml.append('          </box>')
        xml.append('        </geometry>')
        xml.append('        <material>')
        xml.append('          <ambient>0.2 0.2 0.2 1</ambient>')
        xml.append('          <diffuse>0.2 0.2 0.2 1</diffuse>')
        xml.append('          <specular>0.1 0.1 0.1 1</specular>')
        xml.append('        </material>')
        xml.append('      </visual>')
        
        # Segment Collision
        xml.append(f'      <collision name="collision_segment_{i}">')
        xml.append(f'        <pose>{x:.4f} {y:.4f} 0 0 0 {yaw:.4f}</pose>')
        xml.append('        <geometry>')
        xml.append('          <box>')
        xml.append(f'            <size>{L_segment:.4f} {W} {H}</size>')
        xml.append('          </box>')
        xml.append('        </geometry>')
        xml.append('      </collision>')
        
        # Lines
        # Center Line (Yellow)
        # Use chord at center radius
        L_center = 2 * R * math.sin(delta_rad / 2.0) + 0.05
        xml.append(f'      <visual name="visual_center_line_{i}">')
        xml.append(f'        <pose>{x:.4f} {y:.4f} {line_z} 0 0 {yaw:.4f}</pose>')
        xml.append('        <geometry>')
        xml.append('          <box>')
        xml.append(f'            <size>{L_center:.4f} {line_width} {line_height}</size>')
        xml.append('          </box>')
        xml.append('        </geometry>')
        xml.append('        <material>')
        xml.append('          <ambient>1 1 0 1</ambient>')
        xml.append('          <diffuse>1 1 0 1</diffuse>')
        xml.append('        </material>')
        xml.append('      </visual>')
        
        # Inner Line (White)
        dx_in = -side_offset * math.sin(yaw)
        dy_in = side_offset * math.cos(yaw)
        R_in = R - side_offset
        L_in = 2 * R_in * math.sin(delta_rad / 2.0) + 0.05
        
        xml.append(f'      <visual name="visual_inner_line_{i}">')
        xml.append(f'        <pose>{x+dx_in:.4f} {y+dy_in:.4f} {line_z} 0 0 {yaw:.4f}</pose>')
        xml.append('        <geometry>')
        xml.append('          <box>')
        xml.append(f'            <size>{L_in:.4f} {line_width} {line_height}</size>')
        xml.append('          </box>')
        xml.append('        </geometry>')
        xml.append('        <material>')
        xml.append('          <ambient>1 1 1 1</ambient>')
        xml.append('          <diffuse>1 1 1 1</diffuse>')
        xml.append('        </material>')
        xml.append('      </visual>')

        # Outer Line (White)
        dx_out = -(-side_offset) * math.sin(yaw)
        dy_out = (-side_offset) * math.cos(yaw)
        R_out = R + side_offset
        L_out = 2 * R_out * math.sin(delta_rad / 2.0) + 0.05

        xml.append(f'      <visual name="visual_outer_line_{i}">')
        xml.append(f'        <pose>{x+dx_out:.4f} {y+dy_out:.4f} {line_z} 0 0 {yaw:.4f}</pose>')
        xml.append('        <geometry>')
        xml.append('          <box>')
        xml.append(f'            <size>{L_out:.4f} {line_width} {line_height}</size>')
        xml.append('          </box>')
        xml.append('        </geometry>')
        xml.append('        <material>')
        xml.append('          <ambient>1 1 1 1</ambient>')
        xml.append('          <diffuse>1 1 1 1</diffuse>')
        xml.append('        </material>')
        xml.append('      </visual>')

    xml.append('    </link>')
    xml.append('  </model>')
    xml.append('</sdf>')
    
    print('\n'.join(xml))

if __name__ == '__main__':
    generate_sdf()
