import struct

def ascii_to_binary_stl(in_filename, out_filename):
    with open(in_filename, 'r') as f:
        lines = f.readlines()
        
    facets = []
    current_normal = None
    current_vertices = []
    
    for line in lines:
        line = line.strip()
        if line.startswith("facet normal"):
            parts = line.split()
            current_normal = (float(parts[2]), float(parts[3]), float(parts[4]))
            current_vertices = []
        elif line.startswith("vertex"):
            parts = line.split()
            current_vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("endfacet"):
            if current_normal and len(current_vertices) == 3:
                facets.append((current_normal, current_vertices))
                
    with open(out_filename, 'wb') as f:
        # 80 byte header
        f.write(b'\0' * 80)
        # Number of triangles
        f.write(struct.pack('<I', len(facets)))
        
        for normal, vertices in facets:
            f.write(struct.pack('<3f', *normal))
            for vertex in vertices:
                f.write(struct.pack('<3f', *vertex))
            f.write(struct.pack('<H', 0)) # attribute byte count

print("Converting ZED2i.stl...")
ascii_to_binary_stl("src/roar_simulation/meshes/ZED2i.stl", "src/roar_simulation/meshes/ZED2i_bin.stl")
print("Done!")
