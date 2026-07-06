import os
import FreeCAD
import Mesh

files = {
    "Main-Body.FCStd": "main_body.stl",
    "Coxa.FCStd": "coxa.stl",
    "Femur.FCStd": "femur.stl",
    "Tibia.FCStd": "tibia.stl",
    "test_kotak.FCStd": "test_kotak.stl",
    "tabung.FCStd": "tabung.stl"
}

# Determine project root relative to this script's directory
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
fc_dir = os.path.join(project_root, "freecad")
output_dir = os.path.join(project_root, "assets", "meshes")
os.makedirs(output_dir, exist_ok=True)

for fc_file, stl_name in files.items():
    fc_path = os.path.join(fc_dir, fc_file)
    if not os.path.exists(fc_path):
        print(f"Skipping {fc_file}, not found at {fc_path}.")
        continue
    
    doc = FreeCAD.openDocument(fc_path)
    stl_path = os.path.join(output_dir, stl_name)
    
    # Export all Part objects with Shape attribute
    objs_to_export = []
    for obj in doc.Objects:
        if hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull():
            if hasattr(obj, "Visibility") and not obj.Visibility:
                continue
            objs_to_export.append(obj)
    
    # If no visible shape found, just pick the last shape object
    if not objs_to_export:
        for obj in reversed(doc.Objects):
            if hasattr(obj, "Shape") and obj.Shape is not None and not obj.Shape.isNull():
                objs_to_export.append(obj)
                break
                
    if objs_to_export:
        print(f"Exporting {fc_file} -> {stl_path} with {len(objs_to_export)} objects...")
        Mesh.export(objs_to_export, stl_path)
    else:
        print(f"ERROR: No shapes found in {fc_file}")
        
    FreeCAD.closeDocument(doc.Name)

print("Export script execution completed successfully!")
