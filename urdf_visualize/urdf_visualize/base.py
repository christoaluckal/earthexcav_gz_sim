import bpy
import os
import math

# === CONFIGURATION ===
INPUT_FOLDER = os.path.join(os.pardir, "meshes") 
OUTPUT_FOLDER = os.path.join(os.pardir, "meshes")

Rotations = [90,0,0]


# === CLEAR SCENE ===
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# === PROCESS EACH STL FILE ===
for filename in os.listdir(INPUT_FOLDER):
    if filename.lower().endswith(".stl"):
        filepath = os.path.join(INPUT_FOLDER, filename)
        print(f"Processing: {filename}")

        # Import STL
        bpy.ops.import_mesh.stl(filepath=filepath)
        obj = bpy.context.selected_objects[0]

        # Rotate -90° around X
        # obj.rotation_euler[0] = math.radians(ROTATE_X_DEGREES)
        obj.rotation_euler[0] = math.radians(Rotations[0])
        obj.rotation_euler[1] = math.radians(Rotations[1])
        obj.rotation_euler[2] = math.radians(Rotations[2])

        # Apply the rotation so it's baked into mesh
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=False, rotation=True, scale=False)

        # Export to output folder
        output_path = os.path.join(OUTPUT_FOLDER, filename)
        bpy.ops.export_mesh.stl(filepath=output_path, use_selection=True)

        # Clean up before next file
        bpy.ops.object.delete()

print("✅ All STL files processed.")
