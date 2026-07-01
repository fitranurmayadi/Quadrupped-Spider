import pybullet as p
import pybullet_data
import time
import os

# Hubungkan ke PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Muat lantai dasar untuk referensi grid
p.loadURDF("plane.urdf")

# Dapatkan absolute path URDF
urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "assets", "test_axis.urdf"))

print(f"Memuat URDF uji sumbu: {urdf_path}...")
robot_id = p.loadURDF(urdf_path, [0, 0, 0.0], flags=p.URDF_USE_INERTIA_FROM_FILE)

# Atur kamera yaw sudut 0, pitch -30, distance 3.0, target 0,0,0
print("Mengatur kamera ke: Yaw=0, Pitch=-30, Distance=3.0...")
p.resetDebugVisualizerCamera(
    cameraDistance=3.0,
    cameraYaw=0,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0]
)

# Gambar garis koordinat sumbu dunia untuk referensi visual:
# Sumbu X = Merah
# Sumbu Y = Hijau
# Sumbu Z = Biru
p.addUserDebugLine([0, 0, 0], [1.5, 0, 0], [1, 0, 0], lineWidth=5) # X
p.addUserDebugLine([0, 0, 0], [0, 1.5, 0], [0, 1, 0], lineWidth=5) # Y
p.addUserDebugLine([0, 0, 0], [0, 0, 1.5], [0, 0, 1], lineWidth=5) # Z

print("\n=== Uji Sumbu URDF Berhasil Dimuat ===")
print("Perhatikan warna tabung untuk menentukan keselarasan sumbu X dan Y:")
print(" - Tabung MERAH berada di kuadran (+X, +Y)")
print(" - Tabung HIJAU berada di kuadran (+X, -Y)")
print(" - Tabung BIRU berada di kuadran (-X, +Y)")
print(" - Tabung HITAM berada di kuadran (-X, -Y)")
print("=======================================")

# Loop simulasi agar GUI tetap aktif
try:
    while True:
        p.stepSimulation()
        time.sleep(0.01)
except KeyboardInterrupt:
    p.disconnect()
    print("Simulasi dihentikan.")
