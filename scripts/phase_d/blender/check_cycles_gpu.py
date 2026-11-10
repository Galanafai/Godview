"""D5 prep: confirm that Cycles renders on the GPU in headless Blender.

Run: blender -b --factory-startup -P check_cycles_gpu.py -- OUT.png
Enables OptiX (falls back to CUDA), lists the devices, renders the default scene
(cube, light, camera) at 960 x 540 with 64 samples and prints the device used and the
render time.
"""
import sys
import time

import bpy

out = sys.argv[sys.argv.index('--') + 1] if '--' in sys.argv else '/tmp/check.png'
prefs = bpy.context.preferences.addons['cycles'].preferences
chosen = None
for kind in ('OPTIX', 'CUDA'):
    try:
        prefs.compute_device_type = kind
        prefs.refresh_devices() if hasattr(prefs, 'refresh_devices') else prefs.get_devices()
        devs = [d for d in prefs.devices if d.type == kind]
        if devs:
            for d in prefs.devices:
                d.use = d.type == kind
            chosen = kind
            break
    except Exception as e:  # noqa: BLE001
        print('backend', kind, 'unavailable:', e)
print('devices:', [(d.name, d.type, d.use) for d in prefs.devices])
print('compute backend chosen:', chosen)
scene = bpy.context.scene
scene.render.engine = 'CYCLES'
scene.cycles.device = 'GPU'
scene.cycles.samples = 64
scene.render.resolution_x, scene.render.resolution_y = 960, 540
scene.render.resolution_percentage = 100
scene.render.filepath = out
t0 = time.time()
bpy.ops.render.render(write_still=True)
print('render seconds: %.2f' % (time.time() - t0))
print('version', bpy.app.version_string)
