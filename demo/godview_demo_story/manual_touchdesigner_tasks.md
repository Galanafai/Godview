# TouchDesigner Manual Tasks

Step-by-step guide for rendering the GodView demo in TouchDesigner.

---

## 1. Project Setup

### Create New Project
1. File → New
2. Set **FPS**: 30 (Edit → Project Properties → Frame Rate)
3. Set **Resolution**: 1920×1080

### Global Frame Counter
Create a **Count CHOP**:
- Name: `frame_counter`
- Limit: Type=Loop, Min=0, Max=2549 (total frames - 1)
- This is the SINGLE SOURCE OF TRUTH for playback. All components read from this.

---

## 2. NDJSON Import & Caching

### Preload Data (CRITICAL)
**Do NOT read from disk every frame.** Use this approach:

1. Create a **Script DAT** named `data_loader`:
```python
# data_loader callbacks
import json

def onStart():
    # Load all files into memory
    op('packets_table').clear()
    op('states_table').clear()
    op('events_table').clear()
    
    with open('path/to/packets.ndjson', 'r') as f:
        for i, line in enumerate(f):
            data = json.loads(line)
            op('packets_table').appendRow([
                data['frame'],
                data['agent_id'],
                data['delivery_frame'],
                data['signature_valid'],
                json.dumps(data['objects'])
            ])
    # Repeat for world_state.ndjson and events.ndjson
```

2. Create **Table DATs** to hold parsed data:
   - `packets_table`: Columns: `frame, agent_id, delivery_frame, signature_valid, objects_json`
   - `states_table`: Columns: `frame, objects_json`
   - `events_table`: Columns: `frame, event_type, payload_json`

3. Create **indexed lookups** using a **Script CHOP** or **Python** to filter by current frame.

---

## 3. Layout (3-Pane)

### Container Structure
```
root
├── pane_world (1280×1080, left)
├── pane_network (640×540, top-right)  
└── pane_data (640×540, bottom-right)
```

Use a **Container COMP** with **Panel COMP** or **Layout TOP** to arrange.

---

## 4. World View (Pane A)

### Agent Icons (Instanced)
1. Create a **Geo COMP** named `agent_instances`
2. Inside, create a **Rectangle SOP** (for cars) and **Circle SOP** (for pedestrians)
3. Add an **Instance CHOP**:
   - `tx, ty, tz`: Position from agent data
   - `rz`: Yaw rotation
   - `cr, cg, cb`: Color (red for BEFORE, green for AFTER)

### Driving Data
For each frame, query `packets_table` for unique `agent_id` values and their positions.

### Spotlight
Use `storyboard.md` to determine which agents are bright (alpha=1.0) vs dim (alpha=0.3).

### Detection Boxes
- **Dashed outline** for raw detections (BEFORE)
- **Solid outline** for canonical objects (AFTER)

### Covariance Ellipses
- Create a **Circle SOP** with non-uniform scale
- Scale X = `sqrt(covariance[0])`, Scale Y = `sqrt(covariance[3])`

---

## 5. Network View (Pane B)

### Node Layout
Arrange 21 nodes (20 agents + 1 drone) in a circle or grid.
- Use **instanced Circle SOPs**.
- Color: Green for normal, Red for `unknown_x`.

### Packet Animation
1. Filter `packets_table` for packets where:
   - `frame <= current_frame <= delivery_frame`
2. For each matching packet:
   - Source node: `agent_id` position
   - Dest node: Center or next agent
   - Interpolate position: `t = (current_frame - frame) / (delivery_frame - frame)`
3. Render as small bright dots.

### Spoof Rejection
When `event_type == "TRUST_REJECT"`:
- Show red packet hitting a "wall" (stationary flash at center)
- Packet disappears (don't render after event frame)

---

## 6. Data View (Pane C)

### Static Text
Create two **Text TOPs**:

#### Shared ✅
```
SHARED DATA:
✅ class
✅ pose (x, y, z, yaw)
✅ covariance
✅ timestamp
✅ signature
```
Color: Green (`#33FF57`)

#### Not Shared ❌
```
NOT SHARED:
❌ camera frames
❌ LiDAR point cloud
❌ video stream
```
Color: Red (`#FF3333`)

### Event Log (Optional)
Filter `events_table` for `frame == current_frame`.
Display latest event type and payload as scrolling text.

---

## 7. Captions

### Text Overlay
Use a **Text TOP** anchored to bottom-center.
- Font: Bold, 48px
- Color: White with black outline

### Caption Timing
Reference `storyboard.md` frame ranges. Use a **Switch TOP** or **Script** to change caption text based on `frame_counter`.

---

## 8. Recording

### Movie File Out TOP
1. Connect final composite to **Movie File Out TOP**
2. Settings:
   - File: `godview_demo.mp4`
   - Codec: H.264
   - Resolution: 1920×1080
   - FPS: 30
3. Record:
   - Set `frame_counter` to 0
   - Press Record
   - Run timeline from 0 to 2549
   - Stop recording

---

## 9. What NOT To Do

| ❌ Don't | ✅ Do Instead |
|----------|---------------|
| Read NDJSON every frame | Preload to Table DATs |
| Use wall-clock time | Use `frame_counter` CHOP |
| Create one Geo per object | Use instancing |
| Use complex shaders | Keep it flat/schematic |
| Add camera motion | Fixed orthographic view |

---

## Field → Visual Mapping

| NDJSON Field | TouchDesigner Element |
|--------------|----------------------|
| `packets.objects[].pose.position` | Instance tx, ty, tz |
| `packets.objects[].pose.yaw` | Instance rz |
| `packets.objects[].covariance` | Circle SOP scale |
| `packets.signature_valid` | Instance color (green/red) |
| `packets.delivery_frame` | Packet dot interpolation |
| `world_state.objects[].source_agents` | Determines box thickness |
| `events.event_type` | Triggers flash/animation |
