# NDJSON Schema Specification

All records are single-line JSON objects in NDJSON format.

---

## packets.ndjson

### Record Type: `DETECTION`

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `packet_type` | string | ✅ | Always `"DETECTION"` |
| `frame` | int | ✅ | Sensing frame number (0-indexed) |
| `timestamp_ns` | int | ✅ | Sensing time in nanoseconds |
| `agent_id` | string | ✅ | Agent identifier (e.g., `agent_00`, `drone_00`) |
| `delivery_frame` | int | ✅ | Frame when packet arrives (for OOSM modeling) |
| `signature_valid` | bool | ✅ | `true` for legitimate agents, `false` for spoof |
| `objects` | array | ✅ | List of detected objects |

### Object Schema (within `objects` array)

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `local_object_id` | string | ✅ | Agent-local object ID |
| `class` | string | ✅ | Object class: `car`, `pedestrian`, `cyclist`, `truck`, `bus` |
| `pose` | object | ✅ | Position and orientation |
| `pose.position` | object | ✅ | `{x, y, z}` in meters |
| `pose.yaw` | float | ✅ | Heading in radians |
| `covariance` | array | ✅ | 2×2 covariance matrix, flattened: `[σxx, σxy, σyx, σyy]` |
| `velocity` | object | ❌ | Optional: `{x, y, z}` in m/s |

### Example

```json
{"packet_type":"DETECTION","frame":150,"timestamp_ns":5000000000,"agent_id":"agent_02","delivery_frame":150,"signature_valid":true,"objects":[{"local_object_id":"agent_02_ped_01","class":"pedestrian","pose":{"position":{"x":5.12,"y":2.03,"z":0.0},"yaw":0.05},"covariance":[0.5,0.0,0.0,0.5],"velocity":{"x":0.5,"y":0.0,"z":0.0}}]}
```

---

## world_state.ndjson

### Record Type: `CANONICAL_STATE`

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `packet_type` | string | ✅ | Always `"CANONICAL_STATE"` |
| `frame` | int | ✅ | Frame number |
| `timestamp_ns` | int | ✅ | Canonical timestamp |
| `objects` | array | ✅ | List of fused canonical objects |

### Canonical Object Schema

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `canonical_object_id` | string | ✅ | Global unique ID (result of Highlander merge) |
| `class` | string | ✅ | Object class |
| `pose` | object | ✅ | Fused position and orientation |
| `covariance` | array | ✅ | Fused 2×2 covariance |
| `source_agents` | array | ✅ | List of agent IDs that contributed observations |
| `confidence` | float | ✅ | Confidence score [0.0, 1.0] |

### Example

```json
{"packet_type":"CANONICAL_STATE","frame":150,"timestamp_ns":5000000000,"objects":[{"canonical_object_id":"canonical_ped_01","class":"pedestrian","pose":{"position":{"x":5.08,"y":2.01,"z":0.0},"yaw":0.02},"covariance":[0.25,0.0,0.0,0.25],"source_agents":["agent_00","agent_02","drone_00"],"confidence":0.85}]}
```

---

## events.ndjson

### Record Type: `EVENT`

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `packet_type` | string | ✅ | Always `"EVENT"` |
| `frame` | int | ✅ | Frame when event occurred |
| `event_type` | string | ✅ | One of: `MERGE`, `TRUST_REJECT`, `OOSM_CORRECTED`, `HANDOFF_OK`, `SPACE_SEPARATION` |
| `payload` | object | ✅ | Event-specific data |

### Event Payloads

#### `MERGE`
```json
{"from_ids": ["agent_02_car_01", "agent_03_car_01"], "to_id": "canonical_car_01", "reason": "spatial_gating_highlander"}
```

#### `TRUST_REJECT`
```json
{"agent_id": "unknown_x", "object_id": "spoof_bus_01", "reason": "invalid_signature"}
```

#### `OOSM_CORRECTED`
```json
{"agent_id": "agent_07", "delayed_frames": 10, "correction_magnitude_m": 0.8}
```

#### `HANDOFF_OK`
```json
{"object_id": "canonical_car_01", "from_agent": "agent_05", "to_agent": "agent_06", "id_stable": true}
```

#### `SPACE_SEPARATION`
```json
{"drone_id": "drone_00", "car_id": "agent_03", "delta_z": 15.0}
```

### Example

```json
{"packet_type":"EVENT","frame":1140,"event_type":"MERGE","payload":{"from_ids":["agent_02_car_target_01","agent_03_car_target_01"],"to_id":"canonical_car_target_01","reason":"spatial_gating_highlander"}}
```
