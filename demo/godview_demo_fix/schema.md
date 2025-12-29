# NDJSON Schema - GodView Demo Fix

---

## packets.ndjson

### DETECTION Record

| Field | Type | Description |
|-------|------|-------------|
| `packet_type` | string | Always `"DETECTION"` |
| `frame` | int | Sensing frame |
| `timestamp_ns` | int | Sensing time (nanoseconds) |
| `agent_id` | string | `agent_00`...`agent_19`, `drone_00`, `unknown_x` |
| `delivery_frame` | int | Frame when packet arrives (for OOSM) |
| `signature_valid` | bool | `false` for spoof agents |
| `objects` | array | List of detected objects |

### Object Schema

| Field | Type | Description |
|-------|------|-------------|
| `local_object_id` | string | Agent-local ID |
| `class` | string | `car`, `pedestrian`, `cyclist`, `truck`, `cone`, `barrier` |
| `pose.position` | `{x,y,z}` | Position (meters) |
| `pose.yaw` | float | Heading (radians) |
| `covariance` | `[a,b,c,d]` | 2×2 flattened |
| `velocity` | `{x,y,z}` | Velocity (m/s) |

### Example
```json
{"packet_type":"DETECTION","frame":100,"timestamp_ns":3333333333,"agent_id":"agent_02","delivery_frame":100,"signature_valid":true,"objects":[{"local_object_id":"agent_02_ped_01","class":"pedestrian","pose":{"position":{"x":12.1,"y":8.2,"z":0},"yaw":0.05},"covariance":[0.4,0,0,0.4],"velocity":{"x":-0.5,"y":0,"z":0}}]}
```

---

## world_state.ndjson

### CANONICAL_STATE Record

| Field | Type | Description |
|-------|------|-------------|
| `packet_type` | string | Always `"CANONICAL_STATE"` |
| `frame` | int | Frame number |
| `timestamp_ns` | int | Timestamp |
| `objects` | array | Fused canonical objects |

### Canonical Object

| Field | Type | Description |
|-------|------|-------------|
| `canonical_object_id` | string | Global ID |
| `class` | string | Object class |
| `pose` | object | Fused position + yaw |
| `covariance` | array | Fused 2×2 |
| `source_agents` | array | Contributing agent IDs |
| `confidence` | float | 0.0–1.0 |

---

## events.ndjson

### EVENT Record

| Field | Type | Description |
|-------|------|-------------|
| `packet_type` | string | Always `"EVENT"` |
| `frame` | int | Event frame |
| `event_type` | string | `MERGE`, `TRUST_REJECT`, `OOSM_CORRECTED`, `SPACE_SEPARATION`, `PACKET_ARRIVAL` |
| `payload` | object | Event-specific data |

### Event Types

| Type | Payload |
|------|---------|
| `MERGE` | `{from_ids, to_id, reason}` |
| `TRUST_REJECT` | `{agent_id, object_id, reason}` |
| `OOSM_CORRECTED` | `{agent_id, delayed_frames}` |
| `SPACE_SEPARATION` | `{drone_id, car_id, delta_z}` |
| `PACKET_ARRIVAL` | `{from_agent, to_agent, object_id, causes}` |
