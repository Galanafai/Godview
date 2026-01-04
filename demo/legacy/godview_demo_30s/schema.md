# NDJSON Schema - 30s Demo

---

## packets_*.ndjson

```json
{
  "packet_type": "DETECTION",
  "frame": 100,
  "timestamp_ns": 3333333333,
  "agent_id": "agent_A",
  "delivery_frame": 100,
  "signature_valid": true,
  "objects": [
    {
      "local_object_id": "A_ped_00",
      "class": "pedestrian",
      "pose": {"position": {"x": 12.0, "y": 8.0, "z": 0}, "yaw": 0},
      "covariance": [0.3, 0, 0, 0.3],
      "confidence": 0.85
    }
  ]
}
```

---

## world_*.ndjson

```json
{
  "packet_type": "CANONICAL_STATE",
  "frame": 100,
  "timestamp_ns": 3333333333,
  "objects": [
    {
      "canonical_object_id": "canonical_ped_1",
      "class": "pedestrian",
      "pose": {"position": {"x": 12.0, "y": 8.0, "z": 0}, "yaw": 0},
      "covariance": [0.15, 0, 0, 0.15],
      "source_agents": ["agent_A", "drone"],
      "confidence": 0.95
    }
  ]
}
```

---

## events_after.ndjson

| event_type | payload |
|------------|---------|
| PACKET_ARRIVAL | `{src, dst, object_id}` |
| MERGE | `{from_ids, canonical_id}` |
| TRUST_REJECT | `{agent_id, reason}` |
| OOSM_CORRECTED | `{agent_id, delayed_frames}` |
