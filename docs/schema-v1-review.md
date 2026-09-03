# `lidar-camera-scene/1` schema approval

Status: **approved and frozen**
Date: 2026-09-01

## Review result

All seven schemas pass Draft 2020-12 meta-schema validation and match the writer/reader topic, schema-title, encoding, required-field, and cardinality contracts. The final pre-freeze review:

- constrained scene and source timestamps to non-negative integer nanoseconds;
- confirmed signed synchronization deltas remain permitted;
- confirmed strict `additionalProperties: false` object boundaries;
- confirmed canonical camera-name patterns and topic forms;
- confirmed row-major 3×3, 3×4, and 4×4 flattened matrix lengths;
- confirmed fixed little-endian float32 XYZI shape/field metadata;
- confirmed PNG/JPEG and base64 payload declarations;
- corrected the reader to consume the schema field `projection_matrix` rather than the unused spelling `projection`;
- retained runtime validation for invariants JSON Schema cannot express cleanly, including finite values, rigid transforms, graph connectivity, camera uniqueness/order, payload byte lengths, and complete cross-topic frame identity.

## Frozen resource hashes

| Schema | SHA-256 |
|---|---|
| `camera-calibration.schema.json` | `0aaa1d7f82b172d6f19b377071c3f5c3d6a0ae88ffc48893c1634a93b5fad9d6` |
| `camera-image-frame.schema.json` | `d41d012dd7a4257b6cc7936d52f66bd6bc61f9da56641dd1caa8261b67033747` |
| `ego-pose-frame.schema.json` | `60458d7664434ad441318bd57622719301a97864e7cc3b0756f9fc1086efbc38` |
| `point-cloud-frame.schema.json` | `6ddf06824f129e139bac5dea0b269f6a1664b23a5ee83e11ac8b1c35008391cf` |
| `scene-frame.schema.json` | `a1d4446952236777ae69819fb08dd313c5dbc745389a95c5068cfd7db04bab26` |
| `scene-manifest.schema.json` | `126ab79b87a8ebd2fba14d92161b891436d3e891ac85fe9286d3b3cc04089501` |
| `static-transform-tree.schema.json` | `55355c0dcce2f926176ac4e517af609fda1807204788991404c789ed45038ce6` |

## Compatibility policy

The serialized contract and these schema resources are immutable for profile version 1. A change to required fields, field meaning, topic identity, matrix direction, payload encoding, or accepted serialized values requires a new profile version and migration path. Reader bug fixes and stricter enforcement of already-documented v1 invariants do not redefine valid v1 data.
