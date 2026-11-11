# rig_preprocess

This package prepares a user's rigged asset so UniMate can animate it. Shared principles are in the [root AGENTS.md](../../../AGENTS.md). The CLI and its options are in [README.md](README.md). Labeling conventions and review guidance are in [ANNOTATION_GUIDE.md](ANNOTATION_GUIDE.md).

Run the module from the UniMate root in the documented `unimate` environment. It takes `.glb`, `.gltf` or `.fbx` input and writes `cond.npy`, `summary.json`, a canonical `.glb` when the input has a mesh, and optional `motions/` with `--save_clips`.

Annotation uses an LLM by default when one is configured; keys are kept external. Otherwise it falls back to offline rules. A normal `run` stops after writing review material. Running again with `--annotation` builds the asset from the corrected annotation file.

When editing annotation, you may change joint labels and `face_pair`. Keep raw names and joint order unchanged, because the code validates against them.
