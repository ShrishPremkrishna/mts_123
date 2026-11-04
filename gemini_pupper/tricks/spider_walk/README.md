# Selected Lab 6 Spider walk

This pair was selected for the R4 through R6 Lab 6 releases. The trick policy plays the Spider entry and exit clips. The walking policy receives velocity commands in the low stance. The stock `spider` descriptor already declares this skill as its replacement. The loader selects `spider_walk` when both directories are present.

| Artifact | Training origin |
| --- | --- |
| `policy.json` | Team Spider clip tracking run, `model_3998.pt`. [Training, export and replay](../../../foundation/results/lab6-spider-corrected.json) |
| `walk_policy.json` | Team Spider walker, deployment-measured encoder observation experiment, `model_9494.pt`. [Training comparison](../../../foundation/results/spider-encoder-observation-study-reviewed.json) |

The four checksums in `artifact_manifest.json` identify both policies and their exact configuration. Its original host paths record where the export was assembled. The runtime uses the relative filenames in `trick.yaml`.

`walk_limits.json` gives commanded speed caps and measured direction errors. R4/R5 floor sessions demonstrated entry, low walking and a standing return. The eight simulated adverse returns still failed the quiet-return check. Read the historical flags in the descriptors with the later [physical observations](../../../foundation/results/harness-sprint-20261103.json) and [native comparison](../../../foundation/results/spider-encoder-native-reviewed.json).
