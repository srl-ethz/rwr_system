# Cube detector

Copied from [srl-ethz/pose_estimation](https://github.com/srl-ethz/pose_estimation) at commit `db50f96` (2026-01-26), unchanged. Only the files `cube_detection_node.py` needs are included:

- `maskrcnn_detection.py`, `__init__.py`: the detector code
- `finetuned_models/maskrcnn-finetuned_lightning_new_adamw.ckpt`: the finetuned model (pass it as `model_path`; downloaded, see below)
- `pre_trained_cosypose_models/`: the base model, which the code loads first

The two model files (`*.ckpt`, `checkpoint.pth.tar`) are not in git. Download them once from the [GitHub release](https://github.com/srl-ethz/rwr_system/releases/tag/rwr2026-models):
```bash
./download_models.sh
```
