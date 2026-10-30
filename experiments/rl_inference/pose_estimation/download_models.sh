#!/usr/bin/env bash
# Download the cube detector's model files from the rwr_system GitHub release into this folder.
set -euo pipefail
cd "$(dirname "$0")"
URL=https://github.com/srl-ethz/rwr_system/releases/download/rwr2026-models
mkdir -p finetuned_models "pre_trained_cosypose_models/detector-bop-ycbv-synt+real--292971"
curl -fL -o finetuned_models/maskrcnn-finetuned_lightning_new_adamw.ckpt \
    "$URL/maskrcnn-finetuned_lightning_new_adamw.ckpt"
curl -fL -o "pre_trained_cosypose_models/detector-bop-ycbv-synt+real--292971/checkpoint.pth.tar" \
    "$URL/cosypose-detector-bop-ycbv-292971-checkpoint.pth.tar"
