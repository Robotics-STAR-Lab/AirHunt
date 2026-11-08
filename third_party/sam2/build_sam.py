# Copyright (c) Meta Platforms, Inc. and affiliates. Apache-2.0.
# Adapted: retain image-model loading only.
import logging
import torch
from hydra import compose
from hydra.utils import instantiate
from omegaconf import OmegaConf

def build_sam2(config_file, ckpt_path=None, device='cuda', mode='eval', hydra_overrides_extra=[], apply_postprocessing=True, **kwargs):
    if apply_postprocessing:
        hydra_overrides_extra = hydra_overrides_extra.copy()
        hydra_overrides_extra += ['++model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability=true', '++model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta=0.05', '++model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh=0.98']
    cfg = compose(config_name=config_file, overrides=hydra_overrides_extra)
    OmegaConf.resolve(cfg)
    model = instantiate(cfg.model, _recursive_=True)
    _load_checkpoint(model, ckpt_path)
    model = model.to(device)
    if mode == 'eval':
        model.eval()
    return model

def _load_checkpoint(model, ckpt_path):
    if ckpt_path is not None:
        sd = torch.load(ckpt_path, map_location='cpu', weights_only=True)['model']
        missing_keys, unexpected_keys = model.load_state_dict(sd)
        if missing_keys:
            logging.error(missing_keys)
            raise RuntimeError()
        if unexpected_keys:
            logging.error(unexpected_keys)
            raise RuntimeError()
        logging.info('Loaded checkpoint sucessfully')
