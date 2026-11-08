import torch
from PIL import Image


def compute_clip_features_batched(
    image,
    detections,
    clip_model,
    clip_preprocess,
    clip_tokenizer,
    classes,
    device,
    prompt_h=None,
    prompt_w=None,
):
    image = Image.fromarray(image)
    padding = 20
    image_crops = []
    preprocessed_images = []
    text_tokens = []
    original_width, original_height = image.size
    if prompt_h is not None and prompt_w is not None:
        scale_w, scale_h = (prompt_w / original_width, prompt_h / original_height)
    else:
        scale_w, scale_h = (1, 1)
    for idx in range(len(detections.xyxy)):
        x_min, y_min, x_max, y_max = detections.xyxy[idx]
        image_width, image_height = image.size
        left_padding = min(padding, x_min)
        top_padding = min(padding, y_min)
        right_padding = min(padding, image_width - x_max)
        bottom_padding = min(padding, image_height - y_max)
        image_crops.append(image.crop((x_min, y_min, x_max, y_max)))
        x_min -= left_padding
        y_min -= top_padding
        x_max += right_padding
        y_max += bottom_padding
        preprocessed_image = clip_preprocess(
            image.crop((x_min, y_min, x_max, y_max))
        ).unsqueeze(0)
        preprocessed_images.append(preprocessed_image)
        class_id = detections.class_id[idx]
        text_tokens.append(classes[class_id])
    preprocessed_images_batch = torch.cat(preprocessed_images, dim=0).to(device)
    text_tokens_batch = clip_tokenizer(text_tokens).to(device)
    with torch.no_grad():
        image_features = clip_model.encode_image(preprocessed_images_batch)
        image_features /= image_features.norm(dim=-1, keepdim=True)
    image_feats = image_features.cpu().numpy()
    text_feats = []
    if prompt_h is not None and prompt_w is not None:
        for i in range(len(image_crops)):
            crop_w, crop_h = image_crops[i].size
            image_crops[i] = image_crops[i].resize(
                (int(crop_w * scale_w), int(crop_h * scale_h))
            )
    return (image_crops, image_feats, text_feats)
