from openai import OpenAI
from PIL import Image
import base64
from io import BytesIO
import os
import time
import logging
import numpy as np
import re
import json
import cv2
from airhunt.qwen_runtime import PrefilterCache, request_with_retries, metric

_prefilter_cache = PrefilterCache(capacity=128)
client = OpenAI(
    api_key=os.getenv("DASHSCOPE_API_KEY"),
    base_url=os.getenv(
        "DASHSCOPE_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"
    ),
)


def format_content(contents):
    formated_content = []
    for c in contents:
        formated_content.append({"type": "text", "text": c[0]})
        if len(c) == 2:
            formated_content.append(
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{c[1]}",
                        "detail": "high",
                    },
                }
            )
    return formated_content


def call_openai_api(sys_prompt, contents, request_kind="unspecified"):
    message_text = [
        {"role": "system", "content": sys_prompt},
        {"role": "user", "content": format_content(contents)},
    ]
    api = client.with_options(timeout=20.0, max_retries=0)
    completion = request_with_retries(
        lambda: api.chat.completions.create(
            model=os.getenv("DASHSCOPE_MODEL", "qwen3-vl-plus"),
            messages=message_text,
            temperature=float(os.getenv("QWEN_TEMPERATURE", "0")),
            max_tokens=1024,
            top_p=1.0,
            frequency_penalty=0,
            presence_penalty=0,
        ),
        request_kind,
    )
    return completion.choices[0].message.content if completion is not None else None


def task1_explore(
    step, cfg, verbose=False, logger=None, subtask_id=None, cnt_step=None
):
    step["use_prefiltering"] = cfg.prefiltering
    step["top_k_categories"] = cfg.top_k_categories
    question, egocentric_imgs = get_task1_info(step, verbose)
    if len(egocentric_imgs) == 0:
        return (None, None)
    sys_prompt, content = format_task1_prompt(question, egocentric_imgs)
    if verbose:
        logging.info(f"Task1 Input prompt:")
        message = sys_prompt
        for c in content:
            message += c[0]
            if len(c) == 2:
                message += f"[{c[1][:10]}...]"
        logging.info(message)
    retry_bound = 1
    begin_t = time.time()
    for _ in range(retry_bound):
        reply = call_openai_api(sys_prompt, content, request_kind="task1")
        if reply is None:
            print("Task1 call_openai_api returns None, retrying")
            continue
        logging.info(f"task1 orginal reply:{reply}")
        reply = reply.strip()
        task1_scores, task1_reason = parse_task1_reply(reply)
        end_t = time.time()
        logging.info(
            f"=====================Task1 call vlm time: {end_t - begin_t}==========================="
        )
        return (task1_scores, task1_reason)
    end_t = time.time()
    logging.info(f"Task1 call vlm time: {end_t - begin_t}")
    return (None, None)


def task2_explore(
    step, cfg, verbose=False, logger=None, subtask_id=None, cnt_step=None
):
    step["use_prefiltering"] = cfg.prefiltering
    step["top_k_categories"] = cfg.top_k_categories
    (
        question,
        snapshot_imgs,
        snapshot_classes,
        snapshot_crops,
        snapshot_id_mapping,
        snapshot_crop_mapping,
    ) = get_task2_info(step, verbose)
    if len(snapshot_imgs) == 0:
        return (None, None, snapshot_id_mapping, snapshot_crop_mapping)
    if logger is not None and subtask_id is not None and (cnt_step is not None):
        logger.save_snapshot_visualization(
            global_step=cnt_step,
            subtask_id=subtask_id,
            snapshot_imgs=snapshot_imgs,
            snapshot_classes=snapshot_classes,
            snapshot_crops=snapshot_crops,
        )
    sys_prompt, content = format_task2_prompt(
        question, snapshot_imgs, snapshot_classes, snapshot_crops
    )
    if verbose:
        logging.info(f"Task2 Input prompt:")
        message = sys_prompt
        for c in content:
            message += c[0]
            if len(c) == 2:
                message += f"[{c[1][:10]}...]"
        logging.info(message)
    retry_bound = 1
    begin_t = time.time()
    for _ in range(retry_bound):
        reply = call_openai_api(sys_prompt, content, request_kind="task2")
        if reply is None:
            print("Task2 call_openai_api returns None, retrying")
            continue
        logging.info(f"task2 original reply:{reply}")
        reply = reply.strip()
        task2_result, task2_reason = parse_task2_reply(reply)
        if task2_result is not None and validate_snapshot_selection(
            task2_result, snapshot_classes, len(snapshot_imgs)
        ):
            end_t = time.time()
            logging.info(f"Task2 call vlm time: {end_t - begin_t}")
            return (
                task2_result,
                task2_reason,
                snapshot_id_mapping,
                snapshot_crop_mapping,
            )
        else:
            raise ValueError(f"Task2 validation failed, retrying...")
            continue
    end_t = time.time()
    logging.info(f"Task2 call vlm time: {end_t - begin_t}")
    return (None, None, snapshot_id_mapping, snapshot_crop_mapping)


def validate_snapshot_selection(result, snapshot_classes, num_snapshots):
    if result == "no snapshot interested":
        return True
    if not isinstance(result, tuple) or len(result) != 2:
        return False
    snapshot_index, object_index = result
    if (
        not isinstance(snapshot_index, int)
        or snapshot_index < 0
        or snapshot_index >= num_snapshots
    ):
        return False
    snapshot_ids = list(snapshot_classes.keys())
    if snapshot_index >= len(snapshot_ids):
        return False
    selected_snapshot_id = snapshot_ids[snapshot_index]
    num_objects_in_snapshot = len(snapshot_classes[selected_snapshot_id])
    if (
        not isinstance(object_index, int)
        or object_index < 0
        or object_index >= num_objects_in_snapshot
    ):
        return False
    return True


def get_task1_info(step, verbose=False):
    question = step["question"]
    egocentric_imgs = {}
    for rgb_id in step["egocentric_imgs"].keys():
        egocentric_img = step["egocentric_imgs"][rgb_id]["full_img"]
        egocentric_imgs[rgb_id] = encode_tensor2base64(egocentric_img)
    return (question, egocentric_imgs)


def get_task2_info(step, verbose=False):
    question = step["question"]
    snapshot_classes = {}
    snapshot_imgs = {}
    snapshot_crops = {}
    snapshot_clusters = {}
    obj_map = step["obj_map"]
    seen_classes = set()
    for rgb_id in step["snapshot_imgs"].keys():
        snapshot_img = step["snapshot_imgs"][rgb_id]["full_img"]
        snapshot_imgs[rgb_id] = encode_tensor2base64(snapshot_img)
        snapshot_crops[rgb_id] = [
            encode_tensor2base64(resize_small_crop(crop_data["crop"]))
            for crop_data in step["snapshot_imgs"][rgb_id]["object_crop"]
        ]
        snapshot_class = [
            crop_data["obj_class"]
            for crop_data in step["snapshot_imgs"][rgb_id]["object_crop"]
        ]
        cluster_class = [
            obj_map[int(obj_id)] for obj_id in step["snapshot_objects"][rgb_id]
        ]
        seen_classes.update(sorted(list(set(snapshot_class))))
        snapshot_classes[rgb_id] = snapshot_class
        snapshot_clusters[rgb_id] = cluster_class
    keep_index = list(range(len(snapshot_imgs)))
    keep_index_snapshot = {
        rgb_id: list(range(len(snapshot_crops[rgb_id]))) for rgb_id in snapshot_crops
    }
    if step.get("use_prefiltering") is True:
        n_prev_snapshot = len(snapshot_imgs)
        snapshot_classes, keep_index, keep_index_snapshot = prefiltering(
            question,
            snapshot_classes,
            snapshot_clusters,
            seen_classes,
            step["top_k_categories"],
            verbose=verbose,
        )
        snapshot_imgs = {
            rgb_id: snapshot_imgs[rgb_id] for rgb_id in keep_index_snapshot.keys()
        }
        for rgb_id in snapshot_classes.keys():
            snapshot_crops[rgb_id] = [
                snapshot_crops[rgb_id][i] for i in keep_index_snapshot[rgb_id]
            ]
        if verbose:
            logging.info(
                f"Prefiltering snapshot: {n_prev_snapshot} -> {len(snapshot_imgs)}"
            )
    for rgb_id, indices in keep_index_snapshot.items():
        full_img = step["snapshot_imgs"][rgb_id]["full_img"].copy()
        for display_index, original_index in enumerate(indices):
            box = step["snapshot_imgs"][rgb_id]["object_crop"][original_index]["bbox"]
            x1, y1, x2, y2 = box
            cv2.rectangle(full_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(full_img, str(display_index), (x1, max(12, y1 - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        snapshot_imgs[rgb_id] = encode_tensor2base64(full_img)
    return (
        question,
        snapshot_imgs,
        snapshot_classes,
        snapshot_crops,
        keep_index,
        keep_index_snapshot,
    )


def prefiltering(
    question, snapshot_classes, snapshot_clusters, seen_classes, top_k=10, verbose=False
):
    selected_classes = get_prefiltering_classes(question, seen_classes, top_k)
    if verbose:
        logging.info(f"Prefiltering selected classes: {selected_classes}")
    keep_index = [
        k
        for i, k in enumerate(snapshot_clusters.keys())
        if len(set(snapshot_clusters[k]) & set(selected_classes)) > 0
    ]
    keep_snapshot_id = keep_index
    snapshot_classes = {rgb_id: snapshot_classes[rgb_id] for rgb_id in keep_snapshot_id}
    keep_index_snapshot = {}
    for rgb_id in keep_snapshot_id:
        keep_index_snapshot[rgb_id] = [
            i
            for i in range(len(snapshot_classes[rgb_id]))
            if snapshot_classes[rgb_id][i] in selected_classes
        ]
        snapshot_classes[rgb_id] = [
            snapshot_classes[rgb_id][i] for i in keep_index_snapshot[rgb_id]
        ]
    return (snapshot_classes, keep_index, keep_index_snapshot)


def get_prefiltering_classes(question, seen_classes, top_k=10):
    classes = tuple(sorted(set(seen_classes)))
    if not classes:
        metric("prefilter_empty_skip")
        return []
    key = (
        os.getenv("DASHSCOPE_BASE_URL", ""),
        os.getenv("DASHSCOPE_MODEL", "qwen3-vl-plus"),
        question,
        classes,
        top_k,
    )

    def load_selection():
        sys_prompt, content = format_prefiltering_prompt(question, list(classes))
        response = call_openai_api(sys_prompt, content, request_kind="prefilter")
        if response is None or not response.strip():
            return None
        selected = [line.strip() for line in response.strip().split("\n")]
        return [name for name in selected if name in classes][:top_k]

    if os.getenv("QWEN_PREFILTER_CACHE", "1") == "0":
        return load_selection() or []
    return _prefilter_cache.resolve(key, load_selection) or []


def format_prefiltering_prompt(question, class_list=None):
    content = []
    sys_prompt = "You are an AI agent navigating an outdoor environment.\n"
    prompt = (
        "Your goal is to answer questions about the scene through active exploration.\n"
    )
    prompt += "To efficiently solve the problem, you should first identify objects in the scene that are most relevant to the question.\n"
    prompt += "These are the rules for the task:\n"
    prompt += "1. Read through the complete list of available objects.\n"
    prompt += "2. Rank and select the objects that are most likely to help you answer the question.\n"
    prompt += "3. Only output the names of helpful objects from the list.\n"
    prompt += "4. Do not add any extra objects or explanation in your response.\n"
    content.append((prompt,))
    prompt = "Here is an example of selecting helpful objects:\n"
    prompt += "Question: Where can I sit and rest for a while?\n"
    prompt += (
        "Following is a list of objects that you can choose, each object one line:\n"
    )
    prompt += "lamp post\ntree\ntrash can\nbench\nsculpture\nbike rack\nnewsstand\n"
    prompt += "Answer: bench\n"
    content.append((prompt,))
    prompt = f"Following is the concrete content of the task. You should retrieve helpful objects in order:\n"
    prompt += f"Question: {question}"
    content.append((prompt + "\n",))
    prompt = (
        "Following is a list of objects that you can choose, each object one line:\n"
    )
    for cls in class_list:
        prompt += f"{cls}\n"
    prompt += "Answer: "
    content.append((prompt,))
    return (sys_prompt, content)


def format_task1_prompt(question, egocentric_imgs):
    sys_prompt = "You are an agent assisting with outdoor object navigation in a outdoor environment. Your task is to analyze frontier images representing different unexplored directions and estimate a distinct probability for each direction indicating how likely the target object will be found by exploring in that direction.\nDo not include any explanation, reasoning, or commentary in your response—only the final probability estimates.\n\n"
    content = []
    text = "Definitions:\nFrontier: An unexplored region that could potentially lead to new information for answering the question. Assigning a high probability to a frontier suggests you will prioritize exploration in that direction.\n\n"
    text += f"Question: {question}\n\n"
    content.append((text,))
    text = "Task: Frontier Analysis\nCRITICAL REQUIREMENTS:\n1. Provide UNIQUE probability scores for each direction - no duplicate scores are allowed\n2. Base scores on concrete visual evidence and environmental analysis\n3. Consider both immediate visibility and potential hidden locations\n4. Account for typical urban object placement patterns\n5. Assess the overall exploration potential of each direction\n\nSCORING GUIDELINES:\n- 0.0-0.2: Very unlikely - poor visibility, no object-related features\n- 0.2-0.4: Unlikely - limited visibility or few object-related indicators\n- 0.4-0.6: Moderate possibility - some relevant features but uncertain\n- 0.6-0.8: Likely - good visibility and clear object-related environmental features\n- 0.8-1.0: Very likely - excellent visibility and strong indicators of object presence\n\n"
    text += "The following are all the Frontier images that you can explore:\n"
    content.append((text,))
    for i, rgb_id in enumerate(egocentric_imgs.keys()):
        content.append((f"Frontier {i}: ", egocentric_imgs[rgb_id]))
        content.append(("\n",))
    text = f"Based on your comprehensive analysis of all {len(egocentric_imgs)} exploration directions, provide a list of probability scores indicating the likelihood of finding the target object in each direction. \nFormat your response EXACTLY as following example, rememer belows are just examples:\nSCORES: [0.35, 0.72, 0.18, ...]\nIMPORTANT: Provide exactly {len(egocentric_imgs)} scores indicating the likelihood of finding the target object in each direction,the scores can be any float from 0 to 1 depending on your analysis.\nEnsure that all {len(egocentric_imgs)} scores are unique and justified based on your analysis.REMEMBER: DO NOT PROVIDE ANY REASON AND JUSTIFICATION!!!!\n\n"
    content.append((text,))
    return (sys_prompt, content)


def format_task2_prompt(question, snapshot_imgs, snapshot_classes, snapshot_crops):
    sys_prompt = (
        "Verify a visual object-navigation target using the scene images and candidate crops. "
        "Detector labels are unverified proposals and often wrong. Do not infer an object's identity "
        "from its detector label. Check the visible shape, all requested attributes such as colors, "
        "and the object's context in the scene. A similarly colored object of a different type is "
        "not a match. Use the numbered box in the full scene to interpret each context crop. Reject visually inconsistent candidates. "
        "Select a candidate only when its crop itself and the full scene support the requested identity.\n"
    )
    content = []
    text = "Definitions:\nSnapshot: A focused observation of several objects. It contains a full image of the cluster of objects, and separate image crops of each object. Choosing a snapshot means that the object asked in the question is within the cluster of objects that the snapshot represents, and you will choose that object as the final answer to the question.\n\n"
    text += f"Question: {question}\n\n"
    content.append((text,))
    text = (
        "Each full scene has numbered boxes corresponding to Object 0, Object 1, etc. "
        "Each enlarged context crop contains the candidate and a small surrounding margin. "
        "Use its numbered location in the full scene to resolve small or partial views. "
        "Do not select a different object outside the numbered box. Detector labels may be wrong.\n"
    )
    content.append((text,))
    for i, rgb_id in enumerate(snapshot_imgs.keys()):
        content.append((f"Snapshot {i}: ", snapshot_imgs[rgb_id]))
        for j in range(len(snapshot_crops[rgb_id])):
            content.append(
                (
                    f"Object {j} (unverified detector label: {snapshot_classes[rgb_id][j]}): ",
                    snapshot_crops[rgb_id][j],
                )
            )
        content.append(("\n",))
    text = (
        "First write REASON: followed by one sentence describing the visible shape, colors and "
        "context of the candidate you would select, or why the proposals do not match. "
        "Then write exactly one final line: RESULT: Snapshot i, Object j, or "
        "RESULT: No snapshot interested. Reject proposals that do not visibly satisfy the question.\n"
    )
    content.append((text,))
    return (sys_prompt, content)


def parse_task1_reply(reply):
    task1_scores = None
    task1_reason = []
    lines = reply.split("\n")
    for i, line in enumerate(lines):
        line = line.strip()
        if line.startswith("SCORES:"):
            scores_str = line.replace("SCORES:", "").strip()
            try:
                task1_scores = json.loads(scores_str)
            except:
                match = re.search("\\[([\\d\\., ]+)\\]", scores_str)
                if match:
                    try:
                        task1_scores = json.loads("[" + match.group(1) + "]")
                    except:
                        pass
        elif line.startswith("REASON:"):
            frontier_reasons = {}
            current_frontier_reason = []
            for j in range(i + 1, len(lines)):
                next_line = lines[j].strip()
                if next_line.startswith("SCORES:") or next_line.startswith("REASON:"):
                    break
                if next_line.startswith("FRONTIER_"):
                    match = re.search("FRONTIER_(\\d+): (.+)", next_line)
                    if match:
                        frontier_idx = int(match.group(1))
                        frontier_reasons[frontier_idx] = match.group(2).strip()
                elif next_line.startswith("- **Frontier"):
                    if current_frontier_reason:
                        task1_reason.append(" ".join(current_frontier_reason))
                        current_frontier_reason = []
                    match = re.search("- \\*\\*Frontier (\\d+)\\*\\*: (.+)", next_line)
                    if match:
                        current_frontier_reason.append(match.group(2).strip())
                    else:
                        match = re.search(
                            "- \\*\\*Frontier \\d+ \\(Score: [\\d\\.]+\\)\\*\\*: (.+)",
                            next_line,
                        )
                        if match:
                            current_frontier_reason.append(match.group(1).strip())
                elif next_line.startswith("**Frontier"):
                    if current_frontier_reason:
                        task1_reason.append(" ".join(current_frontier_reason))
                        current_frontier_reason = []
                    match = re.search("\\*\\*Frontier (\\d+)\\*\\*: (.+)", next_line)
                    if match:
                        current_frontier_reason.append(match.group(2).strip())
                elif next_line.startswith("Frontier"):
                    if current_frontier_reason:
                        task1_reason.append(" ".join(current_frontier_reason))
                        current_frontier_reason = []
                    match = re.search("Frontier (\\d+).*?: (.+)", next_line)
                    if match:
                        current_frontier_reason.append(match.group(2).strip())
                elif (
                    next_line
                    and (not next_line.startswith("This score is"))
                    and current_frontier_reason
                ):
                    current_frontier_reason.append(next_line)
            if current_frontier_reason:
                task1_reason.append(" ".join(current_frontier_reason))
            if frontier_reasons:
                task1_reason = []
                for i in range(max(frontier_reasons.keys()) + 1):
                    if i in frontier_reasons:
                        task1_reason.append(frontier_reasons[i])
                    else:
                        task1_reason.append("")
    if task1_scores is not None and len(task1_reason) != len(task1_scores):
        logging.info(
            f"Warning: task1_reason length ({len(task1_reason)}) doesn't match task1_scores length ({len(task1_scores)})"
        )
        if len(task1_reason) > len(task1_scores):
            task1_reason = task1_reason[: len(task1_scores)]
        else:
            while len(task1_reason) < len(task1_scores):
                task1_reason.append("")
    return (task1_scores, task1_reason)


def parse_task2_reply(reply):
    match = re.search(
        r"\bRESULT:\s*(No snapshot interested|Snapshot\s+(\d+)\s*,?\s*Object\s+(\d+))",
        reply,
        re.IGNORECASE,
    )
    result = None
    if match:
        result = (
            "no snapshot interested"
            if match.group(2) is None
            else (int(match.group(2)), int(match.group(3)))
        )
    reason = re.search(r"\bREASON:\s*(.*?)(?=\s*RESULT:|\Z)", reply, re.DOTALL | re.IGNORECASE)
    return result, reason.group(1).strip() if reason else None


def resize_small_crop(crop_img, min_size=128):
    h, w = crop_img.shape[:2]
    if min(h, w) <= 0:
        raise ValueError("Empty object crop")
    scale = min(min_size / min(h, w), 512 / max(h, w))
    if scale > 1:
        return np.array(Image.fromarray(crop_img).resize(
            (max(1, round(w * scale)), max(1, round(h * scale))), Image.Resampling.BICUBIC
        ))
    return crop_img


def encode_tensor2base64(img):
    img = Image.fromarray(img)
    buffer = BytesIO()
    img.save(buffer, format="PNG")
    buffer.seek(0)
    img_base64 = base64.b64encode(buffer.read()).decode("utf-8")
    return img_base64
