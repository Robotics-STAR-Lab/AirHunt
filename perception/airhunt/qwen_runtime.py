import json
import logging
import threading
import time
from collections import OrderedDict


def metric(event, **fields):
    logging.info(
        "QWEN_METRIC %s", json.dumps(dict(event=event, **fields), ensure_ascii=False)
    )


class PrefilterCache:

    def __init__(self, capacity=128):
        self.capacity = capacity
        self.data = OrderedDict()
        self.lock = threading.Lock()

    def resolve(self, key, loader):
        with self.lock:
            if key in self.data:
                self.data.move_to_end(key)
                metric("prefilter_cache_hit")
                return list(self.data[key])
        metric("prefilter_cache_miss")
        result = loader()
        if result is None:
            return None
        with self.lock:
            self.data[key] = tuple(result)
            self.data.move_to_end(key)
            while len(self.data) > self.capacity:
                self.data.popitem(last=False)
        return list(result)


def request_with_retries(create, kind, sleep=time.sleep, attempts=3):
    total_start = time.monotonic()
    for attempt in range(1, attempts + 1):
        started = time.monotonic()
        try:
            response = create()
            usage = getattr(response, "usage", None)
            metric(
                "api_success",
                kind=kind,
                attempt=attempt,
                attempt_s=time.monotonic() - started,
                total_s=time.monotonic() - total_start,
                prompt_tokens=getattr(usage, "prompt_tokens", None),
                completion_tokens=getattr(usage, "completion_tokens", None),
            )
            return response
        except Exception as exc:
            status = getattr(exc, "status_code", None)
            transient = (
                status in (408, 409, 429)
                or (isinstance(status, int) and status >= 500)
                or isinstance(exc, (TimeoutError, ConnectionError))
                or (type(exc).__name__ in ("APIConnectionError", "APITimeoutError"))
            )
            metric(
                "api_error",
                kind=kind,
                attempt=attempt,
                status=status,
                error_type=type(exc).__name__,
                elapsed_s=time.monotonic() - started,
            )
            if not transient:
                raise
            if attempt == attempts:
                return None
            delay = min(2 ** (attempt - 1), 4)
            headers = getattr(getattr(exc, "response", None), "headers", {})
            retry_after = headers.get("retry-after") if headers else None
            if retry_after:
                try:
                    requested = float(retry_after)
                    if requested > 5:
                        metric("retry_deferred", kind=kind, retry_after_s=requested)
                        return None
                    delay = max(delay, requested)
                except (TypeError, ValueError):
                    return None
            sleep(delay)
    return None
