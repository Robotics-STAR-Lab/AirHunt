from pathlib import Path
import sysconfig

p = Path(sysconfig.get_paths()["purelib"]) / "msgpackrpc/transport/tcp.py"
s = p.read_text()
s = s.replace(
    "msgpack.Packer(encoding=encodings[0], default=lambda x: x.to_msgpack())",
    "msgpack.Packer(use_bin_type=True, default=lambda x: x.to_msgpack())",
)
s = s.replace(
    "msgpack.Unpacker(encoding=encodings[1])",
    "msgpack.Unpacker(raw=encodings[1] is None, strict_map_key=False)",
)
p.write_text(s)
print("msgpack-rpc transport adapted to the compiled msgpack 1.x API")
