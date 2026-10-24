"""Shared PC-only adapters for the real Python replay/download clients."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load_tool(relative):
    path = ROOT / relative
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ProcessLink:
    def __init__(self, process):
        self.process = process

    def write(self, data):
        self.process.stdin.write(data)
        self.process.stdin.flush()
        return len(data)

    def read(self, count):
        return self.process.stdout.read(count)
