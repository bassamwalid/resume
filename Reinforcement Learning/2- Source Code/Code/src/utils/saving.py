"""Saving, loading, and configuration helpers."""

from __future__ import annotations

import csv
import json
import pickle
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def load_config(path):
    with open(path, "r", encoding="utf-8") as file:
        text = file.read()

    if yaml is not None:
        return yaml.safe_load(text)
    return load_simple_yaml(text)


def load_simple_yaml(text):
    """Parse the small nested key/value YAML subset used by default.yaml."""
    root = {}
    stack = [(-1, root)]

    for raw_line in text.splitlines():
        line_without_comment = raw_line.split("#", 1)[0].rstrip()
        if not line_without_comment.strip():
            continue

        indent = len(line_without_comment) - len(line_without_comment.lstrip(" "))
        key, separator, value = line_without_comment.strip().partition(":")
        if separator != ":":
            raise ValueError(f"unsupported config line: {raw_line}")

        while stack and indent <= stack[-1][0]:
            stack.pop()

        parent = stack[-1][1]
        value = value.strip()
        if value == "":
            child = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = parse_scalar(value)

    return root


def parse_scalar(value):
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "none"}:
        return None
    if (value.startswith('"') and value.endswith('"')) or (
        value.startswith("'") and value.endswith("'")
    ):
        return value[1:-1]

    try:
        if any(character in value for character in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def ensure_output_dirs(output_dir):
    output_path = Path(output_dir)
    for child in ("figures", "logs", "q_tables", "models", "animations"):
        (output_path / child).mkdir(parents=True, exist_ok=True)
    return output_path


def save_json(data, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as file:
        json.dump(to_builtin(data), file, indent=2)


def save_csv(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if not rows:
        return

    fieldnames = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with open(path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_q_table(q_table, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    clean_table = {tuple(state): np.asarray(values, dtype=float) for state, values in q_table.items()}
    with open(path, "wb") as file:
        pickle.dump(clean_table, file)


def load_q_table(path):
    with open(path, "rb") as file:
        return pickle.load(file)


def to_builtin(value):
    if isinstance(value, dict):
        return {str(key): to_builtin(item) for key, item in value.items()}
    if isinstance(value, list):
        return [to_builtin(item) for item in value]
    if isinstance(value, tuple):
        return [to_builtin(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value
