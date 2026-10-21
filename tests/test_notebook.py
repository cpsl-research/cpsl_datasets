"""Execute notebooks/dataset_viewer.ipynb headless against synthetic captures."""
import json
from pathlib import Path

import nbformat
import pytest
from nbclient import NotebookClient
from synth_h5 import build, build_getters, build_human_movement
from synth_v1 import build_v1

from cpsl_datasets.convert_v1 import convert

NB = Path(__file__).parent.parent / "notebooks" / "dataset_viewer.ipynb"


def _v4(tmp_path):
    p = tmp_path / "v4.h5"
    build(p, "v4")
    return p


def _converted_v1(tmp_path):
    tr = build_v1(tmp_path / "v1")
    out = tmp_path / "v1.h5"
    convert(tr.root, out, log=lambda *_: None)
    return out


def _human(tmp_path):
    p = tmp_path / "human.h5"
    build_human_movement(p)
    return p


def _getters(tmp_path):
    p = tmp_path / "getters.h5"
    build_getters(p)
    return p


# CPSL_STREAMS picks a stream where auto-detect would be ambiguous
@pytest.mark.parametrize("make,streams", [
    (_v4, {"radar_adc": "radar_adc"}),
    (_converted_v1, {"radar_pc": "radar_pc", "camera": "camera", "hand": False}),
    (_human, {}),
    (_getters, {}),
], ids=["v4", "converted_v1", "human", "getters"])
def test_notebook_executes(tmp_path, monkeypatch, make, streams):
    path = make(tmp_path)
    monkeypatch.setenv("CPSL_DATASET_PATH", str(path))
    monkeypatch.setenv("CPSL_STREAMS", json.dumps(streams))
    monkeypatch.setenv("MPLBACKEND", "Agg")
    nb = nbformat.read(NB, as_version=4)
    NotebookClient(nb, timeout=120, resources={"metadata": {"path": str(NB.parent)}}).execute()
    out = "".join(o.get("text", "") for c in nb.cells if c.cell_type == "code"
                  for o in c.get("outputs", []))
    assert "frames:" in out


def test_notebook_committed_without_outputs():
    nb = nbformat.read(NB, as_version=4)
    assert all(not c.outputs and c.execution_count is None
               for c in nb.cells if c.cell_type == "code")
