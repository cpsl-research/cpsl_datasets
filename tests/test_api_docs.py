"""The generated pdoc API reference shows every CpslDS kwarg with its default."""
import html
import inspect
import re
import subprocess
import sys
from pathlib import Path

import pytest

from cpsl_datasets import CpslDS

pytest.importorskip("pdoc")

MODULES = ["cpsl_datasets.cpsl_ds", "cpsl_datasets.hdf5",
           "cpsl_datasets.inspect", "cpsl_datasets.convert_v1"]


def _text(path):
    return html.unescape(re.sub(r"<[^>]+>", "", Path(path).read_text()))


@pytest.fixture(scope="module")
def api(tmp_path_factory):
    out = tmp_path_factory.mktemp("api")
    subprocess.run([sys.executable, "-m", "pdoc", "--docformat", "numpy", "-o", str(out), *MODULES],
                   check=True, capture_output=True)
    return out / "cpsl_datasets"


def test_signature_has_23_kwargs():
    params = [p for n, p in inspect.signature(CpslDS.__init__).parameters.items()
              if n not in ("self", "dataset_path")]
    assert len(params) == 23


def test_cpslds_kwargs_and_defaults_listed(api):
    text = _text(api / "cpsl_ds.html")
    params = inspect.signature(CpslDS.__init__).parameters
    for n, p in params.items():
        if n == "self":
            continue
        assert n in text, n
        if p.default is not inspect.Parameter.empty:
            assert "%s=%s" % (n, json_repr(p.default)) in text, n
        # the rendered Parameters list: "name (type, default X):" (pairs share a line)
        line = re.search(r"^(?:\w+, )?%s(?:, \w+)? \((.*)\):" % n, text, re.M)
        assert line, n
        if p.default is not inspect.Parameter.empty:
            assert "default %s" % json_repr(p.default) in line.group(1), n
    assert 'reference' in text and "Selects the reference stream" in text


def json_repr(v):
    return '"%s"' % v if isinstance(v, str) else repr(v)


def test_named_objects_present(api):
    ds = _text(api / "cpsl_ds.html")
    for name in ("match", "row_stamps", "at_time", "window", "read", "interp",
                 "get_radar_data", "Match"):
        assert name in ds, name
    h5 = _text(api / "hdf5.html")
    for name in ("Capture", "Stream", "capture_mode", "sample_rate_hz", "mode"):
        assert name in h5, name
    assert "python -m cpsl_datasets.inspect" in _text(api / "inspect.html")
    assert "convert_v1" in _text(api / "convert_v1.html")
