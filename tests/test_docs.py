"""Run every ```python block in docs/*.md and README.md; check relative links."""
import re
from pathlib import Path

import matplotlib
import pytest

matplotlib.use("Agg")
from synth_h5 import build, build_getters, build_human_movement, build_multi_radar  # noqa: E402

ROOT = Path(__file__).parent.parent
BUILDERS = {
    "v4": lambda p: build(p, "v4"),
    "getters": build_getters,
    "human": build_human_movement,
    "multi": build_multi_radar,
}
PAGES = sorted((ROOT / "docs").glob("*.md"))
FENCE = re.compile(r"```python\n(.*?)```", re.S)


def _blocks():
    """(id, code, synth, filename). Docs blocks name their fixture in a first-line
    `# synth:` comment; README blocks carry none, so they are mapped here."""
    out = []
    for page in PAGES:
        for i, m in enumerate(FENCE.finditer(page.read_text())):
            code = m.group(1)
            tag = re.match(r"# synth: (\w+)", code)
            assert tag, f"{page.name} block {i}: first line must be '# synth: <fixture>'"
            out.append((f"{page.name}#{i}", code, tag.group(1), "capture.h5"))
    # README blocks have no tag: block 0 -> a one-radar file, block 1 continues it
    readme = [("", "human", "capture.h5"),
              ("from cpsl_datasets import CpslDS\n", "multi", "multi_radar.h5")]
    for i, m in enumerate(FENCE.finditer((ROOT / "README.md").read_text())):
        pre, synth, fname = readme[i] if i < len(readme) else ("", "human", "capture.h5")
        out.append((f"README.md#{i}", pre + m.group(1), synth, fname))
    return out


BLOCKS = _blocks()


@pytest.mark.parametrize("code,synth,fname", [b[1:] for b in BLOCKS], ids=[b[0] for b in BLOCKS])
def test_snippet_runs(tmp_path, monkeypatch, code, synth, fname):
    BUILDERS[synth](tmp_path / fname)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr("matplotlib.pyplot.show", lambda *a, **k: None)
    exec(compile(code, "doc-snippet", "exec"), {"__name__": "__doc__"})


def test_docs_have_snippets():
    assert len(BLOCKS) >= 16


LINK = re.compile(r"(?<!\!)\[[^\]]*\]\(([^)\s]+)\)|!\[[^\]]*\]\(([^)\s]+)\)")


def _slug(heading):
    s = re.sub(r"[^\w\- ]", "", heading.strip().lower().replace("`", ""))
    return s.replace(" ", "-")


def _anchors(path):
    in_code, out = False, set()
    for line in path.read_text().splitlines():
        if line.startswith("```"):
            in_code = not in_code
        elif not in_code and line.startswith("#"):
            out.add(_slug(line.lstrip("#")))
    return out


@pytest.mark.parametrize("page", PAGES + [ROOT / "README.md"], ids=lambda p: p.name)
def test_relative_links(page):
    bad = []
    for m in LINK.finditer(page.read_text()):
        target = m.group(1) or m.group(2)
        if re.match(r"[a-z][a-z0-9+.-]*:", target):  # http(s), mailto: not fetched
            continue
        path, _, frag = target.partition("#")
        dest = (page.parent / path).resolve() if path else page
        if not dest.exists():
            bad.append(f"{target}: missing file")
        elif frag and dest.suffix == ".md" and frag not in _anchors(dest):
            bad.append(f"{target}: no such anchor")
    assert not bad, bad
