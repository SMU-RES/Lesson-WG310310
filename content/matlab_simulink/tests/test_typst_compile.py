from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_typst_report_compiles_when_typst_is_available():
    typst = shutil.which("typst")
    if typst is None:
        pytest.skip("typst is not installed")

    (ROOT / "outputs").mkdir(exist_ok=True)
    result = subprocess.run(
        [typst, "compile", "--root", ".", "report/report.typ", "outputs/report.pdf"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert (ROOT / "outputs" / "report.pdf").is_file()
