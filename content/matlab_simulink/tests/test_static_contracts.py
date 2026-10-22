from collections.abc import Iterable
from pathlib import Path
import re
from typing import Protocol, cast

ROOT = Path(__file__).resolve().parents[1]


class RGBImage(Protocol):
    @property
    def size(self) -> tuple[int, int]: ...

    def resize(self, size: tuple[int, int]) -> "RGBImage": ...

    def getdata(self) -> Iterable[tuple[int, int, int]]: ...


def sampled_luminance(path: Path) -> list[float]:
    from PIL import Image

    image = cast(RGBImage, Image.open(path).convert("RGB"))
    width, height = image.size
    sampled = image.resize((max(1, width // 20), max(1, height // 20)))
    return [0.299 * r + 0.587 * g + 0.114 * b for r, g, b in sampled.getdata()]


def read_rel(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_required_files_exist():
    expected = [
        "run_all.m",
        "README.md",
        "matlab/common_plot_style.m",
        "matlab/compute_step_metrics.m",
        "matlab/exp1_digital_pid.m",
        "matlab/exp2_smith_predictor.m",
        "matlab/exp3_discrete_second_order_pid.m",
        "simulink/build_all_models.m",
        "simulink/build_exp1_digital_pid_model.m",
        "simulink/build_exp2_smith_predictor_model.m",
        "simulink/build_exp3_discrete_second_order_model.m",
        "report/report.typ",
        "tests/test_static_contracts.py",
        "tests/test_typst_compile.py",
    ]
    missing = [path for path in expected if not (ROOT / path).is_file()]
    assert not missing, f"missing files: {missing}"


def test_experiment_parameter_contracts():
    exp1 = read_rel("matlab/exp1_digital_pid.m")
    assert "G = 133 / (s^2 + 25*s);" in exp1
    assert re.search(r"^\s*Kp\s*=\s*15\s*;", exp1, re.MULTILINE)
    assert re.search(r"^\s*Ki\s*=\s*2\s*;", exp1, re.MULTILINE)
    assert re.search(r"^\s*Kd\s*=\s*0\.5\s*;", exp1, re.MULTILINE)

    exp2 = read_rel("matlab/exp2_smith_predictor.m")
    assert re.search(r"^\s*tau\s*=\s*1\s*;", exp2, re.MULTILINE)
    assert re.search(r"^\s*stepAmplitude\s*=\s*100\s*;", exp2, re.MULTILINE)
    assert re.search(r"^\s*Kp\s*=\s*40\s*;", exp2, re.MULTILINE)
    assert re.search(r"^\s*Ki\s*=\s*0\.022\s*;", exp2, re.MULTILINE)
    assert "delayFactors = [0.5 1 1.5 2];" in exp2

    exp3 = read_rel("matlab/exp3_discrete_second_order_pid.m")
    assert re.search(r"^\s*Ts\s*=\s*0\.01\s*;", exp3, re.MULTILINE)
    assert "Gz = tf([0.003 0.001], [1 -1.94 0.9418], Ts);" in exp3
    assert re.search(r"^\s*Kp\s*=\s*155\s*;", exp3, re.MULTILINE)
    assert re.search(r"^\s*Ki\s*=\s*1\.5\s*;", exp3, re.MULTILINE)
    assert re.search(r"^\s*Kd\s*=\s*4\.8\s*;", exp3, re.MULTILINE)


def test_simulink_builders_use_programmatic_model_api():
    for rel in [
        "simulink/build_exp1_digital_pid_model.m",
        "simulink/build_exp2_smith_predictor_model.m",
        "simulink/build_exp3_discrete_second_order_model.m",
    ]:
        text = read_rel(rel)
        for token in ["new_system", "add_block", "add_line", "save_system", "apply_print_style", "LineWidth"]:
            assert token in text, f"{token} not found in {rel}"
        assert "Simulink" in text


def test_report_embeds_figures_programs_and_mentions_authoritative_values():
    report = read_rel("report/report.typ")
    assert "《计算机控制系统》实验指导书" in report
    assert "tau=1" in report or "tau = 1" in report
    assert "T_s=0.01" in report or "Ts=0.01" in report
    assert "#image(" in report, "print-ready report should embed generated MATLAB figures"
    assert "附录" not in report, "program listings must be in the main body, not appendix sections"
    assert "计算机控制系统实验报告" not in report, "repeated page header should not appear above report pages"
    assert "#block(" in report, "program listings should be framed as code blocks"
    assert 'fill: rgb("#f7f7f7")' in report
    assert "stroke:" in report
    assert "Source Han Serif SC" in report
    assert "Source Han Sans HW SC" in report
    assert "#show table" not in report, "table show rule should not render literal placeholder text"
    assert "+-" not in report and "+−" not in report
    assert "plus.minus" in report
    assert "#align(center)[" in report and "#table(" in report
    assert "first-line-indent: 2em" in report
    assert "MATLAB Control System Toolbox" not in report
    assert '#prog("../simulink/' not in report
    for hidden_title in [
        "实验一 MATLAB 分析程序：",
        "实验二 MATLAB 分析程序：",
        "实验三 MATLAB 分析程序：",
        "公共运行入口：",
        "公共绘图样式：",
        "性能指标计算：",
    ]:
        assert hidden_title not in report

    guide_required_text = [
        "实验二、Smith 算法的运用",
        "1）、了解数字 PID 控制回路的结构。",
        "4） 整定比例度。",
        "1) WINDOWS 操作系统和 MATLAB 软件。",
        "2) PC 电脑。",
        "3）采用 Smith 控制方法，在 PID 控制中（选用 PI 控制），取 kp＝40，ki＝0.022，假设预测模型精确，阶跃信号输入取 100。",
        "G_p(z)=(0.003z+0.001)/(z^2-1.94z+0.9418)",
    ]
    for item in guide_required_text:
        assert item in report

    required_figures = [
        "exp1_simulink_model.png",
        "exp1_step_response.png",
        "exp1_open_loop_response.png",
        "exp1_sine_response.png",
        "exp1_square_response.png",
        "exp1_sawtooth_response.png",
        "exp1_bode.png",
        "exp1_nyquist.png",
        "exp1_root_locus.png",
        "exp1_pole_zero.png",
        "exp1_kp_sweep.png",
        "exp1_ki_sweep.png",
        "exp1_kd_sweep.png",
        "exp2_simulink_model.png",
        "exp2_smith_response.png",
        "exp2_pi_comparison.png",
        "exp2_bode_smith.png",
        "exp2_bode_conventional.png",
        "exp2_delay_robustness.png",
        "exp3_simulink_model.png",
        "exp3_root_locus.png",
        "exp3_pole_zero_unit_circle.png",
        "exp3_step_response.png",
        "exp3_uncontrolled_comparison.png",
        "exp3_sine_response.png",
        "exp3_square_response.png",
        "exp3_kp_sweep.png",
        "exp3_ki_sweep.png",
        "exp3_kd_sweep.png",
    ]
    for figure in required_figures:
        assert figure in report

    required_program_reads = [
        '../matlab/exp1_digital_pid.m',
        '../matlab/exp2_smith_predictor.m',
        '../matlab/exp3_discrete_second_order_pid.m',
        '../run_all.m',
        '../matlab/common_plot_style.m',
        '../matlab/compute_step_metrics.m',
    ]
    for program in required_program_reads:
        assert f'read(path: "{program}")' in report or f'#prog("{program}"' in report


def test_no_omo_plans_inside_submission_package():
    assert not (ROOT / ".omo" / "plans").exists()



def test_generated_figures_are_print_friendly_white_background():
    try:
        __import__("PIL.Image")
    except ImportError:
        return

    figure_dir = ROOT / "outputs" / "figures"
    if not figure_dir.exists():
        return

    dark_failures: list[str] = []
    for path in figure_dir.glob("*.png"):
        luminance = sampled_luminance(path)
        mean_luminance = sum(luminance) / len(luminance)
        dark_ratio = sum(value < 80 for value in luminance) / len(luminance)
        if mean_luminance < 200 or dark_ratio > 0.10:
            dark_failures.append(f"{path.name}: mean={mean_luminance:.1f}, dark={dark_ratio:.1%}")

    assert not dark_failures, "dark or black-background figures found: " + "; ".join(dark_failures)



def test_simulink_screenshots_have_visible_line_weight():
    try:
        __import__("PIL.Image")
    except ImportError:
        return

    figure_dir = ROOT / "outputs" / "figures"
    names = [
        "exp1_simulink_model.png",
        "exp2_simulink_model.png",
        "exp3_simulink_model.png",
    ]
    missing = [name for name in names if not (figure_dir / name).is_file()]
    if missing:
        return

    weak: list[str] = []
    for name in names:
        luminance = sampled_luminance(figure_dir / name)
        visible_ink_ratio = sum(value < 180 for value in luminance) / len(luminance)
        if visible_ink_ratio < 0.035:
            weak.append(f"{name}: visible_ink={visible_ink_ratio:.1%}")

    assert not weak, "Simulink screenshots still look too thin for printing: " + "; ".join(weak)



def test_exp3_root_locus_uses_discrete_plant_not_full_pid_loop():
    exp3 = read_rel("matlab/exp3_discrete_second_order_pid.m")
    assert "[rootData, ~] = rlocus(Gz);" in exp3
    assert "离散对象 G(z) 的 Z 域根轨迹" in exp3
    assert "rlocus(Lz)" not in exp3
