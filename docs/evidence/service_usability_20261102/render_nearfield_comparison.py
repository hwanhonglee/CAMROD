#!/usr/bin/env python3
"""HH_261102 - Plot recorded native ROS decisions, not invented vehicle motion."""

import argparse
from io import BytesIO
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from PIL import Image


MODES = ["original_exclusions", "revised_radar", "raw_guard", "combined"]
LABELS = ["기존 radar\n제외 설정", "radar 개선만", "LiDAR 보조만", "radar + LiDAR"]
SCENARIOS = ["body_10cm", "body_20cm", "body_30cm", "body_40cm"]


def distance_rows(data, scenarios=SCENARIOS):
    # HH_261102 - Missing/duplicate measurements must fail instead of becoming a convincing green chart.
    if data.get("metadata", {}).get("runtime") != "isolated_ros_actual_nodes":
        raise ValueError("Expected measurements from the isolated native ROS runner")
    selected = {}
    for row in data["rows"]:
        if row["scenario"] not in scenarios:
            continue
        key = row["mode"], row["scenario"]
        output = row["output_vx"]
        if key in selected or not isinstance(row["blocked"], bool) or not math.isfinite(output):
            raise ValueError(f"Invalid or duplicate measurement: {key}")
        if row["blocked"] != (abs(output) < 0.001):
            raise ValueError(f"Output and blocked decision disagree: {key}")
        selected[key] = row
    if set(selected) != {(mode, scenario) for mode in MODES for scenario in scenarios}:
        raise ValueError("All 16 distance/mode measurements are required")
    return selected


def render(data, selected, highlighted=None, scenarios=SCENARIOS, column_labels=None):
    fig = plt.figure(figsize=(14, 9), facecolor="#f3f6f4")
    title = "근접 장애물 정지: 변경 전·후 네 조건 비교" if scenarios == SCENARIOS else "센서면 거리 기준: radar·LiDAR의 보완 관계"
    fig.text(.055, .94, title, fontsize=23, weight="bold", color="#18362c")
    fig.text(.055, .90, "실제 수정한 ROS 노드·최종 제어기 출력 측정 / 합성 센서 입력 / CARLA·실물 주행 아님", color="#52655c")
    ax = fig.add_axes([.23, .39, .71, .42])
    for i, mode in enumerate(MODES):
        for j, scenario in enumerate(scenarios):
            row = selected[mode, scenario]
            stopped = row["blocked"]
            ax.add_patch(Rectangle((j, i), 1, 1,
                                  facecolor="#d8eee1" if stopped else "#fff0d8",
                                  edgecolor="white", linewidth=5))
            ax.text(j + .5, i + .43, "정지 명령" if stopped else "이동 명령 유지",
                    ha="center", va="center", weight="bold", fontsize=15,
                    color="#235c3c" if stopped else "#8a531a")
            ax.text(j + .5, i + .72, f"출력 {row['output_vx']:.2f} m/s",
                    ha="center", va="center", fontsize=11, color="#46574f")
    if highlighted is not None:
        ax.add_patch(Rectangle((highlighted, 0), 1, 4, fill=False,
                              edgecolor="#187fba", linewidth=4))
    ax.set(xlim=(0, 4), ylim=(4, 0), xticks=[.5, 1.5, 2.5, 3.5],
           xticklabels=column_labels or ["차체 앞 10 cm", "차체 앞 20 cm", "차체 앞 30 cm", "차체 앞 40 cm"],
           yticks=[.5, 1.5, 2.5, 3.5], yticklabels=LABELS)
    ax.xaxis.tick_top()
    ax.tick_params(length=0, pad=13)
    for spine in ax.spines.values():
        spine.set_visible(False)

    extras = [row for row in data["rows"] if row["scenario"] not in scenarios]
    passed = sum(row.get("pass") is True for row in data["rows"])
    fig.text(.055, .29, "읽는 방법", weight="bold", fontsize=15, color="#18362c")
    fig.text(.055, .245, "각 칸은 같은 위치의 장애물에 대한 실제 최종 속도 명령입니다. 제동 거리·실제 차량 정지를 측정한 표가 아닙니다.", fontsize=12)
    basis = ("거리 기준은 차체 전면입니다. 센서면 거리와 다르며 TF를 바꾸면 결과도 달라집니다."
             if scenarios == SCENARIOS else "센서면 10 cm 시험점은 기준 LiDAR 전방 시야 밖입니다. 이 조건에서 병용은 radar로 보완합니다.")
    fig.text(.055, .205, basis, fontsize=12)
    fig.text(.055, .165, f"기록된 조건 검사: {passed}/{len(data['rows'])} 일치 · 나머지 거리/노면/차체/입력 소실 등의 측정 {len(extras)}건", fontsize=12)
    fig.text(.055, .133, "기준 TF에서 radar는 차체 전면보다 8.05 cm 뒤에 있습니다. 센서면 30 cm와 차체 앞 30 cm는 다릅니다.", fontsize=11, color="#52655c")
    fig.text(.055, .105, "주의: 시뮬레이션 유용성 비교이며 충돌 안전 인증이 아닙니다. 운영 설정·장착 위치·제동 성능은 현장 검증이 필요합니다.", fontsize=11, color="#6d5542")
    fig.text(.055, .062, "GIF는 네 정적 시험 위치를 강조한 비교표입니다. 로봇이 움직이는 영상이 아닙니다.", fontsize=10, color="#69766e")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path,
                        default=Path(__file__).with_name("nearfield_comparison.json"))
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    selected = distance_rows(data)
    # HH_261102 - Register the actual installed CJK font; a stale Matplotlib cache must not turn Korean into boxes.
    font_path = subprocess.check_output(
        ["fc-match", "-f", "%{file}", "sans:lang=ko"], text=True).strip()
    font_manager.fontManager.addfont(font_path)
    plt.rcParams.update({"font.family": font_manager.FontProperties(fname=font_path).get_name(),
                         "font.size": 13})
    directory = args.input.parent
    figure = render(data, selected)
    figure.savefig(directory / "nearfield_comparison.png", dpi=160)
    plt.close(figure)
    face_scenarios = [f"sensor_face_{distance}cm" for distance in (10, 20, 30, 40)]
    face_rows = distance_rows(data, face_scenarios)
    labels = [f"센서면 {distance} cm\n차체 앞 {face_rows[MODES[0], scenario]['body_clearance_m'] * 100:.2f} cm"
              for distance, scenario in zip((10, 20, 30, 40), face_scenarios)]
    figure = render(data, face_rows, scenarios=face_scenarios, column_labels=labels)
    figure.savefig(directory / "nearfield_sensor_face_comparison.png", dpi=160)
    plt.close(figure)
    frames = []
    for column in range(4):
        figure = render(data, selected, column)
        buffer = BytesIO()
        figure.savefig(buffer, format="png", dpi=110)
        plt.close(figure)
        buffer.seek(0)
        frames.append(Image.open(buffer).convert("RGB"))
    frames[0].save(directory / "nearfield_comparison.gif", save_all=True,
                   append_images=frames[1:], duration=1600, loop=0)
    print(directory / "nearfield_comparison.png")
    print(directory / "nearfield_comparison.gif")


if __name__ == "__main__":
    main()
