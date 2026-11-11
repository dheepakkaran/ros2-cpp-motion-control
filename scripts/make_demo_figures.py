"""Demo figures for README and docs/index.html from results/demo/trajectory.csv.

The CSV comes from the C++ `trajectory_demo` tool (controller core + MuJoCo C API, every 1 ms tick).
Here the logged joint angles are only plotted and re-posed in MuJoCo's Python bindings for a picture;
nothing is simulated again.
Run: MUJOCO_GL=egl uv run --with mujoco --with matplotlib --with pandas --with pillow python scripts/make_demo_figures.py
"""
import os
import pathlib

os.environ.setdefault("MUJOCO_GL", "egl")
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

R = pathlib.Path(__file__).resolve().parents[1]
OUT = R / "docs" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
d = pd.read_csv(R / "results" / "demo" / "trajectory.csv")
J = range(1, 8)
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.edgecolor": "#737373", "axes.labelcolor": "#3d3d3d", "xtick.color": "#737373", "ytick.color": "#737373"})
GRAYS = ["#0a0a0a", "#555555", "#8a8a8a", "#b5b5b5"]

# ---- Figure 1: target vs tracked, error over time, watchdog hold
b = d[d.scenario == "baseline"]
s = d[d.scenario == "commands_stop"]
fig, ax = plt.subplots(1, 3, figsize=(11, 3.5), dpi=150)
for c, j in zip(GRAYS, [1, 2, 4, 6]):
    ax[0].plot(b.t, b[f"des{j}"], color=c, lw=2.4, alpha=.35)
    ax[0].plot(b.t, b[f"q{j}"], color=c, lw=.9, label=f"joint {j}")
ax[0].set_title("Baseline: target (wide) vs tracked (thin)", loc="left", fontsize=10)
ax[0].set_xlabel("seconds"); ax[0].set_ylabel("joint angle (rad)"); ax[0].legend(frameon=False, fontsize=8, ncol=2)
trk = b["mode"] == "tracking"
for c, j in zip(GRAYS, [1, 2, 4, 6]):
    ax[1].plot(b.t[trk], 1000 * (b[f"des{j}"] - b[f"q{j}"])[trk], color=c, lw=.9, label=f"joint {j}")
e = np.concatenate([(b[f"des{j}"] - b[f"q{j}"])[trk].values for j in J])
rms = np.sqrt((e ** 2).mean())
ax[1].set_title(f"Tracking error, RMS {rms:.5f} rad", loc="left", fontsize=10)
ax[1].set_xlabel("seconds"); ax[1].set_ylabel("target - tracked (mrad)")
trip = s.t[s["mode"] == "watchdog_hold"].min()
ax[2].plot(s.t, s.des1, color="#0a0a0a", lw=2.4, alpha=.35, label="target")
ax[2].plot(s.t, s.q1, color="#0a0a0a", lw=.9, label="tracked")
ax[2].plot(b.t, b.des1, color="#b5b5b5", lw=.9, ls="--", label="planned move")
ax[2].axvline(0.8, color="#737373", lw=.8, ls=":"); ax[2].text(0.82, 0.05, "client stops", fontsize=8, color="#555555")
ax[2].axvline(trip, color="#737373", lw=.8, ls=":"); ax[2].text(trip + 0.02, 0.15, f"watchdog trips\n({trip:.3f} s)", fontsize=8, color="#555555")
ax[2].set_title("Commands stop: watchdog holds joint 1", loc="left", fontsize=10)
ax[2].set_xlabel("seconds"); ax[2].set_ylabel("joint 1 (rad)"); ax[2].legend(frameon=False, fontsize=8, loc="lower right")
fig.tight_layout(); fig.savefig(OUT / "demo-tracking.png", facecolor="white"); plt.close(fig)
print(f"baseline RMS {rms:.8f} rad, max {np.abs(e).max():.6f}; watchdog trip at {trip:.3f} s")

# ---- Figure 2: the logged baseline motion, re-posed in MuJoCo and rendered headless
spec = mujoco.MjSpec.from_file(str(R / "assets" / "franka_emika_panda" / "panda_nohand.xml"))
spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                 rgb1=[1, 1, 1], rgb2=[0.92, 0.92, 0.92], width=64, height=64)
spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
w = spec.worldbody
w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.5, 1.5, 0.05], rgba=[0.93, 0.93, 0.93, 1])
w.add_light(pos=[0.8, -0.6, 2.2], dir=[-0.35, 0.25, -1], diffuse=[0.85, 0.85, 0.85], castshadow=True)
w.add_light(pos=[-0.8, 0.8, 2.0], dir=[0.3, -0.3, -1], diffuse=[0.35, 0.35, 0.35])
m = spec.compile()
data = mujoco.MjData(m)
ren = mujoco.Renderer(m, 360, 480)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0.3, 0.05, 0.4]; cam.distance, cam.azimuth, cam.elevation = 1.45, 135, -20
frames = []
for _, row in b.iloc[::40].iterrows():
    data.qpos[:7] = [row[f"q{j}"] for j in J]
    mujoco.mj_kinematics(m, data)
    ren.update_scene(data, cam)
    img = Image.fromarray(ren.render())
    ImageDraw.Draw(img).rectangle([0, 0, 480, 26], fill=(255, 255, 255))
    err = max(abs(row[f"des{j}"] - row[f"q{j}"]) for j in J)
    ImageDraw.Draw(img).text((10, 8), f"t = {row.t:.2f} s   {row['mode']}   max |target - tracked| = {err * 1000:.2f} mrad",
                             fill=(10, 10, 10))
    frames.append(img)
pal = [f.quantize(colors=64, method=Image.Quantize.MEDIANCUT) for f in frames]
pal[0].save(OUT / "demo-panda-move.gif", save_all=True, append_images=pal[1:], duration=40, loop=0, optimize=True)
picks = [frames[i] for i in np.linspace(0, len(frames) - 1, 4).astype(int)]
strip = Image.new("RGB", (480 * 4, 360), "white")
for i, f in enumerate(picks):
    strip.paste(f, (480 * i, 0))
strip.save(OUT / "demo-panda-strip.png", optimize=True)
print(len(frames), "frames")
