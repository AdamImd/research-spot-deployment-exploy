"""Plot measured host production and GPU simulation timing from a completed campaign."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from spot_deploy.contracts import sha256
from spot_deploy.records import atomic_json, verify_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    fig.suptitle("ReLIC host timing · 200 Hz commands / 50 Hz policy", fontsize=13)
    hashes, results = {}, {}
    for device, color in (("cpu", "#1677ad"), ("cuda", "#d46818")):
        cell = args.run / f"{device}-paced"
        if not verify_run(cell, status="completed"):
            raise ValueError(f"invalid run integrity: {cell}")
        result = json.loads((cell / "result.json").read_text())
        rows = [json.loads(s) for s in (cell / "timing.jsonl").read_text().splitlines()]
        for name in ("result.json", "timing.jsonl", "COMPLETE.json", "provider-audit.json"):
            hashes[str(cell / name)] = sha256(cell / name)
        results[device] = result
        t = np.asarray([r["elapsed_s"] for r in rows])
        production = np.asarray([r["production_s"] for r in rows]) * 1000
        response = np.asarray([r["deadline_response_s"] for r in rows]) * 1000
        axes[0, 0].plot(t, production, color=color, alpha=.75, lw=.6, label=device.upper())
        axes[0, 1].plot(t, response, color=color, alpha=.75, lw=.6, label=device.upper())
        axes[1, 0].plot(np.sort(production), np.arange(1, len(rows)+1)/len(rows)*100,
                        color=color, label=device.upper())
        x = np.arange(4) + (0 if device == "cpu" else .36)
        values = [result["timing"][key][stat]*1000 for key, stat in
                  (("core_s", "p99_s"), ("core_s", "max_s"),
                   ("production_s", "p99_s"), ("production_s", "max_s"))]
        axes[1, 1].bar(x, values, width=.34, color=color, label=device.upper())
    axes[0, 0].set(title="Full command production\nState + core + logging + encoding",
                   xlabel="Wall schedule time (s)", ylabel="Production time (ms)", yscale="log")
    axes[0, 1].set(title="Scheduled release to completed command\nIncludes wake-up delay",
                   xlabel="Wall schedule time (s)", ylabel="Response time (ms)", yscale="log")
    axes[1, 0].set(title="Production latency distribution · all measured commands",
                   xlabel="Production time (ms)", ylabel="Cumulative commands (%)", xscale="log")
    axes[1, 1].set(title="Core and complete production · tail and worst case",
                   ylabel="Time (ms)", yscale="log", xticks=np.arange(4)+.18,
                   xticklabels=["Core p99", "Core max", "Production p99", "Production max"])
    for ax in (axes[0, 0], axes[0, 1], axes[1, 1]):
        ax.axhline(5, color="#8b3030", ls="--", lw=1, label="5 ms command interval")
    axes[1, 0].axvline(5, color="#8b3030", ls="--", lw=1)
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.savefig(args.output / "controller-timing.png", dpi=160, bbox_inches="tight", pad_inches=.15)
    fig.savefig(args.output / "controller-timing.pdf", bbox_inches="tight", pad_inches=.15)
    plt.close(fig)
    atomic_json(args.output / "summary.json", dict(results=results, input_sha256=hashes,
                plotting_source_sha256=sha256(Path(__file__)), hardware_access=False))
    atomic_json(args.output / "ARCHIVE.json", {p.name: sha256(p) for p in args.output.iterdir()
                                             if p.is_file()})


if __name__ == "__main__":
    main()
