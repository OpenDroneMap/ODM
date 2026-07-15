#!/usr/bin/env python3
"""Build the runtime ODM Docker image with OCI image labels.

Invoked via `pixi run docker-build` (add `--gpu` for the GPU image, and pass
extra `docker build` flags after `--`). This computes the revision, version and
created build args from the working tree so a locally built image carries the
same OCI labels as the images published by CI.
"""
import argparse
import os
import subprocess
import sys
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def parse_args():
    p = argparse.ArgumentParser(description="Build the runtime ODM Docker image")
    p.add_argument("--gpu", action="store_true", help="Build the GPU image from gpu.Dockerfile")
    p.add_argument("-t", "--tag", default="", help="Image tag (default: opendronemap/odm:latest, or :gpu with --gpu)")
    return p.parse_known_args()


def git_revision():
    try:
        sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip()
    except (subprocess.CalledProcessError, OSError):
        return "unknown"
    dirty = subprocess.check_output(["git", "status", "--porcelain"], cwd=ROOT).strip()
    return sha + "-dirty" if dirty else sha


def main():
    args, docker_args = parse_args()

    with open(os.path.join(ROOT, "VERSION")) as f:
        version = f.read().strip()

    dockerfile = "gpu.Dockerfile" if args.gpu else "Dockerfile"
    tag = args.tag or ("opendronemap/odm:gpu" if args.gpu else "opendronemap/odm:latest")
    build_date = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    cmd = [
        "docker", "build",
        "-f", dockerfile,
        "--target", "runtime",
        "-t", tag,
        "--build-arg", "GIT_COMMIT=%s" % git_revision(),
        "--build-arg", "BUILD_DATE=%s" % build_date,
        "--build-arg", "IMAGE_VERSION=%s" % version,
        *docker_args,
        ".",
    ]
    print(" ".join(cmd))
    sys.exit(subprocess.run(cmd, cwd=ROOT).returncode)


if __name__ == "__main__":
    main()
