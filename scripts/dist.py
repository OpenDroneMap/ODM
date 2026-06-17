#!/usr/bin/env python3
"""Build the Windows ODM installer (.exe) from an already-built pixi workspace.

This packages what `pixi run build` (SuperBuild native binaries) and
`pixi install -e prod` (the runtime conda environment at .pixi/envs/prod)
produce into an Inno Setup installer. It replaces the build+dist halves of the
old configure.py: the build is now done by pixi, so this script only does the
packaging (download Inno Setup + the VC++ redistributable, optionally bundle the
CUDA runtime, optionally code-sign, then run the Inno Setup compiler).

Run it on Windows via `pixi run dist` (see the win-64 task in pixi.toml).
"""
import argparse
import os
import shutil
import sys
import urllib.request
import zipfile

if sys.platform != "win32":
    sys.exit("dist.py builds the Windows installer and must run on Windows.")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOWNLOAD = os.path.join(ROOT, "SuperBuild", "download")

VC_REDIST_URL = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/VC_redist.x64.zip"
INNOSETUP_URL = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/innosetup-portable-win32-6.0.5-3.zip"
AZURE_SIGNING_URL = "https://www.nuget.org/api/v2/package/Microsoft.ArtifactSigning.Client/1.0.115"

# CUDA runtime DLLs to bundle next to the GPU-enabled SuperBuild binaries so the
# installed app runs the GPU SIFT path without a separate CUDA toolkit install.
CUDA_DLL_GLOBS = ("cudart64_", "cublas64_", "cublasLt64_", "cufft64_", "curand64_", "nvrtc64_", "nvrtc-builtins64_")


def parse_args():
    p = argparse.ArgumentParser(description="Build the Windows ODM installer")
    p.add_argument("--signtool-path", default="", help="Path to x64 signtool.exe")
    p.add_argument("--code-sign-cert-path", default="", help="Path to a .pfx code signing certificate")
    p.add_argument("--azure-signing-metadata", default="", help="Path to an Azure Artifact Signing metadata file")
    p.add_argument("--cuda-env", default=os.path.join(ROOT, ".pixi", "envs", "gpu"),
                   help="pixi env that provides the CUDA runtime DLLs to bundle (default: the gpu env)")
    return p.parse_args()


def run(cmd, cwd=ROOT):
    print(cmd)
    import subprocess
    if subprocess.Popen(cmd, shell=True, cwd=cwd, env=os.environ.copy()).wait() != 0:
        raise SystemExit("Command failed: %s" % cmd)


def download(url, dst):
    print("Downloading %s --> %s" % (url, dst))
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    with urllib.request.urlopen(url) as response, open(dst, "wb") as out:
        shutil.copyfileobj(response, out)


def ensure_vc_redist():
    exe = os.path.join(DOWNLOAD, "vc_redist.x64.exe")
    if os.path.isfile(exe):
        return
    archive = os.path.join(DOWNLOAD, "vc_redist.x64.zip")
    download(VC_REDIST_URL, archive)
    print("Extracting --> vc_redist.x64.exe")
    with zipfile.ZipFile(archive) as z:
        z.extractall(DOWNLOAD)


def ensure_innosetup():
    if os.path.isdir(os.path.join(ROOT, "innosetup")):
        return
    archive = os.path.join(DOWNLOAD, "innosetup.zip")
    download(INNOSETUP_URL, archive)
    os.mkdir(os.path.join(ROOT, "innosetup"))
    print("Extracting --> innosetup/")
    with zipfile.ZipFile(archive) as z:
        z.extractall(os.path.join(ROOT, "innosetup"))


def ensure_azure_signing():
    out = os.path.join(ROOT, "azuresigning")
    if os.path.isdir(out):
        return os.path.join(out, "bin", "x64", "Azure.CodeSigning.Dlib.dll")
    archive = os.path.join(DOWNLOAD, "microsoft.artifactsigning.client.1.0.115.nupkg")
    download(AZURE_SIGNING_URL, archive)
    os.mkdir(out)
    print("Extracting --> azuresigning/")
    with zipfile.ZipFile(archive) as z:
        z.extractall(out)
    return os.path.join(out, "bin", "x64", "Azure.CodeSigning.Dlib.dll")


def bundle_cuda_runtime(cuda_env):
    """Copy CUDA runtime DLLs into SuperBuild/install/bin (shipped + on PATH)."""
    src = os.path.join(cuda_env, "Library", "bin")
    dst = os.path.join(ROOT, "SuperBuild", "install", "bin")
    if not os.path.isdir(src):
        print("No CUDA env at %s; skipping CUDA runtime bundling (CPU-only build)." % src)
        return
    copied = 0
    for name in os.listdir(src):
        if name.lower().endswith(".dll") and any(name.startswith(p) for p in CUDA_DLL_GLOBS):
            shutil.copy2(os.path.join(src, name), os.path.join(dst, name))
            copied += 1
    print("Bundled %d CUDA runtime DLL(s) from %s" % (copied, src))


def sign_flags(args):
    if args.signtool_path and args.azure_signing_metadata:
        dlib = os.path.abspath(ensure_azure_signing())
        return ('"/Ssigntool=$q%s$q sign /v /debug /fd SHA256 /tr http://timestamp.acs.microsoft.com '
                '/td SHA256 /dlib $q%s$q /dmdf $q%s$q $f"'
                % (os.path.abspath(args.signtool_path), dlib, os.path.abspath(args.azure_signing_metadata)))
    if args.signtool_path and args.code_sign_cert_path:
        return ('"/Ssigntool=$q%s$q sign /f $q%s$q /fd SHA1 /t http://timestamp.sectigo.com $f"'
                % (os.path.abspath(args.signtool_path), os.path.abspath(args.code_sign_cert_path)))
    return "/DSKIP_SIGN=1"


def main():
    args = parse_args()

    if not os.path.isdir(os.path.join(ROOT, "SuperBuild", "install")):
        sys.exit("SuperBuild/install not found. Run `pixi run build` first.")
    if not os.path.isdir(os.path.join(ROOT, ".pixi", "envs", "prod")):
        sys.exit(".pixi/envs/prod not found. Run `pixi install --locked -e prod` first.")

    os.makedirs(DOWNLOAD, exist_ok=True)
    ensure_vc_redist()
    ensure_innosetup()
    bundle_cuda_runtime(args.cuda_env)

    run('innosetup\\iscc /Qp ' + sign_flags(args) + ' "innosetup.iss"')
    print("Done! Installer created in dist/")


if __name__ == "__main__":
    main()
