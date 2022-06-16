import sys, platform
if sys.platform != 'win32':
    print("This script is for Windows only! Use configure.sh instead.")
    exit(1)
if sys.version_info.major != 3 or sys.version_info.minor != 8:
    print("You need to use Python 3.8.x (due to the requirements.txt). You are using %s instead." % platform.python_version())
    exit(1)

import argparse
import subprocess
import os
import stat
import urllib.request
import shutil 
import zipfile

from venv import EnvBuilder

parser = argparse.ArgumentParser(description='ODM Windows Configure Script')
parser.add_argument('action',
                type=str,
                choices=["build", "clean", "dist", "vcpkg_export"],
                help='Action: %(choices)s')
parser.add_argument('--build-vcpkg',
                    type=bool,
                    help='Build VCPKG environment from scratch instead of downloading prebuilt one.')
parser.add_argument('--vcpkg-archive-url',
                    type=str,
                    default='https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/vcpkg-export-250.zip',
                    required=False,
                    help='Path to VCPKG export archive')
parser.add_argument('--code-sign-cert-path',
                    type=str,
                    default='',
                    required=False,
                    help='Path to pfx code signing certificate')

args = parser.parse_args()

def run(cmd, cwd=os.getcwd()):
    env = os.environ.copy()
    print(cmd)
    p = subprocess.Popen(cmd, shell=True, env=env, cwd=cwd)
    retcode = p.wait()
    if retcode != 0:
        raise Exception("Command returned %s" % retcode)

# https://izziswift.com/shutil-rmtree-fails-on-windows-with-access-is-denied/
def rmtree(top):
    for root, dirs, files in os.walk(top, topdown=False):
        for name in files:
            filename = os.path.join(root, name)
            os.chmod(filename, stat.S_IWUSR)
            os.remove(filename)
        for name in dirs:
            os.rmdir(os.path.join(root, name))
    os.rmdir(top)

def vcpkg_requirements():
    with open("vcpkg-requirements.txt") as f:
        pckgs = list(filter(lambda l: len(l) > 0, map(str.strip, f.read().split("\n"))))
    return pckgs

def build():
    # Create python virtual env
    if not os.path.isdir("venv"):
        print("Creating virtual env --> venv/")
        ebuilder = EnvBuilder(with_pip=True)
        ebuilder.create("venv")

    run("venv\\Scripts\\pip install --ignore-installed -r requirements.txt")
    
    # Download / build VCPKG environment
    if not os.path.isdir("vcpkg"):
        if args.build_vcpkg:
            print("TODO")
            # git clone vcpkg repo
            # bootstrap
            # install requirements

        else:
            if not os.path.exists("vcpkg-env.zip"):
                print("Downloading %s" % args.vcpkg_archive_url)
                with urllib.request.urlopen(args.vcpkg_archive_url) as response, open( "vcpkg-env.zip", 'wb') as out_file:
                    shutil.copyfileobj(response, out_file)
            if not os.path.exists("vcpkg"):
                print("Extracting vcpkg-env.zip --> vcpkg/")
                with zipfile.ZipFile("vcpkg-env.zip") as z:
                    top_dir = z.namelist()[0]
                    z.extractall(".")

                    if os.path.exists(top_dir):
                        os.rename(top_dir, "vcpkg")
                    else:
                        print("Warning! Something looks wrong in the VCPKG archive... check the vcpkg/ directory.")
                safe_remove("vcpkg-env.zip")

    if not os.path.exists(os.path.join("SuperBuild", "build")) or not os.path.exists(os.path.join("SuperBuild", "install")):
        print("Compiling SuperBuild")
        
        build_dir = os.path.join("SuperBuild", "build")
        if not os.path.isdir(build_dir):
            os.mkdir(build_dir)

        toolchain_file = os.path.join(os.getcwd(), "vcpkg", "scripts", "buildsystems", "vcpkg.cmake")
        run("cmake .. -DCMAKE_TOOLCHAIN_FILE=\"%s\"" % toolchain_file,  cwd=build_dir)
        run("cmake --build . --config Release", cwd=build_dir)

def vcpkg_export():
    if not os.path.exists("vcpkg"):
        print("vcpkg directory does not exist. Did you build the environment?")
        exit(1)

    pkgs = vcpkg_requirements()
    out = "vcpkg-export-%s" % odm_version().replace(".", "")
    run("vcpkg\\vcpkg export %s --output=%s --zip" % (" ".join(pkgs), out))

def odm_version():
    with open("VERSION") as f:
        return f.read().split("\n")[0].strip()

def safe_remove(path):
    if os.path.isdir(path):
        rmtree(path)
    elif os.path.isfile(path):
        os.remove(path)

def clean():
    safe_remove("vcpkg-download.zip")
    safe_remove("vcpkg")
    safe_remove("venv")
    safe_remove(os.path.join("SuperBuild", "build"))
    safe_remove(os.path.join("SuperBuild", "download"))
    safe_remove(os.path.join("SuperBuild", "src"))
    safe_remove(os.path.join("SuperBuild", "install"))

def dist():
    if not os.path.exists("SuperBuild\\download"):
        print("You need to run configure.py build before you can run dist")
        exit(1)

    # Download VC++ runtime
    vcredist_path = os.path.join("SuperBuild", "download", "vc_redist.x64.zip")
    if not os.path.isfile(vcredist_path):
        vcredist_url = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/VC_redist.x64.zip"
        print("Downloading %s" % vcredist_url)
        with urllib.request.urlopen(vcredist_url) as response, open(vcredist_path, 'wb') as out_file:
            shutil.copyfileobj(response, out_file)

        print("Extracting --> vc_redist.x64.exe")
        with zipfile.ZipFile(vcredist_path) as z:
            z.extractall(os.path.join("SuperBuild", "download"))

    # Download portable python
    if not os.path.isdir("python38"):
        pythonzip_path = os.path.join("SuperBuild", "download", "python38.zip")
        python_url = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/python-3.8.1-embed-amd64-less-pth.zip"
        if not os.path.exists(pythonzip_path):
            print("Downloading %s" % python_url)
            with urllib.request.urlopen(python_url) as response, open( pythonzip_path, 'wb') as out_file:
                shutil.copyfileobj(response, out_file)
        
        os.mkdir("python38")

        print("Extracting --> python38/")
        with zipfile.ZipFile(pythonzip_path) as z:
            z.extractall("python38")

    # Download signtool
    signtool_path = os.path.join("SuperBuild", "download", "signtool.exe")
    signtool_url = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/signtool.exe"
    if not os.path.exists(signtool_path):
        print("Downloading %s" % signtool_url)
        with urllib.request.urlopen(signtool_url) as response, open(signtool_path, 'wb') as out_file:
            shutil.copyfileobj(response, out_file)

    # Download innosetup
    if not os.path.isdir("innosetup"):
        innosetupzip_path = os.path.join("SuperBuild", "download", "innosetup.zip")
        innosetup_url = "https://github.com/OpenDroneMap/windows-deps/releases/download/2.5.0/innosetup-portable-win32-6.0.5-3.zip"
        if not os.path.exists(innosetupzip_path):
            print("Downloading %s" % innosetup_url)
            with urllib.request.urlopen(innosetup_url) as response, open(innosetupzip_path, 'wb') as out_file:
                shutil.copyfileobj(response, out_file)

        os.mkdir("innosetup")

        print("Extracting --> innosetup/")
        with zipfile.ZipFile(innosetupzip_path) as z:
            z.extractall("innosetup")

    # Run
    cs_flags = ""
    if args.code_sign_cert_path:
        cs_flags = '"/Ssigntool=%s sign /f %s /fd SHA1 /t http://timestamp.sectigo.com $f"' % (signtool_path, args.code_sign_cert_path)
    run("innosetup\\iscc /Qp " + cs_flags  + " \"innosetup.iss\"")

    print("Done! Setup created in dist/")

if args.action == 'build':
    build()
elif args.action == 'vcpkg_export':
    vcpkg_export()
elif args.action == 'dist':
    dist()
elif args.action == 'clean':
    clean()
else:
    args.print_help()
    exit(1)
