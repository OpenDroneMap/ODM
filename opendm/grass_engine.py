import shutil
import tempfile
import subprocess
import os
import sys
import time
from opendm import log
from opendm import system
import locale

from string import Template

class GrassEngine:
    def __init__(self):
        self.grass_binary = system.which('grass7') or \
                            system.which('grass72') or \
                            system.which('grass74') or \
                            system.which('grass76') or \
                            shutil.which('grass78') or \
                            shutil.which('grass80')

        if self.grass_binary is None:
            log.ODM_WARNING("Could not find a GRASS 7 executable. GRASS scripts will not work.")
        else:
            log.ODM_INFO("Initializing GRASS engine using {}".format(self.grass_binary))

    def create_context(self, serialized_context = {}):
        if self.grass_binary is None: raise GrassEngineException("GRASS engine is unavailable")
        return GrassContext(self.grass_binary, **serialized_context)


class GrassContext:
    def __init__(self, grass_binary, tmpdir = None, template_args = {}, location = None, auto_cleanup=True):
        self.grass_binary = grass_binary
        if tmpdir is None:
            tmpdir = tempfile.mkdtemp('_grass_engine')
        self.tmpdir = tmpdir
        self.template_args = template_args
        self.location = location
        self.auto_cleanup = auto_cleanup

    def get_cwd(self):
        return self.tmpdir

    def add_file(self, filename, source, use_as_location=False):
        param = os.path.splitext(filename)[0] # filename without extension

        dst_path = os.path.abspath(os.path.join(self.get_cwd(), filename))
        with open(dst_path, 'w') as f:
            f.write(source)
        self.template_args[param] = dst_path

        if use_as_location:
            self.set_location(self.template_args[param])

        return dst_path

    def add_param(self, param, value):
        self.template_args[param] = value

    def set_location(self, location):
        """
        :param location: either a "epsg:XXXXX" string or a path to a geospatial file defining the location
        """
        if not location.lower().startswith('epsg:'):
            location = os.path.abspath(location)
        self.location = location

    def execute(self, script):
        """
        :param script: path to .grass script
        :return: script output
        """
        if self.location is None: raise GrassEngineException("Location is not set")

        script = os.path.abspath(script)

        # Create grass script via template substitution
        try:
            with open(script) as f:
                script_content = f.read()
        except FileNotFoundError:
            raise GrassEngineException("Script does not exist: {}".format(script))

        tmpl = Template(script_content)

        # Write script to disk
        if not os.path.exists(self.get_cwd()):
            os.mkdir(self.get_cwd())

        with open(os.path.join(self.get_cwd(), 'script.sh'), 'w') as f:
            f.write(tmpl.substitute(self.template_args))

        # Execute it
        log.ODM_INFO("Executing grass script from {}: {} --tmp-location {} --exec bash script.sh".format(self.get_cwd(), self.grass_binary, self.location))
        env = os.environ.copy()
        env["GRASS_ADDON_PATH"] = env.get("GRASS_ADDON_PATH", "") + os.path.abspath(os.path.join("opendm/grass/addons"))
        env["LC_ALL"] = "C.UTF-8"

        filename = os.path.join(self.get_cwd(), 'output.log')
        with open(filename, 'wb') as writer, open(filename, 'rb', 1) as reader:
            p = subprocess.Popen([self.grass_binary, '--tmp-location', self.location, '--exec', 'bash', 'script.sh'],
                                cwd=self.get_cwd(), stdout=subprocess.PIPE, stderr=writer, env=env)
            
            while p.poll() is None:
                sys.stdout.write(reader.read().decode('utf8'))
                time.sleep(0.5)
            
            # Read the remaining
            sys.stdout.write(reader.read().decode('utf8'))

            out, err = p.communicate()
            out = out.decode('utf-8').strip()

            if p.returncode == 0:
                return out
            else:
                raise GrassEngineException("Could not execute GRASS script {} from {}: {}".format(script, self.get_cwd(), err))

    def serialize(self):
        return {
            'tmpdir': self.tmpdir,
            'template_args': self.template_args,
            'location': self.location,
            'auto_cleanup': self.auto_cleanup
        }

    def cleanup(self):
        if os.path.exists(self.get_cwd()):
            shutil.rmtree(self.get_cwd())

    def __del__(self):
        if self.auto_cleanup:
            self.cleanup()

class GrassEngineException(Exception):
    pass

def cleanup_grass_context(serialized_context):
    ctx = grass.create_context(serialized_context)
    ctx.cleanup()

grass = GrassEngine()