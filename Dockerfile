#Pull in previously built packages image with lots of libraries.
FROM packages

# Prepare directories
RUN mkdir /code
WORKDIR /code

# Add repository files
ADD ccd_defs_check.py /code/ccd_defs_check.py
ADD CMakeLists.txt /code/CMakeLists.txt
ADD configure.sh /code/configure.sh
ADD /.git/ /code/.git/
ADD .gitignore /code/.gitignore
ADD .gitmodules /code/.gitmodules
ADD /modules/ /code/modules/
ADD /opendm/ /code/opendm/
ADD /patched_files/ /code/patched_files/
ADD run.py /code/run.py
ADD /scripts/ /code/scripts/
ADD /SuperBuild/cmake/ /code/SuperBuild/cmake/
ADD /SuperBuild/CMakeLists.txt /code/SuperBuild/CMakeLists.txt
ADD /tests/ /code/tests/

# Update submodules
RUN git submodule init && git submodule update

# Build OpenDroneMap
RUN bash ./configure.sh

#Make build folder
RUN mkdir build && cd build && cmake .. && make

#Set environment variables
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"

# Entry point
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/code/"]
