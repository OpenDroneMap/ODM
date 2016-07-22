#Pull in previously built packages image with lots of libraries.
FROM packages

# Prepare directories
RUN mkdir /code
WORKDIR /code

# Copy repository files
COPY ccd_defs_check.py /code/ccd_defs_check.py
COPY CMakeLists.txt /code/CMakeLists.txt
COPY configure.sh /code/configure.sh
COPY /.git/ /code/.git/
COPY .gitignore /code/.gitignore
COPY .gitmodules /code/.gitmodules
COPY /modules/ /code/modules/
COPY /opendm/ /code/opendm/
COPY /patched_files/ /code/patched_files/
COPY run.py /code/run.py
COPY /scripts/ /code/scripts/
COPY /SuperBuild/cmake/ /code/SuperBuild/cmake/
COPY /SuperBuild/CMakeLists.txt /code/SuperBuild/CMakeLists.txt
COPY /tests/ /code/tests/

# Update submodules
RUN git submodule init && git submodule update

#Compile code in SuperBuild and root directories
RUN cd SuperBuild && mkdir build && cd build && cmake .. && make -j$(nproc) \
    && cd ../.. && mkdir build && cd build && cmake .. && make -j$(nproc)

# Entry point
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/code/"]
