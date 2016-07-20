FROM packages

# Prepare directories`
RUN mkdir /code
WORKDIR /code

# Add repository files
ADD . /code/

# Update submodules
RUN git submodule init && git submodule update

# Build OpenDroneMap
RUN bash ./configure.sh

#Make build folder
RUN mkdir build && cd build && cmake .. && make

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"
ENV SESSION="DDODMMAP_SESSION"

# Entry point
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/code/"]
