#HELP FOR USERS: Replace example file paths with your own in the example commands below:
#BUILD COMMAND: docker build -t odm .
#AUTOMATIC RUN COMMAND:
#    docker run -it --rm \
#    -v $(pwd)/images:/code/images \
#    -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
#    -v $(pwd)/odm_texturing:/code/odm_texturing \
#    --user odm_user
#    odm_image
#MANUAL RUN COMMAND: docker run -it -v /home/alex/OpenDroneMap/images:/code/images --rm --entrypoint bash odm
#WORK INSIDE DOCKER IMAGE: docker run -it -v `pwd`:/code/ --rm --entrypoint bash odm
#REBUILD PACKAGES: docker build -t packages -f packages.Dockerfile .

FROM packages

# Add users
RUN useradd -m -U odm

# Prepare directories`
RUN mkdir /code
WORKDIR /code

# Add repository files
ADD . /code/

# Update submodules
RUN git submodule init && git submodule update

# Build OpenDroneMap
RUN bash ./configure.sh && \
    chown -R odm:odm /code

#Make build folder
RUN mkdir build && cd build && cmake .. && make

USER odm

ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/install/lib/python2.7/dist-packages"
ENV PYTHONPATH="$PYTHONPATH:/code/SuperBuild/src/opensfm"
ENV LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/code/SuperBuild/install/lib"
ENV SESSION="DDODMMAP_SESSION"

# Entry point
# VOLUME ["/code/images"]
ENTRYPOINT ["python", "/code/run.py", "--project-path", "/code/"]
