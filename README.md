![ODM Logo](https://user-images.githubusercontent.com/1951843/79699889-438ce580-8260-11ea-9c79-8667834aeab2.png)

An open source command line toolkit for processing aerial drone imagery. ODM turns simple 2D images into:

* Classified Point Clouds
* 3D Textured Models
* Georeferenced Orthorectified Imagery
* Georeferenced Digital Elevation Models

![images-diag](https://user-images.githubusercontent.com/1174901/96644651-5b205600-12f7-11eb-827b-8f4a3a6f3b21.png)

The application is available for Windows, Mac and Linux and it works from the command line, making it ideal for power users, scripts and for integration with other software.

If you would rather not type commands in a shell and are looking for a friendly user interface, check out [WebODM](https://github.com/OpenDroneMap/WebODM).

## Quickstart

The easiest way to run ODM is via docker. To install docker, see [docs.docker.com](https://docs.docker.com). Once you have docker installed and [working](https://docs.docker.com/get-started/#test-docker-installation), you can get ODM by running from a Command Prompt / Terminal:

```bash
docker pull opendronemap/odm
```

Run ODM by placing some images (JPEGs, TIFFs or DNGs) in a folder named “images” (for example `C:\Users\youruser\datasets\project\images` or `/home/youruser/datasets/project/images`) and simply run from a Command Prompt / Terminal:

```bash
# Windows
docker run -ti --rm -v c:/Users/youruser/datasets:/datasets opendronemap/odm --project-path /datasets project
```
```bash
# Mac/Linux
docker run -ti --rm -v /home/youruser/datasets:/datasets opendronemap/odm --project-path /datasets project
```

You can pass [additional parameters](https://docs.opendronemap.org/arguments/) by appending them to the command:

```bash
docker run -ti --rm -v /datasets:/datasets opendronemap/odm --project-path /datasets project [--additional --parameters --here]
```

For example, to generate a DSM (`--dsm`) and increase the orthophoto resolution (`--orthophoto-resolution 2`) :

```bash
docker run -ti --rm -v /datasets:/datasets opendronemap/odm --project-path /datasets project --dsm --orthophoto-resolution 2
```

## Viewing Results

When the process finishes, the results will be organized as follows:

    |-- images/
        |-- img-1234.jpg
        |-- ...
    |-- opensfm/
        |-- see mapillary/opensfm repository for more info
    |-- odm_meshing/
        |-- odm_mesh.ply                    # A 3D mesh
    |-- odm_texturing/
        |-- odm_textured_model.obj          # Textured mesh
        |-- odm_textured_model_geo.obj      # Georeferenced textured mesh
    |-- odm_georeferencing/
        |-- odm_georeferenced_model.laz     # LAZ format point cloud
    |-- odm_orthophoto/
        |-- odm_orthophoto.tif              # Orthophoto GeoTiff

You can use the following free and open source software to open the files generated in ODM:
 * .tif (GeoTIFF): [QGIS](http://www.qgis.org/)
 * .laz (Compressed LAS): [CloudCompare](https://www.cloudcompare.org/)
 * .obj (Wavefront OBJ), .ply (Stanford Triangle Format): [MeshLab](http://www.meshlab.net/)

**Note!** Opening the .tif files generated by ODM in programs such as Photoshop or GIMP might not work (they are GeoTIFFs, not plain TIFFs). Use [QGIS](http://www.qgis.org/) instead.

## API

ODM can be made accessible from a network via [NodeODM](https://github.com/OpenDroneMap/NodeODM).

## Documentation

See http://docs.opendronemap.org for tutorials and more guides.

## Forum

We have a vibrant [community forum](https://community.opendronemap.org/). You can [search it](https://community.opendronemap.org/search?expanded=true) for issues you might be having with ODM and you can post questions there. We encourage users of ODM to participate in the forum and to engage with fellow drone mapping users.

## Windows Setup

ODM can be installed natively on Windows. Just download the latest setup from the [releases](https://github.com/OpenDroneMap/ODM/releases) page. After opening the ODM Console you can process datasets by typing:

```bash
run C:\Users\youruser\datasets\project  [--additional --parameters --here]
```

## GPU Acceleration

ODM has support for doing SIFT feature extraction on a GPU, which is about 2x faster than the CPU on a typical consumer laptop. To use this feature, you need to use the `opendronemap/odm:gpu` docker image instead of `opendronemap/odm` and you need to pass the `--gpus all` flag:

```
docker run -ti --rm -v c:/Users/youruser/datasets:/datasets --gpus all opendronemap/odm:gpu --project-path /datasets project --feature-type sift
```

When you run ODM, if the GPU is recognized, in the first few lines of output you should see:

```
[INFO]    Writing exif overrides
[INFO]    Maximum photo dimensions: 4000px
[INFO]    Found GPU device: Intel(R) OpenCL HD Graphics
[INFO]    Using GPU for extracting SIFT features
```

The SIFT GPU implementation is CUDA-based, so should work with most NVIDIA graphics cards of the GTX 9xx Generation or newer.

If you have an NVIDIA card, you can test that docker is recognizing the GPU by running:

```
docker run --rm --gpus all nvidia/cuda:10.0-base nvidia-smi
```

If you see an output that looks like this:

```
Fri Jul 24 18:51:55 2020       
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 440.82       Driver Version: 440.82       CUDA Version: 10.2     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
```

You're in good shape!

See https://github.com/NVIDIA/nvidia-docker and https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker for information on docker/NVIDIA setup.

## Native Install (Ubuntu 21.04)

You can run ODM natively on Ubuntu 21.04 (although we don't recommend it):  

```bash
git clone https://github.com/OpenDroneMap/ODM
cd ODM
bash configure.sh install
```

You can then process datasets with `./run.sh /datasets/odm_data_aukerman`

## Native Install (MacOS)

You can run ODM natively on Intel/ARM MacOS.

First install:

 * Xcode 13 (not 14, there's currently a bug)
 * [Homebrew](https://docs.brew.sh/Installation)

Then Run:

```bash
git clone https://github.com/OpenDroneMap/ODM
cd ODM
bash configure_macos.sh install
```

You can then process datasets with `./run.sh /datasets/odm_data_aukerman`

This could be improved in the future. [Helps us create a Homebrew formula](https://github.com/OpenDroneMap/ODM/issues/1531).

### Updating a native installation

When updating to a newer version of native ODM, it is recommended that you run:

`bash configure.sh reinstall`

to ensure all the dependent packages and modules get updated.

### Build Docker Images From Source

If you want to rebuild your own docker image (if you have changed the source code, for example), from the ODM folder you can type:

```bash
docker build -t my_odm_image --no-cache .
```
When building your own Docker image, if image size is of importance to you, you should use the ```--squash``` flag, like so:

```bash
docker build --squash -t my_odm_image .
```

This will clean up intermediate steps in the Docker build process, resulting in a significantly smaller image (about half the size).

Experimental flags need to be enabled in Docker to use the ```--squash``` flag. To enable this, insert the following into the file `/etc/docker/daemon.json`:

```json
{
   "experimental": true
}
```

After this, you must restart docker.

## Video Support

Starting from version 3.0.4, ODM can automatically extract images from video files (.mp4, .mov, .lrv, .ts). Just place one or more video files into the `images` folder and run the program as usual. Subtitles files (.srt) with GPS information are also supported. Place .srt files in the `images` folder, making sure that the filenames match. For example, `my_video.mp4` ==> `my_video.srt` (case-sensitive).

## Developers

Help improve our software! We welcome contributions from everyone, whether to add new features, improve speed, fix existing bugs or add support for more cameras. Check our [code of conduct](https://github.com/OpenDroneMap/documents/blob/master/CONDUCT.md), the [contributing guidelines](https://github.com/OpenDroneMap/documents/blob/master/CONTRIBUTING.md) and [how decisions are made](https://github.com/OpenDroneMap/documents/blob/master/GOVERNANCE.md#how-decisions-are-made).


### Installation and first run
For Linux users, the easiest way to modify the software is to make sure docker is installed, clone the repository and then run from a shell:

```bash
$ DATA=/path/to/datasets ./start-dev-env.sh
```

Where `/path/to/datasets` is a directory where you can place test datasets (it can also point to an empty directory if you don't have test datasets).

Run configure to set up the required third party libraries:
```bash
(odmdev) [user:/code] master+* ± bash configure.sh reinstall
```

You can now make changes to the ODM source. When you are ready to test the changes you can simply invoke:

```bash
(odmdev) [user:/code] master+* ± ./run.sh --project-path /datasets mydataset
```
### Stop dev container
```bash
 docker  stop odmdev
```
### To come back to dev environement
change your_username to your username
```bash
docker start odmdev
docker exec -ti odmdev bash
su your_username
```


If you have questions, join the developer's chat at https://community.opendronemap.org/c/developers-chat/21

1. Try to keep commits clean and simple
2. Submit a pull request with detailed changes and test results
3. Have fun!

### Troubleshooting
The dev environment makes use of `opendronemap/nodeodm` by default. You may want to run 
`docker pull opendronemap/nodeodm` before running `./start-dev-env.sh` to avoid using an old cached version.

In order to make a clean build, remove `~/.odm-dev-home` and `ODM/.setupdevenv`.

## Credits

ODM makes use of [several libraries](https://github.com/OpenDroneMap/ODM/blob/master/snap/snapcraft.yaml#L36) and other awesome open source projects to perform its tasks. Among them we'd like to highlight:

 - [OpenSfM](https://github.com/mapillary/OpenSfM)
 - [OpenMVS](https://github.com/cdcseacave/openMVS/)
 - [PDAL](https://github.com/PDAL/PDAL)
 - [Entwine](https://entwine.io/)
 - [MVS Texturing](https://github.com/nmoehrle/mvs-texturing)
 - [GRASS GIS](https://grass.osgeo.org/)
 - [GDAL](https://gdal.org/)
 - [PoissonRecon](https://github.com/mkazhdan/PoissonRecon)


## Citation

> *OpenDroneMap Authors* ODM - A command line toolkit to generate maps, point clouds, 3D models and DEMs from drone, balloon or kite images. **OpenDroneMap/ODM GitHub Page** 2020; [https://github.com/OpenDroneMap/ODM](https://github.com/OpenDroneMap/ODM)

## Trademark

See [Trademark Guidelines](https://github.com/OpenDroneMap/documents/blob/master/TRADEMARK.md)
