#ODM 0.3.1 under Debian 8.10 jessie  - 2018-02-27 by yjmenezes
#https://github.com/OpenDroneMap/OpenDroneMap/wiki/Docker
#git clone https://github.com/OpenDroneMap/OpenDroneMap.git
git clone https://github.com/yjmenezes/OpenDroneMap.git
cd OpenDroneMap
# list images
docker images
#remove old my_odm_image if necessary
docker rmi my_odm_image 
#build a fresh one using instructions from ./Dockerfile
docker build -t my_odm_image . 
#run tests with supplied image set. 
#Mapped host directories for output. -v host_path:container_path
cd tests/test_data/
sudo rm -r odm_* opensfm
docker run -it --rm \
    -v $(pwd)/gcp_list.txt:/code/gcp_list.txt \
    -v $(pwd)/images:/code/images \
    -v $(pwd)/opensfm:/code/opensfm \
    -v $(pwd)/odm_meshing:/code/odm_meshing \
    -v $(pwd)/odm_georeferencing:/code/odm_georeferencing \
    -v $(pwd)/odm_orthophoto:/code/odm_orthophoto \
    -v $(pwd)/odm_texturing:/code/odm_texturing \
    my_odm_image  --mesh-size 100000

