# Merge Preview

Quickly projects drone images on a map by using georeferencing, camera angles and a global DTM. The images are then merged using ODM's split-merge algorithms.

Quality is obviously not good, works only for nadir-only images and requires the images to have gimbal/camera angle information (not all drones provide this information).

Usage:

```
# Install DDB (required for geoprojection)

curl -fsSL https://get.dronedb.app -o get-ddb.sh
sh get-ddb.sh

# Run

python3 mergepreview.py -i images/*.JPG --size 25%
```

## Example

![screen](https://user-images.githubusercontent.com/1951843/134249725-e178489a-e271-4244-abed-e624cd510b88.png)


[Sheffield Park](https://community.opendronemap.org/t/sheffield-park-1/58) images processed with this script.

## Disclaimer

This script is highly experimental. We welcome contributions to improve it.
