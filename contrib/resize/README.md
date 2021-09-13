# Resize

Resize a dataset (and optional GCP file).

Resizes images, keeps Exif data. The EXIF width and height attributes will be updated accordingly also. ODM GCP files are scaled also.

Usage:

```
pip install -r requirements.txt
python3 resize.py -i images/ -o resized/ 25%
python3 resize.py -i images/1.JPG -o resized.JPG 25%
python3 resize.py -i gcp_list.txt -o resized_gcp_list.txt
```

Originally forked from https://github.com/pierotofy/exifimageresize
