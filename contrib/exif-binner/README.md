# exif_binner.py 

Bins multispectral drone images by spectral band, using EXIF data. Also verifies that each bin is complete (i.e. contains all expected bands) and can log errors to a CSV file. Excludes RGB images by default.

## Requirements

- [Pillow](https://pillow.readthedocs.io/en/stable/installation.html) library for reading images and EXIF data.
- [tqdm](https://github.com/tqdm/tqdm#installation) for progress bars - can be removed

## Usage

```
exif_binner.py <args> <path to folder of images to rename> <output folder>
```

Optional arguments:

- `-b`/`--bands <integer>`: Number of expected bands per capture. Default: `5`
- `-s`/`--sequential <True/False>`: Use sequential capture group in filenames rather than original capture ID. Default: `True`
- `-z`/`--zero_pad <integer>`: If using sequential capture groups, zero-pad the group number to this many digits. 0 for no padding, -1 for auto padding. Default: `5`
- `-w`/`--whitespace_replace <string>`: Replace whitespace characters with this character. Default: `-`
- `-l`/`--logfile <filename>`: Write processed image metadata to this CSV file
- `-r`/`--replace_filename <string>`: Use this instead of using the original filename in new filenames.
- `-f`/`--force`: Do not ask for processing confirmation.
- `-g`/`--no_grouping`: Do not apply grouping, only validate and add band name.
- Show these on the command line with `-h`/`--help`.
