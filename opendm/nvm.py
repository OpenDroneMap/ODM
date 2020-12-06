import os
from opendm import log

def replace_nvm_images(src_nvm_file, img_map, dst_nvm_file):
    """
    Create a new NVM file from an existing NVM file
    replacing the image references based on img_map
    where img_map is a dict { "old_image" --> "new_image" } (filename only).
    The function does not write the points information (they are discarded)
    """

    with open(src_nvm_file) as f:
        lines = list(map(str.strip, f.read().split("\n")))
    
    # Quick check
    if len(lines) < 3 or lines[0] != "NVM_V3" or lines[1].strip() != "":
        raise Exception("%s does not seem to be a valid NVM file" % src_nvm_file)
    
    num_images = int(lines[2])
    entries = []

    for l in lines[3:3+num_images]:
        image_path, *p = l.split(" ")

        dir_name = os.path.dirname(image_path)
        file_name = os.path.basename(image_path)

        new_filename = img_map.get(file_name)
        if new_filename is not None:
            entries.append("%s %s" % (os.path.join(dir_name, new_filename), " ".join(p)))
        else:
            log.ODM_WARNING("Cannot find %s in image map for %s" % (file_name, dst_nvm_file)) 
    
    if num_images != len(entries):
        raise Exception("Cannot write %s, not all band images have been matched" % dst_nvm_file)

    with open(dst_nvm_file, "w") as f:
        f.write("NVM_V3\n\n%s\n" % len(entries))
        f.write("\n".join(entries))
        f.write("\n\n0\n0\n\n0")
            