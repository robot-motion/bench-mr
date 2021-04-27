import os,sys,shutil
from pdf2image import convert_from_path

curr_dir = os.getcwd()
original_dir = os.path.join(curr_dir, "stats/")

for filename in os.listdir(original_dir) :
    original_path = original_dir + filename
    base, extension = os.path.splitext(filename)
    new_path = os.path.join(curr_dir, "img", base + ".png")
    pages = convert_from_path(original_path, fmt="png")
    for page in pages :
        print("Saving: ", new_path)
        page.save(new_path, "PNG")

