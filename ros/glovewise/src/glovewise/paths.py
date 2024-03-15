import os


def extpath(old_path, new_ext):
    fdir, fname = os.path.split(old_path)
    outpath = os.path.join(fdir, fname.split(".")[0]) + new_ext
    return outpath
