from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp


def read_step(path):
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise ValueError("OpenCASCADE could not read the STEP file")
    if r.TransferRoots() == 0:
        raise ValueError("STEP contains no transferable roots")
    return r.OneShape()


def _count(shape, kind):
    e = TopExp_Explorer(shape, kind)
    n = 0
    while e.More():
        n += 1
        e.Next()
    return n


def summarize(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    cmin, cmax = box.CornerMin(), box.CornerMax()
    xmin, ymin, zmin = cmin.X(), cmin.Y(), cmin.Z()
    xmax, ymax, zmax = cmax.X(), cmax.Y(), cmax.Z()
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return {
        "solid_count": _count(shape, TopAbs_SOLID),
        "shell_count": _count(shape, TopAbs_SHELL),
        "face_count": _count(shape, TopAbs_FACE),
        "bbox": {"min": [xmin, ymin, zmin], "max": [xmax, ymax, zmax],
                 "size": [xmax - xmin, ymax - ymin, zmax - zmin]},
        "volume": props.Mass(),
        "is_valid": bool(BRepCheck_Analyzer(shape).IsValid()),
        "units_note": "STEP length unit is converted to mm by the OCC reader by default; verify with a known-size model.",
    }
