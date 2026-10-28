import hashlib, json, os, re, time, uuid
from pathlib import Path
from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.responses import JSONResponse

DATA_DIR = Path(os.environ.get("DATA_DIR", "./data"))
MAX_BYTES = int(os.environ.get("MAX_STEP_BYTES", 200 * 1024 * 1024))
app = FastAPI(title="step2xflr5 geometry service")


def job_dir(job_id: str) -> Path:
    if not re.fullmatch(r"job_[0-9a-f]{12}", job_id):
        raise HTTPException(404, "bad job id")
    d = DATA_DIR / "jobs" / job_id
    if not d.exists():
        raise HTTPException(404, "job not found")
    return d


def fail(job_id, stage, code, message, status=422, retryable=False):
    return JSONResponse(status_code=status, content={
        "status": "FAIL", "job_id": job_id, "stage": stage,
        "error": {"code": code, "message": message, "retryable": retryable}, "errors": []})


def write_manifest(d: Path, patch: dict):
    mp = d / "manifest.json"
    m = json.loads(mp.read_text()) if mp.exists() else {}
    m.update(patch)
    mp.write_text(json.dumps(m, indent=2))
    return m


@app.get("/health")
def health():
    return {"ok": True}


@app.post("/v1/jobs")
async def a1_intake(file: UploadFile = File(...)):
    name = file.filename or ""
    if not name.lower().endswith((".step", ".stp")):
        return fail(None, "A1", "BAD_EXTENSION", "File must be .step or .stp")
    data = await file.read()
    if not data:
        return fail(None, "A1", "EMPTY_FILE", "File is empty")
    if len(data) > MAX_BYTES:
        return fail(None, "A1", "TOO_LARGE", f"File exceeds {MAX_BYTES} bytes", 413)
    head = data[:2048].decode("latin-1", "replace")
    if "ISO-10303-21" not in head:
        return fail(None, "A1", "NOT_STEP", "Missing ISO-10303-21 header; not a STEP Part 21 file")
    job_id = "job_" + uuid.uuid4().hex[:12]
    d = DATA_DIR / "jobs" / job_id
    (d / "input").mkdir(parents=True)
    (d / "artifacts").mkdir()
    (d / "input" / "original.step").write_bytes(data)
    sha = hashlib.sha256(data).hexdigest()
    write_manifest(d, {"job_id": job_id, "original_filename": name, "size_bytes": len(data),
                       "source_sha256": sha, "created_at": time.time(), "stages": {"A1": "PASS"}})
    return {"status": "PASS", "job_id": job_id, "stage": "A1", "filename": name,
            "size_bytes": len(data), "source_sha256": sha}


@app.post("/v1/jobs/{job_id}/stages/A2")
def a2_extract(job_id: str):
    d = job_dir(job_id)
    from .cad import read_step, summarize
    try:
        shape = read_step(str(d / "input" / "original.step"))
        result = summarize(shape)
    except Exception as e:  # report, never hang
        write_manifest(d, {"stages": {**json.loads((d / "manifest.json").read_text())["stages"], "A2": "FAIL"}})
        return fail(job_id, "A2", "STEP_READ_FAILED", str(e))
    if result["solid_count"] == 0 and result["face_count"] == 0:
        return fail(job_id, "A2", "NO_GEOMETRY", "STEP parsed but contains no solids or faces")
    (d / "artifacts" / "a2_extraction.json").write_text(json.dumps(result, indent=2))
    st = json.loads((d / "manifest.json").read_text())["stages"]
    st["A2"] = "PASS"
    write_manifest(d, {"stages": st})
    return {"status": "PASS", "job_id": job_id, "stage": "A2", "result": result}


@app.get("/v1/jobs/{job_id}/manifest")
def manifest(job_id: str):
    return json.loads((job_dir(job_id) / "manifest.json").read_text())


# ---------------- A3-A6 ----------------
def _stages(d):
    return json.loads((d / "manifest.json").read_text())["stages"]


def _mark(d, stage, status):
    st = _stages(d)
    st[stage] = status
    write_manifest(d, {"stages": st})


def _art(d, name, obj=None):
    p = d / "artifacts" / name
    if obj is not None:
        p.write_text(json.dumps(obj, indent=2))
    return json.loads(p.read_text())


def _need(d, stage, prev):
    if _stages(d).get(prev) != "PASS":
        raise HTTPException(409, f"{stage} requires {prev} to have passed")


def _load(d):
    from .cad import read_step
    from . import wing
    solids = wing.list_solids(read_step(str(d / "input" / "original.step")))
    return wing, solids


def _run(job_id, stage, prev, fn):
    d = job_dir(job_id)
    _need(d, stage, prev)
    from .wing import StageError
    try:
        result = fn(d)
    except StageError as e:
        _mark(d, stage, "FAIL")
        r = fail(job_id, stage, e.code, e.message)
        return r
    except Exception as e:
        _mark(d, stage, "FAIL")
        return fail(job_id, stage, "UNEXPECTED_" + type(e).__name__, str(e))
    _mark(d, stage, "PASS")
    return {"status": "PASS", "job_id": job_id, "stage": stage, "result": result}


def _cfg(body):
    return (body or {}).get("config") or {}


@app.post("/v1/jobs/{job_id}/stages/A3")
def a3(job_id: str, body: dict = None):
    def fn(d):
        wing, solids = _load(d)
        comps, wi = wing.classify(solids)
        res = {"components": comps, "main_wing_index": wi, "needs_manual_review": len(comps) > 1 and False}
        _art(d, "a3_components.json", res)
        return res
    return _run(job_id, "A3", "A2", fn)


@app.post("/v1/jobs/{job_id}/stages/A4")
def a4(job_id: str, body: dict = None):
    def fn(d):
        wing, solids = _load(d)
        a3r = _art(d, "a3_components.json")
        comp = a3r["components"][a3r["main_wing_index"]]
        m = wing.measure(wing.get_solid(solids, comp), comp, mode=_cfg(body).get("span_mode", "auto"))
        _art(d, "a4_measurement.json", m)
        return m
    return _run(job_id, "A4", "A3", fn)


@app.post("/v1/jobs/{job_id}/stages/A5")
def a5(job_id: str, body: dict = None):
    def fn(d):
        wing, solids = _load(d)
        a3r = _art(d, "a3_components.json")
        comp = a3r["components"][a3r["main_wing_index"]]
        n = int(_cfg(body).get("n_sections", 5))
        if not 2 <= n <= 25:
            raise wing.StageError("BAD_CONFIG", "n_sections must be 2..25")
        secs = wing.extract_sections(wing.get_solid(solids, comp), comp, _art(d, "a4_measurement.json"), n)
        bad = [s["nominal_fraction"] for s in secs if s["status"] != "PASS"]
        _art(d, "a5_sections.json", {"sections": secs})
        if bad:
            raise wing.StageError("SECTION_FAILED", f"Sections failed at span fractions {bad}")
        return {"n_sections": len(secs), "sections": [{k: v for k, v in s.items() if k != "points"} for s in secs]}
    return _run(job_id, "A5", "A4", fn)


@app.post("/v1/jobs/{job_id}/stages/A6")
def a6(job_id: str, body: dict = None):
    def fn(d):
        from . import wing
        import numpy as np
        a3r = _art(d, "a3_components.json")
        comp = a3r["components"][a3r["main_wing_index"]]
        _, c, t = wing._axes(comp)
        secs = _art(d, "a5_sections.json")["sections"]
        tol = float(_cfg(body).get("max_deviation", 0.005))
        out, files, u_dir = [], [], None
        for k, s in enumerate(secs):
            r = wing.reconstruct(np.array(s["points"]), u_dir=u_dir)
            u_dir = r["u_dir"]  # keep orientation consistent across sections
            name = f"section_{k:02d}.dat"
            lines = [f"S2X_section_{k:02d}"] + [f"{x:.6f} {y:.6f}" for x, y in r["coords"]]
            (d / "artifacts" / name).write_text("\n".join(lines) + "\n")
            files.append(name)
            out.append({"index": k, "nominal_fraction": s["nominal_fraction"], "y_from_root": s["y_from_root"],
                        "chord": r["chord"], "twist_deg": r["twist_deg"], "le_xy_raw": r["le_xy_raw"], "u_dir": r["u_dir"],
                        "max_deviation_over_chord": r["max_deviation_over_chord"],
                        "max_thickness_over_chord": r["max_thickness_over_chord"],
                        "te_thickness_over_chord": r["te_thickness_over_chord"], "dat_file": name,
                        "within_tolerance": r["max_deviation_over_chord"] <= tol})
        res = {"airfoil_sections": out, "dat_files": files, "tolerance": tol}
        _art(d, "a6_airfoils.json", res)
        bad = [o["index"] for o in out if not o["within_tolerance"]]
        if bad:
            raise wing.StageError("AIRFOIL_DEVIATION", f"Sections {bad} deviate more than {tol} chord from the CAD curve")
        return res
    return _run(job_id, "A6", "A5", fn)


from fastapi.responses import FileResponse


@app.get("/v1/jobs/{job_id}/artifacts/{name}")
def artifact(job_id: str, name: str):
    d = job_dir(job_id)
    if not re.fullmatch(r"[A-Za-z0-9_.\-]+", name) or not (d / "artifacts" / name).is_file():
        raise HTTPException(404, "artifact not found")
    return FileResponse(d / "artifacts" / name)


# ---------------- A7 export / A9 validate / A13 package ----------------
@app.post("/v1/jobs/{job_id}/stages/A7")
def a7(job_id: str, body: dict = None):
    def fn(d):
        from . import export
        model = export.build_wing_model(_art(d, "a3_components.json"), _art(d, "a4_measurement.json"),
                                        _art(d, "a5_sections.json"), _art(d, "a6_airfoils.json"),
                                        name=_cfg(body).get("wing_name", "STEP2XFLR5_wing"))
        _art(d, "wing_model.json", model)
        (d / "artifacts" / "xflr5_wing.xml").write_text(export.xflr5_xml(model))
        (d / "artifacts" / "xflr5_wing.txt").write_text(export.xflr5_txt(model))
        return {"sections": len(model["sections"]), "half_span_m": model["half_span_m"], "files": ["xflr5_wing.xml", "xflr5_wing.txt"], "notes": model["notes"]}
    return _run(job_id, "A7", "A6", fn)


@app.post("/v1/jobs/{job_id}/stages/A9")
def a9(job_id: str, body: dict = None):
    def fn(d):
        from . import export
        res = export.validate_export((d / "artifacts" / "xflr5_wing.xml").read_text(), _art(d, "a4_measurement.json"), _art(d, "a6_airfoils.json"))
        _art(d, "a9_validation.json", res)
        return res
    return _run(job_id, "A9", "A7", fn)


@app.post("/v1/jobs/{job_id}/stages/A13")
def a13(job_id: str, body: dict = None):
    def fn(d):
        from . import export
        m = json.loads((d / "manifest.json").read_text())
        data, names = export.build_zip(d, job_id, m, _art(d, "a3_components.json"), _art(d, "a4_measurement.json"),
                                       _art(d, "a6_airfoils.json"), _art(d, "a9_validation.json"), _art(d, "wing_model.json"))
        (d / "artifacts" / "result.zip").write_bytes(data)
        return {"zip": "result.zip", "size_bytes": len(data), "members": names,
                "sha256": hashlib.sha256(data).hexdigest(), "download": f"/v1/jobs/{job_id}/artifacts/result.zip"}
    return _run(job_id, "A13", "A9", fn)
