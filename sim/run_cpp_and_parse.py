import subprocess

def run_and_parse(exe_path):
    out = subprocess.check_output([exe_path], text=True)

    results = []
    cur = None
    cur_sol = None

    for line in out.splitlines():
        s = line.strip()
        if not s:
            continue

        if s.startswith("Target"):
            if cur:
                if cur_sol:
                    cur["solutions"].append(cur_sol)
                    cur_sol = None
                results.append(cur)

            inside = s[s.find("(")+1:s.find(")")]
            x_str, y_str = [t.strip() for t in inside.split(",")]
            cur = {"target": (float(x_str), float(y_str)), "reachable": None, "solutions": []}
            continue

        if s.startswith("reachable:") and cur:
            cur["reachable"] = (s.split(":")[1].strip().lower() == "true")
            continue

        if s.startswith("solution") and cur:
            if cur_sol:
                cur["solutions"].append(cur_sol)
            cur_sol = {"theta1": None, "theta2": None}
            continue

        if s.startswith("theta1") and cur_sol is not None:
            cur_sol["theta1"] = float(s.split("=")[1].strip())
            continue

        if s.startswith("theta2") and cur_sol is not None:
            cur_sol["theta2"] = float(s.split("=")[1].strip())
            continue

    if cur:
        if cur_sol:
            cur["solutions"].append(cur_sol)
        results.append(cur)

    return results