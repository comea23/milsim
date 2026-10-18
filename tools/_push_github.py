"""临时脚本：经 api.github.com Git Data API 推送 milsim 工作树增量。

token 从环境变量 GH_TOKEN 读入（不落盘）。流程：ref→commit→tree（recursive）
→ 本地算 blob sha 对比（只传差异）→ create blob → post tree → post commit
→ patch ref → 整树 blob SHA 复核。
"""
import base64
import hashlib
import json
import os
import sys
import urllib.request

REPO = "comea23/milsim"
API = f"https://api.github.com/repos/{REPO}"
TOKEN = os.environ["GH_TOKEN"]


def api(method: str, url: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(
        API + url, method=method,
        data=None if body is None else json.dumps(body).encode(),
        headers={"Authorization": f"Bearer {TOKEN}",
                 "Accept": "application/vnd.github+json",
                 "User-Agent": "milsim-push"})
    with urllib.request.urlopen(req) as r:
        return json.loads(r.read())


def blob_sha(path: str) -> str:
    data = open(path, "rb").read()
    h = hashlib.sha1()
    h.update(b"blob %d\0" % len(data))
    h.update(data)
    return h.hexdigest()


def main() -> None:
    # 1. 收集候选文件
    from pathlib import Path
    root = Path(".")
    cands: set[str] = set()
    # 已跟踪改动
    import subprocess
    st = subprocess.run(["git", "status", "--short"], capture_output=True,
                        text=True, encoding="utf-8", check=True).stdout
    for line in st.splitlines():
        code, p = line[:2], line[3:].strip().strip('"')
        if code.strip() == "M":
            cands.add(p.encode("latin1", "backslashreplace")
                      .decode("gbk", "replace") if "\\3" in p else p)
    # 上面 git status 中文路径是八进制转义——改用 -z 模式重取
    st = subprocess.run(["git", "status", "--short", "-z"], capture_output=True,
                        check=True).stdout.decode("utf-8")
    cands = set()
    for entry in st.split("\0"):
        if not entry or entry.startswith("??"):
            continue
        cands.add(entry[3:])
    # 未跟踪白名单
    pats = [
        "src/milsim/models/damage/**/*.py", "src/milsim/models/supply/**/*.py",
        "src/milsim/models/task/**/*.py", "src/milsim/models/weapon/**/*.py",
        "src/milsim/models/mover/analytic.py",
        "tests/test_analytic.py", "tests/test_cmp_metrics.py",
        "tests/test_degrade.py", "tests/test_freq_hop.py",
        "tests/test_hitl.py", "tests/test_prob_table.py",
        "tests/test_task_manager.py", "tests/test_warhead.py",
        "tests/test_weapon.py",
        "tools/*.py", "tools/*.png", "tools/cmp_metrics/**/*.py",
        "tools/cmp_metrics/*.md",
        "docs/10-*.md",
        "demos/afsim_mover/analytic.txt", "demos/afsim_mover/degrade.txt",
        "demos/afsim_mover/hop_static_cmp.txt", "demos/afsim_mover/rocket.txt",
        "demos/afsim_mover/spy1d_static_cmp.txt",
        "demos/afsim_mover/warhead.txt",
        "demos/afsim_mover/cmp_movers/report.md",
        "demos/afsim_mover/cmp_movers/summary.csv",
        "demos/afsim_mover/cmp_movers/plots/*.png",
        "demos/afsim_mover/cmp_movers/scenarios/**/*.txt",
        "demos/afsim_cmp/*/summary.csv", "demos/afsim_cmp/*/report.md",
    ]
    for pat in pats:
        for f in root.glob(pat):
            if f.is_file():
                cands.add(f.as_posix())
    cands = sorted(c for c in cands if (root / c).is_file())
    print(f"candidates: {len(cands)}")

    # 2. 远端基线
    repo = api("GET", "")
    branch = repo["default_branch"]
    ref = api("GET", f"/git/ref/heads/{branch}")
    head_sha = ref["object"]["sha"]
    commit = api("GET", f"/git/commits/{head_sha}")
    base_tree = commit["tree"]["sha"]
    tree_full = api("GET", f"/git/trees/{base_tree}?recursive=1")
    remote = {e["path"]: e["sha"] for e in tree_full.get("tree", [])
              if e["type"] == "blob"}
    print(f"branch={branch} head={head_sha[:12]} remote_blobs={len(remote)}")

    # 3. 差异对比，只传变化
    changed = [(p, blob_sha(p)) for p in cands if remote.get(p) != blob_sha(p)]
    print(f"changed vs remote: {len(changed)}")
    if not changed:
        print("NOTHING_TO_PUSH")
        return

    tree_entries = []
    for path, sha in changed:
        if remote.get(path) == sha:
            continue
        data = open(path, "rb").read()
        if len(data) > 50 * 1024 * 1024:
            print(f"SKIP too big: {path}")
            continue
        enc = "base64" if b"\0" in data[:8000] else "utf-8"
        content = (base64.b64encode(data).decode() if enc == "base64"
                   else data.decode("utf-8"))
        blob = api("POST", "/git/blobs",
                   {"content": content, "encoding": enc})
        tree_entries.append({"path": path, "mode": "100644",
                             "type": "blob", "sha": blob["sha"]})
        print(f"  blob {path} ({enc}, {len(data)}B)")

    new_tree = api("POST", "/git/trees",
                   {"base_tree": base_tree, "tree": tree_entries})
    msg = ("v0.13.66: 域二~六对拍闭环 + 任务层/毁伤/武器/供应组件 + cmp_metrics "
           "算法库 + MCP 工具化封装\n\n"
           "- 域二 雷达探测 cmp_sensors（§5.12.10.1）、域三 干扰 cmp_jam"
           "（§5.14.16.1）、域四 武器 cmp_weapon（§5.11.8）、域六 通信 cmp_comm"
           "（§5.15.16）\n"
           "- 新组件：task / weapon / damage / supply / analytic mover\n"
           "- 新工具：cmp_metrics 算法库、calibrate_warhead、check_mover、"
           "check_radar_jam、mcp_server（MCP 封装）\n"
           "- tests 1512 通过；docs 00/10 同步")
    new_commit = api("POST", "/git/commits",
                     {"message": msg, "tree": new_tree["sha"],
                      "parents": [head_sha]})
    api("PATCH", f"/git/refs/heads/{branch}",
        {"sha": new_commit["sha"], "force": False})
    print(f"commit: {new_commit['sha'][:12]}")

    # 4. 整树 blob SHA 复核
    verify_tree = api("GET", f"/git/trees/{new_tree['sha']}?recursive=1")
    vt = {e["path"]: e["sha"] for e in verify_tree.get("tree", [])
          if e["type"] == "blob"}
    bad = [p for p, s in changed if vt.get(p) != s]
    missing = [p for p, s in changed if p not in vt]
    if bad or missing:
        print("VERIFY_FAIL", {"bad": bad, "missing": missing})
        sys.exit(1)
    print(f"VERIFY_OK: {len(changed)} paths match remote tree")


if __name__ == "__main__":
    main()
