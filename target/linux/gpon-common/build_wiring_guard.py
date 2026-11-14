#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""build_wiring_guard.py -- offline proof that the SHARED files- trees (the GPON
core, gpon-common; the Ethernet layer, eth-common) are actually wired into
every target that uses them, and wired the only way that works.

WHAT THIS IS
  A no-device, no-build guard over the build wiring of the common GPON layer:
  target/linux/gpon-common/files-<ver>/drivers/net/gpon/.  It reads Makefiles
  and Kconfigs and asserts the handful of invariants whose violation is SILENT
  -- the ones a green build would not reveal.

WHY IT IS COMMON -- and which targets it covers
  Operator, 2026-08-05: "la idea es poner en comun el codigo que corresponde
  para no tener mucho duplicado", and on the two per-target monoliths carrying
  a private protocol copy each: "mal, poner en comun".  The shared tree is
  compiled by realtek-luna (MIPS32, big-endian) and realtek-elnath (aarch64,
  little-endian) from ONE authored copy; this guard is what proves the "one
  copy, two targets" claim instead of asserting it.

THE CORE/SHELL RULE IT OBEYS
  It is a build-wiring guard, not a driver: it touches no device, opens no
  register and never runs on the board.  Nothing it checks may ever require
  hardware, and no file it guards may ever gain an MMIO access -- that second
  half is proven separately by gpon_core_purity_test in dev/rtl9607c-test/.

  Run it with --self-check to see every assertion FAIL on a mutated scratch
  copy.  A guard whose removal leaves the gate green is not a guard.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

VER = "6.18"
SHARED = "target/linux/gpon-common"
SHARED_FILES = SHARED + "/files-" + VER
GPON_DIR = SHARED_FILES + "/drivers/net/gpon"
TARGETS = ("realtek-luna", "realtek-elnath")
# Every shared overlay -> the targets that pull it.  eth-common is Ethernet, not
# GPON: a target with an Ethernet driver of ours pulls it without being a GPON target.
OVERLAYS = {SHARED: TARGETS, "target/linux/eth-common": TARGETS + ("zte-zx",)}
REEXPAND = "Kernel/Patch := $(Kernel/Patch/Default)"


def files_dir_re(overlay):
    """The one line that makes @overlay reachable.  `+=` and nothing else:
    include/kernel.mk:44 sets FILES_DIR with `?=`."""
    return re.compile(r'^FILES_DIR\s*\+=\s*"\$\(TOPDIR\)/%s/files-\$\(KERNEL_PATCHVER\)"\s*$'
                      % re.escape(overlay))


def overlays_of(target):
    return [o for o, ts in OVERLAYS.items() if target in ts]


def pulled_overlays(root):
    """-> {target: [shared overlay]} for every target Makefile that names one: the closed census W1 checks, so a
    target that starts pulling a shared tree is checked the day it does, listed in OVERLAYS or not."""
    found, base = {}, os.path.join(root, "target", "linux")
    for t in sorted(os.listdir(base)):
        try:
            with open(os.path.join(base, t, "Makefile"), encoding="utf-8", errors="replace") as f:
                txt = f.read()
        except OSError:
            continue
        named = [o for o in OVERLAYS if "/%s/files-" % o in txt]
        if named:
            found[t] = named
    return found


HUNK_FILE = re.compile(r'^\+\+\+\s+(?:[ab]|linux-[\d.]+)/(\S+)', re.M)


def patch_sections(text, files):
    """The per-file sections of a unified diff whose +++ names one of @files."""
    lines = text.splitlines(True)
    out, keep = [], False
    for i, line in enumerate(lines):
        if line.startswith("--- ") and i + 1 < len(lines) and lines[i + 1].startswith("+++ "):
            m = HUNK_FILE.match(lines[i + 1])
            keep = bool(m and m.group(1) in files)
        if keep:
            out.append(line)
    return "".join(out)


class Guard:
    def __init__(self, root):
        self.root = root
        self.fails = []
        self.checks = 0

    def p(self, *a):
        return os.path.join(self.root, *a)

    def check(self, ok, name, detail):
        self.checks += 1
        if not ok:
            self.fails.append("%-28s %s" % (name, detail))
        return ok

    def read(self, rel):
        try:
            with open(self.p(rel), "r", encoding="utf-8", errors="replace") as f:
                return f.read()
        except OSError:
            return None

    # -- W1 ---------------------------------------------------------------
    def w1_files_dir_line(self):
        """Each target appends each shared tree it uses, AFTER the target.mk
        include and BEFORE the Kernel/Patch re-expansion.

        Placed before the include, kernel.mk's `?=` never fires and the
        target's OWN files-<ver> disappears -- the whole vendor driver with it.
        Placed after the re-expansion (or with none), the copy command was
        frozen without it and the `+=` is inert (realtek-luna/Makefile).
        """
        pulled = pulled_overlays(self.root)
        targets = sorted({t for ts in OVERLAYS.values() for t in ts} | set(pulled))
        for t in targets:
            rel = "target/linux/%s/Makefile" % t
            txt = self.read(rel)
            if not self.check(txt is not None, "W1.exists", rel + " missing"):
                continue
            lines = txt.splitlines()
            inc = [i for i, l in enumerate(lines)
                   if l.strip() == "include $(INCLUDE_DIR)/target.mk"]
            bt = [i for i, l in enumerate(lines)
                  if "call BuildTarget" in l]
            rx = [i for i, l in enumerate(lines) if l.strip() == REEXPAND]
            anchored = self.check(len(inc) == 1 and len(bt) == 1, "W1.anchors/" + t,
                                  "need one target.mk include and one BuildTarget")
            for o in sorted(set(overlays_of(t)) | set(pulled.get(t, ()))):
                idx = [i for i, l in enumerate(lines) if files_dir_re(o).match(l)]
                if not self.check(len(idx) == 1, "W1.append/" + t,
                                  "expected exactly 1 `FILES_DIR +=` %s line, got %d"
                                  % (o, len(idx))):
                    continue
                if anchored:
                    self.check(inc[0] < idx[0] < bt[0], "W1.order/" + t,
                               "%s FILES_DIR line at %d must sit between include (%d) and "
                               "BuildTarget (%d)" % (o, idx[0] + 1, inc[0] + 1, bt[0] + 1))
                self.check(any(idx[0] < i for i in rx), "W1.reexpand/" + t,
                           "no `%s` after the %s FILES_DIR line: the += is inert" % (REEXPAND, o))
            self.check(not re.search(r'^FILES_DIR\s*:?=[^+]', txt, re.M),
                       "W1.noclobber/" + t,
                       "a plain FILES_DIR assignment would defeat kernel.mk's `?=`")

    # -- W2 ---------------------------------------------------------------
    def w2_trees_disjoint(self):
        """No path may exist in both the shared tree and a target's tree.

        A collision is resolved SILENTLY by $(CP) order (rules.mk:314
        CP:=cp -fpR, later wins), so one target would build a different file
        from the other with nothing to see.
        """
        def rels(base):
            out = set()
            b = self.p(base)
            for dirpath, dirnames, filenames in os.walk(b, followlinks=False):
                for fn in filenames:
                    out.add(os.path.relpath(os.path.join(dirpath, fn), b))
            return out

        trees = {o: rels("%s/files-%s" % (o, VER)) for o in OVERLAYS}
        for o, ts in OVERLAYS.items():
            self.check(bool(trees[o]), "W2.nonempty", o + " has no files")
            for t in ts:
                clash = sorted(trees[o] & rels("target/linux/%s/files-%s" % (t, VER)))
                self.check(not clash, "W2.disjoint/" + t,
                           "path in %s AND %s (later $(CP) silently wins): %s" % (o, t, ", ".join(clash)))
        names = sorted(OVERLAYS)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                clash = sorted(trees[a] & trees[b])
                self.check(not clash, "W2.disjoint/overlays",
                           "path in %s AND %s: %s" % (a, b, ", ".join(clash)))

    # -- W3 ---------------------------------------------------------------
    def w3_o2(self):
        """The shared directory must carry its own -O2.

        realtek-luna forces -Os kernel-wide (patches-6.18/100-cc-optimize-for-
        size.patch) and exempts its datapath objects one by one.  Code moved
        out of gpon-rtl9602c.o into drivers/net/gpon/ silently drops to -Os,
        which shows up only as a throughput number -- on a driver at parity.
        """
        mk = self.read(GPON_DIR + "/Makefile")
        if not self.check(mk is not None, "W3.exists", GPON_DIR + "/Makefile missing"):
            return
        body = "\n".join(l for l in mk.splitlines() if not l.lstrip().startswith("#"))
        self.check(re.search(r'^\s*ccflags-y\s*\+=.*\B-O2\b', body, re.M) is not None,
                   "W3.o2", "no `ccflags-y += -O2`: luna would build this "
                            "directory at -Os")

    # -- W4 ---------------------------------------------------------------
    def w4_objects_accounted(self):
        """Every .c must be BUILT or explicitly PENDING-with-a-reason.

        Three states, never two.  Declared-with-no-source fails the build
        loudly, which is fine.  The dangerous direction is the other one: a .c
        that no line names is dead source which silently never compiles, while
        every test of it passes.  A staged carve legitimately has sources in
        the tree before their consumer moves -- so that state gets a NAME and a
        REASON (`# gpon-pending: <obj> -- why`) instead of being indistinguish-
        able from an oversight.
        """
        mk = self.read(GPON_DIR + "/Makefile")
        if mk is None:
            return
        body = "\n".join(l for l in mk.splitlines() if not l.lstrip().startswith("#"))
        built = set()
        for m in re.finditer(r'^\s*(?:obj-\S+|\S+-\S+)\s*\+?=\s*(.+)$', body, re.M):
            for tok in m.group(1).split():
                if tok.endswith(".o"):
                    built.add(tok[:-2])
        pending = {}
        for m in re.finditer(r'^#\s*gpon-pending:\s*(\S+)\.o\s*--\s*(.+)$', mk, re.M):
            pending[m.group(1)] = m.group(2).strip()
        d = self.p(GPON_DIR)
        present = {f[:-2] for f in os.listdir(d) if f.endswith(".c")} if os.path.isdir(d) else set()

        self.check(not (built - present), "W4.declared_no_src",
                   "declared but no .c: " + ", ".join(sorted(built - present)))
        self.check(not (set(pending) - present), "W4.pending_no_src",
                   "pending but no .c: " + ", ".join(sorted(set(pending) - present)))
        self.check(not (built & set(pending)), "W4.both",
                   "both built and pending: " + ", ".join(sorted(built & set(pending))))
        unaccounted = present - built - set(pending)
        self.check(not unaccounted, "W4.src_never_built",
                   "source present, neither built nor `# gpon-pending:` (silently "
                   "dead): " + ", ".join(sorted(unaccounted)))
        thin = sorted(k for k, v in pending.items() if len(v) < 20)
        self.check(not thin, "W4.pending_reason",
                   "pending with no real reason given: " + ", ".join(thin))

    # -- W5 ---------------------------------------------------------------
    def w5_overlay_is_the_prepatch_base(self):
        """Every patch touching an overlaid file must APPLY to the overlay copy,
        in OpenWrt's order (generic backport/pending/hack, then the target's).

        files- overlays are copied in BEFORE patches (include/quilt.mk:93-107),
        so an overlay holding POST-patch bytes makes the patch fail on a fresh
        prepare.  MEASURED 2026-09-21: arch/mips/Kconfig overlaid with the
        prepared tree's bytes; generic pending 300 then failed 1 hunk of 2.
        The earlier form walked only the target's own patches and only a/b
        hunk paths, so neither generic 300 nor linux-<ver>/ paths were seen.
        """
        pdirs = ["target/linux/generic/%s-%s" % (k, VER)
                 for k in ("backport", "pending", "hack")]
        for t in TARGETS:
            provided = {}
            for base in ["%s/files-%s" % (o, VER) for o in overlays_of(t)] + \
                    ["target/linux/%s/files-%s" % (t, VER)]:
                b = self.p(base)
                for dirpath, _, filenames in os.walk(b, followlinks=False):
                    for fn in filenames:
                        path = os.path.join(dirpath, fn)
                        provided[os.path.relpath(path, b)] = path
            tmp = tempfile.mkdtemp(prefix="gpon-w5-")
            try:
                for pdir in pdirs + ["target/linux/%s/patches-%s" % (t, VER)]:
                    d = self.p(pdir)
                    for fn in sorted(os.listdir(d)) if os.path.isdir(d) else []:
                        if not fn.endswith(".patch"):
                            continue
                        with open(os.path.join(d, fn), "r", encoding="utf-8",
                                  errors="replace") as f:
                            text = f.read()
                        files = sorted({m for m in HUNK_FILE.findall(text)
                                        if m in provided})
                        if not files:
                            continue
                        for rel in files:
                            dst = os.path.join(tmp, rel)
                            if not os.path.exists(dst):
                                os.makedirs(os.path.dirname(dst), exist_ok=True)
                                shutil.copyfile(provided[rel], dst)
                        p = subprocess.run(["patch", "-p1", "-s", "-f",
                                            "--no-backup-if-mismatch"], cwd=tmp,
                                           input=patch_sections(text, files),
                                           capture_output=True, text=True)
                        self.check(p.returncode == 0, "W5.prepatch/" + t,
                                   "%s does not apply to the overlay copy of %s -- "
                                   "the overlay must hold the PRE-patch bytes: %s"
                                   % (fn, ", ".join(files),
                                      (p.stdout + p.stderr).strip()[:200]))
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    # -- W6 ---------------------------------------------------------------
    def w6_no_scanned_makefile(self):
        """No Makefile in the shared tree's top 3 levels.

        include/scan.mk:85 greps `find -L target/linux -maxdepth 3 -name Makefile`
        for `call BuildTarget`.  gpon-common is a files- tree, not a target, and
        must stay invisible to target enumeration.
        """
        for o in OVERLAYS:
            for depth, sub in ((1, o), (2, o + "/files-" + VER),
                               (3, o + "/files-" + VER + "/drivers")):
                p = self.p(sub, "Makefile")
                self.check(not os.path.exists(p), "W6.depth%d" % depth,
                           "%s is at scan.mk depth<=3" % os.path.join(sub, "Makefile"))

    # -- W7 ---------------------------------------------------------------
    def w7_no_symlinks(self):
        """No symlink anywhere in the shared tree.

        scripts/timestamp.pl:21 does `next if -l $file`, so an edit behind a
        symlink NEVER re-triggers the prepare step: the kernel builds a stale
        core, silently.  rules.mk:314 `cp -fpR` also copies the link itself,
        whose relative path cannot resolve inside build_dir.  And the toplevel
        scan uses `find -L`, which would follow it off the tree.
        """
        bad = []
        for o in OVERLAYS:
            for dirpath, dirnames, filenames in os.walk(self.p(o), followlinks=False):
                for n in list(dirnames) + list(filenames):
                    if os.path.islink(os.path.join(dirpath, n)):
                        bad.append(os.path.relpath(os.path.join(dirpath, n), self.root))
        self.check(not bad, "W7.symlink", "symlink in shared tree: " + ", ".join(sorted(bad)))

    # -- W8 ---------------------------------------------------------------
    def w8_parent_wiring(self):
        """The overlaid drivers/net/{Kconfig,Makefile} really reach gpon/."""
        kc = self.read(SHARED_FILES + "/drivers/net/Kconfig")
        mk = self.read(SHARED_FILES + "/drivers/net/Makefile")
        self.check(kc is not None and 'source "drivers/net/gpon/Kconfig"' in kc,
                   "W8.kconfig", "drivers/net/Kconfig does not source gpon/Kconfig")
        self.check(mk is not None
                   and re.search(r'^obj-\$\(CONFIG_GPON_CORE\)\s*\+=\s*gpon/\s*$',
                                 mk or "", re.M) is not None,
                   "W8.makefile", "drivers/net/Makefile does not descend into gpon/")

    # -- W9 ---------------------------------------------------------------
    def w9_someone_selects(self):
        """At least one vendor driver must `select GPON_CORE` in EACH target.

        Nobody selecting it is the quietest possible failure: the parent never
        descends, the whole shared core is never compiled, and every build is
        green.  GPON_CORE deliberately has no `default`, so a select is the
        only way in -- and no config-<ver> needs editing.
        """
        want = {
            "realtek-luna": "target/linux/realtek-luna/files-%s/drivers/net/ethernet/"
                            "realtek/Kconfig" % VER,
            "realtek-elnath": "target/linux/realtek-elnath/files-%s/drivers/net/ethernet/"
                              "cortina/Kconfig" % VER,
        }
        for t, rel in want.items():
            txt = self.read(rel)
            self.check(txt is not None and re.search(r'^\s*select\s+GPON_CORE\b', txt, re.M),
                       "W9.select/" + t,
                       "no `select GPON_CORE` in %s -- the shared core would never "
                       "be compiled for this target, silently" % rel)
        kc = self.read(GPON_DIR + "/Kconfig")
        self.check(kc is not None and re.search(r'^config GPON_CORE\s*$', kc or "", re.M),
                   "W9.declared", "GPON_CORE is not declared in " + GPON_DIR + "/Kconfig")

    # -- W10 --------------------------------------------------------------
    def w10_shared_header_include_path(self):
        """A target that INCLUDES a shared header must carry the -I that finds it.

        The shared tree is a SEPARATE files- overlay merged into the kernel
        source where upstream's own directories land, so a shared header is
        NOT reachable by a relative include from a vendor driver directory.
        Each such directory's Makefile must add

            ccflags-y += -I$(srctree)/drivers/net/gpon

        and the failure without it is loud but LATE -- a "No such file or
        directory" deep in a target build, naming the header and not the
        reason.  It is guarded HERE because the fix is one line in a file
        nothing else checks, and because it was already recorded once as a
        blocker in the Luna driver's own source (gpon-rtl9602c.c, the
        "carries no -I for drivers/net/gpon" note) rather than fixed.

        ★ IT IS DRIVEN BY WHAT THE SOURCES ACTUALLY INCLUDE, not by a list of
        directories kept here.  A hand-kept list is a list somebody forgets to
        extend the day a third target starts using the shared layer -- and the
        guard would then pass by not looking.
        """
        gpon_dir = self.p(GPON_DIR)
        if not os.path.isdir(gpon_dir):
            self.check(False, "W10.sharedhdrs", GPON_DIR + " is not a directory")
            return
        shared = {f for f in os.listdir(gpon_dir) if f.endswith(".h")}
        self.check(bool(shared), "W10.sharedhdrs",
                   "the shared tree declares no header at all")
        inc = re.compile(r'^\s*ccflags-y\s*\+=.*-I\$\(srctree\)/drivers/net/gpon\s*$',
                         re.M)
        want = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.M)

        for t in TARGETS:
            base = self.p("target/linux/%s/files-%s" % (t, VER))
            if not os.path.isdir(base):
                continue
            need = {}          # directory -> the shared headers it includes
            for dirpath, _d, filenames in os.walk(base, followlinks=False):
                if os.path.abspath(dirpath).startswith(os.path.abspath(gpon_dir)):
                    continue
                for fn in filenames:
                    if not fn.endswith((".c", ".h")):
                        continue
                    p = os.path.join(dirpath, fn)
                    try:
                        with open(p, "r", encoding="utf-8", errors="replace") as f:
                            body = f.read()
                    except OSError:
                        continue
                    hit = {h for h in want.findall(body)
                           if h in shared and
                           not os.path.exists(os.path.join(dirpath, h))}
                    if hit:
                        need.setdefault(dirpath, set()).update(hit)
            for dirpath, headers in sorted(need.items()):
                mk = os.path.join(dirpath, "Makefile")
                try:
                    with open(mk, "r", encoding="utf-8", errors="replace") as f:
                        body = f.read()
                except OSError:
                    body = ""
                self.check(inc.search(body) is not None,
                           "W10.include/" + t,
                           "%s includes shared header(s) %s but %s carries no "
                           "`ccflags-y += -I$(srctree)/drivers/net/gpon` -- the "
                           "build fails late, naming the header and not the "
                           "reason"
                           % (os.path.relpath(dirpath, self.root),
                              ", ".join(sorted(headers)),
                              os.path.relpath(mk, self.root)))

    def w11_upgrade_image_keeps_config(self):
        """`mtd -j` appends the saved configuration only where the image stream carries the jffs2
        end-of-filesystem marker: without pad-rootfs it writes nothing and says nothing."""
        writer = re.compile(r'\bmtd\b[^\n]*-j\b')
        recipe = re.compile(r'^\s*IMAGE/sysupgrade\.bin\s*:=([^\n]*\bsysupgrade-tar\b[^\n]*)$', re.M)
        for t in sorted({t for ts in OVERLAYS.values() for t in ts}):
            body = self.read("target/linux/%s/base-files/lib/upgrade/platform.sh" % t)
            if not writer.search(body or ""):
                continue
            mks = self.p("target/linux/%s/image" % t)
            for mk in sorted(os.listdir(mks)) if os.path.isdir(mks) else ():
                for line in recipe.findall(self.read("target/linux/%s/image/%s" % (t, mk)) or ""):
                    self.check("pad-rootfs" in line.split("sysupgrade-tar")[0], "W11.padrootfs/" + t,
                               "%s builds a sysupgrade tar whose root has no pad-rootfs, and %s's "
                               "platform.sh keeps the configuration with `mtd -j`: nothing is kept"
                               % (mk, t))

    def run(self):
        for fn in (self.w1_files_dir_line, self.w2_trees_disjoint, self.w3_o2,
                   self.w4_objects_accounted, self.w5_overlay_is_the_prepatch_base,
                   self.w6_no_scanned_makefile, self.w7_no_symlinks,
                   self.w8_parent_wiring, self.w9_someone_selects,
                   self.w10_shared_header_include_path,
                   self.w11_upgrade_image_keeps_config):
            fn()
        return self.fails


def find_root(start):
    d = os.path.abspath(start)
    while d != "/":
        if os.path.isdir(os.path.join(d, "target/linux")) and \
           os.path.isfile(os.path.join(d, "include/kernel.mk")):
            return d
        d = os.path.dirname(d)
    sys.exit("could not find the openwrt tree root above " + start)


# --------------------------------------------------------------------------
# --self-check: MUTATE the subject in a scratch copy and require each
# assertion to go RED.  Reading the guard proves nothing about it.
# --------------------------------------------------------------------------
MUTATIONS = [
    ("W1.append", "target/linux/realtek-luna/Makefile",
     lambda s: re.sub(r'^FILES_DIR \+=.*$', '', s, flags=re.M)),
    ("W1.order", "target/linux/realtek-luna/Makefile",
     lambda s: re.sub(r'^(FILES_DIR \+=.*)$', '', s, flags=re.M).replace(
         "include $(INCLUDE_DIR)/target.mk",
         'FILES_DIR += "$(TOPDIR)/target/linux/gpon-common/files-$(KERNEL_PATCHVER)"\n'
         "include $(INCLUDE_DIR)/target.mk", 1)),
    ("W1.append", "target/linux/zte-zx/Makefile",
     lambda s: re.sub(r'^FILES_DIR \+=.*eth-common.*$', '', s, flags=re.M)),
    ("W1.reexpand", "target/linux/zte-zx/Makefile",
     lambda s: s.replace(REEXPAND, "")),
    # the same header in two overlays: the later $(CP) silently wins.
    ("W2.disjoint", SHARED_FILES + "/drivers/net/flowcore/flowcore.h",
     lambda s: s, "target/linux/eth-common/files-%s/drivers/net/flowcore/flowcore.h" % VER),
    ("W3.o2", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^ccflags-y \+= -O2$', '', s, flags=re.M)),
    # the "silently dead source" arm: drop one built object and its .c is one
    # nothing accounts for.
    ("W4.src_never_built", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^obj-\$\(CONFIG_GPON_OLT_ROLE\) \+= gpon_olt_platform\.o$', '', s, flags=re.M)),
    # the "reason is a placeholder" arm.
    ("W4.pending_reason", SHARED_FILES + "/drivers/net/gpon/Makefile",
     lambda s: re.sub(r'^obj-\$\(CONFIG_GPON_OLT_ROLE\) \+= gpon_olt_platform\.o$',
                      '# gpon-pending: gpon_olt_platform.o -- later', s, flags=re.M)),
    ("W5.prepatch", "target/linux/realtek-luna/files-%s/arch/mips/Kconfig" % VER,
     lambda s: s.replace("config BOOT_RAW\n\tbool\n\n", "", 1)),
    ("W8.kconfig", SHARED_FILES + "/drivers/net/Kconfig",
     lambda s: s.replace('source "drivers/net/gpon/Kconfig"', '')),
    ("W8.makefile", SHARED_FILES + "/drivers/net/Makefile",
     lambda s: re.sub(r'^obj-\$\(CONFIG_GPON_CORE\) \+= gpon/$', '', s, flags=re.M)),
    ("W9.select", "target/linux/realtek-elnath/files-%s/drivers/net/ethernet/"
                  "cortina/Kconfig" % VER,
     lambda s: re.sub(r'^\tselect GPON_CORE.*$', '', s, flags=re.M)),
    ("W11.padrootfs", "target/linux/realtek-luna/image/taroko.mk",
     lambda s: s.replace("append-rootfs | pad-rootfs | check-size | sysupgrade-tar rootfs=$$$$@",
                         "sysupgrade-tar", 1)),
    # W10: drop the include path from a directory that DOES include a shared
    # header.  Without the guard this is silent here and fails deep inside a
    # target build, naming the header and not the reason.
    ("W10.include", "target/linux/realtek-luna/files-%s/drivers/net/ethernet/"
                    "realtek/Makefile" % VER,
     lambda s: re.sub(r'^ccflags-y \+= -I\$\(srctree\)/drivers/net/gpon$', '',
                      s, flags=re.M)),
]


def self_check(root):
    """Copy only what the guard reads, mutate one thing, require a FAIL."""
    subtrees = list(OVERLAYS) + ["include", "target/linux/realtek-luna",
                                 "target/linux/realtek-elnath", "target/linux/zte-zx"] + \
               ["target/linux/generic/%s-%s" % (k, VER)
                for k in ("backport", "pending", "hack")]
    rc = 0
    for name, rel, mutate, *copy_to in MUTATIONS:
        tmp = tempfile.mkdtemp(prefix="gpon-wiring-")
        try:
            for s in subtrees:
                src, dst = os.path.join(root, s), os.path.join(tmp, s)
                if os.path.isdir(src):
                    shutil.copytree(src, dst, symlinks=True,
                                    ignore=shutil.ignore_patterns("*.o", "*.ko"))
            p = os.path.join(tmp, rel)
            with open(p, "r", encoding="utf-8") as f:
                before = f.read()
            after = mutate(before)
            if before == after and not copy_to:
                print("  %-14s MUTATION IS A NO-OP -- the guard is untested" % name)
                rc = 1
                continue
            if copy_to:     # a mutation that ADDS a file: @rel's bytes at that path
                p = os.path.join(tmp, copy_to[0])
                os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w", encoding="utf-8") as f:
                f.write(after)
            fails = Guard(tmp).run()
            hit = any(f.split()[0].startswith(name) for f in fails)
            print("  %-14s mutate %-58s -> %s" %
                  (name, os.path.basename(rel), "RED (guard works)" if hit
                   else "STILL GREEN -- VACUOUS GUARD"))
            if not hit:
                rc = 1
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    return rc


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=None, help="openwrt tree root (autodetected)")
    ap.add_argument("--self-check", action="store_true",
                    help="mutate each subject in a scratch copy; every check must go RED")
    ap.add_argument("--LABEL", default=None, help="run label, recorded in the output")
    a = ap.parse_args()

    root = a.root or find_root(os.path.dirname(os.path.abspath(__file__)))
    print("gpon build-wiring guard   root=%s%s" %
          (root, "   LABEL=%s" % a.LABEL if a.LABEL else ""))

    g = Guard(root)
    fails = g.run()
    for f in fails:
        print("FAIL  " + f)
    print("%d check(s), %d failed" % (g.checks, len(fails)))

    rc = 1 if fails else 0
    if a.self_check:
        print("\n--self-check: each mutation MUST turn the gate red")
        rc |= self_check(root)
    return rc


if __name__ == "__main__":
    sys.exit(main())
