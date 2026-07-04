#!/usr/bin/env python3

# The MIT License (MIT)

# Copyright (c) 2017 Lancaster University.

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import os
import sys
import optparse
import platform
import json
import shutil
import re
from utils.python.codal_utils import system, build, read_json, checkgit, read_config, update, revision, printstatus, status, get_next_version, lock, delete_build_folder, generate_docs

parser = optparse.OptionParser(usage="usage: %prog target-name-or-url [options]", description="This script manages the build system for a codal device. Passing a target-name generates a codal.json for that devices, to list all devices available specify the target-name as 'ls'.")
parser.add_option('-c', '--clean', dest='clean', action="store_true", help='Whether to clean before building. Applicable only to unix based builds.', default=False)
parser.add_option('-t', '--test-platforms', dest='test_platform', action="store_true", help='Specify whether the target platform is a test platform or not.', default=False)
parser.add_option('-l', '--lock', dest='lock_target', action="store_true", help='Create target-lock.json, updating patch version', default=False)
parser.add_option('-b', '--branch', dest='branch', action="store_true", help='With -l, use vX.X.X-BRANCH.Y', default=False)
parser.add_option('-m', '--minor', dest='update_minor', action="store_true", help='With -l, update minor version', default=False)
parser.add_option('-M', '--major', dest='update_major', action="store_true", help='With -l, update major version', default=False)
parser.add_option('-V', '--version', dest='version', metavar="VERSION", help='With -l, set the version; use "-V v0.0.1" to bootstrap', default=False)
parser.add_option('-v', '--verbose', dest='verbose', action="store_true", help='Increases verbosity)', default=False)
parser.add_option('-u', '--update', dest='update', action="store_true", help='git pull target and libraries, use with "-d/--dev" to update all libraries to their latest master' , default=False)
parser.add_option('-s', '--status', dest='status', action="store_true", help='git status target and libraries', default=False)
parser.add_option('-r', '--revision', dest='revision', action="store", help='Checkout a specific revision of the target', default=False)
parser.add_option('-d', '--dev', dest='dev', action="store_true", help='enable developer mode (does not use target-locked.json)', default=False)
parser.add_option('-g', '--generate-docs', dest='generate_docs', action="store_true", help='generate documentation for the current target', default=False)
parser.add_option('-j', '--parallelism', dest='parallelism', action="store", help='Set the number of parallel threads to build with, if supported', default=10)
parser.add_option('-n', '--lines', dest='detail_lines', action="store", help="Sets the number of detail lines to output (only relevant to --status)", default=3 )
parser.add_option('--fw-only', dest='fw_only', action="store_true", help='Build ONLY the micro:bit firmware; skip the host-simulation library. By default build.py builds BOTH the bench firmware (MICROBIT.hex) and the full-simulation library (tests/_infra/sim/build/libfirmware_host).', default=False)

(options, args) = parser.parse_args()

if not os.path.exists("build"):
    os.mkdir("build")

if options.lock_target:
    lock(options)
    exit(0)

if options.update:
    update(sync_dev = options.dev)
    exit(0)

if options.status:
    status(logLines=options.detail_lines, detail=options.verbose, libs=args)
    exit(0)

if options.revision:
    revision(options.revision)
    exit(0)

# Regenerate DefaultConfig.cpp from the active robot JSON config so
# calibration values are baked into the firmware at compile time.
#
# 077-001: skipped while source/robot/ does not exist. gen_default_config.py
# writes to source/robot/DefaultConfig.cpp -- a directory the greenfield
# rebuild's new source/ tree deliberately does not create until a later
# sprint adds a Robot/ConfigRegistry back. The check is structural (does
# source/robot/ exist?), not a version flag, so it self-heals the moment that
# directory reappears (architecture-update.md Design Rationale Decision 4).
# check_config_sync.py is a separate CI lint (.github/workflows/build.yml),
# not something build.py itself calls -- nothing to condition for it here.
import subprocess as _sp
_source_robot_dir = os.path.join(os.path.dirname(__file__), "source", "robot")
if os.path.isdir(_source_robot_dir):
    _gen = os.path.join(os.path.dirname(__file__), "scripts", "gen_default_config.py")
    _sp.run([sys.executable, _gen], check=True)
else:
    print("build.py: source/robot/ absent -- skipping gen_default_config.py (077-001)")

# Regenerate source/messages/*.h from protos/*.proto (C++11 POD headers).
_gen_msgs = os.path.join(os.path.dirname(__file__), "scripts", "gen_messages.py")
_sp.run([sys.executable, _gen_msgs], check=True)

# out of source build!
os.chdir("build")

test_json = read_json("../utils/targets.json")

# configure the target a user has specified:
if len(args) == 1:

    target_name = args[0]
    target_config = None

    # list all targets
    if target_name == "ls":
        for json_obj in test_json:
            s = "%s: %s" % (json_obj["name"], json_obj["info"])
            if "device_url" in json_obj.keys():
                s += "(%s)" % json_obj["device_url"]
            print(s)
        exit(0)

    # cycle through out targets and check for a match
    for json_obj in test_json:
        if json_obj["name"] != target_name:
            continue

        del json_obj["device_url"]
        del json_obj["info"]

        target_config = json_obj
        break

    if target_config == None and target_name.startswith("http"):
        target_config = {
            "name": re.sub("^.*/", "", target_name),
            "url": target_name,
            "branch": "master",
            "type": "git"
        }

    if target_config == None:
        print("'" + target_name + "'" + " is not a valid target.")
        exit(1)

    # developer mode is for users who wish to contribute, it will clone and checkout commitable branches.
    if options.dev:
        target_config["dev"] = True

    config = {
        "target":target_config
    }

    with open("../codal.json", 'w') as codal_json:
        json.dump(config, codal_json, indent=4)

    # remove the build folder, a user could be swapping targets.
    delete_build_folder()


elif len(args) > 1:
    print("Too many arguments supplied, only one target can be specified.")
    exit(1)

def _project_version():
    """Read the canonical project version from the root pyproject.toml."""
    try:
        root = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(root, "pyproject.toml")) as f:
            for line in f:
                m = re.match(r'\s*version\s*=\s*"([^"]+)"', line)
                if m:
                    return m.group(1)
    except Exception:
        pass
    return "?"


def build_host_sim(clean):
    """Build the host-simulation library (libfirmware_host) via the tests/_infra/sim
    CMake build (HOST_BUILD) — the 'full simulation' target the pytest harness
    uses. Fast: ~8s clean, <1s incremental. Uses absolute paths from __file__ so
    it is correct regardless of the current working directory. Raises on failure.
    """
    import subprocess
    root = os.path.dirname(os.path.abspath(__file__))
    sim_dir = os.path.join(root, "tests", "_infra", "sim")
    build_dir = os.path.join(sim_dir, "build")
    if clean and os.path.isdir(build_dir):
        shutil.rmtree(build_dir)
    os.makedirs(build_dir, exist_ok=True)
    print("\nBuilding host-simulation library (libfirmware_host, HOST_BUILD)...")
    # ROBOT_RUN_MODE=SIM (039-005): the host build uses the io/sim/ device impls
    # (MockHAL etc.). The sim CMakeLists defaults to SIM, but pass it explicitly.
    subprocess.run(
        ["cmake", "-S", sim_dir, "-B", build_dir,
         "-DROBOT_RUN_MODE=SIM"], check=True
    )
    subprocess.run(["cmake", "--build", build_dir, "--parallel"], check=True)


def _host_sim_dir():
    """Absolute path to the host-sim CMake project, wherever tests/ currently lives."""
    root = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(root, "tests", "_infra", "sim")


def print_build_summary(fw_only):
    """Print a one-glance summary so there is no guessing which versions exist."""
    ver = _project_version()
    print("\n=== build summary ===")
    print("  firmware hex   v%s   (bench, BENCH_OTOS_ENABLED)   -> MICROBIT.hex" % ver)
    if fw_only:
        print("  host sim lib   (skipped: --fw-only)")
    elif not os.path.isdir(_host_sim_dir()):
        print("  host sim lib   (skipped: tests/_infra/sim/ absent -- parked in tests_old/, 077-001)")
    else:
        print("  host sim lib   v%s   (HOST_BUILD)   -> tests/_infra/sim/build/libfirmware_host" % ver)
    print()


if not options.test_platform:

    if not os.path.exists("../codal.json"):
        print("No target specified in codal.json, does codal.json exist?")
        exit(1)

    if options.generate_docs:
        generate_docs()
        exit(0)

    build(options.clean, verbose=options.verbose, parallelism=options.parallelism)

    # Dev build = BOTH versions. After the bench firmware (MICROBIT.hex), also
    # build the full-simulation library (libfirmware_host, HOST_BUILD) so a single
    # build always leaves both artifacts in sync — no guessing which you have.
    # --fw-only skips this. The host-sim build is fast (~8s clean, <1s incremental).
    #
    # 077-001: also skipped (structurally) while tests/_infra/sim/ does not
    # exist -- it is parked under tests_old/ by the greenfield rebuild's test
    # rename, and a fresh sim harness under tests/sim/ is later-ticket work
    # (architecture-update.md: "Host-side sim/test builds reference old paths
    # — expected broken; do not chase them this ticket"). Self-heals once
    # tests/_infra/sim/ (or its replacement) reappears.
    if not options.fw_only:
        if os.path.isdir(_host_sim_dir()):
            build_host_sim(options.clean)
        else:
            print("\nbuild.py: tests/_infra/sim/ absent -- skipping host-sim build (077-001)")

    print_build_summary(options.fw_only)
    exit(0)

for json_obj in test_json:

    # some platforms aren't supported by travis, ignore them when testing.
    if "test_ignore" in json_obj:
        print("ignoring: " + json_obj["name"])
        continue

    # ensure we have a clean build tree.
    delete_build_folder()

    # clean libs
    if os.path.exists("../libraries"):
        shutil.rmtree('../libraries')

    # configure the target and tests...
    config = {
        "target":json_obj,
        "output":".",
        "application":"libraries/"+json_obj["name"]+"/tests/"
    }

    with open("../codal.json", 'w') as codal_json:
        json.dump(config, codal_json, indent=4)

    build(True, True, options.parallelism)
