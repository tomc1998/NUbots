#!/usr/bin/env python3

from dockerise import run_on_docker
import b
import os
import pty
import subprocess


@run_on_docker
def register(command):
    # Install help
    command.help = "Open an interactive shell in a docker container"

    command.add_argument(
        "-i",
        "--interactive",
        dest="interactive",
        action="store_true",
        default=False,
        help="perform an interactive configuration using ccmake",
    )
    command.add_argument("args", nargs="...", help="the arguments to pass through to cmake")


@run_on_docker
def run(interactive, args, **kwargs):

    os.chdir(os.path.join(b.project_dir, "..", "build"))

    # If configure then run ccmake else just run cmake
    if interactive:
        exit(
            pty.spawn(["ccmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir])
            << 8
        )
    else:
        exit(
            subprocess.call(
                ["cmake", "-GNinja", "-DCMAKE_TOOLCHAIN_FILE=/usr/local/toolchain.cmake", *args, b.project_dir]
            )
        )
