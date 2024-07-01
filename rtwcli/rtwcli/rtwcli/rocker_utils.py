# Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from typing import List, Union

from rtwcli.constants import DISPLAY_MANAGER_WAYLAND
from rtwcli.utils import get_display_manager, run_command


def generate_rocker_flags(
    disable_nvidia: bool,
    container_name: str,
    hostname: str,
    ssh_abs_path: str,
    ssh_abs_path_in_docker: str,
    final_image_name: str,
    ws_volumes: Union[List[str], None] = None,
    user_override_name: Union[str, None] = None,
) -> List[str]:
    # rocker flags have order, see rocker --help
    rocker_flags = ["--nocache", "--nocleanup", "--git"]

    rocker_flags.extend(["-e", "QT_QPA_PLATFORM=xcb"])
    if not disable_nvidia:
        rocker_flags.extend(["-e", "__GLX_VENDOR_LIBRARY_NAME=nvidia"])
        rocker_flags.extend(["-e", "__NV_PRIME_RENDER_OFFLOAD=1"])
    if get_display_manager() == DISPLAY_MANAGER_WAYLAND:
        waylad_display = os.environ.get("WAYLAND_DISPLAY", None)
        if not waylad_display:
            raise RuntimeError("WAYLAND_DISPLAY is not set.")
        rocker_flags.extend(["-e", "XDG_RUNTIME_DIR=/tmp"])
        rocker_flags.extend(["-e", f"WAYLAND_DISPLAY={waylad_display}"])
        rocker_flags.extend(
            [
                "--volume",
                f"{os.environ['XDG_RUNTIME_DIR']}/{waylad_display}:/tmp/{waylad_display}",
            ]
        )

    rocker_flags.extend(["--hostname", hostname])
    rocker_flags.extend(["--name", container_name])
    rocker_flags.extend(["--network", "host"])

    if not disable_nvidia:
        rocker_flags.extend(["--nvidia", "gpus"])

    rocker_flags.extend(["--user", "--user-preserve-home"])
    if user_override_name:
        rocker_flags.extend(["--user-override-name", user_override_name])

    # rocker volumes
    rocker_flags.append("--volume")
    rocker_flags.append(f"{ssh_abs_path}:{ssh_abs_path_in_docker}:ro")
    if ws_volumes:
        rocker_flags.extend(ws_volumes)

    rocker_flags.append("--x11tmp")
    rocker_flags.extend(["--mode", "interactive"])
    rocker_flags.extend(["--image-name", f"{final_image_name}"])

    return rocker_flags


def execute_rocker_cmd(rocker_flags: List[str], rocker_base_image_name: str) -> bool:
    rocker_cmd = ["rocker"] + rocker_flags + [rocker_base_image_name]
    rocker_cmd_str = " ".join(rocker_cmd)
    print(f"Executing rocker command '{rocker_cmd_str}'")
    return run_command(rocker_cmd)
