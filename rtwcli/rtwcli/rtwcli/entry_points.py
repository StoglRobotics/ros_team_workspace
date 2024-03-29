# Copyright 2016-2017 Dirk Thomas
# Copyright 2017 Open Source Robotics Foundation, Inc.
# Copyright 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from collections import defaultdict
import logging

try:
    import importlib.metadata as importlib_metadata
except ModuleNotFoundError:
    import importlib_metadata


# The group name for entry points identifying extension points.
# While all entry points in this package start with ``rtwcli.`` other
# distributions might define entry points with a different prefix.
# Those need to be declared using this group name.
EXTENSION_POINT_GROUP_NAME = "rtwcli.extension_point"

logger = logging.getLogger(__name__)


def get_all_entry_points():
    """
    Get all entry points related to ``rtwcli`` and any of its extensions.

    :returns: mapping of entry point names to ``EntryPoint`` instances
    :rtype: dict
    """
    extension_points = get_entry_points(EXTENSION_POINT_GROUP_NAME)

    entry_points = defaultdict(dict)

    for dist in importlib_metadata.distributions():
        for ep in dist.entry_points:
            # skip groups which are not registered as extension points
            if ep.group not in extension_points:
                continue

            entry_points[ep.group][ep.name] = (dist, ep)
    return entry_points


def get_entry_points(group_name):
    """
    Get the entry points for a specific group.

    :param str group_name: the name of the ``entry_point`` group
    :returns: mapping of group name to dictionaries which map entry point names
      to ``EntryPoint`` instances
    :rtype: dict
    """
    entry_points_impl = importlib_metadata.entry_points()
    if hasattr(entry_points_impl, "select"):
        groups = entry_points_impl.select(group=group_name)
    else:
        groups = entry_points_impl.get(group_name, [])
    entry_points = {}
    for entry_point in groups:
        entry_points[entry_point.name] = entry_point
    return entry_points


def load_entry_points(group_name, *, exclude_names=None):
    """
    Load the entry points for a specific group.

    :param str group_name: the name of the ``entry_point`` group
    :param iterable exclude_names: the names of the entry points to exclude
    :returns: mapping of entry point name to loaded entry point
    :rtype: dict
    """
    extension_types = {}
    for entry_point in get_entry_points(group_name).values():
        if exclude_names and entry_point.name in exclude_names:
            continue
        try:
            extension_type = entry_point.load()
        except Exception as e:  # noqa: F841
            logger.warning(f"Failed to load entry point '{entry_point.name}': {e}")
            continue
        extension_types[entry_point.name] = extension_type
    return extension_types


def get_first_line_doc(any_type):
    if not any_type.__doc__:
        return ""
    lines = any_type.__doc__.splitlines()
    if not lines:
        return ""
    if lines[0]:
        line = lines[0]
    elif len(lines) > 1:
        line = lines[1]
    return line.strip().rstrip(".")
