# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the PickNik Inc. nor the names of its contributors
#      may be used to endorse or promote products derived from this software
#      without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DAMAGES ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE.

"""Shared pytest fixtures for the lab_sim_collision_monitor integration test.

Re-exporting the MoveIt Pro backend fixtures here (rather than importing them
into the test module) lets the test reference them purely via pytest's fixture
injection — no module-level import of the fixture names, so no flake8
redefinition (F811) noise from using them as test arguments.
"""

from moveit_pro_test_utils.objective_test_fixture import (  # noqa: F401
    execute_objective_resource,
    reset_simulation_before_test,
)
