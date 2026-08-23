# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Smoke test for hangar_sim's copied ``Request Teleoperation`` Objective.

Covers the three teleoperation modes that consume a stored target -- joint
slider, waypoint, and interactive marker -- against the command protocol the
retrieval Behaviors speak.

The checks are static, over the Objective XML. That is deliberate: the
regression this guards is a *wiring* failure, not a motion failure. Calling
``RetrieveRobotStateParameter`` without ``command_type`` aborts the command
before any motion is planned, with

    Failed to create service request: Input data port `command_type` is
    required: set it to `waypoint`, `interactive_marker` or `joint_slider`.

so a test that asserts on the wiring fails for the same reason a live run
would, in milliseconds and with no dependency on a running MoveIt Pro stack.
(The directory's session-scoped ``capture_rosout`` fixture still starts a
``/rosout`` subscriber subprocess for every test here; nothing below depends
on it.)

Driving the modes through the live UI is not available here. ``Teleoperate``
is on ``objectives_integration_test.py``'s skip list because
``DoTeleoperateAction`` rejects its goal when no primary UI is subscribed, so
a headless test cannot open the panel that stores these targets. Exercising
the modes end-to-end stays a manual pre-merge step (see the PR description).
"""

import re
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree

import pytest

OBJECTIVE = (
    Path(__file__).resolve().parent.parent / "objectives" / "request_teleoperation.xml"
)

SCOPE_ID = "TeleoperationCommandScope"
DISCARD_ID = "DiscardTeleoperationCommand"
COMMAND_TYPE = "command_type"

# Retrieval Behavior -> the command_type its branch must declare. Each slot is
# named by the interaction that fills it, so a mismatch here means a target
# stored by one interaction would drive another interaction's motion profile.
RETRIEVALS = {
    "RetrieveRobotStateParameter",
    "RetrievePoseParameter",
}

# The slot each consuming interaction fills.
SLOTS = {"joint_slider", "waypoint", "interactive_marker"}

# Which teleop_mode each consuming slot belongs to, from
# moveit_studio_sdk_msgs/TeleoperationMode. Swapping two of these would route an
# operator's command into another interaction's motion profile and still look
# correct from outside, which is the substitution the command protocol exists to
# prevent -- so the binding is pinned, not just the shape of the gates.
MODE_FOR_SLOT = {
    "waypoint": "3",  # MOVE_TO_WAYPOINT
    "interactive_marker": "4",  # INTERACTIVE_MARKER
    "joint_slider": "5",  # JOINT_SLIDER
}
CONSUMING_MODES = set(MODE_FOR_SLOT.values())
JOG_MODES = {"1", "2"}


def _gates_for(tree: ElementTree.Element, modes: set[str]) -> list[str]:
    """Every ``_while`` gate testing ``teleop_mode`` against one of ``modes``.

    Anchored on a word boundary so a future ``teleop_mode == 12`` cannot be
    picked up by the search for mode 1.
    """
    pattern = re.compile(rf"teleop_mode\s*==\s*({'|'.join(sorted(modes))})\b")
    return [
        gate for n in tree.iter() if (gate := n.get("_while")) and pattern.search(gate)
    ]


@pytest.fixture(scope="module")
def tree() -> ElementTree.Element:
    return ElementTree.parse(OBJECTIVE).getroot().find("BehaviorTree")


def _parents(
    root: ElementTree.Element,
) -> dict[ElementTree.Element, ElementTree.Element]:
    return {child: parent for parent in root.iter() for child in parent}


def _enclosing_scope(
    node: ElementTree.Element, parents: dict
) -> ElementTree.Element | None:
    """Nearest enclosing TeleoperationCommandScope, or None if unscoped."""
    current = parents.get(node)
    while current is not None:
        if current.get("ID") == SCOPE_ID:
            return current
        current = parents.get(current)
    return None


def test_every_retrieval_declares_a_command_type(tree) -> None:
    """Each retrieval names the slot it reads.

    This is the exact failure a copied Objective hits after the command
    protocol lands: the port is required, so an unset one aborts the command.
    """
    # GIVEN every retrieval Behavior in the Objective
    retrievals = [n for n in tree.iter() if n.get("ID") in RETRIEVALS]

    # THEN the Objective uses the retrieval Behaviors at all
    assert retrievals, f"no retrieval Behaviors found in {OBJECTIVE.name}"

    # AND every one of them declares a non-empty command_type
    missing = [n.get("ID") for n in retrievals if not n.get(COMMAND_TYPE)]
    assert not missing, f"retrieval Behaviors without command_type: {missing}"


def test_each_retrieval_is_scoped_to_its_own_command_type(tree) -> None:
    """Each retrieval sits in a scope naming the same slot.

    Without the scope nothing settles the command, and a command halted
    mid-motion holds its slot for the life of the Runtime. A scope naming a
    *different* slot is worse than none: it would settle an unrelated command.
    """
    # GIVEN every retrieval Behavior and its enclosing scope
    parents = _parents(tree)
    retrievals = [n for n in tree.iter() if n.get("ID") in RETRIEVALS]

    # THEN there is exactly one retrieval per consuming slot. Asserted before the
    # loop, and by slot rather than by count: RETRIEVALS is a hand-kept copy of two
    # core class names, so a rename upstream would shrink the list and quietly turn
    # the agreement check below into a no-op -- and counting alone would accept two
    # retrievals on one slot with a third slot unread.
    assert Counter(n.get(COMMAND_TYPE) for n in retrievals) == Counter(SLOTS)

    for retrieval in retrievals:
        scope = _enclosing_scope(retrieval, parents)

        # THEN it is wrapped in a TeleoperationCommandScope
        assert scope is not None, (
            f"{retrieval.get('ID')} with {COMMAND_TYPE}="
            f"{retrieval.get(COMMAND_TYPE)!r} is not inside a {SCOPE_ID}"
        )

        # AND the scope names the same slot the retrieval reads
        assert scope.get(COMMAND_TYPE) == retrieval.get(COMMAND_TYPE), (
            f"{retrieval.get('ID')} reads slot "
            f"{retrieval.get(COMMAND_TYPE)!r} but its scope settles "
            f"{scope.get(COMMAND_TYPE)!r}"
        )


def test_all_three_consuming_slots_are_covered(tree) -> None:
    """Joint slider, waypoint and interactive marker each get one scope.

    Counted rather than set-compared: two scopes on the same slot would settle
    one command twice and leave another mode unscoped, which a set would hide.
    """
    # GIVEN the command_type of every scope in the Objective
    scoped = Counter(
        n.get(COMMAND_TYPE) for n in tree.iter() if n.get("ID") == SCOPE_ID
    )

    # THEN all three consuming modes are represented, exactly once each
    assert scoped == Counter(SLOTS)


def test_no_mode_cancels_a_command_it_does_not_own(tree) -> None:
    """Start of run ends nothing that another producer may have stored.

    Opening an interaction marks the slot so a target left by an earlier one
    is passed over; it used to be discarded, which cancelled by position and
    could end a command the Desktop App had stored a moment before.
    """
    # GIVEN the Objective as it runs
    # THEN it never discards a slot it has not consumed from
    assert not [n for n in tree.iter() if n.get("ID") == DISCARD_ID]


def test_each_slot_is_reached_only_from_its_own_teleoperation_mode(tree) -> None:
    """Mode 5 drives the joint slider, 4 the marker, 3 the waypoint.

    Exchanging two branches leaves every count and every gate syntactically
    intact, so nothing else here would notice -- but a slider target would run
    through the waypoint planner, or a waypoint through straight interpolation
    with no planner between the operator and the wheels.
    """
    # GIVEN each scope and the mode gate it sits under
    parents = _parents(tree)
    for scope in [n for n in tree.iter() if n.get("ID") == SCOPE_ID]:
        slot = scope.get(COMMAND_TYPE)
        gate = None
        current = parents.get(scope)
        while current is not None and gate is None:
            gate = current.get("_while")
            current = parents.get(current)

        # THEN the branch guarding it tests this slot's own mode
        assert gate is not None, f"{slot!r} scope sits under no mode gate"
        assert re.fullmatch(
            rf"teleop_mode\s*==\s*{MODE_FOR_SLOT[slot]}", gate.strip()
        ), f"{slot!r} is reached from {gate!r}, expected mode {MODE_FOR_SLOT[slot]}"


def test_the_consuming_branches_gate_on_the_mode_alone(tree) -> None:
    """Each consuming branch runs on its mode, with no extra precondition.

    Gating them on a start-of-run cleanup made all three modes dead for the
    rest of the session whenever that cleanup failed once.
    """
    # GIVEN the gates on the three consuming branches
    consuming = _gates_for(tree, CONSUMING_MODES)

    # THEN there is one per mode and each tests only the mode
    assert len(consuming) == len(CONSUMING_MODES)
    for gate in consuming:
        assert re.fullmatch(r"teleop_mode\s*==\s*\d+", gate.strip()), gate


def test_jogging_runs_on_its_mode_alone(tree) -> None:
    """Jogging must stay reachable whatever the state of the command slots.

    It stores nothing, and it is a dead-man velocity stream bounded by the
    controllers' command timeout -- on this robot driving the base as well as
    the arm, so releasing the control stops it.
    """
    # GIVEN the gates on the jog branches (teleop_mode 1 and 2)
    jog = _gates_for(tree, JOG_MODES)

    # THEN both exist and neither carries an extra precondition
    assert len(jog) == len(JOG_MODES)
    for gate in jog:
        assert re.fullmatch(r"teleop_mode\s*==\s*\d+", gate.strip()), gate


def test_completion_publishes_to_the_current_ui_namespace(tree) -> None:
    """The completion topic uses the namespace the app subscribes to.

    ``/studio_ui`` was renamed to ``/moveit_pro_ui`` in 9.0; publishing to the
    old name is silently dropped rather than reported.
    """
    # GIVEN every topic the Objective publishes to
    topics = [n.get("topic") for n in tree.iter() if n.get("topic")]

    # THEN the completion signal is published
    assert any(t == "/moveit_pro_ui/motion_ended" for t in topics)

    # AND nothing still targets the retired namespace
    assert not [t for t in topics if t.startswith("/studio_ui")]


def test_pose_branch_gates_execution_on_an_available_operator(tree) -> None:
    """The interactive-marker branch checks the operator is still there.

    The shared approval Subtree drops its gate when nobody is connected, which
    is right for an unattended Objective and wrong here. Spelled skip-on-
    ``'false'`` because the port reaches the scripting language as text: a
    non-canonical value matches neither literal, and this spelling then runs
    the check rather than silently dropping it.
    """
    # GIVEN the interactive-marker scope
    scope = next(
        (
            n
            for n in tree.iter()
            if n.get("ID") == SCOPE_ID and n.get(COMMAND_TYPE) == "interactive_marker"
        ),
        None,
    )
    assert scope is not None, f"no interactive_marker {SCOPE_ID}"

    # WHEN looking for the approval gate inside it
    gate = next((n for n in scope.iter() if n.get("ID") == "IsUserAvailable"), None)

    # THEN the gate is present and fails closed on a non-canonical port value.
    # Whitespace-normalised: the attribute is prettier-formatted, so a reflow
    # must not fail this test for a formatting reason.
    assert gate is not None, "interactive-marker branch has no IsUserAvailable gate"
    assert " ".join(gate.get("_skipIf", "").split()) == (
        "require_user_approval == 'false'"
    )

    # AND the port it reads is declared, so the script cannot throw on a
    # missing blackboard entry when a caller does not remap it
    model = ElementTree.parse(OBJECTIVE).getroot().find("TreeNodesModel")
    ports = {
        port.get("name"): port.get("default")
        for subtree in model
        for port in subtree
        if port.tag.endswith("_port")
    }
    assert ports.get("require_user_approval") == "true"
