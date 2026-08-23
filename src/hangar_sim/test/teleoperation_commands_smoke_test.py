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

# teleop_mode values whose branches consume a stored target, from
# moveit_studio_sdk_msgs/TeleoperationMode. The jog modes (1, 2) consume none.
CONSUMING_MODES = {"3", "4", "5"}
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

    # THEN one retrieval per consuming slot is present. Asserted before the
    # loop: RETRIEVALS is a hand-kept copy of two core class names, and a
    # rename upstream would otherwise shrink the list and quietly turn the
    # slot-agreement check below into a no-op.
    assert len(retrievals) == len(SLOTS)

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


def test_stale_targets_are_discarded_before_the_modes_run(tree) -> None:
    """Every slot is discarded at start of run, and the consumers wait on it.

    A run that inherits a target stored during an earlier run would execute a
    motion the operator asked for at some earlier moment.
    """
    # GIVEN the discards at the top of the Objective
    discarded = Counter(
        n.get(COMMAND_TYPE) for n in tree.iter() if n.get("ID") == DISCARD_ID
    )

    # THEN each consuming slot is discarded exactly once
    assert discarded == Counter(SLOTS)

    # AND the discards run before the modes do, not somewhere inside them
    top = list(tree.find("Control"))
    discard_at = [
        i for i, n in enumerate(top) for _ in n.iter() if _.get("ID") == DISCARD_ID
    ]
    parallel_at = [i for i, n in enumerate(top) if n.get("ID") == "Parallel"]
    assert discard_at and parallel_at
    assert max(discard_at) < min(parallel_at), "discards do not precede the modes"

    # AND every branch that consumes a stored target is gated on that discard
    consuming = _gates_for(tree, CONSUMING_MODES)
    assert len(consuming) == len(CONSUMING_MODES)
    for gate in consuming:
        assert "stored_targets_clear" in gate, f"ungated consuming branch: {gate!r}"


def test_jogging_is_not_gated_on_the_discard(tree) -> None:
    """Jogging must stay reachable when the discard fails.

    Jogging consumes no stored target, and it is how an operator walks the arm
    out of a bad configuration -- so it must not depend on slot cleanup.
    """
    # GIVEN the gates on the jog branches (teleop_mode 1 and 2)
    jog = _gates_for(tree, JOG_MODES)

    # THEN both jog branches exist and neither waits on the discard
    assert len(jog) == len(JOG_MODES)
    for gate in jog:
        assert "stored_targets_clear" not in gate, f"jog branch gated: {gate!r}"


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
