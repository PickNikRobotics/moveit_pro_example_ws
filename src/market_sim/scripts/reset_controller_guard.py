#!/usr/bin/env python3

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

"""Keep one reset's controller restore set across failed Objective instances.

Prepare holds the command bridge, snapshots active command controllers once, and verifies their deactivation. Restore runs only after localization synchronization; it verifies the original set before clearing the snapshot and hold. Empty switches are skipped. Restarting this node loses the snapshot and is not a recovery procedure.
"""

import math
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def active_commands(controllers):
    """Names of active controllers that actually claim command interfaces."""
    return {
        controller.name
        for controller in controllers
        if controller.state == "active" and controller.claimed_interfaces
    }


class ResetControllerGuard(Node):
    """Serialize bounded controller preparation/restoration without erasing failed state."""

    def __init__(self, **kwargs):
        super().__init__("reset_controller_guard", **kwargs)
        self._timeout = float(self.declare_parameter("service_timeout_sec", 3.0).value)
        self._max_age = float(self.declare_parameter("sample_max_age_sec", 0.5).value)
        if not all(
            math.isfinite(value) and value > 0
            for value in (self._timeout, self._max_age)
        ):
            raise ValueError(
                "Reset controller service_timeout_sec and sample_max_age_sec must be finite and positive."
            )
        self._restore = None
        self._busy = False
        self._pending = None
        self._deadline = 0.0
        self._hold = False
        self._saw_localization_hold = False
        self._synchronized = False
        self._ready_at = 0.0
        self.create_subscription(
            Bool, "/market_sim/localization_ready", self._on_ready, 10
        )
        self._hold_pub = self.create_publisher(
            Bool,
            "/market_sim/reset_controller_hold",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._list = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        group = ReentrantCallbackGroup()
        self.create_service(
            Trigger,
            "/market_sim/prepare_reset_controllers",
            self._prepare,
            callback_group=group,
        )
        self.create_service(
            Trigger,
            "/market_sim/restore_reset_controllers",
            self._restore_controllers,
            callback_group=group,
        )
        self.create_timer(0.05, self._tick)

    def _on_ready(self, msg):
        self._ready_at = time.monotonic()
        if self._restore is not None:
            if not msg.data:
                self._saw_localization_hold = True
                self._synchronized = False
            elif self._saw_localization_hold:
                self._synchronized = True

    def _tick(self):
        self._hold_pub.publish(Bool(data=self._hold))
        if self._pending is not None and time.monotonic() >= self._deadline:
            request, waiter = self._pending
            self._pending = None
            request.cancel()
            waiter.set_result(None)

    async def _call(self, client, request):
        if not client.service_is_ready():
            raise RuntimeError(f"{client.srv_name} is unavailable")
        result = Future()
        future = client.call_async(request)
        self._pending = (future, result)
        self._deadline = time.monotonic() + self._timeout

        def completed(done):
            if self._pending is not None and self._pending[0] is done:
                self._pending = None
                result.set_result(done.result())

        future.add_done_callback(completed)
        response = await result
        if response is None:
            raise RuntimeError(f"{client.srv_name} timed out after {self._timeout:g} s")
        return response

    async def _switch_controllers(self, activate, deactivate):
        if not activate and not deactivate:
            return
        request = SwitchController.Request()
        request.activate_controllers = sorted(activate)
        request.deactivate_controllers = sorted(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = int(self._timeout)
        request.timeout.nanosec = int((self._timeout % 1) * 1e9)
        response = await self._call(self._switch, request)
        if not response.ok:
            raise RuntimeError(
                "/controller_manager/switch_controller rejected the reset controller switch"
            )

    async def _prepare(self, _request, response):
        return await self._run(response, restore=False)

    async def _restore_controllers(self, _request, response):
        return await self._run(response, restore=True)

    async def _run(self, response, *, restore):
        operation = "restore" if restore else "prepare"
        if self._busy:
            response.message = (
                "A reset controller operation is pending; wait for its result."
            )
            return response
        self._busy = True
        self._hold = True
        self._hold_pub.publish(Bool(data=True))
        try:
            listed = await self._call(self._list, ListControllers.Request())
            current = active_commands(listed.controller)
            if restore:
                if self._restore is None:
                    raise RuntimeError(
                        "no saved reset controller set; run Reset MuJoCo Sim from its start"
                    )
                if (
                    not self._synchronized
                    or time.monotonic() - self._ready_at > self._max_age
                ):
                    raise RuntimeError(
                        "localization has not completed a fresh reset synchronization"
                    )
                if not self._restore <= {
                    controller.name for controller in listed.controller
                }:
                    raise RuntimeError("a saved reset controller is no longer loaded")
                await self._switch_controllers(
                    self._restore - current, current - self._restore
                )
                expected = self._restore
            else:
                self._synchronized = False
                self._saw_localization_hold = False
                if self._restore is None:
                    self._restore = current
                await self._switch_controllers(set(), current)
                expected = set()
            verified = await self._call(self._list, ListControllers.Request())
            if active_commands(verified.controller) != expected:
                raise RuntimeError(
                    "controller states did not match the requested reset switch"
                )
            if restore:
                self._restore = None
                self._hold = False
            response.success = True
            response.message = f"Reset controllers {operation} complete."
        except RuntimeError as error:
            response.message = f"Reset controller {operation} failed: {error}. Commands remain held; check controller_manager and retry Reset MuJoCo Sim."
            if restore:
                try:
                    listed = await self._call(self._list, ListControllers.Request())
                    await self._switch_controllers(
                        set(), active_commands(listed.controller)
                    )
                    stopped = await self._call(self._list, ListControllers.Request())
                    if active_commands(stopped.controller):
                        raise RuntimeError("command controllers remain active")
                except RuntimeError as stop_error:
                    response.message += (
                        f" Controller stop could not be verified: {stop_error}."
                    )
            self.get_logger().error(response.message)
        finally:
            self._busy = False
            self._hold_pub.publish(Bool(data=self._hold))
        return response


def main():
    rclpy.init()
    node = ResetControllerGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
