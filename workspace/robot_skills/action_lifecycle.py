"""Shared action ownership and lifecycle pause; no stand-alone trial entry."""
from explore import Explorer, ManageLifecycleNodes


class ActionLifecycle(Explorer):
    def execute(self, client, goal, timeout, kind):
        # Record the send future so a late acceptance is canceled after timeout.
        original = client.send_goal_async
        def send(message):
            self.dispatched = True
            self.pending_goal = original(message)
            return self.pending_goal
        client.send_goal_async = send
        try:
            return super().execute(client, goal, timeout, kind)
        finally:
            client.send_goal_async = original


    def pause_navigation(self):
        if not self.manager.wait_for_service(timeout_sec=2):
            return False
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        try:
            return bool(self.wait(self.manager.call_async(request), 8).success)
        except Exception:
            return False
