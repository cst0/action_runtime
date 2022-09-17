#!/usr/bin/env python3

# ros deps
import rospy
from action_runtime.msg import Action, Condition
from action_runtime.srv import (
    ActionTrigger,
    ActionTriggerRequest,
    ActionTriggerResponse,
)
from action_runtime.srv import (
    CreatePlan,
    CreatePlanResponse,
    CreatePlanRequest,
)
from action_runtime.srv import LoadPDDL, LoadPDDLRequest, LoadPDDLResponse
from action_runtime.srv import (
    Observation,
    ObservationRequest,
    ObservationResponse,
)
from action_runtime.srv import (
    RegisterAction,
    RegisterActionRequest,
    RegisterActionResponse,
)
from action_runtime.srv import (
    RegisterCondition,
    RegisterConditionRequest,
    RegisterConditionResponse,
)
from action_runtime.srv import RunAction, RunActionRequest, RunActionResponse
from action_runtime.srv import RunPlan, RunPlanRequest, RunPlanResponse

# pddl deps
import pddl_parser.action as action
import pddl_parser.PDDL as PDDL

# python deps
from typing import List, Union


class ActionRuntime:
    def __init__(self):
        self.srv_create_plan = rospy.Service(
            "create_plan", CreatePlan, self._handle_create_plan
        )
        self.srv_load_pddl = rospy.Service(
            "load_pddl", LoadPDDL, self._handle_load_pddl
        )
        self.srv_register_action = rospy.Service(
            "register_action", RegisterAction, self._handle_register_action
        )
        self.srv_register_condition = rospy.Service(
            "register_condition", RegisterCondition, self._handle_register_condition
        )
        self.srv_run_action = rospy.Service(
            "run_action", RunAction, self._handle_run_action
        )
        self.srv_run_plan = rospy.Service("run_plan", RunPlan, self._handle_run_plan)

        self.actions = {}
        self.action_mappings = {}
        self.conditions = []
        self.observers = {}
        self.domain = None
        self.problem = None
        self.plan = Union[List[action.Action], None]
        self.parser = PDDL.PDDL_Parser()

    def _handle_create_plan(self, _: CreatePlanRequest) -> CreatePlanResponse:
        self.plan = None
        resp = CreatePlanResponse()
        if self.domain is None:
            resp.msg = "No domain loaded"
            resp.success = False
            return resp
        if self.problem is None:
            resp.msg = "No problem loaded"
            resp.success = False
            return resp

        self.parser.scan_tokens(self.domain)
        self.parser.scan_tokens(self.problem)
        self.parser.parse_domain(self.domain)
        self.parser.parse_problem(self.problem)

        self.plan = self.parser.actions

        if self.plan is not None and len(self.plan) > 0:
            resp.success = True
        else:
            resp.success = False
            resp.msg = "No plan found"

        return resp

    def _handle_load_pddl(self, req: LoadPDDLRequest) -> LoadPDDLResponse:
        resp = LoadPDDLResponse()
        if req.domain:
            self.domain = req.filepath
        elif req.problem:
            self.problem = req.filepath
        else:
            resp.msg = "Request must specify either domain or problem"
            resp.success = False
            return resp
        resp.success = True
        return resp

    def _handle_register_action(
        self, req: RegisterActionRequest
    ) -> RegisterActionResponse:
        try:
            self.actions[req.action.name] = rospy.ServiceProxy(
                req.service, ActionTrigger
            )
        except KeyError:
            msg = "Service type {} not found".format(req.service_type)
            rospy.logerr(msg)
            return RegisterActionResponse(success=False, msg=msg)
        return RegisterActionResponse(success=True)

    def _handle_register_condition(
        self, req: RegisterConditionRequest
    ) -> RegisterConditionResponse:
        try:
            self.conditions.append(req.condition)
            self.observers[req.condition] = rospy.ServiceProxy(req.service, Observation)
        except rospy.ServiceException:
            msg = "Service {} not found".format(req.service)
            rospy.logerr(msg)
            return RegisterConditionResponse(success=False, msg=msg)
        return RegisterConditionResponse(success=True)

    def _handle_run_action(self, req: RunActionRequest) -> RunActionResponse:
        action_result = self.run_action(req.action.name, req.action.args)
        return RunActionResponse(success=action_result.success, msg=action_result.msg)

    def _handle_run_plan(self, _: RunPlanRequest) -> RunPlanResponse:
        if self.plan is None:
            return RunPlanResponse(success=False, msg="No plan loaded")

        assert self.plan is not None and isinstance(self.plan, list)
        for action in self.plan:
            action_result = self.run_action(action.name, action.args)
            if not action_result.success:
                return RunPlanResponse(success=False, msg=action_result.msg)
        return RunPlanResponse(success=True)

    def run_action(self, action: str, args: List[str] = []) -> ActionTriggerResponse:
        srv: rospy.ServiceProxy = self.actions[action]
        resp: ActionTriggerResponse = srv(ActionTriggerRequest(args=args))
        return resp

    def observe(self, condition: str, args: List[str] = []) -> ObservationResponse:
        srv: rospy.ServiceProxy = self.observers[condition]
        resp: ObservationResponse = srv(ObservationRequest(args=args))
        return resp
