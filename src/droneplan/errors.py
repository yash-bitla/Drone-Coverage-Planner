class PlannerError(Exception):
    pass


class InvalidInputError(PlannerError, ValueError):
    pass


class InfeasiblePlanError(PlannerError):
    pass
