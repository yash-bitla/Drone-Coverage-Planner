class RoutePlannerError(Exception):
    """Base exception for the code sample."""


class InvalidInputError(RoutePlannerError):
    """Raised when an input cannot describe a valid planning problem."""


class InfeasibleRouteError(RoutePlannerError):
    """Raised when no battery-feasible or obstacle-free plan exists."""
