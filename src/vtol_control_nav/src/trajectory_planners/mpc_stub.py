"""하위 호환성 유지를 위한 재익스포트 모듈."""

from .mpc_planner import MPCPlanner

# 기존 코드/테스트에서 MPCPlannerStub 이름으로 참조하던 코드를 위한 alias
MPCPlannerStub = MPCPlanner

__all__ = ['MPCPlanner', 'MPCPlannerStub']
