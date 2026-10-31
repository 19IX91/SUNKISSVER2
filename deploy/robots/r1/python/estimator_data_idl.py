"""Cyclone DDS type for the R1 rt/EstimatorData topic."""
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types


@dataclass
@annotate.final
@annotate.autoid("sequential")
class EstimatorData_(idl.IdlStruct, typename="unitree_go.msg.dds_.EstimatorData_"):
    contact_estimate: types.array[types.float32, 4]
    position_2d: types.array[types.float32, 3]
    position_3d: types.array[types.float32, 3]
    v_body_3d: types.array[types.float32, 3]
    v_world_3d: types.array[types.float32, 3]
