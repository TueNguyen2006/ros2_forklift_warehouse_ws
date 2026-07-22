# forklift_model

Planned home for simulator-independent forklift kinematics, dynamics, geometry, and tire/load models.

Current model code still lives in `warehouse_visual_localization.core` because it is tightly coupled to the existing visual navigation experiments and tests. Move modules here only when the import boundary can stay algorithm-compatible and tests cover the move.

Expected future layout:

```text
kinematics/
dynamics/
geometry/
tire_model/
```
