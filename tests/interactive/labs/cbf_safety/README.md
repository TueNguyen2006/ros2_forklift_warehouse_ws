# CBF Safety Lab

Purpose:

```text
Animate a rear-steer forklift controlled inside an octagonal safe set.
```

Structure:

```text
app.py
    Matplotlib UI, safe/unsafe drawing, reference selection, Play/Pause loop.

scenarios/
    Future CBF test cases such as narrow safe sets or moving references.

assets/
    Future exported animations or figures.
```

CBF logic lives in `warehouse_visual_localization.core.safety`; this folder only
contains UI and scenario wiring.

