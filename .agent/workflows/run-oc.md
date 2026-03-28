---
description: Run a Python script with the ROCm environment and virtual environment activated
---

// turbo-all

1. Set the HSA override environment variable and activate the virtual environment, then run the requested Python script using a persistent terminal:

```bash
export HSA_OVERRIDE_GFX_VERSION=10.3.0 && source ~/myenv2/bin/activate && python <script>
```

Replace `<script>` with the path to the Python file the user wants to run (e.g. `oc57.py`). Use a persistent terminal so the environment stays active for follow-up runs.
