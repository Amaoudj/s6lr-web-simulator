# Robot source assets

This directory preserves the untouched URDF, original MJCF, and all 18 STL
files used by the web simulator. The browser copies live under
`public/models/s6lr`.

Only the two high-density servo meshes are decimated for browser delivery.
They are each instanced six times by the URDF, so reducing those copies has the
largest impact without changing the robot dimensions or kinematics.

To rebuild the optimized copies:

```bash
python -m pip install -r scripts/requirements-mesh.txt
python scripts/optimize_meshes.py
```
