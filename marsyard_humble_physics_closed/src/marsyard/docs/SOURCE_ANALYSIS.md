# Mars Yard Clean Package Notes

Generated from ERC Mars Yard source files.

Key decisions:
- Removed the brown visual base / rectangular filler.
- Kept only the Mars Yard mesh visible.
- Used the ERC orthophoto as an alpha PNG texture.
- Added low-poly terrain collision from the same height grid.
- Kept SDF because this is a world/environment, not a robot model.

Mesh stats:
- Visual faces kept: 81554
- Visual vertices used: 41417
- Collision vertices: 2553
- Collision triangles: 4868
- Collision downsample step: 4
