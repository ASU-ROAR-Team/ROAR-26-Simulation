import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

a = np.load("../initial_inputs/i_heightmap/heightmap.npz")
b = np.load("outputs/heightmap.npz")

grid_a = a["grid"]
grid_b = b["grid"]

fig, axes = plt.subplots(1, 3, figsize=(18, 6))

im0 = axes[0].imshow(grid_a, origin="lower", cmap="terrain")
axes[0].set_title(f"Reference {grid_a.shape}")
plt.colorbar(im0, ax=axes[0], fraction=0.046)

im1 = axes[1].imshow(grid_b, origin="lower", cmap="terrain")
axes[1].set_title(f"Yours {grid_b.shape}")
plt.colorbar(im1, ax=axes[1], fraction=0.046)

diff = grid_a - grid_b
im2 = axes[2].imshow(diff, origin="lower", cmap="coolwarm", vmin=-1, vmax=1)
axes[2].set_title("Difference (ref - yours)")
plt.colorbar(im2, ax=axes[2], fraction=0.046)

plt.tight_layout()
plt.savefig("heightmap_comparison2.png", dpi=150)
print("Saved heightmap_comparison2.png")

# quantitative check
valid = ~np.isnan(grid_a) & ~np.isnan(grid_b)
print(f"Cells valid in both: {valid.sum()} / {grid_a.size}")
print(f"Mean abs diff (where both valid): {np.abs(diff[valid]).mean():.4f} m")
print(f"Max abs diff: {np.nanmax(np.abs(diff[valid])):.4f} m")
