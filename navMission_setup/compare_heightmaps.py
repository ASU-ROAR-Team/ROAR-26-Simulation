import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REF_PATH = "world_setup/initial_inputs/i_heightmap/heightmap.npz"
YOURS_PATH = "outputs/world_Data_1/heightmap/world_Data_1_d0.012_c0.50_heightmap.npz"

a = np.load(REF_PATH)
b = np.load(YOURS_PATH)

grid_a = a["grid"]
grid_b = b["grid"]

fig, axes = plt.subplots(1, 3, figsize=(18, 6))

im0 = axes[0].imshow(grid_a, origin="lower", cmap="terrain")
axes[0].set_title(f"Reference {grid_a.shape}")
plt.colorbar(im0, ax=axes[0], fraction=0.046)

im1 = axes[1].imshow(grid_b, origin="lower", cmap="terrain")
axes[1].set_title(f"Yours {grid_b.shape}")
plt.colorbar(im1, ax=axes[1], fraction=0.046)

if grid_a.shape == grid_b.shape:
    diff = grid_a - grid_b
    im2 = axes[2].imshow(diff, origin="lower", cmap="coolwarm")
    axes[2].set_title("Difference (ref - yours)")
    plt.colorbar(im2, ax=axes[2], fraction=0.046)
else:
    axes[2].text(0.5, 0.5, f"Shape mismatch:\n{grid_a.shape} vs {grid_b.shape}",
                 ha="center", va="center", transform=axes[2].transAxes)
    axes[2].set_title("Cannot diff — shapes differ")

plt.tight_layout()
plt.savefig("heightmap_comparison.png", dpi=150)
print("Saved heightmap_comparison.png")
