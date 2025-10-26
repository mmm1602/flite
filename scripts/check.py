# flite/scripts/check.py
import numpy as np, pathlib as pl

p = pl.Path(__file__).resolve().parents[1] / "public"

dist = np.load(p / "distance_matrix.npy")
pred = np.load(p / "predecessors.npy")
pts  = np.load(p / "points_lat_long.npy")
assets = np.load(p / "asset_indexes.npy")
photos = np.load(p / "photo_indexes.npy")

print("RAW:")
print("  dist", dist.shape, dist.dtype)
print("  pred", pred.shape, pred.dtype)
print("  pts ", pts.shape,  pts.dtype)

# ---- Harmonize to common N (= min of first dims, but also trim pred's columns) ----
N = min(dist.shape[0], pred.shape[0], pts.shape[0])
if pred.shape[1] != N:
    print(f"[warn] pred has {pred.shape[1]} cols; trimming to {N}")
    pred = pred[:N, :N]
dist = dist[:N, :N]
pts  = pts[:N, :]

print("HARMONIZED:")
print("  dist", dist.shape)
print("  pred", pred.shape)
print("  pts ", pts.shape)

# Check target ranges after harmonizing
def to_range(idx):
    idx = np.asarray(idx).ravel().astype(int)
    if idx.size == 2:
        lo, hi = idx[0], idx[1]     # end is exclusive in the challenge guide
        lo = max(0, lo); hi = min(N, max(lo, hi))
        return list(range(lo, hi))
    return [i for i in idx if 0 <= i < N]

photo_nodes  = to_range(photos)
asset_nodes  = to_range(assets)
print(f"photo nodes kept: {len(photo_nodes)} (min={min(photo_nodes or [None])}, max={max(photo_nodes or [-1])})")
print(f"asset nodes kept: {len(asset_nodes)} (min={min(asset_nodes or [None])}, max={max(asset_nodes or [-1])})")
