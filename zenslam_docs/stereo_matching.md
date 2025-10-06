# Stereo Matching

After detecting and describing features independently in left and right images:

1. Descriptors are matched with a brute-force matcher (Hamming for ORB, L2 for SIFT).
2. Current configuration uses cross-check = true to retain mutually closest matches.
3. Matches are stored in `stereo_frame.matches`.

## Potential Issues
- Descriptor asymmetry (lighting, viewpoint) can reduce raw matches.
- Scale / orientation differences mitigated by multi-scale descriptors (e.g., SIFT).

## Future Improvements
- Lowe’s ratio test
- Symmetry check if cross-check removed
- Epipolar-guided candidate reduction
- Outlier rejection via robust estimators (RANSAC before epipolar pruning)
