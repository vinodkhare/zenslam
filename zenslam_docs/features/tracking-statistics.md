# Tracking Statistics

This document contains tracking statistics from various experimental runs, comparing different configurations and preprocessing options.

---

## Simple RUN

```
================================================================================
OVERALL STATISTICS
================================================================================
Total frames............................ 5977
Mean KLT success rate................... 38.29%
Mean matches............................ 51.2
Mean triangulated....................... 24.8
Mean processing time.................... 0.063s
Mean tracking time...................... 0.044s
Mean detection time (L+R)............... 0.000s
Mean matching time...................... 0.000s
Mean response (L/R)..................... 10.0 / 9.4

================================================================================
PROBLEM FRAMES SUMMARY
================================================================================

Found 2020 problem frames (33.8% of total):
  - Low KLT success rate: 1640 frames
  - Slow processing (>0.085s): 205 frames
  - Low triangulated points: 93 frames
  - Low pose inliers: 82 frames
```

---

## With CLAHE

```
================================================================================
OVERALL STATISTICS
================================================================================
Total frames............................ 5990
Mean KLT success rate................... 36.53%
Mean matches............................ 50.8
Mean triangulated....................... 25.4
Mean processing time.................... 0.072s
Mean tracking time...................... 0.050s
Mean detection time (L+R)............... 0.000s
Mean matching time...................... 0.000s
Mean response (L/R)..................... 21.7 / 20.7

Found 2312 problem frames (38.6% of total):
  - Low KLT success rate: 2008 frames
  - Slow processing (>0.101s): 189 frames
  - Low triangulated points: 88 frames
  - Low pose inliers: 27 frames
```

---

## With larger KLT window size (31x31) as opposed to default (15x15)

```
================================================================================
OVERALL STATISTICS
================================================================================
Total frames............................ 2129
Mean KLT success rate................... 48.72%
Mean matches............................ 65.2
Mean triangulated....................... 28.2
Mean processing time.................... 0.149s
Mean tracking time...................... 0.115s
Mean detection time (L+R)............... 0.000s
Mean matching time...................... 0.000s
Mean response (L/R)..................... 10.0 / 9.3

================================================================================
PROBLEM FRAMES SUMMARY
================================================================================

Found 564 problem frames (26.5% of total):
  - Low KLT success rate: 338 frames
  - Low triangulated points: 110 frames
  - Slow processing (>0.197s): 62 frames
  - Low pose inliers: 54 frames
```

---

## Analysis

Increasing KLT window size seems to improve tracking success at the cost of increased processing time. CLAHE preprocessing does not appear to significantly improve KLT success in this dataset, and may even slightly reduce it, while increasing response values and processing time.

