# Training gates: p3-final-kpwide (legs of 500 iterations, both arms together)

+0 = `/home/ubuntu/runs/opt-lab-2/rp-p2-s42-r2-cont-1500/model_4496.pt`: flat_dr 14, bumpy_dr 35, nompd 39

- leg 1 (+500): bumpy arm {'flat_dr': 14, 'bumpy_dr': 31, 'nompd': 25, 'seam_flat': 'PASS'}, control {'flat_dr': 16, 'bumpy_dr': 38, 'nompd': 28, 'seam_flat': 'PASS'} -> **continue** []
- leg 2 (+1000): bumpy arm {'flat_dr': 34, 'bumpy_dr': 46, 'nompd': 42, 'seam_flat': 'PASS'}, control {'flat_dr': 18, 'bumpy_dr': 47, 'nompd': 34, 'seam_flat': 'PASS'} -> **continue** []
- leg 3 (+1500): bumpy arm {'flat_dr': 11, 'bumpy_dr': 23, 'nompd': 14, 'seam_flat': 'PASS'}, control {'flat_dr': 23, 'bumpy_dr': 52, 'nompd': 51, 'seam_flat': 'PASS'} -> **continue** []
  - extension rule (both arms or neither): bumpy_dr drop +23 (> 10?) with flat_dr drift -3 vs +0 (|.| <= 10?) -> EXTEND +500
- leg 4 (+2000): bumpy arm {'flat_dr': 10, 'bumpy_dr': 18, 'nompd': 7, 'seam_flat': 'PASS'}, control {'flat_dr': 12, 'bumpy_dr': 29, 'nompd': 20, 'seam_flat': 'PASS'} -> **continue** []
  - extension rule (both arms or neither): bumpy_dr drop +5 (> 10?) with flat_dr drift -4 vs +0 (|.| <= 10?) -> no

Finished 4 legs (2000 iterations per arm).
