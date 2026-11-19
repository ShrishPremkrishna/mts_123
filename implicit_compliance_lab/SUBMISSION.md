# Optional Lab 1: Implicit Compliance — Submission

Tej Kosaraju. Short answers to the lab questions. Details and ablations are in `README.md`.

## 1–2. Notebooks

Both notebooks are complete, with outputs:

- Explicit classifier: `notebooks/explicit_classifier.ipynb`
- Implicit classifier: `notebooks/implicit_classifier.ipynb` (window size 200 samples, approximately 0.2 s)

## 3. Accuracy

Validation accuracy at the final epoch (handout 90/10 split):

| Classifier | Contact | Incline | Material |
|---|---|---|---|
| Explicit (DenseTact images, ResNet-50) | — | 98.2 % | 89.8 % |
| Implicit (joint states only, 1D CNN) | 95.9 % | 85.7 % | 79.0 % |

## 4. Comparison

The implicit classifier detects contact with 95.9 % accuracy from the motor joint states alone, without any tactile sensor. On Pupper, joint "effort" is `kp × (commanded angle − measured angle)`, so it measures how far a position-controlled joint deflects under load. The motor therefore acts as a spring, and its deflection is an implicit touch signal. In our tests, adding effort to position and velocity raised incline and material accuracy by approximately 13 points. Thus yes, the motors act as implicit tactile sensors for contact. For terrain identification (incline and material), the tactile images are better. The joint states also lost terrain identity on a recording session that was not in training, while contact detection stayed above 92 %. This agrees with proprioception-only learned locomotion, which needs contact more than terrain identity.

## Deployment challenge

1. **Export:** the trained model is exported as NumPy weights (`robot/implicit_classifier.npz`). The robot runs it with NumPy only, because the lab reports that PyTorch overloaded the Raspberry Pi.
2. **Integration:** `robot/contact_ik.py` extends the Lab 3 `ik.py` node. It buffers the last 200 `joint_states` samples and runs the detector at 50 Hz.
3. **Pause:** on each detected contact, the triangle trajectory holds for 1 s, then continues.

**Result on Pupper:** 0 pauses in 19 s of free swing, and 10 pauses of 1 s in 29 s with a book under the foot. The video `IMG_3875.MOV` shows the run.

**Challenge topics:**

- **Preprocessing:** use the same channel order and the same ~1 kHz stream as the dataset, with no resampling.
- **Filtering:** raw predictions flicker, so contact needs 3 consecutive detections and release needs 5. Only one pause is allowed per touchdown.
- **False positives:** the dataset model fired in free air on our leg, because the dataset leg had the DenseTact foot. Also, the Lab 3 triangle (±15 cm) was out of reach, so effort was abnormally high. We used a smaller triangle that matches the dataset.
- **More data or a new model:** we recorded 25 s of our own leg's free swing. The final detector compares the live effort with this free-swing effort at the same point of the trajectory, and a large difference counts as contact. It uses only the motor signal, and it needs no contact labels.
