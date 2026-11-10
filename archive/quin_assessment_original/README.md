# Historical Assessment Archive — Quin Motion Algorithm Engineer (MAE)
**Provenance & Original Submission Record**

---

## 1. Purpose of this Archive

This directory retains the original technical assessment problem statement, candidate resume, historical submission drafts, and source telemetry dataset for the **Quin Motion Algorithm Engineer** hiring assessment.

To showcase this engineering work as an independent project in Sachin Sagar's professional portfolio without attributing the problem statement or proprietary data to Quin:
1. The active repository has been reworked into an open-source engineering engine: **`edge-imu-motion-analytics`** (*Edge IMU Motion Analytics & Vibration DSP Engine*).
2. The telemetry dataset is generated via a first-principles mathematical and physical data-generating process ([`../../src/generate_synthetic_telemetry.py`](../../src/generate_synthetic_telemetry.py)), which replicates the exact 2-stroke powertrain kinematics, ballistic free-fall plateaus, landing shock impulses, and sensor communication dropouts.
3. The original assessment specification PDFs, draft submissions, and raw dataset are preserved here strictly for historical reference.

---

## 2. Archived Contents

```
archive/quin_assessment_original/
├── README.md                                  # Archive documentation and provenance note
├── mx_imu.csv                                 # Original 6-axis 500 Hz IMU telemetry dataset (45,000 samples)
├── specifications/                            # Original task problem statement and job description
│   ├── MAE Task - Motocross Jump Analytics.pdf # Original assessment instructions & objectives
│   ├── Quin Motion Algorithm Engineer.pdf     # Job description & role overview
│   └── MPCV_Sachin Sagar.pdf                  # Candidate profile / CV
└── submission_reports/                        # Historical draft reports and PDF deliverables
    ├── MAE_task_report_ssagar.pdf             # Historical compiled PDF report
    ├── report_draft.odt                       # Editable LibreOffice report draft
    └── report_draft.rtf                       # Rich text formatted draft
```

