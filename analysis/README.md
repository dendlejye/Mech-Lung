# Analysis

Python scripts behind the Final Report Analysis page and workbook. They read the
twelve suites in `raw_results/` and write everything to `analysis/output/`
(ignored by git).

Run in this order, from any folder (Python 3, `pip install -r requirements.txt`):

```
python pipeline.py   # per-breath and per-stroke figures from every run
python results.py    # peak-window pressures, steady fits, uncertainty, quasi-steady check
python figures.py    # all 37 figures (300 dpi PNG), common-flow values, drop and resistance
python facts.py      # prints the numbers quoted in captions; writes the SDP811 bleed bounds
python page.py       # final_report_analysis.html, CSV tables and a zip of figures and tables
python workbook.py   # xl/Final_Report_Analysis.xlsx
```

`core.py` holds the paths, the suite list and the loader. Each script reads the
previous ones' pickles in `output/work/`, so rerun from `pipeline.py` after
changing a run. The workbook's formulas are calculated when Excel opens it.

## Suites

| Configuration | Suites |
|---|---|
| Native, dual-orifice | `NonMad_NewGain_FullSuite_1` to `_3` |
| Native, nose-only | `NonMad_NoseOnly_1` to `_3` |
| MAD, dual-orifice | `MAD_FullSuite_1` to `_3` |
| MAD, nose-only | `MAD_NoseOnly_1` to `_3` |

## Method

- First breath of each preset and first stroke of each steady mode excluded.
- Flow: SFM3000 raw / 140 less the tare, times the logger's `flow_conv` (room T and
  station P). Pressure: raw / 60 less the tare.
- Fitted peak: half-sine of the commanded phase length fitted to the middle 60 %
  of each phase, timing free; lag is the fitted delay.
- Peak window: mean over ±5 % of the phase about the commanded peak time. Window
  flow is the flow paired with peak pressure.
- Stall: commanded reversal until the 0.1 s moving-mean flow passes 0.5 L/min.
  Ripple: SD of flow about a 0.5 s moving mean, middle 60 % of the phase.
- Steady pressure: the logger's mean over the middle 80 % of each stroke.
  Steady fit p = aQ + bQ|Q| through s1 to s4 per configuration, direction and port.
- Uncertainty: SDP811 0.1 Pa zero-point and 3 % span, with the 1.5 % SFM3000 flow
  uncertainty carried through the local exponent of the steady fit.
- Common flow: p + p_fit(Q_ref) − p_fit(Q_window), Q_ref the native dual-orifice
  window flow for that preset and phase.
