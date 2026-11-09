import json,glob,os
import numpy as np,pandas as pd
# Paths: raw runs live in raw_results/ at the repo root; every script writes
# into analysis/output/ (intermediate pickles in output/work, figures, tables,
# the HTML page and the workbook beside it).
HERE=os.path.dirname(os.path.abspath(__file__))
R=os.path.join(HERE,'..','raw_results')
OUT=os.path.join(HERE,'output')
for _d in ('work','figures','tables','xl'):os.makedirs(os.path.join(OUT,_d),exist_ok=True)
os.chdir(os.path.join(OUT,'work'))
SUITES={'native_do':['NonMad_NewGain_FullSuite_1','NonMad_NewGain_FullSuite_2','NonMad_NewGain_FullSuite_3'],
 'native_no':['NonMad_NoseOnly_1','NonMad_NoseOnly_2','NonMad_NoseOnly_3'],
 'mad_do':['MAD_FullSuite_1','MAD_FullSuite_2','MAD_FullSuite_3'],
 'mad_no':['MAD_NoseOnly_1','MAD_NoseOnly_2','MAD_NoseOnly_3']}
P=[f'p{i}' for i in range(1,8)]
Qt={1:10.,2:15.,3:20.,4:30.}
def load(run):
    j=json.load(open(run+'/meta.json')); s=pd.read_csv(run+'/samples.csv')
    t=j['tare']; c=j['flow_conv']
    s['q']=((s.sfm_raw-32000)/140-t['flow_slm'])*c
    for i,p in enumerate(P):
        s[p]=s[f'{p}_raw']/60-t[f'{p}_Pa']
    s['t']=s.tick*0.01
    return j,s
def runs(suite):
    out={}
    d=[p for p in glob.glob(f'{R}/**/{suite}',recursive=True) if os.path.isdir(p)]
    assert len(d)==1,(suite,d)
    for m in glob.glob(f'{d[0]}/*/meta.json'):
        run=os.path.dirname(m); it=os.path.basename(run).split('_')[-1].split('x')[0]
        out[it]=run
    return out
def breaths(s):
    bs=sorted(s.breath.unique())
    # ten full breaths; trailing one-row stub dropped
    return [s[s.breath==b].reset_index(drop=True) for b in bs if (s.breath==b).sum()>50]
