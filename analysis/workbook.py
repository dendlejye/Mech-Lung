from core import *
from openpyxl import Workbook
from openpyxl.styles import Font,PatternFill,Alignment,Border,Side
from openpyxl.utils import get_column_letter as L
b=pd.read_pickle('b.pkl');pk=pd.read_pickle('pk2.pkl');sr=pd.read_pickle('sr.pkl');F=pd.read_pickle('fits.pkl');st=pd.read_pickle('st.pkl');q=pd.read_pickle('qual.pkl');lk=pd.read_pickle('leak.pkl');B=pd.read_pickle('bleed.pkl');dr=pd.read_pickle('drop.pkl')
CF=['native_do','native_no','mad_do','mad_no']
LAB={'native_do':'Native, dual-orifice','native_no':'Native, nose-only','mad_do':'MAD, dual-orifice','mad_no':'MAD, nose-only'}
PH={'insp':'Inspiration','exp':'Expiration'};DIR={'in':'In','out':'Out'}
wb=Workbook();FN='Arial'
HF=Font(name=FN,bold=True,size=10);BF=Font(name=FN,size=10);VF=Font(name=FN,size=10,color='0000FF');TF=Font(name=FN,bold=True,size=13)
HFILL=PatternFill('solid',fgColor='DCE6F1');thin=Side(style='thin',color='A6A6A6')
def sheet(name,heads,widths=None,first=False):
    ws=wb.active if first else wb.create_sheet(name);ws.title=name
    for j,h in enumerate(heads,1):
        c=ws.cell(1,j,h);c.font=HF;c.fill=HFILL;c.alignment=Alignment(wrap_text=True,vertical='bottom');c.border=Border(bottom=thin)
        ws.column_dimensions[L(j)].width=(widths[j-1] if widths else max(11,min(24,len(h)+3)))
    ws.freeze_panes='A2';ws.row_dimensions[1].height=30
    return ws
def put(ws,r,c,v,fmt=None,val=False):
    x=ws.cell(r,c,v);x.font=VF if val else BF
    if fmt:x.number_format=fmt
    return x
# ---------- Notes (filled last)
notes=wb.active;notes.title='Notes'
# ---------- Per breath
pb=sheet('Per breath',['Configuration','Suite','Preset','Breath','Phase','Target flow (L/min)','Fitted peak (L/min)','Lag (s)','Window flow (L/min)','Raw max (L/min)','Ripple SD (L/min)','Stall (s)','Volume (mL)','Room T (°C)','Station P (hPa)']+[f'{p} (Pa)' for p in P])
bb=b.copy();bb['ci']=bb.cfg.map(CF.index);bb=bb.sort_values(['ci','suite','preset','phase','breath'],ascending=[1,1,1,0,1])
for i,r in enumerate(bb.itertuples(),2):
    vals=[LAB[r.cfg],r.suite,r.preset,r.breath,PH[r.phase],r.target,r.fit,r.lag,r.win,r.rawmax,r.ripple,r.stall,r.vol,r.T,r.Pst]+[getattr(r,p) for p in P]
    for j,v in enumerate(vals,1):put(pb,i,j,v,None if j<=5 else ('0.0' if j in(13,14,15) else '0.000'),val=j>5)
NB=len(bb)+1
pb.cell(1,4).value='Breath (2 to 10)'
# per breath range refs
def R(col):return f"'Per breath'!${col}$2:${col}${NB}"
PCOLS=f"'Per breath'!$P$2:$V${NB}";PHEAD="'Per breath'!$P$1:$V$1"
# ---------- Steady strokes
ss=st[~st['first']].copy();ss['ci']=ss.cfg.map(CF.index);ss=ss.sort_values(['ci','suite','mode','breath'])
sk=sheet('Steady strokes',['Configuration','Suite','Mode','Stroke','Direction','Target flow (L/min)','Mean flow (L/min)','Seal leak (%)']+[f'{p} (Pa)' for p in P])
for i,r in enumerate(ss.itertuples(),2):
    vals=[LAB[r.cfg],r.suite,f's{r.mode}',r.breath,DIR[r.dir],r.target,r.qmean,r.leak_pct]+[getattr(r,p) for p in P]
    for j,v in enumerate(vals,1):put(sk,i,j,v,None if j<=5 else '0.000',val=j>5)
NS=len(ss)+1
def S(col):return f"'Steady strokes'!${col}$2:${col}${NS}"
SPC=f"'Steady strokes'!$I$2:$O${NS}";SPH="'Steady strokes'!$I$1:$O$1"
# ---------- Flow delivery
fd=sheet('Flow delivery',['Configuration','Phase','Preset','Target (L/min)','Fitted peak (L/min)','SD (L/min)','Error vs target','Outside ±1.5% span','Lag (s)','Window flow (L/min)','Raw max (L/min)','Stall (s)','Ripple SD (L/min)','Volume (mL)','Breaths'])
r=2
for c in CF:
    for ph in ('insp','exp'):
        for pre in (1,2,3,4):
            put(fd,r,1,LAB[c]);put(fd,r,2,PH[ph]);put(fd,r,3,pre)
            crit=f"{R('A')},$A{r},{R('E')},$B{r},{R('C')},$C{r}"
            for col,src,fmt in ((4,'F','0.0'),(5,'G','0.00'),(9,'H','0.00'),(10,'I','0.00'),(11,'J','0.00'),(12,'L','0.00'),(13,'K','0.00'),(14,'M','0')):
                put(fd,r,col,f"=AVERAGEIFS({R(src)},{crit})",fmt)
            sd=b[(b.cfg==c)&(b.phase==ph)&(b.preset==pre)].fit.std()
            put(fd,r,6,sd,'0.000',val=True)
            put(fd,r,7,f"=E{r}/D{r}-1",'+0.00%;-0.00%;0.00%')
            put(fd,r,8,f'=IF(ABS(G{r})>0.015,"Yes","No")')
            put(fd,r,15,f"=COUNTIFS({crit})",'0')
            r+=1
# ---------- Peak pressures (long)
pp=sheet('Peak pressures',['Configuration','Preset','Phase','Port','Window flow (L/min)','Pressure (Pa)','Breath SD (Pa)','Suite 1 (Pa)','Suite 2 (Pa)','Suite 3 (Pa)','Suite range (Pa)','Uncertainty ± (Pa)','Steady fit at window flow (Pa)','Breathing − steady (Pa)','p / (½ρU²)','Pressure at common flow (Pa)','Common flow Q_ref (L/min)'])
pk['ci']=pk.cfg.map(CF.index);pk['pi']=pk.phase.map({'insp':0,'exp':1});ppk=pk.sort_values(['ci','pi','preset','port'])
for r,x in enumerate(ppk.itertuples(),2):
    put(pp,r,1,LAB[x.cfg]);put(pp,r,2,x.preset);put(pp,r,3,PH[x.phase]);put(pp,r,4,x.port)
    crit=f"{R('A')},$A{r},{R('C')},$B{r},{R('E')},$C{r}"
    col=f"INDEX({PCOLS},0,MATCH($D{r}&\" (Pa)\",{PHEAD},0))"
    put(pp,r,5,f"=AVERAGEIFS({R('I')},{crit})",'0.00')
    put(pp,r,6,f"=AVERAGEIFS({col},{crit})",'0.00')
    put(pp,r,7,x.sd,'0.000',val=True)
    for k in (1,2,3):put(pp,r,7+k,f"=AVERAGEIFS({col},{crit},{R('B')},{k})",'0.00')
    put(pp,r,11,f"=MAX(H{r}:J{r})-MIN(H{r}:J{r})",'0.000')
    put(pp,r,12,x.u,'0.00',val=True);put(pp,r,13,x.p_steady,'0.00',val=True)
    put(pp,r,14,f"=F{r}-M{r}",'0.00');put(pp,r,15,x.cp,'0.0',val=True);put(pp,r,16,x.p_cf,'0.00',val=True);put(pp,r,17,x.q_ref,'0.00',val=True)
NP=len(ppk)+1
def PPc(col):return f"'Peak pressures'!${col}$2:${col}${NP}"
# ---------- Steady pressures (long)
sp=sheet('Steady pressures',['Configuration','Mode','Direction','Port','Mean flow (L/min)','Strokes','Seal leak (%)','Pressure (Pa)','Stroke SD (Pa)','Suite 1 (Pa)','Suite 2 (Pa)','Suite 3 (Pa)','Suite range (Pa)','Uncertainty ± (Pa)','Steady fit (Pa)'])
sr['ci']=sr.cfg.map(CF.index);sr['di']=sr.dir.map({'in':0,'out':1});ssr=sr.sort_values(['ci','di','mode','port'])
for r,x in enumerate(ssr.itertuples(),2):
    put(sp,r,1,LAB[x.cfg]);put(sp,r,2,f's{x.mode}');put(sp,r,3,DIR[x.dir]);put(sp,r,4,x.port)
    crit=f"{S('A')},$A{r},{S('C')},$B{r},{S('E')},$C{r}"
    col=f"INDEX({SPC},0,MATCH($D{r}&\" (Pa)\",{SPH},0))"
    put(sp,r,5,f"=AVERAGEIFS({S('G')},{crit})",'0.00');put(sp,r,6,f"=COUNTIFS({crit})",'0');put(sp,r,7,f"=AVERAGEIFS({S('H')},{crit})",'0.00')
    put(sp,r,8,f"=AVERAGEIFS({col},{crit})",'0.00');put(sp,r,9,x.sd,'0.000',val=True)
    for k in (1,2,3):put(sp,r,9+k,f"=AVERAGEIFS({col},{crit},{S('B')},{k})",'0.00')
    put(sp,r,13,f"=MAX(J{r}:L{r})-MIN(J{r}:L{r})",'0.000');put(sp,r,14,x.u,'0.00',val=True);put(sp,r,15,x.fit,'0.00',val=True)
NSP=len(ssr)+1
def SPc(col):return f"'Steady pressures'!${col}$2:${col}${NSP}"
# ---------- Steady fits
sf=sheet('Steady fits',['Configuration','Direction','Port','a (Pa per L/min)','b (Pa per (L/min)²)','RMS residual (Pa)','Largest residual (Pa)','Fit at 30 L/min (Pa)'])
F['ci']=F.cfg.map(CF.index);F['di']=F.dir.map({'in':0,'out':1})
for r,x in enumerate(F.sort_values(['ci','di','port']).itertuples(),2):
    put(sf,r,1,LAB[x.cfg]);put(sf,r,2,DIR[x.dir]);put(sf,r,3,x.port)
    put(sf,r,4,float(x.a),'0.0000',val=True);put(sf,r,5,float(x.b),'0.00000',val=True);put(sf,r,6,float(x.rmse),'0.000',val=True);put(sf,r,7,float(x.maxres),'0.000',val=True)
    put(sf,r,8,f'=IF(B{r}="In",-1,1)*(D{r}*30+E{r}*900)','0.00')
# ---------- Drop & resistance
dd=sheet('Drop and resistance',['Configuration','Condition','Phase','Level','Flow (L/min)','|p7 − p1| (Pa)','Uncertainty ± (Pa)','Resistance (Pa per L/min)','Resistance uncertainty ±'])
r=2
for c in CF:
    for ph in ('insp','exp'):
        for lv in (1,2,3,4):
            x=dr[(dr.cfg==c)&(dr.kind=='breathing')&(dr.phase==ph)&(dr.level==lv)].iloc[0]
            put(dd,r,1,LAB[c]);put(dd,r,2,'Breathing');put(dd,r,3,PH[ph]);put(dd,r,4,lv)
            base=f"{PPc('A')},$A{r},{PPc('C')},$C{r},{PPc('B')},$D{r}"
            put(dd,r,5,f"=SUMIFS({PPc('E')},{base},{PPc('D')},\"p1\")",'0.00')
            put(dd,r,6,f"=ABS(SUMIFS({PPc('F')},{base},{PPc('D')},\"p7\")-SUMIFS({PPc('F')},{base},{PPc('D')},\"p1\"))",'0.00')
            put(dd,r,7,x.u,'0.00',val=True);put(dd,r,8,f"=F{r}/E{r}",'0.000');put(dd,r,9,x.uR,'0.000',val=True);r+=1
    for d_,ph in (('in','insp'),('out','exp')):
        for lv in (1,2,3,4):
            x=dr[(dr.cfg==c)&(dr.kind=='steady')&(dr.phase==ph)&(dr.level==lv)].iloc[0]
            put(dd,r,1,LAB[c]);put(dd,r,2,'Steady');put(dd,r,3,DIR[d_]);put(dd,r,4,f's{lv}')
            base=f"{SPc('A')},$A{r},{SPc('C')},$C{r},{SPc('B')},$D{r}"
            put(dd,r,5,f"=ABS(SUMIFS({SPc('E')},{base},{SPc('D')},\"p1\"))",'0.00')
            put(dd,r,6,f"=ABS(SUMIFS({SPc('H')},{base},{SPc('D')},\"p7\")-SUMIFS({SPc('H')},{base},{SPc('D')},\"p1\"))",'0.00')
            put(dd,r,7,x.u,'0.00',val=True);put(dd,r,8,f"=F{r}/E{r}",'0.000');put(dd,r,9,x.uR,'0.000',val=True);r+=1
# ---------- Device and route comparison
def cmp_sheet(name,groups,A_,B_,la,lb):
    ws=sheet(name,['Group','Preset','Phase','Port',f'{la} (Pa)',f'{lb} (Pa)',f'{lb} − {la} (Pa)','Combined uncertainty ± (Pa)',f'{la} at common flow (Pa)',f'{lb} at common flow (Pa)',f'{lb} − {la} at common flow (Pa)'],[20,8,13,7,14,14,16,16,16,16,18])
    r=2
    for g,ga,gb in groups:
        for ph in ('insp','exp'):
            for pre in (1,2,3,4):
                for p in P:
                    put(ws,r,1,g);put(ws,r,2,pre);put(ws,r,3,PH[ph]);put(ws,r,4,p)
                    ca=f"{PPc('A')},\"{LAB[ga]}\",{PPc('B')},$B{r},{PPc('C')},$C{r},{PPc('D')},$D{r}";cb=ca.replace(LAB[ga],LAB[gb])
                    put(ws,r,5,f"=SUMIFS({PPc('F')},{ca})",'0.00');put(ws,r,6,f"=SUMIFS({PPc('F')},{cb})",'0.00');put(ws,r,7,f"=F{r}-E{r}",'+0.00;-0.00;0.00')
                    put(ws,r,8,f"=SQRT(SUMIFS({PPc('L')},{ca})^2+SUMIFS({PPc('L')},{cb})^2)",'0.00')
                    put(ws,r,9,f"=SUMIFS({PPc('P')},{ca})",'0.00');put(ws,r,10,f"=SUMIFS({PPc('P')},{cb})",'0.00');put(ws,r,11,f"=J{r}-I{r}",'+0.00;-0.00;0.00')
                    r+=1
cmp_sheet('Device effect',[('Dual-orifice','native_do','mad_do'),('Nose-only','native_no','mad_no')],None,None,'Native','MAD')
cmp_sheet('Route effect',[('Native','native_do','native_no'),('MAD','mad_do','mad_no')],None,None,'Dual-orifice','Nose-only')
# ---------- Rig summary
rs=sheet('Rig summary',['Configuration','Seal leak s1 (%)','Seal leak s2 (%)','Seal leak s3 (%)','Seal leak s4 (%)','Exp ripple min (L/min)','Exp ripple max (L/min)','Insp ripple min (L/min)','Insp ripple max (L/min)','Stall min (s)','Stall max (s)','Lag min (s)','Lag max (s)'])
FDc=lambda col:f"'Flow delivery'!${col}$2:${col}$33"
for r,c in enumerate(CF,2):
    put(rs,r,1,LAB[c])
    for k in (1,2,3,4):put(rs,r,1+k,f"=AVERAGEIFS({S('H')},{S('A')},$A{r},{S('C')},\"s{k}\")",'0.00')
    for j,(col,ph,fn) in enumerate((('M','Expiration','MIN'),('M','Expiration','MAX'),('M','Inspiration','MIN'),('M','Inspiration','MAX')),6):
        put(rs,r,j,f"=_xlfn.{fn}IFS({FDc(col)},{FDc('A')},$A{r},{FDc('B')},\"{ph}\")",'0.00')
    put(rs,r,10,f"=_xlfn.MINIFS({FDc('L')},{FDc('A')},$A{r})",'0.00');put(rs,r,11,f"=_xlfn.MAXIFS({FDc('L')},{FDc('A')},$A{r})",'0.00')
    put(rs,r,12,f"=_xlfn.MINIFS({FDc('I')},{FDc('A')},$A{r})",'0.00');put(rs,r,13,f"=_xlfn.MAXIFS({FDc('I')},{FDc('A')},$A{r})",'0.00')
rs.cell(7,1,'Volume gaps: median mL per stroke, breaths 2 to 10, two potentiometer-spike breaths excluded').font=HF
for j,h in enumerate(['Configuration','Preset','Push gap (mL)','Return gap (mL)'],1):
    x=rs.cell(8,j,h);x.font=HF;x.fill=HFILL
lk['ci']=lk.cfg.map(CF.index)
for r,x in enumerate(lk.sort_values(['ci','preset']).itertuples(),9):
    put(rs,r,1,LAB[x.cfg]);put(rs,r,2,x.preset);put(rs,r,3,x.push_leak_ml,'0.0',val=True);put(rs,r,4,x.return_leak_ml,'0.0',val=True)
# ---------- Bleed
bl=sheet('SDP811 bleed',['Configuration','Preset','Phase','Window flow (L/min)','Largest port |p| (Pa)','Bleed low (L/min)','Bleed high (L/min)','Low, share of flow','High, share of flow'])
B['ci']=B.cfg.map(CF.index);B['pi']=B.phase.map({'insp':0,'exp':1})
for r,x in enumerate(B.sort_values(['ci','pi','preset']).itertuples(),2):
    put(bl,r,1,LAB[x.cfg]);put(bl,r,2,x.preset);put(bl,r,3,PH[x.phase]);put(bl,r,4,x.q,'0.00',val=True);put(bl,r,5,x.maxp,'0.00',val=True)
    put(bl,r,6,x.lo,'0.0000',val=True);put(bl,r,7,x.hi,'0.0000',val=True);put(bl,r,8,f"=F{r}/D{r}",'0.00%');put(bl,r,9,f"=G{r}/D{r}",'0.00%')
# ---------- Data quality
dq=sheet('Data quality',['Suite','Configuration','Run','Start','Room T (°C)','Station P (hPa)','Rows','Lost rows','Short flow averages','Failed reads','Flow tare (slm)','Largest pressure tare (Pa)','Clock error (ppm)'])
q['ci']=q.cfg.map(CF.index);q['ii']=q['item'].map(lambda s:(1 if s.startswith('s') else 0,s))
order=['NonMad_NewGain_FullSuite_1','NonMad_NewGain_FullSuite_2','NonMad_NewGain_FullSuite_3','NonMad_NoseOnly_1','NonMad_NoseOnly_2','NonMad_NoseOnly_3','MAD_FullSuite_1','MAD_FullSuite_2','MAD_FullSuite_3','MAD_NoseOnly_1','MAD_NoseOnly_2','MAD_NoseOnly_3']
q['oi']=q.suite.map(order.index);qq=q.sort_values(['oi','ii'])
for r,x in enumerate(qq.itertuples(),2):
    item=('Steady '+x.item) if x.item.startswith('s') else f'Preset {x.item}'
    for j,v in enumerate([x.suite,LAB[x.cfg],item,str(x.start).replace('T',' ')[:16] if isinstance(x.start,str) else '',x.T,x.Pst,x.rows,x.lost,x.short,x.failed,x.tflow,x.tpmax,x.clock],1):
        put(dq,r,j,v,{5:'0.0',6:'0.0',7:'#,##0',11:'0.000',12:'0.000',13:'0.00'}.get(j),val=j>=5)
    if abs(x.clock)>1000:dq.cell(r,13).comment=None;dq.cell(r,14,'Invalid estimate from a 91,154 s PC span in the logger; row timing in this run is normal.').font=BF
nq=len(qq)+1;rt=nq+1
put(dq,rt,1,'Total').font=HF
for j in (7,8,9,10):put(dq,rt,j,f"=SUM({L(j)}2:{L(j)}{nq})",'#,##0').font=HF
# ---------- Notes
notes.column_dimensions['A'].width=26;notes.column_dimensions['B'].width=110
notes['A1']='Final Report Analysis: data workbook';notes['A1'].font=TF
rows=[('Source','Twelve full suites logged on 29 and 30 October 2026: native and MAD models, dual-orifice and nose-only, three suites each. Same data and method as the Final Report Analysis page.'),
('Suites','Native dual-orifice: NonMad_NewGain_FullSuite_1 to 3. Native nose-only: NonMad_NoseOnly_1 to 3. MAD dual-orifice: MAD_FullSuite_1 to 3. MAD nose-only: MAD_NoseOnly_1 to 3.'),
('Exclusions','First breath of each preset and first stroke of each steady mode excluded. Breaths 2 to 10 give 27 breaths per condition; steady modes give 12 out and 15 in strokes.'),
('Colour key','Blue text: values computed in Python from the raw logs (fits, SDs, uncertainty, common-flow values). Black text: formulas in this workbook, which recalculate if the per-breath or per-stroke values change.'),
('Fitted peak','Amplitude of a half-sine of the commanded phase length fitted to the middle 60% of each phase, with timing free. Lag is the fitted delay behind the command.'),
('Window','Mean over ±5% of the phase about the commanded peak time. Window flow is the flow paired with peak pressure.'),
('Stall','Time from the commanded reversal until the 0.1 s moving-mean flow passes 0.5 L/min in the new direction.'),
('Ripple','SD of flow about a 0.5 s moving mean over the middle 60% of the phase.'),
('Steady fit','p = aQ + bQ|Q| through s1 to s4 for each configuration, direction and port (Q in L/min, signed, positive out of the cylinder).'),
('Uncertainty','SDP811 0.1 Pa zero-point and 3% span, combined with 1.5% SFM3000 flow uncertainty propagated through the local exponent of the steady fit.'),
('Common flow','p + p_fit(Q_ref) − p_fit(Q_window), with Q_ref the native dual-orifice window flow for that preset and phase. Keeps the measured breathing value and removes only the flow difference.'),
('Normalised','p / (½ρU²), with U the window flow over the 22 mm tube area and ρ from each suite\'s room temperature and station pressure.'),
('SDP811 bleed','Bounds from Sensirion figures of 33 to 43 sccm at 100 Pa and 90 to 115 sccm at 500 Pa [37]: low bound linear to zero, high bound the power law through the upper points.'),
('Sheets','Per breath and Steady strokes hold the analysed values per breath and stroke. Flow delivery, Peak pressures and Steady pressures average them. Drop and resistance, Device effect, Route effect and Rig summary read from those sheets.')]
for i,(k,v) in enumerate(rows,3):
    a=notes.cell(i,1,k);a.font=HF;a.alignment=Alignment(vertical='top');c=notes.cell(i,2,v);c.font=BF;c.alignment=Alignment(wrap_text=True,vertical='top')
for ws in wb.worksheets:
    if ws.title!='Notes':ws.auto_filter.ref=ws.dimensions if ws.title!='Rig summary' else 'A1:M5'
wb.save('../xl/Final_Report_Analysis.xlsx');print('ok',NB,NS,NP,NSP)
