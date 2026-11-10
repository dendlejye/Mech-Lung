from core import *
import html,zipfile,json
b=pd.read_pickle('b.pkl');pk=pd.read_pickle('pk2.pkl');sr=pd.read_pickle('sr.pkl');dr=pd.read_pickle('drop.pkl');st=pd.read_pickle('st.pkl');q=pd.read_pickle('qual.pkl');lk=pd.read_pickle('leak.pkl');F=pd.read_pickle('fits.pkl');B=pd.read_pickle('bleed.pkl')
T='../tables/'
LAB={'native_do':'Native, dual-orifice','native_no':'Native, nose-only','mad_do':'MAD, dual-orifice','mad_no':'MAD, nose-only'}
g=b.groupby(['cfg','preset','phase']).agg(target=('target','mean'),fit=('fit','mean'),fit_sd=('fit','std'),lag=('lag','mean'),win=('win','mean'),rawmax=('rawmax','mean'),stall=('stall','mean'),ripple=('ripple','mean'),vol=('vol','mean')).reset_index();g['err']=(g.fit/g.target-1)*100
g.to_csv(T+'flow_delivery.csv',index=False)
pk.to_csv(T+'peak_pressures.csv',index=False);sr.to_csv(T+'steady_pressures.csv',index=False);F.to_csv(T+'steady_fits.csv',index=False);dr.to_csv(T+'drop_resistance.csv',index=False);B.to_csv(T+'bleed.csv',index=False)
q.to_csv(T+'data_quality.csv',index=False);lk.to_csv(T+'volume_gaps.csv',index=False)
b.to_csv(T+'per_breath.csv',index=False)
# ---- helpers
m=lambda v,n=2:f'{v:.{n}f}'.replace('-','−')
def tbl(head,rows,cls=''):
    h=''.join(f'<th>{html.escape(x)}</th>' for x in head)
    r=''.join('<tr>'+''.join(f'<td>{c}</td>' for c in row)+'</tr>' for row in rows)
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{r}</tbody></table></div>'
N={'n':0}
FIG={}
def fig(name,cap,short=None):
    N['n']+=1;k=N['n'];FIG[name]=k
    return f'''<figure id="f{k}"><div class="plate"><img src="figures/{name}.png" alt="{html.escape(cap[:140])}" loading="lazy"></div><figcaption><span class="fn">Figure R{k}.</span> {cap}<button class="sv" data-f="figures/{name}.png" data-n="{name}.png">Save PNG</button></figcaption></figure>'''
def tab_title(t,csv=None):
    s=f'<h4 class="tt">{t}'
    if csv:s+=f'<button class="sv" data-f="tables/{csv}" data-n="{csv}">Save CSV</button>'
    return s+'</h4>'
def ptab(cfg):
    rows=[]
    for ph,pl in (('insp','Inspiration'),('exp','Expiration')):
        for pre in (1,2,3,4):
            d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase==ph)].set_index('port').loc[P]
            rows.append([pl,str(pre),m(d.q_win.iloc[0])]+[f'{m(d.loc[p,"p"])} ± {m(d.loc[p,"u"])}' for p in P])
    return tab_title(f'Pressure at the peak window, {LAB[cfg].lower().replace('mad','MAD')} (Pa, with combined uncertainty)')+tbl(['Phase','Preset','Window flow (L/min)']+P,rows)
def stab(cfg):
    rows=[]
    for dd,pl in (('in','In'),('out','Out')):
        for mo in (1,2,3,4):
            d=sr[(sr.cfg==cfg)&(sr['mode']==mo)&(sr.dir==dd)].set_index('port').loc[P]
            rows.append([pl,f's{mo}',m(abs(d.q.iloc[0])),str(int(d.n_strokes.iloc[0])),m(d.leak.iloc[0])]+[f'{m(d.loc[p,"p"])} ± {m(d.loc[p,"u"])}' for p in P])
    return tab_title(f'Steady pressure, {LAB[cfg].lower().replace('mad','MAD')} (Pa, with combined uncertainty)')+tbl(['Stroke','Mode','Mean flow (L/min)','Strokes','Leak (%)']+P,rows)
def cfg_block(cfg,steady_sec,unst_sec):
    s=LAB[cfg].lower().replace('mad','MAD');hu='native' in cfg
    ex=pk[(pk.cfg==cfg)&(pk.phase=='insp')&(pk.preset==4)].set_index('port').p
    o=f'<h3 id="s{steady_sec.replace(".","-")}">{steady_sec} {LAB[cfg].split(", ")[1].capitalize()} steady state airflow</h3>'
    o+=fig(f'steady_profile_{cfg}',f'Steady pressure at each port, {s}, with s1 to s4 mean flows. Each point is the mean over the middle 80% of the stroke, over all strokes of three suites after the first (out strokes n = 12, in strokes n = 15). Bars show combined uncertainty.')
    o+=fig(f'steady_pq_{cfg}',f'Pressure against flow at each port, {s}. Filled points are steady modes, the line is the fit p = aQ + bQ|Q| through s1 to s4 for each direction, and open points are unsteady pressures at the peak window, showing how closely unsteady flow follows the steady relationship.')
    o+=stab(cfg)
    o+=f'<h3 id="s{unst_sec.replace(".","-")}">{unst_sec} {LAB[cfg].split(", ")[1].capitalize()} unsteady airflow</h3>'
    o+=fig(f'unsteady_flow_{cfg}',f'Flow delivered in one breath, {s}. Each measured trace is the time-aligned mean of breaths 2 to 10 from three suites (27 breaths), with a band of ±1 SD. The dashed line is the ideal half-sine at the target peak: expiration at half the preset flow, then inspiration at the preset flow. Grey bands mark the ±5% peak windows. The dotted line marks the commanded reversal, and the flat section after each reversal is the actuator stall.')
    o+=fig(f'unsteady_pressure_breath_{cfg}',f'Pressure over one breath at each port, {s}, as one panel per port so that no trace hides another. Lines are the 27-breath mean, smoothed by a 50 ms moving mean for display only, with time normalised by each preset\'s breath period.')
    o+=fig(f'unsteady_profile_{cfg}',f'Pressure at the peak window at each port, {s}. Mean over ±5% of the phase about the commanded peak time, breaths 2 to 10 of three suites. Bars show combined uncertainty. Ports are evenly spaced by number, not by distance along the airway.')
    o+=ptab(cfg)
    return o
# flow table
ft=[]
for cfg in LAB:
    for ph,pl in (('insp','Insp'),('exp','Exp')):
        for pre in (1,2,3,4):
            r=g[(g.cfg==cfg)&(g.preset==pre)&(g.phase==ph)].iloc[0]
            flag=' class="flag"' if abs(r.err)>1.5 else ''
            ft.append([LAB[cfg],pl,str(pre),m(r.target,1),m(r.fit),m(r.fit_sd,3),f'<span{flag}>{("+" if r.err>0 else "")}{m(r.err)}</span>',m(r.lag),m(r.win),m(r.rawmax),m(r.stall),m(r.ripple),m(r.vol,0)])
# steady fits
fitrows=[]
for cfg in LAB:
    for dd in ('in','out'):
        for p in P:
            r=F[(F.cfg==cfg)&(F.dir==dd)&(F.port==p)].iloc[0]
            fitrows.append([LAB[cfg],{'in':'In','out':'Out'}[dd],p,m(r.a,4),m(r.b,5),m(r.rmse,3),m(r.maxres,3)])
# drop table
dt=[]
for ph,pl in (('insp','Inspiration'),('exp','Expiration')):
    for lv in (1,2,3,4):
        row=[pl,str(lv)]
        for c in LAB:
            r=dr[(dr.cfg==c)&(dr.kind=='breathing')&(dr.phase==ph)&(dr.level==lv)].iloc[0]
            row.append(f'{m(r.q)} | {m(r.dp)} ± {m(r.u)} | {m(r.R,3)}')
        dt.append(row)
# quality
qt=[]
for su,G in q.groupby('suite',sort=False):
    qt.append([su,LAB[G.cfg.iloc[0]],min(G.start.dropna().astype(str))[:16].replace('T',' '),f'{G["T"].iloc[0]:.1f}',f'{G.Pst.iloc[0]:.1f}',f'{G.rows.sum():,}',str(G.lost.sum()),str(G.short.sum()),str(G.failed.sum())])
order=['NonMad_NewGain_FullSuite_1','NonMad_NewGain_FullSuite_2','NonMad_NewGain_FullSuite_3','NonMad_NoseOnly_1','NonMad_NoseOnly_2','NonMad_NoseOnly_3','MAD_FullSuite_1','MAD_FullSuite_2','MAD_FullSuite_3','MAD_NoseOnly_1','MAD_NoseOnly_2','MAD_NoseOnly_3']
qt=[r[1:] for r in sorted(qt,key=lambda r:order.index(r[0]))]
rh=[]
for c in LAB:
    x=g[g.cfg==c];e=x[x.phase=='exp'];i=x[x.phase=='insp'];lp=st[(~st['first'])&(st.cfg==c)].groupby('mode').leak_pct.mean()
    rh.append([LAB[c],f'{m(lp.min(),1)} to {m(lp.max(),1)}',f'{m(e.ripple.min())} to {m(e.ripple.max())}',f'{m(i.ripple.min())} to {m(i.ripple.max())}',f'{m(x.stall.min())} to {m(x.stall.max())}',f'{m(x.lag.min())} to {m(x.lag.max())}'])
# bleed summary
bl=[[LAB[c],f'{m(B[B.cfg==c].maxp.max(),1)}',f'{m(B[B.cfg==c].lop.max())}',f'{m(B[B.cfg==c].hip.max())}'] for c in LAB]
# text pieces
d=lambda c,pre,ph='insp':dr[(dr.cfg==c)&(dr.kind=='breathing')&(dr.phase==ph)&(dr.level==pre)].iloc[0]
pv=lambda c,pre,p,k='p',ph='insp':pk[(pk.cfg==c)&(pk.preset==pre)&(pk.port==p)&(pk.phase==ph)].iloc[0][k]
red=lambda a,n,pre:100*(1-d(a,pre).dp/d(n,pre).dp)
q_ins=pk[pk.phase=='insp'];
body=''
# 5.1 common
sec_common=fig('flow_error','Delivered flow against target for every configuration. Unsteady panels use the fitted half-sine peak, steady panels the mean over the middle 80% of the stroke. The grey band is the ±1.5% flow meter span. Gains were tuned on the native dual-orifice model, so delivery falls with added airway resistance, most in native nose-only.')
sec_common+=tab_title('Flow delivery per configuration, preset and phase, breaths 2 to 10 of three suites','flow_delivery.csv')+tbl(['Configuration','Phase','Preset','Target','Fitted peak','SD','Error %','Lag (s)','Window flow','Raw max','Stall (s)','Ripple SD','Volume (mL)'],ft,'dense')
sec_common+=fig('rig_behaviour','Actuator behaviour by preset. Lag is the fitted half-sine delay behind the command. Stall is the time from the commanded reversal until the 0.1 s moving-mean flow passes 0.5 L/min in the new direction. Ripple is the SD of flow about a 0.5 s moving mean over the middle 60% of the phase. All three barely change with airway configuration.')
sec_common+=fig('leak','Seal behaviour. Push and return gaps are piston volume minus flow meter volume, median over breaths 2 to 10 of three suites, with two breaths excluded for potentiometer spikes. Steady leak is the difference between potentiometer and flow meter rates. The return gap and the steady leak rise with airway suction, the signature of a pressure-driven seal leak.')
sec_common+=tab_title('Values for the placeholders in Section 3.3')+tbl(['Configuration','Steady seal leak by mode (%)','Stick-slip: expiration ripple SD (L/min)','Inspiration ripple SD (L/min)','Reversal stall (s)','Lag (s)'],rh)
sec_common+=fig('quasi_steady','Unsteady pressure against the steady fit read at the same window flow. Top panels: every port, preset and configuration, with equality dashed. Bottom panels: the difference at each port. The largest differences are '+m(pk[pk.phase=='insp']['diff'].abs().max())+' Pa on inspiration and '+m(pk[pk.phase=='exp']['diff'].abs().max())+' Pa on expiration, the latter at native p5, where the two-term fit itself misses the steady points by up to '+m(F[(F.dir=='out')&(F.cfg.str.contains('native'))].maxres.max())+' Pa.')
sec_common+=fig('repeatability','Repeatability of pressure at the peak window, all 224 combinations of port, preset, phase and configuration. Left: range of the three suite means. Right: breath-to-breath SD. The dashed line is the SDP811 uncertainty (0.1 Pa zero-point and 3% span). The only points above it are native nose-only preset 4 expiration at p2 to p6, with SD up to '+m(pk[(pk.cfg=='native_no')&(pk.preset==4)&(pk.phase=='exp')].sd.max())+' Pa at p5.')
sec_common+=tab_title('Steady fit coefficients, p = aQ + bQ|Q|, Q in L/min, p in Pa','steady_fits.csv')+tbl(['Configuration','Direction','Port','a','b','RMS residual (Pa)','Largest residual (Pa)'],fitrows,'dense scroll')
sec_common+=tab_title('SDP811 flow-through, seven sensors together, largest value per configuration','bleed.csv')+tbl(['Configuration','Largest port pressure (Pa)','Low bound (% of window flow)','High bound (% of window flow)'],bl)
sec_common+=f'<p class="note">Bounds use Sensirion\'s figures of 33 to 43 sccm at 100 Pa and 90 to 115 sccm at 500 Pa [37]: a linear fall to zero for the low bound, and the power law through the upper points for the high bound. Every port lies below 100 Pa, outside the published range. The high bound of {m(B.hip.max())}% sits inside the 1.5% flow uncertainty.</p>'
sec_common+=tab_title('Suites used and logging quality','data_quality.csv')+tbl(['Configuration','Start','T (°C)','Station P (hPa)','Rows','Lost rows','Short flow averages','Failed reads'],qt)
sec_common+='<p class="note">The nine failed reads in MAD_FullSuite_1 preset 1 are consecutive flow reads in breath 3, inside the fit span but outside the peak window. They are skipped in the fit, and pressure in that breath is unaffected. Board clock error is within ±1.6 ppm apart from NonMad_NoseOnly_3 preset 1, whose estimate of −999,327 ppm comes from a 91,154 s PC span in the logger\'s estimate. Row timing in that run is normal. An outlier scan of window flow, volume and pressure over all breaths, at six scaled median absolute deviations, flagged one value differing by 0.04 Pa, so no breath was excluded.</p>'
page_cfg=''
page_cfg+='<h3 class="band" id="native">5.1.1 Native airway results</h3>'+cfg_block('native_do','5.1.1.1','5.1.1.2').replace('Dual-orifice steady','Dual-orifice steady')+cfg_block('native_no','5.1.1.3','5.1.1.4')
page_cfg+='<h3 class="band" id="mad">5.1.2 MAD assisted airway results</h3>'+cfg_block('mad_do','5.1.2.1','5.1.2.2')+cfg_block('mad_no','5.1.2.3','5.1.2.4')
cmpq=lambda pre,ph='insp':(pv('native_no',pre,'p4','q_win',ph),pv('mad_no',pre,'p4','q_win',ph))
c=''
c+='<h3 id="dev">Effect of the device</h3>'
c+=fig('device_profile_insp_measured',f'Native and MAD pressure at the inspiratory peak window, as measured. Panel headings give native and MAD window flow. Bars show combined uncertainty. Port locations are approximated between the models, so the overall shape carries more weight than any one port.')
c+=fig('device_profile_insp_commonflow',f'The same comparison at a common flow. Each pressure is moved along its own configuration\'s steady fit to the native dual-orifice window flow for that preset: p + p_fit(Q_ref) − p_fit(Q_window). This keeps the measured unsteady value and removes only the flow difference.')
c+=fig('device_delta_insp',f'MAD minus native pressure on inspiration, as measured and at common flow. Positive values mean weaker suction with the MAD. Native nose-only ran furthest below target, so correcting flow raises the p4 change at preset 4 from {m(pv("mad_no",4,"p4")-pv("native_no",4,"p4"))} to {m(pv("mad_no",4,"p4","p_cf")-pv("native_no",4,"p4","p_cf"))} Pa. Bars combine both uncertainties in quadrature, which overstates them where the sensor span error is shared.')
c+=fig('device_profile_exp_measured','Native and MAD pressure at the expiratory peak window, as measured. Expiratory pressures are secondary in this project, and native p5 reads close to zero.')
c+=fig('device_profile_exp_commonflow','The same expiratory comparison at a common flow.')
c+=fig('device_delta_exp','MAD minus native pressure on expiration, as measured and at common flow.')
c+='<h3 id="route">Effect of breathing route</h3>'
c+=fig('route_profile_insp_measured','Dual-orifice and nose-only pressure at the inspiratory peak window for each model, as measured. Panel headings give dual-orifice and nose-only window flow.')
c+=fig('route_profile_insp_commonflow','The same route comparison at a common flow.')
c+=fig('route_delta_insp','Nose-only minus dual-orifice pressure on inspiration, as measured and at common flow. Blocking the mouth adds suction at every port, with the largest change at p4 in the native model.')
c+=fig('route_delta_exp','Nose-only minus dual-orifice pressure on expiration, as measured and at common flow.')
c+='<h3 id="norm">Normalised pressure, pressure drop and resistance</h3>'
c+=fig('cp_profile_insp',f'Inspiratory pressure normalised by inlet dynamic pressure, p / (½ρU²), using the 22 mm tube, window flow and the density of each suite. The profile shapes collapse, the magnitudes do not: native dual-orifice p1 moves from {m(pv("native_do",1,"p1","cp"),1)} at preset 1 to {m(pv("native_do",4,"p1","cp"),1)} at preset 4, because pressure at p1 scales with flow to a power of about 1.7 to 1.8, not 2.')
c+=fig('drop_resistance','Pressure drop between p7 and p1 and resistance Δp/Q against window flow. Bars combine the uncertainty of both ports and, for resistance, the 1.5% flow uncertainty.')
c+=tab_title('Window flow (L/min) | |p7 − p1| (Pa) | resistance (Pa per L/min), unsteady presets','drop_resistance.csv')+tbl(['Phase','Preset']+list(LAB.values()),dt,'dense')
# findings
f=[]
f.append(f'<b>All twelve suites are clean.</b> {q.rows.sum():,} rows, {q.lost.sum()} lost, {q.failed.sum()} failed reads. Between-suite range is at most {m(pk.range.max())} Pa at any port, against peak pressures up to {m(abs(pk.p.min()),1)} Pa.')
f.append(f'<b>The MAD reduces inspiratory suction from p1 to p5 at every preset, and the p7 to p1 drop falls in both routes.</b> At preset 4 the drop falls from {m(d("native_do",4).dp,1)} to {m(d("mad_do",4).dp,1)} Pa dual-orifice ({m(red("mad_do","native_do",4),0)}%) and from {m(d("native_no",4).dp,1)} to {m(d("mad_no",4).dp,1)} Pa nose-only ({m(red("mad_no","native_no",4),0)}%). Suction is slightly stronger with the MAD at p6 and p7.')
f.append(f'<b>Native nose-only has a distinct profile.</b> Suction peaks at p4 ({m(pv("native_no",4,"p4"),1)} Pa at preset 4) with a sharp step from p5 ({m(pv("native_no",4,"p5"),1)} Pa) and partial recovery towards p1 ({m(pv("native_no",4,"p1"),1)} Pa). It is the only configuration where p4 reads the strongest suction, at every preset.')
f.append(f'<b>Breathing follows the steady relationship.</b> Breathing and steady fit differ by at most {m(pk[pk.phase=="insp"]["diff"].abs().max())} Pa on inspiration and {m(pk[pk.phase=="exp"]["diff"].abs().max())} Pa on expiration.')
f.append(f'<b>Delivered flow depends on resistance.</b> Native nose-only falls {m(abs(g[(g.cfg=="native_no")&(g.preset==4)&(g.phase=="insp")].err.iloc[0]))}% below target at preset 4, and presets 2 to 4 inspiration lie outside the 1.5% span. Preset 1 expiration is over target by 2.1 to 2.6% in every configuration.')
find=''.join(f'<li>{x}</li>' for x in f)
ed=[
('3.3','Replace the X placeholders with the table under Rig performance. Seal leak: 1.8 to 7.8% by steady mode. Stick-slip as expiration ripple SD: 1.18 to 1.75 L/min. Reversal stall: 0.33 to 1.41 s. Inspiration ripple SD: 0.91 to 1.94 L/min.'),
('4.2.2',f'Quotes lag of 0.04 to 0.16 s and stall of 0.24 to 1.05 s from the two-suite page. Native three-suite values are lag {m(g[g.cfg=="native_do"].lag.min())} to {m(g[g.cfg=="native_do"].lag.max())} s and stall {m(g[g.cfg=="native_do"].stall.min())} to {m(g[g.cfg=="native_do"].stall.max())} s. The old stall definition could not be reproduced exactly, so the new figure needs the definition stated in Section 4.4 (see Figure R{FIG["rig_behaviour"]}).'),
('4.3.2','Quotes 16 tares on 29 October with flow zeros of 0.004 to 0.015 slm. The 96 tares used here give 0.002 to 0.019 slm, and pressure zeros within ±0.05 Pa. The largest flow zero is 0.19% of the lowest target flow.'),
('4.1, 4.4','Steady modes run ten strokes, not ten breaths: five out and five in. With the first out stroke excluded, each suite gives four out and five in. "Averaged over the breaths after the first" should read strokes.'),
('4.4',f'The example of 29.36 against 30.01 L/min at preset 4 inspiration comes from two suites. Three suites give {m(pv("native_do",4,"p1","q_win"))} against {m(g[(g.cfg=="native_do")&(g.preset==4)&(g.phase=="insp")].fit.iloc[0])} L/min.'),
('4.4','Add a sentence for the common-flow correction, which this page shows beside the as-measured figures. It also needs Q_ref stated: the native dual-orifice window flow for each preset.'),
('4.4','Direct steady comparison with CFD at 10, 15, 20 and 30 L/min carries a flow gap in native nose-only (s4 in at 29.02 L/min). Read the measured steady fit at the CFD flow, or give the researchers the measured steady flows.'),
('5.1 and 5.2','Section 5.1 has no home for comparisons between configurations. They are placed under 5.3 here. CFD has not been run, so Sections 5.2 and 5.3 agreement metrics remain empty.')]
edits=''.join(f'<tr><td class="sec">{a}</td><td>{html.escape(t)}</td></tr>' for a,t in ed)
csvs=['flow_delivery.csv','peak_pressures.csv','steady_pressures.csv','steady_fits.csv','drop_resistance.csv','bleed.csv','data_quality.csv','volume_gaps.csv','per_breath.csv']
with zipfile.ZipFile('../final_report_figures_and_tables.zip','w',zipfile.ZIP_DEFLATED) as z:
    import glob,os
    for p in sorted(glob.glob('../figures/*.png')):z.write(p,'figures/'+os.path.basename(p))
    for c_ in csvs:z.write(T+c_,'tables/'+c_)
nf=N['n']
page=f'''<title>Final Report Analysis</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Serif:wght@500&display=swap">
<style>
/* Layout: one reading column in report section order, figures on white plates so they match how they will sit in the report */
:root{{--bg:#f4f6f8;--fg:#1b2127;--mute:#5a6570;--line:#d5dbe1;--panel:#ffffff;--accent:#1c5cab;--flag:#b5400f;--flagbg:#fbe9df;--sans:"IBM Plex Sans",system-ui,-apple-system,"Segoe UI",sans-serif;--serif:"IBM Plex Serif",Georgia,serif}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#14181c;--fg:#e6eaee;--mute:#a0abb6;--line:#333b43;--panel:#1c2126;--accent:#7fb2ee;--flag:#f0a07a;--flagbg:#3a2419;color-scheme:dark}}}}
:root[data-theme="dark"]{{--bg:#14181c;--fg:#e6eaee;--mute:#a0abb6;--line:#333b43;--panel:#1c2126;--accent:#7fb2ee;--flag:#f0a07a;--flagbg:#3a2419;color-scheme:dark}}
body{{background:var(--bg);color:var(--fg);font:15px/1.55 var(--sans)}}
.wrap{{max-width:60rem;margin:0 auto;padding-block:2rem 4rem;padding-left:max(16px,env(safe-area-inset-left,0px));padding-right:max(16px,env(safe-area-inset-right,0px));display:flex;flex-direction:column;gap:1.5rem}}
h1,h2{{font-family:var(--serif);font-weight:500;text-wrap:balance;margin:0}}
h1{{font-size:1.9rem;line-height:1.2}}
h2{{font-size:1.4rem;border-top:2px solid var(--fg);padding-top:.7rem;margin-top:1.5rem}}
h3{{font-size:1.02rem;font-weight:600;margin:1rem 0 0}}
h3.band{{font-size:1.15rem;border-bottom:1px solid var(--line);padding-bottom:.35rem;margin-top:1.75rem}}
.sub{{color:var(--mute);margin:.35rem 0 0;max-width:46rem}}
ul.find{{margin:0;padding-left:1.15rem;display:flex;flex-direction:column;gap:.5rem;max-width:48rem}}
ul.find b{{font-weight:600}}
figure{{margin:0;display:flex;flex-direction:column;gap:.5rem;min-width:0}}
.plate{{background:#fff;border:1px solid var(--line);padding:.5rem}}
.plate img{{display:block;width:100%;height:auto}}
figcaption{{font-size:.875rem;color:var(--mute);max-width:52rem}}
.fn{{font-weight:600;color:var(--fg);font-variant-numeric:tabular-nums}}
button.sv{{font:500 .75rem var(--sans);color:var(--accent);background:none;border:1px solid var(--line);border-radius:3px;padding:.1rem .45rem;margin-left:.5rem;cursor:pointer;white-space:nowrap}}
button.sv:hover{{border-color:var(--accent)}}
button.sv:focus-visible{{outline:2px solid var(--accent);outline-offset:1px}}
.tt{{font-size:.875rem;font-weight:600;margin:.4rem 0 -.8rem}}
.tw{{overflow-x:auto;border:1px solid var(--line);background:var(--panel)}}
.tw:has(table.scroll){{max-height:22rem;overflow-y:auto}}
table{{border-collapse:collapse;width:100%;font-size:.78rem;font-variant-numeric:tabular-nums}}
th,td{{padding:.3rem .55rem;text-align:right;white-space:nowrap;border-bottom:1px solid var(--line)}}
th{{font-weight:500;color:var(--mute);background:var(--panel);position:sticky;top:0}}
th:first-child,td:first-child{{text-align:left}}
tbody tr:last-child td{{border-bottom:0}}
.flag{{color:var(--flag);background:var(--flagbg);padding:0 .25rem;font-weight:500}}
.note{{font-size:.875rem;color:var(--mute);margin:0;max-width:52rem}}
table.edits td{{white-space:normal;text-align:left;vertical-align:top;font-size:.875rem;font-variant-numeric:normal}}
table.edits td.sec{{white-space:nowrap;font-weight:500}}
nav{{font-size:.875rem;display:flex;flex-wrap:wrap;gap:.25rem 1rem}}
nav a{{color:var(--accent)}}
.pending{{border:1px dashed var(--line);padding:.8rem 1rem;color:var(--mute);font-size:.9rem;max-width:52rem}}
.bar{{display:flex;flex-wrap:wrap;align-items:center;gap:.6rem}}
#st{{font-size:.8rem;color:var(--mute)}}
</style>
<div class="wrap">
<header>
<h1>Final Report Analysis</h1>
<p class="sub">Measured results for NEF4101 and NEF4201, built to Section 4.4 of the Capstone Final Report from twelve full suites on 29 and 30 October 2026: native and MAD models, dual-orifice and nose-only, three suites each. Inspiratory peak flows of 10, 15, 20 and 30 L/min. First breath or stroke excluded.</p>
</header>
<div class="bar"><span id="st"></span></div>
<nav aria-label="Sections"><a href="#findings">Findings</a><a href="#edits">Report edits</a><a href="#rig">Rig performance</a><a href="#native">5.1.1 Native</a><a href="#mad">5.1.2 MAD</a><a href="#cfd">5.2 CFD</a><a href="#cmp">5.3 Comparison</a></nav>
<section id="findings" style="display:flex;flex-direction:column;gap:.8rem"><h2>Findings</h2><ul class="find">{find}</ul></section>
<section id="edits" style="display:flex;flex-direction:column;gap:.8rem"><h2>Report text these results affect</h2><div class="tw"><table class="edits"><tbody>{edits}</tbody></table></div></section>
<section id="rig" style="display:flex;flex-direction:column;gap:1.5rem"><h2>Rig performance and data quality</h2><p class="note">Supports Sections 3.3, 4.2.2 and 4.4 and the opening of Section 5.1. Uncertainty on all pressure figures combines 0.1 Pa zero-point and 3% span with the 1.5% flow uncertainty, propagated through the local exponent of the steady fit.</p>{sec_common}</section>
<section style="display:flex;flex-direction:column;gap:1.5rem"><h2>5.1 Experimental results</h2>{page_cfg}</section>
<section id="cfd" style="display:flex;flex-direction:column;gap:.8rem"><h2>5.2 CFD simulation results</h2><p class="pending">No figures yet. CFD has not been run, so the overlays, Δp, mean absolute error, mean bias and regression metrics in Section 4.4 wait on its results. The measured values they need are in the CSV tables.</p></section>
<section id="cmp" style="display:flex;flex-direction:column;gap:1.5rem"><h2>5.3 Comparative analysis: measured part</h2><p class="note">Comparison figures come in pairs: as measured, with each pressure at its own window flow, and at common flow. Differences at one port should not be read as the effect of the device alone, because port locations are approximated between models and the CT scans differ in posture and segmentation.</p>{c}</section>
</div>
<script>
(async()=>{{
const st=document.getElementById('st');
let dl=null;try{{dl=await window.claude?.use?.('downloads')}}catch(e){{}}
if(!dl){{document.querySelectorAll('button.sv').forEach(b=>b.hidden=true);st.textContent='Saving is unavailable in this view. Right-click a figure to save it.';return}}
document.addEventListener('click',async ev=>{{
const b=ev.target.closest('button.sv');if(!b)return;
const t=b.textContent;b.disabled=true;b.textContent='Preparing';
try{{const r=await fetch(b.dataset.f);const buf=await r.arrayBuffer();await dl.save({{filename:b.dataset.n,data:new Uint8Array(buf)}});b.textContent=t}}
catch(e){{b.textContent=(e&&e.code==='declined')?t:'Save failed'}}
finally{{b.disabled=false}}
}});
}})();
</script>
'''
open('../final_report_analysis.html','w').write(page)
print(nf,len(page),os.path.getsize('../final_report_figures_and_tables.zip')/1e6)
