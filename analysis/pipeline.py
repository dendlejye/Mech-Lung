from core import *
V=700.;DT=.01
def stall_time(q):
    m=pd.Series(q).rolling(10,center=True,min_periods=1).mean().values
    return np.argmax(m>.5)*DT
def fit_phase(t,q,T):
    lag=0.
    for _ in range(12):
        fi=(t>=lag+.2*T)&(t<=lag+.8*T)&np.isfinite(q)
        X=np.c_[np.sin(np.pi*t[fi]/T),np.cos(np.pi*t[fi]/T)]
        a,c=np.linalg.lstsq(X,q[fi],rcond=None)[0];lag=-np.arctan2(c,a)*T/np.pi
    return np.hypot(a,c),lag
br=[];wav={};st=[];vol=[];qual=[];stall=[]
for cfg,suites in SUITES.items():
  for si,suite in enumerate(suites,1):
    rr=runs(suite)
    for k in '1234':
      j,s=load(rr[k]);pre=int(k);qt=Qt[pre];c=j['config']
      T=c['period_ms'][pre-1]/1000;Te=np.pi*V/(2*qt*1000/60)*2;Ti=T-Te
      bl=breaths(s);assert len(bl)==10
      for bi,b in enumerate(bl,1):
        t=np.arange(len(b))*DT;q=b.q.values
        for ph,t0,TT,sg,tgt in (('exp',0,Te,1,qt/2),('insp',Te,Ti,-1,qt)):
          tp=t-t0;A,lag=fit_phase(tp,sg*q,TT)
          w=np.abs(tp-TT/2)<=.05*TT;i=(tp>=0)&(tp<TT);mid=(tp>=.2*TT)&(tp<=.8*TT)
          ma=pd.Series(q).rolling(51,center=True,min_periods=1).mean().values
          rec=dict(cfg=cfg,suite=si,preset=pre,breath=bi,phase=ph,target=tgt,fit=A,lag=lag,win=sg*np.nanmean(q[w]),rawmax=np.nanmax(sg*q[i]),ripple=np.nanstd((q-ma)[mid]),stall=stall_time(sg*q[i]),vol=np.nansum(sg*q[i])*DT/60*1000,T=j['temp_c'],Pst=j['p_station_hpa'],Te=Te,Ti=Ti)
          for p in P: rec[p]=b[p].values[w].mean()
          br.append(rec)
      for mi,m in enumerate(j['breaths'],1):
        vol.append(dict(cfg=cfg,suite=si,preset=pre,breath=mi,**{x:m[x] for x in ('push_leak_ml','return_leak_ml','net_ml','push_leak_pct')}))
      n=min(len(b) for b in bl[1:])
      wav[(cfg,si,pre)]={key:np.array([b[key].values[:n] for b in bl[1:]]) for key in ['q']+P}
      wav[(cfg,si,pre,'meta')]=dict(Te=Te,Ti=Ti,T=T,qt=qt)
      qual.append(dict(cfg=cfg,suite=suite,item=k,rows=j['rows'],lost=int((np.diff(s.tick)-1).sum()),short=int((s.sfm_n<4).sum()),failed=sum(b['failed_reads'] for b in j['breaths']),clock=j['clock']['ppm'],T=j['temp_c'],Pst=j['p_station_hpa'],tflow=j['tare']['flow_slm'],start=j['start'],tpmax=max(abs(j['tare'][f'{p}_Pa']) for p in P)))
    for k in ['s1','s2','s3','s4']:
      j,s=load(rr[k]);pre=int(k[1]);qt=Qt[pre]
      for m in j['strokes']:
        rec=dict(cfg=cfg,suite=si,mode=pre,breath=m['breath'],dir=m['dir'],first=m['first'],target=qt,qmean=m['mean_flow_Lmin'],leak_pct=m['leak_pct'],T=j['temp_c'],Pst=j['p_station_hpa'])
        for i,p in enumerate(P): rec[p]=m['p_mean_Pa'][i]
        st.append(rec)
      qual.append(dict(cfg=cfg,suite=suite,item=k,rows=j['rows'],lost=int((np.diff(s.tick)-1).sum()),short=int((s.sfm_n<4).sum()),failed=sum(m['failed_reads'] for m in j['strokes']),clock=j['clock']['ppm'],T=j['temp_c'],Pst=j['p_station_hpa'],tflow=j['tare']['flow_slm'],start=j['start'],tpmax=max(abs(j['tare'][f'{p}_Pa']) for p in P)))
br=pd.DataFrame(br);st=pd.DataFrame(st);vol=pd.DataFrame(vol);qual=pd.DataFrame(qual)
for n,d in (('br',br),('st',st),('vol',vol),('qual',qual)): d.to_pickle(n+'.pkl')
import pickle;pickle.dump(wav,open('wav.pkl','wb'))
