from core import *
pd.set_option('display.width',240,'display.max_rows',200,'display.max_columns',40)
b=pd.read_pickle('b.pkl');pk=pd.read_pickle('pk2.pkl');sr=pd.read_pickle('sr.pkl');dr=pd.read_pickle('drop.pkl');st=pd.read_pickle('st.pkl');q=pd.read_pickle('qual.pkl');lk=pd.read_pickle('leak.pkl');F=pd.read_pickle('fits.pkl')
print(q[['rows','lost','short','failed']].sum(), q.tflow.agg(['min','max']),q.tpmax.max())
print(q.groupby('cfg')[['T','Pst']].agg(['min','max']))
x=q[q.clock.abs()<1000].clock;print('clock',x.min(),x.max())
print(pk.assign(ad=pk['diff'].abs()).groupby('phase').ad.max(), pk.range.max(), sr.range.max(), pk.sd.max(), pk.assign(r=pk.range/pk.u).groupby('phase').r.max())
g=b.groupby(['cfg','preset','phase'])[['fit','target','lag','stall','ripple','win','rawmax','vol']].mean().reset_index();g['err']=(g.fit/g.target-1)*100
print(g.groupby('phase')[['err','lag','stall','ripple']].agg(['min','max']).round(2));print(g[g.err.abs()>1.5][['cfg','preset','phase','fit','err']].round(2))
print(g.groupby('cfg')[['lag','stall','ripple']].agg(['min','max']).round(2))
e=st[~st['first']].groupby(['cfg','mode']).leak_pct.mean().unstack().round(2);print(e)
s2=st[~st['first']].groupby(['cfg','mode','dir']).agg(q=('qmean',lambda x:x.abs().mean()),t=('target','mean'));s2['err']=(s2.q/s2.t-1)*100;print(s2.err.unstack(['mode','dir']).round(2))
print(lk.round(1))
d=dr[dr.kind=='breathing'];print(d.pivot_table(index=['phase','level'],columns='cfg',values='dp').round(2));print(d.pivot_table(index=['phase','level'],columns='cfg',values='R').round(3))
print('Cp');print(pk[pk.phase=='insp'].pivot_table(index='cfg',columns='preset',values='cp',aggfunc=lambda x:x.iloc[0]).round(1))
print('p1 exponent');
for c in pk.cfg.unique():
    d=pk[(pk.cfg==c)&(pk.phase=='insp')&(pk.port=='p1')];print(c,np.polyfit(np.log(d.q_win),np.log(-d.p),1)[0].round(2))
    y=pk[(pk.cfg==c)&(pk.phase=='insp')&(pk.port=='p4')];print('  p4',np.polyfit(np.log(y.q_win),np.log(-y.p),1)[0].round(2))
print(F.groupby(['cfg','dir'])[['maxres']].max().round(2))
# bleed
rows=[]
for (c,pr,ph),G in pk.groupby(['cfg','preset','phase']):
    ps=G.p.abs().values;n=np.log(115/43)/np.log(5);hi=(43*(ps/100)**n).sum()/1000;lo=(33*ps/100).sum()/1000;qq=G.q_win.iloc[0]
    rows.append(dict(cfg=c,preset=pr,phase=ph,q=qq,maxp=ps.max(),lo=lo,hi=hi,lop=lo/qq*100,hip=hi/qq*100))
B=pd.DataFrame(rows);B.to_pickle('bleed.pkl');print(B.sort_values('hip').tail(3).round(3));print(B.hip.max(),B.lop.max())
# device effect
for r in ('do','no'):
  for pre in (1,4):
    a=pk[(pk.cfg==f'mad_{r}')&(pk.preset==pre)&(pk.phase=='insp')].set_index('port');n=pk[(pk.cfg==f'native_{r}')&(pk.preset==pre)&(pk.phase=='insp')].set_index('port')
    print(r,pre,(a.p-n.p).round(2).values,(a.p_cf-n.p_cf).round(2).values)
