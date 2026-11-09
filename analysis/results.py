from core import *
import pickle
br=pd.read_pickle('br.pkl');st=pd.read_pickle('st.pkl');vol=pd.read_pickle('vol.pkl');qual=pd.read_pickle('qual.pkl')
wav=pickle.load(open('wav.pkl','rb'))
b=br[br.breath>1].copy();s=st[~st['first']].copy()
A=np.pi*.022**2/4
def rho(T,P): return P*100/(287.05*(T+273.15))
# steady fit per cfg/port/dir, using stroke means pooled per mode
sm=s.groupby(['cfg','mode','dir'])[['qmean']+P].mean().reset_index()
fits={}
for (cfg,d),g in sm.groupby(['cfg','dir']):
    for p in P:
        Q=g.qmean.values;X=np.c_[Q,Q*np.abs(Q)];c,*_=np.linalg.lstsq(X,g[p].values,rcond=None)
        pred=X@c;res=g[p].values-pred
        fits[(cfg,d,p)]=dict(a=c[0],b=c[1],rmse=np.sqrt((res**2).mean()),maxres=np.abs(res).max())
fdf=pd.DataFrame(fits).T;fdf.index.names=['cfg','dir','port'];fdf=fdf.reset_index()
def pfit(cfg,d,p,Q):
    f=fits[(cfg,d,p)];return f['a']*Q+f['b']*Q*abs(Q)
def localn(cfg,d,p,Q):
    f=fits[(cfg,d,p)];Q=abs(Q);v=f['a']*Q+f['b']*Q*Q
    # n=dlnp/dlnQ
    return (f['a']*Q+2*f['b']*Q*Q)/v if abs(v)>1e-6 else np.nan
rows=[]
for (cfg,pre,ph),g in b.groupby(['cfg','preset','phase']):
    Qsgn=1 if ph=='exp' else -1;d='out' if ph=='exp' else 'in'
    qw=g.win.mean();T=g['T'].mean();Pst=g.Pst.mean();r=rho(T,Pst);U=qw/60000/A;dyn=.5*r*U*U
    for p in P:
        m=g[p].mean();sd=g[p].std(ddof=1)
        su=g.groupby('suite')[p].mean();wsd=np.sqrt((g.groupby('suite')[p].var(ddof=1)).mean())
        ps=pfit(cfg,d,p,Qsgn*qw);n=localn(cfg,d,p,qw)
        us=np.hypot(.1,.03*abs(m));uq=abs(n)*.015*abs(m) if np.isfinite(n) else 0;u=np.hypot(us,uq)
        rows.append(dict(cfg=cfg,preset=pre,phase=ph,port=p,p=m,sd=sd,s1=su[1],s2=su[2],s3=su[3],range=su.max()-su.min(),within=wsd,q_win=qw,p_steady=ps,diff=m-ps,n=n,u_sensor=us,u_flow=uq,u=u,cp=m/dyn,dyn=dyn))
pk=pd.DataFrame(rows)
sr=[]
for (cfg,mode,d),g in s.groupby(['cfg','mode','dir']):
    q=g.qmean.mean();T=g['T'].mean();Pst=g.Pst.mean();r=rho(T,Pst);U=abs(q)/60000/A;dyn=.5*r*U*U
    for p in P:
        m=g[p].mean();su=g.groupby('suite')[p].mean();n=localn(cfg,d,p,q)
        us=np.hypot(.1,.03*abs(m));uq=abs(n)*.015*abs(m) if np.isfinite(n) else 0
        sr.append(dict(cfg=cfg,mode=mode,dir=d,port=p,p=m,sd=g[p].std(ddof=1),s1=su[1],s2=su[2],s3=su[3],range=su.max()-su.min(),q=q,target=g.target.mean(),leak=g.leak_pct.mean(),n_strokes=len(g),u=np.hypot(us,uq),cp=m/dyn,fit=pfit(cfg,d,p,q)))
sr=pd.DataFrame(sr)
pk.to_pickle('pk.pkl');sr.to_pickle('sr.pkl');fdf.to_pickle('fits.pkl');b.to_pickle('b.pkl')
if __name__=='__main__':
    pd.set_option('display.width',250,'display.max_rows',400,'display.max_columns',40)
    print('fit resid per cfg dir');print(fdf.groupby(['cfg','dir'])[['rmse','maxres']].max().round(3))
    print('QS |diff| Pa max per cfg/phase');print(pk.assign(ad=pk['diff'].abs()).groupby(['cfg','phase']).ad.agg(['max','mean']).round(2))
    print(pk.assign(ad=pk['diff'].abs()).sort_values('ad',ascending=False).head(8)[['cfg','preset','phase','port','p','p_steady','diff','q_win']].round(2))
    print('range between suites');print(pk.groupby(['cfg','phase'])[['range','sd','within']].max().round(3));print(pk.sort_values('range',ascending=False).head(10)[['cfg','preset','phase','port','p','s1','s2','s3','range','sd','u']].round(2))
    print(sr.sort_values('range',ascending=False).head(10)[['cfg','mode','dir','port','p','s1','s2','s3','range','sd','u']].round(2))
    # flow per suite
    f=b.groupby(['cfg','suite','preset','phase'])[['fit','win']].mean().unstack('suite').round(2);print(f)
    print('drop');
    d=pk.pivot_table(index=['cfg','preset','phase'],columns='port',values='p');print((d.p7-d.p1).unstack('phase').round(2).unstack('cfg'))
    print('Cp insp preset 4');print(pk[(pk.phase=='insp')].pivot_table(index=['cfg','preset'],columns='port',values='cp').round(2))
    print('u max',pk.u.max(),pk.groupby('phase').u.max())
    print(vol.groupby(['cfg','preset'])[['push_leak_ml','return_leak_ml','net_ml','push_leak_pct']].mean().round(1))
    print(s.groupby(['cfg','dir']).leak_pct.agg(['min','max']).round(2))
