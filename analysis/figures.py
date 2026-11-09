from core import *
import pickle,json
import matplotlib as mpl;mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
b=pd.read_pickle('b.pkl');pk=pd.read_pickle('pk.pkl');sr=pd.read_pickle('sr.pkl');fits=pd.read_pickle('fits.pkl');vol=pd.read_pickle('vol.pkl');st=pd.read_pickle('st.pkl')
wav=pickle.load(open('wav.pkl','rb'))
FD='../figures/'
BLUE,ORANGE='#2a78d6','#eb6834';INK='#1f1f1d';MUTE='#6b6a66';GRID='#e6e5e1'
RAMP={1:'#86b6ef',2:'#5598e7',3:'#256abf',4:'#104281'}
RAMP_O={1:'#f4b48f',2:'#ee8a5c',3:'#d95926',4:'#a33f15'}
LAB={'native_do':'Native, dual-orifice','native_no':'Native, nose-only','mad_do':'MAD, dual-orifice','mad_no':'MAD, nose-only'}
ST={'native_do':dict(c=BLUE,ls='-',m='o',mf=BLUE),'native_no':dict(c=BLUE,ls='--',m='s',mf='white'),'mad_do':dict(c=ORANGE,ls='-',m='^',mf=ORANGE),'mad_no':dict(c=ORANGE,ls='--',m='D',mf='white')}
CF=list(LAB)
mpl.rcParams.update({'font.family':'Inter','font.size':8,'axes.edgecolor':'#8a8984','axes.linewidth':.6,'axes.labelcolor':INK,'xtick.color':MUTE,'ytick.color':MUTE,'text.color':INK,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.color':GRID,'grid.linewidth':.5,'axes.axisbelow':True,'legend.frameon':False,'legend.fontsize':7.5,'axes.titlesize':8.5,'axes.titleweight':'medium','axes.titlelocation':'left','xtick.major.size':2.5,'ytick.major.size':2.5,'xtick.major.width':.5,'ytick.major.width':.5,'savefig.dpi':300,'figure.dpi':100,'savefig.facecolor':'white','lines.linewidth':1.3})
W=6.7
def save(f,name):
    f.savefig(FD+name+'.png');plt.close(f)
def zero(ax): ax.axhline(0,color='#8a8984',lw=.6,zorder=1)
PH={'insp':'Inspiration','exp':'Expiration'}
# ---------- config figures
def flow_fig(cfg):
    f,ax=plt.subplots(2,2,figsize=(W,3.8),constrained_layout=True)
    for pre,a in zip((1,2,3,4),ax.flat):
        arr=[wav[(cfg,s,pre)]['q'] for s in (1,2,3)];n=min(x.shape[1] for x in arr)
        A=np.vstack([x[:,:n] for x in arr]);m=np.nanmean(A,0);sd=np.nanstd(A,0,ddof=1);t=np.arange(n)*.01
        M=wav[(cfg,1,pre,'meta')];Te,Ti,qt=M['Te'],M['Ti'],M['qt']
        ideal=np.where(t<Te,qt/2*np.sin(np.pi*t/Te),-qt*np.sin(np.pi*(t-Te)/Ti))
        zero(a)
        a.fill_between(t,m-sd,m+sd,color=BLUE if 'native' in cfg else ORANGE,alpha=.22,lw=0,label='±1 SD')
        a.plot(t,m,color=BLUE if 'native' in cfg else ORANGE,lw=.8,label='Measured mean')
        a.plot(t,ideal,color=INK,lw=1,ls=(0,(4,2)),label='Ideal half-sine')
        c=Te+Ti/2;a.axvspan(c-.05*Ti,c+.05*Ti,color='#b9b8b2',alpha=.45,lw=0,label='Peak window');c=Te/2;a.axvspan(c-.05*Te,c+.05*Te,color='#b9b8b2',alpha=.45,lw=0)
        a.axvline(Te,color=MUTE,lw=.5,ls=':')
        a.set_title(f'Preset {pre}, {qt:.0f} L/min');a.set_xlim(0,t[-1])
        if pre>2:a.set_xlabel('Time from start of breath (s)')
        if pre%2:a.set_ylabel('Flow (L/min)')
    h,l=ax.flat[0].get_legend_handles_labels();f.legend(h,l,ncol=4,loc='outside lower center')
    save(f,f'unsteady_flow_{cfg}')
def pwave_fig(cfg):
    f,ax=plt.subplots(2,4,figsize=(W,3.5),sharex=True,sharey=True,constrained_layout=True)
    ramp=RAMP if 'native' in cfg else RAMP_O
    ax=ax.flat
    for i,p in enumerate(P):
        a=ax[i];zero(a)
        for pre in (1,2,3,4):
            arr=[wav[(cfg,s,pre)][p] for s in (1,2,3)];n=min(x.shape[1] for x in arr)
            A=np.vstack([x[:,:n] for x in arr]);m=A.mean(0)
            m=pd.Series(m).rolling(5,center=True,min_periods=1).mean().values
            a.plot(np.arange(n)/n,m,color=ramp[pre],lw=.9,label=f'Preset {pre}, {Qt[pre]:.0f} L/min')
        a.axvline(2/3,color=MUTE,lw=.5,ls=':')
        a.set_title(p+(' (nearest trachea)' if p=='p1' else ' (furthest)' if p=='p7' else ''),fontsize=7.5)
        a.tick_params(labelbottom=True);a.set_xlim(0,1);a.set_xticks([0,1/3,2/3,1],['0','1/3','2/3','1'])
    a=ax[7];a.axis('off')
    h,l=ax[0].get_legend_handles_labels();a.legend(h,l,loc='center',title='Mean of 27 breaths',title_fontsize=7.5)
    for i in (0,4):ax[i].set_ylabel('Gauge pressure (Pa)')
    f.supxlabel('Fraction of breath (expiration before 2/3, inspiration after)',fontsize=8)
    save(f,f'unsteady_pressure_breath_{cfg}')
def prof(a,df,color,ls='-',marker='o',mf=None,label=None,yk='p',uk='u',lw=1.2):
    df=df.set_index('port').loc[P];x=np.arange(7)
    a.errorbar(x,df[yk],yerr=df[uk],color=color,ls=ls,marker=marker,ms=3.6,mfc=mf or color,mew=.8,lw=lw,capsize=1.5,elinewidth=.7,label=label)
def setports(a):
    a.set_xticks(range(7),P);a.set_xlim(-.3,6.3)
def uprof_fig(cfg):
    f,ax=plt.subplots(1,2,figsize=(W,3.3),constrained_layout=True)
    ramp=RAMP if 'native' in cfg else RAMP_O
    for a,ph in zip(ax,('insp','exp')):
        zero(a)
        for pre in (4,3,2,1):
            d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase==ph)]
            q=d.q_win.iloc[0];prof(a,d,ramp[pre],label=f'Preset {pre}, {q:.2f} L/min')
        setports(a);a.set_title(PH[ph]+' at peak window');a.set_xlabel('Port (trachea to nose and mouth)');a.legend(loc='upper center',bbox_to_anchor=(.5,-.17),ncol=2,fontsize=7)
    ax[0].set_ylabel('Gauge pressure (Pa)')
    save(f,f'unsteady_profile_{cfg}')
def sprof_fig(cfg):
    f,ax=plt.subplots(1,2,figsize=(W,3.3),constrained_layout=True)
    ramp=RAMP if 'native' in cfg else RAMP_O
    for a,d in zip(ax,('in','out')):
        zero(a)
        for m in (4,3,2,1):
            x=sr[(sr.cfg==cfg)&(sr['mode']==m)&(sr.dir==d)];q=abs(x.q.iloc[0]);prof(a,x,ramp[m],label=f's{m}, {q:.2f} L/min')
        setports(a);a.set_title({'in':'Inspiratory stroke','out':'Expiratory stroke'}[d]);a.set_xlabel('Port (trachea to nose and mouth)');a.legend(loc='upper center',bbox_to_anchor=(.5,-.17),ncol=2,fontsize=7)
    ax[0].set_ylabel('Gauge pressure (Pa)')
    save(f,f'steady_profile_{cfg}')
def pq_fig(cfg):
    f,ax=plt.subplots(2,4,figsize=(W,3.5),sharex=True,sharey=True,constrained_layout=True)
    col=BLUE if 'native' in cfg else ORANGE;ax=ax.flat
    for i,p in enumerate(P):
        a=ax[i];zero(a);a.axvline(0,color='#8a8984',lw=.6,zorder=1)
        for d,sg in (('in',-1),('out',1)):
            F=fits[(fits.cfg==cfg)&(fits.dir==d)&(fits.port==p)].iloc[0]
            q=np.linspace(0,31,60)*sg;a.plot(q,F.a*q+F.b*q*np.abs(q),color=INK,lw=.8,label='Steady fit' if d=='in' else None)
            x=sr[(sr.cfg==cfg)&(sr.dir==d)&(sr.port==p)];a.plot(x.q,x.p,'o',color=col,ms=3.8,label='Steady mode' if d=='in' else None)
            ph={'in':'insp','out':'exp'}[d]
            y=pk[(pk.cfg==cfg)&(pk.phase==ph)&(pk.port==p)];a.plot(sg*y.q_win,y.p,'o',mfc='white',mec=INK,ms=3.4,mew=.7,label='Unsteady, peak window' if d=='in' else None)
        a.set_title(p,fontsize=7.5);a.tick_params(labelbottom=True)
    a=ax[7];a.axis('off');h,l=ax[0].get_legend_handles_labels();a.legend(h,l,loc='center')
    for i in (0,4):ax[i].set_ylabel('Gauge pressure (Pa)')
    f.supxlabel('Flow (L/min), positive out of cylinder (expiration)',fontsize=8)
    save(f,f'steady_pq_{cfg}')
for c in CF:
    flow_fig(c);pwave_fig(c);uprof_fig(c);sprof_fig(c);pq_fig(c)
# ---------- common flow frame
ref=pk[(pk.cfg=='native_do')].groupby(['preset','phase']).q_win.first().to_dict()
def fitval(cfg,d,p,Q):
    F=fits[(fits.cfg==cfg)&(fits.dir==d)&(fits.port==p)].iloc[0];return F.a*Q+F.b*Q*abs(Q)
pk['p_cf']=[r.p+fitval(r.cfg,{'insp':'in','exp':'out'}[r.phase],r.port,({'insp':-1,'exp':1}[r.phase])*ref[(r.preset,r.phase)])-fitval(r.cfg,{'insp':'in','exp':'out'}[r.phase],r.port,({'insp':-1,'exp':1}[r.phase])*r.q_win) for r in pk.itertuples()]
pk['q_ref']=[ref[(r.preset,r.phase)] for r in pk.itertuples()]
pk.to_pickle('pk2.pkl')
def cmp_prof(ph,key,name,yl):
    f,ax=plt.subplots(2,4,figsize=(W,3.9),sharex=True,constrained_layout=True)
    for r,route in enumerate(('do','no')):
        for k,pre in enumerate((1,2,3,4)):
            a=ax[r,k];zero(a)
            for cfg in (f'native_{route}',f'mad_{route}'):
                d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase==ph)]
                s=ST[cfg];prof(a,d,s['c'],s['ls'],s['m'],s['mf'],yk=key,label=cfg.split('_')[0].replace('native','Native').replace('mad','MAD'),uk='u')
                qq=d.q_win.iloc[0] if key=='p' else d.q_ref.iloc[0]
            if key=='p':
                qa=pk[(pk.cfg==f'native_{route}')&(pk.preset==pre)&(pk.phase==ph)].q_win.iloc[0];qb=pk[(pk.cfg==f'mad_{route}')&(pk.preset==pre)&(pk.phase==ph)].q_win.iloc[0]
                txt=f'{qa:.2f} / {qb:.2f} L/min'
            else: txt=f'both at {qq:.2f} L/min'
            setports(a);a.tick_params(labelbottom=True,axis='x',labelsize=6.5);a.set_title(f'Preset {pre}\n{txt}',fontsize=7)
            if k==0:a.set_ylabel(('Dual-orifice' if route=='do' else 'Nose-only')+'\n'+yl)
    h=[Line2D([],[],color=ST[c]['c'],ls=ST[c]['ls'],marker=ST[c]['m'],mfc=ST[c]['mf'],ms=3.6,label=LAB[c]) for c in CF]
    f.legend(handles=h,ncol=4,loc='outside lower center')
    save(f,name)
cmp_prof('insp','p','device_profile_insp_measured','Pressure (Pa)');cmp_prof('insp','p_cf','device_profile_insp_commonflow','Pressure (Pa)');cmp_prof('exp','p','device_profile_exp_measured','Pressure (Pa)');cmp_prof('exp','p_cf','device_profile_exp_commonflow','Pressure (Pa)')
def delta_fig(ph,kind):
    # kind device: mad-native at route ; route: no-do at model
    f,ax=plt.subplots(2,2,figsize=(W,4.2),sharex=True,sharey='row',constrained_layout=True)
    rows=(('do','no') if kind=='device' else ('native','mad'))
    for r,gr in enumerate(rows):
        for c,key in enumerate(('p','p_cf')):
            a=ax[r,c];zero(a)
            ramp=(RAMP if kind=='device' or gr=='native' else RAMP_O)
            for pre in (4,3,2,1):
                if kind=='device':A,B=f'mad_{gr}',f'native_{gr}'
                else:A,B=f'{gr}_no',f'{gr}_do'
                da=pk[(pk.cfg==A)&(pk.preset==pre)&(pk.phase==ph)].set_index('port').loc[P];db=pk[(pk.cfg==B)&(pk.preset==pre)&(pk.phase==ph)].set_index('port').loc[P]
                y=da[key]-db[key];u=np.hypot(da.u,db.u)
                a.errorbar(range(7),y,yerr=u,color=ramp[pre],marker='o',ms=3.4,lw=1.1,capsize=1.5,elinewidth=.7,label=f'Preset {pre}')
            setports(a);a.tick_params(labelbottom=True)
            t={'device':{'do':'Dual-orifice','no':'Nose-only'},'route':{'native':'Native','mad':'MAD'}}[kind][gr]
            a.set_title(t+(', as measured' if key=='p' else ', common flow'))
            if c==0:a.set_ylabel(('MAD minus native' if kind=='device' else 'Nose-only minus dual-orifice')+' (Pa)')
            if r==0 and c==1:a.legend(loc='best')
            if r==1:a.set_xlabel('Port (trachea to nose and mouth)')
    save(f,f'{kind}_delta_{ph}')
for k in ('device','route'):
    for ph in ('insp','exp'):delta_fig(ph,k)
# route profile (per model) as measured and common flow
def route_prof(ph,key,name):
    f,ax=plt.subplots(2,4,figsize=(W,3.9),sharex=True,constrained_layout=True)
    for r,mod in enumerate(('native','mad')):
        for k,pre in enumerate((1,2,3,4)):
            a=ax[r,k];zero(a)
            for cfg in (f'{mod}_do',f'{mod}_no'):
                d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase==ph)];s=ST[cfg];prof(a,d,s['c'],s['ls'],s['m'],s['mf'],yk=key)
            if key=='p':
                qa=pk[(pk.cfg==f'{mod}_do')&(pk.preset==pre)&(pk.phase==ph)].q_win.iloc[0];qb=pk[(pk.cfg==f'{mod}_no')&(pk.preset==pre)&(pk.phase==ph)].q_win.iloc[0];txt=f'{qa:.2f} / {qb:.2f} L/min'
            else:txt=f'both at {d.q_ref.iloc[0]:.2f} L/min'
            setports(a);a.tick_params(labelbottom=True,axis='x',labelsize=6.5);a.set_title(f'Preset {pre}\n{txt}',fontsize=7)
            if k==0:a.set_ylabel(('Native' if mod=='native' else 'MAD')+'\nPressure (Pa)')
    h=[Line2D([],[],color=ST[c]['c'],ls=ST[c]['ls'],marker=ST[c]['m'],mfc=ST[c]['mf'],ms=3.6,label=LAB[c]) for c in CF]
    f.legend(handles=h,ncol=4,loc='outside lower center')
    save(f,name)
route_prof('insp','p','route_profile_insp_measured');route_prof('insp','p_cf','route_profile_insp_commonflow')
# Cp
f,ax=plt.subplots(1,4,figsize=(W,2.7),sharey=True,constrained_layout=True)
for a,cfg in zip(ax,('native_do','native_no','mad_do','mad_no')):
    zero(a);ramp=RAMP if 'native' in cfg else RAMP_O
    for pre in (4,3,2,1):
        d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase=='insp')].set_index('port').loc[P];a.plot(range(7),d.cp,color=ramp[pre],marker='o',ms=3,lw=1,label=f'Preset {pre}')
    setports(a);a.tick_params(axis='x',labelsize=6.5);a.set_title(LAB[cfg],fontsize=7.5)
ax[0].set_ylabel('p / (½ρU²)');ax[3].legend(loc='lower right',fontsize=6.5);f.supxlabel('Port (trachea to nose and mouth)',fontsize=8)
save(f,'cp_profile_insp')
# drop & resistance
dr=[]
for cfg in CF:
    for ph in('insp','exp'):
        for pre in (1,2,3,4):
            d=pk[(pk.cfg==cfg)&(pk.preset==pre)&(pk.phase==ph)].set_index('port')
            dp=abs(d.loc['p7','p']-d.loc['p1','p']);q=d.q_win.iloc[0];ug=np.hypot(d.loc['p7','u'],d.loc['p1','u'])
            dr.append(dict(cfg=cfg,kind='breathing',phase=ph,level=pre,q=q,dp=dp,u=ug,R=dp/q,uR=np.hypot(ug/dp,.015)*dp/q))
    for dd,ph in (('in','insp'),('out','exp')):
        for m in (1,2,3,4):
            d=sr[(sr.cfg==cfg)&(sr['mode']==m)&(sr.dir==dd)].set_index('port');dp=abs(d.loc['p7','p']-d.loc['p1','p']);q=abs(d.q.iloc[0]);ug=np.hypot(d.loc['p7','u'],d.loc['p1','u'])
            dr.append(dict(cfg=cfg,kind='steady',phase=ph,level=m,q=q,dp=dp,u=ug,R=dp/q,uR=np.hypot(ug/dp,.015)*dp/q))
dr=pd.DataFrame(dr);dr.to_pickle('drop.pkl')
f,ax=plt.subplots(2,2,figsize=(W,4.3),sharex='col',constrained_layout=True)
for j,(ph) in enumerate(('insp','exp')):
    for cfg in CF:
        s=ST[cfg];d=dr[(dr.cfg==cfg)&(dr.kind=='breathing')&(dr.phase==ph)];e=dr[(dr.cfg==cfg)&(dr.kind=='steady')&(dr.phase==ph)]
        ax[j,0].errorbar(d.q,d.dp,yerr=d.u,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.8,capsize=1.5,elinewidth=.7,label=LAB[cfg])
        ax[j,1].errorbar(d.q,d.R,yerr=d.uR,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.8,capsize=1.5,elinewidth=.7)
    ax[j,0].set_ylabel(f'|p7 − p1| (Pa)');ax[j,1].set_ylabel('Resistance (Pa per L/min)');ax[j,0].set_title(PH[ph]+', pressure drop');ax[j,1].set_title(PH[ph]+', resistance');ax[j,0].set_ylim(bottom=0);ax[j,1].set_ylim(bottom=0)
ax[1,0].set_xlabel('Window flow (L/min)');ax[1,1].set_xlabel('Window flow (L/min)');ax[0,0].legend(loc='upper left')
save(f,'drop_resistance')
# quasi steady
f,ax=plt.subplots(2,2,figsize=(W,4.6),constrained_layout=True)
for j,ph in enumerate(('insp','exp')):
    a=ax[0,j];v=pk[pk.phase==ph];lim=[min(v.p.min(),v.p_steady.min())*1.05,max(v.p.max(),v.p_steady.max(),0)*1.05+(.5)]
    a.plot(lim,lim,color=MUTE,lw=.7,ls=(0,(4,2)))
    for cfg in CF:
        s=ST[cfg];d=v[v.cfg==cfg];a.plot(d.p_steady,d.p,s['m'],mec=s['c'],mfc=s['mf'],ms=3.2,mew=.7,label=LAB[cfg],ls='')
    a.set_xlabel('Steady fit at window flow (Pa)');a.set_ylabel('Unsteady at peak window (Pa)');a.set_title(PH[ph])
    if j==0:a.legend(loc='upper left')
    a=ax[1,j];zero(a)
    off={c:(i-1.5)*.13 for i,c in enumerate(CF)}
    for cfg in CF:
        s=ST[cfg];d=v[v.cfg==cfg];xs=[P.index(q)+off[cfg] for q in d.port];a.plot(xs,d['diff'],s['m'],mec=s['c'],mfc=s['mf'],ms=2.8,mew=.6,ls='')
    setports(a);a.set_ylabel('Unsteady − steady fit (Pa)');a.set_xlabel('Port')
save(f,'quasi_steady')
# repeat
f,ax=plt.subplots(1,2,figsize=(W,2.9),sharex=True,sharey=True,constrained_layout=True)
for a,k,t in zip(ax,('range','sd'),('Between-suite range of suite means','Breath-to-breath SD, 27 breaths')):
    for cfg in CF:
        s=ST[cfg];d=pk[pk.cfg==cfg];a.plot(d.p.abs(),d[k],s['m'],mec=s['c'],mfc=s['mf'],ms=2.8,mew=.6,ls='',label=LAB[cfg])
    x=np.logspace(-1.3,2,80);a.plot(x,np.hypot(.1,.03*x),color=INK,lw=.9,ls=(0,(4,2)),label='SDP811 uncertainty')
    a.set_xscale('log');a.set_yscale('log');a.set_title(t);a.set_xlabel('|Pressure| at peak window (Pa)');a.set_xlim(.05,100)
ax[0].set_ylabel('Pa');ax[0].legend(loc='upper left')
save(f,'repeatability')
# flow error
f,ax=plt.subplots(1,4,figsize=(W,2.7),sharey=True,constrained_layout=True)
g=b.groupby(['cfg','preset','phase'])[['fit','target']].mean().reset_index();g['err']=(g.fit/g.target-1)*100
s2=st[~st['first']].groupby(['cfg','mode','dir']).agg(q=('qmean',lambda x:x.abs().mean()),t=('target','mean')).reset_index();s2['err']=(s2.q/s2.t-1)*100
for a,(frame,fil,t,x) in zip(ax,((g,'exp','Unsteady, expiration','preset'),(g,'insp','Unsteady, inspiration','preset'),(s2,'out','Steady, out','mode'),(s2,'in','Steady, in','mode'))):
    a.axhspan(-1.5,1.5,color='#dad9d4',alpha=.5,lw=0);zero(a)
    d0=frame[frame['phase' if x=='preset' else 'dir']==fil]
    for cfg in CF:
        s=ST[cfg];d=d0[d0.cfg==cfg];a.plot(d[x],d.err,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.6,lw=1,label=LAB[cfg])
    a.set_title(t,fontsize=7.5);a.set_xticks((1,2,3,4));a.set_xlabel('Preset' if x=='preset' else 'Steady mode')
ax[0].set_ylabel('Delivered peak or mean flow error (%)');h,l=ax[0].get_legend_handles_labels();f.legend(h,l,ncol=4,loc='outside lower center')
save(f,'flow_error')
# rig behaviour
f,ax=plt.subplots(3,2,figsize=(W,5),sharex=True,sharey='row',constrained_layout=True)
gg=b.groupby(['cfg','preset','phase'])[['lag','stall','ripple']].mean().reset_index()
for r,(k,yl) in enumerate((('lag','Lag behind command (s)'),('stall','Reversal stall (s)'),('ripple','Ripple SD (L/min)'))):
    for c,ph in enumerate(('exp','insp')):
        a=ax[r,c];d0=gg[gg.phase==ph]
        for cfg in CF:
            s=ST[cfg];d=d0[d0.cfg==cfg];a.plot(d.preset,d[k],color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.6,lw=1,label=LAB[cfg])
        a.set_ylim(0,gg[k].max()*1.12)
        if r==0:a.set_title(PH[ph])
        if c==0:a.set_ylabel(yl)
        if r==2:a.set_xlabel('Preset');a.set_xticks((1,2,3,4))
ax[1,1].legend(loc='upper right')
save(f,'rig_behaviour')
# leak
v2=vol[(vol.breath>1)&(vol.push_leak_ml.abs()<500)&(vol.return_leak_ml.abs()<500)]
print('excluded pot glitches',len(vol[vol.breath>1])-len(v2))
lm=v2.groupby(['cfg','preset'])[['push_leak_ml','return_leak_ml']].median().reset_index();lm.to_pickle('leak.pkl')
f,ax=plt.subplots(1,3,figsize=(W,2.6),constrained_layout=True)
for cfg in CF:
    s=ST[cfg];d=lm[lm.cfg==cfg];ax[0].plot(d.preset,d.push_leak_ml,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.6,lw=1,label=LAB[cfg]);ax[1].plot(d.preset,d.return_leak_ml,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.6,lw=1)
    e=st[(~st['first'])&(st.cfg==cfg)].groupby('mode').leak_pct.mean();ax[2].plot(e.index,e.values,color=s['c'],ls=s['ls'],marker=s['m'],mfc=s['mf'],ms=3.6,lw=1)
ax[0].set_title('Push volume gap');ax[1].set_title('Return volume gap');ax[2].set_title('Steady seal leak')
ax[0].set_ylabel('Median, mL per stroke');ax[1].set_ylabel('Median, mL per stroke');ax[2].set_ylabel('% of piston rate')
ax[0].set_xlabel('Preset');ax[1].set_xlabel('Preset');ax[2].set_xlabel('Steady mode')
for a in ax:a.set_xticks((1,2,3,4));a.set_ylim(bottom=0)
h,l=ax[0].get_legend_handles_labels();f.legend(h,l,ncol=4,loc='outside lower center')
save(f,'leak')
