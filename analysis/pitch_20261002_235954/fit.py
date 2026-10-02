"""Reproduce empirical pitch voltage feedforward from settled hold plateaus.
Requires numpy/matplotlib; outputs are evidence, not a force calibration.
"""
import csv,json,math,os,hashlib
from pathlib import Path
import numpy as np
os.environ.setdefault('MPLCONFIGDIR','/tmp/rm-pitch-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
LOG=ROOT/'monitor/logger_20261002_235954_867422.txt'
with LOG.open() as f:
 rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f,delimiter='\t')]
runs=[];run=[]
for r in rows:
 valid=math.isfinite(r['pitch_speed_target_rpm']) and r['gimbal_enabled']==1 and r['gimbal_startup_ready']==1
 if run and (not valid or abs(r['gimbal_pitch_target_deg']-run[-1]['gimbal_pitch_target_deg'])>.02 or r['timestamp_ms']-run[-1]['timestamp_ms']>100):
  runs.append(run);run=[]
 if valid:run.append(r)
if run:runs.append(run)
points=[]
for run in runs:
 if run[-1]['timestamp_ms']-run[0]['timestamp_ms']<2000:continue
 settled=[r for r in run if r['timestamp_ms']-run[0]['timestamp_ms']>=1000 and abs(r['pitch_speed_actual_rpm'])<=1 and abs(r['pitch_command_raw'])<7900]
 if len(settled)<30:continue
 q=np.array([r['gimbal_pitch_actual_deg']*8192/360 for r in settled]);u=np.array([r['pitch_command_raw'] for r in settled])
 if np.ptp(q)>10:continue
 points.append(dict(start_ms=run[0]['timestamp_ms'],end_ms=run[-1]['timestamp_ms'],target_ticks=run[0]['gimbal_pitch_target_deg']*8192/360,actual_ticks=float(np.median(q)),command=float(np.median(u)),command_min=float(u.min()),command_max=float(u.max()),n=len(settled)))
x=np.array([(p['actual_ticks']-1971)/1000 for p in points]);y=np.array([p['command'] for p in points])
results={}
for degree in [1,2]:
 c=np.polynomial.polynomial.polyfit(x,y,degree)
 pred=np.polynomial.polynomial.polyval(x,c)
 loo=[]
 for i in range(len(x)):
  keep=np.arange(len(x))!=i;ci=np.polynomial.polynomial.polyfit(x[keep],y[keep],degree)
  loo.append(np.polynomial.polynomial.polyval(x[i],ci)-y[i])
 results[str(degree)]={'coefficients':c.tolist(),'rmse':float(np.sqrt(np.mean((pred-y)**2))),'loo_rmse':float(np.sqrt(np.mean(np.array(loo)**2))),'max_residual':float(np.max(abs(pred-y)))}
# Use the lower-degree baseline unless quadratic improves cross-validation.
degree=min(results,key=lambda k:results[k]['loo_rmse'])
report={'source':str(LOG.relative_to(ROOT)),'sha256':hashlib.sha256(LOG.read_bytes()).hexdigest(),'normalization':'x=(actual_ticks-1971)/1000','points':points,'models':results,'selected_degree':int(degree),'valid_ticks':[min(p['actual_ticks'] for p in points),max(p['actual_ticks'] for p in points)],'limitations':'Equal weight per hold; friction/hysteresis and changing I contaminate equilibrium. No endpoint extrapolation; no force/current conversion.'}
(OUT/'fit.json').write_text(json.dumps(report,indent=2)+'\n')
with (OUT/'holds.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(points[0]));w.writeheader();w.writerows(points)
q=np.linspace(report['valid_ticks'][0],report['valid_ticks'][1],300)
plt.figure(figsize=(8,4.5))
for d,res in results.items():plt.plot(q,np.polynomial.polynomial.polyval((q-1971)/1000,res['coefficients']),label=f'degree {d}, LOO RMSE {res["loo_rmse"]:.0f}')
plt.scatter(x*1000+1971,y,color='black',label='hold medians')
plt.xlabel('Actual encoder ticks');plt.ylabel('Voltage command counts');plt.grid(alpha=.3);plt.legend();plt.tight_layout();plt.savefig(OUT/'fit.png',dpi=160)
print(json.dumps({k:report[k] for k in ['models','selected_degree','valid_ticks']},indent=2))
