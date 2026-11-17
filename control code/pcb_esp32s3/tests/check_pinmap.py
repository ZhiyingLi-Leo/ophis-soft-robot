"""Compare firmware constants with the manufactured-project pin netlist."""
import csv,re,sys,json
from pathlib import Path
root=Path(__file__).parents[1]
net=Path(sys.argv[1])
rows=list(csv.DictReader(net.open(encoding='utf-8-sig')))
native=[]
for line in (net.parent/'design_data/PCB单板原生文档.txt').read_text().splitlines():
    h,b=line.rstrip('|').split('||',1);native.append((json.loads(h),json.loads(b)))
u1=next(b['parentId'] for h,b in native if h['type']=='ATTR' and b.get('key')=='Designator' and b.get('value')=='U1')
native_nets={json.loads(h['id'])[2]:b['padNet'] for h,b in native if h['type']=='PAD_NET' and json.loads(h['id'])[1]==u1}
for r in rows:
    if r['Designator']=='U1' and r['Net']:assert native_nets[r['Pin']]==r['Net'],r
pins={r['Net']:int(r['Pin Name'][2:]) for r in rows if r['Designator']=='U1' and r['Pin Name'].startswith('IO') and r['Net']}
header=(root/'firmware/include/board_config.h').read_text()
for symbol,net_suffix in [('STEP','STEP'),('DIR','DIR'),('ENABLE','ENABLE'),('FAULT_N','FAULT_N'),('CURRENT_PWM','ISET_PWM')]:
 actual=list(map(int,re.search(r'\b'+symbol+r'\[4\] = \{([^}]+)',header)[1].split(',')))
 assert actual==[pins[f'M{i}_{net_suffix}'] for i in range(1,5)],symbol
for symbol,netname in [('WAKE','DRIVER_WAKE_CMD'),('EXT_SELECTED','EXT_POWER_SELECTED'),('USB_PRESENT_N','USB_PRESENT_N')]:
 assert int(re.search(r'\b'+symbol+r' = (\d+)',header)[1])==pins[netname],symbol
print('PASS: 23 MCU control/status GPIOs match PCB netlist; USB GPIO19/20 left reserved')
