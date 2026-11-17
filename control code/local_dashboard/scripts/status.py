"""Read device state without issuing movement commands."""
from experiment_api import Robot
state=Robot().state()
for axis in state['axes']:
    position=f"{axis['position']:.3f} mm" if axis['online'] else '未接入'
    print(f"电缸 {axis['id']}: {position}"+(f" · {axis['error']}" if axis['error'] else ''))
p=state['propulsion']
print('propulsion / 推进器: '+(p['backend'] if p['online'] else '未接入'))
for motor in p['motors'] if p['online'] else []:
    print(f"M{motor['id']}: {motor['speed']} step/s · 电流设定 {motor['current_ma']} mA")
print('状态读取完成；未发送运动指令。')
