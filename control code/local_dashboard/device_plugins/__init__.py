"""Device registry. Existing actuators retain their shared-bus safety controller."""
import copy

class DeviceRegistry:
    def __init__(self, devices):
        self.devices = {d.key: d for d in devices}

    def snapshot(self):
        return {key: dict(kind=device.kind,capabilities=list(device.operations),state=device.snapshot()) for key, device in self.devices.items()}

    def command(self, key, operation, arguments, context):
        if key not in self.devices:
            raise ValueError('未知设备: '+str(key))
        if not isinstance(arguments, dict):
            raise ValueError('设备参数必须是对象')
        return self.devices[key].command(operation, arguments, context)

class CylinderDevice:
    kind="cylinder"
    operations=("connect","disconnect","arm","move","jog","stop")
    def __init__(self, engine, index):
        self.engine, self.index = engine, index
        self.key = 'cylinder.'+str(engine.axes[index]['id'])

    def snapshot(self):
        return copy.deepcopy(self.engine.axes[self.index])

    def command(self, operation, args, context):
        actions = dict(connect='connect_axis', disconnect='disconnect_axis', arm='arm',
                       move='move', jog='jog', stop='stop_axis')
        if operation not in actions: raise ValueError('电缸不支持此操作')
        values = {'axis':self.index}
        if operation in ('move','jog'): values['value']=args['mm']
        if operation=='connect': values['port']=args.get('port')
        return self.engine.action(dict(context, action=actions[operation], **values))

class PropulsionDevice:
    kind='propulsion'
    operations=('connect','disconnect','motor','all','stop')
    key = 'propulsion'
    def __init__(self, engine): self.engine=engine
    def snapshot(self):
        return copy.deepcopy(self.engine.prop)
    def command(self, operation, args, context):
        if operation in ('connect','disconnect','stop'):
            values=dict(action={'connect':'connect_prop','disconnect':'disconnect_prop','stop':'stop_prop'}[operation])
            if operation=='connect': values['port']=args.get('port')
        elif operation=='motor':
            motor=args['motor']
            if type(motor) is not int or not 1<=motor<=4: raise ValueError('电机编号必须为 1–4')
            values=dict(action='motor',motor=motor-1,value=args['speed'])
        elif operation=='all': values=dict(action='propulsion',value=args['speed'])
        else: raise ValueError('推进器不支持此操作')
        return self.engine.action(dict(context,**values))
