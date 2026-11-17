import asyncio,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]/'host'))
from client import Client,DryLink,BLELink,USBLink
from control import axis_speed,parse_args

class HostTests(unittest.IsolatedAsyncioTestCase):
 async def test_client(self):
  c=Client(DryLink());await c.open();await c.arm();await c.velocity(400,2)
  self.assertEqual((await c.status())['speed'],[0,400,0,0])
  await asyncio.sleep(.25);self.assertIsNone(c.failure)
  with self.assertRaises(ValueError):await c.velocity(16001)
  with self.assertRaises(ValueError):await c.command('V400\nA1')
  await c.close();self.assertEqual(c.link.armed,0)
 async def test_wrong_board(self):
  class Wrong(DryLink):
   async def write(self,line):self.q.put_nowait('ERR COMMAND')
  with self.assertRaisesRegex(RuntimeError,'ERR COMMAND'):await Client(Wrong()).open()
 async def test_ble_fragmentation(self):
  link=BLELink();link._notify(None,b'STATUS {"a":');link._notify(None,b'1}\nOK H\n')
  self.assertEqual(await link.readline(),'STATUS {"a":1}')
  self.assertEqual(await link.readline(),'OK H')
 async def test_heartbeat_failure(self):
  class Bad(DryLink):
   async def write(self,line):
    if line=='H':self.q.put_nowait('ERR NOT_OWNER')
    else:await super().write(line)
  c=Client(Bad());await c.open();await c.arm();await asyncio.sleep(.3)
  self.assertIsNotNone(c.failure)
  with self.assertRaises(RuntimeError):await c.velocity(400)
  await c.close()
 def test_axis(self):
  self.assertEqual(axis_speed(.05,3200,.12),0);self.assertEqual(axis_speed(-1,3200,.12),-3200)
  with self.assertRaises(ValueError):axis_speed(float('nan'),3200,.12)
 def test_arguments(self):
  for argv in [['--dry-run','jog','--duration','nan'],['--dry-run','jog','--speed','16001'],['--dry-run','gamepad','--axis-index','-1']]:
   with self.assertRaises(SystemExit):parse_args(argv)

if __name__=='__main__':unittest.main()
