"""Minimal adapter tutorial: inject a one-joint MuJoCo backend with torque input.

Run: python -m examples.mujoco_adapter
Robot platforms and indoor scenes belong to application projects like MujocoSim.
"""
import asyncio
import math

from rsim.runtime import Runtime
from rsim.drivers import MuJoCo
from rsim.adapters.mujoco import CommandSpec, Observation
from rsim.core import CommandRejected


class JointBackend:
    def open(self):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string('''<mujoco>
          <option timestep="0.002" gravity="0 0 0"/>
          <worldbody><body><joint name="hinge" type="hinge" damping=".1"/>
            <geom type="capsule" fromto="0 0 0 .3 0 0" size=".03" mass="1"/>
          </body></worldbody>
          <actuator><motor joint="hinge"/></actuator>
        </mujoco>''')
        self.data = mujoco.MjData(self.model)

    def step(self, duration, commands):
        self.data.ctrl[0] = commands['torque']
        self.mj.mj_step(self.model, self.data, nstep=round(duration/self.model.opt.timestep))

    def sample(self, demand, *, force=False):
        return Observation(self.data.time, {'angle': float(self.data.qpos[0])}, {'unit': 'rad'})

    def reset(self): self.mj.mj_resetData(self.model, self.data)
    def stop(self):
        if hasattr(self, 'data'): self.data.ctrl[:] = 0
    def close(self): pass  # No renderer or external resources in this small example.


def validate_torque(value):
    if not isinstance(value, (float, int)) or not math.isfinite(value):
        raise CommandRejected('torque must be finite')


def is_zero(value): return value == 0.


async def main():
    source = MuJoCo(JointBackend, outputs={'angle': 8},
                     commands={'torque': CommandSpec(0., is_zero, validate_torque)}, motion_enabled=True)
    async with Runtime(source):
        await source.torque.set(.1, ttl=.15)
        await asyncio.sleep(.25)
        print('angle [rad]:', (await source.angle.get()).data)
        print('expired torque:', source.current_commands['torque'])


if __name__ == '__main__':
    asyncio.run(main())
