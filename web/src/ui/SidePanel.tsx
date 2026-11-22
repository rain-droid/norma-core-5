import { useEffect, useState } from 'react';
import { manualStore, runTool, setTorque } from '../actions.ts';
import type { JointState } from '../api/types.ts';
import { app } from '../state/app.ts';
import { useStore } from '../state/store.ts';

const DEG = 180 / Math.PI;
/** `current` is in raw servo units of about 6.5 mA; the bar is full at 2 A. */
const MA_PER_UNIT = 6.5;
const FULL_SCALE_MA = 2000;
const WARM_C = 55;
const HOT_C = 65;
const JOG_STEPS_DEG = [2, 5, 15];

/** Feetech status byte, as in the SCServo SDK. */
const FAULT_BITS: [number, string][] = [
  [1, 'voltage'],
  [2, 'sensor'],
  [4, 'overheat'],
  [8, 'overcurrent'],
  [32, 'overload'],
];

function faultName(error: number): string {
  const names = FAULT_BITS.filter(([bit]) => error & bit).map(([, n]) => n);
  return names.length ? names.join(' + ') : `fault 0x${error.toString(16)}`;
}

/** The gripper is the last servo on either arm; it reports an open fraction (0..1), not an angle. */
const isGripper = (j: JointState) => j.name === 'rev_motor_08';
const jointLabel = (j: JointState) => (isGripper(j) ? 'grip' : `J${j.id}`);
const jointValue = (j: JointState) => (isGripper(j) ? `${Math.round(j.q * 100)}%` : `${(j.q * DEG).toFixed(0)}°`);
const clamp01 = (v: number) => Math.min(1, Math.max(0, v));

function JointRow({ j }: { j: JointState }) {
  const span = j.upper - j.lower || 1;
  const now = clamp01((j.q - j.lower) / span) * 100;
  const goal = clamp01((j.q_goal - j.lower) / span) * 100;
  const zero = (0 - j.lower) / span;
  const mA = Math.abs(j.current) * MA_PER_UNIT;
  const load = clamp01(mA / FULL_SCALE_MA);
  const fault = j.error !== 0;
  const heat = j.temp >= HOT_C ? 'hot' : j.temp >= WARM_C ? 'warm' : 'ok';

  return (
    <div className="joint" data-fault={fault}>
      <span className="joint__id">{jointLabel(j)}</span>
      <span>
        <span
          className="range"
          title={
            isGripper(j)
              ? `${Math.round(j.q * 100)}% open, ${Math.round(j.q_goal * 100)}% commanded`
              : `${(j.q * DEG).toFixed(1)}° measured, ${(j.q_goal * DEG).toFixed(1)}° commanded, limits ${(j.lower * DEG).toFixed(0)}° to ${(j.upper * DEG).toFixed(0)}°`
          }
        >
          {zero > 0.02 && zero < 0.98 && <i className="range__zero" style={{ left: `${zero * 100}%` }} />}
          <i className="range__goal" style={{ left: `${goal}%` }} />
          <i className="range__now" style={{ left: `${now}%` }} />
        </span>
      </span>
      <span className="joint__num">{jointValue(j)}</span>
      <span title={`${Math.round(mA)} mA, load ${j.load}`}>
        <span className="load" data-high={load > 0.75}>
          <i style={{ transform: `scaleX(${load.toFixed(3)})` }} />
        </span>
      </span>
      {fault ? (
        <span className="joint__num">
          <span className="fault-tag" title={`servo status byte 0x${j.error.toString(16)}`}>
            {faultName(j.error)}
          </span>
        </span>
      ) : (
        <span className="joint__num joint__temp" data-heat={heat}>
          {Math.round(j.temp)}°C
        </span>
      )}
    </div>
  );
}

function Joints() {
  const joints = useStore(app, (s) => s.telemetry?.joints ?? null);
  if (!joints || joints.length === 0) {
    return <p style={{ margin: '0.4rem 0 0.2rem', color: 'var(--ink-soft)' }}>No joint data yet.</p>;
  }
  return (
    <div className="joints">
      <span />
      <span className="joints__cap">within limits</span>
      <span className="joints__cap">angle</span>
      <span className="joints__cap">load</span>
      <span className="joints__cap">temp</span>
      {joints.map((j) => (
        <JointRow key={j.id} j={j} />
      ))}
    </div>
  );
}

function Manual() {
  const gestures = useStore(app, (s) => s.meta?.gestures ?? []);
  const joints = useStore(app, (s) => s.telemetry?.joints ?? []);
  const torque = useStore(app, (s) => s.status?.torque ?? true);
  const real = useStore(app, (s) => s.status?.mode === 'real');
  const online = useStore(app, (s) => s.link === 'open');
  const pending = useStore(manualStore, (s) => s.pending);
  const result = useStore(manualStore, (s) => s.result);
  const [step, setStep] = useState(JOG_STEPS_DEG[1]);
  const [armed, setArmed] = useState(false);
  const off = !online || pending !== null;

  // Releasing torque on the real arm drops it, so it takes a second click.
  useEffect(() => {
    if (!armed) return;
    const id = window.setTimeout(() => setArmed(false), 4000);
    return () => window.clearTimeout(id);
  }, [armed]);

  const jog = (id: number, sign: 1 | -1) =>
    runTool(`jog-${id}`, 'move_joints', { joints: { [String(id)]: (sign * step) / DEG }, relative: true });

  return (
    <div className="manual">
      {gestures.length > 0 && (
        <div className="manual__group">
          <span className="manual__label">Gestures</span>
          <span className="manual__btns">
            {gestures.map((g) => (
              <button key={g} type="button" className="btn btn--robot" disabled={off} onClick={() => runTool(g, 'gesture', { name: g })}>
                {g}
              </button>
            ))}
          </span>
        </div>
      )}
      <div className="manual__group">
        <span className="manual__label">Pose</span>
        <span className="manual__btns">
          <button type="button" className="btn" disabled={off} onClick={() => runTool('home', 'home')}>
            Home
          </button>
          <button type="button" className="btn" disabled={off} onClick={() => runTool('rest', 'rest')}>
            Rest
          </button>
        </span>
      </div>
      <div className="manual__group">
        <span className="manual__label">Gripper</span>
        <span className="manual__btns">
          <button type="button" className="btn" disabled={off} onClick={() => runTool('open', 'gripper', { open: 1 })}>
            Open
          </button>
          <button type="button" className="btn" disabled={off} onClick={() => runTool('close', 'gripper', { open: 0 })}>
            Close
          </button>
        </span>
      </div>
      <div className="manual__group">
        <span className="manual__label">Torque</span>
        <span className="manual__btns">
          {torque ? (
            armed ? (
              <button
                type="button"
                className="btn btn--danger"
                disabled={off}
                onClick={() => {
                  setArmed(false);
                  void setTorque(false);
                }}
              >
                The arm will drop. Release it
              </button>
            ) : (
              <button
                type="button"
                className="btn"
                disabled={off}
                onClick={() => (real ? setArmed(true) : void setTorque(false))}
              >
                Release (arm goes limp)
              </button>
            )
          ) : (
            <button type="button" className="btn btn--robot" disabled={off} onClick={() => void setTorque(true)}>
              Turn torque on
            </button>
          )}
        </span>
      </div>

      <div className="manual__group">
        <span className="manual__label">Jog step</span>
        <span className="seg" role="tablist" aria-label="Jog step size">
          {JOG_STEPS_DEG.map((d) => (
            <button key={d} type="button" role="tab" aria-selected={step === d} onClick={() => setStep(d)}>
              {d}°
            </button>
          ))}
        </span>
      </div>

      <div className="jog">
        {joints.map((j) => (
          <div className="jog__row" key={j.id}>
            <span className="joint__id">{jointLabel(j)}</span>
            <span>{jointValue(j)}</span>
            <span className="jog__pair">
              <button type="button" className="btn" disabled={off || !torque} onClick={() => jog(j.id, -1)} aria-label={`${jointLabel(j)} minus ${step} degrees`}>
                −
              </button>
              <button type="button" className="btn" disabled={off || !torque} onClick={() => jog(j.id, 1)} aria-label={`${jointLabel(j)} plus ${step} degrees`}>
                +
              </button>
            </span>
          </div>
        ))}
      </div>

      <div className="manual__result" data-ok={result ? result.ok : undefined} role="status">
        {pending ? 'working…' : (result?.text ?? 'Every button here calls a robot tool as "UI".')}
      </div>
    </div>
  );
}

/** Joint readouts, with the manual controls one tab away. */
export function SidePanel() {
  const [tab, setTab] = useState<'joints' | 'manual'>('joints');
  const width = useStore(app, (s) => (s.telemetry ? Math.round(s.telemetry.gripper.width_mm) : null));
  const holding = useStore(app, (s) => s.status?.holding ?? false);
  const stale = useStore(app, (s) => s.everOpened && s.link !== 'open');

  return (
    <section className="panel" aria-label="Joints and manual controls" data-stale={stale}>
      <div className="panel__head">
        <span className="seg" role="tablist">
          <button type="button" role="tab" aria-selected={tab === 'joints'} onClick={() => setTab('joints')}>
            Joints
          </button>
          <button type="button" role="tab" aria-selected={tab === 'manual'} onClick={() => setTab('manual')}>
            Manual
          </button>
        </span>
        {width !== null && (
          <span className="panel__note">
            gripper <b>{width} mm</b>
            {holding && <b>, holding</b>}
          </span>
        )}
      </div>
      <div className="panel__body">{tab === 'joints' ? <Joints /> : <Manual />}</div>
    </section>
  );
}
