import { sourceLabel } from '../actions.ts';
import { app } from '../state/app.ts';
import { useStore } from '../state/store.ts';
import { CheckIcon } from './icons.tsx';

/** Mode, links, calibration, torque and what the arm is doing, in one strip. */
export function StatusStrip() {
  const status = useStore(app, (s) => s.status);
  const link = useStore(app, (s) => s.link);

  const online = link === 'open' && status !== null;

  return (
    <div className="status" role="status" aria-label="Robot status">
      <div className="status__seg status__mode" data-mode={status ? status.mode : 'none'}>
        {status ? (status.mode === 'real' ? 'REAL' : 'SIM') : '···'}
      </div>

      {!online ? (
        <div className="status__seg" data-tone={link === 'closed' ? 'bad' : undefined}>
          <span className="dot" data-pulse="true" />
          {link === 'closed' ? 'daemon offline' : 'connecting'}
        </div>
      ) : (
        <>
          <div className="status__seg" data-tone={status.connected ? undefined : 'bad'}>
            <span className="dot" data-on={status.connected} />
            {status.connected ? 'connected' : 'robot link down'}
          </div>
          <div className="status__seg" data-optional="true" data-tone={status.calibrated ? undefined : 'warn'}>
            {status.calibrated ? (
              <>
                <CheckIcon style={{ width: '0.95rem', height: '0.95rem' }} />
                calibrated
              </>
            ) : (
              'not calibrated'
            )}
          </div>
          <div className="status__seg" data-optional="true" data-tone={status.torque ? undefined : 'warn'}>
            {status.torque ? 'torque on' : 'torque off, arm is limp'}
          </div>
          <div className="status__seg status__activity" data-busy={status.busy}>
            {status.busy ? (
              <>
                <span className="dot" data-pulse="true" />
                {status.activity || 'busy'}
                {status.source && <span className="source">{sourceLabel(status.source)}</span>}
              </>
            ) : (
              'idle'
            )}
          </div>
        </>
      )}
    </div>
  );
}
