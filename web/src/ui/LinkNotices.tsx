import { useEffect, useState } from 'react';
import { api, enterDemoMode } from '../api/index.ts';
import { app } from '../state/app.ts';
import { useStore } from '../state/store.ts';

/** How long a first connection may take before demo mode is offered. */
const OFFER_DEMO_AFTER_MS = 3500;

function useCountdown(until: number | null): number | null {
  const [, tick] = useState(0);
  useEffect(() => {
    if (until === null) return;
    const id = window.setInterval(() => tick((n) => n + 1), 250);
    return () => window.clearInterval(id);
  }, [until]);
  return until === null ? null : Math.max(0, Math.ceil((until - Date.now()) / 1000));
}

/**
 * Connection trouble, said plainly. After a connection has been lost: a red
 * banner while it reconnects. If there never was one: an offer to run the demo.
 */
export function LinkNotices() {
  const link = useStore(app, (s) => s.link);
  const everOpened = useStore(app, (s) => s.everOpened);
  const transport = useStore(app, (s) => s.transport);
  const retryAt = useStore(app, (s) => s.retryAt);
  const seconds = useCountdown(link === 'closed' ? retryAt : null);
  const [offerDemo, setOfferDemo] = useState(false);

  useEffect(() => {
    if (transport !== 'live' || everOpened) {
      setOfferDemo(false);
      return;
    }
    const id = window.setTimeout(() => setOfferDemo(true), OFFER_DEMO_AFTER_MS);
    return () => window.clearTimeout(id);
  }, [transport, everOpened]);

  if (link === 'open') return null;

  if (everOpened) {
    return (
      <div className="banner" role="alert">
        <span>
          Lost the robot daemon. What you see is the last known pose.{' '}
          {seconds !== null && seconds > 0 ? `Reconnecting in ${seconds} s` : 'Reconnecting'}
        </span>
        <button type="button" onClick={() => api.retryNow()}>
          Retry now
        </button>
      </div>
    );
  }

  if (!offerDemo) return null;

  return (
    <div className="card" role="dialog" aria-labelledby="no-daemon-title">
      <h1 id="no-daemon-title">Can't reach the robot daemon</h1>
      <p>
        Nothing is answering at <code>{location.host}/ws</code>. Start the daemon and this page connects by itself. Or
        run the demo: a simulated arm with a scripted conversation, no hardware needed.
      </p>
      <div className="card__actions">
        <button type="button" className="btn btn--robot" onClick={enterDemoMode}>
          Run demo mode
        </button>
        <span className="card__wait">
          <span className="spinner" />
          still trying the daemon
        </span>
      </div>
    </div>
  );
}
