import { useState } from 'react';
import { myControl, releaseControl, requestControl, setControlName, takeControl } from '../control.ts';
import { app } from '../state/app.ts';
import { shallowEqual, useStore } from '../state/store.ts';

/** Who has the arm, who waits for it, and this page's buttons to join, leave or take over. */
export function ControlPanel() {
  const control = useStore(app, (s) => s.control);
  const online = useStore(app, (s) => s.link === 'open');
  const me = useStore(myControl, (s) => s, shallowEqual);
  const [editing, setEditing] = useState(false);

  const holder = control?.holder ?? null;
  const queue = control?.queue ?? [];
  const off = !online || me.pending;
  const someoneElse = holder !== null && me.state !== 'granted';

  let line: string;
  if (me.state === 'granted') line = 'You have control. Nobody else can move the arm.';
  else if (me.state === 'waiting') line = `You are number ${me.position ?? '?'} in line.`;
  else if (holder) line = `${holder.name}${holder.kind === 'agent' ? ' (agent)' : ''} has control.`;
  else line = 'Free: anyone can send single commands.';

  return (
    <section className="panel control" aria-label="Control of the arm" data-mine={me.state === 'granted'}>
      <div className="control__row">
        <span className="control__line" role="status">
          {line}
        </span>
        <span className="control__btns">
          {me.state === 'none' && (
            <button type="button" className="btn" disabled={off} onClick={() => void requestControl()}>
              {holder ? 'Get in line' : 'Take control'}
            </button>
          )}
          {me.state !== 'none' && (
            <button type="button" className="btn" disabled={!online} onClick={() => void releaseControl()}>
              {me.state === 'granted' ? 'Release' : 'Leave line'}
            </button>
          )}
          {someoneElse && (
            <button
              type="button"
              className="btn btn--danger"
              disabled={off}
              title="Take the arm now. Whatever it is doing stops; the current holder gets it back after you."
              onClick={() => void takeControl()}
            >
              Take over
            </button>
          )}
        </span>
      </div>
      {queue.length > 0 && (
        <div className="control__queue">
          waiting: {queue.map((t, i) => `${i + 1}. ${t.name}${t.kind === 'agent' ? ' (agent)' : ''}`).join(', ')}
        </div>
      )}
      <div className="control__me">
        you are{' '}
        {editing ? (
          <input
            className="control__name"
            defaultValue={me.name}
            autoFocus
            maxLength={40}
            onBlur={(e) => {
              setControlName(e.currentTarget.value);
              setEditing(false);
            }}
            onKeyDown={(e) => {
              if (e.key === 'Enter') e.currentTarget.blur();
            }}
          />
        ) : (
          <button type="button" className="control__namebtn" onClick={() => setEditing(true)} title="Change the name others see">
            {me.name}
          </button>
        )}
        {me.error && <span className="control__error"> · {me.error}</span>}
      </div>
    </section>
  );
}
