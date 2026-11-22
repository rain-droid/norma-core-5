import { sourceLabel } from '../actions.ts';
import { app, type ToolItem } from '../state/app.ts';
import { shallowEqual, useStore } from '../state/store.ts';
import { formatArgs } from './Timeline.tsx';
import { CheckIcon, CrossIcon } from './icons.tsx';

/**
 * What is going on right now and what comes next: the tool that is running and
 * the motion it is in, above the list of steps the agent announced for its task.
 * The history of what happened is the timeline below; this is the look ahead.
 */
export function StepsPanel() {
  const steps = useStore(app, (s) => s.steps);
  const online = useStore(app, (s) => s.link === 'open');
  const now = useStore(
    app,
    (s) => {
      let running: ToolItem | null = null;
      for (let i = s.timeline.length - 1; i >= 0 && i >= s.timeline.length - 40; i--) {
        const it = s.timeline[i];
        if (it.kind === 'tool' && it.status === 'running') {
          running = it;
          break;
        }
      }
      const activity = s.status?.activity ?? '';
      return {
        tool: running?.tool ?? '',
        args: running ? formatArgs(running.args) : '',
        // "idle" between two motions of one tool is not worth a word
        motion: s.status?.busy && activity && activity !== 'idle' ? activity : '',
        source: running?.source ?? s.status?.source ?? null,
      };
    },
    shallowEqual,
  );

  const total = steps?.steps.length ?? 0;
  const done = steps ? Math.min(steps.current, total) : 0;
  const finished = total > 0 && done === total;
  const working = Boolean(now.tool || now.motion);

  return (
    <section className="steps" aria-label="What the robot is doing and what comes next">
      <div className="steps__now" data-working={working}>
        <span className="steps__tag">now</span>
        {working ? (
          <>
            <span className="spinner" role="img" aria-label="working" />
            <span className="steps__what" title={`${now.tool} ${now.args}`.trim()}>
              {now.tool || now.motion}
              {now.tool && now.motion && <i>{now.motion}</i>}
            </span>
            {now.source && (
              <span className="source" data-source={now.source}>
                {sourceLabel(now.source)}
              </span>
            )}
          </>
        ) : (
          <span className="steps__what steps__what--quiet">
            {!online ? 'no link to the daemon' : finished ? 'done' : total ? 'between steps' : 'nothing running'}
          </span>
        )}
      </div>

      {steps && total > 0 ? (
        <>
          <div className="steps__head">
            <span className="steps__title" title={steps.title}>
              {steps.title || 'Plan'}
            </span>
            <span className="steps__count">
              {done} of {total}
            </span>
          </div>
          <div className="steps__bar" role="progressbar" aria-valuemin={0} aria-valuemax={total} aria-valuenow={done}>
            <span style={{ width: `${(done / total) * 100}%` }} />
          </div>
          <ol className="steps__list">
            {steps.steps.map((text, i) => {
              const state = i < done ? 'done' : i > done ? 'next' : steps.failed ? 'failed' : 'active';
              return (
                <li key={`${i}:${text}`} data-state={state} aria-current={state === 'active' ? 'step' : undefined}>
                  <span className="steps__mark">
                    {state === 'done' ? <CheckIcon aria-label="done" /> : state === 'failed' ? <CrossIcon aria-label="failed" /> : i + 1}
                  </span>
                  <span className="steps__text">{text}</span>
                </li>
              );
            })}
          </ol>
        </>
      ) : (
        <div className="steps__empty">No plan announced. The agent lists its steps here when a task has more than one.</div>
      )}
    </section>
  );
}
