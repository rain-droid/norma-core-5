import { memo, useEffect, useLayoutEffect, useRef, useState, type FormEvent } from 'react';
import { sourceLabel } from '../actions.ts';
import { api } from '../api/index.ts';
import { app, type ChatItem, type LogItem, type TimelineItem, type ToolItem } from '../state/app.ts';
import { useStore } from '../state/store.ts';
import { CheckIcon, CrossIcon, DownIcon, KeyboardIcon, MicIcon, SendIcon, WarnIcon } from './icons.tsx';

/** How close to the bottom still counts as "following along". */
const STICK_PX = 64;

function formatValue(v: unknown): string {
  if (typeof v === 'number') return Number.isInteger(v) ? String(v) : v.toFixed(2).replace(/\.?0+$/, '');
  if (typeof v === 'string') return v;
  if (Array.isArray(v)) return `[${v.map(formatValue).join(', ')}]`;
  if (v && typeof v === 'object') {
    return Object.entries(v)
      .map(([k, x]) => `${k}: ${formatValue(x)}`)
      .join(', ');
  }
  return String(v);
}

/** The few arguments worth showing on one line. */
export function formatArgs(args: Record<string, unknown>): string {
  const entries = Object.entries(args).filter(([k]) => !k.startsWith('_'));
  if (entries.length === 1 && typeof entries[0][1] === 'string') return entries[0][1];
  return entries
    .map(([k, v]) => {
      if (v === true) return k;
      if (v === false) return `${k}: no`;
      return `${k} ${formatValue(v)}`;
    })
    .join(', ');
}

function formatDuration(ms: number): string {
  return ms < 1000 ? `${Math.round(ms)} ms` : `${(ms / 1000).toFixed(1)} s`;
}

const Bubble = memo(function Bubble({ item, latest }: { item: ChatItem; latest: boolean }) {
  const robot = item.role === 'assistant';
  return (
    <div className={`bubble ${robot ? 'bubble--robot' : 'bubble--user'}`}>
      {!robot &&
        (item.via === 'voice' ? (
          <MicIcon className="bubble__via" aria-label="said" />
        ) : (
          <KeyboardIcon className="bubble__via" aria-label="typed" />
        ))}
      {item.text.trim()}
      {/* "More is coming" only makes sense on the bubble that is still growing. */}
      {robot && !item.final && latest && <span className="bubble__more" aria-hidden="true" />}
    </div>
  );
});

const ToolRow = memo(function ToolRow({ item }: { item: ToolItem }) {
  const args = formatArgs(item.args);
  const line = item.status === 'failed' ? item.summary || item.error || 'failed' : item.summary;
  return (
    <div className="row" data-status={item.status}>
      <span className="row__icon">
        {item.status === 'running' ? (
          <span className="spinner" role="img" aria-label="running" />
        ) : item.status === 'ok' ? (
          <CheckIcon aria-label="ok" />
        ) : (
          <CrossIcon aria-label="failed" />
        )}
      </span>
      <span className="row__head">
        <span className="row__tool">{item.tool}</span>
        {args && <span className="row__args">{args}</span>}
      </span>
      <span className="row__meta">
        {item.durationMs !== null && <span>{formatDuration(item.durationMs)}</span>}
        {item.source && (
          <span className="source" data-source={item.source}>
            {sourceLabel(item.source)}
          </span>
        )}
      </span>
      {line && (
        <span className="row__summary" title={line}>
          {line}
        </span>
      )}
    </div>
  );
});

const LogRow = memo(function LogRow({ item }: { item: LogItem }) {
  return (
    <div className="row row--log" data-level={item.level}>
      <span className="row__icon">
        <WarnIcon aria-label={item.level === 'error' ? 'error' : 'warning'} />
      </span>
      <span className="row__summary">{item.text}</span>
    </div>
  );
});

function Item({ item, latest }: { item: TimelineItem; latest: boolean }) {
  if (item.kind === 'chat') return item.text.trim() ? <Bubble item={item} latest={latest} /> : null;
  if (item.kind === 'tool') return <ToolRow item={item} />;
  return <LogRow item={item} />;
}

/** The conversation, with tool calls and warnings woven in where they happened. */
export function Timeline() {
  const items = useStore(app, (s) => s.timeline);
  const online = useStore(app, (s) => s.link === 'open');
  const scrollRef = useRef<HTMLDivElement>(null);
  const stick = useRef(true);
  const [behind, setBehind] = useState(false);

  const toBottom = () => {
    const el = scrollRef.current;
    if (el) el.scrollTop = el.scrollHeight;
  };

  useLayoutEffect(() => {
    if (stick.current) toBottom();
    else setBehind(true);
  }, [items]);

  // Keep following when the column itself changes height.
  useEffect(() => {
    const el = scrollRef.current;
    if (!el) return;
    const ro = new ResizeObserver(() => {
      if (stick.current) toBottom();
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const onScroll = () => {
    const el = scrollRef.current;
    if (!el) return;
    stick.current = el.scrollHeight - el.scrollTop - el.clientHeight < STICK_PX;
    if (stick.current) setBehind(false);
  };

  return (
    <div className="timeline">
      <div className="timeline__scroll" ref={scrollRef} onScroll={onScroll} tabIndex={0} aria-label="Conversation and activity" role="log">
        {items.length === 0 && (
          <div className="timeline__empty">
            {online
              ? 'Nothing yet. Hold space and say something, or type below.'
              : 'The conversation shows up here once the daemon is connected.'}
          </div>
        )}
        {items.map((item, i) => (
          <Item key={item.key} item={item} latest={i === items.length - 1} />
        ))}
      </div>
      {behind && (
        <button
          type="button"
          className="timeline__jump"
          onClick={() => {
            stick.current = true;
            setBehind(false);
            toBottom();
          }}
        >
          <DownIcon />
          Jump to latest
        </button>
      )}
    </div>
  );
}

/** Text box that posts to /api/chat. */
export function Composer() {
  const [text, setText] = useState('');
  const [sending, setSending] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const online = useStore(app, (s) => s.link === 'open');

  const submit = async (e: FormEvent) => {
    e.preventDefault();
    const body = text.trim();
    if (!body || sending) return;
    setSending(true);
    setError(null);
    try {
      await api.chat(body);
      setText('');
    } catch (err) {
      setError(`Not sent: ${err instanceof Error ? err.message : 'the daemon did not answer'}`);
    } finally {
      setSending(false);
    }
  };

  return (
    <>
      {error && (
        <div className="row row--log" data-level="error" role="alert">
          <span className="row__icon">
            <WarnIcon />
          </span>
          <span className="row__summary">{error}</span>
        </div>
      )}
      <form className="composer" onSubmit={submit}>
        <input
          type="text"
          value={text}
          onChange={(e) => setText(e.target.value)}
          placeholder={online ? 'Type to the robot' : 'Waiting for the daemon'}
          aria-label="Message to the robot"
          autoComplete="off"
          spellCheck={false}
          enterKeyHint="send"
        />
        <button type="submit" disabled={!text.trim() || sending || !online} aria-label="Send message">
          <SendIcon />
        </button>
      </form>
    </>
  );
}
